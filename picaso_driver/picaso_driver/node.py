"""Publish PiCASo acquisitions without blocking the ROS executor on USB reads."""

from array import array
from dataclasses import fields
from datetime import datetime, timezone
from pathlib import Path
import sys
import threading
import time

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from picaso_msgs.msg import Capture
from picaso_driver.protocol import (
    Acquisition, BIT_TO_MICROPHONE, DriverConfig, PicasoSerial,
    RANGE_PER_EVENT, SAMPLE_RATE_HZ, SAMPLES_PER_EVENT, load_waveform,
)

PARAMETER_DESCRIPTIONS = {
    'port': 'Serial device path; empty enables USB VID/PID discovery.',
    'serial_number': 'Optional USB serial identifier used during automatic discovery.',
    'baud_rate': 'Positive serial baud rate (USB CDC normally uses 115200).',
    'mode': 'A: emit the DAC waveform and capture; P: passive capture only.',
    'events': 'Receive events in 1..32, used when max_range is zero.',
    'max_range': 'Nominal capture window in metres; positive overrides events, zero uses events.',
    'shots': 'Successful captures before stopping acquisition; zero runs continuously.',
    'tag': 'Measurement label saved in Capture messages; surrounding whitespace is trimmed.',
    'waveform_file': 'Active-mode DAC CSV; empty selects the bundled sweep, ignored in passive mode.',
    'frame_id': 'Nonempty frame identifier for capture messages.',
    'capture_period': 'Minimum period between commands, including acquisition time; zero is continuous.',
    'serial_timeout': 'Positive serial read polling interval; cannot exceed capture_timeout.',
    'capture_timeout': 'Positive total deadline for receiving a complete capture.',
    'reconnect_period': 'Positive delay before retrying a failed serial or camera connection.',
    'upload_settle_time': 'Nonnegative wait after waveform upload for firmware command processing.',
    'camera_index': 'Webcam device index; -1 disables the camera.',
    'camera_warmup_frames': 'Frames discarded after each camera open for exposure settling; default 10.',
    'camera_frame_id': 'Nonempty image frame identifier when the camera is enabled.',
    'camera_jpeg_quality': 'JPEG quality in 1..100.',
    'camera_max_age': 'Positive maximum age in seconds of the pre-command image paired with a capture.',
}


def timestamp(nanoseconds: int) -> Time:
    """Convert host realtime nanoseconds to a ROS stamp.

    :param nanoseconds: CLOCK_REALTIME count since the Unix epoch.
    :return: Exact seconds and nanoseconds, without float rounding.
    """
    seconds, fraction = divmod(nanoseconds, 1_000_000_000)
    return Time(sec=seconds, nanosec=fraction)


def capture_message(data: Acquisition, config: DriverConfig, capture_id: int, waveform: array) -> Capture:
    """Represent a complete acquisition without losing the recorder clock fields.

    :param data: Packed PDM words and sampled host clocks.
    :param config: Acquisition mode and frame settings.
    :param capture_id: Successful capture counter, retained across reconnects.
    :param waveform: Exact DAC table uploaded for this session, empty for passive mode.
    :return: Self-contained, losslessly reconstructable capture message.
    """
    message = Capture()
    message.header.stamp = timestamp(data.t_cmd_unix_ns)
    message.header.frame_id = config.frame_id
    message.capture_id = capture_id
    message.mode = config.mode
    message.events = config.capture_events
    message.tag = config.tag
    message.byte_count = len(data.words) * 2
    for name in ('t_cmd_unix_ns', 't_done_unix_ns', 't_cmd_mono_ns', 't_done_mono_ns'):
        setattr(message, name, getattr(data, name))
    seconds, fraction = divmod(data.t_cmd_unix_ns, 1_000_000_000)
    message.t_cmd_utc = datetime.fromtimestamp(seconds, timezone.utc).replace(
        microsecond=fraction // 1000).isoformat(timespec='microseconds')
    message.sample_rate_hz = SAMPLE_RATE_HZ
    message.samples_per_event = SAMPLES_PER_EVENT
    message.channel_count = 16
    message.bit_to_microphone = list(BIT_TO_MICROPHONE)
    message.pdm_words = data.words
    message.waveform_sample_rate_hz = SAMPLE_RATE_HZ / 10
    message.transmit_waveform = waveform
    return message


class CameraWorker:
    """Keep a bounded latest webcam image without blocking sonar serial I/O."""

    def __init__(self, config: DriverConfig, stop: threading.Event, logger) -> None:
        """Start optional camera acquisition in an independent daemon worker.

        :param config: Webcam device, encoding and freshness settings.
        :param stop: Shared node shutdown event.
        :param logger: Node logger for optional-camera failures and connection details.
        """
        self.config, self.stop = config, stop
        self.logger = logger
        self.lock = threading.Lock()
        self.image = None
        self.received = 0.0
        self.worker = threading.Thread(target=self.capture_loop, name='picaso-camera', daemon=True)
        self.worker.start()

    def capture_loop(self) -> None:
        """Encode latest frames; webcam failures do not reconnect the sonar."""
        try:
            import cv2
        except ImportError as error:
            self.logger.warning(f'Optional webcam disabled: OpenCV is unavailable ({error})')
            return
        backend = (cv2.CAP_DSHOW if sys.platform == 'win32' else
                   cv2.CAP_AVFOUNDATION if sys.platform == 'darwin' else cv2.CAP_ANY)
        while not self.stop.is_set():
            camera = None
            try:
                camera = cv2.VideoCapture(self.config.camera_index, backend)
                if not camera.isOpened():
                    raise OSError(f'Cannot open camera {self.config.camera_index}')
                for _ in range(self.config.camera_warmup_frames):
                    if self.stop.is_set():
                        break
                    success, _ = camera.read()
                    if not success:
                        raise OSError('Camera stopped delivering frames during warmup')
                if self.stop.is_set():
                    break
                self.logger.info(
                    f'Camera {self.config.camera_index}: '
                    f'{int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
                    f'{int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))}, '
                    f'{camera.get(cv2.CAP_PROP_FPS):.1f} fps; warmup complete')
                while not self.stop.is_set() and camera.isOpened():
                    success, frame = camera.read()
                    received, received_mono = time.time_ns(), time.monotonic()
                    if not success:
                        raise OSError('Camera stopped delivering frames')
                    success, jpeg = cv2.imencode('.jpg', frame,
                                                [cv2.IMWRITE_JPEG_QUALITY, self.config.camera_jpeg_quality])
                    if not success:
                        raise OSError('Camera JPEG encoding failed')
                    message = CompressedImage()
                    message.header.stamp = timestamp(received)
                    message.header.frame_id = self.config.camera_frame_id
                    message.format = 'jpeg'
                    message.data = jpeg.tobytes()
                    with self.lock:
                        self.image, self.received = message, received_mono
            except Exception as error:
                if not self.stop.is_set():
                    self.logger.warning(f'Optional webcam unavailable: {error}', throttle_duration_sec=5.0)
            finally:
                with self.lock:
                    self.image = None
                if camera is not None:
                    camera.release()
            self.stop.wait(self.config.reconnect_period)

    def latest(self) -> CompressedImage | None:
        """Return a recent image without waiting for a camera read.

        :return: Latest JPEG, or None when missing or older than camera_max_age.
        """
        with self.lock:
            return self.image if time.monotonic() - self.received <= self.config.camera_max_age else None


class AirSonarDriver(Node):
    """Own one PiCASo transport and publish complete captures with diagnostics."""

    def __init__(self) -> None:
        """Declare immutable parameters and start the bounded acquisition worker."""
        super().__init__('air_sonar')
        defaults = DriverConfig()
        parameters = {field.name: self.declare_parameter(
            field.name, getattr(defaults, field.name),
            ParameterDescriptor(read_only=True, description=PARAMETER_DESCRIPTIONS[field.name])).value
            for field in fields(defaults)}
        self.config = DriverConfig(**parameters)
        waveform = Path(self.config.waveform_file or 'sweeps/sweep_512_taper30.csv').expanduser()
        if not waveform.is_absolute() and not waveform.is_file():
            waveform = Path(get_package_share_directory('picaso_driver')) / waveform
        self.waveform = load_waveform(waveform) if self.config.mode == 'A' else array('H')
        self.capture_pub = self.create_publisher(Capture, '~/capture', 2)
        self.image_pub = self.create_publisher(CompressedImage, '~/image/compressed', 2)
        self.status_pub = self.create_publisher(DiagnosticArray, '~/status', 2)
        self.shutdown_event = threading.Event()
        self.sensor = PicasoSerial(self.config, self.waveform, self.shutdown_event)
        self.lock = threading.Lock()
        self.status = {'state': 'connecting', 'captures': 0, 'errors': 0, 'message': 'Waiting for USB device',
                       'tag': self.config.tag, 'events': self.config.capture_events,
                       'max_range': self.config.max_range,
                       'capture_range_m': self.config.capture_events * RANGE_PER_EVENT,
                       'shots': self.config.shots,
                       'waveform_file': str(waveform) if self.waveform else ''}
        self.camera = (CameraWorker(self.config, self.shutdown_event, self.get_logger())
                       if self.config.camera_index >= 0 else None)
        self.create_timer(1.0, self.publish_status)
        self.worker = threading.Thread(target=self.acquire_loop, name='picaso-acquisition', daemon=True)
        self.worker.start()
        if self.config.max_range > 0:
            self.get_logger().info(
                f'max_range {self.config.max_range:.2f} m -> {self.config.capture_events} events '
                f'(capture depth {self.config.capture_events * RANGE_PER_EVENT:.2f} m)')
        if self.waveform:
            self.get_logger().info(f'Active waveform: {waveform}; {len(self.waveform)} DAC samples')
        self.get_logger().info(
            f'PiCASo {self.config.mode}, {self.config.capture_events} events '
            f'(~{self.config.capture_events * RANGE_PER_EVENT:.2f} m capture window), '
            f'shots={self.config.shots or "continuous"}, tag={self.config.tag!r}; '
            'publishing packed uint16 PDM on ~/capture')
        if self.config.max_range > 32 * RANGE_PER_EVENT:
            self.get_logger().warning('max_range exceeds the firmware limit; capture window capped at 32 events')

    def grab_image(self) -> CompressedImage | None:
        """Associate a recent optional webcam frame with the next sonar command.

        :return: JPEG with host receive-time stamp, or None when disabled/unavailable.
        """
        if self.camera is None:
            return None
        image = self.camera.latest()
        if image is None:
            self.get_logger().warning('Optional webcam frame unavailable', throttle_duration_sec=10.0)
        return image

    def acquire_loop(self) -> None:
        """Acquire sequentially, discarding partial packets and reconnecting on errors.

        :return: None; exits when shutdown is requested.
        """
        while not self.shutdown_event.is_set() and self.context.ok():
            try:
                self.sensor.open()
                self.get_logger().info(
                    f'Serial port: {self.sensor.connection.port}; '
                    f'read poll {self.config.serial_timeout:.2f} s, '
                    f'capture deadline {self.config.capture_timeout:.2f} s')
                while not self.shutdown_event.is_set() and self.context.ok():
                    began = time.monotonic()
                    image = self.grab_image()
                    data = self.sensor.acquire()
                    if self.shutdown_event.is_set() or not self.context.ok():
                        break
                    with self.lock:
                        capture_id = self.status['captures'] + 1
                    message = capture_message(data, self.config, capture_id, self.waveform)
                    if image is not None:
                        message.has_image = True
                        message.image_stamp = image.header.stamp
                        self.image_pub.publish(image)
                    self.capture_pub.publish(message)
                    duration = (data.t_done_mono_ns - data.t_cmd_mono_ns) / 1e9
                    throughput = message.byte_count / duration / 1e6 if duration > 0 else 0.0
                    self.get_logger().debug(
                        f'Capture #{capture_id}: {message.byte_count} bytes, '
                        f'{duration * 1000:.1f} ms, {throughput:.2f} MB/s'
                        f'{", paired JPEG" if message.has_image else ""}')
                    with self.lock:
                        self.status.update(state='streaming', captures=capture_id, message='Receiving complete captures',
                                           last_capture_duration_sec=duration,
                                           last_capture_bytes=message.byte_count,
                                           last_capture_megabytes_per_sec=throughput,
                                           last_capture_id=capture_id, port=self.sensor.connection.port)
                        if self.config.shots and capture_id >= self.config.shots:
                            self.status.update(state='completed', message='Requested capture count reached')
                    if self.config.shots and capture_id >= self.config.shots:
                        self.get_logger().info(f'Completed {capture_id} captures; acquisition stopped')
                        self.shutdown_event.set()
                        return
                    self.shutdown_event.wait(max(0.0, self.config.capture_period - (time.monotonic() - began)))
            except Exception as error:
                if self.shutdown_event.is_set() or not self.context.ok():
                    break
                with self.lock:
                    self.status.update(state='reconnecting', errors=self.status['errors'] + 1, message=str(error))
                self.get_logger().warning(f'PiCASo capture unavailable: {error}', throttle_duration_sec=5.0)
            finally:
                self.sensor.close()
            self.shutdown_event.wait(self.config.reconnect_period)

    def publish_status(self) -> None:
        """Expose transport failures and successful capture counters to ROS tools."""
        with self.lock:
            values = self.status.copy()
        diagnostic = DiagnosticStatus()
        diagnostic.name = f'{self.get_name()}/serial'
        diagnostic.hardware_id = values.get('port', self.config.port or '0483:5740')
        diagnostic.level = DiagnosticStatus.OK if values['state'] in ('streaming', 'completed') else DiagnosticStatus.WARN
        diagnostic.message = str(values['message'])
        diagnostic.values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
        report = DiagnosticArray()
        report.header.stamp = timestamp(time.time_ns())
        report.status = [diagnostic]
        self.status_pub.publish(report)

    def stop(self) -> None:
        """Interrupt USB I/O and join the worker before destroying ROS publishers."""
        self.shutdown_event.set()
        self.sensor.interrupt()
        self.worker.join(timeout=self.config.serial_timeout + 2.0)
        if self.worker.is_alive():
            self.get_logger().error('Acquisition worker did not stop before shutdown deadline')
        if self.camera is not None:
            # V4L2 reads can block on a faulty camera. Its daemon owns no ROS
            # publisher and must never delay sonar shutdown indefinitely.
            self.camera.worker.join(timeout=.2)


def main(args: list[str] | None = None) -> None:
    """Run only the sonar node and release its device on shutdown.

    :param args: Optional ROS command-line arguments.
    :return: None.
    """
    rclpy.init(args=args)
    node = None
    try:
        node = AirSonarDriver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
