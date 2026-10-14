"""Publish PiCASo acquisitions without blocking the ROS executor on USB reads."""

from array import array
from dataclasses import fields
from datetime import datetime, timezone
from pathlib import Path
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
    SAMPLE_RATE_HZ, SAMPLES_PER_EVENT, load_waveform,
)


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
    message.events = config.events
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

    def __init__(self, config: DriverConfig, stop: threading.Event) -> None:
        """Start optional camera acquisition in an independent daemon worker.

        :param config: Webcam device, encoding and freshness settings.
        :param stop: Shared node shutdown event.
        """
        self.config, self.stop = config, stop
        self.lock = threading.Lock()
        self.image = None
        self.received = 0.0
        self.worker = threading.Thread(target=self.capture_loop, name='picaso-camera', daemon=True)
        self.worker.start()

    def capture_loop(self) -> None:
        """Encode latest frames; webcam failures do not reconnect the sonar."""
        import cv2
        while not self.stop.is_set():
            camera = None
            try:
                camera = cv2.VideoCapture(self.config.camera_index, cv2.CAP_V4L2)
                while not self.stop.is_set() and camera.isOpened():
                    success, frame = camera.read()
                    received, received_mono = time.time_ns(), time.monotonic()
                    if not success:
                        break
                    success, jpeg = cv2.imencode('.jpg', frame,
                                                [cv2.IMWRITE_JPEG_QUALITY, self.config.camera_jpeg_quality])
                    if not success:
                        continue
                    message = CompressedImage()
                    message.header.stamp = timestamp(received)
                    message.header.frame_id = self.config.camera_frame_id
                    message.format = 'jpeg'
                    message.data = jpeg.tobytes()
                    with self.lock:
                        self.image, self.received = message, received_mono
            except Exception:
                # The camera is optional. The ROS thread reports stale/missing
                # images, while serial acquisition continues independently.
                pass
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
            field.name, getattr(defaults, field.name), ParameterDescriptor(read_only=True)).value
            for field in fields(defaults)}
        self.config = DriverConfig(**parameters)
        waveform = self.config.waveform_file or str(
            Path(get_package_share_directory('picaso_driver')) / 'sweeps/sweep_512_taper30.csv')
        self.waveform = load_waveform(Path(waveform)) if self.config.mode == 'A' else array('H')
        self.capture_pub = self.create_publisher(Capture, '~/capture', 2)
        self.image_pub = self.create_publisher(CompressedImage, '~/image/compressed', 2)
        self.status_pub = self.create_publisher(DiagnosticArray, '~/status', 2)
        self.shutdown_event = threading.Event()
        self.sensor = PicasoSerial(self.config, self.waveform, self.shutdown_event)
        self.lock = threading.Lock()
        self.status = {'state': 'connecting', 'captures': 0, 'errors': 0, 'message': 'Waiting for USB device'}
        self.camera = (CameraWorker(self.config, self.shutdown_event)
                       if self.config.camera_index >= 0 else None)
        self.create_timer(1.0, self.publish_status)
        self.worker = threading.Thread(target=self.acquire_loop, name='picaso-acquisition', daemon=True)
        self.worker.start()
        self.get_logger().info(
            f'PiCASo {self.config.mode}, {self.config.events} events; publishing packed uint16 PDM on ~/capture')

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
                    with self.lock:
                        self.status.update(state='streaming', captures=capture_id, message='Receiving complete captures',
                                           last_capture_duration_sec=(data.t_done_mono_ns - data.t_cmd_mono_ns) / 1e9,
                                           last_capture_id=capture_id, port=self.sensor.connection.port)
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
        diagnostic.level = DiagnosticStatus.OK if values['state'] == 'streaming' else DiagnosticStatus.WARN
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
