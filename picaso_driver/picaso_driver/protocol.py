"""PiCASo's unframed USB protocol; independent of ROS and offline DSP tools."""

from array import array
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import threading
import time
import warnings

import serial
from serial.tools import list_ports

SAMPLES_PER_EVENT = 8192
WAVEFORM_SAMPLES = 1024
SAMPLE_RATE_HZ = 4.44444e6
# Same nominal 343 m/s round-trip conversion as picaso_record.py.
RANGE_PER_EVENT = SAMPLES_PER_EVENT / SAMPLE_RATE_HZ * 343.0 / 2
BIT_TO_MICROPHONE = tuple(range(2, 17, 2)) + tuple(range(1, 16, 2))


@dataclass(frozen=True)
class DriverConfig:
    """Read-only acquisition parameters; restart the node to change them."""

    port: str = '/dev/picaso'
    serial_number: str = ''
    baud_rate: int = 115200
    mode: str = 'A'
    events: int = 20
    max_range: float = 0.0
    shots: int = 0
    tag: str = ''
    waveform_file: str = ''
    frame_id: str = 'air_sonar_link'
    capture_period: float = 0.0
    serial_timeout: float = 1.0
    capture_timeout: float = 10.0
    reconnect_period: float = 2.0
    upload_settle_time: float = 0.3
    camera_index: int = -1
    camera_warmup_frames: int = 10
    camera_frame_id: str = 'air_sonar_camera_optical_frame'
    camera_jpeg_quality: int = 90
    camera_max_age: float = 1.0

    def __post_init__(self) -> None:
        """Validate parameter types, protocol lengths, frames and finite timeouts."""
        for name in ('port', 'serial_number', 'mode', 'tag', 'waveform_file',
                     'frame_id', 'camera_frame_id'):
            if not isinstance(getattr(self, name), str):
                raise ValueError(f'{name} must be a string')
        for name in ('baud_rate', 'events', 'shots', 'camera_index',
                     'camera_warmup_frames', 'camera_jpeg_quality'):
            if type(getattr(self, name)) is not int:
                raise ValueError(f'{name} must be an integer')
        if self.mode not in ('A', 'P'):
            raise ValueError('mode must be A (active) or P (passive)')
        if not 1 <= self.events <= 32:
            raise ValueError('events must be in 1..32')
        if self.baud_rate <= 0:
            raise ValueError('baud_rate must be positive')
        for name in ('serial_timeout', 'capture_timeout', 'reconnect_period', 'camera_max_age'):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('capture_period', 'upload_settle_time', 'max_range'):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        for name in ('serial_timeout', 'capture_timeout', 'reconnect_period',
                     'capture_period', 'upload_settle_time'):
            if getattr(self, name) > threading.TIMEOUT_MAX:
                raise ValueError(f'{name} exceeds the supported wait limit of {threading.TIMEOUT_MAX} seconds')
        if self.serial_timeout > self.capture_timeout:
            raise ValueError('serial_timeout must not exceed capture_timeout')
        if not self.frame_id.strip():
            raise ValueError('frame_id must not be empty or whitespace')
        if self.camera_index < -1:
            raise ValueError('camera_index must be -1 (disabled) or a nonnegative device index')
        if self.camera_index >= 0 and not self.camera_frame_id.strip():
            raise ValueError('camera_frame_id must not be empty when the camera is enabled')
        if self.camera_warmup_frames < 0:
            raise ValueError('camera_warmup_frames must be nonnegative')
        if not 1 <= self.camera_jpeg_quality <= 100:
            raise ValueError('camera_jpeg_quality must be in 1..100')
        if self.shots < 0:
            raise ValueError('shots must be nonnegative; zero means continuous acquisition')
        object.__setattr__(self, 'tag', self.tag.strip())

    @property
    def capture_events(self) -> int:
        """Resolve the range override to the firmware's bounded event count.

        :return: Configured events, or ceil(max_range / range_per_event), capped at 32.
        """
        if not self.max_range:
            return self.events
        # Equivalent to the recorder's max(1, min(32, ceil(range / RANGE_PER_EVENT))).
        # Bound the division first so even an enormous finite range cannot overflow.
        return max(1, min(32, math.ceil(min(self.max_range / RANGE_PER_EVENT, 32))))


@dataclass(frozen=True)
class Acquisition:
    """Complete PDM data and host clocks sampled around the serial transaction."""

    words: array
    t_cmd_unix_ns: int
    t_done_unix_ns: int
    t_cmd_mono_ns: int
    t_done_mono_ns: int


def load_waveform(path: Path) -> array:
    """Load and validate a DAC table, padding unused entries with silence.

    :param path: Nonempty CSV of integer samples in 0..4095; use the first 1024.
    :return: The complete 1024-entry uint16 table sent to the firmware.
    """
    with path.open(newline='') as stream:
        values = [int(cell.strip()) for row in csv.reader(stream) for cell in row if cell.strip()]
    if not values or any(not 0 <= v <= 4095 for v in values):
        raise ValueError('Waveform must contain integer DAC values in 0..4095 and must not be empty')
    if len(values) > WAVEFORM_SAMPLES:
        warnings.warn(f'Waveform has {len(values)} samples; using the first {WAVEFORM_SAMPLES}',
                      RuntimeWarning, stacklevel=2)
        values = values[:WAVEFORM_SAMPLES]
    return array('H', values + [2048] * (WAVEFORM_SAMPLES - len(values)))


class PicasoSerial:
    """Own one serial connection, including waveform upload and exact-length reads."""

    def __init__(self, config: DriverConfig, waveform: array, stop: threading.Event) -> None:
        """Configure the transport without opening the device.

        :param config: Validated acquisition settings.
        :param waveform: Full DAC table, or empty for passive acquisition.
        :param stop: Event used to interrupt reconnect and acquisition waits.
        """
        self.config = config
        self.waveform = waveform
        self.stop = stop
        self.connection: serial.Serial | None = None

    def open(self) -> None:
        """Connect, discard stale data until quiet, and upload the active waveform.

        :return: None; failures are retried by the node, not hidden here.
        """
        port = self.config.port
        if not port:
            ports = list(list_ports.comports())
            matches = [p.device for p in ports
                       if (p.vid, p.pid) == (0x0483, 0x5740)
                       and (not self.config.serial_number or p.serial_number == self.config.serial_number)]
            if len(matches) != 1:
                available = ', '.join(
                    f'{p.device} (VID={p.vid!r}, PID={p.pid!r}, serial={p.serial_number!r})'
                    for p in ports) or 'none'
                raise OSError(f'Expected one PiCASo USB device, found {len(matches)}; '
                              f'set port or serial_number. Available ports: {available}')
            port = matches[0]
        self.connection = serial.Serial(port, self.config.baud_rate, timeout=self.config.serial_timeout,
                                        write_timeout=self.config.capture_timeout, exclusive=True)
        self.connection.reset_input_buffer()
        # There is no packet header or reset command. Require a quiet interval
        # before sending new commands after a potentially partial prior response.
        deadline = time.monotonic() + self.config.capture_timeout
        while self.connection.read(4096):
            if self.stop.is_set() or time.monotonic() >= deadline:
                raise TimeoutError('Serial input did not become quiet before acquisition')
        if self.stop.is_set():
            raise InterruptedError('Serial connection interrupted')
        if self.waveform:
            payload = ''.join(f'D,{i},{value}\n' for i, value in enumerate(self.waveform)).encode('ascii')
            self._write(payload)
            self.stop.wait(self.config.upload_settle_time)
        if self.stop.is_set():
            raise InterruptedError('Serial connection interrupted')

    def _write(self, payload: bytes) -> None:
        """Write one complete command/table or fail without starting a capture."""
        if self.connection.write(payload) != len(payload):
            raise serial.SerialException('Incomplete command write')

    def acquire(self) -> Acquisition:
        """Send one capture command and accumulate exactly the expected PDM words.

        :return: A complete acquisition; partial captures raise and are never published.
        """
        total = self.config.capture_events * SAMPLES_PER_EVENT * 2
        if self.stop.is_set():
            raise InterruptedError('Acquisition interrupted before sending a command')
        command_unix, command_mono = time.time_ns(), time.monotonic_ns()
        self._write(f'{self.config.mode},{self.config.capture_events},0\n'.encode('ascii'))
        deadline = command_mono + int(self.config.capture_timeout * 1_000_000_000)
        data = bytearray()
        while len(data) < total:
            remaining = (deadline - time.monotonic_ns()) / 1_000_000_000
            if self.stop.is_set() or remaining <= 0:
                raise TimeoutError(f'Capture interrupted/timed out: {len(data)}/{total} bytes')
            self.connection.timeout = min(self.config.serial_timeout, remaining)
            chunk = self.connection.read(min(total - len(data), 65536))
            if not chunk:
                # A quiet poll is not an incomplete capture until its full deadline.
                continue
            data.extend(chunk)
        done_unix, done_mono = time.time_ns(), time.monotonic_ns()
        self.connection.timeout = self.config.serial_timeout
        words = array('H')
        words.frombytes(data)
        if sys.byteorder != 'little':
            words.byteswap()
        return Acquisition(words, command_unix, done_unix, command_mono, done_mono)

    def close(self) -> None:
        """Release a disconnected or idle device.

        :return: None.
        """
        if self.connection is not None:
            self.connection.close()
            self.connection = None

    def interrupt(self) -> None:
        """Unblock an in-flight read/write so shutdown need not wait for a timeout."""
        connection = self.connection
        if connection is not None:
            try:
                connection.cancel_read()
                connection.cancel_write()
            except (OSError, serial.SerialException):
                pass
