"""Check lossless message clocks and optional webcam JPEG encoding."""

from array import array
import threading
from unittest.mock import Mock, patch
import time

import cv2
import numpy as np

from picaso_driver.node import CameraWorker, capture_message
from picaso_driver.protocol import Acquisition, DriverConfig


def test_nanoseconds_and_utc_are_not_rounded_through_float():
    data = Acquisition(array('H', [1, 256, 32768, 65535]),
                       1791966761123456789, 1791966761223456790, 1234567890123, 1234667890123)
    message = capture_message(data, DriverConfig(events=1), 42, array('H'))
    assert message.header.stamp.sec == 1791966761
    assert message.header.stamp.nanosec == 123456789
    assert message.t_cmd_unix_ns == data.t_cmd_unix_ns
    assert message.t_done_unix_ns == data.t_done_unix_ns
    assert message.t_cmd_mono_ns == data.t_cmd_mono_ns
    assert message.t_done_mono_ns == data.t_done_mono_ns
    assert message.t_cmd_utc.endswith('.123456+00:00')
    assert message.pdm_words == data.words
    assert message.capture_id == 42


def test_optional_camera_encodes_jpeg_and_uses_its_own_receipt_time():
    frame = np.full((16, 32, 3), 100, dtype=np.uint8)
    camera = Mock()
    camera.isOpened.return_value = True
    camera.read.return_value = (True, frame)
    stop = threading.Event()
    before = time.time_ns()
    with patch('cv2.VideoCapture', return_value=camera):
        worker = CameraWorker(DriverConfig(camera_index=0), stop)
        try:
            image = None
            deadline = time.monotonic() + 2
            while image is None and time.monotonic() < deadline:
                image = worker.latest()
                time.sleep(.001)
            after = time.time_ns()
        finally:
            stop.set()
            worker.worker.join(timeout=1)
    assert image is not None
    stamp = image.header.stamp.sec * 10**9 + image.header.stamp.nanosec
    assert before <= stamp <= after
    assert image.header.frame_id == 'air_sonar_camera_optical_frame'
    assert image.format == 'jpeg'
    decoded = cv2.imdecode(np.frombuffer(bytes(image.data), dtype=np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == frame.shape
    np.testing.assert_allclose(decoded, frame, atol=1)


def test_camera_stall_never_blocks_sonar_snapshot():
    stop, release_read = threading.Event(), threading.Event()
    camera = Mock()
    camera.isOpened.return_value = True
    camera.read.side_effect = lambda: (release_read.wait(2), None)
    with patch('cv2.VideoCapture', return_value=camera):
        worker = CameraWorker(DriverConfig(camera_index=0), stop)
        try:
            before = time.monotonic()
            assert worker.latest() is None
            assert time.monotonic() - before < .1
        finally:
            stop.set()
            release_read.set()
            worker.worker.join(timeout=1)
