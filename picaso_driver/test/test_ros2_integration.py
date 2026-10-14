"""Launch the actual bringup driver against a PTY, then inspect DDS messages."""

from array import array
from datetime import datetime
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

from diagnostic_msgs.msg import DiagnosticArray
import pytest
import rclpy
from picaso_msgs.msg import Capture

from serial_emulator import SerialEmulator, payload


@pytest.mark.parametrize('mode', ['A', 'P'])
def test_bringup_launch_and_dds_capture(tmp_path, mode):
    emulator = SerialEmulator(partial_first=True)
    config = tmp_path / 'test.yaml'
    config.write_text('air_sonar:\n  ros__parameters:\n'
                      f'    mode: {mode}\n    events: 20\n    serial_timeout: 0.1\n'
                      '    capture_timeout: 3.0\n    reconnect_period: 0.1\n'
                      '    upload_settle_time: 0.01\n    capture_period: 0.1\n')
    log_path = tmp_path / 'launch.log'
    rclpy.init()
    node = rclpy.create_node('picaso_driver_test')
    messages, diagnostics = [], []
    node.create_subscription(Capture, '/air_sonar/capture', messages.append, 10)
    node.create_subscription(DiagnosticArray, '/air_sonar/status', diagnostics.append, 10)
    with log_path.open('w') as log:
        process = subprocess.Popen(['ros2', 'launch', 'robot_bringup', 'air_sonar.launch.py',
                                    f'port:={emulator.port}', f'config_file:={config}'], stdout=log, stderr=log)
        try:
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline and (len(messages) < 3 or not diagnostics):
                rclpy.spin_once(node, timeout_sec=.1)
                if process.poll() is not None:
                    pytest.fail(log_path.read_text())
            assert len(messages) >= 3, log_path.read_text()
            for message in messages:
                assert message.mode == mode and message.events == 20
                assert message.byte_count == 327680 and len(message.pdm_words) == 163840
                words = array('H', message.pdm_words)
                if sys.byteorder != 'little':
                    words.byteswap()
                assert words.tobytes() == payload(20)
                assert message.t_cmd_unix_ns == message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
                assert message.t_done_unix_ns >= message.t_cmd_unix_ns
                assert message.t_done_mono_ns > message.t_cmd_mono_ns
                utc_ns = int(datetime.fromisoformat(message.t_cmd_utc).timestamp() * 1e9)
                assert abs(utc_ns - message.t_cmd_unix_ns) < 2000
                assert list(message.bit_to_microphone) == [2, 4, 6, 8, 10, 12, 14, 16, 1, 3, 5, 7, 9, 11, 13, 15]
                assert message.sample_rate_hz == 4444440.0
                assert len(message.transmit_waveform) == (1024 if mode == 'A' else 0)
                assert not message.has_image
            values = {v.key: v.value for v in diagnostics[-1].status[0].values}
            assert int(values['errors']) >= 1
            assert messages[0].capture_id == 1  # Partial response was not published.
            report = {'mode': mode, 'complete_messages': len(messages), 'capture_bytes': messages[0].byte_count,
                      'diagnostics': values, 'partial_response_dropped': True}
            print(json.dumps(report))
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            node.destroy_node()
            rclpy.shutdown()
            emulator.close()
