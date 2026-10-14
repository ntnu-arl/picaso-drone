"""Check the wire format, waveform padding, timestamps and timeout recovery."""

from array import array
import sys
import threading
import time
from types import SimpleNamespace

import pytest

from picaso_driver.protocol import DriverConfig, PicasoSerial, load_waveform
from serial_emulator import SerialEmulator, payload


def original_bytes(words: array) -> bytes:
    copy = array('H', words)
    if sys.byteorder != 'little':
        copy.byteswap()
    return copy.tobytes()


@pytest.mark.parametrize('mode', ['A', 'P'])
def test_real_serial_fragments_words_and_upload_padding(tmp_path, mode):
    waveform_file = tmp_path / 'sweep.csv'
    waveform_file.write_text('0,4095,1234\n')
    waveform = load_waveform(waveform_file) if mode == 'A' else array('H')
    emulator = SerialEmulator()
    sensor = PicasoSerial(DriverConfig(port=emulator.port, events=1, mode=mode,
                         serial_timeout=.1, capture_timeout=2., upload_settle_time=0.), waveform, threading.Event())
    try:
        sensor.open()
        before = time.time_ns()
        capture = sensor.acquire()
        assert original_bytes(capture.words) == payload(1)
        assert len(capture.words) == 8192
        assert capture.words[:4] == array('H', [1, 256, 32768, 65535])
        assert before <= capture.t_cmd_unix_ns <= capture.t_done_unix_ns <= time.time_ns()
        assert capture.t_cmd_mono_ns < capture.t_done_mono_ns
        assert f'{mode},1,0' in emulator.commands
        uploads = [c for c in emulator.commands if c.startswith('D,')]
        assert len(uploads) == (1024 if mode == 'A' else 0)
        if mode == 'A':
            assert uploads[:3] == ['D,0,0', 'D,1,4095', 'D,2,1234']
            assert uploads[-1] == 'D,1023,2048'
    finally:
        sensor.close()
        emulator.close()


def test_partial_capture_is_rejected_and_waveform_reuploaded(tmp_path):
    file = tmp_path / 'sweep.csv'
    file.write_text('2048')
    emulator = SerialEmulator(partial_first=True)
    sensor = PicasoSerial(DriverConfig(port=emulator.port, events=1, serial_timeout=.1,
                         capture_timeout=2., upload_settle_time=0.), load_waveform(file), threading.Event())
    try:
        sensor.open()
        with pytest.raises(TimeoutError, match='8192/16384'):
            sensor.acquire()
        sensor.close()
        sensor.open()
        assert original_bytes(sensor.acquire().words) == payload(1)
        assert len([c for c in emulator.commands if c.startswith('D,')]) == 2048
    finally:
        sensor.close()
        emulator.close()


@pytest.mark.parametrize('content', ['', '4096', '-1', '1.5', ','.join(['1'] * 1025)])
def test_invalid_waveform_rejected(tmp_path, content):
    file = tmp_path / 'sweep.csv'
    file.write_text(content)
    with pytest.raises(ValueError):
        load_waveform(file)


@pytest.mark.parametrize('settings', [{'events': 0}, {'events': 33}, {'mode': 'X'},
                                    {'serial_timeout': 0.}, {'capture_timeout': float('nan')},
                                    {'capture_period': -1.}, {'camera_jpeg_quality': 101}])
def test_invalid_driver_parameters(settings):
    with pytest.raises(ValueError):
        DriverConfig(**settings)


def test_usb_auto_discovery_filters_by_serial_number(monkeypatch):
    from picaso_driver.protocol import list_ports
    emulator = SerialEmulator()
    monkeypatch.setattr(list_ports, 'comports', lambda: [
        SimpleNamespace(device='/dev/not-the-sonar', vid=0x0483, pid=0x5740, serial_number='other'),
        SimpleNamespace(device=emulator.port, vid=0x0483, pid=0x5740, serial_number='sonar')])
    sensor = PicasoSerial(DriverConfig(port='', serial_number='sonar', mode='P', events=1,
                         serial_timeout=.1), array('H'), threading.Event())
    try:
        sensor.open()
        assert original_bytes(sensor.acquire().words) == payload(1)
    finally:
        sensor.close()
        emulator.close()
