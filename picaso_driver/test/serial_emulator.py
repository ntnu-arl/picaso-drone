"""Deterministic PiCASo firmware peer on a Linux serial pseudo-terminal."""

from array import array
import os
import select
import sys
import threading
import time


def payload(events: int) -> bytes:
    """Return packed little-endian test words exercising both sample bytes."""
    words = array('H', [0x0001, 0x0100, 0x8000, 0xFFFF, 0x1234, 0, 0x00FF, 0xFF00]) * (events * 1024)
    if sys.byteorder != 'little':
        words.byteswap()
    return words.tobytes()


class SerialEmulator:
    """Accept waveform/capture commands and return deliberately fragmented data."""

    def __init__(self, partial_first: bool = False) -> None:
        self.master, self.slave = os.openpty()
        self.port = os.ttyname(self.slave)
        self.partial_first = partial_first
        self.commands: list[str] = []
        self.captures = 0
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self) -> None:
        """Serve the real command grammar; no socket or mock serial implementation."""
        buffered = b''
        try:
            while not self.stop.is_set():
                if not select.select([self.master], [], [], .05)[0]:
                    continue
                buffered += os.read(self.master, 4096)
                while b'\n' in buffered:
                    line, buffered = buffered.split(b'\n', 1)
                    command = line.decode('ascii').strip()
                    self.commands.append(command)
                    if command.startswith(('A,', 'P,')):
                        events = int(command.split(',')[1])
                        self.captures += 1
                        raw = payload(events)
                        if self.partial_first and self.captures == 1:
                            raw = raw[:len(raw) // 2]
                        for offset in range(0, len(raw), 511):
                            if self.stop.is_set():
                                return
                            data = memoryview(raw)[offset:offset + 511]
                            while data:
                                data = data[os.write(self.master, data):]
                            time.sleep(.0001)
        except OSError:
            pass

    def close(self) -> None:
        """Release both pseudo-terminal ends and the peer thread."""
        self.stop.set()
        os.close(self.master)
        os.close(self.slave)
        self.thread.join(timeout=1)
