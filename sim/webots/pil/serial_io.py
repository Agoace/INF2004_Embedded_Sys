"""Opt-in Linux Debug Probe UART access. Importing this module opens nothing."""

import glob
import os
from pathlib import Path
import select
import termios


class SerialError(OSError):
    pass


def discover(selector=None, *, directory=Path('/dev/serial/by-id')):
    """Require one named Probe symlink. Never select a guessed ttyACM number."""
    directory = Path(directory)
    candidates = sorted(Path(p) for p in glob.glob(str(directory / '*Debug_Probe*')))
    if selector is not None:
        selected = Path(selector)
        if selected.parent != directory or 'Debug_Probe' not in selected.name:
            raise SerialError('select a Debug_Probe path under /dev/serial/by-id')
        candidates = [p for p in candidates if p == selected]
    if len(candidates) != 1 or not candidates[0].is_symlink():
        raise SerialError('need exactly one Debug_Probe by-id symlink; supply a selector')
    return candidates[0]


class ProbeSerial:
    """Exclusive, nonblocking 921600 8N1/no-flow-control descriptor.

    This constructor is called only by the explicit hardware CLI mode. A PC
    write timestamp does not establish when the Pico receives or stages a frame.
    """
    def __init__(self, path):
        path = Path(path)
        if path.parent != Path('/dev/serial/by-id') or 'Debug_Probe' not in path.name:
            raise SerialError('refuse a non-Probe by-id path')
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            import fcntl
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            attrs = termios.tcgetattr(self.fd)
            attrs[0] = attrs[1] = attrs[3] = 0
            attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            attrs[4] = attrs[5] = termios.B921600
            attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        except BaseException:
            self.close()
            raise

    def reset_input_buffer(self):
        """Drop queued kernel input; the caller must also drain Probe USB data."""
        termios.tcflush(self.fd, termios.TCIFLUSH)

    def read(self, maximum=4096):
        if not 1 <= maximum <= 4096:
            raise ValueError('read bound must be1..4096')
        if not select.select([self.fd], [], [], 0)[0]:
            return b''
        try:
            data = os.read(self.fd, maximum)
        except BlockingIOError:
            return b''
        if not data:
            raise SerialError('Probe disconnected')
        return data

    host_spans_supported = True

    def read_timed(self, maximum, now_us, *, observer=None, loop=0):
        """Measurement-only stamps around the same nonblocking select/read.

        No packet-size wait or changed tty flags. Empty polls are not logged.
        """
        if not 1 <= maximum <= 4096:
            raise ValueError('read bound must be1..4096')
        span = observer.begin('select', loop) if observer else None
        timing = {'select_enter_pc_us': now_us()}
        try:
            ready = select.select([self.fd], [], [], 0)[0]
        finally:
            timing['select_return_pc_us'] = now_us()
            if observer: observer.end(span)
        if not ready:
            return b'', timing
        span = observer.begin('read', loop) if observer else None
        timing['read_enter_pc_us'] = now_us()
        try:
            data = os.read(self.fd, maximum)
            timing['read_return_pc_us'] = now_us()
        except BlockingIOError:
            timing['read_return_pc_us'] = now_us()
            return b'', timing
        finally:
            if observer: observer.end(span)
        if not data:
            raise SerialError('Probe disconnected')
        return data, timing

    def write(self, data):
        if len(data) > 4096:
            raise ValueError('write exceeds4096 bytes')
        try:
            return os.write(self.fd, data)
        except BlockingIOError:
            return 0

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
