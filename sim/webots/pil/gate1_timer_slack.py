"""Read only the calling Linux thread's timer slack; never set a policy."""
import ctypes
import sys
import threading


def read_timer_slack():
    result = {'status': 'UNKNOWN', 'value_ns': None,
              'source': 'prctl(PR_GET_TIMERSLACK)',
              'calling_thread_native_id': threading.get_native_id()}
    if sys.platform != 'linux':
        return dict(result, reason='not Linux')
    try:
        libc = ctypes.CDLL(None, use_errno=True)
        get = libc.prctl
        # prctl is variadic: give every argument an explicit C width.
        get.argtypes = [ctypes.c_int] + [ctypes.c_ulong] * 4
        get.restype = ctypes.c_int
        ctypes.set_errno(0)
        value = get(30, 0, 0, 0, 0)  # Linux PR_GET_TIMERSLACK, not SET (29).
        if value < 0:
            return dict(result, reason='PR_GET_TIMERSLACK failed', errno=ctypes.get_errno())
        return dict(result, status='KNOWN', value_ns=value)
    except (AttributeError, OSError, TypeError, ValueError) as error:
        return dict(result, reason=type(error).__name__ + ': ' + str(error))
