"""Read-only caller PR_GET_TIMERSLACK; no SET or policy change."""
import ctypes
import threading
import unittest
from unittest.mock import Mock, patch
from pil.gate1_timer_slack import read_timer_slack


class TimerSlackTests(unittest.TestCase):
    def test_get_uses_explicit_widths_and_calling_thread(self):
        get=Mock(return_value=50000)
        with patch('pil.gate1_timer_slack.sys.platform','linux'),patch('pil.gate1_timer_slack.ctypes.CDLL')as libc:
            libc.return_value.prctl=get
            result=read_timer_slack()
            libc.assert_called_once_with(None,use_errno=True)
        get.assert_called_once_with(30,0,0,0,0)
        self.assertEqual(get.argtypes,[ctypes.c_int]+[ctypes.c_ulong]*4)
        self.assertEqual(result['value_ns'],50000)
        self.assertEqual(result['status'],'KNOWN')
        self.assertEqual(result['calling_thread_native_id'],threading.get_native_id())

    def test_error_is_unknown_not_zero_or_guessed_default(self):
        with patch('pil.gate1_timer_slack.sys.platform','linux'),patch('pil.gate1_timer_slack.ctypes.CDLL')as libc,patch('pil.gate1_timer_slack.ctypes.get_errno',return_value=1):
            libc.return_value.prctl.return_value=-1
            result=read_timer_slack()
        self.assertEqual(result['status'],'UNKNOWN')
        self.assertIsNone(result['value_ns'])
        self.assertEqual(result['errno'],1)

    def test_non_linux_does_not_call_libc(self):
        with patch('pil.gate1_timer_slack.sys.platform','other'),patch('pil.gate1_timer_slack.ctypes.CDLL')as libc:
            self.assertEqual(read_timer_slack()['status'],'UNKNOWN')
            libc.assert_not_called()


if __name__=='__main__':unittest.main()
