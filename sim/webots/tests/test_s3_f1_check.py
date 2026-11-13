import unittest
from sim.webots.pil.s3_f1_check import accuracy_summary, time_summary


class GateTests(unittest.TestCase):
    def test_accuracy_gate_includes_reference_uncertainty(self):
        passed = accuracy_summary([.1, .2, .3], [.01, .02, .03])
        self.assertEqual(passed['check'], 'PASS')
        self.assertEqual(accuracy_summary([7.9], [.2])['check'], 'FAIL')
        self.assertEqual(accuracy_summary([1, 3], [0, 0])['check'], 'FAIL')
        self.assertEqual(accuracy_summary([], [])['check'], 'FAIL')

    def test_timing_gate_never_removes_a_scheduling_outlier(self):
        self.assertEqual(time_summary([100]*1000)['check'], 'PASS')
        times = [100]*1000+[501]
        result = time_summary(times)
        self.assertEqual(result['check'], 'FAIL')
        self.assertEqual(result['maximum_us'], 501)
        self.assertEqual(result['over_500us'], 1)


if __name__ == '__main__':
    unittest.main()
