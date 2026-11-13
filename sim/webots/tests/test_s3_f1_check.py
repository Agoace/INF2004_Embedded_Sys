import gc
import unittest
from sim.webots.pil.s3_f1_check import (
    MeasurementBuffers, accuracy_summary, time_summary, timing_gc_policy)


class GateTests(unittest.TestCase):
    def test_accuracy_gate_includes_reference_uncertainty(self):
        passed = accuracy_summary([.1, .2, .3], [.01, .02, .03])
        self.assertEqual(passed['check'], 'PASS')
        self.assertEqual(accuracy_summary([7.9], [.2])['check'], 'FAIL')
        self.assertEqual(accuracy_summary([1, 3], [0, 0])['check'], 'FAIL')
        self.assertEqual(accuracy_summary([], [])['check'], 'FAIL')

    def test_measurements_use_fixed_numeric_storage_not_retained_frame_tuples(self):
        buffers = MeasurementBuffers(10)
        size = buffers.elapsed.buffer_info()
        buffers.record(3, 500.1, 80, 3, 10, 20)
        self.assertEqual(buffers.elapsed.buffer_info(), size)
        self.assertEqual(list(buffers.rows())[3], (3, 500.1, 80, 3, 10, 20))

    def test_gc_experiment_restores_policy_and_freeze_state(self):
        enabled, frozen = gc.isenabled(), gc.get_freeze_count()
        with timing_gc_policy('freeze-disable') as info:
            self.assertFalse(gc.isenabled())
            self.assertGreater(gc.get_freeze_count(), 0)
            self.assertEqual(info['mode'], 'freeze-disable')
        self.assertEqual(gc.isenabled(), enabled)
        self.assertEqual(gc.get_freeze_count(), frozen)

    def test_timing_gate_never_removes_a_scheduling_outlier(self):
        self.assertEqual(time_summary([100]*1000)['check'], 'PASS')
        times = [100]*1000+[501]
        result = time_summary(times)
        self.assertEqual(result['check'], 'FAIL')
        self.assertEqual(result['maximum_us'], 501)
        self.assertEqual(result['over_500us'], 1)


if __name__ == '__main__':
    unittest.main()
