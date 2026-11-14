"""Real operator selectors; all capture/device access is mocked."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec=importlib.util.spec_from_file_location('pico_cdc_tool',Path(__file__).resolve().parents[1]/'tools/gate1_pico_cdc_capture.py')
tool=importlib.util.module_from_spec(spec);spec.loader.exec_module(tool)

class PicoCDCSelectorTests(unittest.TestCase):
    def invoke(self,name,accepted):
        with tempfile.TemporaryDirectory()as tmp:
            argv=['gate1_pico_cdc_capture.py','--hardware-authorized','--device',
                  '/dev/serial/by-id/'+name,'--output',str(Path(tmp)/'fresh')]
            with patch('sys.argv',argv),patch.object(tool,'capture_cdc',return_value={})as capture,patch.object(tool,'PicoCDC')as port:
                if accepted:
                    self.assertEqual(tool.main(),2)
                    capture.assert_called_once()
                else:
                    with self.assertRaises(SystemExit)as error:tool.main()
                    self.assertEqual(error.exception.code,2)
                    capture.assert_not_called()
                port.assert_not_called()

    def test_real_mtk3pico_by_id_is_accepted(self):
        self.invoke('usb-micro_T-Kernel_Forum_MTK3_Thread-Metric_RP2040_MTK3PICO-if00',True)

    def test_real_debug_probe_is_rejected(self):
        self.invoke('usb-Raspberry_Pi_Debug_Probe__CMSIS-DAP__E665B83887591E2E-if01',False)

    def test_cmsis_dap_cannot_pass_legacy_pico_prefix(self):
        self.invoke('usb-Raspberry_Pi_Pico_CMSIS-DAP-if00',False)

    def test_other_mtk3_identity_is_rejected(self):
        self.invoke('usb-micro_T-Kernel_Forum_MTK3_Thread-Metric_RP2040_OTHER-if00',False)

    def test_legacy_pico_by_id_remains_accepted(self):
        self.invoke('usb-Raspberry_Pi_Pico_E660123456789ABC-if00',True)
