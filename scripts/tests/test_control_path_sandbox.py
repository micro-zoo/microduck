import os,sys,tempfile,subprocess,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import check_calibrated_control_path as inner
import run_calibrated_control_path as outer

class Tests(unittest.TestCase):
    def test_direct_root_execution_is_refused(self):
        with patch.object(inner.os,'geteuid',return_value=0):
            with self.assertRaisesRegex(RuntimeError,'Refusing root'):inner.assert_test_isolation()
    def test_visible_manager_socket_is_refused_even_without_privileges(self):
        with patch.object(inner.os,'geteuid',return_value=65534), patch.object(inner.Path,'read_text',return_value='NoNewPrivs:\t1\nCapEff:\t0000000000000000\n'), patch.dict(os.environ,{'DUCK_CONTROL_PATH_SANDBOX':'1'}), patch.object(inner.os,'stat',return_value=object()):
            with self.assertRaisesRegex(RuntimeError,'must be hidden'):inner.assert_test_isolation()
    def test_launcher_removes_devices_privileges_and_host_manager_access(self):
        cmd=outer.sandbox_command('fixture',Path('/var/tmp/source'),Path('/var/tmp/output'),65534,65534,True)
        for value in ('User=65534','Group=65534','NoNewPrivileges=yes','PrivateDevices=yes','CapabilityBoundingSet=','SystemCallFilter=~@reboot @mount','InaccessiblePaths=-/run/systemd -/run/dbus -/etc/robot -/opt/robot'):
            self.assertIn('--property='+value,cmd)
        self.assertIn('/run/fixture/source/bin/robotd',cmd)
    def test_poweroff_stub_records_arguments_without_executing_them(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);commands=inner.host_action_stubs(root);log=root/'actions.log';marker=root/'must-not-exist'
            subprocess.run([str(commands/'setsid'),'sh','-c',f'touch {marker}; systemctl poweroff'],env={**os.environ,'DUCK_TEST_HOST_ACTION_LOG':str(log)},check=True)
            self.assertIn('systemctl poweroff',log.read_text());self.assertFalse(marker.exists())
if __name__=='__main__':unittest.main()
