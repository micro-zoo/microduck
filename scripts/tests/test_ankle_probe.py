import sys,unittest,os,tempfile,select,struct
from pathlib import Path
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import probe_ankle_position as ankle
class Protocol:
    HEADER=b'\xff\xff\xfd\x00'
    @staticmethod
    def stuff(b):return b
    @staticmethod
    def crc16(b):return 0
class Tests(unittest.TestCase):
    def sample(self,**kw):
        s={'torque_enable':1,'hardware_error':0,'status_error':0,'watchdog':ankle.WATCHDOG,'current_ma':20,'pwm_raw':10,'voltage_v':5.0,'temperature_c':29,'position_tick':2000,'position_trajectory_tick':1990,'velocity_raw':1,'elapsed_s':1.0};s.update(kw);return s
    def test_fault_current_temperature_reverse_and_stall_abort(self):
        for s in [self.sample(current_ma=101),self.sample(temperature_c=40),self.sample(position_tick=2004),self.sample(hardware_error=32),self.sample(pwm_raw=64)]:
            with self.subTest(s=s):
                with self.assertRaises(RuntimeError):ankle.check_sample(s,2000,-1,1990,29,[])
        with self.assertRaisesRegex(RuntimeError,'No progress'):
            ankle.check_sample(self.sample(),2000,-1,1990,29,[self.sample(elapsed_s=.4)])
    def test_only_ankle_toe_down_goals_and_low_output_are_writable(self):
        p=object.__new__(ankle.Probe);p.p=Protocol;p.p_gain=400;p.id=14;p.sign=-1;p.start=2000;p.saved={}
        p.packet(116,(1966).to_bytes(4,'little',signed=True))
        for a,v in [(116,(2001).to_bytes(4,'little')),(116,(1965).to_bytes(4,'little')),(100,(885).to_bytes(2,'little')),(20,bytes(4)),(11,b'\x01')]:
            with self.assertRaises(ValueError):p.packet(a,v)
        p.id=13
        with self.assertRaises(ValueError):p.packet(64,b'\x01')
    def test_failed_torque_off_prevents_all_restoration_writes(self):
        p=object.__new__(ankle.Probe);p.emergency_off=Mock(side_effect=RuntimeError('off not confirmed'));p.send=Mock()
        with self.assertRaises(RuntimeError):p.restore()
        p.send.assert_not_called()
    def test_emergency_off_does_not_depend_on_logging(self):
        p=object.__new__(ankle.Probe);p.send=Mock();p.read=Mock(return_value=(b'\x00',0));p.armed=True;p.journal=Mock();p.guard=None
        p.emergency_off();self.assertFalse(p.armed)
        p.send.assert_called_once_with(64,b'\x00',verify=False)
        p.journal.record.assert_not_called()
    def test_independent_cutoff_sends_only_off_packet_on_deadline(self):
        read_fd,write_fd=os.pipe()
        try:
            with tempfile.TemporaryDirectory() as d:
                guard=ankle.Cutoff(write_fd,b'OFF',.05,str(Path(d)/'guard.log'))
                self.assertTrue(select.select([read_fd],[],[],1)[0])
                self.assertEqual(os.read(read_fd,3),b'OFF')
                self.assertEqual(guard.cancel(),2)
        finally:os.close(read_fd);os.close(write_fd)
    def test_confirmed_off_cancels_independent_packet(self):
        read_fd,write_fd=os.pipe()
        try:
            with tempfile.TemporaryDirectory() as d:
                guard=ankle.Cutoff(write_fd,b'OFF',1,str(Path(d)/'guard.log'))
                self.assertEqual(guard.cancel(),0)
                self.assertFalse(select.select([read_fd],[],[],.02)[0])
        finally:os.close(read_fd);os.close(write_fd)
    def test_closed_parent_pipe_triggers_immediate_cutoff(self):
        read_fd,write_fd=os.pipe()
        try:
            with tempfile.TemporaryDirectory() as d:
                guard=ankle.Cutoff(write_fd,b'OFF',5,str(Path(d)/'guard.log'))
                os.close(guard.fd);guard.fd=None
                self.assertTrue(select.select([read_fd],[],[],1)[0])
                self.assertEqual(os.read(read_fd,3),b'OFF')
                _,status=os.waitpid(guard.pid,0);self.assertEqual(os.waitstatus_to_exitcode(status),2)
        finally:os.close(read_fd);os.close(write_fd)
    def test_pid_uses_verified_d_i_p_addresses_before_power(self):
        p=object.__new__(ankle.Probe);p.p_gain=400;registers={}
        p.write=lambda a,v,n:registers.update({a:v})
        p.read=lambda a,n:(struct.pack('<HHH',registers[80],registers[82],registers[84]),0)
        p.configure_pid()
        self.assertEqual(registers,{80:0,82:0,84:400,88:0,90:0})
        p.read=lambda a,n:(struct.pack('<HHH',400,0,0),0)
        with self.assertRaisesRegex(RuntimeError,'D/I/P readback'):p.configure_pid()
    def test_slow_profile_and_new_error_do_not_count_as_a_sustained_stall(self):
        # Command endpoint can be far away while the actual profile is still near the pose.
        ankle.check_sample(self.sample(position_trajectory_tick=1999),2000,-1,1977,29,[self.sample(elapsed_s=.4,position_trajectory_tick=2000)])
        # A newly grown following error has not persisted for the stall window yet.
        ankle.check_sample(self.sample(position_trajectory_tick=1990),2000,-1,1990,29,[self.sample(elapsed_s=.4,position_trajectory_tick=2000)])
if __name__=='__main__':unittest.main()
