import sys,unittest,struct,re
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import configure_extended_position as mode
import export_joint_zero as export
class Protocol:
    HEADER=b'\xff\xff\xfd\x00'
    @staticmethod
    def stuff(b):return b
    @staticmethod
    def crc16(b):return 0
class FakeBus:
    def __init__(self):
        self.data={}
        for id in mode.IDS:
            b=bytearray(147);b[:2]=(1200).to_bytes(2,'little');b[7]=id;b[8]=3;b[11]=3;b[12]=255;b[13]=2
            b[80:82]=(200).to_bytes(2,'little');b[132:136]=(4071 if id==20 else 532 if id==13 else 2048).to_bytes(4,'little',signed=True)
            self.data[id]=b
    def read(self,id,a,n):return bytes(self.data[id][a:a+n])
class FakeWire:
    def __init__(self,bus):self.bus=bus;self.writes=[]
    def assert_free(self):pass
    def exchange(self,p,timeout):
        id=p[4];a=int.from_bytes(p[8:10],'little');data=p[10:-2]
        self.writes.append((id,a,data));self.bus.data[id][a:a+len(data)]=data
        if a==11:self.bus.data[id][80:82]=(800).to_bytes(2,'little')
        return [SimpleNamespace(device=id,body=b'\x55\x00')]
class Journal:
    def record(self,*args,**kw):pass
class Tests(unittest.TestCase):
    def entries(self):return {id:{'zero_tick':4071 if id==20 else 532 if id==13 else 2048,'limits_rad':[-1.5,1.5]} for id in mode.IDS}
    def test_mode_tool_never_permits_torque_or_broadcast_writes(self):
        for id,a,value in [(20,64,b'\x01'),(254,11,b'\x04'),(20,11,b'\x02'),(20,20,b'\0'*4)]:
            with self.assertRaises(ValueError):mode.packet(Protocol,id,a,value)
    @patch.object(mode,'stopped')
    @patch.object(mode.time,'sleep')
    def test_mode_and_signed_goal_probes_restore_gains_and_hold_pose(self,*_):
        bus=FakeBus();wire=FakeWire(bus)
        mode.execute(Protocol,wire,bus,self.entries(),Journal(),True,probe_goals=True)
        for id,b in bus.data.items():
            self.assertEqual(b[11],4);self.assertEqual(b[64],0);self.assertEqual(int.from_bytes(b[80:82],'little'),200)
            self.assertEqual(b[116:120],b[132:136])
        goals=[int.from_bytes(v,'little',signed=True) for _,a,v in wire.writes if a==116]
        self.assertTrue(any(v<0 for v in goals));self.assertTrue(any(v>4095 for v in goals))
    @patch.object(mode,'stopped')
    @patch.object(mode.time,'sleep')
    def test_temporary_probe_restores_original_modes_and_gains(self,*_):
        bus=FakeBus();wire=FakeWire(bus)
        mode.execute(Protocol,wire,bus,self.entries(),Journal(),True,True)
        for b in bus.data.values():
            self.assertEqual(b[11],3);self.assertEqual(b[64],0)
            self.assertEqual(int.from_bytes(b[80:82],'little'),200)
            self.assertEqual(b[116:120],b[132:136])
    @patch.object(mode,'stopped')
    @patch.object(mode.time,'sleep')
    def test_non_latching_goal_aborts_but_restores_mode_and_tuning(self,*_):
        class TrackingBus(FakeBus):
            def read(self,id,a,n):
                self.data[id][116:120]=self.data[id][132:136]
                return super().read(id,a,n)
        bus=TrackingBus();wire=FakeWire(bus)
        with self.assertRaisesRegex(RuntimeError,'did not retain'):
            mode.execute(Protocol,wire,bus,self.entries(),Journal(),True,True,True)
        self.assertTrue(all(b[11]==3 and b[64]==0 for b in bus.data.values()))
        self.assertTrue(all(int.from_bytes(b[80:82],'little')==200 for b in bus.data.values()))
    def test_one_powered_motor_prevents_every_write(self):
        bus=FakeBus();bus.data[34][64]=1;wire=FakeWire(bus)
        with self.assertRaises(RuntimeError):mode.execute(Protocol,wire,bus,self.entries(),Journal(),True)
        self.assertEqual(wire.writes,[])
    def test_export_joint_order_matches_runtime_sources(self):
        root=Path(__file__).resolve().parents[2]
        model=(root/'duck-control/src/model.rs').read_text()
        proto=(root/'duck-ipc-proto/src/lib.rs').read_text()
        ids=re.search(r'pub const JOINT_IDS:.*?= \[(.*?)\];',model,re.S).group(1)
        ids=list(map(int,re.findall(r'\d+',re.sub(r'//[^\n]*','',ids))))
        names=re.search(r'pub const JOINT_NAMES:.*?= \[(.*?)\];',proto,re.S).group(1)
        names=re.findall(r'"([^"]+)"',names)
        self.assertEqual(list(export.JOINTS.items()),list(zip(ids,names)))
    def test_export_uses_current_model_limits_and_preserves_closed_mouth(self):
        source={'mouth_reference':'closed_is_zero','joints':[{'id':id,'name':name,'zero_tick':4050 if id==34 else 4071,'reference_rad':0} for id,name in export.JOINTS.items()]}
        data=export.convert(source,extended=True)
        self.assertEqual(data['position_mode'],'extended_position');self.assertEqual(len(data['joints']),15)
        mouth=next(e for e in data['joints'] if e['id']==34)
        self.assertTrue(0<=mouth['zero_tick']<4096)
        self.assertAlmostEqual((mouth['zero_tick']+export.MOUTH_CLOSED/export.RADIANS_PER_TICK)%4096,4050)
        self.assertEqual(mouth['limits_rad'][0],export.MOUTH_CLOSED)
if __name__=='__main__':unittest.main()
