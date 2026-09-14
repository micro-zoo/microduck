import sys, struct, unittest, math, copy
from pathlib import Path
from unittest.mock import Mock, MagicMock
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import run_guarded_home as home


class Protocol:
    HEADER = b'\xff\xff\xfd\0'
    stuff = staticmethod(lambda x:x)
    crc16 = staticmethod(lambda x:0)
    instruction_packet = staticmethod(lambda id, op, data: bytes([id,op])+data)


class Tests(unittest.TestCase):
    def recovery_fixture(self):
        from export_joint_zero import JOINTS
        joints=[];before={}
        for id,name in JOINTS.items():
            zero={20:4071,30:1044}.get(id,2048)
            limits=[-math.pi/2,math.pi/2]
            if id==20:limits=[math.radians(-25),math.radians(30)]
            if id==30:limits[1]=math.radians(60)
            joints.append({'id':id,'name':name,'zero_tick':zero,'limits_rad':limits})
            b=bytearray(147);b[11]=3;raw=zero
            if id==20:raw=12
            if id==30:raw=round(zero+math.radians(63)*4096/math.tau)
            struct.pack_into('<i',b,132,raw);before[id]=b
        return {'position_mode':'extended_position','joints':joints},before

    def test_seam_and_slumped_neck_recover_without_editing_normal_calibration(self):
        document,before=self.recovery_fixture();original=copy.deepcopy(document)
        source=Path(__file__).resolve().parents[2]/'duck-control/src/model.rs'
        recovery,changes=home.recovery_calibration(document,before,source,10)
        self.assertEqual(document,original)
        self.assertEqual([c['id'] for c in changes],[30])
        neck=next(j for j in recovery['joints'] if j['id']==30)
        self.assertGreater(neck['limits_rad'][1],math.radians(63))
        struct.pack_into('<i',before[20],132,4108)
        second,_=home.recovery_calibration(document,before,source,10)
        self.assertEqual(second,recovery)

    def test_recovery_refuses_powered_or_implausible_pose(self):
        document,before=self.recovery_fixture()
        source=Path(__file__).resolve().parents[2]/'duck-control/src/model.rs'
        before[14][64]=1
        with self.assertRaisesRegex(ValueError,'OFF'):home.recovery_calibration(document,before,source,10)
        before[14][64]=0;struct.pack_into('<i',before[20],132,round(4071+math.radians(40)*4096/math.tau))
        with self.assertRaisesRegex(ValueError,'outside'):home.recovery_calibration(document,before,source,15)

    def test_hanging_neck_uses_long_slow_recovery_without_choosing_a_full_turn(self):
        document,before=self.recovery_fixture()
        source=Path(__file__).resolve().parents[2]/'duck-control/src/model.rs'
        struct.pack_into('<i',before[30],132,round(1044-math.radians(140)*4096/math.tau))
        with self.assertRaisesRegex(ValueError,'distance or speed'):home.recovery_calibration(document,before,source,10)
        result,_=home.recovery_calibration(document,before,source,30)
        neck=next(j for j in result['joints'] if j['id']==30)
        self.assertLess(neck['limits_rad'][0],math.radians(-140))
        self.assertLess(neck['limits_rad'][1]-neck['limits_rad'][0],math.tau)

    def test_restore_cannot_change_mode_torque_or_an_uncaptured_gain(self):
        before=bytearray(147);before[11]=3;before[84:86]=struct.pack('<H',400)
        home.restore_packet(Protocol,14,84,struct.pack('<H',400),before)
        for address,data in ((84,struct.pack('<H',800)),(64,b'\1'),(11,b'\3')):
            with self.assertRaises(RuntimeError):home.restore_packet(Protocol,14,address,data,before)

    def test_unconfirmed_off_stops_restoration_but_off_reaches_every_motor(self):
        protocol=Mock();wire=Mock();protocol.LinuxPort.return_value=MagicMock();protocol.LinuxPort.return_value.__enter__.return_value=wire
        protocol.instruction_packet=Protocol.instruction_packet
        protocol.ServoBus.return_value.read.return_value=b'\1'
        with self.assertRaisesRegex(RuntimeError,'Cannot confirm'):
            home.reconcile(protocol,'/dev/test',{id:bytes(147) for id in home.modes.IDS})
        self.assertEqual(wire.exchange.call_count,15)
        self.assertTrue(all(call.args[0][2:]==struct.pack('<HB',64,0) for call in wire.exchange.call_args_list))

    def test_extended_cleanup_confirms_all_off_before_restoring_mode_and_reset_gains(self):
        _,before=self.recovery_fixture()
        for b in before.values():struct.pack_into('<H',b,84,400)
        registers={id:bytearray(b) for id,b in before.items()}
        for b in registers.values():b[11]=4;b[64]=1;struct.pack_into('<H',b,84,800)
        protocol=Mock();wire=Mock();protocol.LinuxPort.return_value=MagicMock();protocol.LinuxPort.return_value.__enter__.return_value=wire
        for name in ('HEADER','stuff','crc16','instruction_packet'):setattr(protocol,name,getattr(Protocol,name))
        protocol.ServoBus.return_value.read.side_effect=lambda id,a,n:bytes(registers[id][a:a+n])
        mode_writes=[]
        def exchange(raw,timeout):
            if raw.startswith(Protocol.HEADER):id=raw[4];body=raw[7:-2]
            else:id=raw[0];body=raw[1:]
            address=struct.unpack_from('<H',body,1)[0];data=body[3:]
            if address==11:
                self.assertTrue(all(b[64]==0 for b in registers.values()))
                registers[id][84:86]=bytes(2);mode_writes.append(id)
            registers[id][address:address+len(data)]=data
            return []
        wire.exchange.side_effect=exchange
        from unittest.mock import patch
        with patch.object(home.modes,'snapshot',side_effect=lambda bus,id:bytes(registers[id])):
            final=home.reconcile(protocol,'/dev/test',before,restore_modes=True)
        self.assertEqual(mode_writes,list(home.modes.IDS))
        self.assertTrue(all(bytes.fromhex(final[id])==bytes(before[id]) for id in before))


if __name__=='__main__':unittest.main()
