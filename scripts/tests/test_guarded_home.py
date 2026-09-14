import sys, struct, unittest
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


if __name__=='__main__':unittest.main()
