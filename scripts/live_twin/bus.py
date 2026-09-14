"""Read-only group telemetry. Uses the existing serial exclusivity and CRC parser."""
from pathlib import Path
import struct,sys,time,math,importlib
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
try:import servo_config as protocol
except ModuleNotFoundError:protocol=None

def load_protocol(directory):
    global protocol
    sys.path.insert(0,str(directory));protocol=importlib.import_module('servo_config');return protocol

MOTORS=[(20,'left_hip_yaw'),(21,'left_hip_roll'),(22,'left_hip_pitch'),(23,'left_knee'),(24,'left_ankle'),
        (30,'neck_pitch'),(31,'head_pitch'),(32,'head_yaw'),(33,'head_roll'),(34,'mouth'),
        (10,'right_hip_yaw'),(11,'right_hip_roll'),(12,'right_hip_pitch'),(13,'right_knee'),(14,'right_ankle')]
IDS=[id for id,name in MOTORS]
RAD_PER_TICK=2*math.pi/4096

def sync_packet(ids,address=64,size=83):
    if not ids or len(set(ids))!=len(ids) or any(id not in IDS for id in ids):
        raise ValueError('Sync read requires distinct configured motor IDs')
    if address!=64 or size!=83:raise ValueError('Only the telemetry block is allowed')
    body=protocol.stuff(bytes([0x82])+struct.pack('<HH',address,size)+bytes(ids))
    packet=protocol.HEADER+b'\xfe'+struct.pack('<H',len(body)+2)+body
    return packet+struct.pack('<H',protocol.crc16(packet))

def status_payload(frame,expected_size=None):
    if len(frame.body)<2 or frame.body[0]!=0x55:raise protocol.CommunicationError('Unexpected status frame')
    error=frame.body[1]
    if error&0x7f:raise protocol.CommunicationError(f'Motor {frame.device} status error 0x{error:02x}')
    payload=frame.body[2:]
    if expected_size is not None and len(payload)!=expected_size:raise protocol.CommunicationError('Wrong telemetry payload length')
    return payload,error

def decode_telemetry(frame):
    data,status_error=status_payload(frame,83)
    return {'id':frame.device,'torque':data[0],'hardware_error':data[6],'watchdog':data[34],
        'goal_raw_tick':struct.unpack_from('<i',data,52)[0],
        'current_ma':struct.unpack_from('<h',data,62)[0],
        'velocity_raw':struct.unpack_from('<i',data,64)[0],
        'raw_tick':struct.unpack_from('<i',data,68)[0],
        'voltage_v':struct.unpack_from('<H',data,80)[0]/10,'temperature_c':data[82],
        'status_error':status_error}

class ReadBus:
    def __init__(self,wire):self.wire=wire
    def read(self,id,address,size):
        packet=protocol.instruction_packet(id,2,struct.pack('<HH',address,size))
        frames=self.wire.exchange(packet,.06)
        if len(frames)!=1 or frames[0].device!=id:raise protocol.CommunicationError(f'ID {id} did not give one read response')
        return status_payload(frames[0],size)
    def metadata(self,id):
        data,error=self.read(id,0,64)
        actual={'id':data[7],'model':int.from_bytes(data[0:2],'little'),'firmware':data[6],
            'baud_code':data[8],'return_delay':data[9],'drive_mode':data[10],'operating_mode':data[11],
            'secondary_id':data[12],'protocol':data[13],'homing_offset':struct.unpack_from('<i',data,20)[0],
            'max_position_tick':struct.unpack_from('<I',data,48)[0],
            'min_position_tick':struct.unpack_from('<I',data,52)[0],'status_error':error}
        if (actual['id'],actual['model'],actual['baud_code'],actual['protocol'])!=(id,1200,3,2):
            raise protocol.CommunicationError(f'Unexpected identity/baud at ID {id}: {actual}')
        return actual
    def sample(self):
        started=time.monotonic()
        frames=self.wire.exchange(sync_packet(IDS),.08)
        found={};errors=[];seen=set()
        for frame in frames:
            if frame.device not in IDS or frame.device in seen:
                raise protocol.CommunicationError('Unexpected or duplicate ID in group read')
            seen.add(frame.device)
            try:found[frame.device]=decode_telemetry(frame)
            except protocol.CommunicationError as error:errors.append(str(error))
        return found,errors,(time.monotonic()-started)*1000

def relative_tick(raw,zero):
    # Read-only local joint coordinates across the encoder seam. Never use this
    # modulo conversion to construct a single-turn position command.
    return (raw-zero+2048)%4096-2048
