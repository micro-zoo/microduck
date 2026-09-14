#!/usr/bin/env python3
"""Independent pose guardian: kill the goal producer before sending torque OFF.

Runs separately from the web server. It requires browser leases AND progress from
the Rust control loop. All UART packets in this module disable torque only.
"""
import argparse,json,os,select,signal,struct,time
from pathlib import Path

IDS=(20,21,22,23,24,30,31,32,33,34,10,11,12,13,14)
BROWSER_TIMEOUT=2.0
CONTROL_TIMEOUT=0.5
STARTUP_TIMEOUT=10.0

def off_packets():
    result=bytearray()
    for id in IDS:
        raw=b'\xff\xff\xfd\0'+bytes([id])+struct.pack('<H',6)+b'\x03\x40\0\0'
        crc=0
        for byte in raw:
            crc^=byte<<8
            for _ in range(8):crc=((crc<<1)^(0x8005 if crc&0x8000 else 0))&65535
        result.extend(raw+struct.pack('<H',crc))
    return bytes(result)

class Deadlines:
    def __init__(self,now):
        self.browser=self.progress=now;self.powered=False;self.disarmed=False
    def update(self,byte,now):
        if byte==ord('B'):self.browser=now
        elif byte==ord('P'):self.progress=now
        elif byte==ord('T'):self.powered=True;self.disarmed=False;self.progress=now
        elif byte==ord('D'):self.powered=False;self.disarmed=True
        else:raise ValueError('Unknown guardian message')
    def expired(self,now):
        if now-self.browser>BROWSER_TIMEOUT:return 'browser_lease_expired'
        if not self.disarmed and now-self.progress>(CONTROL_TIMEOUT if self.powered else STARTUP_TIMEOUT):
            return 'control_progress_expired'
        return None

def run(serial_fd,beat_fd,ack_fd,result_path):
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,signal.SIG_IGN)
    clock=Deadlines(time.monotonic());buffer=bytearray();pidfd=None;reason=None
    try:
        while reason is None:
            ready=select.select([beat_fd],[],[],.02)[0]
            if ready:
                data=os.read(beat_fd,4096)
                if not data:reason='supervisor_pipe_closed';break
                buffer.extend(data)
                while buffer:
                    byte=buffer[0]
                    if byte==ord('A'):
                        if len(buffer)<5:break
                        if pidfd is not None:raise RuntimeError('Producer already registered')
                        pid=struct.unpack_from('<I',buffer,1)[0];del buffer[:5]
                        # Parent does not reap the producer until this ack: its PID
                        # cannot be reused while we acquire the stable pidfd handle.
                        pidfd=os.pidfd_open(pid);os.write(ack_fd,b'1');continue
                    del buffer[0]
                    if byte==ord('X'):return
                    if byte==ord('S'):reason='stop_requested';break
                    clock.update(byte,time.monotonic())
            reason=reason or clock.expired(time.monotonic())
    except BaseException as error:reason='guardian_error: '+str(error)
    finally:
        if reason is not None:
            if pidfd is not None:
                try:signal.pidfd_send_signal(pidfd,signal.SIGKILL)
                except ProcessLookupError:pass
            # Allow an interrupted packet to time out, then append only OFF packets.
            time.sleep(.01);packets=off_packets();sent=0;end=time.monotonic()+.5
            while sent<len(packets) and time.monotonic()<end:
                if not select.select([],[serial_fd],[],.02)[1]:continue
                try:sent+=os.write(serial_fd,packets[sent:])
                except BlockingIOError:pass
            Path(result_path).write_text(json.dumps({'reason':reason,'off_bytes_sent':sent,'expected_off_bytes':len(packets)})+'\n')
        if pidfd is not None:os.close(pidfd)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--serial-fd',type=int,required=True);p.add_argument('--beat-fd',type=int,required=True);p.add_argument('--ack-fd',type=int,required=True);p.add_argument('--result',required=True)
    a=p.parse_args();run(a.serial_fd,a.beat_fd,a.ack_fd,a.result)
