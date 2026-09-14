#!/usr/bin/env python3
"""Supported, one-sided ankle test: toe DOWN only, 1 degree then 3 degrees total.

No home/return motion. Only the selected ankle is powered. The other 14 must be
OFF. Current is input current, not measured joint torque. Logs contain every
powered telemetry sample. Uses the installed servo_config.py UART transport.
"""
import argparse
from collections import deque
import json
import math
import os
import select
from pathlib import Path
import signal
import struct
import sys
import time
import configure_extended_position as modes

PWM_CAP=60                 # 6.78% output ceiling in Mode 4
CURRENT_CAP_MA=100
WATCHDOG=15                # 300 ms without packets -> firmware stop
PERIOD=.020                # aim for 50 Hz; measured interval is recorded
MAX_GAP=.090
MAX_SECONDS=6.0
DEG_PER_TICK=360/4096
MAX_COUNTS=round(3/DEG_PER_TICK)
SAVED={11:1,98:1,**dict(modes.RESTORE)}


def check_sample(sample,start,sign,target,initial_temp,history):
    if sample['torque_enable']!=1 or sample['hardware_error'] or sample['status_error'] or sample['watchdog']!=WATCHDOG:
        raise RuntimeError('Torque/watchdog/hardware status changed')
    if abs(sample['current_ma'])>CURRENT_CAP_MA:raise RuntimeError('Input current limit exceeded')
    if abs(sample['pwm_raw'])>PWM_CAP+3:raise RuntimeError('PWM exceeded test output ceiling')
    if not 4.5<=sample['voltage_v']<=5.5:raise RuntimeError('Supply voltage outside test range')
    if sample['temperature_c']>=40 or sample['temperature_c']-initial_temp>=3:raise RuntimeError('Temperature limit reached')
    progress=sign*(sample['position_tick']-start)
    if progress < -3:raise RuntimeError('Motion toward the fixture stop detected')
    if progress > MAX_COUNTS+3:raise RuntimeError('Excursion exceeded 3-degree test envelope')
    if abs(sample['velocity_raw'])>6:raise RuntimeError('Unexpected angular speed')
    error=abs(sample['position_trajectory_tick']-sample['position_tick'])
    if error>16:raise RuntimeError('Position tracking error exceeded 1.4 degrees')
    # A low-current jam is still a jam: do not wait for heating or a large current.
    if history and sample['elapsed_s']-history[0]['elapsed_s']>=.45 and error>=8:
        travel=sign*(sample['position_tick']-history[0]['position_tick'])
        persistent=all(abs(s['position_trajectory_tick']-s['position_tick'])>=8 for s in history)
        if persistent and travel<2:raise RuntimeError('No progress under sustained trajectory error; possible fixture contact')


class Cutoff:
    """Independent bounded torque-off, including parent death or a blocked fsync.

    The device bus watchdog stops motion but is not an independently confirmed
    torque-off. This child owns only one already-validated torque-OFF packet.
    """
    def __init__(self,serial_fd,off_packet,seconds,log_path):
        read_fd,write_fd=os.pipe()
        log_fd=os.open(log_path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
        self.pid=os.fork()
        if self.pid==0:
            os.close(write_fd)
            os.setsid()
            for sig in (signal.SIGHUP,signal.SIGINT,signal.SIGTERM):signal.signal(sig,signal.SIG_IGN)
            cause='deadline'
            try:
                ready=select.select([read_fd],[],[],seconds)[0]
                if ready:
                    if os.read(read_fd,1)==b'x':os._exit(0)
                    cause='parent_pipe_closed'
                sent=0;deadline=time.monotonic()+.5
                while sent<len(off_packet) and time.monotonic()<deadline:
                    if not select.select([],[serial_fd],[],.02)[1]:continue
                    try:sent+=os.write(serial_fd,off_packet[sent:])
                    except BlockingIOError:pass
                record=json.dumps({'event':'independent_torque_off','cause':cause,'bytes_sent':sent}).encode()
                os.write(log_fd,record+b'\n');os.fsync(log_fd)
                os._exit(2 if sent==len(off_packet) else 3)
            except BaseException:os._exit(4)
        os.close(read_fd);os.close(log_fd);self.fd=write_fd
    def cancel(self):
        if self.fd is None:return 0
        try:os.write(self.fd,b'x')
        except BrokenPipeError:pass
        os.close(self.fd);self.fd=None
        _,status=os.waitpid(self.pid,0)
        return os.waitstatus_to_exitcode(status)


class Probe:
    def __init__(self,protocol,wire,bus,journal,id,p_gain=400):
        self.p,self.wire,self.bus,self.journal,self.id=protocol,wire,bus,journal,id
        if p_gain not in (400,800):raise ValueError('Probe P gain is limited to 400 or 800')
        self.p_gain=p_gain
        self.sign=-1 if id==14 else 1 # model ankle axes: right -Y, left +Y; toe-down rotation is +Y
        self.before=modes.snapshot(bus,id)
        self.saved={a:self.before[a:a+n] for a,n in SAVED.items()}
        if self.before[11]!=3 or self.before[98]!=0 or self.before[63]&0x24!=0x24:
            raise RuntimeError('Require Mode 3, watchdog 0 and overheating/overload shutdown enabled')
        if int.from_bytes(self.before[36:38],'little')<PWM_CAP:raise RuntimeError('Existing PWM limit is below probe ceiling')
        self.initial_temp=self.before[146]
        if self.initial_temp>35:raise RuntimeError('Allow ankle to cool before testing')
        self.start=None;self.target=None;self.armed=False;self.driving=False
        self.started=time.monotonic();self.last_sample=None;self.samples=[];self.history=deque();self.guard=None;self.guard_triggered=False
        self.journal.record('ankle_original',id=id,direction='toe_down',sign=self.sign,registers={a:v.hex() for a,v in self.saved.items()},snapshot=self.before.hex(),limits={'pwm_raw':PWM_CAP,'p_gain':self.p_gain,'input_current_ma':CURRENT_CAP_MA,'max_degrees':3,'max_seconds':MAX_SECONDS})
    def packet(self,address,data,restoring=False):
        allow={11:(b'\x03',b'\x04'),64:(b'\x00',b'\x01'),98:(b'\x00',bytes([WATCHDOG])),100:(bytes(2),PWM_CAP.to_bytes(2,'little')),80:(bytes(2),),82:(bytes(2),),84:(self.p_gain.to_bytes(2,'little'),),88:(bytes(2),),90:(bytes(2),),108:((1).to_bytes(4,'little'),),112:((2).to_bytes(4,'little'),)}
        permitted=data in allow.get(address,())
        if address==116 and self.start is not None and len(data)==4:
            value=int.from_bytes(data,'little',signed=True)
            progress=self.sign*(value-self.start)
            permitted=0<=progress<=MAX_COUNTS
        if restoring and self.saved.get(address)==data:permitted=True
        if not permitted or self.id not in (14,24):raise ValueError('Write outside selected ankle test envelope')
        body=self.p.stuff(b'\x03'+struct.pack('<H',address)+data)
        raw=self.p.HEADER+bytes([self.id])+struct.pack('<H',len(body)+2)+body
        return raw+struct.pack('<H',self.p.crc16(raw))
    def read(self,address,length,timeout=.008):
        frames=self.wire.exchange(self.p.instruction_packet(self.id,2,struct.pack('<HH',address,length)),timeout)
        if len(frames)!=1 or frames[0].device!=self.id or len(frames[0].body)!=length+2 or frames[0].body[0]!=0x55 or frames[0].body[1]&0x7f:
            raise RuntimeError('Telemetry read failed; stop rather than continuing blind')
        return frames[0].body[2:],frames[0].body[1]
    def send(self,address,data,verify=True,restoring=False):
        frames=self.wire.exchange(self.packet(address,data,restoring),.008)
        if len(frames)>1 or any(f.device!=self.id or len(f.body)!=2 or f.body[0]!=0x55 or f.body[1]&0x7f for f in frames):raise RuntimeError('Write rejected; no blind resend')
        if verify and self.read(address,len(data))[0]!=data:raise RuntimeError(f'Register {address} readback mismatch')
    def write(self,address,value,length=1,verify=True):
        data=value.to_bytes(length,'little',signed=value<0)
        self.journal.record('ankle_write_intent',id=self.id,address=address,value=value)
        self.send(address,data,verify)
        if self.armed:self.sample()
    def sample(self):
        data,error=self.read(64,83)
        now=time.monotonic();gap=None if self.last_sample is None else now-self.last_sample;self.last_sample=now
        sample={'elapsed_s':now-self.started,'gap_s':gap,'torque_enable':data[0],'hardware_error':data[6],'watchdog':data[34],
                'goal_tick':struct.unpack_from('<i',data,52)[0],'pwm_raw':struct.unpack_from('<h',data,60)[0],
                'current_ma':struct.unpack_from('<h',data,62)[0],'velocity_raw':struct.unpack_from('<i',data,64)[0],
                'position_tick':struct.unpack_from('<i',data,68)[0],
                'velocity_trajectory_raw':struct.unpack_from('<i',data,72)[0],
                'position_trajectory_tick':struct.unpack_from('<i',data,76)[0],'voltage_v':struct.unpack_from('<H',data,80)[0]/10,
                'temperature_c':data[82],'status_error':error,'command_tick':self.target,'joint_torque_nm':None}
        self.samples.append(sample)
        if self.armed:
            # Emergency shutdown must not wait for a successful log write.
            try:
                if gap is not None and gap>MAX_GAP:raise RuntimeError('Telemetry gap exceeded 90 ms')
                if self.driving and now-self.drive_started>MAX_SECONDS:raise RuntimeError('Powered test timed out')
                check_sample(sample,self.start,self.sign,self.target,self.initial_temp,self.history)
            except BaseException:
                self.emergency_off()
                self.journal.record('ankle_sample',id=self.id,**sample)
                raise
        self.journal.record('ankle_sample',id=self.id,**sample)
        self.history.append(sample)
        while self.history and now-self.started-self.history[0]['elapsed_s']>.55:self.history.popleft()
        return sample
    def emergency_off(self):
        # No journal, service process or goal write may delay this idempotent operation.
        errors=[]
        for _ in range(3):
            try:
                self.send(64,b'\x00',verify=False)
                if self.read(64,1,timeout=.03)[0]==b'\x00':
                    self.armed=False
                    if self.guard is not None:
                        self.guard_triggered=self.guard.cancel()!=0 or self.guard_triggered
                    return
            except BaseException as e:errors.append(str(e))
        raise RuntimeError('Torque OFF could not be confirmed; disconnect servo power: '+str(errors))
    def restore(self):
        self.emergency_off()
        self.send(98,b'\x00')
        self.send(11,self.saved[11],restoring=True)
        for a,data in self.saved.items():
            if a not in (11,98) and self.read(a,len(data))[0]!=data:self.send(a,data,restoring=True)
        self.send(98,self.saved[98],restoring=True)
        final=modes.snapshot(self.bus,self.id)
        if final[:64]!=self.before[:64] or any(final[a:a+len(v)]!=v for a,v in self.saved.items()):raise RuntimeError('Original mode/tuning not fully restored')
        self.sample()
        self.journal.record('ankle_restored',id=self.id,snapshot=final.hex(),torque_enable=0,independent_cutoff_fired=self.guard_triggered)
        if self.guard_triggered:raise RuntimeError('Independent torque-off protection fired; test not complete')
    def configure_pid(self):
        # XL330 control table order is D(80), I(82), P(84), not P/I/D.
        for address,value in ((80,0),(82,0),(84,self.p_gain),(88,0),(90,0)):
            self.write(address,value,2)
        if struct.unpack('<HHH',self.read(80,6)[0])!=(0,0,self.p_gain):
            raise RuntimeError('D/I/P readback does not match the verified control table')

    def run(self):
        try:
            self.sample()
            self.write(11,4)
            self.write(100,0,2) # torque may be enabled only with zero output first
            self.configure_pid()
            self.write(108,1,4);self.write(112,2,4)
            self.write(98,WATCHDOG)
            self.start=int.from_bytes(self.read(132,4)[0],'little',signed=True);self.target=self.start
            self.write(116,self.start,4,verify=False)
            self.guard=Cutoff(self.wire.fd,self.packet(64,b'\x00'),8.0,str(self.journal.path)+'.cutoff.json')
            self.write(64,1);self.armed=True;self.last_sample=None
            first=self.sample() # zero PWM: establish the actual hold position before output
            self.start=first['position_tick'];self.target=self.start;self.history.clear()
            self.write(116,self.start,4,verify=False)
            self.drive_started=time.monotonic();self.driving=True
            self.write(100,PWM_CAP,2)
            last_status=0
            for degrees in (1,3):
                endpoint=self.start+self.sign*round(degrees/DEG_PER_TICK)
                settled=0
                self.target=endpoint;self.history.clear()
                self.write(116,endpoint,4)
                while settled<5:
                    tick=time.monotonic()
                    sample=self.sample()
                    error=abs(sample['position_tick']-endpoint)
                    settled=settled+1 if error<=3 and abs(sample['velocity_raw'])<=1 else 0
                    if tick-last_status>.5:
                        print(f"ID {self.id}: down {self.sign*(sample['position_tick']-self.start)*DEG_PER_TICK:.2f} deg, I={sample['current_ma']} mA, PWM={sample['pwm_raw']}, T={sample['temperature_c']} C",flush=True);last_status=tick
                    time.sleep(max(0,PERIOD-(time.monotonic()-tick)))
                self.journal.record('ankle_stage_complete',id=self.id,degrees=degrees,position_tick=sample['position_tick'],goal_tick=endpoint)
                print(f'ID {self.id}: {degrees}-degree stage passed',flush=True)

            self.journal.record('ankle_motion_complete',id=self.id,start_tick=self.start,end_tick=sample['position_tick'])
        finally:self.restore()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--id',type=int,choices=(14,24),default=14)
    p.add_argument('--p-gain',type=int,choices=(400,800),default=400)
    p.add_argument('--protocol-dir',type=Path,default=Path('/root/calibration'));p.add_argument('--port',default='/dev/serial0');p.add_argument('--apply',action='store_true')
    a=p.parse_args();sys.path.insert(0,str(a.protocol_dir));import servo_config as protocol
    modes.stopped()
    for number in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
        signal.signal(number,lambda sig,frame:(_ for _ in ()).throw(InterruptedError(f'signal {sig}')))
    journal=protocol.Journal();print(f'Journal: {journal.path}',flush=True)
    try:
        with protocol.LinuxPort(a.port) as wire:
            wire.open_serial();wire.set_baud(1000000);bus=protocol.ServoBus(wire)
            before={id:modes.snapshot(bus,id) for id in modes.IDS}
            probe=Probe(protocol,wire,bus,journal,a.id,a.p_gain)
            if a.apply:probe.run()
            else:print(f'Read-only preflight: ID {a.id}, toe down, 1 then 3 degrees, all motors OFF')
            for id in modes.IDS:
                final=modes.snapshot(bus,id)
                if final[64]!=0:raise RuntimeError('A motor remains powered')
                if id!=a.id and final[:64]!=before[id][:64]:raise RuntimeError('An unrelated motor changed')
            journal.record('ankle_final_all_off',ids=modes.IDS)
    except BaseException as e:
        journal.record('ankle_failed',error=str(e));raise
    finally:journal.close()
if __name__=='__main__':main()
