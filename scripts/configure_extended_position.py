#!/usr/bin/env python3
"""Set 15 already-calibrated, torque-OFF XL330s to Mode 4; optionally probe signed goals.

Uses the installed servo_config transport (--protocol-dir); never enables torque.
Both robotd and microduck-twin must be stopped. --apply performs writes; otherwise
only read the bus and print the plan. Logs reuse the transport's durable journal.
"""
import argparse
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import time
from export_joint_zero import JOINTS

IDS=(20,21,22,23,24,30,31,32,33,34,10,11,12,13,14)
RESTORE=((76,2),(78,2),(80,2),(82,2),(84,2),(88,2),(90,2),(100,2),(102,2),(108,4),(112,4))

def packet(protocol,id,address,data):
    if id not in IDS:raise ValueError('Only configured motor IDs can be written')
    allowed=(address==11 and data in (b'\x03',b'\x04')) or (address,len(data)) in RESTORE or (address==116 and len(data)==4 and -1048575<=int.from_bytes(data,'little',signed=True)<=1048575)
    if not allowed:raise ValueError('Write outside mode/profile/goal operation')
    body=protocol.stuff(b'\x03'+struct.pack('<H',address)+data)
    raw=protocol.HEADER+bytes([id])+struct.pack('<H',len(body)+2)+body
    return raw+struct.pack('<H',protocol.crc16(raw))


def stopped():
    for unit in ('robotd','microduck-twin'):
        state=subprocess.check_output(['systemctl','show',unit,'--property=ActiveState','--value'],text=True).strip()
        if state not in ('inactive','failed'):raise RuntimeError(f'{unit} must remain stopped')

def snapshot(bus,id):
    data=bus.read(id,0,147)
    if int.from_bytes(data[0:2],'little')!=1200 or data[7]!=id or data[8]!=3 or data[10]!=0 or data[11] not in (3,4) or data[12]!=255 or data[13]!=2 or int.from_bytes(data[20:24],'little',signed=True)!=0:
        raise RuntimeError(f'ID {id}: unexpected hardware configuration')
    if data[64]!=0 or data[70]!=0:raise RuntimeError(f'ID {id}: torque must be OFF and errors clear')
    return data

def goals(entry,present):
    low,high=entry['limits_rad'];zero=entry['zero_tick'];r=2*math.pi/4096
    turns=round(((present-zero)*r-(low+high)/2)/(2*math.pi))
    origin=zero+turns*4096
    q=(present-origin)*r
    if not low-r<=q<=high+r:raise ValueError('Current joint pose does not fit model limits')
    result=[]
    for value in (low,high):
        raw=origin+value/r;nearest=round(raw)
        if abs(raw-nearest)<1e-9:raw=nearest
        result.append(math.trunc(raw))
    return result

def execute(protocol,wire,bus,entries,journal,apply,restore_original=False,probe_goals=False,service_check=None,progress=None,ram_settle=.03):
    before={id:snapshot(bus,id) for id in IDS}
    journal.record('extended_preflight',snapshot={id:v.hex() for id,v in before.items()},apply=apply)
    if not apply:
        print(json.dumps({'mode':4,'ids':IDS,'torque':'all off','writes':False}));return
    def write(id,address,data,verify=True):
        (service_check or stopped)();wire.assert_free()
        if bus.read(id,64,1)!=b'\x00':raise RuntimeError('Torque changed; stop immediately')
        journal.record('extended_write_intent',id=id,address=address,data=data.hex())
        frames=wire.exchange(packet(protocol,id,address,data),.12)
        if any(f.device!=id or f.body!=b'\x55\x00' for f in frames) or len(frames)>1:raise RuntimeError('Unexpected WRITE reply; no resend')
        time.sleep(.03 if address<64 else ram_settle)
        actual=bus.read(id,address,len(data))
        journal.record('extended_write_readback',id=id,address=address,actual=actual.hex(),matches=actual==data)
        if verify and actual!=data:raise RuntimeError(f'ID {id} register {address} did not retain written value; no resend')
    def restore_tuning(id,prior):
        # Read the contiguous tuning block once; retain per-write readback checks.
        current=bus.read(id,76,40)
        for address,length in RESTORE:
            value=prior[address:address+length]
            if current[address-76:address-76+length]!=value:write(id,address,value)
    def hold_present(id):
        # Firmware 53 was observed to track present position in Goal Position while
        # torque is OFF, so a current-pose hold need not read back one identical tick.
        block=bus.read(id,116,20)
        goal=int.from_bytes(block[:4],'little',signed=True)
        present=int.from_bytes(block[16:20],'little',signed=True)
        if abs(goal-present)>2:
            write(id,116,bus.read(id,132,4),verify=False)
            block=bus.read(id,116,20)
            goal=int.from_bytes(block[:4],'little',signed=True)
            present=int.from_bytes(block[16:20],'little',signed=True)
        if abs(goal-present)>2:raise RuntimeError('Could not confirm a current-pose hold goal')
        journal.record('extended_hold_confirmed',id=id,goal=goal,present=present)
    for id in IDS:
        prior=before[id];probes=[]
        try:
            if prior[11]!=4:write(id,11,b'\x04')
            after=snapshot(bus,id)
            expected=bytearray(prior[:64]);expected[11]=4
            if after[:64]!=bytes(expected):raise RuntimeError('EEPROM changed outside Operating Mode')
            restore_tuning(id,prior)
            if probe_goals:
                probes=goals(entries[id],int.from_bytes(after[132:136],'little',signed=True))
                for value in probes:write(id,116,value.to_bytes(4,'little',signed=True))
        finally:
            # Reconcile the actual mode even if a goal readback failed. Restoration is
            # based on the captured register, with torque-OFF and service checks intact.
            if restore_original and bus.read(id,11,1)!=prior[11:12]:write(id,11,prior[11:12])
            restore_tuning(id,prior)
            hold_present(id)
            final=snapshot(bus,id)
            expected=bytearray(prior[:64]);expected[11]=prior[11] if restore_original else 4
            if final[:64]!=bytes(expected) or any(final[a:a+n]!=prior[a:a+n] for a,n in RESTORE):raise RuntimeError('Mode/tuning reconciliation failed')
            journal.record('extended_final_state',id=id,final=final.hex(),restored_original=restore_original)
        journal.record('extended_motor_complete',id=id,goals=probes,goal_probes=probe_goals)
        if progress:progress(id,final,IDS.index(id)+1,len(IDS))
        print(f'ID {id}: Mode 4 verified; final mode {final[11]}, torque OFF, tuning and hold confirmed',flush=True)
    journal.record('extended_complete',ids=IDS,torque='all off',torque_ever_enabled=False,goal_probes=probe_goals)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('calibration',type=Path);p.add_argument('--port',default='/dev/serial0')
    p.add_argument('--protocol-dir',type=Path,default=Path('/root/calibration'))
    p.add_argument('--apply',action='store_true')
    p.add_argument('--restore-original-mode',action='store_true',help='Validate Mode 4 then restore each original mode and tuning')
    p.add_argument('--probe-goals',action='store_true',help='Optional strict goal-register probe; aborts/restores if torque-off firmware does not retain goals')
    a=p.parse_args()
    data=json.loads(a.calibration.read_text());entries={e['id']:e for e in data['joints']}
    if data.get('position_mode')!='extended_position' or len(data['joints'])!=15 or set(entries)!=set(IDS):raise ValueError('A complete extended-position calibration is required')
    for id,entry in entries.items():
        if type(id) is not int or entry['name']!=JOINTS[id]:raise ValueError('Joint ID/name mismatch')
        lo,hi=entry['limits_rad'];zero=entry['zero_tick']
        if any(type(q) not in (float,int) for q in (lo,hi)):raise ValueError('Numeric joint limits required')
        if type(zero) not in (int,float) or not math.isfinite(zero) or not 0<=zero<4096 or not all(math.isfinite(q) for q in (lo,hi)) or not -math.pi<=lo<hi<=math.pi or hi-lo>=2*math.pi-4*math.pi/4096:
            raise ValueError('Invalid zero or ambiguous limits')
    sys.path.insert(0,str(a.protocol_dir));import servo_config as protocol
    stopped();journal=protocol.Journal();print(f'Journal: {journal.path}',flush=True)
    try:
        with protocol.LinuxPort(a.port) as wire:
            wire.open_serial();wire.set_baud(1000000)
            execute(protocol,wire,protocol.ServoBus(wire),entries,journal,a.apply,a.restore_original_mode,a.probe_goals)
    except BaseException as e:journal.record('extended_stopped',reason=str(e));raise
    finally:journal.close()
if __name__=='__main__':main()
