#!/usr/bin/env python3
"""Run robotd's supported HOME probe with an independent UART torque-off child.

Both normal bus consumers must already be stopped. This invokes the real calibrated
robotd init path, never a policy. Success and failure both end torque OFF.
"""
import argparse
import json
import copy
import math
import os
from pathlib import Path
import signal
import re
import struct
import subprocess
import sys

import configure_extended_position as modes
from probe_ankle_position import Cutoff


def recovery_calibration(document, before, model_source, duration,pose='home'):
    """Select encoder turns for this HOME-only transaction, including a slumped pose.

    The daemon still commands only its fixed HOME segment, at bounded speed/current.
    Normal calibration/model limits are not edited or installed by this function.
    """
    from export_joint_zero import JOINTS
    source = Path(model_source).read_text()
    def array(name, convert):
        body = re.search(r'pub const '+name+r':.*?= \[(.*?)\];',source,re.S).group(1)
        return [convert(v.strip()) for v in re.sub(r'//[^\n]*','',body).split(',') if v.strip()]
    ids = array('JOINT_IDS',int); home = array('DEFAULT_POSITION',float)
    if ids != list(modes.IDS) or len(home) != 15:
        raise ValueError('HOME source does not match the configured joints')
    if pose=='zero':
        from export_joint_zero import MOUTH_CLOSED
        home=[MOUTH_CLOSED if id==34 else 0. for id in ids]
    elif pose!='home':raise ValueError('Unsupported pose')
    result = copy.deepcopy(document)
    entries = result['joints']
    if result.get('position_mode') != 'extended_position' or len(entries)!=15 or {j['id'] for j in entries}!=set(ids):
        raise ValueError('Require a complete extended calibration')
    changes=[];radians_per_tick=math.tau/4096
    for entry in entries:
        id=entry['id'];zero=entry['zero_tick'];lo,hi=entry['limits_rad'];goal=home[ids.index(id)]
        if entry['name']!=JOINTS[id] or not math.isfinite(zero) or not 0<=zero<4096 or not -math.pi<=lo<hi<=math.pi:
            raise ValueError('Invalid joint zero or model interval')
        if not lo<=goal<=hi:
            raise ValueError(f'ID {id}: HOME target is outside the normal model interval')
        if before[id][64]!=0 or before[id][70]!=0:
            raise ValueError('Recovery preparation requires all motors OFF without errors')
        raw=struct.unpack_from('<i',before[id],132)[0]
        delta=((raw-zero)*radians_per_tick-goal+math.pi)%math.tau-math.pi
        start=goal+delta
        travel_limit=math.radians(175 if id==30 else 90)
        if abs(delta)>travel_limit or abs(delta)/duration>math.radians(6):
            raise ValueError(f'ID {id}: HOME path exceeds the diagnostic distance or speed')
        if not -math.pi<=start<=math.pi:
            raise ValueError(f'ID {id}: HOME recovery would cross the model angular branch')
        if id!=30 and (start<lo-math.radians(5) or start>hi+math.radians(5)):
            raise ValueError(f'ID {id}: start is more than 5 degrees outside the model; inspect pose/calibration')
        bounds=[max(-math.pi,min(lo,start-math.radians(3))),min(math.pi,max(hi,start+math.radians(3)))]
        if bounds[1]-bounds[0]>=math.tau-2*radians_per_tick:
            raise ValueError('Recovery interval cannot identify one encoder turn')
        if bounds!=[lo,hi]:changes.append({'id':id,'model_limits_rad':[lo,hi],'recovery_limits_rad':bounds,'start_rad':start})
        entry['limits_rad']=bounds
    return result,changes


def restore_packet(protocol, id, address, data, original):
    if id not in modes.IDS or original[address:address+len(data)] != data:
        raise RuntimeError('Restore must use the captured register value')
    if (address, len(data)) not in modes.RESTORE and not (address == 98 and data == b'\0'):
        raise RuntimeError('Restore outside the RAM tuning allowlist')
    body = protocol.stuff(b'\3' + struct.pack('<H', address) + data)
    raw = protocol.HEADER + bytes([id]) + struct.pack('<H', len(body)+2) + body
    return raw + struct.pack('<H', protocol.crc16(raw))


def reconcile(protocol, port, before, restore_modes=False):
    with protocol.LinuxPort(port) as wire:
        wire.open_serial(); wire.set_baud(1000000)
        bus = protocol.ServoBus(wire)
        # Torque-off must reach every ID, even if one motor no longer responds.
        for id in modes.IDS:
            try:
                wire.exchange(protocol.instruction_packet(id, 3, struct.pack('<HB', 64, 0)), .01)
            except Exception:
                pass
        if any(bus.read(id, 64, 1) != b'\0' for id in modes.IDS):
            raise RuntimeError('Cannot confirm all motors OFF; disconnect servo power')
        for id in modes.IDS:
            if restore_modes and bus.read(id,11,1)!=before[id][11:12]:
                wire.exchange(modes.packet(protocol,id,11,before[id][11:12]),.03)
                if bus.read(id,11,1)!=before[id][11:12]:
                    raise RuntimeError(f'ID {id}: original operating mode was not restored')
            current=bus.read(id,76,40)
            for address, length in ((98, 1), *modes.RESTORE):
                wanted = before[id][address:address+length]
                if current[address-76:address-76+length] != wanted:
                    wire.exchange(restore_packet(protocol, id, address, wanted, before[id]), .01)
                    if bus.read(id, address, length) != wanted:
                        raise RuntimeError(f'ID {id} register {address} restoration failed')
        final = {id: modes.snapshot(bus, id) for id in modes.IDS}
        if any(final[id][:64] != before[id][:64] or any(final[id][a:a+n]!=before[id][a:a+n] for a,n in ((98,1),*modes.RESTORE)) for id in modes.IDS):
            raise RuntimeError('Unexpected EEPROM change; all motors are OFF')
        return {id: data.hex() for id, data in final.items()}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--robotd', type=Path, required=True)
    configuration=p.add_mutually_exclusive_group(required=True)
    configuration.add_argument('--params', type=Path)
    configuration.add_argument('--extended-calibration',type=Path,help='Temporarily use Mode 4 for a HOME path across an encoder seam; restore original modes on exit')
    p.add_argument('--model-source',type=Path,default=Path(__file__).resolve().parents[1]/'duck-control/src/model.rs')
    p.add_argument('--port', default='/dev/serial0')
    p.add_argument('--protocol-dir', type=Path, default=Path('/root/calibration'))
    p.add_argument('--duration', type=int, default=10, choices=range(5,31))
    p.add_argument('--higher-effort',action='store_true')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if sys.platform != 'linux' or os.geteuid() != 0:
        raise SystemExit('The hardware HOME launcher requires root on Linux')
    modes.stopped()
    a.output.mkdir(parents=True, exist_ok=False)
    sys.path.insert(0, str(a.protocol_dir))
    import servo_config as protocol
    for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(number, lambda sig, frame: (_ for _ in ()).throw(InterruptedError(f'signal {sig}')))
    with protocol.LinuxPort(a.port) as wire:
        wire.open_serial(); wire.set_baud(1000000)
        before = {id: modes.snapshot(protocol.ServoBus(wire), id) for id in modes.IDS}
    if any(data[98] != 0 for data in before.values()):
        raise RuntimeError('Require watchdog clear before HOME')
    (a.output/'before.json').write_text(json.dumps({id:data.hex() for id,data in before.items()}, indent=2)+'\n')
    off = b''.join(protocol.instruction_packet(id, 3, struct.pack('<HB',64,0)) for id in modes.IDS)
    fd = None; guard = None
    process = None
    result = {'complete':False, 'reached':False, 'all_off':False, 'settings_restored':False}
    try:
        params_path=a.params
        if a.extended_calibration:
            cal,changes=recovery_calibration(json.loads(a.extended_calibration.read_text()),before,a.model_source,a.duration)
            cal_path=a.output.resolve()/'home-only-calibration.json'
            cal_path.write_text(json.dumps(cal,indent=2)+'\n')
            (a.output/'recovery-intervals.json').write_text(json.dumps(changes,indent=2)+'\n')
            params_path=a.output.resolve()/'home-only.toml'
            params_path.write_text(f'[bus]\nport={json.dumps(a.port)}\ncalibration={json.dumps(str(cal_path))}\n[audio]\nenabled=false\n')
            journal=protocol.Journal()
            try:
                with protocol.LinuxPort(a.port) as wire:
                    wire.open_serial();wire.set_baud(1000000)
                    modes.execute(protocol,wire,protocol.ServoBus(wire),{j['id']:j for j in cal['joints']},journal,True)
            finally:journal.close()
        fd=os.open(a.port,os.O_WRONLY | os.O_NOCTTY | os.O_NONBLOCK)
        guard=Cutoff(fd,off,a.duration+12,str(a.output/'cutoff.json'))
        env = {**os.environ, 'DUCK_HOME_WATCHDOG_FD':str(guard.fd),
            'DUCK_RUNTIME_DIR':str(a.output.resolve()/'run')}
        command = [str(a.robotd.resolve()), '--params', str(params_path.resolve()), '--port', a.port,
            '--socket', str(a.output.resolve()/'init.sock'), 'init', '--guarded',
            '--duration', f'{a.duration}s', '--telemetry', str(a.output.resolve()/'telemetry.jsonl')]
        if a.higher_effort:command.append('--higher-effort')
        with (a.output/'robotd.log').open('w') as log:
            process = subprocess.Popen(command, env=env, pass_fds=(guard.fd,), stdout=log, stderr=subprocess.STDOUT)
            result['exit_code'] = process.wait(timeout=a.duration+10)
        events = [json.loads(line) for line in (a.output/'telemetry.jsonl').read_text().splitlines()]
        cleanup = next((e for e in reversed(events) if e.get('event')=='home_cleanup'), {})
        result['reached'] = result['exit_code']==0 and cleanup.get('reached',False)
        if cleanup.get('all_off') and cleanup.get('settings_restored'):
            result['independent_cutoff_fired'] = guard.cancel()!=0
        else:
            # EOF orders an immediate independent OFF packet after an incomplete run.
            os.close(guard.fd); guard.fd=None
            _,status=os.waitpid(guard.pid,0)
            result['independent_cutoff_fired'] = True
            result['cutoff_exit_code'] = os.waitstatus_to_exitcode(status)
    except BaseException as e:
        result['error'] = str(e)
    finally:
        if process is not None and process.poll() is None:
            process.kill(); process.wait()
        if guard is not None and guard.fd is not None:
            os.close(guard.fd); guard.fd=None
            os.waitpid(guard.pid,0)
            result['independent_cutoff_fired'] = True
        if fd is not None:os.close(fd)
        try:
            final = reconcile(protocol, a.port, before,restore_modes=a.extended_calibration is not None)
            (a.output/'after.json').write_text(json.dumps(final,indent=2)+'\n')
            result.update(all_off=True, settings_restored=True)
        except BaseException as e:
            result['restoration_error']=str(e)
        result['complete'] = result['reached'] and result['all_off'] and result['settings_restored'] and not result.get('independent_cutoff_fired',False)
        (a.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2),flush=True)
    if not result['complete']: raise SystemExit(1)


if __name__ == '__main__': main()
