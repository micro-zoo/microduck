#!/usr/bin/env python3
"""Run robotd's supported HOME probe with an independent UART torque-off child.

Both normal bus consumers must already be stopped. This invokes the real calibrated
robotd init path, never a policy. Success and failure both end torque OFF.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys

import configure_extended_position as modes
from probe_ankle_position import Cutoff


def restore_packet(protocol, id, address, data, original):
    if id not in modes.IDS or original[address:address+len(data)] != data:
        raise RuntimeError('Restore must use the captured register value')
    if (address, len(data)) not in modes.RESTORE and not (address == 98 and data == b'\0'):
        raise RuntimeError('Restore outside the RAM tuning allowlist')
    body = protocol.stuff(b'\3' + struct.pack('<H', address) + data)
    raw = protocol.HEADER + bytes([id]) + struct.pack('<H', len(body)+2) + body
    return raw + struct.pack('<H', protocol.crc16(raw))


def reconcile(protocol, port, before):
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
            for address, length in ((98, 1), *modes.RESTORE):
                wanted = before[id][address:address+length]
                if bus.read(id, address, length) != wanted:
                    wire.exchange(restore_packet(protocol, id, address, wanted, before[id]), .01)
                    if bus.read(id, address, length) != wanted:
                        raise RuntimeError(f'ID {id} register {address} restoration failed')
        final = {id: modes.snapshot(bus, id) for id in modes.IDS}
        if any(final[id][:64] != before[id][:64] for id in modes.IDS):
            raise RuntimeError('Unexpected EEPROM change; all motors are OFF')
        return {id: data.hex() for id, data in final.items()}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--robotd', type=Path, required=True)
    p.add_argument('--params', type=Path, required=True)
    p.add_argument('--port', default='/dev/serial0')
    p.add_argument('--protocol-dir', type=Path, default=Path('/root/calibration'))
    p.add_argument('--duration', type=int, default=10, choices=range(5,31))
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
    fd = os.open(a.port, os.O_WRONLY | os.O_NOCTTY | os.O_NONBLOCK)
    guard = Cutoff(fd, off, a.duration+12, str(a.output/'cutoff.json'))
    process = None
    result = {'complete':False, 'reached':False, 'all_off':False, 'settings_restored':False}
    try:
        env = {**os.environ, 'DUCK_HOME_WATCHDOG_FD':str(guard.fd),
            'DUCK_RUNTIME_DIR':str(a.output.resolve()/'run')}
        command = [str(a.robotd.resolve()), '--params', str(a.params.resolve()), '--port', a.port,
            '--socket', str(a.output.resolve()/'init.sock'), 'init', '--guarded',
            '--duration', f'{a.duration}s', '--telemetry', str(a.output.resolve()/'telemetry.jsonl')]
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
        if guard.fd is not None:
            os.close(guard.fd); guard.fd=None
            os.waitpid(guard.pid,0)
            result['independent_cutoff_fired'] = True
        os.close(fd)
        try:
            final = reconcile(protocol, a.port, before)
            (a.output/'after.json').write_text(json.dumps(final,indent=2)+'\n')
            result.update(all_off=True, settings_restored=True)
        except BaseException as e:
            result['restoration_error']=str(e)
        result['complete'] = result['reached'] and result['all_off'] and result['settings_restored'] and not result.get('independent_cutoff_fired',False)
        (a.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2),flush=True)
    if not result['complete']: raise SystemExit(1)


if __name__ == '__main__': main()
