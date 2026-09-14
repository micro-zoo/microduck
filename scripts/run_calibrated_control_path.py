#!/usr/bin/env python3
"""Run the virtual-bus check in an unprivileged systemd sandbox, never directly.

Requires root only to create the isolated transient unit. The test, daemon and CLI
run as nobody with private devices, no capabilities, no privilege escalation and
inaccessible host-management sockets. Production services are not replaced.
"""
import argparse
import json
import os
from pathlib import Path
import pwd
import shutil
import subprocess
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[1]

def sandbox_command(unit,bundle,output,uid,gid,has_calibration):
    runtime=f'/run/{unit}';source=runtime+'/source';destination=runtime+'/output'
    properties=[f'User={uid}',f'Group={gid}',f'RuntimeDirectory={unit}',
        'NoNewPrivileges=yes','PrivateDevices=yes','PrivateTmp=yes',
        'ProtectSystem=strict','ProtectHome=yes','ProtectKernelTunables=yes',
        'ProtectKernelModules=yes','ProtectControlGroups=yes',
        'CapabilityBoundingSet=','RestrictAddressFamilies=AF_UNIX',
        'SystemCallFilter=~@reboot @mount','SystemCallErrorNumber=EPERM',
        'InaccessiblePaths=-/run/systemd -/run/dbus -/etc/robot -/opt/robot',
        f'BindReadOnlyPaths={bundle}:{source}',f'BindPaths={output}:{destination}']
    cmd=['systemd-run','--quiet','--wait','--collect','--pipe',f'--unit={unit}']
    cmd += ['--property='+value for value in properties]
    cmd += ['--setenv=DUCK_CONTROL_PATH_SANDBOX=1','/usr/bin/python3',source+'/scripts/check_calibrated_control_path.py',
        '--robotd',source+'/bin/robotd','--robotctl',source+'/bin/robotctl','--output',destination]
    if has_calibration:cmd+=['--calibration',source+'/calibration.json']
    return cmd

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--robotd',type=Path,required=True);p.add_argument('--robotctl',type=Path,required=True)
    p.add_argument('--calibration',type=Path);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if sys.platform!='linux' or os.geteuid()!=0:raise SystemExit('Use sudo on Linux; the transient test unit itself runs unprivileged.')
    account=pwd.getpwnam('nobody');uid,gid=account.pw_uid,account.pw_gid
    if uid==0:raise RuntimeError('Test user must not be root')
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'result.json').write_text('{"complete":false}\n')
    # /run is mounted noexec on the robot. Bind executable source from /var/lib;
    # the writable output still belongs on /run and is never executed.
    with tempfile.TemporaryDirectory(prefix='cal-path-source-',dir='/var/lib') as src, tempfile.TemporaryDirectory(prefix='cal-path-output-',dir='/run') as dst:
        bundle=Path(src);output=Path(dst)
        files={
            'bin/robotd':args.robotd,'bin/robotctl':args.robotctl,
            'scripts/check_calibrated_control_path.py':ROOT/'scripts/check_calibrated_control_path.py',
            'scripts/export_joint_zero.py':ROOT/'scripts/export_joint_zero.py',
            'scripts/run_guarded_home.py':ROOT/'scripts/run_guarded_home.py',
            'scripts/configure_extended_position.py':ROOT/'scripts/configure_extended_position.py',
            'scripts/probe_ankle_position.py':ROOT/'scripts/probe_ankle_position.py',
            'duck-control/src/model.rs':ROOT/'duck-control/src/model.rs',
            'kinematics/assets/alpha/robot_walk.xml':ROOT/'kinematics/assets/alpha/robot_walk.xml',
        }
        if args.calibration:files['calibration.json']=args.calibration
        for name,path in files.items():
            target=bundle/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,target)
        for path in [bundle,*bundle.rglob('*')]:
            os.chown(path,0,gid);path.chmod(0o750 if path.is_dir() or path.parent.name=='bin' else 0o640)
        os.chown(output,uid,gid);output.chmod(0o700)
        unit=f'calibrated-control-test-{os.getpid()}'
        cmd=sandbox_command(unit,bundle,output,uid,gid,args.calibration is not None)
        outcome=subprocess.run(cmd)
        if any(path.is_symlink() for path in output.rglob('*')):raise RuntimeError('Unexpected symlink in sandbox output')
        shutil.copytree(output,args.output,dirs_exist_ok=True)
        if outcome.returncode:raise SystemExit(outcome.returncode)
        result=json.loads((args.output/'result.json').read_text())
        if not result.get('complete'):raise RuntimeError('Sandbox did not produce a completed result')
        print(f'Sandboxed control-path check completed: {args.output}')
if __name__=='__main__':main()
