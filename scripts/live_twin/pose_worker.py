#!/usr/bin/env python3
"""Supervised HOME/zero session, launched in a separate systemd unit from the UI."""
import argparse,json,os,select,signal,socket,struct,subprocess,sys,threading,time
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
import configure_extended_position as modes
from run_guarded_home import recovery_calibration,reconcile
from reply_transport import accelerated

def save(path,value):
    path=Path(path);tmp=path.with_suffix(path.suffix+'.tmp')
    with tmp.open('w') as f:json.dump(value,f,indent=2);f.write('\n');f.flush();os.fsync(f.fileno())
    os.replace(tmp,path)

def daemon_stopped():
    result=subprocess.run(['systemctl','show','robotd.service','--property=ActiveState','--value'],capture_output=True,text=True,timeout=3,check=True)
    if result.stdout.strip() not in ('inactive','failed'):raise RuntimeError('robotd must be stopped')

class Commands:
    def __init__(self,path):
        self.socket=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM);self.socket.bind(str(path));os.chmod(path,0o600)
        self.socket.settimeout(.05);self.stop=threading.Event();self.done=threading.Event();self.lock=threading.Lock()
        self.lease=time.monotonic();self.reason=None;self.beat=None;self.control=None
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def pulse(self,byte):
        with self.lock:
            if self.beat is not None:
                try:os.write(self.beat,byte)
                except OSError:self.stop.set();self.reason=self.reason or 'guardian_pipe_failed'
    def cancel(self,reason):
        self.reason=self.reason or reason;self.stop.set();self.pulse(b'S')
    def run(self):
        while not self.done.is_set():
            try:
                value=self.socket.recv(16)
                if value==b'S':self.cancel('user_relax')
                elif value in (b'B',b'H',b'Z'):
                    self.lease=time.monotonic();self.pulse(b'B')
                    if value!=b'B':
                        with self.lock:
                            if self.control is not None:
                                try:os.write(self.control,value)
                                except OSError:self.stop.set();self.reason='control_pipe_failed'
                else:self.cancel('invalid_local_command')
            except socket.timeout:pass
            except OSError:
                if not self.done.is_set():self.cancel('command_socket_failed')
            if time.monotonic()-self.lease>2:self.cancel('browser_disconnected')
    def check(self):
        if self.stop.is_set():raise InterruptedError(self.reason or 'stopped')
        daemon_stopped()
    def close(self):
        self.done.set();self.thread.join(1);self.socket.close()

def transport(directory):
    sys.path.insert(0,str(directory));import servo_config
    return accelerated(servo_config)

def cleanup(run,port,protocol_dir):
    run=Path(run);p=run/'before.json'
    current=run/'result.json'
    if current.exists():
        try:
            state=json.loads(current.read_text())
            if state.get('all_off') and state.get('settings_restored'):return
        except ValueError:pass
    if not p.exists():return relax_only(run,port,protocol_dir)
    protocol=transport(protocol_dir);before={int(k):bytes.fromhex(v) for k,v in json.loads(p.read_text()).items()}
    if set(before)!=set(modes.IDS):raise RuntimeError('Incomplete saved motor settings')
    final=reconcile(protocol,port,before,restore_modes=True)
    save(run/'after.json',final)
    save(current,{'complete':False,'all_off':True,'settings_restored':True,'reason':'recovered_after_worker_exit'})

def relax_only(root,port,protocol_dir):
    protocol=transport(protocol_dir);daemon_stopped()
    with protocol.LinuxPort(port) as wire:
        wire.open_serial();wire.set_baud(1000000);bus=protocol.ServoBus(wire)
        for id in modes.IDS:
            try:wire.exchange(protocol.instruction_packet(id,3,struct.pack('<HB',64,0)),.01)
            except Exception:pass
        if any(bus.read(id,64,1)!=b'\0' for id in modes.IDS):raise RuntimeError('Cannot confirm all motors OFF')
    save(Path(root)/'result.json',{'complete':True,'all_off':True,'settings_restored':True,'reason':'user_relax'})

def run(args):
    if sys.platform!='linux' or os.geteuid()!=0:raise RuntimeError('Hardware pose worker requires root on Linux')
    root=args.run.resolve();root.mkdir(parents=True,exist_ok=True);os.chmod(root,0o700)
    commands=Commands(root/'command.sock');protocol=transport(args.protocol_dir)
    for sig in (signal.SIGTERM,signal.SIGINT,signal.SIGHUP):signal.signal(sig,lambda sig,frame:commands.cancel('worker_stopped'))
    before=None;native=None;guardian=None;registered=False;fds=[];result={'all_off':False,'settings_restored':False,'complete':False}
    try:
        commands.check()
        # The web reader has already released its descriptor. LinuxPort enforces
        # the same kernel exclusivity and owner checks as the maintenance tools.
        with protocol.LinuxPort(args.port) as wire:
            wire.open_serial();wire.set_baud(1000000)
            bus=protocol.ServoBus(wire);before={id:modes.snapshot(bus,id) for id in modes.IDS}
        save(root/'before.json',{id:data.hex() for id,data in before.items()})
        cal,changes=recovery_calibration(json.loads(args.calibration.read_text()),before,args.model_source,30,pose=args.pose)
        save(root/'calibration.json',cal);save(root/'recovery-intervals.json',changes)
        (root/'params.toml').write_text(f'[bus]\nport={json.dumps(args.port)}\ncalibration={json.dumps(str(root/"calibration.json"))}\n[audio]\nenabled=false\n')
        def preparation(id=None,data=None,done=0,total=15):
            save(root/'preparation.json',{'event':'preparation','id':id,'snapshot':data.hex() if data is not None else None,'done':done,'total':total})
        preparation()
        journal=protocol.Journal()
        try:
            with protocol.LinuxPort(args.port) as wire:
                wire.open_serial();wire.set_baud(1000000)
                modes.execute(protocol,wire,protocol.ServoBus(wire),{j['id']:j for j in cal['joints']},journal,True,service_check=commands.check,progress=preparation,ram_settle=0)
        finally:journal.close()
        commands.check()
        serial_fd=os.open(args.port,os.O_WRONLY|os.O_NOCTTY|os.O_NONBLOCK);fds.append(serial_fd)
        br,bw=os.pipe();ar,aw=os.pipe();cr,cw=os.pipe();fds.extend([br,bw,ar,aw,cr,cw])
        os.set_blocking(bw,False);os.set_blocking(cw,False)
        guardian=subprocess.Popen([sys.executable,str(HERE/'guardian.py'),'--serial-fd',str(serial_fd),'--beat-fd',str(br),'--ack-fd',str(aw),'--result',str(root/'guardian.json')],pass_fds=(serial_fd,br,aw))
        with commands.lock:commands.beat=bw
        commands.pulse(b'B')
        env={**os.environ,'DUCK_HOME_WATCHDOG_FD':str(bw),'DUCK_POSE_CONTROL_FD':str(cr),'DUCK_RUNTIME_DIR':str(root/'runtime')}
        command=[str(args.robotd),'--params',str(root/'params.toml'),'--port',args.port,'--socket',str(root/'robotd.sock'),
            'init','--guarded','--interactive','--higher-effort','--pose',args.pose,'--duration','30s','--telemetry',str(root/'telemetry.jsonl')]
        with (root/'robotd.log').open('w') as log:
            native=subprocess.Popen(command,env=env,pass_fds=(bw,cr),stdout=log,stderr=subprocess.STDOUT)
        os.write(bw,b'A'+struct.pack('<I',native.pid))
        # Do not reap native until the guardian has acquired its pidfd identity.
        if not select.select([ar],[],[],1)[0] or os.read(ar,1)!=b'1':raise RuntimeError('Independent guardian did not acknowledge the producer')
        registered=True
        commands.check()
        with commands.lock:commands.control=cw
        os.write(cw,b'G')
        while native.poll() is None:
            if guardian.poll() is not None:
                commands.cancel('guardian_ended');break
            time.sleep(.02)
        if native.poll() is not None:result['controller_exit_code']=native.returncode
        result['reason']=commands.reason or 'controller_finished'
    except BaseException as error:
        result['error']=str(error);result['reason']=commands.reason or 'control_failed'
    finally:
        if guardian is not None and not registered:
            # No G/start byte was sent. Stop the guardian before reaping native so
            # a late PID registration can never refer to a reused process ID.
            if guardian.poll() is None:guardian.kill()
            guardian.wait()
        if native is not None and native.poll() is None:native.kill();native.wait()
        commands.pulse(b'S')
        commands.close()
        if guardian is not None:
            try:guardian.wait(timeout=2)
            except subprocess.TimeoutExpired:guardian.kill();guardian.wait()
        with commands.lock:commands.control=None;commands.beat=None
        for fd in fds:
            try:os.close(fd)
            except OSError:pass
        if before is not None:
            save(root/'preparation.json',{'event':'restoring'})
            try:
                final=reconcile(protocol,args.port,before,restore_modes=True);save(root/'after.json',final)
                result.update(all_off=True,settings_restored=True)
            except BaseException as error:result['restoration_error']=str(error)
        if (root/'guardian.json').exists():result['guardian']=json.loads((root/'guardian.json').read_text())
        result['complete']=result['all_off'] and result['settings_restored']
        save(root/'result.json',result)
    if not result['complete']:raise SystemExit(1)

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--port',default='/dev/serial0');p.add_argument('--protocol-dir',type=Path,default=Path('/root/calibration'))
    p.add_argument('--cleanup',action='store_true');p.add_argument('--relax-only',action='store_true');p.add_argument('--robotd',type=Path);p.add_argument('--calibration',type=Path);p.add_argument('--model-source',type=Path);p.add_argument('--pose',choices=('home','zero'),default='home')
    a=p.parse_args()
    if a.cleanup:return cleanup(a.run,a.port,a.protocol_dir)
    if a.relax_only:return relax_only(a.run,a.port,a.protocol_dir)
    if not all((a.robotd,a.calibration,a.model_source)):p.error('robotd, calibration and model source are required')
    run(a)

if __name__=='__main__':main()
