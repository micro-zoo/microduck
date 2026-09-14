#!/usr/bin/env python3
"""Linux-only end-to-end check using real robotd/robotctl and a private PTY bus.

Never opens a hardware serial path. The 15 servos and IMU below are register-level
emulators, not a physics simulation or evidence of real motor tracking.
"""
import argparse,json,math,os,pty,re,select,socket,struct,subprocess,tempfile,threading,time,tty
import copy
from pathlib import Path

HEADER=b'\xff\xff\xfd\x00'
R=2*math.pi/4096

def crc(data):
    value=0
    for b in data:
        value^=b<<8
        for _ in range(8):value=((value<<1)^(0x8005 if value&0x8000 else 0))&65535
    return value

def packet(id,payload):
    body=(b'\x55\x00'+payload).replace(b'\xff\xff\xfd',b'\xff\xff\xfd\xfd')
    raw=HEADER+bytes([id])+struct.pack('<H',len(body)+2)+body
    return raw+struct.pack('<H',crc(raw))

def source_array(path,name,kind):
    text=Path(path).read_text();body=re.search(r'pub const '+name+r':.*?= \[(.*?)\];',text,re.S).group(1)
    return [kind(x.strip()) for x in re.sub(r'//[^\n]*','',body).split(',') if x.strip()]

def assert_test_isolation():
    if os.geteuid()==0:raise RuntimeError('Refusing root test daemon; use run_calibrated_control_path.py')
    status=dict(line.split(':',1) for line in Path('/proc/self/status').read_text().splitlines() if ':' in line)
    if status.get('NoNewPrivs','').strip()!='1' or int(status.get('CapEff','0').strip(),16)!=0:
        raise RuntimeError('Requires NoNewPrivileges and no effective capabilities')
    if os.environ.get('DUCK_CONTROL_PATH_SANDBOX')!='1':raise RuntimeError('Requires the systemd sandbox launcher')
    for endpoint in ('/run/systemd/private','/run/dbus/system_bus_socket'):
        try:os.stat(endpoint)
        except (FileNotFoundError,PermissionError):continue
        raise RuntimeError(f'Host management endpoint must be hidden: {endpoint}')
    for device in Path('/dev').iterdir():
        if device.name.startswith(('ttyS','ttyUSB','ttyACM','ttyAMA','i2c-','spidev','gpiochip')) and device.is_char_device():
            raise RuntimeError(f'Physical device must be hidden by PrivateDevices: {device}')
    return {'uid':os.geteuid(),'no_new_privileges':True,'effective_capabilities':0,'host_manager_endpoints_hidden':True,'physical_devices_hidden':True}

def host_action_stubs(root):
    commands=root/'host-action-stubs';commands.mkdir()
    body = """#!/bin/sh
printf '%s\\n' "$0 $*" >> "$DUCK_TEST_HOST_ACTION_LOG"
exit 0
"""
    for name in ('setsid','systemctl','poweroff','reboot','shutdown'):
        path=commands/name;path.write_text(body);path.chmod(0o755)
    return commands


class Bus:
    def __init__(self,calibration,ids,mode=4):
        self.ids=ids;self.cal={j['id']:j for j in calibration['joints']}
        self.master,self.slave=pty.openpty();tty.setraw(self.slave)
        self.path=os.ttyname(self.slave)
        if not self.path.startswith('/dev/pts/'):raise RuntimeError('Requires a private Linux /dev/pts device')
        self.lock=threading.RLock();self.stop=threading.Event();self.errors=[];self.writes=[];self.unsafe_enables=[];self.frames=0;self.imu_samples=0
        self.regs={};self.origins={};self.expected_start=[];self.last_goal_request={}
        for id in ids:
            zero=self.cal[id]['zero_tick'];q={20:.2,13:-1.,32:2.,34:-5*math.pi/180}.get(id,0.)
            unwrapped=round(zero+q/R);raw=unwrapped%4096
            self.origins[id]=zero+raw-unwrapped;self.expected_start.append((raw-self.origins[id])*R)
            d=bytearray(256);d[:2]=(1200).to_bytes(2,'little');d[6]=53;d[7]=id;d[8]=3;d[11]=mode;d[12]=255;d[13]=2
            d[48:52]=struct.pack('<i',4095);d[62]=255;d[63]=52;d[68]=2
            d[116:120]=d[132:136]=struct.pack('<i',raw);d[144:146]=struct.pack('<H',51);d[146]=29
            self.regs[id]=d
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def read(self,id,a,n):
        if id==200:
            d=bytearray(256)
            # Synthetic upright IMU, with a changing gyro LSB to mark fresh samples.
            self.imu_samples+=1
            d[124:136]=struct.pack('<hhh',self.imu_samples%2,0,0)+struct.pack('<eee',0,math.sqrt(.5),0)
            return bytes(d[a:a+n])
        return bytes(self.regs[id][a:a+n])
    def write(self,id,a,data):
        d=self.regs[id];value=int.from_bytes(data,'little',signed=a==116)
        self.writes.append({'t':time.monotonic(),'id':id,'address':a,'value':value,'torque_before':d[64]})
        if a==64 and value==1:
            goal=struct.unpack_from('<i',d,116)[0];pos=struct.unpack_from('<i',d,132)[0]
            requested=self.last_goal_request.get(id)
            if requested is None or abs(requested-pos)>1 or abs(goal-pos)>1:self.unsafe_enables.append((id,goal,pos,requested))
        d[a:a+len(data)]=data
        if a==116:
            self.last_goal_request[id]=value
            if d[64]:d[132:136]=data # ideal instantaneous actuator, no dynamics
            else:d[116:120]=d[132:136] # observed XL330 torque-off behavior
    def dispatch(self,id,body):
        self.frames+=1;op=body[0];data=body[1:]
        if op==1:
            return b''.join(packet(i,struct.pack('<HB',1200,53)) for i in (self.ids if id==254 else [id]) if i in self.regs or i==200)
        if op==2:
            a,n=struct.unpack('<HH',data);return packet(id,self.read(id,a,n))
        if op==3:
            a=struct.unpack_from('<H',data)[0];self.write(id,a,data[2:]);return packet(id,b'')
        if op==0x82:
            a,n=struct.unpack_from('<HH',data);return b''.join(packet(i,self.read(i,a,n)) for i in data[4:])
        if op==0x83:
            a,n=struct.unpack_from('<HH',data)
            for pos in range(4,len(data),n+1):self.write(data[pos],a,data[pos+1:pos+1+n])
            return b''
        if op==8:
            d=self.regs[id];old=struct.unpack_from('<i',d,132)[0];new=old%4096
            self.origins[id]-=old-new
            self.last_goal_request.pop(id,None)
            d[64]=0;d[116:120]=d[132:136]=struct.pack('<i',new)
            self.writes.append({'t':time.monotonic(),'id':id,'address':'reboot','value':new})
            return packet(id,b'')
        raise RuntimeError(f'Unexpected virtual bus instruction {op:#x}')
    def run(self):
        buffer=bytearray()
        try:
            while not self.stop.is_set():
                if not select.select([self.master],[],[],.05)[0]:continue
                buffer.extend(os.read(self.master,8192))
                while len(buffer)>=7:
                    if buffer[:4]!=HEADER:raise RuntimeError('Malformed virtual-bus header')
                    length=int.from_bytes(buffer[5:7],'little')+7
                    if len(buffer)<length:break
                    raw=bytes(buffer[:length]);del buffer[:length]
                    if crc(raw[:-2])!=int.from_bytes(raw[-2:],'little'):raise RuntimeError('Bad instruction CRC')
                    body=raw[7:-2].replace(b'\xff\xff\xfd\xfd',b'\xff\xff\xfd')
                    with self.lock:reply=self.dispatch(raw[4],body)
                    if reply:os.write(self.master,reply)
        except BaseException as e:self.errors.append(str(e))
    def manual_move(self,id,delta):
        with self.lock:
            d=self.regs[id];assert d[64]==0
            raw=struct.unpack_from('<i',d,132)[0]+delta
            d[116:120]=d[132:136]=struct.pack('<i',raw)

    def snapshot(self):
        with self.lock:return {id:{'raw':struct.unpack_from('<i',d,132)[0],'torque':d[64],'origin':self.origins[id]} for id,d in self.regs.items()}
    def close(self):
        self.stop.set();self.thread.join(1);os.close(self.slave);os.close(self.master)

class Fixture:
    def __init__(self,args,mode=4,battery_shutdown=False):
        self.args=args;self.temp=tempfile.TemporaryDirectory(prefix='cal-path-');self.root=Path(self.temp.name)
        self.ids=source_array(args.model_source,'JOINT_IDS',int);self.home=source_array(args.model_source,'DEFAULT_POSITION',float)
        self.cal=json.loads(args.calibration.read_text());self.bus=Bus(self.cal,self.ids,mode)
        self.socket=self.root/'robotd.sock';self.log=self.root/'robotd.log';self.params=self.root/'robotd.toml'
        self.params.write_text(f'[bus]\nport = "{self.bus.path}"\ncalibration = "{args.calibration}"\n[audio]\nenabled = false\n[safety]\nbattery_empty_shutdown = {str(battery_shutdown).lower()}\n')
        self.actions=self.root/'host-actions.log'
        commands=host_action_stubs(self.root)
        self.env={**os.environ,'DUCK_RUNTIME_DIR':str(self.root/'run'),'RUST_LOG':'info','PATH':str(commands)+os.pathsep+os.environ.get('PATH','/usr/bin:/bin'),'DUCK_TEST_HOST_ACTION_LOG':str(self.actions)}
        self.process=subprocess.Popen([str(args.robotd),'--no-policy','--port',self.bus.path,'--params',str(self.params),'--socket',str(self.socket)],stdout=self.log.open('w'),stderr=subprocess.STDOUT,env=self.env)
        self.monitor=None;self.state_path=self.root/'states.jsonl'
    def cli(self,*command):
        return json.loads(subprocess.check_output([str(self.args.robotctl),'--robot-socket',str(self.socket),'--socket',str(self.root/'updater.sock'),'--config-socket',str(self.root/'config.sock'),*command,'--json'],env=self.env,text=True,timeout=5))
    def rpc(self,method):
        with socket.socket(socket.AF_UNIX) as s:
            s.settimeout(2);s.connect(str(self.socket));s.sendall((json.dumps({'jsonrpc':'2.0','id':1,'method':method,'params':{}})+'\n').encode())
            return json.loads(s.makefile().readline())['result']
    def wait(self,predicate,timeout=8):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if self.bus.errors:raise RuntimeError(self.bus.errors)
            if self.process.poll() is not None:raise RuntimeError(self.log.read_text())
            try:
                value=predicate()
                if value:return value
            except (OSError,KeyError,IndexError,json.JSONDecodeError):pass
            time.sleep(.04)
        raise RuntimeError('Timed out: '+self.log.read_text()[-4000:])
    def state(self):
        if not self.state_path.exists():return None
        for line in reversed(self.state_path.read_text().splitlines()):
            try:
                d=json.loads(line)
                if d.get('method')=='robot.state':d=d['params']
                if 'joints' in d:return d
            except json.JSONDecodeError:pass
        return None
    def start_monitor(self):
        self.monitor=subprocess.Popen([str(self.args.robotctl),'--robot-socket',str(self.socket),'--socket',str(self.root/'updater.sock'),'--config-socket',str(self.root/'config.sock'),'--pad-socket',str(self.root/'pad.sock'),'--tof-socket',str(self.root/'tof.sock'),'monitor','--hz','20','--json'],stdout=self.state_path.open('w'),stderr=(self.root/'monitor.err').open('w'),env=self.env)
    def home_reached(self):
        snap=self.bus.snapshot();state=self.state()
        if not state or len(state.get('joints',[]))!=15 or state.get('policy')!='held':return False
        return all(snap[id]['torque']==1 and abs((snap[id]['raw']-snap[id]['origin'])*R-self.home[i])<=1.01*R and abs(state['joints'][i]-self.home[i])<=1.01*R for i,id in enumerate(self.ids))
    def close(self):
        for p in (self.monitor,self.process):
            if p and p.poll() is None:
                p.terminate()
                try:p.wait(5)
                except subprocess.TimeoutExpired:p.kill();p.wait()
        self.bus.close();self.temp.cleanup()

def check_guarded_home(args):
    """Real standalone init, including a motor-current fault and verified restoration."""
    ids=source_array(args.model_source,'JOINT_IDS',int)
    home=source_array(args.model_source,'DEFAULT_POSITION',float)
    original=json.loads(args.calibration.read_text())
    checks={}
    for mode,fault in ((3,False),(3,True),(4,False),(4,True)):
        cal=copy.deepcopy(original) if mode==4 else {'joints':[{k:v for k,v in j.items() if k!='limits_rad'} for j in original['joints']]}
        duration=10 if mode==4 else 5
        with tempfile.TemporaryDirectory(prefix='guarded-home-') as d:
            root=Path(d);bus=Bus(cal,ids,mode=mode)
            try:
                # All starts are close to HOME and inside a single encoder revolution.
                # Give the mouth and hip yaw a negative offset to avoid the 4095 seam.
                with bus.lock:
                    for j,id in enumerate(ids):
                        b=bus.regs[id];zero=bus.cal[id]['zero_tick']
                        q=home[j]-.04
                        if mode==4 and id==20:q=.04
                        if mode==4 and id==30:q=math.radians(63)
                        raw=round(zero+q/R)
                        if mode==4:raw%=4096
                        assert 0<=raw<=4095
                        b[36:38]=struct.pack('<H',885);b[38:40]=struct.pack('<H',1750)
                        b[63]=52;b[84:86]=struct.pack('<H',400);b[100:102]=struct.pack('<H',885)
                        b[116:120]=b[132:136]=struct.pack('<i',raw)
                    before={id:bytes(b) for id,b in bus.regs.items()}
                if mode==4:
                    from run_guarded_home import recovery_calibration
                    cal,changes=recovery_calibration(cal,before,args.model_source,duration)
                    assert any(c['id']==30 for c in changes)
                original_read=bus.read
                def read(id,a,n):
                    b=bytearray(original_read(id,a,n))
                    if fault and id==14 and bus.regs[id][64] and a<=126 and a+n>=128:
                        b[126-a:128-a]=struct.pack('<h',800 if mode==4 else 400)
                    return bytes(b)
                bus.read=read
                calibration=root/'calibration.json';calibration.write_text(json.dumps(cal))
                params=root/'params.toml';params.write_text(f'[bus]\nport="{bus.path}"\ncalibration="{calibration}"\n[audio]\nenabled=false\n')
                telemetry=root/'telemetry.jsonl'
                read_fd,write_fd=os.pipe()
                env={**os.environ,'DUCK_HOME_WATCHDOG_FD':str(write_fd),'DUCK_RUNTIME_DIR':str(root/'run')}
                try:
                    out=subprocess.run([str(args.robotd),'--params',str(params),'--socket',str(root/'init.sock'),
                        'init','--guarded','--duration',f'{duration}s','--telemetry',str(telemetry),*(['--higher-effort'] if mode==4 else [])],
                        env=env,pass_fds=(write_fd,),capture_output=True,text=True,timeout=duration+7)
                finally:os.close(read_fd);os.close(write_fd)
                events=[json.loads(line) for line in telemetry.read_text().splitlines()]
                key='guarded_home_fault' if fault else 'guarded_home_success'
                if mode==4:key+='_extended'
                (args.output/(key+'.jsonl')).write_text(telemetry.read_text())
                (args.output/(key+'.log')).write_text(out.stdout+out.stderr)
                (args.output/(key+'-writes.json')).write_text(json.dumps(bus.writes,indent=2))
                tail=events[-1]
                assert tail['event']=='home_cleanup' and tail['all_off'] and tail['settings_restored'],(out.stderr,tail)
                assert (out.returncode==0)==(not fault),(out.stderr,tail)
                assert tail['reached']==(not fault)
                assert not bus.unsafe_enables,bus.unsafe_enables
                for id,b in bus.regs.items():
                    assert b[:64]==before[id][:64] and b[64]==0
                    for a,n in ((80,6),(88,4),(98,1),(100,2),(108,8)):
                        assert b[a:a+n]==before[id][a:a+n],(id,a)
                checks[key+'_relaxes_and_restores']=True
            finally:bus.close()
    return checks

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--robotd',type=Path,required=True);p.add_argument('--robotctl',type=Path,required=True);p.add_argument('--calibration',type=Path,help='Existing extended calibration; omit for a generated synthetic fixture')
    p.add_argument('--model-source',type=Path,default=Path(__file__).resolve().parents[1]/'duck-control/src/model.rs');p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();isolation=assert_test_isolation();args.output.mkdir(parents=True,exist_ok=True)
    # A failed rerun must never leave an earlier successful result looking current.
    (args.output/'result.json').write_text('{"complete":false}\n')
    if args.calibration is None:
        from export_joint_zero import JOINTS,convert
        source={'mouth_reference':'closed_is_zero','joints':[{'id':id,'name':name,'zero_tick':{20:4071,13:532,32:3069,34:2054}.get(id,2048),'reference_rad':0} for id,name in JOINTS.items()]}
        args.calibration=args.output/'synthetic-calibration.json'
        args.calibration.write_text(json.dumps(convert(source,extended=True))+'\n')
    for key in ('robotd','robotctl','calibration','model_source'):setattr(args,key,getattr(args,key).resolve())
    f=Fixture(args);result={'backend':'private Linux PTY, emulated motors and IMU','physical_motor_or_imu_io':False,'isolation':isolation,'checks':{}}
    try:
        f.wait(lambda:f.socket.exists());f.start_monitor()
        initial=f.wait(lambda:f.state() if f.state() and len(f.state()['joints'])==15 else None)
        assert all(abs(a-b)<1e-6 for a,b in zip(initial['joints'],f.bus.expected_start)),(initial['joints'],f.bus.expected_start)
        assert all(v['torque']==0 for v in f.bus.snapshot().values())
        result['checks']['startup_calibrated_readback']=True
        # Move an unpowered virtual ankle after startup. The old hold target is now stale,
        # so torque-enable must read and preload the new pose rather than chase that target.
        f.bus.manual_move(14,-16)
        ankle=f.ids.index(14);expected=f.bus.expected_start[ankle]-16*R
        f.wait(lambda:f.state() and abs(f.state()['joints'][ankle]-expected)<1e-6)
        result['virtual_manual_move']={'id':14,'delta_ticks':-16}
        result['configure']=f.cli('configure','--file',str(f.params),'--list')
        assert 'bus.calibration' in json.dumps(result['configure'])
        assert f.cli('robot','init')['accepted'];f.wait(lambda:f.state() and f.state()['policy']=='homing');f.wait(f.home_reached)
        first=f.bus.snapshot();state=f.state()
        assert any(v['raw']<0 for v in first.values()) and any(v['raw']>4095 for v in first.values()),first
        result['checks']['init_crosses_signed_encoder_boundaries']=True
        result['first_home']=first;result['monitor_home_joints']=state['joints'];result['expected_home']=f.home
        result['look']=f.cli('robot','look','1','0.2','0')
        assert all(math.isfinite(v) for v in result['look']['head'].values())
        time.sleep(.3)
        assert {i:v['raw'] for i,v in first.items()}=={i:v['raw'] for i,v in f.bus.snapshot().items()}
        result['checks']['look_is_ik_intent_not_direct_motion_without_policy']=True
        assert f.cli('robot','reboot-motors')['accepted']
        f.wait(lambda:len([w for w in f.bus.writes if w['address']=='reboot'])==15)
        time.sleep(.7)
        assert all(v['torque']==0 for v in f.bus.snapshot().values())
        assert f.cli('robot','init')['accepted'];f.wait(lambda:f.state() and f.state()['policy']=='homing');f.wait(f.home_reached)
        result['checks']['reboot_rebinds_turn_origins']=True;result['second_home']=f.bus.snapshot()
        assert not f.bus.unsafe_enables,f.bus.unsafe_enables
        result['checks']['current_pose_preloaded_before_torque']=True
        assert f.cli('robot','relax','--yes')['accepted'];f.wait(lambda:all(v['torque']==0 for v in f.bus.snapshot().values()))
        result['checks']['relax_reaches_all_emulated_motors']=True
        assert f.rpc('robot.shutdown')['accepted']
        f.wait(lambda:f.actions.exists() and 'systemctl poweroff' in f.actions.read_text())
        result['checks']['explicit_poweroff_is_recorded_not_executed']=True
    finally:
        (args.output/'robotd.log').write_text(f.log.read_text())
        if f.state_path.exists():(args.output/'states.jsonl').write_text(f.state_path.read_text())
        (args.output/'bus-writes.json').write_text(json.dumps(f.bus.writes,indent=2)+'\n')
        if (f.root/'monitor.err').exists():(args.output/'monitor.err').write_text((f.root/'monitor.err').read_text())
        if f.actions.exists():(args.output/'host-actions.log').write_text(f.actions.read_text())
        f.close()
    bad=Fixture(args,mode=3)
    try:
        bad.wait(lambda:bad.socket.exists());assert bad.cli('robot','init')['accepted'];time.sleep(1.2)
        assert not bad.bus.writes,bad.bus.writes
        assert 'configured position mode 4' in bad.log.read_text()
        result['checks']['wrong_motor_mode_blocks_all_writes']=True
    finally:
        (args.output/'mode-mismatch.log').write_text(bad.log.read_text());bad.close()
    low=Fixture(args,battery_shutdown=True)
    try:
        low.wait(lambda:low.actions.exists() and 'systemctl poweroff' in low.actions.read_text())
        assert all(v['torque']==0 for v in low.bus.snapshot().values())
        result['checks']['low_voltage_poweroff_is_recorded_not_executed']=True
        result['simulated_motor_voltage_v']=5.1
    finally:
        (args.output/'low-voltage.log').write_text(low.log.read_text())
        if low.actions.exists():(args.output/'low-voltage-host-actions.log').write_text(low.actions.read_text())
        low.close()
    result['checks'].update(check_guarded_home(args))
    from check_live_pose import check_live_pose
    result['checks'].update(check_live_pose(args))
    result['complete']=True
    (args.output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
