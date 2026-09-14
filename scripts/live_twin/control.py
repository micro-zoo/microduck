"""Web-session ownership, UART handoff and a separately supervised pose worker."""
import ipaddress,json,math,os,secrets,shlex,socket,subprocess,sys,tempfile,threading,time
from pathlib import Path
from guardian import BROWSER_TIMEOUT
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from export_joint_zero import convert

class ControlError(Exception):
    def __init__(self,message,status=409):super().__init__(message);self.status=status

class Access:
    def __init__(self,hosts,networks):
        self.hosts=set(hosts);self.networks=[ipaddress.ip_network(v) for v in networks]
        self.lock=threading.Lock();self.sessions={};self.next_id=0
    def allowed(self,peer,host,origin=None):
        try:address=ipaddress.ip_address(peer)
        except ValueError:return False
        return host in self.hosts and any(address in n for n in self.networks) and (origin is None or origin=='http://'+host)
    def create(self):
        with self.lock:
            now=time.monotonic();self.sessions={k:v for k,v in self.sessions.items() if now-v['seen']<3600}
            if len(self.sessions)>=32:raise ControlError('控制页面过多，请关闭不使用的页面',429)
            self.next_id+=1;token=secrets.token_urlsafe(32);session={'id':self.next_id,'seen':now};self.sessions[token]=session
            return {'id':session['id'],'token':token}
    def identify(self,token):
        with self.lock:
            item=self.sessions.get(token);now=time.monotonic()
            if not item or now-item['seen']>3600:raise ControlError('控制会话已过期，请刷新页面',403)
            item['seen']=now;return item['id']

class SystemdRunner:
    def __init__(self,robotd,model_source,port,protocol_dir):
        self.robotd=Path(robotd);self.model_source=Path(model_source);self.port=port;self.protocol_dir=Path(protocol_dir)
    def start(self,run,pose):
        worker=HERE/'pose_worker.py';base=[sys.executable,str(worker),'--run',str(run),'--port',self.port,'--protocol-dir',str(self.protocol_dir)]
        cmd=base+(['--relax-only'] if pose=='relax' else ['--robotd',str(self.robotd),'--model-source',str(self.model_source),'--calibration',str(run/'source-calibration.json'),'--pose',pose])
        cleanup=base+['--cleanup']
        unit='microduck-ui-'+run.name
        command=['systemd-run','--quiet','--wait','--collect','--pipe','--unit='+unit,
            '--property=Type=exec','--property=KillMode=mixed','--property=TimeoutStopSec=15',
            '--property=NoNewPrivileges=yes','--property=Conflicts=robotd.service','--property=Before=robotd.service',
            '--property=ExecStopPost='+shlex.join(cleanup),*cmd]
        output=(run/'worker.log').open('w')
        try:return subprocess.Popen(command,stdout=output,stderr=subprocess.STDOUT)
        finally:output.close()
    def send(self,run,message):
        with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as s:
            s.setblocking(False);s.sendto(message,str(run/'command.sock'))
    def recover(self,run):
        # A server restart never adopts a previous browser's lease. Stop its worker,
        # then reconcile only that transaction's captured settings.
        try:self.send(run,b'S')
        except OSError:pass
        unit='microduck-ui-'+run.name+'.service'
        subprocess.run(['systemctl','stop',unit],capture_output=True,timeout=20)
        subprocess.run([sys.executable,str(HERE/'pose_worker.py'),'--run',str(run),'--port',self.port,
            '--protocol-dir',str(self.protocol_dir),'--cleanup'],check=True,timeout=20)

def message_for(result):
    if not result.get('all_off'):return '无法确认卸力，请断开舵机电源后检查连接'
    if not result.get('settings_restored'):return '已卸力，但原电机设置未完全恢复，请检查日志'
    reason=result.get('reason','')
    if reason=='browser_disconnected':return '网页断联，已自动卸力'
    if reason in ('user_relax','worker_stopped'):return '已卸力'
    if result.get('error') or reason in ('control_failed','controller_finished','guardian_ended'):
        return '动作已停止并卸力，请查看控制记录'
    return '已卸力'

class Controller:
    def __init__(self,state,runner,runs,model_xml):
        self.state=state;self.runner=runner;self.runs=Path(runs).resolve();self.runs.mkdir(parents=True,exist_ok=True);self.runs.chmod(0o700)
        self.model_xml=Path(model_xml);self.lock=threading.RLock();self.run=None;self.process=None;self.owner=None
        self.last_lease=0.;self.lease_expired=False;self.closing=False;self.stop_requested=False;self.awaiting_move=False;self.thread=None
        self.status={'enabled':True,'phase':'idle','pose':None,'owner_id':None,'message':'已卸力 · 请托住机器人后操作','progress':None,'mode_active':False}
        self._publish()
        pointer=self.runs/'active-run.json'
        if pointer.exists():
            data=json.loads(pointer.read_text());previous=self.runs/data['name']
            if previous.parent!=self.runs or not previous.is_dir():raise RuntimeError('Invalid pending control run')
            self._update(phase='recovering',message='正在恢复上次控制会话')
            try:
                self.runner.recover(previous);pointer.unlink()
                self._update(phase='idle',message='上次会话已停止并卸力')
            except Exception as e:self._update(phase='fault',message='上次控制恢复未完成，请点击卸力重试',last_result={'error':str(e)})
    def _publish(self):self.state.set_control(dict(self.status))
    def _update(self,**values):
        with self.lock:self.status.update(values);self._publish()
    def _send(self,byte,required=False):
        if self.run is None:return
        try:self.runner.send(self.run,byte)
        except OSError:
            if required:raise ControlError('控制进程未就绪，请先卸力后重试',503)
    def heartbeat(self,owner):
        with self.lock:
            if self.owner!=owner or self.run is None:return
            self.last_lease=time.monotonic();self._send(b'B')
    def start(self,pose,owner):
        if pose not in ('home','zero'):raise ControlError('未知动作',400)
        with self.lock:
            if self.closing:raise ControlError('服务正在关闭',503)
            if self.status['phase']=='fault':raise ControlError('上次控制恢复未完成，请先点击卸力')
            if self.run is not None:
                if self.owner!=owner:raise ControlError('另一个页面正在控制，可点击卸力结束')
                if self.status['phase']!='holding':raise ControlError('请等待当前动作完成，或点击卸力')
                self._send(b'H' if pose=='home' else b'Z',required=True);self.last_lease=time.monotonic()
                self.awaiting_move=True
                self.status.update(phase='moving',pose=pose,progress=0.,message='正在回 HOME' if pose=='home' else '正在回零');self._publish();return
            frame=self.state.payload();motors=frame['motors']
            if frame.get('calibration_error') or len(motors)!=15 or not all(m.get('online') and m.get('calibrated') and not m.get('hardware_error') and not m.get('status_error') for m in motors):
                raise ControlError('需要 15 个电机在线、标定匹配且无硬件错误')
            if any(m.get('torque') for m in motors):raise ControlError('电机已上力，请先点击卸力')
            self._launch(pose,owner)
    def _launch(self,pose,owner):
        self.run=Path(tempfile.mkdtemp(prefix='pose-',dir=self.runs));self.owner=owner if pose!='relax' else None
        self.last_lease=time.monotonic();self.lease_expired=False;self.stop_requested=False;self.awaiting_move=False
        if pose!='relax':
            try:cal=convert(self.state.calibration,extended=True,model=self.model_xml)
            except Exception as e:self.run=None;self.owner=None;raise ControlError('无法生成控制标定：'+str(e),400)
            (self.run/'source-calibration.json').write_text(json.dumps(cal)+'\n')
        (self.runs/'active-run.json').write_text(json.dumps({'name':self.run.name})+'\n')
        self.status.update(phase='stopping' if pose=='relax' else 'preparing',pose=pose,owner_id=self.owner,progress=0.,mode_active=False,message='正在卸力' if pose=='relax' else '正在准备电机 · 保持躯干支撑')
        self._publish();self.thread=threading.Thread(target=self._work,args=(self.run,pose),daemon=True);self.thread.start()
    def relax(self):
        with self.lock:
            pointer=self.runs/'active-run.json'
            if self.run is None and pointer.exists():
                previous=self.runs/json.loads(pointer.read_text())['name']
                if previous.parent!=self.runs:raise ControlError('控制恢复记录无效',503)
                self.run=previous;self.stop_requested=True
                self._update(phase='stopping',message='正在确认卸力并恢复电机设置')
                self.thread=threading.Thread(target=self._recover,args=(previous,),daemon=True);self.thread.start();return
            if self.run is None:self._launch('relax',None);return
            self.stop_requested=True;self._send(b'S');self.status.update(phase='stopping',message='正在卸力');self._publish()
    def _recover(self,run):
        result={}
        try:
            self.state.pause_bus();self.runner.recover(run)
            result=json.loads((run/'result.json').read_text()) if (run/'result.json').exists() else {}
        except Exception as e:result={'error':str(e)}
        finally:
            with self.lock:
                okay=result.get('all_off') and result.get('settings_restored')
                if okay:
                    self.state.mark_torque_off()
                    try:(self.runs/'active-run.json').unlink()
                    except FileNotFoundError:pass
                self.state.resume_bus();self.run=None
                self._update(phase='idle' if okay else 'fault',owner_id=None,mode_active=False,message=message_for(result),last_result=result)
    def _telemetry(self,event):
        kind=event.get('event')
        if kind=='home_preflight':self._update(mode_active=True)
        elif kind=='pose_moving':
            with self.lock:
                self.awaiting_move=False;self._update(pose=event['pose'],duration_s=event['duration_s'])
        elif kind in ('home_sample','home_reached','zero_reached','home_fault_sample'):
            s=event['sample'];self.state.publish_pose_sample(s,event.get('target'))
            with self.lock:
                if self.stop_requested or self.awaiting_move:return
                holding=kind in ('home_reached','zero_reached') or event.get('phase')=='holding'
                pose=event.get('pose',self.status['pose'])
                self.status.update(phase='holding' if holding else 'moving',pose=pose,progress=1. if holding else event.get('progress',0.),mode_active=True,
                    message=('保持 HOME' if pose=='home' else '保持零位') if holding else ('正在回 HOME' if pose=='home' else '正在回零'))
                self._publish()
        elif kind=='home_cleanup' and event.get('all_off'):self.state.mark_torque_off()
    def _work(self,run,pose):
        process=None;cursor=0;pending='';result={}
        try:
            self.state.pause_bus()
            if pose!='relax' and (self.stop_requested or time.monotonic()-self.last_lease>BROWSER_TIMEOUT):pose='relax'
            process=self.runner.start(run,pose)
            with self.lock:self.process=process
            while True:
                telemetry=run/'telemetry.jsonl'
                if telemetry.exists():
                    with telemetry.open() as f:f.seek(cursor);chunk=f.read(262144);cursor=f.tell()
                    pending+=chunk
                    if len(pending)>524288:raise RuntimeError('Invalid telemetry record size')
                    lines=pending.split('\n');pending=lines.pop()
                    for line in lines:
                        if line:self._telemetry(json.loads(line))
                if pose!='relax':
                    with self.lock:
                        expired=time.monotonic()-self.last_lease>BROWSER_TIMEOUT
                        self.lease_expired=self.lease_expired or expired
                        if expired or self.stop_requested:
                            self.stop_requested=True;self._send(b'S');self.status.update(phase='stopping',message='网页断联，正在卸力' if expired else '正在卸力');self._publish()
                if process.poll() is not None:break
                time.sleep(.05)
            if (run/'result.json').exists():
                result=json.loads((run/'result.json').read_text())
                if self.lease_expired:result['reason']='browser_disconnected'
            else:raise RuntimeError('Control worker exited without a result')
        except BaseException as e:
            result={'error':str(e)}
            try:
                self.runner.recover(run)
                if (run/'result.json').exists():result={**json.loads((run/'result.json').read_text()),'error':str(e)}
            except Exception as recovery:result['restoration_error']=str(recovery)
        finally:
            if result.get('all_off'):self.state.mark_torque_off()
            with self.lock:
                if result.get('all_off') and result.get('settings_restored'):
                    try:(self.runs/'active-run.json').unlink()
                    except FileNotFoundError:pass
                # Complete the old UART handoff before another HTTP request can
                # create its run or overwrite the recovery pointer.
                self.state.resume_bus()
                self.run=None;self.process=None;self.owner=None
                self.status.update(phase='idle' if result.get('all_off') and result.get('settings_restored') else 'fault',owner_id=None,progress=None,mode_active=False,
                    message=message_for(result),last_result=result)
                self._publish()
    def close(self):
        with self.lock:
            self.closing=True
            if self.run is not None:self.stop_requested=True;self._send(b'S')
        if self.thread:self.thread.join(20)
