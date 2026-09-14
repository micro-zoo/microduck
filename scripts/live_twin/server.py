#!/usr/bin/env python3
"""Live Twin telemetry and supervised HOME/zero/relax control on the USB network."""
from pathlib import Path
from http.server import ThreadingHTTPServer,SimpleHTTPRequestHandler
from collections import deque
import argparse,json,math,signal,subprocess,threading,time,sys
from bus import ReadBus,MOTORS,IDS,RAD_PER_TICK,relative_tick,protocol
import bus as bus_module
from control import Access,Controller,SystemdRunner,ControlError
HERE=Path(__file__).resolve().parent

class State:
    def __init__(self,calibration_path):
        self.lock=threading.Condition();self.stop=threading.Event();self.sequence=0
        self.reader_transition=threading.Lock();self.reader_pause=threading.Event();self.reader_idle=threading.Event();self.reader_idle.set()
        self.control={'enabled':False,'phase':'offline','message':'仅预览 · 未启用电机控制','mode_active':False,'owner_id':None}
        self.latest={};self.metadata={};self.times=deque(maxlen=100);self.cycle_ms=None;self.last_error='等待总线连接'
        self.calibration=None;self.calibration_error=None
        self.catalog=json.loads((HERE/'dist/assets/model.json').read_text(encoding='utf-8'))['motors']
        try:
            document=json.loads(Path(calibration_path).read_text(encoding='utf-8'))
            expected=dict(MOTORS);entries=document['joints']
            if len(entries)!=15 or {e['id'] for e in entries}!=set(IDS):raise ValueError('Calibration must contain exactly 15 motor IDs')
            for entry in entries:
                if entry['name']!=expected[entry['id']] or type(entry['zero_tick']) not in (int,float) or not math.isfinite(entry['zero_tick']) or not 0<=entry['zero_tick']<=4095:
                    raise ValueError('Invalid joint zero entry')
            if document.get('mouth_reference')!='closed_is_zero':raise ValueError('Expected explicit closed-mouth-zero reference')
            self.calibration=document
        except (OSError,ValueError,KeyError,TypeError) as error:self.calibration_error=str(error)
        self.zeros={e['id']:e for e in self.calibration['joints']} if self.calibration else {}
    def set_control(self,value):
        with self.lock:self.control=value;self.sequence+=1;self.lock.notify_all()
    def pause_bus(self):
        with self.reader_transition:self.reader_pause.set()
        if not self.reader_idle.wait(4):raise RuntimeError('Telemetry reader did not release UART')
    def resume_bus(self):
        with self.reader_transition:self.reader_pause.clear()
    def publish_pose_sample(self,sample,target):
        fields=('raw_ticks','currents_ma','pwm','velocities','volts','temperatures','torque','errors','watchdog','positions')
        for key in fields:
            values=sample.get(key)
            if not isinstance(values,list) or len(values)!=15 or any(type(v) not in (int,float) or not math.isfinite(v) for v in values):
                raise ValueError('Invalid controller telemetry')
        frame={id:{'id':id,'raw_tick':sample['raw_ticks'][j],'current_ma':sample['currents_ma'][j],
            'pwm_raw':sample['pwm'][j],'velocity_raw':sample['velocities'][j]/(.229*math.tau/60),
            'voltage_v':sample['volts'][j],'temperature_c':sample['temperatures'][j],'torque':sample['torque'][j],
            'hardware_error':sample['errors'][j],'watchdog':sample['watchdog'][j],'status_error':0,
            'operating_mode':4} for j,id in enumerate(IDS)}
        self.publish(frame,[],0)
    def mark_torque_off(self):
        with self.lock:
            # Do not refresh pose timestamps: the robot may move freely after relax.
            self.latest={id:({**raw,'torque':0},stamp) for id,(raw,stamp) in self.latest.items()}
            self.sequence+=1;self.lock.notify_all()
    def publish(self,frame,errors,cycle):
        now=time.monotonic()
        with self.lock:
            for id,raw in frame.items():self.latest[id]=(raw,now-(cycle or 0)/1000)
            if frame:self.times.append(now)
            self.cycle_ms=cycle;self.last_error='; '.join(errors) if errors else None
            self.sequence+=1;self.lock.notify_all()
    def payload(self):
        now=time.monotonic()
        with self.lock:
            motors=[]
            for definition in self.catalog:
                id=definition['id'];raw,stamp=self.latest.get(id,({},None));zero=self.zeros.get(id)
                age=(now-stamp)*1000 if stamp is not None else None
                online=age is not None and age<500
                meta=self.metadata.get(id)
                mismatch=False
                if zero and meta:
                    old=zero.get('metadata',{})
                    keys=('id','model','baud_code','drive_mode','operating_mode','homing_offset','protocol')
                    mismatch=any(meta.get(k)!=old.get(k) for k in keys if not (k=='operating_mode' and self.control.get('mode_active') and raw.get('operating_mode')==4))
                calibrated=bool(zero and meta and not mismatch)
                angle=relative_tick(raw['raw_tick'],zero['zero_tick'])*RAD_PER_TICK if online and calibrated else None
                motor={**definition,**raw,'online':online,'age_ms':age,'calibrated':calibrated,'calibration_mismatch':mismatch,
                    'angle_rad':angle,'angle_deg':math.degrees(angle) if angle is not None else None,
                    'zero_tick':zero['zero_tick'] if zero else None,
                    'velocity_rad_s':raw.get('velocity_raw',0)*.229*2*math.pi/60 if online else None,
                    'single_turn_range_risk':zero.get('single_turn_range_risk',False) if zero else False,
                    'positive_margin_deg':zero.get('positive_margin_deg') if zero else None,
                    'negative_margin_deg':zero.get('negative_margin_deg') if zero else None}
                motors.append(motor)
            recent=[t for t in self.times if now-t<5]
            hz=(len(recent)-1)/(recent[-1]-recent[0]) if len(recent)>1 else 0.0
            cal={k:self.calibration[k] for k in ('captured_at','motor_count','mouth_reference','reference')} if self.calibration else None
            return {'sequence':self.sequence,'motors':motors,'read_hz':hz,'cycle_ms':self.cycle_ms,
                'calibration':cal,'calibration_error':self.calibration_error,'last_error':self.last_error,
                'read_only':not self.control.get('enabled'), 'control':dict(self.control)}

def assert_robotd_stopped():
    result=subprocess.run(['systemctl','show','robotd.service','--property=ActiveState','--value'],capture_output=True,text=True,timeout=3,check=True)
    if result.stdout.strip() not in ('inactive','failed'):raise RuntimeError('robotd 正在使用总线，请先停止 robotd')

def read_forever(state,port):
    while not state.stop.is_set():
        with state.reader_transition:
            paused=state.reader_pause.is_set()
            if not paused:state.reader_idle.clear()
        if paused:state.stop.wait(.05);continue
        try:
            assert_robotd_stopped()
            with protocol.LinuxPort(port) as wire:
                wire.open_serial();wire.set_baud(1000000);bus=ReadBus(wire)
                meta={};errors=[]
                for id in IDS:
                    try:meta[id]=bus.metadata(id)
                    except Exception as e:errors.append(str(e))
                with state.lock:state.metadata=meta
                next_metadata=time.monotonic()+10
                while not state.stop.is_set() and not state.reader_pause.is_set():
                    started=time.monotonic()
                    try:
                        frame,errors,cycle=bus.sample()
                        state.publish(frame,errors,cycle)
                    except Exception as error:state.publish({},[str(error)],None)
                    if time.monotonic()>=next_metadata:
                        assert_robotd_stopped();wire.assert_free()
                        # Only retry metadata for missing devices. Known devices' critical
                        # settings are immutable while this process exclusively owns UART.
                        for id in IDS:
                            if id not in state.metadata:
                                try:
                                    current=bus.metadata(id)
                                    with state.lock:state.metadata[id]=current
                                except Exception:pass
                        next_metadata=time.monotonic()+10
                    state.stop.wait(max(0,.1-(time.monotonic()-started)))
        except Exception as error:
            state.publish({},[str(error)],None)
            state.stop.wait(1)
        finally:state.reader_idle.set()

def handler_for(state,controller=None,access=None,assets_dir=None):
    class Handler(SimpleHTTPRequestHandler):
        protocol_version='HTTP/1.1'
        def __init__(self,*args,**kwargs):super().__init__(*args,directory=str(HERE/'dist'),**kwargs)
        def log_message(self,format,*args):
            if len(args)>1 and str(args[1]) not in ('200','304'):super().log_message(format,*args)
        def json_response(self,payload,status=200):
            data=json.dumps(payload,ensure_ascii=False,allow_nan=False,separators=(',',':')).encode()
            self.send_response(status);self.send_header('Content-Type','application/json; charset=utf-8');self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)
        def do_GET(self):
            route=self.path.split('?',1)[0]
            if route=='/api/control/session':
                if controller is None:return self.json_response({'error':'此页面仅预览，未连接控制器'},503)
                if not access.allowed(self.client_address[0],self.headers.get('Host',''),self.headers.get('Origin')):
                    return self.json_response({'error':'控制仅限 USB 调试网络，请使用控制板的 USB 地址'},403)
                try:return self.json_response(access.create())
                except ControlError as e:return self.json_response({'error':str(e)},e.status)
            if route=='/api/state':return self.json_response(state.payload())
            if route=='/api/calibration':return self.json_response(state.calibration or {'error':state.calibration_error},200 if state.calibration else 503)
            if route=='/api/events':
                self.send_response(200);self.send_header('Content-Type','text/event-stream');self.send_header('Cache-Control','no-cache');self.send_header('Connection','close');self.end_headers()
                previous=-1
                try:
                    while not state.stop.is_set():
                        with state.lock:
                            if previous==state.sequence:state.lock.wait(timeout=.8)
                        data=state.payload();previous=data['sequence']
                        self.wfile.write(('event: telemetry\ndata: '+json.dumps(data,ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n\n').encode());self.wfile.flush()
                except (BrokenPipeError,ConnectionResetError,TimeoutError):pass
                self.close_connection=True;return
            if route.startswith('/api/'):return self.json_response({'error':'not found'},404)
            return super().do_GET()
        def do_POST(self):
            try:
                if controller is None:raise ControlError('此页面仅预览，未连接控制器',503)
                if not access.allowed(self.client_address[0],self.headers.get('Host',''),self.headers.get('Origin')):raise ControlError('控制来源未授权',403)
                owner=access.identify(self.headers.get('X-Microduck-Control',''))
                if self.headers.get_content_type()!='application/json':raise ControlError('需要 JSON 请求',415)
                length=int(self.headers.get('Content-Length','0'))
                if not 0<=length<=4096:raise ControlError('请求过大',413)
                body=json.loads(self.rfile.read(length) or b'{}')
                if body!={}:raise ControlError('动作不接受自定义电机目标或参数',400)
                route=self.path.split('?',1)[0]
                if route=='/api/control/heartbeat':controller.heartbeat(owner)
                elif route=='/api/control/relax':controller.relax()
                elif route in ('/api/control/home','/api/control/zero'):controller.start(route.rsplit('/',1)[1],owner)
                else:raise ControlError('未知控制接口',404)
                return self.json_response({'accepted':True},202)
            except ControlError as e:
                self.close_connection=True;return self.json_response({'error':str(e)},e.status)
            except (ValueError,TypeError):
                self.close_connection=True;return self.json_response({'error':'请求格式错误'},400)
        def do_PUT(self):return self.json_response({'error':'method not allowed'},405)
        def do_DELETE(self):return self.do_PUT()
        def translate_path(self,path):
            translated=Path(super().translate_path(path))
            route=path.split('?',1)[0]
            if assets_dir is not None and route.startswith(('/assets/','/vendor/')):
                relative=translated.relative_to(HERE/'dist')
                candidate=(Path(assets_dir)/relative).resolve()
                if candidate.is_relative_to(Path(assets_dir).resolve()):return str(candidate)
            return str(translated)
        def list_directory(self,path):return self.json_response({'error':'directory listing disabled'},403)
        def end_headers(self):
            if not self.path.split('?',1)[0].endswith('.stl') and not self.path.startswith('/api/'):
                self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff')
            self.send_header('Referrer-Policy','no-referrer')
            self.send_header('X-Frame-Options','DENY')
            super().end_headers()
    return Handler

def main():
    global protocol
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--listen',default='127.0.0.1');parser.add_argument('--http-port',type=int,default=8765)
    parser.add_argument('--serial-port',default='/dev/serial0');parser.add_argument('--calibration',type=Path,default=HERE/'calibration.json')
    parser.add_argument('--offline',action='store_true',help='Static preview only; never opens a serial port')
    parser.add_argument('--assets-dir',type=Path,default=HERE/'dist')
    parser.add_argument('--protocol-dir',type=Path,default=HERE.parent)
    parser.add_argument('--robotd',type=Path,default=Path('/root/calibration/control-path/bin/robotd'))
    parser.add_argument('--model-source',type=Path,default=Path('/root/calibration/control-path/duck-control/src/model.rs'))
    parser.add_argument('--model-xml',type=Path,default=Path('/root/calibration/control-path/kinematics/assets/alpha/robot_walk.xml'))
    parser.add_argument('--control-runs',type=Path,default=Path('/var/lib/robot/live-twin/runs'))
    parser.add_argument('--control-host',action='append',default=[])
    parser.add_argument('--control-network',action='append',default=['127.0.0.0/8','192.168.77.0/24'])
    args=parser.parse_args();state=State(args.calibration)
    worker=None;controller=None;access=None
    if not args.offline:
        if not sys.platform.startswith('linux'):parser.error('Hardware telemetry runs on the Linux controller; use --offline for preview')
        protocol=bus_module.load_protocol(args.protocol_dir)
        controller=Controller(state,SystemdRunner(args.robotd,args.model_source,args.serial_port,args.protocol_dir),args.control_runs,args.model_xml)
        access=Access(args.control_host or [f'192.168.77.1:{args.http_port}',f'127.0.0.1:{args.http_port}',f'localhost:{args.http_port}'],args.control_network)
        worker=threading.Thread(target=read_forever,args=(state,args.serial_port),daemon=True);worker.start()
    else:state.last_error='本地模型预览 · 未连接硬件'
    server=ThreadingHTTPServer((args.listen,args.http_port),handler_for(state,controller,access,args.assets_dir));server.daemon_threads=True
    def interrupted(sig,frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted)
    print(f'Local URL: http://{args.listen}:{server.server_address[1]}',flush=True)
    try:server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:pass
    finally:
        if controller:controller.close()
        state.stop.set()
        with state.lock:state.lock.notify_all()
        server.server_close()
        if worker:worker.join(timeout=4)
if __name__=='__main__':main()
