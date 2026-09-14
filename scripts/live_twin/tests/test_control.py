import http.client,json,math,sys,tempfile,threading,time,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from server import State,handler_for
from control import Access,Controller,ControlError
from guardian import Deadlines,off_packets,IDS
from http.server import ThreadingHTTPServer
from bus import MOTORS

ROOT=Path(__file__).resolve().parents[3]

def fixture():
    joints=[]
    for id,name in MOTORS:
        joints.append({'id':id,'name':name,'zero_tick':2048,'reference_rad':0,
            'metadata':{'id':id,'model':1200,'baud_code':3,'drive_mode':0,'operating_mode':3,'homing_offset':0,'protocol':2}})
    return {'captured_at':'2026-09-14T00:00:00Z','motor_count':15,'mouth_reference':'closed_is_zero','reference':'test','joints':joints}

def state_for(root):
    document=fixture();path=root/'fixture.json';path.write_text(json.dumps(document));state=State(path)
    state.metadata={j['id']:j['metadata'] for j in document['joints']}
    state.publish({id:{'id':id,'raw_tick':2048,'torque':0,'hardware_error':0,'status_error':0,'velocity_raw':0,'voltage_v':5.1,'temperature_c':29,'current_ma':0,'watchdog':0} for id in IDS},[],0)
    return state

def sample():
    return {'raw_ticks':[2048]*15,'currents_ma':[0]*15,'pwm':[0]*15,'velocities':[0.]*15,'volts':[5.1]*15,
        'temperatures':[29]*15,'torque':[1]*15,'errors':[0]*15,'watchdog':[15]*15,'positions':[0.]*15,'elapsed_s':1.}

class Process:
    def __init__(self):self.code=None
    def poll(self):return self.code

class FakeRunner:
    def __init__(self):self.process=None;self.sent=[];self.run=None
    def start(self,run,pose):
        self.run=run;self.process=Process()
        if pose=='relax':self.finish()
        return self.process
    def finish(self):
        if self.run:(self.run/'result.json').write_text(json.dumps({'all_off':True,'settings_restored':True,'reason':'user_relax'}))
        if self.process:self.process.code=0
    def send(self,run,message):
        self.sent.append(message)
        if message==b'S':self.finish()
    def recover(self,run):self.run=run;self.finish()

class Tests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name);self.state=state_for(self.root);self.runner=FakeRunner()
        self.controller=Controller(self.state,self.runner,self.root/'runs',ROOT/'kinematics/assets/alpha/robot_walk.xml')
    def tearDown(self):self.controller.close();self.temp.cleanup()
    def wait(self,predicate):
        end=time.monotonic()+2
        while not predicate():
            if time.monotonic()>end:self.fail('timed out')
            time.sleep(.01)
    def test_single_owner_and_stop_are_not_queued_behind_motion(self):
        self.controller.start('home',1);self.wait(lambda:self.runner.process is not None)
        with self.assertRaises(ControlError):self.controller.start('zero',1)
        with self.assertRaises(ControlError):self.controller.start('home',2)
        self.controller.heartbeat(2);self.assertNotIn(b'B',self.runner.sent)
        self.controller.relax();self.wait(lambda:self.controller.run is None)
        self.assertIn(b'S',self.runner.sent);self.assertEqual(self.state.control['phase'],'idle')
        self.assertFalse(self.state.reader_pause.is_set())
    def test_holding_can_retarget_and_old_holding_samples_cannot_reenable_buttons(self):
        self.controller.start('home',1);self.wait(lambda:self.runner.process is not None)
        self.controller._telemetry({'event':'home_reached','pose':'home','sample':sample()})
        self.controller.start('zero',1);self.assertIn(b'Z',self.runner.sent)
        self.controller._telemetry({'event':'home_sample','pose':'home','phase':'holding','sample':sample()})
        self.assertEqual(self.state.control['phase'],'moving');self.assertEqual(self.state.control['pose'],'zero')
        self.controller._telemetry({'event':'pose_moving','pose':'zero','duration_s':5})
        self.controller._telemetry({'event':'zero_reached','pose':'zero','sample':sample()})
        self.assertEqual(self.state.control['phase'],'holding')
    def test_missing_browser_lease_stops_worker(self):
        with patch('control.BROWSER_TIMEOUT',.05):
            self.controller.start('home',1);self.wait(lambda:self.runner.process is not None)
            self.wait(lambda:self.controller.run is None)
        self.assertIn(b'S',self.runner.sent)
    def test_foreign_origin_and_missing_token_cannot_start_motion(self):
        access=Access(['127.0.0.1:8765'],['127.0.0.0/8'])
        self.assertFalse(access.allowed('127.0.0.1','127.0.0.1:8765','http://evil.example'))
        self.assertFalse(access.allowed('10.1.2.3','127.0.0.1:8765'))
        self.assertFalse(access.allowed('127.0.0.1','evil.example:8765'))
        with self.assertRaises(ControlError):access.identify('unknown')
        session=access.create();self.assertEqual(access.identify(session['token']),session['id'])
    def test_http_controls_require_capability_and_reject_custom_targets(self):
        access=Access([],['127.0.0.0/8']);httpd=ThreadingHTTPServer(('127.0.0.1',0),handler_for(self.state,self.controller,access));port=httpd.server_address[1]
        access.hosts.add(f'127.0.0.1:{port}');thread=threading.Thread(target=httpd.serve_forever,daemon=True);thread.start()
        def request(method,path,body=None,headers=None):
            conn=http.client.HTTPConnection('127.0.0.1',port,timeout=2);conn.request(method,path,body=body,headers=headers or {});r=conn.getresponse();data=json.loads(r.read());conn.close();return r.status,data
        try:
            status,session=request('GET','/api/control/session');self.assertEqual(status,200)
            h={'Content-Type':'application/json','X-Microduck-Control':session['token']}
            self.assertEqual(request('POST','/api/control/home','{}',{'Content-Type':'application/json'})[0],403)
            self.assertEqual(request('POST','/api/control/home','{}',{**h,'Origin':'http://other.example'})[0],403)
            self.assertEqual(request('POST','/api/control/home','{"targets":[99]}',h)[0],400)
            self.assertEqual(request('POST','/api/control/home','{}',h)[0],202)
            self.assertEqual(request('POST','/api/control/relax','{}',h)[0],202)
        finally:httpd.shutdown();httpd.server_close();thread.join()
    def test_restart_recovers_previous_run_before_accepting_new_motion(self):
        self.controller.close()
        run=self.root/'runs'/'pose-interrupted';run.mkdir()
        (self.root/'runs'/'active-run.json').write_text(json.dumps({'name':run.name}))
        self.controller=Controller(self.state,self.runner,self.root/'runs',ROOT/'kinematics/assets/alpha/robot_walk.xml')
        self.assertEqual(self.state.control['phase'],'idle')
        self.assertFalse((self.root/'runs'/'active-run.json').exists())
        self.assertTrue(json.loads((run/'result.json').read_text())['all_off'])
        self.assertIsNone(self.controller.owner)

    def test_guardian_requires_browser_and_control_progress_independently(self):
        d=Deadlines(0);d.update(ord('T'),0);d.update(ord('P'),.4)
        self.assertEqual(d.expired(.91),'control_progress_expired')
        d.update(ord('P'),2.1);self.assertEqual(d.expired(2.1),'browser_lease_expired')
        d.update(ord('B'),2.1);self.assertIsNone(d.expired(2.1))
    def test_guardian_can_only_emit_fifteen_torque_off_packets(self):
        raw=off_packets();self.assertEqual(len(raw),15*13)
        for j,id in enumerate(IDS):
            packet=raw[j*13:(j+1)*13]
            self.assertEqual(packet[:4],b'\xff\xff\xfd\0');self.assertEqual(packet[4],id)
            self.assertEqual(packet[7:11],b'\3\x40\0\0')

if __name__=='__main__':unittest.main()
