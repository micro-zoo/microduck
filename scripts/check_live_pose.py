"""Interactive native pose/guardian checks; imported only by the isolated PTY test."""
import json,os,select,signal,struct,subprocess,sys,tempfile,threading,time
from pathlib import Path

def check_live_pose(args):
    from check_calibrated_control_path import Bus,source_array,assert_test_isolation
    assert_test_isolation()
    ids=source_array(args.model_source,'JOINT_IDS',int)
    home=source_array(args.model_source,'DEFAULT_POSITION',float)
    cal=json.loads(args.calibration.read_text());checks={}
    guardian_script=Path(__file__).parent/'live_twin/guardian.py'
    for scenario in ('retarget','browser_loss','control_stall','producer_death'):
        with tempfile.TemporaryDirectory(prefix='live-pose-') as directory:
            root=Path(directory);bus=Bus(cal,ids);native=None;guardian=None;fds=[]
            beat_stop=threading.Event();beater=None
            try:
                with bus.lock:
                    for j,id in enumerate(ids):
                        b=bus.regs[id];raw=round(bus.cal[id]['zero_tick']+home[j]*4096/(2*3.141592653589793))
                        b[36:38]=struct.pack('<H',885);b[38:40]=struct.pack('<H',1750)
                        b[84:86]=struct.pack('<H',400);b[100:102]=struct.pack('<H',885)
                        b[116:120]=b[132:136]=struct.pack('<i',raw)
                    before={id:bytes(b) for id,b in bus.regs.items()}
                params=root/'params.toml';params.write_text(f'[bus]\nport="{bus.path}"\ncalibration="{args.calibration}"\n[audio]\nenabled=false\n')
                telemetry=root/'telemetry.jsonl'
                br,bw=os.pipe();ar,aw=os.pipe();cr,cw=os.pipe();fds=[br,bw,ar,aw,cr,cw]
                guardian=subprocess.Popen([sys.executable,str(guardian_script),'--serial-fd',str(bus.slave),
                    '--beat-fd',str(br),'--ack-fd',str(aw),'--result',str(root/'guardian.json')],pass_fds=(bus.slave,br,aw))
                def beats():
                    while not beat_stop.is_set():
                        os.write(bw,b'B');beat_stop.wait(.1)
                beater=threading.Thread(target=beats,daemon=True);beater.start()
                env={**os.environ,'DUCK_HOME_WATCHDOG_FD':str(bw),'DUCK_POSE_CONTROL_FD':str(cr),'DUCK_RUNTIME_DIR':str(root/'runtime')}
                with (root/'native.log').open('w') as log:
                    native=subprocess.Popen([str(args.robotd),'--params',str(params),'--socket',str(root/'robotd.sock'),
                        'init','--guarded','--interactive','--higher-effort','--pose','home','--duration','30s','--telemetry',str(telemetry)],
                        env=env,pass_fds=(bw,cr),stdout=log,stderr=subprocess.STDOUT)
                os.write(bw,b'A'+struct.pack('<I',native.pid))
                assert select.select([ar],[],[],2)[0] and os.read(ar,1)==b'1'
                os.write(cw,b'G')
                recorded=[];offset=0;pending=''
                def events():
                    nonlocal offset,pending
                    if not telemetry.exists():return recorded
                    with telemetry.open() as stream:
                        stream.seek(offset);pending+=stream.read();offset=stream.tell()
                    lines=pending.split('\n');pending=lines.pop()
                    recorded.extend(json.loads(line) for line in lines if line)
                    return recorded
                def wait(predicate,timeout=15):
                    deadline=time.monotonic()+timeout
                    while not predicate():
                        if time.monotonic()>deadline:raise AssertionError((scenario,bus.errors,(root/'native.log').read_text(),events()[-1:]))
                        time.sleep(.02)
                wait(lambda:any(e['event']=='home_reached' for e in events()))
                time.sleep(.2)
                assert native.poll() is None and all(b[64]==1 for b in bus.regs.values())
                if scenario=='retarget':
                    os.write(cw,b'Z')
                    wait(lambda:any(e['event']=='zero_reached' for e in events()))
                    assert native.poll() is None and all(b[64]==1 for b in bus.regs.values())
                    zero=next(e['sample']['positions'] for e in events() if e['event']=='zero_reached')
                    assert all(abs(q-(-5*3.141592653589793/180 if id==34 else 0))<.002 for id,q in zip(ids,zero))
                    os.write(cw,b'S');native.wait(timeout=3)
                    assert native.returncode==0
                    assert events()[-1]['all_off'] and events()[-1]['settings_restored']
                    for id,b in bus.regs.items():
                        assert b[:64]==before[id][:64] and b[64]==0
                        for address,size in ((80,6),(88,4),(98,1),(100,2),(108,8)):
                            assert b[address:address+size]==before[id][address:address+size]
                    os.write(bw,b'X');guardian.wait(timeout=2)
                    checks['home_and_zero_hold_until_relax_then_restore']=True
                else:
                    triggered=time.monotonic()
                    if scenario=='browser_loss':beat_stop.set();beater.join()
                    elif scenario=='control_stall':os.kill(native.pid,signal.SIGSTOP)
                    else:native.kill()
                    guardian.wait(timeout=3);native.wait(timeout=2)
                    wait(lambda:all(b[64]==0 for b in bus.regs.values()),1)
                    report=json.loads((root/'guardian.json').read_text())
                    expected='browser_lease_expired' if scenario=='browser_loss' else 'control_progress_expired'
                    assert report['reason']==expected and report['off_bytes_sent']==195,report
                    assert time.monotonic()-triggered<(2.6 if scenario=='browser_loss' else 1.1)
                    assert not any(w['address']==116 and w['t']>triggered+.6 for w in bus.writes) if scenario!='browser_loss' else True
                    checks[scenario+'_independent_off']=True
                assert not bus.unsafe_enables,bus.unsafe_enables
                (args.output/('live-'+scenario+'.jsonl')).write_text(telemetry.read_text())
                (args.output/('live-'+scenario+'-writes.json')).write_text(json.dumps(bus.writes))
                if (root/'guardian.json').exists():(args.output/('live-'+scenario+'-guardian.json')).write_text((root/'guardian.json').read_text())
            finally:
                for name in ('native.log','telemetry.jsonl','guardian.json'):
                    if (root/name).exists():(args.output/('live-'+scenario+'-'+name)).write_text((root/name).read_text())
                # Stop registration before reaping a producer on a failed test.
                if guardian is not None and guardian.poll() is None:guardian.kill();guardian.wait()
                if native is not None and native.poll() is None:native.kill();native.wait()
                beat_stop.set()
                if beater:beater.join(1)
                for fd in fds:os.close(fd)
                bus.close()
    return checks
