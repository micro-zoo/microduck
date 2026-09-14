import * as THREE from 'three';
import {OrbitControls} from './vendor/three/OrbitControls.js';
import {STLLoader} from './vendor/three/STLLoader.js';
import {createPoseGraph} from './rig.js';
import {format,finite,motorUsable,poseAngles,samplingPaused} from './state.js';
import {createControls} from './controls.js';
const controlUI=createControls();
const $=id=>document.getElementById(id);
let catalog=[],selectedId=20,lastFrame=null,lastReceived=0,poseGraph=null,selectedMeshes=[];
const rows=new Map(),histories=new Map(),meshByMotor=new Map();
const groupNames={left:'01 / LEFT LEG',head:'02 / HEAD + MOUTH',right:'03 / RIGHT LEG'};
function selectMotor(id){selectedId=id;for(const [key,row]of rows)row.classList.toggle('selected',key===id);highlight();updateDetails();}
function buildList(){
 for(const group of ['left','head','right']){
  const section=document.createElement('section');section.className='joint-group';
  const title=document.createElement('div');title.className='group-label';title.textContent=groupNames[group];section.append(title);
  for(const item of catalog.filter(m=>m.group===group)){
   const row=document.createElement('button');row.type='button';row.className='joint-row missing';row.setAttribute('aria-label',`${item.label}，ID ${item.id}`);
   const id=document.createElement('span');id.className='motor-id';id.textContent=String(item.id);
   const label=document.createElement('span');label.className='motor-label';label.textContent=item.label;
   const value=document.createElement('span');value.className='motor-angle';value.textContent='—';
   row.append(id,label,value);row.addEventListener('click',()=>selectMotor(item.id));section.append(row);rows.set(item.id,row);
  }
  $('joint-list').append(section);
 }
 selectMotor(selectedId);
}
function highlight(){
 for(const mesh of selectedMeshes){mesh.material.color.copy(mesh.userData.baseColor);mesh.material.emissive?.setHex(0x000000);}
 selectedMeshes=meshByMotor.get(selectedId)||[];
 for(const mesh of selectedMeshes){mesh.material.color.copy(mesh.userData.baseColor).lerp(new THREE.Color('#bce44b'),.4);mesh.material.emissive?.setHex(0x172000);}
}
function updateDetails(){
 const definition=catalog.find(m=>m.id===selectedId);if(!definition)return;
 const motor=lastFrame?.motors?.find(m=>m.id===selectedId),age=performance.now()-lastReceived;
 const live=motorUsable(motor,age);
 $('selected-id').textContent=String(selectedId);$('selected-label').textContent=definition.label;$('selected-name').textContent=definition.name;
 $('selected-angle').textContent=live?format(motor.angle_deg):'—';
 $('selected-freshness').textContent=live?`${Math.round((motor.age_ms||0)+age)} ms`:'数据不可用';
 $('raw-tick').textContent=motor?.online&&age<1500?String(motor.raw_tick):'—';
 $('zero-tick').textContent=finite(motor?.zero_tick)?format(motor.zero_tick,Number.isInteger(motor.zero_tick)?0:1):'—';
 for(const [element,key,digits]of [['velocity','velocity_rad_s',3],['temperature','temperature_c',0],['voltage','voltage_v',1],['current','current_ma',0]])$(element).textContent=motor?.online&&age<1500?format(motor[key],digits):'—';
 $('torque').textContent=motor?.online&&age<1500?(motor.torque?'ON':'OFF'):'—';
 $('torque').className=motor?.online&&age<1500?(motor.torque?'on':'off'):'';
 const alert=$('motor-alert');let message='等待当前电机状态',fault=false;
 if(motor?.online&&age<1500){
  if(motor.hardware_error||motor.status_error){message=`硬件状态 0x${(motor.hardware_error||motor.status_error).toString(16).padStart(2,'0')}`;fault=true;}
  else if(motor.calibration_mismatch){message='电机设置已改变，请重新核对标定';fault=true;}
  else if(motor.single_turn_range_risk){message=`单圈余量 +${format(motor.positive_margin_deg,1)}° / −${format(motor.negative_margin_deg,1)}°，未覆盖模型范围`;fault=true;}
  else if(definition.name==='mouth')message='闭口 = 0° · 嘴部近似铰链，电机角按 1:1 显示';
  else message=motor.calibrated?'读数正常 · 已应用工装零位':'读数可用，尚无零位标定';
 }
 alert.textContent=message;alert.classList.toggle('fault',fault);
 const history=histories.get(selectedId)||[],now=performance.now();
 const points=history.filter(p=>now-p.t<30000);
 const amplitude=Math.max(.5,...points.map(p=>Math.abs(p.v)))*1.1;
 $('trace-line').setAttribute('points',points.map(p=>`${Math.max(0,260-(now-p.t)/30000*260).toFixed(1)},${(48-p.v/amplitude*42).toFixed(1)}`).join(' '));
 $('trace-range').textContent=points.length?`±${amplitude.toFixed(2)}°`:'等待采样';
}
function paintState(){
 const age=performance.now()-lastReceived,frame=lastFrame;
 const streamFresh=Boolean(frame&&age<1500),online=streamFresh?frame.motors.filter(m=>m.online).length:0;
 const paused=samplingPaused(frame,age);
 const status=paused?'partial':online===15?'live':online?'partial':'offline';
 $('connection').className=`connection ${status}`;
 $('connection').querySelector('span').textContent=paused?(frame.control.phase==='preparing'?'PREPARING / 准备电机':'RESTORING / 恢复电机'):status==='live'?'LIVE / 实时':status==='partial'?`PARTIAL / ${online} 在线`:frame?'OFFLINE / 遥测中断':'等待遥测';
 $('online-count').innerHTML=paused?'交接中':`${frame?online:'—'}<small>/ 15</small>`;
 $('rate').innerHTML=`${streamFresh?format(frame.read_hz,1):'—'}<small>Hz</small>`;
 const valid=streamFresh?frame.motors.filter(m=>motorUsable(m,age)):[];
 $('max-error').innerHTML=`${valid.length?format(Math.max(...valid.map(m=>Math.abs(m.angle_deg)))):'—'}<small>°</small>`;
 if(frame?.calibration){
  const date=new Date(frame.calibration.captured_at);
  $('cal-time').textContent=`${frame.calibration.motor_count} 路已标定 · ${Number.isNaN(date.valueOf())?'时间未同步':date.toLocaleString('zh-CN',{month:'2-digit',day:'2-digit',hour:'2-digit',minute:'2-digit'})}`;
 }
 for(const [id,row]of rows){
  const motor=frame?.motors.find(m=>m.id===id),good=motorUsable(motor,age);
  row.querySelector('.motor-angle').textContent=good?format(motor.angle_deg):'—';
  row.classList.toggle('missing',!good);row.classList.toggle('fault',Boolean(motor?.hardware_error||motor?.single_turn_range_risk));
  row.title=motor?.single_turn_range_risk?'零位已采集；单圈运动范围待处理':motor?.online?'':'暂无当前读数';
 }
 $('model-status').textContent=paused?'控制准备／恢复中 · 未更新的关节保留最后姿态':valid.length===15?'已补偿位置 · 实时跟随':valid.length?`实时 ${valid.length}/15 · 缺失关节保持最后姿态`:'模型参考 / 最后姿态 · 非实时';
 $('model-status').className=`stage-status ${valid.length===15?'live':''}`;
 const risks=frame?.motors.filter(m=>m.single_turn_range_risk).length||0;
 const mouth=frame?.motors.find(m=>m.name==='mouth');
 const marker=$('mouth-marker');if(marker)marker.textContent=`34 · ${motorUsable(mouth,age)?format(mouth.angle_deg)+'°':'—'}`;
 $('footer-status').textContent=paused?'串口由控制程序使用，完成后自动恢复实时采样':frame?.calibration_error?'标定记录不可用':streamFresh&&online?`${format(frame.cycle_ms,1)} ms / 读取周期${risks?` · ${risks} 路单圈行程待检查`:''}`:frame?.last_error||'等待控制板连接';
 updateDetails();
}
function receive(frame){
 if(!Array.isArray(frame.motors))return;
 lastFrame=frame;lastReceived=performance.now();
 controlUI.update(frame);
 for(const motor of frame.motors){if(motorUsable(motor)){const h=histories.get(motor.id)||[];h.push({t:lastReceived,v:motor.angle_deg});while(h.length&&lastReceived-h[0].t>30000)h.shift();histories.set(motor.id,h);}}
 poseGraph?.setAngles(poseAngles(frame.motors));paintState();
}
function connect(){
 const source=new EventSource('/api/events');
 source.addEventListener('telemetry',event=>{try{receive(JSON.parse(event.data));}catch(error){console.error('Telemetry payload',error);}});
 source.onerror=()=>{lastReceived=0;paintState();controlUI.disconnected();};
 fetch('/api/state',{cache:'no-store'}).then(r=>r.json()).then(receive).catch(()=>paintState());
}
async function buildModel(definition){
 const container=$('viewport');
 try{
  const renderer=new THREE.WebGLRenderer({antialias:true,alpha:true});renderer.setPixelRatio(Math.min(devicePixelRatio||1,1.75));renderer.outputColorSpace=THREE.SRGBColorSpace;
  container.append(renderer.domElement);
  const scene=new THREE.Scene();scene.background=new THREE.Color('#f2f4e9');
  const camera=new THREE.PerspectiveCamera(38,1,.001,10);camera.up.set(0,0,1);
  const controls=new OrbitControls(camera,renderer.domElement);controls.target.set(0,0,.025);controls.enableDamping=true;controls.dampingFactor=.09;controls.minDistance=.27;controls.maxDistance=1;controls.maxPolarAngle=Math.PI*.8;
  const views={orbit:[.40,-.43,.27],front:[.52,0,.035],side:[0,-.52,.04]};
  const setView=name=>{camera.position.fromArray(views[name]);camera.lookAt(controls.target);controls.update();};setView('orbit');
  for(const button of document.querySelectorAll('[data-view]'))button.addEventListener('click',()=>{for(const b of document.querySelectorAll('[data-view]'))b.classList.toggle('active',b===button);setView(button.dataset.view);});
  scene.add(new THREE.HemisphereLight(0xffffff,0x839079,2.1));
  const key=new THREE.DirectionalLight(0xffffff,2.4);key.position.set(.5,-.6,.8);scene.add(key);
  const grid=new THREE.GridHelper(.6,24,0xaab29d,0xcfd5c4);grid.rotation.x=Math.PI/2;grid.position.z=-.108;grid.material.transparent=true;grid.material.opacity=.65;scene.add(grid);
  const axes=new THREE.AxesHelper(.036);axes.position.set(-.1,-.09,-.105);scene.add(axes);
  poseGraph=createPoseGraph(definition);scene.add(poseGraph.root);
  let mouthAnchor=null,mouthLabel=null;
  function locateMouth(body){for(const site of body.sites||[])if(site.name==='mouth_tip'){
    mouthAnchor=new THREE.Object3D();mouthAnchor.position.fromArray(site.position);poseGraph.geometryParents.get(body.name).add(mouthAnchor);
    mouthLabel=document.createElement('button');mouthLabel.id='mouth-marker';mouthLabel.className='mouth-marker';mouthLabel.type='button';mouthLabel.textContent='34 · —';mouthLabel.title='嘴部实时角度；近似铰链，传动比例待核对';mouthLabel.addEventListener('click',()=>selectMotor(34));container.append(mouthLabel);
  }for(const child of body.children)locateMouth(child);}locateMouth(definition.root);
  const projectedMouth=new THREE.Vector3();
  const loader=new STLLoader(),cache=new Map(),pickable=[];let loaded=0;
  function geometry(file){if(!cache.has(file))cache.set(file,loader.loadAsync(`/assets/meshes/${file}`));return cache.get(file);}
  async function populate(body){
   const parent=poseGraph.geometryParents.get(body.name),owner=body.joint?.id;
   await Promise.all(body.geoms.map(async geom=>{
    const shape=await geometry(geom.mesh);
    const color=new THREE.Color().setRGB(...geom.color.slice(0,3));
    const material=new THREE.MeshStandardMaterial({color,roughness:.85,metalness:.02,flatShading:true});
    const mesh=new THREE.Mesh(shape,material);mesh.position.fromArray(geom.position);const[w,x,y,z]=geom.quaternion;mesh.quaternion.set(x,y,z,w).normalize();
    mesh.userData={motorId:owner,baseColor:color.clone()};parent.add(mesh);pickable.push(mesh);
    if(owner){if(!meshByMotor.has(owner))meshByMotor.set(owner,[]);meshByMotor.get(owner).push(mesh);}
    loaded++;$('model-loading').textContent=`载入结构 ${loaded} 件`;
   }));
   await Promise.all(body.children.map(populate));
  }
  const resized=()=>{const width=container.clientWidth,height=container.clientHeight;if(!width||!height)return;camera.aspect=width/height;camera.updateProjectionMatrix();renderer.setSize(width,height,false);};new ResizeObserver(resized).observe(container);resized();
  const raycaster=new THREE.Raycaster(),pointer=new THREE.Vector2();let down=null;
  renderer.domElement.addEventListener('pointerdown',e=>{down=[e.clientX,e.clientY];});
  renderer.domElement.addEventListener('pointerup',e=>{if(!down||Math.hypot(e.clientX-down[0],e.clientY-down[1])>5)return;const rect=renderer.domElement.getBoundingClientRect();pointer.set((e.clientX-rect.left)/rect.width*2-1,-(e.clientY-rect.top)/rect.height*2+1);raycaster.setFromCamera(pointer,camera);const hit=raycaster.intersectObjects(pickable,false).find(h=>h.object.userData.motorId);if(hit)selectMotor(hit.object.userData.motorId);});
  renderer.setAnimationLoop(()=>{controls.update();renderer.render(scene,camera);
    if(mouthAnchor&&mouthLabel){mouthAnchor.getWorldPosition(projectedMouth);projectedMouth.project(camera);mouthLabel.style.display=projectedMouth.z<1&&projectedMouth.z>-1?'block':'none';mouthLabel.style.left=`${(projectedMouth.x*.5+.5)*container.clientWidth}px`;mouthLabel.style.top=`${(-projectedMouth.y*.5+.5)*container.clientHeight}px`;}
  });
  await populate(definition.root);$('model-loading').remove();highlight();if(lastFrame)poseGraph.setAngles(poseAngles(lastFrame.motors));
 }catch(error){const loading=$('model-loading');if(loading){loading.classList.add('error');loading.textContent='三维模型无法加载，位置读数仍可使用。';}console.error(error);}
}
fetch('/assets/model.json').then(r=>{if(!r.ok)throw Error('model data missing');return r.json();}).then(definition=>{catalog=definition.motors;buildList();connect();buildModel(definition);setInterval(paintState,250);}).catch(error=>{$('model-loading').textContent='模型定义加载失败';console.error(error);});
