import * as THREE from 'three';
import {OrbitControls} from './vendor/three/OrbitControls.js';
import {STLLoader} from './vendor/three/STLLoader.js';
import {createPoseGraph} from './rig.js';
import {format,finite,motorUsable,poseAngles,quaternionEuler,torqueStatus} from './state.js';
const $=id=>document.getElementById(id);
let catalog=[],selectedId=20,lastFrame=null,lastReceived=0,poseGraph=null,selectedMeshes=[];
const rows=new Map(),thermalRows=new Map(),histories=new Map(),meshByMotor=new Map();
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
function buildThermals(){
 for(const item of catalog){
  const cell=document.createElement('div');cell.className='thermal-cell';
  const name=document.createElement('span');name.textContent=`${item.id} / ${item.label}`;
  const value=document.createElement('strong');value.textContent='—';
  cell.append(name,value);$('thermal-grid').append(cell);thermalRows.set(item.id,{cell,value});
 }
}
function paintSensors(frame,age,streamFresh){
 const healthFresh=streamFresh&&finite(frame?.health_age_ms)&&frame.health_age_ms+age<3000;
 const health=healthFresh?frame.health:null;
 const soc=health?.cpu_temp_c;
 $('soc-temp').textContent=format(soc,0);
 $('soc-state').textContent=finite(soc)?'实时':'暂无读数';
 $('soc-state').className=`sensor-state ${finite(soc)?'live':''}`;
 const throttle=health?.cpu_throttle;
 const throttleRatio=throttle?.max_level>0?throttle.level/throttle.max_level:
  throttle?.max_khz>0?1-throttle.khz/throttle.max_khz:0;
 $('soc-meter').style.width=`${Math.max(0,Math.min(100,throttleRatio*100))}%`;
 $('soc-meter').parentElement.title='处理器频率受限程度';
 $('soc-throttle').textContent=throttle&&finite(throttle.khz)?`${format(throttle.khz/1000,0)} / ${format(throttle.max_khz/1000,0)} MHz${throttle.level>0?` · 节流 ${throttle.level}/${throttle.max_level}`:''}`:'频率状态 —';
 const temperatures=frame?.motors?.filter(m=>streamFresh&&finite(m.temperature_c))||[];
 $('thermal-state').textContent=temperatures.length===15?'15 / 15 实时':temperatures.length?`${temperatures.length} / 15 有读数`:'暂无读数';
 $('thermal-state').className=`sensor-state ${temperatures.length===15?'live':''}`;
 $('thermal-max').textContent=temperatures.length?`最高 ${format(Math.max(...temperatures.map(m=>m.temperature_c)),0)} °C · ${health?.motors?.hottest||'—'}`:'最高 —';
 for(const [id,{cell,value}] of thermalRows){
  const motor=streamFresh?frame.motors.find(m=>m.id===id):null;
  value.textContent=finite(motor?.temperature_c)?`${format(motor.temperature_c,0)}°`:'—';
  cell.classList.toggle('hottest',Boolean(temperatures.length&&motor?.name===health?.motors?.hottest));
 }
 const showAttitude=(prefix,quat,live,caption)=>{
  const euler=live?quaternionEuler(quat):null;
  for(const axis of ['roll','pitch','yaw'])$(`${prefix}-${axis}`).textContent=euler?format(euler[axis],1):'—';
  $(`${prefix}-imu-state`).textContent=euler?'实时':caption;
  $(`${prefix}-imu-state`).className=`sensor-state ${euler?'live':''}`;
 };
 showAttitude('body',frame?.robot_state?.imu?.quat,streamFresh,streamFresh?'姿态不可用':'等待遥测');
 $('body-imu-age').textContent=streamFresh?`${format((frame.age_ms||0)+age,0)} ms`:'—';
 const head=frame?.head_imu,headFresh=age<1500&&head?.status==='live'&&finite(head.age_ms)&&head.age_ms+age<1500;
 showAttitude('head',head?.frame?.quat,headFresh,head?.status==='unavailable'?'未启用 / 不可用':head?.status==='stale'?'数据过期':'等待 tofd');
 $('head-imu-age').textContent=headFresh?`${format(head.age_ms+age,0)} ms`:head?.unavailable||'—';
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
 $('selected-freshness').textContent=live?`${Math.round((motor.age_ms??lastFrame?.age_ms??0)+age)} ms`:'数据不可用';
 $('raw-tick').textContent=motor?.online&&age<1500&&finite(motor.raw_tick)?String(motor.raw_tick):'—';
 $('zero-tick').textContent=finite(motor?.zero_tick)?format(motor.zero_tick,Number.isInteger(motor.zero_tick)?0:1):'—';
 const target=live&&finite(motor.target_rad)?motor.target_rad*180/Math.PI:null;
 $('target-angle').textContent=format(target);
 $('tracking-error').textContent=finite(target)?format(target-motor.angle_deg):'—';
 for(const [element,key,digits]of [['velocity','velocity_rad_s',3],['temperature','temperature_c',0],['voltage','voltage_v',1],['current','current_ma',0]])$(element).textContent=motor?.online&&age<1500?format(motor[key],digits):'—';
 const torque=torqueStatus(motor,age);
 $('torque').textContent=torque;
 $('torque').className=torque==='—'?'':torque.toLowerCase();
 const alert=$('motor-alert');let message='等待当前电机状态',fault=false;
 if(motor?.online&&age<1500){
  if(motor.hardware_error||motor.status_error){message=`硬件状态 0x${(motor.hardware_error||motor.status_error).toString(16).padStart(2,'0')}`;fault=true;}
  else message=definition.name==='mouth'?'模型闭口 = −5° · 三维铰链以闭口为 0°':'robotd 模型角 · 未提供的电机数据以 — 显示';
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
 const streamFresh=Boolean(frame?.connection==='live'&&age<1500),online=streamFresh?frame.motors.filter(m=>m.online).length:0;
 const status=online===15?'live':online?'partial':'offline';
 $('connection').className=`connection ${status}`;
 $('connection').querySelector('span').textContent=status==='live'?'LIVE / 实时':status==='partial'?`PARTIAL / ${online} 在线`:frame?.connection==='stale'?'STALE / 遥测过期':'OFFLINE / 等待 robotd';
 $('online-count').innerHTML=`${frame?online:'—'}<small>/ 15</small>`;
 $('rate').innerHTML=`${streamFresh?format(frame.read_hz,1):'—'}<small>Hz</small>`;
 const valid=streamFresh?frame.motors.filter(m=>motorUsable(m,age)):[];
 const visualAngles=Object.values(poseAngles(valid,age,frame?.source));
 $('max-error').innerHTML=`${visualAngles.length?format(Math.max(...visualAngles.map(angle=>Math.abs(angle*180/Math.PI)))):'—'}<small>°</small>`;
 for(const [id,row]of rows){
  const motor=frame?.motors.find(m=>m.id===id),good=motorUsable(motor,age);
  row.querySelector('.motor-angle').textContent=good?format(motor.angle_deg):'—';
  row.classList.toggle('missing',!good);row.classList.toggle('fault',Boolean(motor?.hardware_error));
  row.title=motor?.online?'':'暂无当前读数';
 }
 $('model-status').textContent=valid.length===15?'robotd 模型角 · 实时跟随':valid.length?`实时 ${valid.length}/15 · 缺失关节保持最后姿态`:'模型参考 / 最后姿态 · 非实时';
 $('model-status').className=`stage-status ${valid.length===15?'live':''}`;
 const mouth=frame?.motors.find(m=>m.name==='mouth');
 const marker=$('mouth-marker');if(marker){const visual=poseAngles(mouth?[mouth]:[],age,frame?.source).mouth;marker.textContent=`34 · ${finite(visual)?format(visual*180/Math.PI)+'°':'—'}`;}
 $('footer-status').textContent=streamFresh&&online?`robotd 只读订阅 · 状态年龄 ${format((frame.age_ms||0)+age,0)} ms`:frame?.connection==='offline'?'等待 robotd 连接':frame?.last_error||'等待 robotd 状态';
 paintSensors(frame,age,streamFresh);
 updateDetails();
}
function receive(frame){
 if(!Array.isArray(frame.motors))return;
 lastFrame=frame;lastReceived=performance.now();
 for(const motor of frame.motors){if(motorUsable(motor)){const h=histories.get(motor.id)||[];h.push({t:lastReceived,v:motor.angle_deg});while(h.length&&lastReceived-h[0].t>30000)h.shift();histories.set(motor.id,h);}}
 poseGraph?.setAngles(poseAngles(frame.motors,0,frame.source));paintState();
}
function connect(){
 const source=new EventSource('/api/events');
 const onTelemetry=event=>{try{receive(JSON.parse(event.data));}catch(error){console.error('Telemetry payload',error);}};
 source.addEventListener('telemetry',onTelemetry);
 source.addEventListener('message',onTelemetry);
 source.onerror=()=>{lastReceived=0;paintState();};
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
    mouthLabel=document.createElement('button');mouthLabel.id='mouth-marker';mouthLabel.className='mouth-marker';mouthLabel.type='button';mouthLabel.textContent='34 · —';mouthLabel.title='嘴部视觉铰链角；robotd 模型闭口为 −5°';mouthLabel.addEventListener('click',()=>selectMotor(34));container.append(mouthLabel);
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
  await populate(definition.root);$('model-loading').remove();highlight();if(lastFrame)poseGraph.setAngles(poseAngles(lastFrame.motors,performance.now()-lastReceived,lastFrame.source));
 }catch(error){const loading=$('model-loading');if(loading){loading.classList.add('error');loading.textContent='三维模型无法加载，位置读数仍可使用。';}console.error(error);}
}
fetch('/assets/model.json').then(r=>{if(!r.ok)throw Error('model data missing');return r.json();}).then(definition=>{catalog=definition.motors;buildList();buildThermals();connect();buildModel(definition);setInterval(paintState,250);}).catch(error=>{$('model-loading').textContent='模型定义加载失败';console.error(error);});
