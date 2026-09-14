export function buttonState(frame,session,age,pending=false){
 const c=frame?.control||{},fresh=age<1500,own=Boolean(session&&c.owner_id===session.id);
 const ready=fresh&&frame?.motors?.length===15&&frame.motors.every(m=>m.online&&m.calibrated&&!m.hardware_error&&!m.status_error&&!m.torque);
 const canMove=Boolean(session&&c.enabled&&fresh&&!pending&&((c.phase==='idle'&&ready)||(c.phase==='holding'&&own)));
 return {canMove,canRelax:Boolean(session&&c.enabled),own};
}

export function createControls(){
 const $=id=>document.getElementById(id);
 let session=null,frame=null,received=0,pending=false,leaseInFlight=false,error='';
 const showError=message=>{error=message;$('control-error').textContent=message;$('control-error').hidden=!message;};
 const request=async(action,keepalive=false)=>{
  if(!session)throw Error('控制器尚未连接');
  const response=await fetch('/api/control/'+action,{method:'POST',headers:{'Content-Type':'application/json','X-Microduck-Control':session.token},body:'{}',keepalive,cache:'no-store'});
  const result=await response.json();if(!response.ok)throw Error(result.error||'控制请求失败');return result;
 };
 function paint(){
  const age=performance.now()-received,c=frame?.control||{},buttons=buttonState(frame,session,age,pending);
  $('home-button').disabled=!buttons.canMove;$('zero-button').disabled=!buttons.canMove;$('relax-button').disabled=!buttons.canRelax;
  $('control-message').textContent=age>=1500&&frame?'连接中断 · 等待自动卸力':c.message||(session?'等待电机状态':'连接控制器…');
  $('control-indicator').className='control-indicator '+(c.phase==='holding'?'holding':['preparing','moving','stopping','recovering'].includes(c.phase)?'busy':c.phase==='fault'?'fault':'');
  $('control-progress').textContent=c.phase==='preparing'&&Number.isFinite(c.prepared)?`${c.prepared} / 15`:c.phase==='moving'&&Number.isFinite(c.progress)?`${Math.round(c.progress*100)}%`:'';
  $('control-mode').textContent=c.mode_active?'● CONTROL / MODE 4':'● TELEMETRY';
  $('control-hint').textContent=c.phase==='preparing'?'准备约需 30 秒，随后缓慢移动到位；请继续托住躯干。可随时点击卸力取消。':c.owner_id&&!buttons.own?'另一个页面正在控制；卸力按钮仍可停止动作。':c.phase==='holding'?'已保持姿态，可切换 HOME／回零，或点击卸力。回零为页面零位，嘴部闭合。':'请托住躯干。HOME 为默认姿态；回零为标定零位（嘴闭合）。网页断联自动卸力。';
  if(!c.enabled&&frame)$('control-message').textContent=c.message||'仅预览 · 未启用控制';
 }
 async function act(action){
  if(pending&&action!=='relax')return;
  pending=action!=='relax';showError('');paint();
  try{await request(action);}catch(e){showError(e.message);}finally{pending=false;paint();}
 }
 $('home-button').addEventListener('click',()=>act('home'));
 $('zero-button').addEventListener('click',()=>act('zero'));
 $('relax-button').addEventListener('click',()=>act('relax'));
 fetch('/api/control/session',{cache:'no-store'}).then(async r=>{const data=await r.json();if(!r.ok)throw Error(data.error);session=data;paint();}).catch(e=>showError(e.message));
 setInterval(async()=>{
  const c=frame?.control;if(!session||c?.owner_id!==session.id||!['preparing','moving','holding'].includes(c.phase)||leaseInFlight)return;
  const age=performance.now()-received;
  if(age>=1500||(c.phase!=='preparing'&&!frame.motors.every(m=>m.online))){
   request('relax',true).catch(()=>{});return;
  }
  leaseInFlight=true;
  try{await request('heartbeat');}catch(e){showError('控制连接中断，正在自动卸力');}finally{leaseInFlight=false;}
 },400);
 window.addEventListener('pagehide',()=>{if(session&&frame?.control?.owner_id===session.id)request('relax',true).catch(()=>{});});
 setInterval(paint,250);
 return {update(value){frame=value;received=performance.now();paint();},disconnected(){received=0;paint();if(session&&frame?.control?.owner_id===session.id)request('relax',true).catch(()=>{});}};
}
