export const finite=value=>typeof value==='number'&&Number.isFinite(value);
export function format(value,digits=2){return finite(value)?value.toFixed(digits):'—';}
export function motorUsable(motor,streamAge=0){return Boolean(motor?.online&&motor.calibrated&&finite(motor.angle_rad)&&streamAge<1500);}
export function poseAngles(motors,streamAge=0,source=null){
 // The visual jaw is closed at zero; robotd's model closes it at -5 degrees.
 // Keep telemetry in model coordinates and adjust only the visual hinge.
 return Object.fromEntries(motors.filter(m=>motorUsable(m,streamAge)).map(m=>[m.name,m.angle_rad+(source==='robotd-ipc'&&m.name==='mouth'?5*Math.PI/180:0)]));
}
export function torqueStatus(motor,streamAge=0){
 if(!motor?.online||streamAge>=1500||motor.torque_known===false)return '—';
 if(motor.torque===true||motor.torque===1)return 'ON';
 if(motor.torque===false||motor.torque===0)return 'OFF';
 return '—';
}
export function quaternionEuler(quat){
 if(!Array.isArray(quat)||quat.length!==4||!quat.every(finite))return null;
 const norm=Math.hypot(...quat);
 if(norm<.5||norm>1.5)return null;
 const [w,x,y,z]=quat.map(v=>v/norm),degree=180/Math.PI;
 return {
  roll:Math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))*degree,
  pitch:Math.asin(Math.max(-1,Math.min(1,2*(w*y-z*x))))*degree,
  yaw:Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))*degree,
 };
}
