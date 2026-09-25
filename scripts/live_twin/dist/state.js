export const finite=value=>typeof value==='number'&&Number.isFinite(value);
export function localAngle(raw,zero){return ((((raw-zero+2048)%4096)+4096)%4096-2048)*2*Math.PI/4096;}
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

export function samplingPaused(frame,age=0){return age<1500&&['preparing','stopping','recovering'].includes(frame?.control?.phase);}
