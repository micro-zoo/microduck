export const finite=value=>typeof value==='number'&&Number.isFinite(value);
export function localAngle(raw,zero){return ((((raw-zero+2048)%4096)+4096)%4096-2048)*2*Math.PI/4096;}
export function format(value,digits=2){return finite(value)?value.toFixed(digits):'—';}
export function motorUsable(motor,streamAge=0){return Boolean(motor?.online&&motor.calibrated&&finite(motor.angle_rad)&&streamAge<1500);}
export function poseAngles(motors,streamAge=0){return Object.fromEntries(motors.filter(m=>motorUsable(m,streamAge)).map(m=>[m.name,m.angle_rad]));}
