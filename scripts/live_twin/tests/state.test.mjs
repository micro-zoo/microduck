import test from 'node:test';
import assert from 'node:assert/strict';
import {poseAngles,torqueStatus} from '../dist/state.js';

test('IPC mouth conversion affects the visual hinge only',()=>{
 const motors=[
  {name:'mouth',angle_rad:-5*Math.PI/180,online:true,calibrated:true},
  {name:'head_yaw',angle_rad:.2,online:true,calibrated:true},
 ];
 const visual=poseAngles(motors,0,'robotd-ipc');
 assert.equal(visual.mouth,0);
 assert.equal(visual.head_yaw,.2);
 assert.equal(motors[0].angle_rad,-5*Math.PI/180);
 assert.equal(poseAngles(motors).mouth,-5*Math.PI/180);
 assert.deepEqual(poseAngles(motors,2000,'robotd-ipc'),{});
});

test('unknown or stale torque never appears as OFF',()=>{
 assert.equal(torqueStatus({online:true,torque:null,torque_known:false}),'—');
 assert.equal(torqueStatus({online:true}),'—');
 assert.equal(torqueStatus({online:true,torque:0}),'OFF');
 assert.equal(torqueStatus({online:true,torque:1}),'ON');
 assert.equal(torqueStatus({online:true,torque:0},2000),'—');
});
