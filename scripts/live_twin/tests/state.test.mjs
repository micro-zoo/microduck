import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from '../dist/vendor/three/three.module.js';
import {createPoseGraph} from '../dist/rig.js';
import {poseAngles,quaternionEuler,trunkOrientation,torqueStatus} from '../dist/state.js';

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

test('IMU quaternion is reported as roll pitch yaw without invented orientation',()=>{
 const roll=quaternionEuler([Math.SQRT1_2,Math.SQRT1_2,0,0]);
 assert.ok(Math.abs(roll.roll-90)<1e-9);
 assert.ok(Math.abs(roll.pitch)<1e-9);
 assert.ok(Math.abs(roll.yaw)<1e-9);
 assert.equal(quaternionEuler([0,0,0,0]),null);
 assert.equal(quaternionEuler(null),null);
});

test('trunk follows measured tilt with only the initial yaw anchored',()=>{
 const yaw90=[Math.SQRT1_2,0,0,Math.SQRT1_2];
 const first=trunkOrientation(yaw90);
 assert.ok(Math.abs(quaternionEuler(first.quat).yaw)<1e-9);
 const roll30=[Math.cos(Math.PI/12),Math.sin(Math.PI/12),0,0];
 const raw=[
  yaw90[0]*roll30[0],
  yaw90[0]*roll30[1],
  yaw90[3]*roll30[1],
  yaw90[3]*roll30[0],
 ];
 const displayed=quaternionEuler(trunkOrientation(raw,first.referenceYaw).quat);
 assert.ok(Math.abs(displayed.roll-30)<1e-9);
 assert.ok(Math.abs(displayed.pitch)<1e-9);
 assert.ok(Math.abs(displayed.yaw)<1e-9);
 assert.equal(trunkOrientation([0,0,0,0]),null);
});

test('trunk IMU rotation carries the whole kinematic tree',()=>{
 const body=(name,position,children=[])=>({name,position,quaternion:[1,0,0,0],geoms:[],children});
 const graph=createPoseGraph({root:body('trunk_base',[0,0,0],[body('head',[0,0,1])])});
 graph.setTrunkOrientation([Math.SQRT1_2,Math.SQRT1_2,0,0]);
 const position=graph.bodies.get('head').getWorldPosition(new THREE.Vector3());
 assert.ok(Math.abs(position.x)<1e-9);
 assert.ok(Math.abs(position.y+1)<1e-9);
 assert.ok(Math.abs(position.z)<1e-9);
});
