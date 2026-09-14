import test from 'node:test';
import assert from 'node:assert/strict';
import {buttonState} from '../dist/controls.js';
const motors=Array.from({length:15},()=>({online:true,calibrated:true,torque:0,hardware_error:0,status_error:0}));
test('movement requires fresh complete telemetry; unload remains available',()=>{
 const frame={motors,control:{enabled:true,phase:'idle',owner_id:null}};
 assert.equal(buttonState(frame,{id:1},0).canMove,true);
 assert.equal(buttonState(frame,{id:1},2000).canMove,false);
 assert.equal(buttonState(frame,{id:1},2000).canRelax,true);
 assert.equal(buttonState({...frame,motors:motors.slice(1)},{id:1},0).canMove,false);
});
test('only the holding owner can change poses; busy actions cannot be double submitted',()=>{
 const frame={motors,control:{enabled:true,phase:'holding',owner_id:1}};
 assert.equal(buttonState(frame,{id:1},0).canMove,true);
 assert.equal(buttonState(frame,{id:2},0).canMove,false);
 assert.equal(buttonState(frame,{id:1},0,true).canMove,false);
 assert.equal(buttonState({...frame,control:{...frame.control,phase:'moving'}},{id:1},0).canMove,false);
});

import {samplingPaused} from '../dist/state.js';
test('preparation is a UART handoff, while a stale connection remains disconnected',()=>{
 assert.equal(samplingPaused({control:{phase:'preparing'}},0),true);
 assert.equal(samplingPaused({control:{phase:'stopping'}},0),true);
 assert.equal(samplingPaused({control:{phase:'preparing'}},2000),false);
 assert.equal(samplingPaused({control:{phase:'moving'}},0),false);
});
