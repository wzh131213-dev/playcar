'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const {RigidBodyVehicle,Vector3:V}=require('../physics/rigidbody-vehicle-engine.js');
function plane(){
 const a={x:-100,y:0,z:-100},b={x:-100,y:0,z:100},c={x:100,y:0,z:100},d={x:100,y:0,z:-100};
 const tris=[[a,b,c],[a,c,d]];
 return {closestPoint(p,max=Infinity){let best=null;for(const tri of tris){const y=0,point={x:V.clamp(p.x,-100,100),y,z:V.clamp(p.z,-100,100)},distance=Math.abs(p.y-y);if(distance<=max&&(!best||distance<best.distance))best={point,normal:{x:0,y:1,z:0},distance,signedDistance:p.y-y};}return best;}};
}
function make(){return new RigidBodyVehicle({surfaceMesh:plane(),groundPlaneY:0,position:{x:0,y:.3,z:0}});}
test('fixed simulation step is exactly 1/120 second',()=>{const e=make();assert.equal(e.c.fixedDt,1/120);const r=e.advance(1/60,{throttle:1});assert.equal(r.steps,2);});
test('four independent wheel contacts create suspension loads',()=>{const e=make();e.advance(1/120,{});assert.equal(e.telemetry.contacts,4);assert.equal(e.telemetry.normalLoads.length,4);assert.ok(e.telemetry.normalLoads.every(Number.isFinite));assert.ok(e.telemetry.normalLoads.some(x=>x>0),'compressed springs should generate support load');});
test('engine torque, gears and wheel angular velocity respond to throttle',()=>{const e=make();for(let i=0;i<120;i++)e.advance(1/120,{throttle:1});assert.ok(e.state.wheelOmega.some(x=>x>0));assert.ok(e.telemetry.rpm>=e.c.engine.idleRpm);});
test('combined tire force is clamped by mu times normal load',()=>{const e=make();e.advance(1/120,{throttle:1,steer:1});for(const f of e.telemetry.forces){assert.ok(Math.hypot(f.Fx,f.Fy)<=e.c.tire.mu*f.normalLoad+1e-6);}});
test('semi-implicit integration keeps quaternion normalized and state finite',()=>{const e=make();for(let i=0;i<240;i++)e.advance(1/120,{throttle:.5,steer:.2});const q=e.state.orientation;assert.ok(Math.abs(Math.hypot(q.x,q.y,q.z,q.w)-1)<1e-8);for(const v of Object.values(e.state.position))assert.ok(Number.isFinite(v));});
test('render interpolation returns a normalized orientation',()=>{const e=make();e.advance(.01,{throttle:1});const p=e.interpolatedState();assert.ok(Math.abs(Math.hypot(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)-1)<1e-8);});

test('front-wheel-drive differential layout drives only the front axle',()=>{const e=new RigidBodyVehicle({surfaceMesh:plane(),groundPlaneY:0,position:{x:0,y:.3,z:0},config:{differential:{type:'fwd',limitedSlip:2.2}}});for(let i=0;i<120;i++)e.advance(1/120,{throttle:1});assert.ok(e.state.wheelOmega[0]>0&&e.state.wheelOmega[1]>0);assert.deepEqual(e.telemetry.drivenWheels,[0,1]);assert.equal(e.telemetry.differential,'fwd');});
test('anti-roll and driver-aid telemetry remains finite and exposed',()=>{const e=make();e.advance(1/120,{throttle:.5,steer:.3});assert.deepEqual(e.telemetry.antiRoll,{front:e.c.antiRoll.front,rear:e.c.antiRoll.rear});assert.equal(e.telemetry.differential,'open');assert.equal(typeof e.telemetry.absActive,'boolean');assert.equal(typeof e.telemetry.tcsActive,'boolean');for(const n of e.telemetry.normalLoads)assert.ok(Number.isFinite(n));});

test('limited-slip differential biases torque toward the slower wheel',()=>{const e=new RigidBodyVehicle({surfaceMesh:plane(),groundPlaneY:0,position:{x:0,y:.3,z:0},config:{differential:{type:'lsd',limitedSlip:3},aids:{abs:false,tcs:false,absSlip:.18,tcsSlip:.22}}});e.state.wheelOmega[2]=0;e.state.wheelOmega[3]=24;e.advance(1/120,{throttle:1});const rear=e.telemetry.forces.filter(f=>f.wheel>=2);assert.equal(rear.length,2);assert.ok(rear[0].driveTorque>rear[1].driveTorque);assert.ok(Math.abs(rear[0].driveTorque+rear[1].driveTorque)>0);});
