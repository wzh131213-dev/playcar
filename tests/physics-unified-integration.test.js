'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
const bundle=fs.readFileSync(path.join(__dirname,'../physics/playcar-physics-all-in-one.js'),'utf8');

test('single runtime embeds the rigid-body solver before scene startup',()=>{
  assert.match(bundle,/root\.PlayCarRigidBody=\{RigidBodyVehicle/);
  assert.match(bundle,/class UnifiedPlayCarPhysicsEngine extends LegacyPlayCarPhysicsEngine/);
  assert.match(bundle,/api\.PlayCarPhysicsEngine=UnifiedPlayCarPhysicsEngine/);
});
test('unified runtime preserves legacy compatibility and weather grip mapping',()=>{
  assert.match(bundle,/LegacyPlayCarPhysicsEngine\.prototype\._applyStaticCollisions\.call\(this\)/);
  assert.match(bundle,/setWeather\(weather=\{\}\)/);
  assert.match(bundle,/this\.config\.tireGrip\*\(1-\.42\*this\.weather\.wetness\)/);
  assert.match(bundle,/debugSnapshot\(\).*solver:'rigid-body-2\.0\.0'/);
});
test('render loop consumes four wheel angular velocities and front steering',()=>{
  assert.match(bundle,/vehicle\.getWheelState\(\)/);
  assert.match(bundle,/wheelState\.omega\[ix\]/);
  assert.match(bundle,/w\.pivot\.rotation\.y=w\.z<0\?wheelState\.steer:0/);
});
test('reset sends the spawn position through the unified engine API',()=>{
  assert.match(bundle,/vehicle\.reset\(\{x:0,y:\.08,z:34\}\)/);
});

test('all-in-one browser bundle parses as valid JavaScript',()=>{assert.doesNotThrow(()=>new vm.Script(bundle));});

test('runtime triangle mesh preserves per-triangle friction material',()=>{
  const source=bundle.slice(0,bundle.indexOf('(async function startPlayCar3DScene'));
  const context={console};vm.runInNewContext(source,context);
  const Mesh=context.PlayCarPhysicsEngine.TriangleMesh;
  const a={x:-2,y:0,z:-2},b={x:2,y:0,z:-2},c={x:0,y:0,z:2};
  const mesh=Mesh.fromTriangles([[a,b,c,.27]]);
  const hit=mesh.closestPoint({x:0,y:1,z:0},5,{x:0,y:1,z:0});
  assert.equal(hit.friction,.27);
});
test('unified vehicle exposes limited-slip differential tuning',()=>{
  const source=bundle.slice(0,bundle.indexOf('(async function startPlayCar3DScene'));
  const context={console};vm.runInNewContext(source,context);
  const Engine=context.PlayCarPhysicsEngine.PlayCarPhysicsEngine;
  const engine=new Engine({groundPlaneY:0,position:{x:0,y:0,z:0}});
  assert.equal(engine.rigidBody.c.differential.type,'lsd');
  assert.equal(engine.rigidBody.c.differential.layout,'rwd');
  const result=engine.setDifferential('lsd','awd',3.1);
  assert.deepEqual(JSON.parse(JSON.stringify(result)),{type:'lsd',layout:'awd',limitedSlip:3.1});
});

test('embedded solver applies triangle material friction to tire forces',()=>{
  const source=bundle.slice(0,bundle.indexOf('(async function startPlayCar3DScene'));
  const context={console};vm.runInNewContext(source,context);
  const mesh={closestPoint(p){return {point:{x:p.x,y:0,z:p.z},normal:{x:0,y:1,z:0},distance:Math.abs(p.y),signedDistance:p.y,friction:.2};}};
  const Engine=context.PlayCarRigidBody.RigidBodyVehicle;
  const engine=new Engine({surfaceMesh:mesh,groundPlaneY:0,position:{x:0,y:.3,z:0}});
  engine.state.velocity={x:5,y:0,z:0};engine.advance(1/120,{});
  assert.ok(engine.telemetry.forces.length>0);
  assert.ok(engine.telemetry.forces[0].frictionCoefficient<engine.c.tire.mu);
});
test('embedded solver exposes the new tire dynamics configuration',()=>{
  const source=bundle.slice(0,bundle.indexOf('(async function startPlayCar3DScene'));
  const context={console};vm.runInNewContext(source,context);
  const tire=context.PlayCarRigidBody.defaults.tire;
  assert.ok(tire.relaxationAngle>0);
  assert.ok(tire.referenceLoad>0);
  assert.ok(tire.loadSensitivity>0);
  assert.ok(tire.aligningTrail>0);
});

test('all-in-one bundle exports the same Pacejka and friction-ellipse tire solver',()=>{
  const source=bundle.slice(0,bundle.indexOf('(async function startPlayCar3DScene'));
  const context={console};vm.runInNewContext(source,context);
  const api=context.PlayCarRigidBody;
  assert.equal(api.version,'2.4.0');
  assert.equal(typeof api.calculateTireForces,'function');
  const tire={mu:1,corneringStiffness:78000,pacejka:{enabled:true,B:0,C:1.3,E:.97,peak:1},frictionEllipse:{longitudinal:1,lateral:1}};
  const lateral=api.calculateTireForces(0,.18,3000,1,tire);
  const combined=api.calculateTireForces(2400,.18,3000,1,tire);
  assert.ok(Math.abs(combined.Fy)<Math.abs(lateral.Fy));
  assert.ok(combined.utilization<=1+1e-9);
});

test('all-in-one bundle exposes progressive bump-stop suspension telemetry',()=>{
  const source=bundle.slice(0,bundle.indexOf('(async function startPlayCar3DScene'));
  const context={console};vm.runInNewContext(source,context);
  const Engine=context.PlayCarRigidBody.RigidBodyVehicle;
  const engine=new Engine({groundPlaneY:0,position:{x:0,y:.3,z:0}});
  engine.advance(1/120,{});
  assert.equal(context.PlayCarRigidBody.version,'2.4.0');
  assert.equal(engine.telemetry.suspension.length,4);
  assert.ok(engine.telemetry.suspension[0].bumpStopForce>0);
});

test('embedded rigid-body solver reports conserved lateral cornering load transfer',()=>{
 const source=bundle.slice(0,bundle.indexOf('(async function startPlayCar3DScene'));
 const context={console};vm.runInNewContext(source,context);
 const Engine=context.PlayCarRigidBody.RigidBodyVehicle;
 const engine=new Engine({groundPlaneY:0,position:{x:0,y:.3,z:0}});
 engine.state.velocity={x:0,y:0,z:-20};engine.advance(1/120,{steer:1});
 assert.ok(engine.telemetry.loadTransfer.lateral>0);
 const n=engine.telemetry.normalLoads;
 assert.ok(n[0]+n[2]>n[1]+n[3]);
});
