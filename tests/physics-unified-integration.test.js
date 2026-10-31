'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
const bundle=fs.readFileSync(path.join(__dirname,'../physics/playcar-physics-all-in-one.js'),'utf8');

test('single runtime embeds the rigid-body solver before scene startup',()=>{
  assert.match(bundle,/root\.PlayCarRigidBody\s*=\s*api/);
  assert.match(bundle,/RigidBodyVehicle,/);
  assert.match(bundle,/class UnifiedPlayCarPhysicsEngine extends LegacyPlayCarPhysicsEngine/);
  assert.match(bundle,/PlayCarPhysicsEngine\s*=\s*UnifiedPlayCarPhysicsEngine/);
});
test('unified runtime preserves legacy compatibility and weather grip mapping',()=>{
  assert.match(bundle,/LegacyPlayCarPhysicsEngine\.prototype\._applyStaticCollisions\.call\(this\)/);
  assert.match(bundle,/setWeather\(weather=\{\}\)/);
  assert.match(bundle,/tireGrip\s*\*\s*\(1\s*-\s*0\.42\s*\*\s*this\.weather\.wetness\)/);
  assert.match(bundle,/solver:\s*this\.solver/);
  assert.match(bundle,/UNIFIED_VERSION\s*=\s*'3\.0\.0'/);
});
test('render loop consumes four wheel angular velocities and front steering',()=>{
  assert.match(bundle,/vehicle\.getWheelState\(\)/);
  assert.match(bundle,/wheelState\.omega\[ix\]/);
  assert.match(bundle,/w\.pivot\.rotation\.y=w\.z<0\?-wheelState\.steer:0/);
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
  assert.equal(api.version,'3.0.0');
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
  assert.equal(context.PlayCarRigidBody.version,'3.0.0');
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

test('all-in-one bundle embeds configurable chassis roll and pitch stabilization',()=>{
 const source=bundle.slice(0,bundle.indexOf('(async function startPlayCar3DScene'));
 const context={console};vm.runInNewContext(source,context);
 const api=context.PlayCarRigidBody;
 assert.equal(api.version,'3.0.0');
 assert.deepEqual(JSON.parse(JSON.stringify(api.defaults.body)),{rollStiffness:9000,rollDamping:2200,pitchStiffness:14000,pitchDamping:2800});
 const Engine=api.RigidBodyVehicle;
 const engine=new Engine({position:{x:0,y:10,z:0},config:{body:{rollStiffness:0,rollDamping:0}}});
 assert.equal(engine.c.body.rollStiffness,0);
 assert.ok(engine.c.body.pitchStiffness>0);
});


test('3.0.0 bundle exposes hardpoint suspension geometry and solved wheel kinematics',()=>{
 const source=bundle.slice(0,bundle.indexOf('(async function startPlayCar3DScene'));
 const context={console};vm.runInNewContext(source,context);
 const api=context.PlayCarRigidBody;
 assert.equal(api.version,'3.0.0');
 assert.equal(api.defaults.suspensionGeometry.front.hardpoints.enabled,true);
 const engine=new api.RigidBodyVehicle({groundPlaneY:0,position:{x:0,y:.3,z:0}});
 engine.advance(1/120,{});
 assert.ok(engine.telemetry.wheelGeometry.every(w=>w.hardpointValid));
 assert.ok(engine.telemetry.wheelGeometry.every(w=>w.instantCenter&&Number.isFinite(w.rollCenterHeight)));
});


test('3.0.0 bundle exposes 3D caster, KPI and steering-axis jacking',()=>{
 const source=bundle.slice(0,bundle.indexOf('(async function startPlayCar3DScene'));
 const context={console};vm.runInNewContext(source,context);
 const api=context.PlayCarRigidBody;
 assert.equal(api.version,'3.0.0');
 assert.equal(api.defaults.suspensionGeometry.front.casterDeg,5.5);
 assert.equal(api.defaults.suspensionGeometry.front.kingpinInclinationDeg,8.5);
 const engine=new api.RigidBodyVehicle({groundPlaneY:0});
 const g=engine._wheelKinematics(0,0,{x:0,y:1,z:0},{steer:.5});
 assert.ok(Number.isFinite(g.jackingHeight));
 assert.ok(g.jackingHeight>0);
 assert.ok(Number.isFinite(g.steeringAxis.x)&&Number.isFinite(g.steeringAxis.y)&&Number.isFinite(g.steeringAxis.z));
});
