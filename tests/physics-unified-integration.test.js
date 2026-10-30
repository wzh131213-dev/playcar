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
