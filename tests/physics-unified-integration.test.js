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
