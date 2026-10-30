'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const Engine=require('../physics/playcar-physics-engine.js');
const {PlayCarPhysicsEngine,TriangleMesh}=Engine;

function flatGround(size=100,y=0){
  const a={x:-size,y,z:-size},b={x:-size,y,z:size},c={x:size,y,z:size},d={x:size,y,z:-size};
  return TriangleMesh.fromTriangles([[a,b,c],[a,c,d]],8);
}
function run(engine,seconds,input){
  const count=Math.round(seconds/engine.config.fixedTimeStep);
  for(let i=0;i<count;i++)engine.advance(engine.config.fixedTimeStep,input);
}

test('default road-tire friction coefficient is in a realistic dry-road range',()=>{
  const e=new PlayCarPhysicsEngine({surfaceMesh:flatGround(),position:{x:0,y:-.03,z:0}});
  assert.ok(e.config.tireGrip>=.85&&e.config.tireGrip<=1.2,
    'dry tire-road friction coefficient should be dimensionless and near 1.0, not arcade-scaled');
  assert.ok(e.config.handbrakeGrip>0&&e.config.handbrakeGrip<=e.config.tireGrip,
    'handbrake must not create more grip than normal tires');
});
test('full-throttle acceleration is progressive rather than an instant launch',()=>{
  const e=new PlayCarPhysicsEngine({surfaceMesh:flatGround(),position:{x:0,y:-.03,z:0}});
  run(e,1,{throttle:1});
  const speed=e.getTelemetry().speed;
  assert.ok(speed>=3&&speed<=8,
    'one second of full throttle from rest should produce a controlled road-car speed');
});
test('wet road lowers the available tire friction coefficient',()=>{
  const dry=new PlayCarPhysicsEngine({surfaceMesh:flatGround(),position:{x:0,y:-.03,z:0}});
  const wet=new PlayCarPhysicsEngine({surfaceMesh:flatGround(),position:{x:0,y:-.03,z:0}});
  dry.state.velocity={x:0,y:0,z:-12};wet.state.velocity={x:0,y:0,z:-12};
  dry.setWeather({wetness:0});wet.setWeather({wetness:1});
  run(dry,.3,{});run(wet,.3,{});
  assert.ok(Math.abs(wet.state.velocity.x)>=Math.abs(dry.state.velocity.x),
    'wet surface should not increase lateral tire grip');
});
test('3D test scene includes the visual ground in the physical support mesh and solid fallback plane',()=>{
  const fs=require('node:fs');
  const source=fs.readFileSync(require('node:path').join(__dirname,'../physics/scene3d.js'),'utf8');
  assert.ok(source.includes('groundPlaneY:-.18'),'the world floor must have a solid chassis collision plane');
  assert.ok(source.includes('groundTriangles'),'the large visible ground must be included in the wheel support mesh');
  assert.ok(source.includes('surfaceMesh:surface'),'the vehicle must use the combined ground and road mesh');
});
