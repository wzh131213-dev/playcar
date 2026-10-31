'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const root=path.join(__dirname,'..');
const html=fs.readFileSync(path.join(root,'index.html'),'utf8');
const bundle=fs.readFileSync(path.join(root,'physics','playcar-physics-all-in-one.js'),'utf8');

test('root page is the PlayCar interactive physics control center',()=>{
  assert.match(html,/<title>PlayCar · Physics Control Center<\/title>/);
  assert.match(html,/id="scene3d"/);
  assert.match(html,/playcar-physics-all-in-one\.js\?v=3\.0\.0/);
  assert.match(bundle,/PlayCarPhysicsEngine/);
});
test('physics page exposes live touch controls, telemetry and automated checks',()=>{
  for(const id of ['scene-gas','scene-brake','scene-left','scene-right','scene-handbrake','reset-car','run-checks','check-results','stat-speed','stat-grip'])
    assert.ok(html.includes('id="'+id+'"'),'missing control: '+id);
  assert.match(bundle,/pointerdown/);
  assert.match(bundle,/requestAnimationFrame/);
  assert.match(bundle,/PlayCarRuntime/);
});
