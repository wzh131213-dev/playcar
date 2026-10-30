'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const html=fs.readFileSync(path.join(__dirname,'..','index.html'),'utf8');
const scenePath=path.join(__dirname,'..','physics','scene3d.js');

test('physics lab exposes a dedicated interactive 3D scene canvas',()=>{
  assert.match(html,/id="scene3d"/);
  assert.match(html,/physics\\/scene3d\\.js/);
  assert.match(html,/id="scene-status"/);
});
test('3D scene creates a renderer, perspective camera, lighting and solid collision meshes',()=>{
  const source=fs.readFileSync(scenePath,'utf8');
  for(const token of ['WebGLRenderer','PerspectiveCamera','HemisphereLight','DirectionalLight','PlaneGeometry','BoxGeometry','MeshStandardMaterial'])
    assert.ok(source.includes(token),'missing 3D scene feature: '+token);
  assert.match(source,/surfaceMesh:surface/);
  assert.match(source,/colliderMeshes:\\[walls\\]/);
});
test('3D scene provides keyboard and pointer-safe touch driving controls',()=>{
  const source=fs.readFileSync(scenePath,'utf8');
  for(const token of ['pointerdown','pointerup','pointercancel','keydown','keyup','blur','touchAction'])
    assert.ok(source.includes(token),'missing input handling: '+token);
});
test('3D scene includes ramp, barriers, reset and telemetry hooks',()=>{
  const source=fs.readFileSync(scenePath,'utf8');
  for(const token of ['createRamp','createBarrier','resetVehicle','getTelemetry','requestAnimationFrame'])
    assert.ok(source.includes(token),'missing simulation feature: '+token);
});
