'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const root=path.join(__dirname,'..');
const html=fs.readFileSync(path.join(root,'index.html'),'utf8');
const bundle=fs.readFileSync(path.join(root,'physics','playcar-physics-all-in-one.js'),'utf8');
const source=fs.readFileSync(path.join(root,'physics','scene3d.js'),'utf8');

test('physics lab loads the bundled interactive 3D runtime and canvas host',()=>{
  assert.ok(html.includes('id="scene3d"'));
  assert.ok(html.includes('id="scene-status"'));
  assert.ok(html.includes('./physics/playcar-physics-all-in-one.js?v=2.0.0'));
  assert.ok(bundle.includes('scene3d')&&bundle.includes('WebGLRenderer'));
});
test('3D scene creates renderer, camera, lights and collision meshes',()=>{
  for(const token of ['WebGLRenderer','PerspectiveCamera','HemisphereLight','DirectionalLight','PlaneGeometry','BoxGeometry','MeshStandardMaterial','surfaceMesh:surface','colliderMeshes:[walls]'])
    assert.ok(source.includes(token),'missing 3D scene feature: '+token);
});
test('3D scene provides keyboard and pointer-safe touch driving controls',()=>{
  for(const token of ['pointerdown','pointerup','pointercancel','keydown','keyup','blur','touchAction'])
    assert.ok(source.includes(token),'missing input handling: '+token);
});
test('3D scene includes ramp, barriers, reset and telemetry hooks',()=>{
  for(const token of ['createRamp','createBarrier','resetVehicle','getTelemetry','requestAnimationFrame'])
    assert.ok(source.includes(token),'missing simulation feature: '+token);
});
