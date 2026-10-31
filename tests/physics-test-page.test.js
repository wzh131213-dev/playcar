'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const root=path.join(__dirname,'..');
const html=fs.readFileSync(path.join(root,'index.html'),'utf8');
const sceneHtml=fs.readFileSync(path.join(root,'test-scene.html'),'utf8');
const sceneSource=fs.readFileSync(path.join(root,'physics','test-scene.js'),'utf8');
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

test('lab page exposes a weather preset menu wired to the engine',()=>{
  assert.match(html,/id="weather-preset"/);
  assert.match(html,/PlayCarWeather/);
  for(const preset of ['clear','overcast','rain','storm','fog','snow'])
    assert.ok(html.includes('value="'+preset+'"'),'missing lab weather preset: '+preset);
});
test('test scene page hosts the proving ground and the real check suite',()=>{
  assert.match(sceneHtml,/<title>PlayCar · 物理测试场<\/title>/);
  assert.match(sceneHtml,/id="scene3d"/);
  assert.match(sceneHtml,/physics\/rigidbody-core\.js\?v=3\.0\.0/);
  assert.match(sceneHtml,/physics\/rigidbody-vehicle-engine\.js\?v=3\.0\.0/);
  assert.match(sceneHtml,/physics\/playcar-physics-engine\.js\?v=3\.0\.0/);
  assert.match(sceneHtml,/physics\/playcar-unified\.js\?v=3\.0\.0/);
  assert.doesNotMatch(sceneHtml,/playcar-physics-all-in-one/);
  assert.match(sceneHtml,/physics\/physics-checks\.js\?v=1\.1\.0/);
  assert.match(sceneHtml,/physics\/test-scene\.js\?v=1\.0\.0/);
  assert.match(sceneHtml,/physics\/weather-system\.js\?v=1\.0\.0/);
  assert.match(sceneHtml,/physics\/weather-visuals\.js\?v=1\.0\.0/);
  for(const preset of ['clear','overcast','rain','storm','fog','snow','night'])
    assert.ok(sceneHtml.includes('data-weather="'+preset+'"'),'missing weather preset button: '+preset);
  assert.match(sceneHtml,/PlayCarRuntime\.setWeatherPreset/);
  for(const id of ['scene-gas','scene-brake','scene-left','scene-right','scene-handbrake','scene-reset','scene-follow','run-checks','check-list','download-checks','stat-speed','stat-ms'])
    assert.ok(sceneHtml.includes('id="'+id+'"'),'missing test scene control: '+id);
  assert.match(sceneHtml,/PlayCarPhysicsChecks\.runAll\(\)/);
  assert.match(sceneHtml,/new Blob\(/);
  assert.match(sceneHtml,/playcar:ready/);
  assert.doesNotMatch(sceneHtml,/(?:src|href)="file:/);
});
test('proving ground scene wires the runtime facade and all six test surfaces',()=>{
  assert.match(sceneSource,/window\.PlayCarRuntime/);
  assert.match(sceneSource,/playcar:ready/);
  assert.match(sceneSource,/TriangleMesh\.fromTriangles/);
  assert.match(sceneSource,/addProp/);
  assert.match(sceneSource,/setWeather/);
  assert.match(sceneSource,/PlayCarWeather/);
  assert.match(sceneSource,/createPlayCarWeatherVisuals/);
  assert.match(sceneSource,/requestAnimationFrame/);
  for(const marker of ['ICE','WET','DRY','Solid wall','Launch ramp','Washboard'])
    assert.ok(sceneSource.includes(marker),'missing proving-ground feature: '+marker);
});
