#!/usr/bin/env node
/* Build physics/playcar-physics-all-in-one.js from the individual modules.
 * Keeping the bundle generated avoids the source/bundle drift that previously
 * required hand-editing two copies of the vehicle solver.
 */
'use strict';
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const root = path.join(__dirname, '..');
const read = (...parts) => fs.readFileSync(path.join(root, ...parts), 'utf8');
const version = require(path.join(root, 'package.json')).version;

const legacy = read('physics', 'playcar-physics-engine.js').trimEnd();
const core = read('physics', 'rigidbody-core.js').trimEnd();
const vehicle = read('physics', 'rigidbody-vehicle-engine.js').trimEnd();
const unified = read('physics', 'playcar-unified.js').trimEnd();

let scene = read('physics', 'scene3d.js');
scene = scene.replace(/^import\s+\*\s+as\s+THREE[^\n]*\n/, '');
scene = scene.trimStart();

const indentedScene = scene.split('\n').map(line => (line ? '    ' + line : '')).join('\n');

const bundle = `/* PlayCar Physics Engine + Three.js Scene - single-file build ${version}.
 * GENERATED FILE - edit physics/*.js and run "npm run build" instead.
 *
 * Sections:
 *   1. Legacy kinematic compatibility engine (PlayCarPhysicsEngine 1.8 API)
 *   2. Rigid-body core (broadphase, manifolds, sequential-impulse solver)
 *   3. Rigid-body vehicle (suspension, combined-slip tires, drivetrain, aids)
 *   4. Unified runtime facade
 *   5. Three.js driving lab
 */

${legacy}

${core}

${vehicle}

${unified}

(async function startPlayCar3DScene(){
  let THREE;
  const sources = [
    './vendor/three.module.min.js',
    'https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.min.js',
    'https://unpkg.com/three@0.180.0/build/three.module.min.js'
  ];
  let lastError;
  for (const source of sources) {
    try { THREE = await import(source); break; }
    catch (error) { lastError = error; }
  }
  if (!THREE) {
    const status = document.getElementById('scene-status');
    if (status) status.textContent = '3D 模块加载失败：本地 vendor/ 与两个 Three.js CDN 均无法加载。请检查网络或稍后重试。';
    console.error('PlayCar Three.js module load failed:', lastError);
    return;
  }
  try {
${indentedScene}
  } catch (error) {
    const status = document.getElementById('scene-status');
    if (status) status.textContent = '3D 场景运行失败：' + (error && error.message ? error.message : String(error));
    console.error('PlayCar 3D scene failed:', error);
    throw error;
  }
})();
`;

const target = path.join(root, 'physics', 'playcar-physics-all-in-one.js');
fs.writeFileSync(target, bundle, 'utf8');
new vm.Script(bundle);
console.log(`built ${path.relative(root, target)} (${(bundle.length / 1024).toFixed(1)} KiB, ${bundle.split('\n').length} lines)`);
