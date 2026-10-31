'use strict';
/* Runs the real physics/scene3d.js module headlessly with a stubbed WebGL
 * renderer. Everything else (Three.js scene graph, the track builder, the
 * unified 3.0 engine, props, telemetry and the 60 Hz render loop) is the
 * shipping code. This catches scene/runtime integration errors that pure
 * string-matching tests cannot.
 */
const test = require('node:test');
const assert = require('node:assert/strict');
const { spawnSync } = require('node:child_process');
const path = require('node:path');

test('3D scene boots, mounts the renderer and drives the car', () => {
  const script = path.join(__dirname, '..', 'tools', 'scene-smoke.mjs');
  const result = spawnSync(process.execPath, [script], {
    cwd: path.join(__dirname, '..'),
    encoding: 'utf8',
    timeout: 120000
  });
  assert.equal(result.status, 0, `scene smoke failed:\n${result.stdout}\n${result.stderr}`);
  const line = result.stdout.split('\n').reverse().find(l => l.startsWith('RESULT '));
  assert.ok(line, `scene smoke produced no RESULT line:\n${result.stdout}\n${result.stderr}`);
  const parsed = JSON.parse(line.slice('RESULT '.length));
  assert.equal(parsed.ok, true);
  assert.equal(parsed.solver, 'rigid-body-3.0.0');
  assert.ok(parsed.speed > 1, `car speed ${parsed.speed}`);
  assert.ok(Math.abs(parsed.position.z - 34) > 1, 'car should leave the start line');
  assert.equal(parsed.wheelContacts, 4);
  assert.ok(parsed.children >= 1, 'scene host should contain the canvas and HUD');
});
