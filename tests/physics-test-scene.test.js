'use strict';
/* Drives physics/test-scene.js through the headless three.js harness. */
const test = require('node:test');
const assert = require('node:assert/strict');
const path = require('node:path');
const { execFileSync } = require('node:child_process');
const root = path.join(__dirname, '..');

test('proving ground scene builds, drives and reports telemetry', () => {
  const output = execFileSync(process.execPath, [path.join(root, 'tools', 'scene-smoke.mjs')], {
    cwd: root,
    encoding: 'utf8',
    timeout: 120000,
    env: { ...process.env, PLAYCAR_SCENE: 'physics/test-scene.js', PLAYCAR_START_Z: '88' }
  });
  const line = output.split('\n').find(entry => entry.startsWith('RESULT '));
  assert.ok(line, 'smoke did not print RESULT:\n' + output);
  const result = JSON.parse(line.slice('RESULT '.length));
  assert.equal(result.ok, true);
  assert.equal(result.scene, 'physics/test-scene.js');
  assert.equal(result.version, '3.0.0');
  assert.equal(result.wheelContacts, 4, 'all four wheels must stay supported');
  assert.ok(result.speed > 5, `vehicle should be moving, got ${result.speed}`);
  assert.ok(result.position.z < 80, 'vehicle should have travelled down the runway');
  assert.ok(Number.isFinite(result.position.y) && result.position.y > 0.2 && result.position.y < 2,
    'vehicle should ride at wheel height, got ' + result.position.y);
});
