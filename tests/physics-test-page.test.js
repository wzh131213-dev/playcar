'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');

const html = fs.readFileSync(path.join(__dirname, '..', 'index.html'), 'utf8');

test('root page is a physics test page, not the racing game', () => {
  assert.match(html, /PlayCar Physics Lab/i);
  assert.match(html, /physics\/playcar-physics-engine\.js/);
  assert.doesNotMatch(html, /POLY.?RUSH|TIME ATTACK|THREE\.js|cdnjs\.cloudflare\.com\/ajax\/libs\/three/i);
});

test('physics test page exposes live controls and automated checks', () => {
  for (const id of ['run-tests', 'reset-car', 'throttle', 'brake', 'steer-left', 'steer-right', 'pitch-test', 'test-results']) {
    assert.match(html, new RegExp('id=["\\']' + id + '["\\']'), 'missing control: ' + id);
  }
  assert.match(html, /new PlayCarPhysicsEngine/);
  assert.match(html, /airPitchStability|pitch-test/);
  assert.match(html, /test-results/);
});
