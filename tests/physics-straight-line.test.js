'use strict';
/* Straight-line stability of the shipping 3.0 configuration.
 *
 * Static camber and toe are per-wheel alignment values: they must be mirrored
 * left/right. Applying one signed angle to both sides steers the whole axle and
 * used to make the car drift and spin above ~90 km/h even with zero steering
 * input. This test drives the real unified engine (same config as the 3D lab)
 * and requires it to track straight, and to damp a yaw perturbation.
 */
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

function loadEngine() {
  const bundle = fs.readFileSync(path.join(__dirname, '../physics/playcar-physics-all-in-one.js'), 'utf8');
  const marker = bundle.indexOf('(async function startPlayCar3DScene');
  assert.ok(marker > 0, 'bundle scene marker missing');
  const context = { console, performance: globalThis.performance };
  vm.createContext(context);
  vm.runInContext(bundle.slice(0, marker), context, { filename: 'playcar-physics-prefix.js' });
  return context.PlayCarPhysicsEngine;
}

const LAB_CONFIG = {
  gravity: 9.81,
  maxSpeed: 62,
  engineAcceleration: 6.4,
  brakeAcceleration: 11,
  tireGrip: 1.08,
  handbrakeGrip: 0.7,
  maxSteerAngle: 0.5,
  highSpeedSteerScale: 0.44,
  suspensionRestLength: 0.3,
  suspensionReach: 0.52,
  bodyHalfExtents: { x: 0.84, y: 0.4, z: 1.7 },
  bodyCenterY: 0.2
};

function makeCar(API) {
  const half = 400;
  const surface = API.TriangleMesh.fromTriangles([
    [{ x: -half, y: 0, z: half }, { x: -half, y: 0, z: -half }, { x: half, y: 0, z: -half }, 1],
    [{ x: -half, y: 0, z: half }, { x: half, y: 0, z: -half }, { x: half, y: 0, z: half }, 1]
  ], 8);
  const car = new API.PlayCarPhysicsEngine({
    surfaceMesh: surface, groundPlaneY: 0, position: { x: 0, y: 0.08, z: 200 },
    preset: 'simcade', config: LAB_CONFIG
  });
  const rigid = car.rigidBody;
  rigid.c.aids.esc = true;
  rigid.c.suspension.progressive = 0.45;
  rigid.c.suspension.dampingBump = 4200;
  rigid.c.suspension.dampingRebound = 6200;
  rigid.c.tire.relaxationLength = 0.32;
  return { car, rigid };
}

function yawOf(state) {
  const o = state.orientation;
  return Math.atan2(2 * (o.w * o.y + o.x * o.z), 1 - 2 * (o.y * o.y + o.z * o.z));
}
function speedOf(state) {
  return Math.hypot(state.velocity.x, state.velocity.z);
}

test('zero-steer full throttle tracks straight up to highway speed', () => {
  const API = loadEngine();
  const { car, rigid } = makeCar(API);
  let peakYaw = 0;
  for (let i = 0; i < 900; i++) {
    car.advance(1 / 60, { throttle: 1, steer: 0, brake: 0, handbrake: 0 });
    peakYaw = Math.max(peakYaw, Math.abs(yawOf(rigid.state)));
  }
  const speed = speedOf(rigid.state);
  assert.ok(speed > 40, `car must reach highway speed, got ${speed.toFixed(1)} m/s`);
  assert.ok(peakYaw < 0.05, `straight-line yaw drift must stay under 2.9 deg, got ${(peakYaw * 180 / Math.PI).toFixed(2)}`);
  assert.ok(Math.abs(rigid.state.position.x) < 1.5, `lateral drift must stay bounded, got ${rigid.state.position.x.toFixed(2)} m`);
});

test('a yaw perturbation decays at highway speed instead of spinning the car', () => {
  const API = loadEngine();
  const { car, rigid } = makeCar(API);
  const target = 25;
  for (let i = 0; i < 900 && speedOf(rigid.state) < target; i++) {
    const throttle = Math.max(0.1, Math.min(1, (target - speedOf(rigid.state)) * 0.5 + 0.15));
    car.advance(1 / 60, { throttle, steer: 0 });
  }
  assert.ok(speedOf(rigid.state) > target - 2, 'car must reach the perturbation speed');
  const base = yawOf(rigid.state);
  rigid.state.angularVelocity.y += 0.03;
  let peak = 0;
  let last = 0;
  for (let i = 0; i < 180; i++) {
    car.advance(1 / 60, { throttle: 0.25, steer: 0 });
    let delta = yawOf(rigid.state) - base;
    while (delta > Math.PI) delta -= 2 * Math.PI;
    while (delta < -Math.PI) delta += 2 * Math.PI;
    peak = Math.max(peak, Math.abs(delta));
    last = Math.abs(delta);
  }
  assert.ok(peak < 0.1, `yaw excursion must stay under 5.7 deg, got ${(peak * 180 / Math.PI).toFixed(2)}`);
  assert.ok(Math.abs(rigid.state.angularVelocity.y) < 0.02,
    `yaw rate must decay from 0.03 rad/s, got ${rigid.state.angularVelocity.y.toFixed(4)} (peak ${(peak * 180 / Math.PI).toFixed(2)} deg, final ${(last * 180 / Math.PI).toFixed(2)} deg)`);
});
