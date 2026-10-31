'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const Core = require('../physics/rigidbody-core.js');
const Vehicle = require('../physics/rigidbody-vehicle-engine.js');

function flatMesh(friction = 1) {
  const a = { x: -200, y: 0, z: -200 };
  const b = { x: -200, y: 0, z: 200 };
  const c = { x: 200, y: 0, z: 200 };
  const d = { x: 200, y: 0, z: -200 };
  return [a, b, c, d].length ? [
    { a, b, c, friction },
    { a, b: c, c: d, friction }
  ] : [];
}

function legacyPlane() {
  const a = { x: -200, y: 0, z: -200 };
  const b = { x: -200, y: 0, z: 200 };
  const c = { x: 200, y: 0, z: 200 };
  const d = { x: 200, y: 0, z: -200 };
  return {
    triangles: [[a, b, c], [a, c, d]],
    closestPoint(p, max = Infinity, preferredNormal = null) {
      const y = 0;
      const point = { x: p.x, y, z: p.z };
      const distance = Math.abs(p.y - y);
      if (distance > max) return null;
      return {
        point, normal: preferredNormal ? { ...preferredNormal } : { x: 0, y: 1, z: 0 },
        distance, signedDistance: p.y - y, friction: 1
      };
    }
  };
}

function bundleContext() {
  const bundle = fs.readFileSync(path.join(__dirname, '../physics/playcar-physics-all-in-one.js'), 'utf8');
  const source = bundle.slice(0, bundle.indexOf('(async function startPlayCar3DScene'));
  const context = { console };
  vm.runInNewContext(source, context);
  return context;
}

test('combined-slip tire model respects the friction circle and driver inputs', () => {
  const tire = Vehicle.defaults.tire;
  const base = { normalLoad: 3500, frictionCoefficient: 1, tire };
  const pureLateral = Vehicle.calculateCombinedTireForces({ ...base, slipRatio: 0, slipAngle: 0.15 });
  const pureLongitudinal = Vehicle.calculateCombinedTireForces({ ...base, slipRatio: 0.15, slipAngle: 0 });
  const combined = Vehicle.calculateCombinedTireForces({ ...base, slipRatio: 0.2, slipAngle: 0.2 });
  assert.ok(Math.hypot(pureLateral.Fx, pureLateral.Fy) > 0.25 * 3500);
  assert.ok(Math.hypot(pureLongitudinal.Fx, pureLongitudinal.Fy) > 0.25 * 3500);
  assert.ok(Math.hypot(combined.Fx, combined.Fy) <= 3500 + 1e-6, 'combined force must stay inside the friction circle');
  assert.ok(Math.abs(combined.Fy) < Math.abs(pureLateral.Fy) + 1e-6, 'longitudinal demand must reduce lateral force');
  assert.ok(combined.utilization <= 1 + 1e-9);
});

test('combined-slip tire force flips direction with slip and is finite at extreme slip', () => {
  const tire = Vehicle.defaults.tire;
  const base = { normalLoad: 3000, frictionCoefficient: 1, tire, slipRatio: 0 };
  const left = Vehicle.calculateCombinedTireForces({ ...base, slipAngle: 0.4 });
  const right = Vehicle.calculateCombinedTireForces({ ...base, slipAngle: -0.4 });
  assert.ok(left.Fy * right.Fy < 0, 'lateral force must change sign with slip angle');
  const extreme = Vehicle.calculateCombinedTireForces({ ...base, slipRatio: 50, slipAngle: 1.7 });
  assert.ok(Number.isFinite(extreme.Fx) && Number.isFinite(extreme.Fy));
  assert.ok(Math.hypot(extreme.Fx, extreme.Fy) <= 3000 + 1e-6);
});

test('progressive springs and asymmetric dampers are exposed in suspension telemetry', () => {
  const mesh = { triangles: flatMesh(), closestPoint: legacyPlane().closestPoint };
  const engine = new Vehicle.RigidBodyVehicle({
    surfaceMesh: mesh, groundPlaneY: 0, position: { x: 0, y: 0.3, z: 0 },
    config: { suspension: { progressive: 0.8, dampingBump: 3000, dampingRebound: 7000 } }
  });
  engine.advance(1 / 120, {});
  const s = engine.telemetry.suspension[0];
  assert.ok(Number.isFinite(s.springForce) && s.springForce > 0);
  assert.ok(Number.isFinite(s.damperForce));
  assert.equal(engine.c.suspension.dampingBump, 3000);
});

test('ABS engages on a low-friction surface under hard braking', () => {
  const mesh = { triangles: flatMesh(0.35), closestPoint: legacyPlane().closestPoint };
  const engine = new Vehicle.RigidBodyVehicle({
    surfaceMesh: mesh, groundPlaneY: 0, position: { x: 0, y: 0.3, z: 0 },
    config: { aids: { abs: true, tcs: true, absSlip: 0.12 } }
  });
  engine.state.velocity = { x: 0, y: 0, z: -35 };
  let sawAbs = false;
  for (let i = 0; i < 60; i++) {
    engine.advance(1 / 120, { brake: 1 });
    if (engine.telemetry.absActive) sawAbs = true;
  }
  assert.ok(sawAbs, 'ABS must intervene before the wheels lock completely');
});

test('engine mounts props in the same rigid-body world and they settle on the road', () => {
  const mesh = { triangles: flatMesh(), closestPoint: legacyPlane().closestPoint };
  const engine = new Vehicle.RigidBodyVehicle({
    surfaceMesh: mesh, groundPlaneY: 0, position: { x: 0, y: 0.3, z: 0 }
  });
  const prop = engine.addProp({
    shape: 'box', halfExtents: { x: 0.25, y: 0.25, z: 0.25 }, mass: 5,
    position: { x: 4, y: 2, z: 0 }
  });
  for (let i = 0; i < 240; i++) engine.advance(1 / 120, {});
  assert.ok(prop.position.y > 0.2 && prop.position.y < 0.3, `prop rest height ${prop.position.y}`);
  assert.ok(prop.sleeping, 'settled prop should sleep');
});

test('chassis cannot tunnel through a thin wall at race speed', () => {
  const a = { x: 0, y: -3, z: -30 }, b = { x: 0, y: 3, z: -30 }, c = { x: 0, y: 3, z: 30 }, d = { x: 0, y: -3, z: 30 };
  const wall = { triangles: [[a, b, c, 1], [a, c, d, 1]], closestPoint: () => null };
  const engine = new Vehicle.RigidBodyVehicle({
    surfaceMesh: null, colliderMeshes: [wall], groundPlaneY: -0.18,
    position: { x: -6, y: 0.7, z: 0 }
  });
  engine.state.velocity = { x: 45, y: 0, z: 0 };
  for (let i = 0; i < 90; i++) engine.advance(1 / 120, {});
  assert.ok(engine.state.position.x < 0.6, `car tunnelled through wall: x=${engine.state.position.x}`);
});

test('unified runtime advances the rigid vehicle and exposes 3.0 telemetry', () => {
  const context = bundleContext();
  const Engine = context.PlayCarPhysicsEngine.PlayCarPhysicsEngine;
  const engine = new Engine({
    surfaceMesh: legacyPlane(),
    groundPlaneY: 0,
    position: { x: 0, y: 0.08, z: 0 }
  });
  assert.equal(context.PlayCarPhysicsEngine.version, '3.0.0');
  assert.ok(engine.rigidBody, 'unified runtime must own a rigid-body vehicle');
  for (let i = 0; i < 240; i++) engine.advance(1 / 120, { throttle: 1, steer: 0.15 });
  const t = engine.getTelemetry();
  assert.ok(Number.isFinite(t.speed) && t.speed > 2, `car should be moving, speed=${t.speed}`);
  assert.ok(t.gear >= 1);
  assert.ok(t.rpm >= 900);
  assert.ok(Array.isArray(t.tireLoads) && t.tireLoads.length === 4);
  assert.ok(Array.isArray(t.wheelSlip) && t.wheelSlip.length === 4);
  assert.ok(Number.isFinite(engine.state.position.x));
  assert.ok(Math.abs(Math.hypot(engine.state.orientation.x, engine.state.orientation.y,
    engine.state.orientation.z, engine.state.orientation.w) - 1) < 1e-8);
});

test('unified runtime keeps legacy weather, config and differential mapping', () => {
  const context = bundleContext();
  const Engine = context.PlayCarPhysicsEngine.PlayCarPhysicsEngine;
  const engine = new Engine({ surfaceMesh: legacyPlane(), groundPlaneY: 0, position: { x: 0, y: 0.08, z: 0 } });
  const dryMu = engine.rigidBody.c.tire.mu;
  engine.setWeather({ wetness: 1 });
  assert.ok(engine.rigidBody.c.tire.mu < dryMu, 'wet weather must lower the tire friction coefficient');
  engine.setConfig({ tireGrip: 1.4 });
  assert.ok(Math.abs(engine.rigidBody.c.tire.mu - 1.4 * 0.58) < 1e-9);
  const diff = engine.setDifferential('lsd', 'awd', 3.4);
  assert.deepEqual(JSON.parse(JSON.stringify(diff)), { type: 'lsd', layout: 'awd', limitedSlip: 3.4 });
  engine.advance(1 / 120, { throttle: 1 });
  assert.deepEqual(JSON.parse(JSON.stringify(engine.getTelemetry().drivenWheels)), [0, 1, 2, 3]);
});

test('unified snapshot contains state, telemetry, debug, config and rigid diagnostics', () => {
  const context = bundleContext();
  const Engine = context.PlayCarPhysicsEngine.PlayCarPhysicsEngine;
  const engine = new Engine({ surfaceMesh: legacyPlane(), groundPlaneY: 0, position: { x: 0, y: 0.08, z: 0 } });
  engine.advance(1 / 30, { throttle: 0.5 });
  const snap = engine.getSnapshot();
  for (const key of ['state', 'telemetry', 'debug', 'config', 'weather', 'input', 'timing', 'rigid']) {
    assert.ok(key in snap, `snapshot missing ${key}`);
  }
  assert.equal(snap.solver, 'rigid-body-3.0.0');
  assert.ok(snap.rigid.world.bodies >= 0);
});

test('rigid engine configuration is tunable at runtime without breaking the step', () => {
  const context = bundleContext();
  const Engine = context.PlayCarPhysicsEngine.PlayCarPhysicsEngine;
  const engine = new Engine({ surfaceMesh: legacyPlane(), groundPlaneY: 0, position: { x: 0, y: 0.08, z: 0 } });
  engine.setRigidConfig({
    suspension: { stiffness: 40000, dampingRebound: 7000 },
    tire: { mu: 1.2, combined: { peak: 0.95 } },
    aids: { esc: true }
  });
  assert.equal(engine.rigidBody.c.tire.mu, 1.2);
  assert.equal(engine.rigidBody.c.suspension.stiffness, 40000);
  engine.advance(1 / 120, { throttle: 1, steer: 0.3 });
  assert.ok(Number.isFinite(engine.state.position.y));
});
