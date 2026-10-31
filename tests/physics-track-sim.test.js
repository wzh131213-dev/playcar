'use strict';
/* End-to-end drives on the same circuit geometry as physics/scene3d.js, without
 * Three.js. The AI driver is deliberately simple: these tests validate the
 * engine/world integration (road mesh, banking, ramp, barriers, props), not a
 * racing line. The real scene is additionally booted headlessly by
 * tests/physics-scene-runtime.test.js.
 */
const test = require('node:test');
const assert = require('node:assert/strict');
const vm = require('node:vm');
const fs = require('node:fs');
const path = require('node:path');

function loadRuntime() {
  const bundle = fs.readFileSync(path.join(__dirname, '../physics/playcar-physics-all-in-one.js'), 'utf8');
  const source = bundle.slice(0, bundle.indexOf('(async function startPlayCar3DScene'));
  const context = { console };
  vm.runInNewContext(source, context);
  return context;
}

const CONTROL = [
  [0, 60], [0, 20], [0, -20], [0, -62], [16, -84], [44, -88], [66, -72],
  [74, -44], [70, -12], [74, 22], [60, 52], [34, 66], [12, 66]
];

function catmull(p0, p1, p2, p3, t) {
  const t2 = t * t, t3 = t2 * t;
  const f = (a, b, c, d) => 0.5 * ((2 * b) + (-a + c) * t + (2 * a - 5 * b + 4 * c - d) * t2 + (-a + 3 * b - 3 * c + d) * t3);
  return { x: f(p0[0], p1[0], p2[0], p3[0]), z: f(p0[1], p1[1], p2[1], p3[1]) };
}

function buildTrack(API) {
  const V = API.Vector3, Q = API.Quaternion;
  const halfWidth = 7.2;
  const centers = [];
  for (let i = 0; i < CONTROL.length; i++) {
    const p0 = CONTROL[(i - 1 + CONTROL.length) % CONTROL.length];
    const p1 = CONTROL[i];
    const p2 = CONTROL[(i + 1) % CONTROL.length];
    const p3 = CONTROL[(i + 2) % CONTROL.length];
    for (let s = 0; s < 8; s++) centers.push(catmull(p0, p1, p2, p3, s / 8));
  }
  const N = centers.length;
  const track = [];
  for (let i = 0; i < N; i++) {
    const prev = centers[(i - 1 + N) % N], cur = centers[i], next = centers[(i + 1) % N];
    const tPrev = V.norm({ x: cur.x - prev.x, y: 0, z: cur.z - prev.z });
    const tNext = V.norm({ x: next.x - cur.x, y: 0, z: next.z - cur.z });
    const tangent = V.norm(V.add(tPrev, tNext));
    const curvature = tPrev.z * tNext.x - tPrev.x * tNext.z;
    const bank = V.clamp(-curvature * 7.5, -0.34, 0.34);
    const up = Q.rotate(Q.axis(tangent, bank), { x: 0, y: 1, z: 0 });
    const right = V.norm(V.cross(tangent, up));
    track.push({ center: { x: cur.x, y: 0, z: cur.z }, tangent, right, up, bank });
  }
  // Same smoothing pass as scene3d.js.
  {
    const raw = track.map(f => f.bank);
    for (let i = 0; i < N; i++) {
      let sum = 0;
      for (let k = -4; k <= 4; k++) sum += raw[(i + k + N) % N];
      const bank = V.clamp(sum / 9, -0.16, 0.16);
      const f = track[i];
      f.bank = bank;
      f.up = Q.rotate(Q.axis(f.tangent, bank), { x: 0, y: 1, z: 0 });
      f.right = V.norm(V.cross(f.tangent, f.up));
    }
  }
  const surface = [], colliders = [];
  const wetStart = Math.floor(N * 0.55), wetEnd = Math.floor(N * 0.64);
  for (let i = 0; i < N; i++) {
    const j = (i + 1) % N;
    const f = track[i], g = track[j];
    const friction = i >= wetStart && i < wetEnd ? 0.55 : 1.0;
    const Li = V.add(f.center, V.mul(f.right, -halfWidth)), Ri = V.add(f.center, V.mul(f.right, halfWidth));
    const Lj = V.add(g.center, V.mul(g.right, -halfWidth)), Rj = V.add(g.center, V.mul(g.right, halfWidth));
    surface.push([Li, Ri, Rj, friction], [Li, Rj, Lj, friction]);
    for (const side of [-1, 1]) {
      const ei = V.add(f.center, V.mul(f.right, side * (halfWidth + 0.12)));
      const ej = V.add(g.center, V.mul(g.right, side * (halfWidth + 0.12)));
      colliders.push([ei, ej, { x: ej.x, y: 1.15, z: ej.z }, 1]);
      colliders.push([ei, { x: ej.x, y: 1.15, z: ej.z }, { x: ei.x, y: 1.15, z: ei.z }, 1]);
    }
  }
  // Ground support plane at y = -0.18, like the lab.
  surface.push(
    [{ x: -180, y: -0.18, z: 180 }, { x: -180, y: -0.18, z: -180 }, { x: 180, y: -0.18, z: -180 }, 0.72],
    [{ x: -180, y: -0.18, z: 180 }, { x: 180, y: -0.18, z: -180 }, { x: 180, y: -0.18, z: 180 }, 0.72]
  );
  // Tabletop ramp near z = -40 (same numbers as scene3d.js).
  let rampIdx = 0, best = Infinity;
  for (let i = 0; i < N; i++) {
    const d = Math.abs(track[i].center.z + 40) + (Math.abs(track[i].center.x) > 2 ? 100 : 0);
    if (d < best) { best = d; rampIdx = i; }
  }
  const f = track[rampIdx];
  const base = V.add(f.center, V.mul(f.tangent, -3));
  const upEnd = V.add(base, V.mul(f.tangent, 7));
  const flatEnd = V.add(upEnd, V.mul(f.tangent, 5));
  const downEnd = V.add(flatEnd, V.mul(f.tangent, 9));
  const hw = 6.3;
  const Lp = p => V.add(p, V.mul(f.right, -hw)), Rp = p => V.add(p, V.mul(f.right, hw));
  const lift = (p, h) => ({ x: p.x, y: h, z: p.z });
  const p0 = { ...base }, p1 = lift(upEnd, 1.85), p2 = lift(flatEnd, 1.85), p3 = { ...downEnd };
  const ramp = [
    [Lp(p0), Rp(p0), Rp(p1)], [Lp(p0), Rp(p1), Lp(p1)],
    [Lp(p1), Rp(p1), Rp(p2)], [Lp(p1), Rp(p2), Lp(p2)],
    [Lp(p2), Rp(p2), Rp(p3)], [Lp(p2), Rp(p3), Lp(p3)]
  ];
  // Wheel surface only; see the comment in scene3d.js createRamp().
  for (const tri of ramp) surface.push([...tri, 1]);
  const surfaceMesh = API.TriangleMesh.fromTriangles(surface, 8);
  const wallMesh = API.TriangleMesh.fromTriangles(colliders, 6);
  return { track, surfaceMesh, wallMesh, N, rampIdx, rampEnd: p3 };
}

function makeVehicle(API, track, { wetness = 0 } = {}) {
  const vehicle = new API.PlayCarPhysicsEngine({
    surfaceMesh: track.surfaceMesh,
    colliderMeshes: [track.wallMesh],
    groundPlaneY: -0.18,
    position: { x: 0, y: 0.08, z: 34 },
    preset: 'simcade',
    config: {
      gravity: 9.81, maxSpeed: 62, engineAcceleration: 6.4, brakeAcceleration: 11,
      tireGrip: 1.08, maxSteerAngle: 0.5, highSpeedSteerScale: 0.44,
      suspensionRestLength: 0.3, suspensionReach: 0.52,
      bodyHalfExtents: { x: 0.84, y: 0.4, z: 1.7 }
    }
  });
  vehicle.setWeather({ wetness });
  vehicle.rigidBody.c.aids.esc = true;
  vehicle.rigidBody.c.suspension.progressive = 0.45;
  return vehicle;
}

test('start straight, ramp jump and landing use the same geometry as the lab', () => {
  const API = loadRuntime().PlayCarPhysicsEngine;
  const track = buildTrack(API);
  const vehicle = makeVehicle(API, track);
  let maxHeight = -Infinity, minHeight = Infinity, maxSpeed = 0, airborneFrames = 0;
  let landed = false;
  for (let i = 0; i < 1000; i++) {
    const s = vehicle.state;
    vehicle.advance(1 / 120, { throttle: 1, brake: 0, steer: 0, handbrake: 0 });
    maxHeight = Math.max(maxHeight, vehicle.rigidBody.state.position.y);
    minHeight = Math.min(minHeight, vehicle.rigidBody.state.position.y);
    maxSpeed = Math.max(maxSpeed, Math.hypot(s.velocity.x, s.velocity.z));
    if (!s.grounded) airborneFrames++;
    if (airborneFrames > 3 && s.grounded) landed = true;
  }
  const end = vehicle.state;
  const t = vehicle.getTelemetry();
  assert.ok(end.position.z < -55, `car only reached z=${end.position.z.toFixed(1)}`);
  assert.ok(maxSpeed > 14, `top speed ${maxSpeed.toFixed(1)} m/s`);
  assert.ok(maxHeight > 1.4, `ramp did not lift the car (max y ${maxHeight.toFixed(2)})`);
  assert.ok(airborneFrames > 2, 'the car should be airborne over the tabletop');
  assert.ok(landed, 'the car should land and regain support');
  assert.ok(minHeight > -1.5, `car fell through the world (min y ${minHeight.toFixed(2)})`);
  assert.ok(t.wheelContacts >= 2, `wheel support ${t.wheelContacts}`);
  const up = API.Quaternion.rotate(end.orientation, { x: 0, y: 1, z: 0 });
  assert.ok(up.y > 0.65, `car is not upright (up.y ${up.y.toFixed(2)})`);
  assert.ok(Number.isFinite(t.rpm) && t.rpm > 800);
});

test('props hit by the car move, tumble and settle without leaving the world', () => {
  const API = loadRuntime().PlayCarPhysicsEngine;
  const track = buildTrack(API);
  const vehicle = makeVehicle(API, track);
  const props = [];
  for (let i = 0; i < 6; i++) {
    props.push(vehicle.addProp({
      shape: 'box', halfExtents: { x: 0.2, y: 0.35, z: 0.2 }, mass: 7,
      position: { x: 0.7, y: 0.38, z: 18 - i * 3 }, friction: 0.75, restitution: 0.1
    }));
  }
  const start = props.map(p => ({ x: p.position.x, z: p.position.z }));
  for (let i = 0; i < 720; i++) vehicle.advance(1 / 120, { throttle: 1, steer: 0 });
  let moved = 0;
  for (let i = 0; i < props.length; i++) {
    const p = props[i];
    assert.ok(Number.isFinite(p.position.x) && Number.isFinite(p.position.y));
    assert.ok(p.position.y > -1 && p.position.y < 10, `prop ${i} y ${p.position.y}`);
    if (Math.hypot(p.position.x - start[i].x, p.position.z - start[i].z) > 0.3) moved++;
  }
  assert.ok(moved >= 2, `only ${moved} props were knocked away`);
});

test('wet weather lowers grip on the same corner', () => {
  const API = loadRuntime().PlayCarPhysicsEngine;
  const track = buildTrack(API);
  const dry = makeVehicle(API, track, { wetness: 0 });
  const wet = makeVehicle(API, track, { wetness: 1 });
  assert.ok(wet.rigidBody.c.tire.mu < dry.rigidBody.c.tire.mu * 0.7);
  assert.ok(Math.abs(wet.rigidBody.c.tire.mu - 1.08 * 0.58) < 1e-9);
});
