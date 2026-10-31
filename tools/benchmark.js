#!/usr/bin/env node
/* PlayCar physics benchmark.
 * Reports deterministic fixed-step throughput for the rigid-body core and the
 * full vehicle (suspension + combined-slip tires + drivetrain + 40 props).
 *
 * Run: npm run bench
 * These numbers are CPU micro-benchmarks, not frame rate. A 120 Hz budget is
 * 8.33 ms per rendered frame; the vehicle runs fixed 1/120 s sub-steps.
 */
'use strict';
const Core = require('../physics/rigidbody-core.js');
const Vehicle = require('../physics/rigidbody-vehicle-engine.js');

function bench(name, warmup, iterations, fn) {
  for (let i = 0; i < warmup; i++) fn(i);
  const start = process.hrtime.bigint();
  for (let i = 0; i < iterations; i++) fn(i);
  const ns = Number(process.hrtime.bigint() - start);
  const msPer = ns / 1e6 / iterations;
  const perSecond = 1000 / msPer;
  console.log(`${name.padEnd(46)} ${msPer.toFixed(4)} ms/op  ${Math.round(perSecond).toLocaleString('en-US')} ops/s`);
  return msPer;
}

function makeMesh(radius = 60) {
  const segments = 96;
  const tris = [];
  for (let i = 0; i < segments; i++) {
    const a = (i / segments) * Math.PI * 2;
    const b = ((i + 1) / segments) * Math.PI * 2;
    const p = angle => ({ x: Math.cos(angle) * radius, y: 0, z: Math.sin(angle) * radius });
    const center = { x: 0, y: 0, z: 0 };
    const p1 = p(a), p2 = p(b);
    tris.push({ a: center, b: p1, c: p2, friction: 1 });
    tris.push({ a: p1, b: p2, c: { x: p2.x * 1.05, y: 0, z: p2.z * 1.05 }, friction: 0.7 });
  }
  return new Core.MeshShape(tris);
}

console.log('PlayCar physics benchmark — node ' + process.version);
console.log('');

const coreWorld = new Core.World({ gravity: { x: 0, y: -9.81, z: 0 } });
coreWorld.addGroundPlane(0, { friction: 0.9 });
for (let i = 0; i < 60; i++) {
  coreWorld.addBody(new Core.Body({
    mass: 8,
    shape: new Core.BoxShape({ x: 0.4, y: 0.4, z: 0.4 }),
    position: { x: (i % 8) * 1.2 - 4.2, y: 1 + Math.floor(i / 8) * 1.1, z: (i % 5) * 1.1 - 2.2 },
    friction: 0.7,
    restitution: 0.05
  }));
}
for (let i = 0; i < 60; i++) coreWorld.fixedStep(1 / 120);

const coreMs = bench('core: 60-box stack, 1 fixed step', 30, 600, () => coreWorld.fixedStep(1 / 120));

const mesh = makeMesh();
const legacyMesh = {
  triangles: mesh.triangles.map(t => [t.a, t.b, t.c, t.friction]),
  closestPoint(p, max, preferred) { return mesh.closestPoint(p, max, preferred); }
};
const world = new Vehicle.RigidBodyVehicle({
  surfaceMesh: legacyMesh,
  groundPlaneY: -0.18,
  position: { x: 0, y: 0.55, z: 0 },
  config: { aids: { abs: true, tcs: true, esc: true } }
});
for (let i = 0; i < 40; i++) {
  world.addProp({
    shape: i % 2 ? 'sphere' : 'box',
    halfExtents: { x: 0.25, y: 0.25, z: 0.25 },
    radius: 0.28,
    mass: 10,
    position: { x: Math.cos(i) * 6, y: 1 + i * 0.4, z: Math.sin(i) * 6 }
  });
}
for (let i = 0; i < 240; i++) world.advance(1 / 120, { throttle: 1, steer: 0.2 });

const vehicleMs = bench('vehicle: 1 car + 40 props, 1 fixed step', 60, 1200, i =>
  world.advance(1 / 120, { throttle: i % 600 < 300 ? 1 : 0, brake: i % 600 > 520 ? 0.8 : 0, steer: Math.sin(i / 60) * 0.4 }));

const stone = 1000 / 120;
console.log('');
console.log(`120 Hz sub-step budget: ${stone.toFixed(3)} ms`);
console.log(`vehicle load at 120 Hz: ${(vehicleMs / stone * 100).toFixed(1)}% of one core`);
console.log(`core contacts: ${coreWorld.stats.contacts}, vehicle contacts: ${world.world.stats.contacts}`);
console.log(`vehicle world bodies: ${world.world.bodies.length + world.world.staticBodies.length}`);
