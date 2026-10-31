'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const Core = require('../physics/rigidbody-core.js');
const { World, Body, BoxShape, SphereShape, PlaneShape, MeshShape, Vec3: V } = Core;

function makeWorld() {
  return new World({ gravity: { x: 0, y: -9.81, z: 0 } });
}

function step(world, seconds) {
  const steps = Math.round(seconds / world.settings.fixedDt);
  for (let i = 0; i < steps; i++) world.fixedStep(world.settings.fixedDt);
}

test('core exposes a complete rigid-body API', () => {
  assert.equal(Core.version, '3.0.0');
  for (const key of ['World', 'Body', 'BoxShape', 'SphereShape', 'PlaneShape', 'MeshShape']) {
    assert.equal(typeof Core[key], 'function', missing(key));
  }
  assert.equal(typeof Core.collideBoxBox, 'function');
  assert.equal(typeof Core.collideTriangleBox, 'function');
});
function missing(k) { return 'missing core export: ' + k; }

test('box falls, lands on a plane and comes to rest without sinking', () => {
  const world = makeWorld();
  world.addGroundPlane(0, { friction: 0.9 });
  const box = new Body({
    mass: 10, shape: new BoxShape({ x: 0.5, y: 0.5, z: 0.5 }),
    position: { x: 0, y: 3, z: 0 }, friction: 0.8, restitution: 0, allowSleep: true
  });
  world.addBody(box);
  step(world, 2.5);
  assert.ok(box.position.y > 0.47 && box.position.y < 0.56, `unexpected rest height ${box.position.y}`);
  assert.ok(Math.abs(box.linearVelocity.y) < 0.05, 'box should be at rest');
  assert.ok(box.sleeping, 'settled box should sleep');
});

test('box does not tunnel through a thin static triangle mesh at high speed', () => {
  const world = makeWorld();
  const wall = MeshShape.fromTriangles([
    [{ x: 0, y: -5, z: -5 }, { x: 0, y: 5, z: -5 }, { x: 0, y: 5, z: 5 }],
    [{ x: 0, y: -5, z: -5 }, { x: 0, y: 5, z: 5 }, { x: 0, y: -5, z: 5 }]
  ]);
  world.addStaticMesh(wall, { friction: 0.4 });
  const box = new Body({
    mass: 20, shape: new BoxShape({ x: 0.3, y: 0.3, z: 0.3 }),
    position: { x: -2, y: 0, z: 0 }, velocity: { x: 120, y: 0, z: 0 },
    friction: 0.5, restitution: 0.1, allowSleep: false, gravityScale: 0
  });
  world.addBody(box);
  for (let i = 0; i < 60; i++) world.fixedStep(1 / 120);
  assert.ok(box.position.x < 0.05, `fast box tunnelled through wall: x=${box.position.x}`);
});

test('restitution produces a measurable bounce', () => {
  const world = makeWorld();
  world.addGroundPlane(0);
  const ball = new Body({
    mass: 1, shape: new SphereShape(0.5), position: { x: 0, y: 3, z: 0 },
    restitution: 0.7, friction: 0.1, allowSleep: false
  });
  world.addBody(ball);
  let landed = false, maxAfter = 0;
  for (let i = 0; i < 480; i++) {
    world.fixedStep(1 / 120);
    if (!landed && ball.position.y < 0.75) landed = true;
    if (landed) maxAfter = Math.max(maxAfter, ball.position.y);
  }
  assert.ok(maxAfter > 0.8, `expected a bounce, peak height was ${maxAfter}`);
  assert.ok(maxAfter < 3, 'restitution must not add energy');
});

test('friction holds a box on a 30 degree slope and low friction lets it slide', () => {
  const slopeNormal = V.normalize({ x: Math.sin(Math.PI / 6), y: Math.cos(Math.PI / 6), z: 0 });
  // Align the box face with the slope so the test isolates Coulomb friction
  // rather than the edge-tipping that an axis-aligned cube would exhibit.
  const axis = V.normalize(V.cross({ x: 0, y: 1, z: 0 }, slopeNormal));
  const angle = Math.acos(V.dot({ x: 0, y: 1, z: 0 }, slopeNormal));
  const orientation = Core.Quaternion.axisAngle(axis, angle);
  const build = friction => {
    const world = makeWorld();
    world.addBody(new Body({ type: 'static', shape: new PlaneShape(slopeNormal, 0), friction }));
    const box = new Body({
      mass: 5, shape: new BoxShape({ x: 0.4, y: 0.4, z: 0.4 }),
      position: V.mul(slopeNormal, 0.45), orientation, friction, restitution: 0,
      linearDamping: 0, angularDamping: 0, allowSleep: false
    });
    world.addBody(box);
    step(world, 1.5);
    return box;
  };
  const grippy = build(1.1);
  const slippery = build(0.05);
  const start = V.mul(slopeNormal, 0.45);
  assert.ok(V.distance(grippy.position, start) < 0.1, 'high friction box must stay on the slope');
  assert.ok(V.distance(slippery.position, start) > 0.4, 'low friction box must slide down');
});

test('two stacked boxes remain stable and conserve separation', () => {
  const world = makeWorld();
  world.addGroundPlane(0, { friction: 1 });
  const a = new Body({ mass: 4, shape: new BoxShape({ x: 0.5, y: 0.5, z: 0.5 }), position: { x: 0, y: 0.5, z: 0 }, friction: 1, restitution: 0, allowSleep: false });
  const b = new Body({ mass: 4, shape: new BoxShape({ x: 0.5, y: 0.5, z: 0.5 }), position: { x: 0, y: 1.5, z: 0 }, friction: 1, restitution: 0, allowSleep: false });
  world.addBody(a); world.addBody(b);
  step(world, 2);
  assert.ok(Math.abs(a.position.y - 0.5) < 0.08, `lower box moved: ${a.position.y}`);
  assert.ok(Math.abs(b.position.y - 1.5) < 0.12, `upper box moved: ${b.position.y}`);
  assert.ok(Math.abs(b.position.x) < 0.05, 'stack should not drift sideways');
});

test('mesh raycast returns the nearest surface and its friction material', () => {
  const mesh = MeshShape.fromTriangles([
    [{ x: -5, y: 0, z: -5 }, { x: 5, y: 0, z: -5 }, { x: -5, y: 0, z: 5 }, 0.25],
    [{ x: -5, y: 3, z: -5 }, { x: 5, y: 3, z: -5 }, { x: -5, y: 3, z: 5 }, 0.9]
  ]);
  const world = makeWorld();
  world.addStaticMesh(mesh);
  const hit = world.raycast({ x: 0, y: 10, z: 0 }, { x: 0, y: -1, z: 0 }, 20);
  assert.equal(hit.hit, true);
  assert.ok(Math.abs(hit.point.y - 3) < 1e-6, `expected upper surface at y=3, got ${hit.point.y}`);
  assert.equal(hit.friction, 0.9);
});

test('mesh.closestPoint is material aware and winding independent', () => {
  const mesh = MeshShape.fromTriangles([
    [{ x: -5, y: 0.1, z: -5 }, { x: 5, y: 0.1, z: -5 }, { x: -5, y: 0.1, z: 5 }, 0, 0.4]
  ]);
  const hit = mesh.closestPoint({ x: 0, y: 0.6, z: 0 }, 2, { x: 0, y: 1, z: 0 });
  assert.ok(Math.abs(hit.point.y - 0.1) < 1e-6);
  assert.ok(hit.normal.y > 0.99);
  assert.equal(hit.friction, 0);
});

test('simulation is deterministic for identical inputs', () => {
  const run = () => {
    const world = makeWorld();
    world.addGroundPlane(0, { friction: 0.6 });
    const box = new Body({ mass: 3, shape: new BoxShape({ x: 0.4, y: 0.4, z: 0.4 }), position: { x: 0, y: 2, z: 0 }, velocity: { x: 0.7, y: 0, z: 0.3 }, friction: 0.6 });
    world.addBody(box);
    for (let i = 0; i < 180; i++) world.fixedStep(1 / 120);
    return { p: box.position, q: box.orientation, v: box.linearVelocity };
  };
  const a = run(), b = run();
  assert.deepEqual(a, b);
});

test('world reports contact counts and supports begin-contact events', () => {
  const world = makeWorld();
  world.addGroundPlane(0);
  world.events.beginContact = [];
  const ball = new Body({ mass: 1, shape: new SphereShape(0.3), position: { x: 0, y: 2, z: 0 }, allowSleep: false });
  world.addBody(ball);
  let sawContact = false;
  for (let i = 0; i < 240; i++) {
    world.fixedStep(1 / 120);
    if (world.events.beginContact.length) sawContact = true;
  }
  assert.ok(sawContact, 'landing must raise a begin-contact event');
  assert.ok(world.stats.contacts >= 1);
});

test('sleeping bodies wake when another body touches them', () => {
  const world = makeWorld();
  world.addGroundPlane(0, { friction: 1 });
  const a = new Body({ mass: 2, shape: new BoxShape({ x: 0.4, y: 0.4, z: 0.4 }), position: { x: 0, y: 0.4, z: 0 }, friction: 1, restitution: 0 });
  world.addBody(a);
  step(world, 1.5);
  assert.ok(a.sleeping, 'isolated box should sleep');
  const b = new Body({ mass: 2, shape: new BoxShape({ x: 0.2, y: 0.2, z: 0.2 }), position: { x: 0, y: 2, z: 0 }, velocity: { x: 0, y: -5, z: 0 }, restitution: 0 });
  world.addBody(b);
  step(world, 0.6);
  assert.equal(a.sleeping, false, 'impact must wake the sleeping box');
});
