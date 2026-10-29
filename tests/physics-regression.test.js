'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const { PlayCarPhysicsEngine, TriangleMesh, Quaternion, Vector3 } = require('../physics/playcar-physics-engine.js');

function makeInvertedFlatRoad(y = 0.1) {
  // Deliberately reverse triangle winding: a solid surface must still support the car.
  const a = { x: -50, y, z: -50 };
  const b = { x: 50, y, z: -50 };
  const c = { x: -50, y, z: 50 };
  const d = { x: 50, y, z: 50 };
  return TriangleMesh.fromTriangles([[a, b, c], [b, d, c]], 8);
}

function makePhysics() {
  const road = makeInvertedFlatRoad();
  const physics = new PlayCarPhysicsEngine({ surfaceMesh: road, colliderMeshes: [road] });
  physics.reset({ x: 0, y: 0.12, z: 0 }, 0);
  return { physics, road };
}

test('closestPoint keeps the nearest triangle instead of the last in-range triangle', () => {
  const triangleAt = y => [
    { x: -5, y, z: -5 },
    { x: 5, y, z: -5 },
    { x: -5, y, z: 5 }
  ];
  // Deliberately list the near surface first and the farther surface second.
  const mesh = TriangleMesh.fromTriangles([triangleAt(0.1), triangleAt(1.0)], 8);
  const hit = mesh.closestPoint({ x: 0, y: 0.2, z: 0 }, 2, { x: 0, y: 1, z: 0 });
  assert.ok(hit, 'wheel probe should find a surface');
  assert.ok(Math.abs(hit.point.y - 0.1) < 1e-6,
    `expected nearest surface y=0.1, got y=${hit.point.y}`);
});

test('road contact normal is oriented toward the supported car even if triangle winding is inverted', () => {
  const road = makeInvertedFlatRoad();
  const hit = road.closestPoint({ x: 0, y: 0.2, z: 0 }, 1, { x: 0, y: 1, z: 0 });
  assert.ok(hit, 'wheel probe should find the road');
  assert.ok(hit.normal.y > 0.99, `expected upward contact normal, got ${JSON.stringify(hit.normal)}`);
  assert.ok(hit.signedDistance > 0, 'probe above road must have positive signed distance');
});

test('car falling onto road becomes grounded and does not tunnel through the solid surface', () => {
  const { physics } = makePhysics();
  physics.reset({ x: 0, y: 0.48, z: 0 }, 0);
  for (let i = 0; i < 240; i++) physics.advance(1 / 120, {});
  assert.equal(physics.state.grounded, true, 'car should settle onto the road');
  assert.ok(physics.state.position.y > 0.0, `car tunneled below the road: y=${physics.state.position.y}`);
  assert.ok(physics.state.contactNormal.y > 0.9, 'road support normal should point upward');
});

test('left and right inputs turn the car in their named directions on a solid road', () => {
  function runTurn(input) {
    const { physics } = makePhysics();
    physics.state.velocity = { x: 0, y: 0, z: -10 };
    for (let i = 0; i < 24; i++) physics.advance(1 / 120, input);
    const forward = Quaternion.rotate(physics.state.orientation, { x: 0, y: 0, z: -1 });
    return forward.x;
  }
  assert.ok(runTurn({ left: true }) < -0.01, 'left input must steer toward negative X');
  assert.ok(runTurn({ right: true }) > 0.01, 'right input must steer toward positive X');
});
