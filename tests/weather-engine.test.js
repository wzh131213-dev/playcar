'use strict';
/* Weather → physics coupling in the unified 3.0 engine: wind produces real
 * aerodynamic forces, snow/cold change the tire friction coefficient, and the
 * weather state survives into snapshots. Runs the generated bundle. */
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const root = path.join(__dirname, '..');

function loadEngine() {
  const bundle = fs.readFileSync(path.join(root, 'physics', 'playcar-physics-all-in-one.js'), 'utf8');
  const marker = bundle.indexOf('(async function startPlayCar3DScene');
  assert.ok(marker > 0, 'bundle scene marker missing');
  const context = { console, performance: globalThis.performance };
  vm.createContext(context);
  vm.runInContext(bundle.slice(0, marker), context, { filename: 'playcar-physics-prefix.js' });
  return context.PlayCarPhysicsEngine;
}

function flatSurface(API, half = 300) {
  return API.TriangleMesh.fromTriangles([
    [{ x: -half, y: 0, z: half }, { x: -half, y: 0, z: -half }, { x: half, y: 0, z: -half }, 1],
    [{ x: -half, y: 0, z: half }, { x: half, y: 0, z: -half }, { x: half, y: 0, z: half }, 1]
  ], 8);
}

function makeCar(API, weather) {
  const car = new API.PlayCarPhysicsEngine({
    surfaceMesh: flatSurface(API), groundPlaneY: 0, position: { x: 0, y: 0.08, z: 0 },
    preset: 'simcade', config: { tireGrip: 1, maxSpeed: 60 }
  });
  if (weather) car.setWeather(weather);
  return car;
}

test('crosswind produces a side force and turns the car into the wind', () => {
  const API = loadEngine();
  const calm = makeCar(API);
  const windy = makeCar(API, { wind: { x: 15, z: 0 }, wetness: 0 });
  for (const car of [calm, windy]) {
    car.rigidBody.state.velocity = { x: 0, y: 0, z: -20 };
    car.rigidBody.previous.velocity = { x: 0, y: 0, z: -20 };
    car._legacyKey = '';
    for (let i = 0; i < 240; i++) car.advance(1 / 120, { throttle: 0.2, steer: 0 });
  }
  const calmTelemetry = calm.rigidBody.telemetry;
  const windyTelemetry = windy.rigidBody.telemetry;
  assert.ok(Math.abs(calmTelemetry.sideForce) < 1, 'no wind, no side force');
  assert.ok(windyTelemetry.sideForce > 150, 'crosswind side force, got ' + windyTelemetry.sideForce.toFixed(0) + ' N');
  assert.ok(Math.abs(windyTelemetry.crosswind + 15) < 0.5, 'relative lateral airspeed');
  assert.ok(Math.abs(windyTelemetry.wind.x - 15) < 1e-9, 'wind is exposed in telemetry');
  const drift = Math.abs(windy.rigidBody.state.position.x - calm.rigidBody.state.position.x);
  assert.ok(drift > 0.05, 'the car must respond to the wind, drift ' + drift.toFixed(3) + ' m');
});

test('headwind slows the car and tailwind speeds it up', () => {
  const API = loadEngine();
  const speed = weather => {
    const car = makeCar(API, weather);
    for (let i = 0; i < 600; i++) car.advance(1 / 60, { throttle: 1 });
    return Math.hypot(car.rigidBody.state.velocity.x, car.rigidBody.state.velocity.z);
  };
  const calm = speed();
  const headwind = speed({ wind: { x: 0, z: 15 } });
  const tailwind = speed({ wind: { x: 0, z: -15 } });
  assert.ok(headwind < calm - 1, `headwind ${headwind.toFixed(1)} vs calm ${calm.toFixed(1)}`);
  assert.ok(tailwind > calm + 0.5, `tailwind ${tailwind.toFixed(1)} vs calm ${calm.toFixed(1)}`);
});

test('snow and temperature lower the tire friction coefficient', () => {
  const API = loadEngine();
  const dry = makeCar(API, { wetness: 0, snow: 0, temperature: 20 });
  const snowy = makeCar(API, { wetness: 0, snow: 1, temperature: 20 });
  const cold = makeCar(API, { wetness: 0, snow: 0, temperature: -10 });
  const wet = makeCar(API, { wetness: 1, snow: 0, temperature: 20 });
  assert.ok(Math.abs(dry.rigidBody.c.tire.mu - 1) < 1e-9);
  assert.ok(Math.abs(wet.rigidBody.c.tire.mu - 0.58) < 1e-9);
  assert.ok(Math.abs(snowy.rigidBody.c.tire.mu - 0.68) < 1e-9);
  assert.ok(Math.abs(cold.rigidBody.c.tire.mu - 0.88) < 1e-9);
});

test('weather forwards wind and air density into the rigid body', () => {
  const API = loadEngine();
  const car = makeCar(API, { wind: { x: 12, z: -4 }, temperature: 30 });
  assert.equal(car.rigidBody.c.wind.x, 12);
  assert.equal(car.rigidBody.c.wind.z, -4);
  assert.ok(Math.abs(car.rigidBody.c.airDensity - 1.164) < 0.005, 'warm air is less dense');
  const snapshot = car.getSnapshot();
  assert.equal(snapshot.weather.wind.x, 12);
  assert.equal(snapshot.weather.temperature, 30);
  assert.equal(typeof snapshot.weather.snow, 'number');
});
