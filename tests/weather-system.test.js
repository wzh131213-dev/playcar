'use strict';
/* PlayCar Weather System: presets, transitions, surface water, lightning and
 * the physics mapping. Pure module tests, no browser or Three.js. */
const test = require('node:test');
const assert = require('node:assert/strict');
const { WeatherSystem, PRESETS } = require('../physics/weather-system.js');

function storm(seed = 1) {
  const weather = new WeatherSystem({ preset: 'clear', seed });
  weather.setPreset('storm', { immediate: true });
  return weather;
}

test('weather presets are complete and listed', () => {
  const weather = new WeatherSystem();
  const list = weather.listPresets();
  assert.equal(list.length, 7);
  assert.deepEqual(list.map(p => p.id).sort(), ['clear', 'fog', 'night', 'overcast', 'rain', 'snow', 'storm']);
  for (const preset of list) {
    assert.ok(preset.label.length > 0, preset.id + ' needs a label');
    const state = PRESETS[preset.id];
    for (const key of ['wetness', 'rain', 'snow', 'fog', 'cloud', 'temperature', 'windSpeed', 'windAngle']) {
      assert.ok(Number.isFinite(state[key]), `${preset.id}.${key} must be finite`);
    }
  }
  assert.throws(() => weather.setPreset('hurricane'), /unknown preset/);
});

test('setPreset with immediate applies every field and the wind vector', () => {
  const weather = new WeatherSystem();
  weather.setPreset('storm', { immediate: true });
  const state = weather.getState();
  assert.equal(state.preset, 'storm');
  assert.equal(state.wetness, 1);
  assert.equal(state.rain, 1);
  assert.equal(state.temperature, 12);
  assert.equal(state.transition, 1);
  const angle = 150 * Math.PI / 180;
  assert.ok(Math.abs(state.wind.x - Math.sin(angle) * 14) < 1e-9);
  assert.ok(Math.abs(state.wind.z - Math.cos(angle) * 14) < 1e-9);
  // getState must be a copy, not a live reference.
  state.wind.x = 999;
  assert.ok(Math.abs(weather.getState().wind.x - Math.sin(angle) * 14) < 1e-9);
});

test('preset changes blend smoothly over the requested duration', () => {
  const weather = new WeatherSystem({ preset: 'clear' });
  weather.setPreset('rain', { duration: 4 });
  const start = weather.getState();
  assert.equal(start.transition, 0);
  assert.equal(start.wetness, 0);
  for (let i = 0; i < 20; i++) weather.update(0.1);
  const mid = weather.getState();
  assert.ok(mid.transition > 0.4 && mid.transition < 0.6, 'transition at half time, got ' + mid.transition);
  assert.ok(mid.rain > 0.2 && mid.rain < 0.4, 'rain should be half blended, got ' + mid.rain);
  assert.ok(mid.wetness > 0 && mid.wetness < 0.72, 'surface water soaks in gradually, got ' + mid.wetness);
  for (let i = 0; i < 600; i++) weather.update(1 / 60);
  const end = weather.getState();
  assert.equal(end.transition, 1);
  assert.ok(Math.abs(end.rain - 0.55) < 1e-9);
  assert.ok(end.wetness > 0.7, 'rain must wet the surface, got ' + end.wetness);
});

test('rain soaks the surface and clear weather dries it again', () => {
  const weather = new WeatherSystem({ preset: 'clear' });
  weather.setPreset('rain', { immediate: true });
  for (let i = 0; i < 120; i++) weather.update(1 / 10);
  assert.ok(weather.getState().wetness > 0.6, 'twelve seconds of rain should soak the track');
  weather.setPreset('clear', { duration: 0.5 });
  const soaked = weather.getState().wetness;
  for (let i = 0; i < 600; i++) weather.update(1 / 10);
  const dried = weather.getState().wetness;
  assert.ok(dried < soaked * 0.35, `drying must remove water, ${soaked.toFixed(2)} -> ${dried.toFixed(2)}`);
});

test('lightning is deterministic for a given seed', () => {
  const a = storm(42);
  const b = storm(42);
  const c = storm(7);
  let strikesA = 0, strikesB = 0, strikesC = 0;
  for (let i = 0; i < 60 * 120; i++) {
    strikesA = a.update(1 / 60).strikes;
    strikesB = b.update(1 / 60).strikes;
    strikesC = c.update(1 / 60).strikes;
  }
  assert.ok(strikesA > 5, 'two minutes of storm should produce lightning, got ' + strikesA);
  assert.equal(strikesA, strikesB, 'same seed must produce identical strikes');
  assert.notEqual(strikesA, strikesC, 'different seeds should diverge');
});

test('grip factor matches the engine weather mapping', () => {
  const weather = new WeatherSystem({ preset: 'clear' });
  assert.ok(Math.abs(weather.gripFactor(1) - 1) < 1e-9);
  weather.set({ wetness: 1 });
  assert.ok(Math.abs(weather.gripFactor(1) - 0.58) < 1e-9, 'wet track: 1 - 0.42');
  weather.set({ wetness: 1, snow: 1 });
  assert.ok(Math.abs(weather.gripFactor(1) - 0.58 * 0.68) < 1e-9, 'snow: 1 - 0.32');
  weather.set({ wetness: 0, snow: 0, temperature: -10 });
  assert.ok(Math.abs(weather.gripFactor(1) - 0.88) < 1e-9, 'cold tarmac loses 12%');
  weather.set({ temperature: 40 });
  assert.ok(Math.abs(weather.gripFactor(1) - 0.968) < 1e-9, 'hot tarmac loses 3.2%');
});

test('air density follows the air temperature', () => {
  const weather = new WeatherSystem();
  weather.set({ temperature: 0 });
  assert.ok(Math.abs(weather.airDensity() - 1.292) < 0.005);
  weather.set({ temperature: 30 });
  assert.ok(Math.abs(weather.airDensity() - 1.164) < 0.005);
});

test('applyTo pushes wetness, wind, snow and temperature into the engine', () => {
  const weather = storm(3);
  weather.set({ temperature: 5, snow: 1 });
  const calls = [];
  const engine = { setWeather(patch) { calls.push(patch); return patch; } };
  const state = weather.applyTo(engine);
  assert.equal(calls.length, 1);
  const patch = calls[0];
  assert.ok(Math.abs(patch.wetness - state.wetness) < 1e-12);
  assert.deepEqual(patch.wind, state.wind);
  assert.equal(patch.snow, 1);
  assert.equal(patch.temperature, 5);
  assert.ok(Number.isFinite(patch.airDensity));
  assert.deepEqual(weather.applyTo(null), state);
});
