'use strict';
/* Headless Three.js test for physics/weather-visuals.js: particles, fog, light
 * dimming, wet materials, lightning and disposal. No WebGL, no DOM. */
const test = require('node:test');
const assert = require('node:assert/strict');
const { createPlayCarWeatherVisuals } = require('../physics/weather-visuals.js');

test('weather visuals animate rain, snow, fog, wet asphalt and lightning', async () => {
  const THREE = await import('../vendor/three.module.min.js');
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0c1a2a);
  scene.fog = new THREE.Fog(0x0c1a2a, 160, 420);
  const sun = new THREE.DirectionalLight(0xffffff, 2.7);
  const hemi = new THREE.HemisphereLight(0xffffff, 0x111111, 1.9);
  const camera = new THREE.PerspectiveCamera(60, 1, 0.1, 500);
  camera.position.set(0, 5, 0);
  const road = new THREE.MeshStandardMaterial({ color: 0x34424e, roughness: 0.9, metalness: 0 });
  const visuals = createPlayCarWeatherVisuals(THREE, scene, { camera, sun, hemi, materials: [road] });

  // Clear day: no particles, bright sun.
  visuals.update(0.1, { wetness: 0, rain: 0, snow: 0, fog: 0.05, sun: 1, wind: { x: 0, z: 0 } });
  assert.equal(visuals.rain.points.visible, false);
  assert.equal(visuals.snow.points.visible, false);
  assert.ok(sun.intensity > 2, 'clear weather keeps the sun strong');

  // Storm: rain, fog, dim sun, glossy road.
  const before = Array.from(visuals.rain.points.geometry.attributes.position.array.slice(0, 12));
  for (let i = 0; i < 90; i++) {
    visuals.update(1 / 60, { wetness: 1, rain: 1, snow: 0, fog: 1, sun: 0.2, wind: { x: 10, z: 0 } });
  }
  const after = Array.from(visuals.rain.points.geometry.attributes.position.array.slice(0, 12));
  assert.equal(visuals.rain.points.visible, true);
  assert.ok(visuals.rain.material.opacity > 0.4, 'rain opacity follows the intensity');
  assert.ok(after.some((value, index) => value !== before[index]), 'rain particles must move');
  assert.ok(scene.fog.near < 40, 'fog closes in, near=' + scene.fog.near);
  assert.ok(scene.fog.far < 150, 'fog far plane=' + scene.fog.far);
  assert.ok(sun.intensity < 1.2, 'sun dims in a storm, got ' + sun.intensity);
  assert.ok(road.roughness < 0.35, 'wet asphalt is glossy, roughness=' + road.roughness);
  assert.ok(road.metalness > 0.2, 'wet asphalt gains sheen, metalness=' + road.metalness);

  // Lightning flash.
  for (let i = 0; i < 30; i++) {
    visuals.update(1 / 60, { wetness: 1, rain: 1, fog: 1, sun: 0.2, flash: 1 });
  }
  assert.ok(visuals.flashLight.intensity > 1, 'flash light fires, got ' + visuals.flashLight.intensity);

  // Snow replaces rain.
  for (let i = 0; i < 180; i++) {
    visuals.update(1 / 60, { wetness: 0.4, rain: 0, snow: 1, fog: 0.4, sun: 0.5, flash: 0 });
  }
  assert.equal(visuals.snow.points.visible, true);
  assert.equal(visuals.rain.points.visible, false);
  assert.ok(visuals.snow.material.opacity > 0.5);

  // Dispose restores the scene's lighting and materials.
  visuals.dispose();
  assert.ok(Math.abs(sun.intensity - 2.7) < 1e-6, 'dispose restores the sun');
  assert.ok(Math.abs(road.roughness - 0.9) < 1e-6, 'dispose restores roughness');
  assert.ok(Math.abs(road.metalness) < 1e-6, 'dispose restores metalness');
  assert.equal(scene.children.includes(visuals.flashLight), false);
});
