/* PlayCar Weather Visuals 1.0.0
 *
 * Three.js presentation layer for physics/weather-system.js. It does not own
 * any physics: pass it the weather state each frame and it renders:
 *
 *   - rain / snow particles that follow the camera and slant with the wind,
 *   - sky and fog colour/density, sun + hemisphere dimming, exposure,
 *   - wet-surface roughness/metalness on the materials you hand it,
 *   - deterministic lightning flashes (ambient light + sky flash).
 *
 * Usage:
 *   const visuals = createPlayCarWeatherVisuals(THREE, scene, {
 *     camera, sun, hemi, renderer, materials: [roadMaterial]
 *   });
 *   // each frame:
 *   visuals.update(dt, weather.getState(), camera);
 *
 * The module is a classic script (no imports) so it can be concatenated into
 * the single-file bundle and also loaded with a plain <script> tag by the
 * standalone proving-ground page. In Node/headless it still builds real
 * three.js objects; only WebGL rendering is stubbed by the smoke harness.
 */
(function (root) {
  'use strict';

  const VERSION = '1.0.0';
  const clamp01 = x => Math.max(0, Math.min(1, Number(x) || 0));
  const lerp = (a, b, t) => a + (b - a) * t;

  function mulberry32(seed) {
    let a = seed >>> 0;
    return function random() {
      a = (a + 0x6D2B79F5) >>> 0;
      let t = a;
      t = Math.imul(t ^ (t >>> 15), t | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  function createPlayCarWeatherVisuals(THREE, scene, options = {}) {
    if (!THREE || !scene) throw new Error('createPlayCarWeatherVisuals: THREE and scene are required');
    const camera = options.camera || null;
    const renderer = options.renderer || null;
    const sun = options.sun || null;
    const hemi = options.hemi || null;
    const materials = Array.isArray(options.materials) ? options.materials.filter(Boolean) : [];
    const seed = Number.isFinite(options.seed) ? options.seed : 0x9e3779b9;
    const random = mulberry32(seed);

    const baseSun = sun ? sun.intensity : 1;
    const baseHemi = hemi ? hemi.intensity : 1;
    const baseExposure = renderer && Number.isFinite(renderer.toneMappingExposure) ? renderer.toneMappingExposure : 1;
    const baseMaterials = materials.map(material => ({
      material,
      roughness: Number.isFinite(material.roughness) ? material.roughness : 0.9,
      metalness: Number.isFinite(material.metalness) ? material.metalness : 0,
      color: material.color && material.color.isColor ? material.color.clone() : null
    }));

    // Keep a private fog so we never overwrite a scene that owns its own.
    let fog = scene.fog;
    if (!fog && THREE.Fog) {
      fog = new THREE.Fog(0x0c1a2a, 160, 420);
      scene.fog = fog;
    }
    const fogBase = fog ? { near: fog.near, far: fog.far } : { near: 160, far: 420 };
    if (!scene.background || !scene.background.isColor) {
      scene.background = new THREE.Color(0x0c1a2a);
    }
    const skyBase = scene.background.clone();

    const flashLight = new THREE.AmbientLight(0xd7e6ff, 0);
    flashLight.name = 'PlayCarWeatherFlash';
    scene.add(flashLight);

    const RAIN_COUNT = options.rainCount || 1200;
    const SNOW_COUNT = options.snowCount || 700;
    const BOX = { x: 70, y: 38, z: 70 };

    function makeParticles(count, color, size, opacity) {
      const positions = new Float32Array(count * 3);
      for (let i = 0; i < count; i++) {
        positions[i * 3] = (random() - 0.5) * BOX.x;
        positions[i * 3 + 1] = random() * BOX.y;
        positions[i * 3 + 2] = (random() - 0.5) * BOX.z;
      }
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
      const material = new THREE.PointsMaterial({
        color, size, sizeAttenuation: true, transparent: true, opacity,
        depthWrite: false, fog: false
      });
      const points = new THREE.Points(geometry, material);
      points.frustumCulled = false;
      points.visible = false;
      points.name = 'PlayCarWeatherParticles';
      scene.add(points);
      return { points, positions, count, material };
    }

    const rain = makeParticles(RAIN_COUNT, 0xa9d4ee, 0.085, 0.55);
    const snow = makeParticles(SNOW_COUNT, 0xffffff, 0.17, 0.85);

    const target = { wet: 0, rain: 0, snow: 0, fog: 0, sun: 1, flash: 0, sky: skyBase.getHex() };
    const current = { wet: 0, rain: 0, snow: 0, fog: 0, sun: 1, flash: 0, sky: skyBase.getHex() };
    const wind = { x: 0, z: 0 };
    const tmpColor = new THREE.Color();
    let disposed = false;

    function normalize(state = {}) {
      const wet = clamp01(state.wetness);
      const rainAmount = state.rain === undefined ? wet : clamp01(state.rain);
      const snowAmount = clamp01(state.snow);
      const fogAmount = state.fog === undefined ? clamp01(wet * 0.6) : clamp01(state.fog);
      const sunScale = state.sun === undefined ? 1 : Math.max(0, Math.min(2, state.sun));
      return {
        wet, rain: rainAmount, snow: snowAmount, fog: fogAmount,
        sun: sunScale,
        flash: clamp01(state.flash),
        sky: Number.isFinite(state.sky) ? state.sky : null,
        wind: state.wind || { x: 0, z: 0 }
      };
    }

    function stepParticles(system, dt, fallSpeed, driftScale) {
      const { positions, count } = system;
      const cx = camera ? camera.position.x : (options.center ? options.center.x : 0);
      const cy = camera ? camera.position.y : (options.center ? options.center.y : 0);
      const cz = camera ? camera.position.z : (options.center ? options.center.z : 0);
      const halfX = BOX.x / 2, halfZ = BOX.z / 2;
      for (let i = 0; i < count; i++) {
        const ix = i * 3;
        positions[ix] += wind.x * driftScale * dt;
        positions[ix + 1] -= fallSpeed * dt;
        positions[ix + 2] += wind.z * driftScale * dt;
        // Wrap around the camera so the field never empties.
        if (positions[ix + 1] < -BOX.y * 0.5) {
          positions[ix + 1] += BOX.y;
          positions[ix] = cx + (random() - 0.5) * BOX.x;
          positions[ix + 2] = cz + (random() - 0.5) * BOX.z;
        }
        if (positions[ix] - cx > halfX) positions[ix] -= BOX.x;
        else if (positions[ix] - cx < -halfX) positions[ix] += BOX.x;
        if (positions[ix + 2] - cz > halfZ) positions[ix + 2] -= BOX.z;
        else if (positions[ix + 2] - cz < -halfZ) positions[ix + 2] += BOX.z;
        if (positions[ix + 1] > cy + BOX.y * 0.6) positions[ix + 1] -= BOX.y;
      }
      system.points.geometry.attributes.position.needsUpdate = true;
    }

    function update(dt, state) {
      if (disposed) return;
      const step = Math.max(0, Math.min(0.25, Number(dt) || 0));
      const input = normalize(state || {});
      const k = Math.min(1, step * 2.2);
      target.wet = input.wet; target.rain = input.rain; target.snow = input.snow;
      target.fog = input.fog; target.sun = input.sun; target.flash = input.flash;
      if (input.sky !== null) target.sky = input.sky;
      wind.x = input.wind.x || 0;
      wind.z = input.wind.z || 0;
      for (const key of ['wet', 'rain', 'snow', 'fog', 'sun', 'flash']) current[key] += (target[key] - current[key]) * k;
      current.sky = mixHex(current.sky, target.sky, k);

      // Sky and fog.
      skyBase.setHex(current.sky);
      if (scene.background && scene.background.isColor) {
        const flashBoost = current.flash * 0.55;
        scene.background.copy(skyBase).lerp(tmpColor.setHex(0xdfe9ff), flashBoost);
      }
      if (fog && fog.color) {
        fog.color.setHex(current.sky);
        fog.near = lerp(fogBase.near, 14, current.fog);
        fog.far = lerp(fogBase.far, 85, current.fog);
      }

      // Lights: dim with cloud cover/sun scale, flash with lightning.
      if (sun) sun.intensity = baseSun * current.sun + current.flash * 1.4;
      if (hemi) hemi.intensity = baseHemi * (0.35 + 0.65 * current.sun) + current.flash * 2.2;
      flashLight.intensity = current.flash * 3.2;
      if (renderer && Number.isFinite(renderer.toneMappingExposure)) {
        renderer.toneMappingExposure = baseExposure * lerp(1, 0.92, current.wet) * (1 + current.flash * 0.25);
      }

      // Wet tarmac.
      for (const entry of baseMaterials) {
        const material = entry.material;
        if (Number.isFinite(material.roughness)) {
          material.roughness = lerp(entry.roughness, 0.18, current.wet);
        }
        if (Number.isFinite(material.metalness)) {
          material.metalness = lerp(entry.metalness, 0.32, current.wet);
        }
        if (entry.color && material.color) {
          material.color.copy(entry.color).multiplyScalar(1 - 0.12 * current.wet);
        }
      }

      // Particles (they follow the camera captured at construction time).
      rain.points.visible = current.rain > 0.02;
      rain.material.opacity = 0.62 * current.rain;
      snow.points.visible = current.snow > 0.02;
      snow.material.opacity = 0.9 * current.snow;
      if (camera && rain.points.visible) stepParticles(rain, step, 22, 0.85);
      if (camera && snow.points.visible) stepParticles(snow, step, 1.9, 1.25);
      return snapshot();
    }

    function mixHex(a, b, t) {
      const ar = (a >> 16) & 255, ag = (a >> 8) & 255, ab = a & 255;
      const br = (b >> 16) & 255, bg = (b >> 8) & 255, bb = b & 255;
      return (Math.round(lerp(ar, br, t)) << 16) | (Math.round(lerp(ag, bg, t)) << 8) | Math.round(lerp(ab, bb, t));
    }

    function snapshot() {
      return {
        wetness: current.wet, rain: current.rain, snow: current.snow, fog: current.fog,
        sun: current.sun, flash: current.flash, sky: current.sky, wind: { ...wind }
      };
    }

    function dispose() {
      if (disposed) return;
      disposed = true;
      scene.remove(flashLight);
      for (const system of [rain, snow]) {
        scene.remove(system.points);
        system.points.geometry.dispose();
        system.material.dispose();
      }
      for (const entry of baseMaterials) {
        if (Number.isFinite(entry.roughness)) entry.material.roughness = entry.roughness;
        if (Number.isFinite(entry.metalness)) entry.material.metalness = entry.metalness;
        if (entry.color && entry.material.color) entry.material.color.copy(entry.color);
      }
      if (sun) sun.intensity = baseSun;
      if (hemi) hemi.intensity = baseHemi;
      if (renderer && Number.isFinite(renderer.toneMappingExposure)) renderer.toneMappingExposure = baseExposure;
    }

    return {
      version: VERSION,
      update,
      dispose,
      snapshot,
      setCamera(value) { camera = value; },
      rain, snow, flashLight,
      get state() { return snapshot(); }
    };
  }

  root.createPlayCarWeatherVisuals = createPlayCarWeatherVisuals;
  root.PlayCarWeatherVisuals = { create: createPlayCarWeatherVisuals, version: VERSION };
  if (typeof module !== 'undefined' && module.exports) module.exports = { createPlayCarWeatherVisuals, version: VERSION };
})(typeof globalThis !== 'undefined' ? globalThis : this);
