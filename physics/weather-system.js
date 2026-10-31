/* PlayCar Weather System 1.0.0
 *
 * A deterministic, dependency-free weather model for the 3.0 physics stack.
 * It owns the environment state (surface wetness, rain/snow intensity, fog,
 * cloud cover, temperature, wind, lightning) and feeds the engine:
 *
 *   weather.setPreset('storm');            // smooth 6 s transition
 *   weather.update(dt);                    // wetness soaks / dries over time
 *   weather.applyTo(engine);               // engine.setWeather(...) + wind
 *
 * Physics coupling lives in the engine, not here:
 *   - wetness scales the tire friction coefficient (1 - 0.42 * wetness),
 *   - snow scales it further, cold/hot tarmac trims it slightly,
 *   - wind changes the relative airspeed for drag/downforce and adds a
 *     crosswind side force with a weathervaning yaw moment,
 *   - temperature changes air density and therefore drag/downforce.
 *
 * The model is pure data + timers: no DOM, no Three.js, no Math.random unless
 * you pass your own RNG. That keeps it testable in Node and reusable by both
 * the 3D lab and the proving ground.
 */
(function (root) {
  'use strict';

  const VERSION = '1.0.0';
  const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
  const clamp01 = x => clamp(x, 0, 1);
  const lerp = (a, b, t) => a + (b - a) * t;
  const smoothstep = t => t * t * (3 - 2 * t);

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

  /* Presets. Angles are degrees the wind blows TOWARD, 0 = +Z, 90 = +X.
   * `sun` scales the directional light, `exposure` the tone mapping, `sky` the
   * background/fog colour. Wetness evolves toward the preset value at runtime. */
  const PRESETS = {
    clear: {
      label: '晴朗', wetness: 0, rain: 0, snow: 0, fog: 0.06, cloud: 0.08,
      temperature: 24, windSpeed: 1.5, windAngle: 210, lightning: 0,
      sky: 0x0c1a2a, sun: 1, exposure: 1.05
    },
    overcast: {
      label: '阴天', wetness: 0.18, rain: 0, snow: 0, fog: 0.2, cloud: 0.8,
      temperature: 17, windSpeed: 4, windAngle: 190, lightning: 0,
      sky: 0x1a2836, sun: 0.42, exposure: 0.98
    },
    rain: {
      label: '小雨', wetness: 0.72, rain: 0.55, snow: 0, fog: 0.34, cloud: 0.92,
      temperature: 14, windSpeed: 6, windAngle: 170, lightning: 0,
      sky: 0x16212c, sun: 0.28, exposure: 0.92
    },
    storm: {
      label: '暴雨', wetness: 1, rain: 1, snow: 0, fog: 0.55, cloud: 1,
      temperature: 12, windSpeed: 14, windAngle: 150, lightning: 18,
      sky: 0x0f1720, sun: 0.16, exposure: 0.84
    },
    fog: {
      label: '浓雾', wetness: 0.5, rain: 0.12, snow: 0, fog: 0.95, cloud: 0.9,
      temperature: 10, windSpeed: 2, windAngle: 240, lightning: 0,
      sky: 0x2a3540, sun: 0.24, exposure: 0.96
    },
    snow: {
      label: '降雪', wetness: 0.55, rain: 0, snow: 1, fog: 0.45, cloud: 0.95,
      temperature: -3, windSpeed: 5, windAngle: 200, lightning: 0,
      sky: 0x1b2635, sun: 0.32, exposure: 0.95
    },
    night: {
      label: '夜晚', wetness: 0.2, rain: 0, snow: 0, fog: 0.28, cloud: 0.35,
      temperature: 9, windSpeed: 3, windAngle: 200, lightning: 0,
      sky: 0x050a12, sun: 0.05, exposure: 0.9
    }
  };

  const NUMERIC = ['wetness', 'rain', 'snow', 'fog', 'cloud', 'temperature', 'windSpeed', 'windAngle', 'lightning', 'sun', 'exposure'];

  function presetSnapshot(name) {
    const preset = PRESETS[name];
    if (!preset) throw new Error('PlayCarWeather: unknown preset "' + name + '"');
    return { ...preset };
  }

  class WeatherSystem {
    constructor(options = {}) {
      this.presets = PRESETS;
      this.seed = Number.isFinite(options.seed) ? (options.seed >>> 0) : 0x5eed1;
      this.random = typeof options.random === 'function' ? options.random : mulberry32(this.seed);
      const start = PRESETS[options.preset] ? options.preset : 'clear';
      this.preset = start;
      this.target = presetSnapshot(start);
      this.state = {
        ...presetSnapshot(start),
        preset: start,
        transition: 1,
        flash: 0,
        strikes: 0,
        wind: { x: 0, z: 0 },
        time: 0
      };
      this._boltTimer = 0;
      this._recomputeWind();
    }

    getState() { return this.snapshot(); }

    snapshot() {
      return {
        ...this.state,
        wind: { ...this.state.wind }
      };
    }

    listPresets() {
      return Object.entries(PRESETS).map(([id, p]) => ({ id, label: p.label }));
    }

    /* Blend to a preset. duration = 0 / immediate applies instantly. */
    setPreset(name, options = {}) {
      if (!PRESETS[name]) throw new Error('PlayCarWeather: unknown preset "' + name + '"');
      const duration = Number.isFinite(options.duration) ? Math.max(0, options.duration) : 6;
      this.preset = name;
      this.state.preset = name;
      this.target = presetSnapshot(name);
      if (duration === 0 || options.immediate) {
        for (const key of NUMERIC) this.state[key] = this.target[key];
        this.state.sky = this.target.sky;
        this.state.transition = 1;
      } else {
        this._from = {};
        for (const key of NUMERIC) this._from[key] = this.state[key];
        this._from.sky = this.state.sky;
        this._duration = duration;
        this._elapsed = 0;
        this.state.transition = 0;
      }
      this._recomputeWind();
      return this.snapshot();
    }

    /* Manual override (sliders, scripts, tests). Unknown keys are ignored. */
    set(patch = {}) {
      for (const key of NUMERIC) {
        if (Number.isFinite(patch[key])) this.state[key] = patch[key];
      }
      if (Number.isFinite(patch.sky)) {
        this.state.sky = patch.sky;
        this.target.sky = patch.sky;
        this._from = null;
      }
      if (Number.isFinite(patch.windSpeed) || Number.isFinite(patch.windAngle)) this._recomputeWind();
      this.state.preset = patch.preset || this.preset;
      return this.snapshot();
    }

    update(dt) {
      const step = clamp(Number(dt) || 0, 0, 0.25);
      if (!step) return this.snapshot();
      this.state.time += step;

      /* 1. preset transition: everything except wetness, which soaks/dries. */
      if (this._from) {
        this._elapsed += step;
        const raw = this._duration > 0 ? clamp01(this._elapsed / this._duration) : 1;
        const t = smoothstep(raw);
        for (const key of NUMERIC) this.state[key] = lerp(this._from[key], this.target[key], t);
        this.state.sky = mixHex(this._from.sky, this.target.sky, t);
        this.state.transition = raw;
        if (raw >= 1) {
          this._from = null;
          this.state.transition = 1;
        }
      } else {
        // No transition: keep rain/snow/fog/wind/temperature glued to the preset
        // target, but leave wetness to the surface model below.
        for (const key of NUMERIC) {
          if (key === 'wetness') continue;
          this.state[key] = this.target[key];
        }
        this.state.sky = this.target.sky;
        this.state.transition = 1;
      }

      /* 2. surface water: rain wets, dry weather evaporates. */
      const targetWetness = this._from ? this.state.wetness : this.target.wetness;
      const wetRate = 0.35 + 0.45 * this.state.rain + 0.1 * this.state.snow;
      const dryRate = 0.045 + 0.02 * clamp01(this.state.temperature / 30) + 0.01 * this.state.windSpeed;
      if (targetWetness > this.state.wetness) {
        this.state.wetness = Math.min(targetWetness, this.state.wetness + wetRate * step);
      } else {
        this.state.wetness = Math.max(targetWetness, this.state.wetness - dryRate * step);
      }
      this.state.wetness = clamp01(this.state.wetness);

      /* 3. wind vector (kept in m/s, x/z components). */
      this._recomputeWind();

      /* 4. deterministic lightning + flash decay. */
      this.state.flash *= Math.exp(-step * 7);
      if (this.state.lightning > 0) {
        this._boltTimer -= step;
        if (this._boltTimer <= 0) {
          this.state.flash = 1;
          this.state.strikes += 1;
          const rate = Math.max(0.01, this.state.lightning / 60); // strikes per second
          this._boltTimer = -Math.log(1 - this.random() * 0.999) / rate;
        }
      } else {
        this._boltTimer = 0;
      }
      if (this.state.flash < 1e-4) this.state.flash = 0;
      return this.snapshot();
    }

    _recomputeWind() {
      const angle = (this.state.windAngle * Math.PI) / 180;
      const speed = Math.max(0, this.state.windSpeed);
      this.state.wind = { x: Math.sin(angle) * speed, z: Math.cos(angle) * speed };
    }

    windVector() { return { ...this.state.wind }; }

    /* Effective tire grip multiplier for the current surface. Mirrors the
     * engine mapping so tests can compare model and engine directly. */
    gripFactor(baseGrip = 1) {
      const cold = 1 - Math.min(0.12, Math.max(0, 7 - this.state.temperature) * 0.012);
      const heat = 1 - Math.min(0.06, Math.max(0, this.state.temperature - 32) * 0.004);
      return baseGrip * (1 - 0.42 * this.state.wetness) * (1 - 0.32 * this.state.snow) * cold * heat;
    }

    /* Air density at the current air temperature (sea level, ISA). */
    airDensity() { return 1.225 * (288.15 / (273.15 + clamp(this.state.temperature, -40, 60))); }

    /* Push the state into a unified 3.0 engine. Returns the applied weather. */
    applyTo(engine) {
      if (!engine || typeof engine.setWeather !== 'function') return this.snapshot();
      const state = this.state;
      engine.setWeather({
        wetness: state.wetness,
        wind: { ...state.wind },
        rain: state.rain,
        snow: state.snow,
        fog: state.fog,
        temperature: state.temperature,
        airDensity: this.airDensity()
      });
      return this.snapshot();
    }
  }

  function mixHex(a, b, t) {
    const ar = (a >> 16) & 255, ag = (a >> 8) & 255, ab = a & 255;
    const br = (b >> 16) & 255, bg = (b >> 8) & 255, bb = b & 255;
    return (Math.round(lerp(ar, br, t)) << 16) | (Math.round(lerp(ag, bg, t)) << 8) | Math.round(lerp(ab, bb, t));
  }

  const api = {
    WeatherSystem,
    PRESETS,
    version: VERSION,
    create(options) { return new WeatherSystem(options); }
  };

  root.PlayCarWeather = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(typeof globalThis !== 'undefined' ? globalThis : this);
