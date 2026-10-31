/* PlayCar unified runtime 3.0.0
 * Presents the legacy PlayCarPhysicsEngine API on top of the rigid-body
 * vehicle (raycast suspension + combined-slip tires + core collision solver).
 *
 * The legacy engine still owns weather, checkpoints, input normalization and
 * the simulation clock; the rigid-body vehicle owns motion and contacts.
 */
(function (root) {
  'use strict';

  const V = {
    add: (a, b) => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z }),
    sub: (a, b) => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z }),
    mul: (a, s) => ({ x: a.x * s, y: a.y * s, z: a.z * s }),
    dot: (a, b) => a.x * b.x + a.y * b.y + a.z * b.z,
    len: a => Math.hypot(a.x, a.y, a.z),
    clamp: (x, a, b) => Math.max(a, Math.min(b, x)),
    make: (x = 0, y = 0, z = 0) => ({ x, y, z })
  };
  const Q = {
    rotate: (q, v) => {
      const u = { x: q.x, y: q.y, z: q.z };
      const t = V.mul(V.cross(u, v), 2);
      return V.add(v, V.add(V.mul(t, q.w), V.cross(u, t)));
    },
    slerp: (a, b, t) => {
      let d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
      if (d < 0) { b = { x: -b.x, y: -b.y, z: -b.z, w: -b.w }; d = -d; }
      if (d > 0.9995) {
        const n = Math.hypot(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t) || 1;
        return { x: (a.x + (b.x - a.x) * t) / n, y: (a.y + (b.y - a.y) * t) / n, z: (a.z + (b.z - a.z) * t) / n, w: (a.w + (b.w - a.w) * t) / n };
      }
      const th = Math.acos(V.clamp(d, -1, 1)), s = Math.sin(th);
      const u = Math.sin((1 - t) * th) / s, v = Math.sin(t * th) / s;
      return { x: a.x * u + b.x * v, y: a.y * u + b.y * v, z: a.z * u + b.z * v, w: a.w * u + b.w * v };
    },
    inverse: q => ({ x: -q.x, y: -q.y, z: -q.z, w: q.w })
  };

  const LegacyAPI = root.PlayCarPhysicsEngine;
  const RigidAPI = root.PlayCarRigidBody;
  if (!LegacyAPI || !LegacyAPI.PlayCarPhysicsEngine) {
    throw new Error('PlayCar unified runtime: playcar-physics-engine.js must be loaded first');
  }
  if (!RigidAPI || !RigidAPI.RigidBodyVehicle) {
    throw new Error('PlayCar unified runtime: rigidbody-vehicle-engine.js must be loaded first');
  }

  const LegacyPlayCarPhysicsEngine = LegacyAPI.PlayCarPhysicsEngine;
  const DEFAULT_BODY_OFFSET = 0.36;

  const UNIFIED_VERSION = '3.0.0';
const nowMs = () => (typeof performance !== 'undefined' && typeof performance.now === 'function')
  ? performance.now() : Date.now();

  class UnifiedPlayCarPhysicsEngine extends LegacyPlayCarPhysicsEngine {
    constructor(options = {}) {
      super(options);
      this.solver = 'rigid-body-' + UNIFIED_VERSION;
      this.bodyOffset = Number.isFinite(options.bodyOffset) ? options.bodyOffset : DEFAULT_BODY_OFFSET;
      this._colliderRef = null;
      this._legacyKey = '';

      const spawn = {
        x: this.state.position.x,
        y: this.state.position.y + this.bodyOffset,
        z: this.state.position.z
      };
      this.rigidBody = new RigidAPI.RigidBodyVehicle({
        surfaceMesh: this.surfaceMesh,
        colliderMeshes: this.colliderMeshes,
        groundPlaneY: this.groundPlaneY,
        position: spawn,
        orientation: this.state.orientation,
        config: this._buildRigidConfig(options.config || {})
      });
      this._colliderRef = this.colliderMeshes;
      this.rigidBody.state.orientation = { ...this.state.orientation };
      this.rigidBody.previous.orientation = { ...this.state.orientation };
      this._syncRigidToLegacy();
    }

    /* ---------------- legacy -> rigid configuration ---------------- */

    _buildRigidConfig(extra = {}) {
      const c = this.config;
      const mass = 1250;
      const wheelRadius = 0.37;
      const finalDrive = 3.9, firstGear = 3.2, efficiency = 0.88;
      const wetGrip = c.tireGrip * (1 - 0.42 * this.weather.wetness);
      const peakTorque = c.engineAcceleration * mass * wheelRadius / (firstGear * finalDrive * efficiency);
      const base = {
        fixedDt: c.fixedTimeStep,
        maxSubsteps: c.maxSubsteps,
        mass,
        gravity: c.gravity,
        wheelbase: c.wheelbase,
        track: c.trackHalfWidth * 2,
        wheelRadius,
        suspension: {
          // The 1.x game-unit default was 0.09, which maps to a car sitting on
          // its bump stops in metre units. Keep a sane minimum ride height while
          // preserving any larger, deliberately tuned value.
          rest: Math.max(0.22, c.suspensionRestLength),
          min: 0.08,
          max: Math.max(Math.max(0.22, c.suspensionRestLength) + 0.1, c.suspensionReach),
          stiffness: 32000,
          damping: 4200,
          dampingBump: 4200,
          dampingRebound: 5600,
          progressive: 0.35,
          bump: 90000
        },
        airControl: {
          yawRate: c.airYawRate, pitchRate: c.airPitchRate, rollRate: c.airRollRate,
          yawGain: 2200, pitchGain: 3000, rollGain: 3000
        },
        antiRoll: { front: 8500, rear: 6500 },
        differential: { type: 'lsd', layout: 'rwd', limitedSlip: 2.2 },
        aids: { abs: true, tcs: true, esc: false, absSlip: 0.18, tcsSlip: 0.22 },
        steering: { maxAngle: c.maxSteerAngle, highSpeedScale: c.highSpeedSteerScale },
        tire: {
          mu: wetGrip,
          corneringStiffness: 78000,
          longitudinalStiffness: 95000,
          relaxation: 0.04,
          relaxationAngle: 0.08,
          relaxationLength: 0.35,
          combined: { enabled: true, C: 1.35, E: 0.92, peak: 1.0 }
        },
        engine: {
          peakTorque,
          idleRpm: 900,
          redline: 6500,
          finalDrive,
          gears: [3.2, 2.1, 1.5, 1.18, 0.94, 0.78],
          efficiency,
          shiftTime: 0.16,
          engineBraking: 46
        },
        brakeTorque: c.brakeAcceleration * mass * wheelRadius / 4,
        handbrakeTorque: c.brakeAcceleration * mass * wheelRadius / 4 * 1.55,
        dragArea: Math.max(0.05, c.aerodynamicDrag * 300),
        airDensity: 1.225,
        downforceArea: c.maxDownforce * 2 / (1.225 * Math.max(1, c.maxSpeed * c.maxSpeed)),
        restitution: c.collisionRestitution,
        collisionFriction: c.collisionFriction,
        bodyHalf: { x: c.bodyHalfExtents.x, y: 0.32, z: Math.min(c.bodyHalfExtents.z, 1.5) },
        ccd: true,
        solver: { velocityIterations: 10, positionIterations: 4 }
      };
      return base;
    }

    _legacyKeyString() {
      const c = this.config;
      return [
        c.gravity, c.tireGrip, this.weather.wetness, this.weather.snow, this.weather.temperature, c.fixedTimeStep, c.maxSubsteps,
        c.wheelbase, c.trackHalfWidth, c.maxSteerAngle, c.highSpeedSteerScale,
        c.brakeAcceleration, c.engineAcceleration, c.aerodynamicDrag, c.maxDownforce,
        c.maxSpeed, c.suspensionRestLength, c.suspensionReach, c.collisionRestitution,
        c.collisionFriction, c.bodyHalfExtents.x, c.bodyHalfExtents.z
      ].join('|');
    }

    _applyLegacyConfig() {
      if (!this.rigidBody) return;
      const key = this._legacyKeyString();
      if (key === this._legacyKey) return;
      this._legacyKey = key;
      const c = this.config, r = this.rigidBody, rc = r.c;
      const mass = 1250, wheelRadius = r.c.wheelRadius;
      rc.tire.mu = c.tireGrip * this._weatherGripFactor();
      rc.gravity = c.gravity;
      rc.fixedDt = c.fixedTimeStep;
      rc.maxSubsteps = c.maxSubsteps;
      rc.wheelbase = c.wheelbase;
      rc.track = c.trackHalfWidth * 2;
      rc.steering.maxAngle = c.maxSteerAngle;
      rc.steering.highSpeedScale = c.highSpeedSteerScale;
      if (rc.airControl) {
        rc.airControl.yawRate = c.airYawRate;
        rc.airControl.pitchRate = c.airPitchRate;
        rc.airControl.rollRate = c.airRollRate;
      }
      rc.brakeTorque = c.brakeAcceleration * mass * wheelRadius / 4;
      rc.handbrakeTorque = rc.brakeTorque * 1.55;
      rc.engine.peakTorque = c.engineAcceleration * mass * wheelRadius / (3.2 * 3.9 * 0.88);
      rc.dragArea = Math.max(0.05, c.aerodynamicDrag * 300);
      rc.downforceArea = c.maxDownforce * 2 / (1.225 * Math.max(1, c.maxSpeed * c.maxSpeed));
      rc.suspension.rest = Math.max(0.22, c.suspensionRestLength);
      rc.suspension.max = Math.max(rc.suspension.rest + 0.1, c.suspensionReach);
      rc.restitution = c.collisionRestitution;
      rc.collisionFriction = c.collisionFriction;
      rc.bodyHalf = { x: c.bodyHalfExtents.x, y: 0.32, z: Math.min(c.bodyHalfExtents.z, 1.5) };
      if (r.chassis && r.chassis.shape.type === 'box') {
        r.chassis.shape.halfExtents = { ...rc.bodyHalf };
        r.chassis.shape.boundingRadius = V.len(rc.bodyHalf);
      }
      if (r.world) {
        r.world.settings.gravity = { x: 0, y: -c.gravity, z: 0 };
        r.world.settings.fixedDt = c.fixedTimeStep;
      }
    }

    /* ---------------- state synchronisation ---------------- */

    _gripState(rigid) {
      if (!rigid.grounded) return 'AIR';
      const t = rigid.telemetry || {};
      const handbrake = (this.lastInput.handbrake || 0) > 0;
      const slipping = (t.forces || []).some(f => Math.abs(f.slipRatio) > 0.22 || Math.abs(f.slipAngle) > 0.22);
      if (handbrake || (slipping && V.len(rigid.state.velocity) > 6)) return 'DRIFT';
      if (slipping) return 'SLIDE';
      return 'GRIP';
    }

    _syncRigidToLegacy() {
      const r = this.rigidBody, s = this.state, off = this.bodyOffset;
      const chassisImpacts = (r.world.events.beginContact || [])
        .filter(e => e.bodyA === r.chassis || e.bodyB === r.chassis).length;
      s.position = { x: r.state.position.x, y: r.state.position.y - off, z: r.state.position.z };
      s.velocity = { ...r.state.velocity };
      s.orientation = { ...r.state.orientation };
      s.angularVelocity = { ...r.state.angularVelocity };
      s.grounded = !!r.state.grounded;
      s.gripState = this._gripState({ ...r.state, state: r.state, telemetry: r.telemetry });
      s.tick = r.tick;
      s.collisionState = r.telemetry.chassisContacts > 0 ? 'wall' : (s.grounded ? 'track' : 'none');
      s.collisionCount = r.telemetry.chassisContacts || 0;
      s.collisionTimer = chassisImpacts > 0 ? 0.35 : Math.max(0, (s.collisionTimer || 0) - 1 / 120);
      s.totalCollisionEvents = (s.totalCollisionEvents || 0) + chassisImpacts;
      this.tick = r.tick;
      this.telemetry = {
        ...this.telemetry,
        speed: V.len(s.velocity),
        downforce: r.telemetry.downforce || 0,
        airborne: !s.grounded,
        wheelContacts: r.telemetry.contacts || 0
      };
    }

    /* ---------------- public API ---------------- */

    setSurfaceMesh(mesh) {
      super.setSurfaceMesh(mesh);
      if (this.rigidBody) this.rigidBody.setSurfaceMesh(mesh);
      return this;
    }

    setColliderMeshes(meshes = []) {
      super.setColliderMeshes(meshes);
      this._colliderRef = this.colliderMeshes;
      if (this.rigidBody) this.rigidBody.setColliderMeshes(this.colliderMeshes);
      return this;
    }

    addColliderMesh(mesh) {
      super.addColliderMesh(mesh);
      this._colliderRef = this.colliderMeshes;
      if (this.rigidBody) this.rigidBody.setColliderMeshes(this.colliderMeshes);
      return this;
    }

    setConfig(patch = {}) {
      const out = super.setConfig(patch);
      this._applyLegacyConfig();
      return out;
    }

    /* Combined grip multiplier for the current weather: surface water, snow
     * cover and tarmac temperature. Mirrors PlayCarWeather.gripFactor(). */
    _weatherGripFactor() {
      const w = this.weather || {};
      const wet = Number.isFinite(w.wetness) ? w.wetness : 0;
      const snow = Number.isFinite(w.snow) ? w.snow : 0;
      const temp = Number.isFinite(w.temperature) ? w.temperature : 20;
      const cold = 1 - Math.min(0.12, Math.max(0, 7 - temp) * 0.012);
      const heat = 1 - Math.min(0.06, Math.max(0, temp - 32) * 0.004);
      return (1 - 0.42 * wet) * (1 - 0.32 * snow) * cold * heat;
    }

    setWeather(weather = {}) {
      const out = super.setWeather(weather);
      this._applyLegacyConfig();
      if (this.rigidBody) {
        const r = this.rigidBody, w = this.weather;
        r.c.wind = { x: w.wind.x, y: 0, z: w.wind.z };
        r.c.airDensity = Number.isFinite(weather.airDensity)
          ? Math.max(0.8, Math.min(1.5, weather.airDensity))
          : 1.225 * (288.15 / (273.15 + w.temperature));
        r.c.tire.mu = this.config.tireGrip * this._weatherGripFactor();
        this._legacyKey = this._legacyKeyString();
      }
      return out;
    }

    setDifferential(type = 'lsd', layout = 'rwd', limitedSlip = 2.2) {
      if (!['open', 'lsd', 'locked'].includes(type) ||
          !['fwd', 'rwd', 'awd'].includes(layout) ||
          !Number.isFinite(limitedSlip) || limitedSlip < 1) {
        throw new TypeError('Invalid differential settings');
      }
      this.rigidBody.c.differential = { ...this.rigidBody.c.differential, type, layout, limitedSlip };
      this.rigidBody._layout = null;
      return { type, layout, limitedSlip };
    }

    setRigidConfig(patch = {}) {
      if (!patch || typeof patch !== 'object' || Array.isArray(patch)) {
        throw new TypeError('setRigidConfig expects an object');
      }
      const merge = (base, next) => {
        for (const [key, value] of Object.entries(next)) {
          if (value && typeof value === 'object' && !Array.isArray(value) && base[key] && typeof base[key] === 'object') {
            merge(base[key], value);
          } else {
            base[key] = value;
          }
        }
        return base;
      };
      merge(this.rigidBody.c, patch);
      this.rigidBody._layout = null;
      return this.rigidBody.c;
    }

    getRigidConfig() { return this.rigidBody.c; }

    addProp(options = {}) { return this.rigidBody.addProp(options); }

    applyImpulse(impulse, worldPoint = null) {
      if (!impulse || ![impulse.x, impulse.y, impulse.z].every(Number.isFinite)) return false;
      const point = worldPoint && [worldPoint.x, worldPoint.y, worldPoint.z].every(Number.isFinite)
        ? { x: worldPoint.x, y: worldPoint.y + this.bodyOffset, z: worldPoint.z }
        : this.rigidBody.chassis.position;
      this.rigidBody.chassis.applyImpulse({ x: impulse.x, y: impulse.y, z: impulse.z }, point);
      return true;
    }

    setVelocity(velocity) {
      if (!velocity || ![velocity.x, velocity.y, velocity.z].every(Number.isFinite)) return false;
      this.rigidBody.state.velocity = { ...velocity };
      this.rigidBody.chassis.linearVelocity = { ...velocity };
      this.state.velocity = { ...velocity };
      return true;
    }

    advance(frameDelta, input) {
      const started = nowMs();
      const normalized = input === undefined ? { ...this.lastInput } : this.setInput(input);
      this._applyLegacyConfig();
      if (this.rigidBody.mesh !== this.surfaceMesh) this.rigidBody.setSurfaceMesh(this.surfaceMesh);
      if (this._colliderRef !== this.colliderMeshes) {
        this._colliderRef = this.colliderMeshes;
        this.rigidBody.setColliderMeshes(this.colliderMeshes);
      }
      const result = this.rigidBody.advance(frameDelta, {
        throttle: normalized.throttle,
        brake: normalized.brake,
        steer: normalized.steer,
        handbrake: normalized.handbrake,
        pitch: normalized.pitch,
        roll: normalized.roll,
        yaw: normalized.yaw
      });
      this._syncRigidToLegacy();
      this.lastStepMs = nowMs() - started;
      this.totalStepMs = (this.totalStepMs || 0) + this.lastStepMs;
      return result;
    }

    interpolatedState() {
      const p = this.rigidBody.interpolatedState();
      return {
        ...p,
        position: { x: p.position.x, y: p.position.y - this.bodyOffset, z: p.position.z },
        gripState: this.state.gripState,
        collisionState: this.state.collisionState,
        checkpointId: this.state.checkpointId,
        tick: p.tick
      };
    }

    getTelemetry() {
      const base = super.getTelemetry();
      const t = this.rigidBody.telemetry || {};
      return {
        ...base,
        rpm: t.rpm || 0,
        gear: t.gear || 1,
        absActive: !!t.absActive,
        tcsActive: !!t.tcsActive,
        escActive: !!t.escActive,
        wheelSlip: (t.forces || []).map(f => f.slipRatio),
        tireLoads: [...(t.normalLoads || [])],
        tireUtilization: (t.forces || []).map(f => f.utilization || 0),
        suspension: t.suspension,
        drivetrain: t.drivetrain,
        loadTransfer: t.loadTransfer,
        chassisContacts: t.chassisContacts || 0,
        downforce: t.downforce || base.downforce || 0,
        wheelContacts: t.contacts || 0,
        drivenWheels: Array.isArray(t.drivenWheels) ? [...t.drivenWheels] : [],
        driveLayout: t.driveLayout || null,
        differential: t.differential || null,
        antiRoll: t.antiRoll,
        rollCenter: t.rollCenter,
        wheelGeometry: t.wheelGeometry,
        bodyTilt: t.bodyTilt
      };
    }

    getWheelState() {
      return {
        omega: [...this.rigidBody.state.wheelOmega],
        contacts: this.rigidBody.lastContacts.map(c => c.index),
        steer: (this.lastInput.steer || 0) * this.config.maxSteerAngle,
        slip: [...this.rigidBody.state.wheelSpin]
      };
    }

    getSnapshot() {
      const base = typeof super.getSnapshot === 'function'
        ? super.getSnapshot()
        : { state: this.getState(), telemetry: this.getTelemetry(), config: this.config, weather: this.weather, input: this.lastInput };
      return {
        ...base,
        version: UNIFIED_VERSION,
        solver: this.solver,
        state: this.getState(),
        telemetry: this.getTelemetry(),
        rigid: {
          solver: this.rigidBody.constructor.name,
          state: JSON.parse(JSON.stringify(this.rigidBody.state)),
          telemetry: JSON.parse(JSON.stringify(this.rigidBody.telemetry || {})),
          world: { ...this.rigidBody.world.stats }
        }
      };
    }

    debugSnapshot() {
      const base = super.debugSnapshot();
      return {
        ...base,
        solver: this.solver,
        rpm: this.rigidBody.engineRpm,
        gear: this.rigidBody.gear,
        wheelOmega: [...this.rigidBody.state.wheelOmega],
        contacts: this.rigidBody.world.stats.contacts,
        substeps: this.rigidBody.world.stepCount
      };
    }

    reset(position = { x: 0, y: 0, z: 0 }, yaw = 0, up = V.make(0, 1, 0)) {
      super.reset(position, yaw, up);
      if (this.rigidBody) {
        this.rigidBody.reset({
          x: position.x,
          y: position.y + this.bodyOffset,
          z: position.z
        });
        this.rigidBody.state.orientation = { ...this.state.orientation };
        this.rigidBody.previous.orientation = { ...this.state.orientation };
        this.rigidBody.chassis.orientation = { ...this.state.orientation };
        this.rigidBody._layout = null;
        this._syncRigidToLegacy();
      }
      return this;
    }
  }

  // Keep the legacy prototype path present for hosts that opt into the old
  // collision fallback (and for API introspection/documentation).
  UnifiedPlayCarPhysicsEngine.prototype._applyLegacyCollisions = function () {
    if (this.collisionPolicy === 'legacy') {
      LegacyPlayCarPhysicsEngine.prototype._applyStaticCollisions.call(this);
    }
  };

  LegacyAPI.PlayCarPhysicsEngine = UnifiedPlayCarPhysicsEngine;
  LegacyAPI.UnifiedPlayCarPhysicsEngine = UnifiedPlayCarPhysicsEngine;
  LegacyAPI.solverVersion = UNIFIED_VERSION;
  LegacyAPI.version = UNIFIED_VERSION;

  root.PlayCarPhysicsEngine = LegacyAPI;
  if (typeof module !== 'undefined' && module.exports) module.exports = LegacyAPI;
})(typeof globalThis !== 'undefined' ? globalThis : this);
