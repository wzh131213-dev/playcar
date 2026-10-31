/* PlayCar rigid-body vehicle physics 3.0.0
 * Fixed-step 120 Hz vehicle dynamics on top of the PlayCar rigid-body core.
 * SI-like units: metres, seconds, kilograms, newtons, radians.
 *
 * Layers
 *  1. Rigid-body chassis (box) integrated and collision-solved by rigidbody-core.
 *  2. Four independent ray-cast suspension units with progressive springs,
 *     asymmetric bump/rebound damping, bump stops and anti-roll bars.
 *  3. Combined-slip tire model (normalised-slip similarity method, Pacejka
 *     Magic Formula shape) with relaxation length, load sensitivity, camber
 *     thrust, rolling resistance and per-surface friction.
 *  4. Engine torque curve, gearbox with shift time and engine braking,
 *     open / limited-slip / locked differentials and driven-wheel layouts.
 *  5. ABS, TCS and optional ESC driver aids, aerodynamic drag/downforce.
 *
 * The module is dependency-free except for ./rigidbody-core.js. It exports a
 * browser global (PlayCarRigidBody) and a CommonJS namespace.
 */
(function (root) {
  'use strict';

  const V = {
    add: (a, b) => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z }),
    sub: (a, b) => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z }),
    mul: (a, s) => ({ x: a.x * s, y: a.y * s, z: a.z * s }),
    dot: (a, b) => a.x * b.x + a.y * b.y + a.z * b.z,
    cross: (a, b) => ({ x: a.y * b.z - a.z * b.y, y: a.z * b.x - a.x * b.z, z: a.x * b.y - a.y * b.x }),
    len: a => Math.hypot(a.x, a.y, a.z),
    len2: a => a.x * a.x + a.y * a.y + a.z * a.z,
    norm: a => {
      const n = Math.hypot(a.x, a.y, a.z) || 1;
      return { x: a.x / n, y: a.y / n, z: a.z / n };
    },
    clamp: (x, a, b) => Math.max(a, Math.min(b, x)),
    zero: () => ({ x: 0, y: 0, z: 0 })
  };

  const Q = {
    id: () => ({ x: 0, y: 0, z: 0, w: 1 }),
    mul: (a, b) => ({
      x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
      w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    }),
    norm: q => {
      const n = Math.hypot(q.x, q.y, q.z, q.w) || 1;
      return { x: q.x / n, y: q.y / n, z: q.z / n, w: q.w / n };
    },
    rotate: (q, v) => {
      const u = { x: q.x, y: q.y, z: q.z };
      const t = V.mul(V.cross(u, v), 2);
      return V.add(v, V.add(V.mul(t, q.w), V.cross(u, t)));
    },
    rotateInverse: (q, v) => Q.rotate({ x: -q.x, y: -q.y, z: -q.z, w: q.w }, v),
    axis: (axis, angle) => {
      const n = V.norm(axis), s = Math.sin(angle / 2);
      return { x: n.x * s, y: n.y * s, z: n.z * s, w: Math.cos(angle / 2) };
    },
    slerp: (a, b, t) => {
      let d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
      if (d < 0) { b = { x: -b.x, y: -b.y, z: -b.z, w: -b.w }; d = -d; }
      if (d > 0.9995) {
        return Q.norm({ x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t, z: a.z + (b.z - a.z) * t, w: a.w + (b.w - a.w) * t });
      }
      const th = Math.acos(V.clamp(d, -1, 1)), s = Math.sin(th);
      const u = Math.sin((1 - t) * th) / s, v = Math.sin(t * th) / s;
      return Q.norm({ x: a.x * u + b.x * v, y: a.y * u + b.y * v, z: a.z * u + b.z * v, w: a.w * u + b.w * v });
    }
  };

  /* ------------------------------------------------------------------ */
  /* Defaults                                                           */
  /* ------------------------------------------------------------------ */

  const DEFAULTS = {
    fixedDt: 1 / 120,
    maxSubsteps: 24,
    mass: 1250,
    gravity: 9.81,
    inertia: { x: 520, y: 1450, z: 1650 },
    com: { x: 0, y: 0.52, z: 0 },
    wheelbase: 2.65,
    track: 1.56,
    wheelRadius: 0.32,
    wheelMass: 18,
    suspension: {
      rest: 0.32, min: 0.08, max: 0.48,
      stiffness: 32000, damping: 4200,
      dampingBump: 4200, dampingRebound: 5600,
      progressive: 0.35, bump: 90000, bumpStopRange: 0.12
    },
    suspensionGeometry: {
      front: {
        camberStatic: -0.5, camberGain: -22, toeStatic: 0, toeGain: 0.35,
        rollCenterHeight: 0.12, rollCenterTravelGain: 0.10, rollCenterLateralGain: 0.08,
        rollCamberGain: 0.28, casterDeg: 5.5, kingpinInclinationDeg: 8.5,
        scrubRadius: 0.035, mechanicalTrail: 0.045,
        hardpoints: {
          enabled: true,
          upperInner: { x: 0.28, y: 0.82 }, upperOuter: { x: 0.68, y: 0.72 },
          lowerInner: { x: 0.30, y: 0.34 }, lowerOuter: { x: 0.76, y: 0.30 },
          wheelCenterX: 0.88
        }
      },
      rear: {
        camberStatic: -1.0, camberGain: -15, toeStatic: 0.08, toeGain: -0.22,
        rollCenterHeight: 0.17, rollCenterTravelGain: 0.07, rollCenterLateralGain: 0.06,
        rollCamberGain: 0.18, casterDeg: 0, kingpinInclinationDeg: 0,
        scrubRadius: 0, mechanicalTrail: 0,
        hardpoints: {
          enabled: true,
          upperInner: { x: 0.30, y: 0.80 }, upperOuter: { x: 0.69, y: 0.70 },
          lowerInner: { x: 0.32, y: 0.33 }, lowerOuter: { x: 0.77, y: 0.29 },
          wheelCenterX: 0.89
        }
      }
    },
    tire: {
      mu: 1.08,
      corneringStiffness: 78000,
      longitudinalStiffness: 95000,
      relaxation: 0.04,
      relaxationAngle: 0.08,
      relaxationLength: 0.35,
      loadSensitivity: 0.12,
      referenceLoad: 3000,
      aligningTrail: 0.08,
      camberThrustGain: 0.08,
      camberStiffness: 0.55,
      rollingResistance: 0.012,
      pacejka: { enabled: false, B: 0, C: 1.3, E: 0.97, peak: 1 },
      frictionEllipse: { longitudinal: 1, lateral: 1 },
      combined: { enabled: true, C: 1.35, E: 0.92, peak: 1.0 }
    },
    engine: {
      peakTorque: 280, idleRpm: 900, redline: 6500,
      finalDrive: 3.9, gears: [3.2, 2.1, 1.5, 1.18, 0.94, 0.78],
      efficiency: 0.88,
      engineInertia: 0.22,
      shiftTime: 0.16,
      engineBraking: 46,
      revLimiter: 6650,
      torqueCurve: null
    },
    antiRoll: { front: 8500, rear: 6500 },
    body: { rollStiffness: 9000, rollDamping: 2200, pitchStiffness: 14000, pitchDamping: 2800 },
    differential: { type: 'open', limitedSlip: 2.2, preload: 0, ramp: 0 },
    aids: { abs: true, tcs: true, esc: false, absSlip: 0.18, tcsSlip: 0.22, escYawThreshold: 0.14, escBrakeTorque: 1600 },
    steering: { maxAngle: 0.48, highSpeedScale: 0.42, speedReference: 55 },
    brakeTorque: 4200,
    handbrakeTorque: 6500,
    dragArea: 0.68,
    airDensity: 1.225,
    downforceArea: 0.0,
    downforceFront: 0.45,
    restitution: 0.08,
    collisionFriction: 0.55,
    bodyHalf: { x: 0.88, y: 0.48, z: 2.05 },
    ccd: true,
    solver: { velocityIterations: 10, positionIterations: 4, slop: 0.005 },
    collisionSurface: true
  };

  const Core = (typeof module !== 'undefined' && module.exports)
    ? require('./rigidbody-core.js')
    : root.PlayCarRigidBodyCore;

  if (!Core || !Core.World) {
    throw new Error('PlayCarRigidBody: rigidbody-core.js must be loaded before rigidbody-vehicle-engine.js');
  }

  /* ------------------------------------------------------------------ */
  /* Utilities                                                          */
  /* ------------------------------------------------------------------ */

  function clone(o) { return JSON.parse(JSON.stringify(o)); }

  function isPlainObject(value) {
    return value !== null && typeof value === 'object' && !Array.isArray(value);
  }

  function mergeConfig(base, patch) {
    if (!isPlainObject(patch)) return patch === undefined ? base : patch;
    const out = isPlainObject(base) ? { ...base } : {};
    for (const key of Object.keys(patch)) {
      if (isPlainObject(patch[key]) && isPlainObject(out[key])) out[key] = mergeConfig(out[key], patch[key]);
      else out[key] = patch[key];
    }
    return out;
  }

  function circleIntersections(a, ra, b, rb) {
    const dx = b.x - a.x, dy = b.y - a.y, d = Math.hypot(dx, dy);
    if (d < 1e-8 || d > ra + rb + 1e-8 || d < Math.abs(ra - rb) - 1e-8) return [];
    const along = (ra * ra - rb * rb + d * d) / (2 * d);
    const height = Math.sqrt(Math.max(0, ra * ra - along * along));
    const x = a.x + along * dx / d, y = a.y + along * dy / d;
    const px = -dy * height / d, py = dx * height / d;
    return [{ x: x + px, y: y + py }, { x: x - px, y: y - py }];
  }

  function lineIntersection(a, b, c, d) {
    const rx = b.x - a.x, ry = b.y - a.y, sx = d.x - c.x, sy = d.y - c.y;
    const den = rx * sy - ry * sx;
    if (Math.abs(den) < 1e-8) return null;
    const t = ((c.x - a.x) * sy - (c.y - a.y) * sx) / den;
    return { x: a.x + t * rx, y: a.y + t * ry };
  }

  function rotateVectorAroundAxis(vector, axis, angle) {
    const a = V.norm(axis), c = Math.cos(angle), sn = Math.sin(angle);
    return V.add(V.add(V.mul(vector, c), V.mul(V.cross(a, vector), sn)), V.mul(a, V.dot(a, vector) * (1 - c)));
  }

  function solveHardpointKinematics(g, compression, fallbackHeight) {
    const hp = g.hardpoints;
    if (!hp || !hp.enabled) return null;
    const ui = hp.upperInner, li = hp.lowerInner, uo = hp.upperOuter, lo = hp.lowerOuter;
    if (![ui, li, uo, lo].every(p => Number.isFinite(p && p.x) && Number.isFinite(p && p.y))) {
      return { valid: false, rollCenterHeight: fallbackHeight, wheelOffsetX: 0, camberDeltaDeg: 0 };
    }
    const distance = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);
    const lowerLength = distance(li, lo), upperLength = distance(ui, uo), uprightLength = distance(uo, lo);
    const solveAt = travel => {
      const y = lo.y + travel;
      const dy = y - li.y;
      const remain = lowerLength * lowerLength - dy * dy;
      if (remain < 0) return null;
      const dx = Math.sqrt(Math.max(0, remain));
      const lower = { x: li.x + (lo.x >= li.x ? dx : -dx), y };
      const candidates = circleIntersections(ui, upperLength, lower, uprightLength);
      if (!candidates.length) return null;
      candidates.sort((a, b) => distance(a, uo) - distance(b, uo));
      return { lower, upper: candidates[0] };
    };
    const current = solveAt(compression), baseline = solveAt(0);
    if (!current || !baseline) {
      return { valid: false, rollCenterHeight: fallbackHeight, wheelOffsetX: 0, camberDeltaDeg: 0, instantCenter: null };
    }
    const angle = p => Math.atan2(p.upper.x - p.lower.x, p.upper.y - p.lower.y);
    const camberDeltaDeg = (angle(current) - angle(baseline)) * 180 / Math.PI;
    const ic = lineIntersection(ui, current.upper, li, current.lower);
    const contactX = Number.isFinite(hp.contactPatchX) ? hp.contactPatchX : (hp.wheelCenterX ?? 0.88);
    let rollCenterHeight = fallbackHeight;
    if (ic && Math.abs(ic.x - contactX) > 1e-5) {
      const candidate = ic.y * (0 - contactX) / (ic.x - contactX);
      if (Number.isFinite(candidate)) rollCenterHeight = V.clamp(candidate, -2, 2);
    }
    return {
      valid: true, rollCenterHeight, wheelOffsetX: current.lower.x - lo.x,
      camberDeltaDeg, instantCenter: ic, upperOuter: current.upper, lowerOuter: current.lower
    };
  }

  /* ------------------------------------------------------------------ */
  /* Tire models                                                        */
  /* ------------------------------------------------------------------ */

  // Backwards-compatible solver used by existing integrations and tests.
  function calculateTireForces(FxRaw, slipAngle, normalLoad, frictionCoefficient, tire = {}) {
    const cap = Math.max(0, frictionCoefficient * normalLoad);
    const pacejka = tire.pacejka || {};
    let FyRaw;
    if (pacejka.enabled) {
      const C = Math.max(0.1, Number(pacejka.C) || 1.3);
      const E = Number.isFinite(Number(pacejka.E)) ? Number(pacejka.E) : 0.97;
      const peak = Math.max(0, Number(pacejka.peak ?? 1));
      const D = cap * peak;
      const B = Number(pacejka.B) > 0
        ? Number(pacejka.B)
        : Math.max(0.001, (Number(tire.corneringStiffness) || 78000) / (C * Math.max(D, 1)));
      const x = B * slipAngle;
      FyRaw = -D * Math.sin(C * Math.atan(x - E * (x - Math.atan(x))));
    } else {
      FyRaw = -(Number(tire.corneringStiffness) || 78000) * slipAngle;
    }
    const ellipse = tire.frictionEllipse || {};
    const longitudinal = Math.max(0.01, Number(ellipse.longitudinal) || 1);
    const lateral = Math.max(0.01, Number(ellipse.lateral) || 1);
    const nx = cap * longitudinal, ny = cap * lateral;
    const utilization = cap > 0 ? Math.hypot(FxRaw / nx, FyRaw / ny) : Infinity;
    const scale = utilization > 1 ? 1 / utilization : 1;
    const Fx = cap > 0 ? FxRaw * scale : 0;
    const Fy = cap > 0 ? FyRaw * scale : 0;
    return { Fx, Fy, FyRaw, utilization: cap > 0 ? Math.hypot(Fx / nx, Fy / ny) : 0, scale };
  }

  function magicFormula(x, B, C, D, E) {
    return D * Math.sin(C * Math.atan(B * x - E * (B * x - Math.atan(B * x))));
  }

  /**
   * Combined-slip tire force using the normalised-slip similarity method.
   * Longitudinal and lateral slips are combined into one slip vector before
   * the Magic Formula is evaluated, so the resulting force always lies inside
   * a single friction circle and turn-in / power-oversteer stay coupled.
   */
  function calculateCombinedTireForces(options = {}) {
    const normalLoad = Math.max(0, Number(options.normalLoad) || 0);
    const frictionCoefficient = Math.max(0, Number(options.frictionCoefficient) || 0);
    const cap = normalLoad * frictionCoefficient;
    if (cap <= 1e-9) {
      return { Fx: 0, Fy: 0, FyRaw: 0, util: 0, utilization: 0, scale: 1, combinedSlip: 0 };
    }
    const tire = options.tire || {};
    const combined = tire.combined || {};
    const slipRatio = Number(options.slipRatio) || 0;
    const slipAngle = Number(options.slipAngle) || 0;
    const C = Math.max(0.1, Number(combined.C) || 1.35);
    const E = V.clamp(Number.isFinite(Number(combined.E)) ? Number(combined.E) : 0.92, -1, 1);
    const peak = Math.max(0.05, Number(combined.peak) || 1);
    const D = cap * peak;
    const corneringStiffness = Number(tire.corneringStiffness) || 78000;
    const B = Number(combined.B) > 0 ? Number(combined.B) : Math.max(0.5, corneringStiffness / (C * Math.max(D, 1)));

    // Normalised slips keep the combined force continuous as either slip grows.
    const denom = 1 + Math.abs(slipRatio);
    const sx = slipRatio / denom;
    const sy = Math.tan(slipAngle) / denom;
    const s = Math.hypot(sx, sy);
    const F = s > 1e-9 ? magicFormula(s, B, C, D, E) : 0;
    const ux = s > 1e-9 ? sx / s : 0;
    const uy = s > 1e-9 ? sy / s : 0;

    let Fx = F * ux;
    let Fy = -F * uy;

    // Camber thrust: a cambered tire produces lateral force even at zero slip.
    const camber = Number(options.camber) || 0;
    if (camber && tire.camberStiffness) Fy -= camber * cap * Math.max(0, Number(tire.camberStiffness));

    // Rolling resistance opposes the wheel's rolling direction.
    const rollingResistance = Math.max(0, Number(tire.rollingResistance) || 0);
    if (rollingResistance) {
      const rollingSign = Math.sign(options.wheelSurfaceSpeed ?? options.longitudinalSpeed ?? 0);
      if (rollingSign) Fx -= rollingSign * rollingResistance * normalLoad;
    }

    let magnitude = Math.hypot(Fx, Fy);
    let scale = 1;
    if (magnitude > cap) {
      scale = cap / magnitude;
      Fx *= scale;
      Fy *= scale;
      magnitude = cap;
    }
    return {
      Fx, Fy, FyRaw: -F, utilization: magnitude / cap, util: magnitude / cap, scale,
      combinedSlip: s, directionX: ux, directionY: uy
    };
  }

  /* ------------------------------------------------------------------ */
  /* Vehicle                                                            */
  /* ------------------------------------------------------------------ */

  const MOUNTS = [[-1, -1], [1, -1], [-1, 1], [1, 1]]; // [side, axle]

  class RigidBodyVehicle {
    constructor(options = {}) {
      const config = mergeConfig(clone(DEFAULTS), options.config || {});
      this.c = config;
      // Keep arrays and nested special values exact.
      this.c.inertia = mergeConfig(clone(DEFAULTS.inertia), options.config?.inertia || {});
      this.c.com = mergeConfig(clone(DEFAULTS.com), options.config?.com || {});
      this.c.suspension = mergeConfig(clone(DEFAULTS.suspension), options.config?.suspension || {});
      this.c.suspensionGeometry = {
        front: mergeConfig(clone(DEFAULTS.suspensionGeometry.front), options.config?.suspensionGeometry?.front || {}),
        rear: mergeConfig(clone(DEFAULTS.suspensionGeometry.rear), options.config?.suspensionGeometry?.rear || {})
      };
      this.c.tire = mergeConfig(clone(DEFAULTS.tire), options.config?.tire || {});
      this.c.tire.pacejka = mergeConfig(clone(DEFAULTS.tire.pacejka), options.config?.tire?.pacejka || {});
      this.c.tire.combined = mergeConfig(clone(DEFAULTS.tire.combined), options.config?.tire?.combined || {});
      this.c.tire.frictionEllipse = mergeConfig(clone(DEFAULTS.tire.frictionEllipse), options.config?.tire?.frictionEllipse || {});
      this.c.engine = mergeConfig(clone(DEFAULTS.engine), options.config?.engine || {});
      this.c.antiRoll = mergeConfig(clone(DEFAULTS.antiRoll), options.config?.antiRoll || {});
      this.c.body = mergeConfig(clone(DEFAULTS.body), options.config?.body || {});
      this.c.differential = mergeConfig(clone(DEFAULTS.differential), options.config?.differential || {});
      this.c.aids = mergeConfig(clone(DEFAULTS.aids), options.config?.aids || {});
      this.c.steering = mergeConfig(clone(DEFAULTS.steering), options.config?.steering || {});
      this.c.solver = mergeConfig(clone(DEFAULTS.solver), options.config?.solver || {});
      this.c.bodyHalf = mergeConfig(clone(DEFAULTS.bodyHalf), options.config?.bodyHalf || {});

      this.mesh = options.surfaceMesh || null;
      this.colliders = options.colliderMeshes || [];
      this.groundY = options.groundPlaneY ?? null;

      this.accumulator = 0;
      this.gear = 1;
      this.engineRpm = this.c.engine.idleRpm;
      this.shiftTimer = 0;
      this.tick = 0;

      const start = options.position || { x: 0, y: 0.65, z: 0 };
      this.state = {
        position: { ...start },
        velocity: V.zero(),
        orientation: options.orientation ? { ...options.orientation } : Q.id(),
        angularVelocity: V.zero(),
        wheelOmega: [0, 0, 0, 0],
        wheelSpin: [0, 0, 0, 0],
        grounded: false
      };
      if (options.velocity) this.state.velocity = { ...options.velocity };
      this.previous = clone(this.state);
      this.lastContacts = [];
      this.telemetry = {};
      this._slipState = [0, 0, 0, 0];
      this._slipAngleState = [0, 0, 0, 0];
      this.aidState = { abs: false, tcs: false, esc: false };
      this.lastInput = {};
      this.world = null;
      this.chassis = null;
      this._staticBodies = { surface: null, colliders: [] };
      this._buildWorld();
    }

    /* ---------------- collision world ---------------- */

    _buildWorld() {
      const c = this.c;
      this.world = new Core.World({
        gravity: { x: 0, y: -c.gravity, z: 0 },
        fixedDt: c.fixedDt,
        velocityIterations: c.solver.velocityIterations,
        positionIterations: c.solver.positionIterations,
        slop: c.solver.slop,
        allowSleep: true
      });
      this.chassis = new Core.Body({
        type: 'dynamic',
        mass: c.mass,
        inertia: { ...c.inertia },
        shape: new Core.BoxShape(c.bodyHalf),
        position: this.state.position,
        orientation: this.state.orientation,
        velocity: this.state.velocity,
        angularVelocity: this.state.angularVelocity,
        friction: c.collisionFriction,
        restitution: c.restitution,
        linearDamping: 0,
        angularDamping: 0,
        allowSleep: false,
        ccd: c.ccd
      });
      this.world.addBody(this.chassis);
      if (Number.isFinite(this.groundY)) {
        this.world.addGroundPlane(this.groundY, { friction: 1, restitution: 0.02 });
      }
      this._refreshStaticBodies();
    }

    _toCoreMesh(mesh) {
      if (!mesh) return null;
      if (mesh instanceof Core.MeshShape) return mesh;
      if (Array.isArray(mesh.triangles) && mesh.triangles.length) {
        if (mesh._playcarCoreMesh && mesh._playcarCoreMeshSource === mesh.triangles.length) return mesh._playcarCoreMesh;
        const coreMesh = Core.MeshShape.fromLegacyMesh(mesh);
        mesh._playcarCoreMesh = coreMesh;
        mesh._playcarCoreMeshSource = mesh.triangles.length;
        return coreMesh;
      }
      if (typeof mesh.raycast === 'function' && typeof mesh.queryAABB === 'function') return mesh;
      return null;
    }

    _refreshStaticBodies() {
      if (!this.world) return;
      for (const body of Object.values(this._staticBodies).flat()) {
        if (body) this.world.removeBody(body);
      }
      this._staticBodies = { surface: null, colliders: [] };
      if (this.c.collisionSurface) {
        const mesh = this._toCoreMesh(this.mesh);
        if (mesh) this._staticBodies.surface = this.world.addStaticMesh(mesh, {
          friction: 1, restitution: 0.02
        });
      }
      for (const collider of this.colliders || []) {
        const mesh = this._toCoreMesh(collider);
        if (mesh) {
          this._staticBodies.colliders.push(this.world.addStaticMesh(mesh, {
            friction: this.c.collisionFriction, restitution: this.c.restitution
          }));
        }
      }
    }

    setSurfaceMesh(mesh) {
      this.mesh = mesh;
      this._refreshStaticBodies();
      return this;
    }

    setColliderMeshes(meshes) {
      this.colliders = meshes || [];
      this._refreshStaticBodies();
      return this;
    }

    /** Add a dynamic rigid body (cone, barrel, crate) to the vehicle's world. */
    addProp(options = {}) {
      const shape = options.shape === 'sphere'
        ? new Core.SphereShape(options.radius ?? 0.35)
        : new Core.BoxShape(options.halfExtents || { x: 0.25, y: 0.35, z: 0.25 });
      const body = new Core.Body({
        mass: options.mass ?? 12,
        shape,
        position: options.position || { x: 0, y: 1, z: 0 },
        orientation: options.orientation,
        friction: options.friction ?? 0.7,
        restitution: options.restitution ?? 0.15,
        linearDamping: options.linearDamping ?? 0.08,
        angularDamping: options.angularDamping ?? 0.12
      });
      this.world.addBody(body);
      return body;
    }

    addStaticBox(halfExtents, position, options = {}) {
      const body = new Core.Body({
        type: 'static',
        shape: new Core.BoxShape(halfExtents),
        position,
        friction: options.friction ?? 0.8,
        restitution: options.restitution ?? 0.1
      });
      this.world.addBody(body);
      return body;
    }

    /* ---------------- state helpers ---------------- */

    _syncBodyFromState() {
      const body = this.chassis, s = this.state;
      if (!body) return;
      body.position = { ...s.position };
      body.orientation = { ...s.orientation };
      body.linearVelocity = { ...s.velocity };
      body.angularVelocity = { ...s.angularVelocity };
      body.updateAABB();
    }

    _syncStateFromBody() {
      const body = this.chassis, s = this.state;
      s.position = { ...body.position };
      s.orientation = { ...body.orientation };
      s.velocity = { ...body.linearVelocity };
      s.angularVelocity = { ...body.angularVelocity };
    }

    _invInertiaWorld(v) {
      const q = this.state.orientation;
      const local = Q.rotateInverse(q, v);
      const i = this.c.inertia;
      return Q.rotate(q, { x: local.x / i.x, y: local.y / i.y, z: local.z / i.z });
    }

    _pointVelocity(r) {
      return V.add(this.state.velocity, V.cross(this.state.angularVelocity, r));
    }

    _applyForce(F, r) {
      if (!this.chassis) {
        this.state.velocity = V.add(this.state.velocity, V.mul(F, this.c.fixedDt / this.c.mass));
        this.state.angularVelocity = V.add(this.state.angularVelocity,
          V.mul(this._invInertiaWorld(V.cross(r, F)), this.c.fixedDt));
        return;
      }
      this.chassis.applyForce(F, V.add(this.state.position, r || V.zero()));
    }

    _applyTorque(torque) {
      if (!this.chassis) {
        this.state.angularVelocity = V.add(this.state.angularVelocity,
          V.mul(this._invInertiaWorld(torque), this.c.fixedDt));
        return;
      }
      this.chassis.applyTorque(torque);
    }

    reset(position = { x: 0, y: 0.65, z: 0 }) {
      this.state = {
        position: { ...position },
        velocity: V.zero(),
        orientation: Q.id(),
        angularVelocity: V.zero(),
        wheelOmega: [0, 0, 0, 0],
        wheelSpin: [0, 0, 0, 0],
        grounded: false
      };
      this.previous = clone(this.state);
      this.accumulator = 0;
      this.tick = 0;
      this.gear = 1;
      this.engineRpm = this.c.engine.idleRpm;
      this.shiftTimer = 0;
      this._slipState = [0, 0, 0, 0];
      this._slipAngleState = [0, 0, 0, 0];
      this.aidState = { abs: false, tcs: false, esc: false };
      this.lastContacts = [];
      this.telemetry = {};
      if (this.chassis) {
        this.chassis.position = { ...position };
        this.chassis.orientation = Q.id();
        this.chassis.linearVelocity = V.zero();
        this.chassis.angularVelocity = V.zero();
        this.chassis.sleeping = false;
        this.chassis.sleepTimer = 0;
        this.chassis.updateAABB();
      }
      if (this.world) {
        this.world.time = 0;
        this.world.stepCount = 0;
        this.world.accumulator = 0;
        this.world._warmCache = new Map();
      }
      return this;
    }

    /* ---------------- wheel geometry / contacts ---------------- */

    _wheelKinematics(index, compression, normal, input = {}) {
      const c = this.c, s = this.state, front = index < 2, side = (index === 0 || index === 2) ? -1 : 1;
      const g = front ? c.suspensionGeometry.front : c.suspensionGeometry.rear;
      const tilt = this._bodyTilt();
      const travel = V.clamp(compression / Math.max(0.001, c.suspension.max - c.suspension.min), -1, 1);
      const fallbackRollCenter = Math.max(0, g.rollCenterHeight + g.rollCenterTravelGain * compression + g.rollCenterLateralGain * side * compression);
      const hardpoint = solveHardpointKinematics(g, compression, fallbackRollCenter);
      const camberDeg = g.camberStatic + g.camberGain * compression +
        (hardpoint && hardpoint.valid ? hardpoint.camberDeltaDeg : 0) + g.rollCamberGain * tilt.roll * 180 / Math.PI;
      const toeDeg = g.toeStatic + g.toeGain * compression;
      const speed = V.len(s.velocity);
      const steer = front
        ? V.clamp(input.steer || 0, -1, 1) * c.steering.maxAngle *
          (1 - (1 - c.steering.highSpeedScale) * V.clamp(speed / (c.steering.speedReference || 55), 0, 1))
        : 0;
      const n = V.norm(normal);
      const bodyForward = Q.rotate(s.orientation, { x: 0, y: 0, z: -1 });
      let forward = V.norm(V.sub(bodyForward, V.mul(n, V.dot(bodyForward, n))));
      if (V.len(forward) < 1e-8) forward = V.norm(V.cross({ x: 1, y: 0, z: 0 }, n));
      const right0 = V.norm(V.cross(forward, n));
      const angle = steer + toeDeg * Math.PI / 180;
      const caster = V.clamp(Number(g.casterDeg) || 0, -25, 25) * Math.PI / 180;
      const kpi = V.clamp(Number(g.kingpinInclinationDeg) || 0, 0, 30) * Math.PI / 180;
      const steeringAxis = V.mul(V.norm(V.add(
        V.mul(n, Math.cos(kpi) * Math.cos(caster)),
        V.add(V.mul(right0, -side * Math.sin(kpi) * Math.cos(caster)), V.mul(forward, -Math.sin(caster)))
      )), -1);
      const steered = rotateVectorAroundAxis(forward, steeringAxis, angle);
      forward = V.norm(V.sub(steered, V.mul(n, V.dot(steered, n))));
      const right = V.norm(V.cross(forward, n));
      const steeringNormal = rotateVectorAroundAxis(n, steeringAxis, angle);
      const steeringCamberDeg = Math.atan2(V.dot(steeringNormal, right0), Math.max(1e-8, V.dot(steeringNormal, n))) * 180 / Math.PI;
      const effectiveCamberDeg = camberDeg + steeringCamberDeg;
      const scrubRadius = Math.max(0, Number(g.scrubRadius) || 0);
      const mechanicalTrail = Math.max(0, Number(g.mechanicalTrail) || 0);
      const jackingHeight = scrubRadius * Math.sin(angle) * Math.sin(kpi);
      const camberedNormal = Q.rotate(Q.axis(forward, effectiveCamberDeg * Math.PI / 180), n);
      const camberedRight = V.norm(V.cross(forward, camberedNormal));
      const nominalRollCenter = front ? DEFAULTS.suspensionGeometry.front.rollCenterHeight : DEFAULTS.suspensionGeometry.rear.rollCenterHeight;
      const rollCenterHeight = hardpoint && hardpoint.valid
        ? hardpoint.rollCenterHeight + (fallbackRollCenter - nominalRollCenter)
        : fallbackRollCenter;
      return {
        camber: effectiveCamberDeg * Math.PI / 180, camberDeg: effectiveCamberDeg, steeringCamberDeg,
        toe: toeDeg * Math.PI / 180, toeDeg, steer, forward, right, steeringAxis,
        casterDeg: caster * 180 / Math.PI, kingpinInclinationDeg: kpi * 180 / Math.PI,
        scrubRadius, mechanicalTrail, jackingHeight, camberedNormal, camberedRight,
        rollCenterHeight, travel, side, axle: front ? 'front' : 'rear',
        hardpointValid: !!(hardpoint && hardpoint.valid),
        instantCenter: (hardpoint && hardpoint.instantCenter) || null,
        upperOuter: (hardpoint && hardpoint.upperOuter) || null,
        lowerOuter: (hardpoint && hardpoint.lowerOuter) || null,
        wheelOffsetX: (hardpoint && hardpoint.wheelOffsetX) || 0
      };
    }

    _contacts(input = {}) {
      const c = this.c, s = this.state, out = [], q = s.orientation;
      const halfWheelbase = c.wheelbase / 2, halfTrack = c.track / 2;
      for (let i = 0; i < 4; i++) {
        const mount = MOUNTS[i];
        const local = { x: mount[0] * halfTrack, y: 0, z: mount[1] * halfWheelbase };
        const r = Q.rotate(q, local);
        const origin = V.add(s.position, r);
        const reach = c.suspension.max + c.wheelRadius;
        let hit = null;
        if (this.mesh && typeof this.mesh.closestPoint === 'function') {
          hit = this.mesh.closestPoint(origin, reach + c.wheelRadius, { x: 0, y: 1, z: 0 });
        }
        if (!hit && Number.isFinite(this.groundY)) {
          const dist = origin.y - this.groundY;
          if (dist < reach + c.wheelRadius) {
            hit = {
              point: { x: origin.x, y: this.groundY, z: origin.z },
              normal: { x: 0, y: 1, z: 0 },
              distance: Math.max(0, dist),
              signedDistance: dist - c.wheelRadius
            };
          }
        }
        if (!hit) continue;
        let n = V.norm(hit.normal || { x: 0, y: 1, z: 0 });
        if (V.dot(n, { x: 0, y: 1, z: 0 }) < 0) n = V.mul(n, -1);
        const rawLength = hit.distance - c.wheelRadius;
        const length = V.clamp(rawLength, c.suspension.min, c.suspension.max);
        const compression = c.suspension.rest - length;
        const geometry = this._wheelKinematics(i, compression, n, input);
        const rawPoint = hit.point || V.sub(origin, V.mul(n, hit.distance));
        const bodyRight = Q.rotate(q, { x: 1, y: 0, z: 0 });
        const tangentRight = V.norm(V.sub(bodyRight, V.mul(n, V.dot(bodyRight, n))));
        const point = V.add(rawPoint, V.mul(tangentRight, geometry.wheelOffsetX * geometry.side));
        const arm = V.sub(point, s.position);
        const pv = this._pointVelocity(arm);
        const vn = V.dot(pv, n);
        const wheelCenter = V.add(point, V.mul(n, c.wheelRadius + geometry.jackingHeight));
        out.push({
          index: i, point, normal: n, arm, compression, length, rawLength,
          normalVelocity: vn, velocity: pv, local, wheelCenter, geometry,
          tireForward: geometry.forward, tireRight: geometry.right,
          friction: Number.isFinite(hit.friction) ? Math.max(0, hit.friction) : 1,
          restitution: Number.isFinite(hit.restitution) ? hit.restitution : 0.05
        });
      }
      return out;
    }

    _bodyTilt() {
      const q = this.state.orientation;
      const up = Q.rotate(q, { x: 0, y: 1, z: 0 });
      const localAngularVelocity = Q.rotateInverse(q, this.state.angularVelocity);
      return {
        roll: Math.atan2(-up.x, Math.max(1e-9, up.y)),
        pitch: Math.atan2(up.z, Math.max(1e-9, up.y)),
        rollRate: localAngularVelocity.z,
        pitchRate: localAngularVelocity.x
      };
    }

    _applyBodyStabilization() {
      const b = this.c.body || {}, tilt = this._bodyTilt();
      const localTorque = {
        x: -Math.max(0, b.pitchStiffness || 0) * tilt.pitch - Math.max(0, b.pitchDamping || 0) * tilt.pitchRate,
        y: 0,
        z: -Math.max(0, b.rollStiffness || 0) * tilt.roll - Math.max(0, b.rollDamping || 0) * tilt.rollRate
      };
      if (V.len(localTorque) > 0) this._applyTorque(Q.rotate(this.state.orientation, localTorque));
    }

    /* ---------------- drivetrain ---------------- */

    _drivetrainLayout() {
      const c = this.c;
      const rawType = c.differential.type || 'open';
      const driveType = c.differential.layout || (['fwd', 'rwd', 'awd'].includes(rawType) ? rawType : 'rwd');
      const diffType = ['lsd', 'limited-slip', 'locked'].includes(rawType)
        ? (rawType === 'locked' ? 'locked' : 'lsd')
        : 'open';
      const driven = [0, 1, 2, 3].filter(i => driveType === 'awd' ||
        (driveType === 'fwd' && i < 2) ||
        (driveType !== 'fwd' && driveType !== 'awd' && i >= 2));
      return { driveType, diffType, driven };
    }

    _engineTorqueAtRpm(rpm) {
      const e = this.c.engine;
      if (Array.isArray(e.torqueCurve) && e.torqueCurve.length) {
        const curve = e.torqueCurve;
        if (rpm <= curve[0][0]) return e.peakTorque * curve[0][1];
        for (let i = 1; i < curve.length; i++) {
          if (rpm <= curve[i][0]) {
            const [r0, t0] = curve[i - 1], [r1, t1] = curve[i];
            const t = (rpm - r0) / Math.max(1, r1 - r0);
            return e.peakTorque * (t0 + (t1 - t0) * t);
          }
        }
        return e.peakTorque * curve[curve.length - 1][1];
      }
      return e.peakTorque * V.clamp(1 - Math.abs(rpm - 3800) / 6500, 0.25, 1);
    }

    _updateDrivetrain(dt, input, wheelSpeeds) {
      const c = this.c, e = c.engine, s = this.state;
      const drivenWheels = this._layout.driven;
      const drivenSpeed = drivenWheels.length
        ? drivenWheels.reduce((sum, i) => sum + Math.abs(s.wheelOmega[i]), 0) / drivenWheels.length
        : s.wheelOmega.reduce((sum, w) => sum + Math.abs(w), 0) / 4;
      const gearRatio = e.gears[this.gear - 1] || 1;
      const wheelRpm = drivenSpeed * c.wheelRadius * e.finalDrive * gearRatio * 60 / (2 * Math.PI);
      this.engineRpm = V.clamp(wheelRpm, e.idleRpm, e.redline);

      if (this.shiftTimer > 0) {
        this.shiftTimer = Math.max(0, this.shiftTimer - dt);
      } else {
        if (this.engineRpm > e.redline * 0.96 && this.gear < e.gears.length) {
          this.gear++;
          this.shiftTimer = Math.max(0, Number(e.shiftTime) || 0);
        } else if (this.engineRpm < e.redline * 0.55 && this.gear > 1) {
          this.gear--;
          this.shiftTimer = Math.max(0, Number(e.shiftTime) || 0);
        }
      }

      let throttle = V.clamp(input.throttle || 0, 0, 1);
      if (this.engineRpm > (e.revLimiter || e.redline)) throttle = 0;
      const shifting = this.shiftTimer > 0;
      let torque = this._engineTorqueAtRpm(this.engineRpm) * throttle;
      if (shifting) torque = 0;

      // Engine braking when the driver is off the throttle.
      const longitudinalSpeed = V.dot(s.velocity, Q.rotate(s.orientation, { x: 0, y: 0, z: -1 }));
      if (throttle < 0.02 && !shifting && Math.abs(longitudinalSpeed) > 0.5) {
        const brakeTorque = -(Number(e.engineBraking) || 0) * (this.engineRpm / Math.max(1, e.redline)) *
          Math.sign(longitudinalSpeed);
        torque += brakeTorque;
      }

      const totalDriveTorque = torque * gearRatio * e.finalDrive * e.efficiency;
      return { totalDriveTorque, throttle, drivenWheels, gearRatio, shifting, longitudinalSpeed };
    }

    _distributeDriveTorque(totalDriveTorque) {
      const c = this.c, s = this.state;
      const wheelDriveTorque = [0, 0, 0, 0];
      const driven = this._layout.driven;
      const diffType = this._layout.diffType;
      const limitedSlip = Math.max(1, Number(c.differential.limitedSlip) || 1);
      const preload = Math.max(0, Number(c.differential.preload) || 0);
      for (const axle of [[0, 1], [2, 3]]) {
        const pair = axle.filter(i => driven.includes(i));
        if (!pair.length) continue;
        const base = totalDriveTorque / Math.max(1, driven.length);
        if (pair.length === 2 && diffType === 'locked') {
          wheelDriveTorque[pair[0]] = base;
          wheelDriveTorque[pair[1]] = base;
        } else if (pair.length === 2 && diffType === 'lsd') {
          const a = pair[0], b = pair[1];
          const wa = Math.abs(s.wheelOmega[a]), wb = Math.abs(s.wheelOmega[b]);
          if (Math.abs(wa - wb) < 0.25) {
            wheelDriveTorque[a] = base;
            wheelDriveTorque[b] = base;
          } else {
            const slow = wa < wb ? a : b;
            const fast = slow === a ? b : a;
            const bias = Math.max(1, limitedSlip + preload * 0.001);
            wheelDriveTorque[slow] = base * 2 * bias / (1 + bias);
            wheelDriveTorque[fast] = base * 2 / (1 + bias);
          }
        } else {
          for (const i of pair) wheelDriveTorque[i] = base;
        }
      }
      return wheelDriveTorque;
    }

    /* ---------------- fixed step ---------------- */

    // Redistribute support load proportionally within an axle/side. Equal
    // per-wheel deltas can drive an already lightly loaded wheel negative;
    // proportional shares keep the sum exactly conserved and non-negative.
    _distributeLoad(contacts, delta, normalLoad, suspensionTelemetry) {
      if (!contacts.length || delta === 0) return;
      const total = contacts.reduce((sum, h) => sum + normalLoad[h.index], 0);
      if (total <= 1e-9) return;
      for (const h of contacts) {
        const share = normalLoad[h.index] / total;
        normalLoad[h.index] = Math.max(0, normalLoad[h.index] + delta * share);
        const applied = normalLoad[h.index] - suspensionTelemetry[h.index].normalLoad;
        if (applied) this._applyForce(V.mul(h.normal, applied), h.arm);
        suspensionTelemetry[h.index].normalLoad = normalLoad[h.index];
      }
    }

    _fixed(dt, input) {
      const c = this.c, s = this.state;
      this.previous = clone(s);
      this.aidState = { abs: false, tcs: false, esc: false };
      this._syncBodyFromState();

      const contacts = this._contacts(input);
      this.lastContacts = contacts;
      const normalLoad = [0, 0, 0, 0];
      const forces = [];
      const suspensionTelemetry = Array.from({ length: 4 }, (_, wheel) => ({
        wheel, compression: 0, rawLength: null, bumpStopCompression: 0, bumpStopForce: 0,
        springForce: 0, damperForce: 0, normalLoad: 0, velocity: 0
      }));

      /* 1. suspension: progressive spring, asymmetric damper, bump stop. */
      for (const h of contacts) {
        const susp = c.suspension;
        const bumpStopCompression = Math.max(0, susp.min - h.rawLength);
        const bumpStopForce = Math.max(0, susp.bump || 0) * bumpStopCompression *
          (1 + bumpStopCompression / Math.max(0.001, susp.bumpStopRange || 0.12));
        const compression = Math.max(0, h.compression);
        const springForce = susp.stiffness * compression * (1 + (susp.progressive || 0) * compression);
        const dampingRate = h.normalVelocity < 0
          ? (Number.isFinite(susp.dampingBump) ? susp.dampingBump : susp.damping)
          : (Number.isFinite(susp.dampingRebound) ? susp.dampingRebound : susp.damping);
        const damperForce = -dampingRate * h.normalVelocity;
        const load = Math.max(0, springForce + damperForce + bumpStopForce);
        normalLoad[h.index] = load;
        suspensionTelemetry[h.index] = {
          wheel: h.index, compression, rawLength: h.rawLength,
          bumpStopCompression, bumpStopForce, springForce, damperForce,
          normalLoad: load, velocity: h.normalVelocity
        };
        this._applyForce(V.mul(h.normal, load), h.arm);
      }

      /* 2. anti-roll bars. */
      for (const [left, right, rate] of [[0, 1, c.antiRoll.front], [2, 3, c.antiRoll.rear]]) {
        const a = contacts.find(h => h.index === left);
        const b = contacts.find(h => h.index === right);
        if (a && b && rate > 0) {
          const transfer = (a.compression - b.compression) * rate;
          const fa = Math.max(-normalLoad[left], transfer);
          const fb = Math.max(-normalLoad[right], -transfer);
          normalLoad[left] += fa;
          normalLoad[right] += fb;
          this._applyForce(V.mul(a.normal, fa), a.arm);
          this._applyForce(V.mul(b.normal, fb), b.arm);
          suspensionTelemetry[left].normalLoad = normalLoad[left];
          suspensionTelemetry[right].normalLoad = normalLoad[right];
        }
      }

      /* 3. aerodynamics. */
      const forward = Q.rotate(s.orientation, { x: 0, y: 0, z: -1 });
      const right = Q.rotate(s.orientation, { x: 1, y: 0, z: 0 });
      const speed = V.len(s.velocity);
      const drag = 0.5 * c.airDensity * c.dragArea * speed * speed;
      if (speed > 1e-5) this._applyForce(V.mul(s.velocity, -drag / speed), V.zero());
      const downforce = 0.5 * c.airDensity * c.downforceArea * speed * speed;
      if (downforce) this._applyForce(V.mul(Q.rotate(s.orientation, { x: 0, y: -1, z: 0 }), downforce), V.zero());

      /* 4. drivetrain. */
      this._layout = this._layout || this._drivetrainLayout();
      const drivetrain = this._updateDrivetrain(dt, input, s.wheelOmega);
      const throttle = drivetrain.throttle;
      const brake = V.clamp(input.brake || 0, 0, 1);
      const steer = V.clamp(input.steer || 0, -1, 1);
      const handbrake = V.clamp(input.handbrake || 0, 0, 1);
      const longitudinalSpeed = drivetrain.longitudinalSpeed;

      /* 5. longitudinal load transfer estimate. */
      const driveForce = drivetrain.totalDriveTorque / Math.max(0.05, c.wheelRadius);
      const brakeForce = longitudinalSpeed === 0 ? 0 :
        brake * c.brakeTorque * 4 / Math.max(0.05, c.wheelRadius) * Math.sign(longitudinalSpeed);
      const requestedLongitudinalForce = driveForce - brakeForce;
      const requestedAcceleration = V.clamp(requestedLongitudinalForce / c.mass, -c.tire.mu * c.gravity, c.tire.mu * c.gravity);
      const requestedTransfer = c.mass * requestedAcceleration * Math.max(0, c.com.y) / Math.max(0.1, c.wheelbase);
      const frontContacts = contacts.filter(h => h.index < 2);
      const rearContacts = contacts.filter(h => h.index >= 2);
      const frontAvailable = frontContacts.reduce((sum, h) => sum + normalLoad[h.index], 0);
      const rearAvailable = rearContacts.reduce((sum, h) => sum + normalLoad[h.index], 0);
      const longitudinalTransfer = frontContacts.length && rearContacts.length
        ? V.clamp(requestedTransfer, -rearAvailable, frontAvailable) : 0;
      this._distributeLoad(frontContacts, -longitudinalTransfer, normalLoad, suspensionTelemetry);
      this._distributeLoad(rearContacts, longitudinalTransfer, normalLoad, suspensionTelemetry);

      /* 6. lateral load transfer estimate. */
      const steeringForTransfer = steer * c.steering.maxAngle *
        (1 - (1 - c.steering.highSpeedScale) * V.clamp(speed / (c.steering.speedReference || 55), 0, 1));
      const requestedLateralAcceleration = longitudinalSpeed * Math.abs(longitudinalSpeed) *
        Math.tan(steeringForTransfer) / Math.max(0.1, c.wheelbase);
      const axleRollCenter = axle => axle.length
        ? axle.reduce((sum, h) => sum + h.geometry.rollCenterHeight, 0) / axle.length : 0;
      const frontRollCenterHeight = axleRollCenter(frontContacts);
      const rearRollCenterHeight = axleRollCenter(rearContacts);
      const effectiveRollCenterHeight = (frontRollCenterHeight + rearRollCenterHeight) * 0.5;
      const lateralMomentArm = Math.max(0.05, c.com.y - effectiveRollCenterHeight);
      const requestedLateralTransfer = c.mass * requestedLateralAcceleration * lateralMomentArm / Math.max(0.1, c.track);
      const leftContacts = contacts.filter(h => h.index === 0 || h.index === 2);
      const rightContacts = contacts.filter(h => h.index === 1 || h.index === 3);
      const leftAvailable = leftContacts.reduce((sum, h) => sum + normalLoad[h.index], 0);
      const rightAvailable = rightContacts.reduce((sum, h) => sum + normalLoad[h.index], 0);
      const lateralTransfer = leftContacts.length && rightContacts.length
        ? V.clamp(requestedLateralTransfer, -leftAvailable, rightAvailable) : 0;
      this._distributeLoad(leftContacts, lateralTransfer, normalLoad, suspensionTelemetry);
      this._distributeLoad(rightContacts, -lateralTransfer, normalLoad, suspensionTelemetry);
      const loadTransferTelemetry = { longitudinal: -longitudinalTransfer, lateral: lateralTransfer };

      /* 7. drive torque distribution. */
      const wheelDriveTorque = this._distributeDriveTorque(drivetrain.totalDriveTorque);

      /* 8. combined-slip tire forces, ABS / TCS / ESC, wheel spin dynamics. */
      const steerAngle = steer * c.steering.maxAngle *
        (1 - (1 - c.steering.highSpeedScale) * V.clamp(speed / (c.steering.speedReference || 55), 0, 1));
      const targetYawRate = longitudinalSpeed * Math.tan(steerAngle) / Math.max(0.1, c.wheelbase);
      const yawError = s.angularVelocity.y - targetYawRate;
      const escEnabled = !!c.aids.esc && speed > 6;
      if (escEnabled && Math.abs(yawError) > c.aids.escYawThreshold) this.aidState.esc = true;

      for (const h of contacts) {
        const i = h.index;
        const rear = i >= 2;
        const wheelForward = h.tireForward || forward;
        const wheelRight = h.tireRight || right;
        const vLong = V.dot(h.velocity, wheelForward);
        const vLat = V.dot(h.velocity, wheelRight);
        const omega = s.wheelOmega[i];

        const rawSlip = (omega * c.wheelRadius - vLong) / Math.max(2, Math.abs(vLong));
        const relax = V.clamp(dt / Math.max(0.001, c.tire.relaxation), 0, 1);
        this._slipState[i] += (rawSlip - this._slipState[i]) * relax;
        const slipRatio = this._slipState[i];

        const rawSlipAngle = Math.atan2(vLat, Math.max(1, Math.abs(vLong)));
        const relaxationTime = Math.max(0.02, Math.min(
          Number(c.tire.relaxationAngle) || 0.08,
          (Number(c.tire.relaxationLength) || 0.35) / Math.max(3, Math.abs(vLong))
        ));
        const angleRelax = V.clamp(dt / relaxationTime, 0, 1);
        this._slipAngleState[i] += (rawSlipAngle - this._slipAngleState[i]) * angleRelax;
        let slipAngle = this._slipAngleState[i];
        slipAngle += h.geometry.camber * (c.tire.camberThrustGain ?? 0.08);

        let wheelDrive = wheelDriveTorque[i];
        if (c.aids.tcs && wheelDrive > 0 && slipRatio > c.aids.tcsSlip) {
          wheelDrive *= V.clamp(c.aids.tcsSlip / Math.max(0.001, slipRatio), 0.15, 1);
          this.aidState.tcs = true;
        }
        if (this.aidState.esc && wheelDrive > 0) wheelDrive *= 0.65;

        const loadSensitivityFactor = V.clamp(
          Math.pow(Math.max(1, normalLoad[i]) / Math.max(1, c.tire.referenceLoad || 3000), -(c.tire.loadSensitivity || 0)),
          0.55, 1.25
        );
        const frictionCoefficient = c.tire.mu * h.friction * loadSensitivityFactor;
        const driveAxleForce = wheelDrive / Math.max(0.05, c.wheelRadius);
        let tireForce;
        if (c.tire.combined && c.tire.combined.enabled) {
          tireForce = calculateCombinedTireForces({
            slipRatio: slipRatio + driveAxleForce / Math.max(1, c.tire.longitudinalStiffness),
            slipAngle,
            normalLoad: normalLoad[i],
            frictionCoefficient,
            tire: c.tire,
            camber: h.geometry.camber,
            wheelSurfaceSpeed: vLong
          });
        } else {
          const legacy = calculateTireForces(
            c.tire.longitudinalStiffness * slipRatio + wheelDrive / Math.max(0.05, c.wheelRadius),
            slipAngle, normalLoad[i], frictionCoefficient, c.tire
          );
          tireForce = { ...legacy, combinedSlip: Math.abs(slipRatio) + Math.abs(slipAngle), utilization: legacy.utilization };
        }

        let Fx = tireForce.Fx, Fy = tireForce.Fy;
        const cap = Math.max(0, frictionCoefficient * normalLoad[i]);
        let magnitude = Math.hypot(Fx, Fy);
        if (cap > 0 && magnitude > cap) {
          const scale = cap / magnitude;
          Fx *= scale; Fy *= scale; magnitude = cap;
        }

        const aligningMoment = -Fy * (c.tire.aligningTrail || 0) * V.clamp(1 - Math.abs(slipAngle) / 0.6, 0, 1);
        this._applyForce(V.add(V.mul(wheelForward, Fx), V.mul(wheelRight, Fy)), h.arm);
        if (aligningMoment) this._applyTorque(V.mul(h.normal, aligningMoment));

        let brakeTorque = brake * c.brakeTorque + (handbrake > 0 && rear ? handbrake * c.handbrakeTorque : 0);
        if (c.aids.abs && brakeTorque > 0 && Math.abs(slipRatio) > c.aids.absSlip) {
          brakeTorque *= 0.35;
          this.aidState.abs = true;
        }
        if (this.aidState.esc && h.index < 2 && Math.sign(yawError) !== 0) {
          // Brake the front wheel on the outside of the yaw error.
          const outside = yawError > 0 ? 0 : 1;
          if (h.index === outside) brakeTorque += c.aids.escBrakeTorque;
        }

        const wheelAccel = (wheelDrive - Fx * c.wheelRadius - Math.sign(omega || vLong) * brakeTorque) /
          (c.wheelMass * c.wheelRadius * c.wheelRadius);
        s.wheelOmega[i] += wheelAccel * dt;
        if (Math.abs(s.wheelOmega[i]) < 0.5 && brake > 0.8) s.wheelOmega[i] = 0;
        s.wheelSpin[i] = slipRatio;

        forces.push({
          wheel: i, Fx, Fy, FyRaw: tireForce.FyRaw, normalLoad: normalLoad[i],
          slipRatio, slipAngle, rawSlipAngle, aligningMoment, loadSensitivityFactor,
          frictionCoefficient, driveTorque: wheelDrive, brakeTorque,
          utilization: cap > 0 ? magnitude / cap : 0,
          combinedSlip: tireForce.combinedSlip, wheelOmega: s.wheelOmega[i]
        });
      }

      /* 9. chassis attitude stabilization and rigid-body integration. */
      this._applyBodyStabilization();
      this.world.fixedStep(dt);
      this._syncStateFromBody();

      /* 10. post-step support state + telemetry. */
      const after = this._contacts(input);
      s.grounded = after.length >= 2;
      this.tick++;
      const chassisContacts = this.world.manifolds.reduce((n, m) =>
        n + (m.bodyA === this.chassis || m.bodyB === this.chassis ? m.manifold.points.length : 0), 0);
      this.telemetry = {
        tick: this.tick,
        contacts: after.length,
        normalLoads: normalLoad,
        forces,
        suspension: suspensionTelemetry,
        wheelGeometry: after.map(h => ({
          wheel: h.index, axle: h.geometry.axle, camber: h.geometry.camber,
          camberDeg: h.geometry.camberDeg, toe: h.geometry.toe, toeDeg: h.geometry.toeDeg,
          rollCenterHeight: h.geometry.rollCenterHeight, travel: h.geometry.travel,
          hardpointValid: h.geometry.hardpointValid,
          instantCenter: h.geometry.instantCenter ? { ...h.geometry.instantCenter } : null,
          upperOuter: h.geometry.upperOuter ? { ...h.geometry.upperOuter } : null,
          lowerOuter: h.geometry.lowerOuter ? { ...h.geometry.lowerOuter } : null,
          wheelOffsetX: h.geometry.wheelOffsetX, casterDeg: h.geometry.casterDeg,
          kingpinInclinationDeg: h.geometry.kingpinInclinationDeg,
          steeringCamberDeg: h.geometry.steeringCamberDeg, scrubRadius: h.geometry.scrubRadius,
          mechanicalTrail: h.geometry.mechanicalTrail, jackingHeight: h.geometry.jackingHeight,
          steeringAxis: { ...h.geometry.steeringAxis }, wheelCenter: { ...h.wheelCenter },
          contactNormal: { ...h.normal }, tireForward: { ...h.tireForward }, tireRight: { ...h.tireRight }
        })),
        rollCenter: {
          front: frontRollCenterHeight, rear: rearRollCenterHeight,
          average: effectiveRollCenterHeight, lateralMomentArm
        },
        bodyTilt: this._bodyTilt(),
        bodyStabilization: { ...c.body },
        gear: this.gear, rpm: this.engineRpm,
        speed, downforce,
        absActive: this.aidState.abs, tcsActive: this.aidState.tcs, escActive: this.aidState.esc,
        antiRoll: { front: c.antiRoll.front, rear: c.antiRoll.rear },
        loadTransfer: loadTransferTelemetry,
        differential: this._layout.diffType, driveLayout: this._layout.driveType,
        drivenWheels: [...this._layout.driven],
        drivetrain: {
          gear: this.gear, rpm: this.engineRpm, shifting: drivetrain.shifting,
          shiftTimer: this.shiftTimer, gearRatio: drivetrain.gearRatio,
          wheelTorque: drivetrain.totalDriveTorque, throttle,
          engineBraking: throttle < 0.02 && Math.abs(longitudinalSpeed) > 0.5
        },
        aids: { abs: this.aidState.abs, tcs: this.aidState.tcs, esc: this.aidState.esc },
        chassisContacts,
        collisionImpacts: this.world.stats.contacts
      };
    }

    advance(frameDelta, input = {}) {
      this.lastInput = input || {};
      const dt = Math.max(0, Math.min(0.25, Number(frameDelta) || 0));
      this.accumulator += dt;
      let steps = 0;
      while (this.accumulator + 1e-10 >= this.c.fixedDt && steps < this.c.maxSubsteps) {
        this._fixed(this.c.fixedDt, this.lastInput);
        this.accumulator -= this.c.fixedDt;
        steps++;
      }
      if (this.accumulator >= this.c.fixedDt) this.accumulator %= this.c.fixedDt;
      return { steps, alpha: V.clamp(this.accumulator / this.c.fixedDt, 0, 1) };
    }

    interpolatedState() {
      const a = this.previous, b = this.state;
      const t = V.clamp(this.accumulator / this.c.fixedDt, 0, 1);
      return {
        position: V.add(a.position, V.mul(V.sub(b.position, a.position), t)),
        velocity: V.add(a.velocity, V.mul(V.sub(b.velocity, a.velocity), t)),
        orientation: Q.slerp(a.orientation, b.orientation, t),
        grounded: b.grounded,
        wheelOmega: [...b.wheelOmega]
      };
    }

    getWheelState() {
      return {
        omega: [...this.state.wheelOmega],
        contacts: this.lastContacts.map(c => c.index),
        steer: (this.lastInput.steer || 0) * this.c.steering.maxAngle
      };
    }

    getTelemetry() { return this.telemetry; }

    getSnapshot() {
      return {
        state: clone(this.state),
        telemetry: clone(this.telemetry),
        config: clone(this.c),
        stats: { ...this.world.stats, time: this.world.time }
      };
    }
  }

  const api = {
    RigidBodyVehicle,
    Vector3: V,
    Quaternion: Q,
    calculateTireForces,
    calculateCombinedTireForces,
    magicFormula,
    solveHardpointKinematics,
    defaults: clone(DEFAULTS),
    core: Core,
    version: '3.0.0'
  };

  root.PlayCarRigidBody = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(typeof globalThis !== 'undefined' ? globalThis : this);
