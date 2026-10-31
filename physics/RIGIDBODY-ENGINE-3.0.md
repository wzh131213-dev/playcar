# PlayCar Rigid-Body Engine 3.0

Design, API and validation notes for the 3.0 physics stack.

- `physics/rigidbody-core.js` — general 3D rigid-body core.
- `physics/rigidbody-vehicle-engine.js` — vehicle dynamics layer.
- `physics/playcar-unified.js` — 3.0 runtime with the 1.8-compatible facade.
- `physics/playcar-physics-all-in-one.js` — generated single-file build.

Everything is plain JavaScript with no WASM and no runtime dependency. Three.js is
used only by the browser lab for rendering.

## 1. Conventions and units

| Item | Value |
| --- | --- |
| Units | metres, kilograms, seconds, radians |
| Up axis | +Y |
| Vehicle forward | local -Z (`Quaternion.rotate(q, {x:0,y:0,z:-1})`) |
| Steering input | -1 = left, +1 = right |
| Positive yaw | counter-clockwise seen from +Y (right-hand rule) |
| Gravity | 9.81 m/s² (the legacy game-unit value 24 is mapped at the adapter) |
| Fixed step | 1/120 s, accumulator with a 24-substep cap |

The vehicle body origin is the **suspension mount plane**, not the ground. At static
ride height it is roughly `wheelRadius + suspension length` above the contact patch.

## 2. Core architecture

```
Body ── shape: Box | Sphere | Plane | Mesh
  │
World
  ├─ integrateVelocity(dt, gravity)
  ├─ updateAABB()
  ├─ collectManifolds(dt)       dynamic pairs + static meshes / planes
  ├─ ContactConstraint.preStep  effective mass, bias, warm start
  ├─ solveVelocity (N iterations, deterministic key order)
  ├─ integratePosition(dt)
  ├─ solvePosition (M iterations, non-linear projection)
  ├─ storeImpulses              warm-start cache for the next step
  └─ begin/end contact events, sleeping islands, stats
```

### Shapes and narrowphase

- **BoxShape** — SAT with 15 axes. Face contacts collect a clipped polygon; edge
  contacts fall back to the deepest segment pair. Manifolds are reduced to at most
  four points with stable local anchors.
- **SphereShape** — sphere/sphere, sphere/box (closest point on OBB) and sphere/plane.
- **PlaneShape** — infinite plane; box vertices are projected and reduced.
- **MeshShape** — triangle soup built from a `TriangleMesh`, `BufferGeometry` or raw
  triangles. A SAH BVH is built once; triangle queries return only nearby leaves.
  `MeshShape.raycast` and `closestPoint` are available for gameplay queries.

Per-triangle `friction` and `restitution` are read from the source triangle (the
legacy `TriangleMesh` stores them) and propagated to the contact constraint, so a
wet patch of road really does slide more than a dry one, for the chassis as well as
for the tires.

### Solver

Sequential impulses with:

- accumulated normal impulse clamped to `[0, ∞)`;
- Coulomb friction with a two-axis tangent basis and the friction circle;
- restitution applied above a small relative-velocity threshold (so resting
  contacts do not jitter);
- Baumgarte position bias (`beta = 0.2`, slop 5 mm) plus a separate non-linear
  position pass;
- warm starting keyed by stable body-pair and local-anchor contact IDs;
- deterministic ordering by body-pair key, identical across runs.

The vehicle chassis uses 10 velocity iterations and 4 position iterations by
default. The lab's 60-box stack and 40-prop scenes run at ~0.6–1.6 ms per step.

### Speculative contacts

Every dynamic body computes a lookahead

```
lookahead = max(0.01, |v| · dt · (body.ccd ? 1.25 : 0.2) + 2 · slop)
```

and the narrowphase keeps contacts that will be touched within that distance. The
contact constraint therefore stops a fast body at the surface instead of after it
has already crossed. This is how 120 m/s projectiles and 45 m/s cars are prevented
from tunnelling through thin walls in the test suite. It is *not* a swept-box
continuous solver; very thin, very fast, highly rotating bodies can still miss.

### Sleeping

Bodies under 0.06 m/s and 0.12 rad/s for 0.6 s sleep. They wake on contact with an
awake body, force, torque or impulse. The car chassis is created with
`allowSleep: false`, so the engine and aids keep running while the car is
stationary. Props sleep normally.

### Ray casts

`World.raycast(origin, direction, maxDistance, options)` returns the closest
`{ hit, distance, point, normal, body, shape, friction, restitution }`. Boxes, spheres,
planes and BVH meshes are supported. `options.filter` can reject bodies.

### Events

`world.events.beginContact` and `.endContact` are filled each step. Each entry
carries the bodies and manifold (begin) or the pair key (end).

## 3. Vehicle model

### 3.1 Suspension

Four independent corners at ±wheelbase/2 and ±track/2. Each step:

1. **Support query** — the surface mesh's `closestPoint` around the mount, falling
   back to `groundPlaneY`. The returned normal is oriented upward; the hit friction
   is stored on the contact.
2. **Length and compression** — `length = clamp(distance − wheelRadius, min, max)`,
   `compression = rest − length`.
3. **Spring** — `F = k · x · (1 + progressive · x)`.
4. **Damper** — bump rate when the wheel approaches, rebound rate when it extends.
5. **Bump stop** — extra progressive force below `min`.
6. **Anti-roll bar** — compression difference across an axle, clamped so no wheel
   load becomes negative.
7. **Load transfer** — longitudinal and lateral transfer routed with
   `_distributeLoad`, proportional to each wheel's current load. The sum is
   conserved and loads stay non-negative.
8. **Tire forces** at the contact patch, applied at the wheel arm.

The support length is also exported for rendering, which is why the lab's wheels
visibly droop and compress.

### 3.2 Tires

`calculateTireForces({ slipAngle, slipRatio, normalLoad, frictionCoefficient })`
returns `{ Fx, Fy, Mz, utilization, ... }`.

- Slip angle comes from the wheel's velocity in its steered frame; the slip ratio
  comes from wheel angular speed versus ground speed.
- A Pacejka-style magic formula (`B, C, E, peak`) is available; the default uses a
  calibrated curve with load sensitivity:
  `mu_eff = mu · (1 − loadSensitivity · (load/referenceLoad − 1))`.
- Relaxation length and relaxation angle low-pass the slip inputs so a step input
  does not produce an instant force spike.
- Combined slip (`calculateCombinedTireForces`) applies a normalised combined-slip
  curve and then clamps the resultant to the friction ellipse. Longitudinal demand
  reduces lateral capability; the total never exceeds `mu · load`.
- Camber thrust, rolling resistance and aligning torque are included.

### 3.3 Drivetrain

- 6-speed automatic: gear ratios `[3.2, 2.1, 1.5, 1.18, 0.94, 0.78]`, final drive
  3.9, shift time 0.16 s, rev limiter 6650, idle 900.
- Engine torque is derived from `engineAcceleration` in the legacy config, so old
  tuning values keep their feel.
- Open, limited-slip (TBR) and locked differentials; RWD/FWD/AWD layouts.
- Engine braking and a handbrake that locks the rear axle.
- Wheel spin is integrated per wheel from drive torque, brake torque and tire
  reaction, so wheelspin and lock-up are emergent rather than scripted.

### 3.4 Aids

- **ABS**: cuts brake torque on a wheel when `|slipRatio| > absSlip` and reports
  `absActive`.
- **TCS**: reduces throttle when a driven wheel exceeds `tcsSlip`.
- **ESC**: compares yaw rate with the steering-derived target and applies a brake
  torque to the outside front wheel when the error exceeds `escYawThreshold`.
All three are configurable and exposed in telemetry.

### 3.5 Body, aero and geometry

- Drag: `0.5 · rho · CdA · v²` opposite velocity.
- Downforce: `0.5 · rho · ClA · v²` downward at the body origin.
- Chassis roll/pitch stabilisation springs and dampers (not auto-alignment):
  they keep the body planted without steering the car for the player.
- Hardpoint suspension geometry solves camber/toe migration, instant centre,
  roll-centre height, caster, KPI, scrub radius, mechanical trail and jacking height.
  Disable with `suspensionGeometry.front.hardpoints.enabled = false`.

## 4. Public API

### 4.1 `RigidBodyVehicle`

```js
const car = new RigidBodyVehicle({
  surfaceMesh,            // { closestPoint(point, maxDistance, up) } or TriangleMesh
  colliderMeshes: [],     // static meshes for the chassis (walls, kerbs, props)
  groundPlaneY: 0,        // optional infinite fallback plane
  position: { x: 0, y: 0.6, z: 0 },
  yaw: 0,
  config: { /* deep-merged over defaults */ }
});
```

Methods:

| Method | Description |
| --- | --- |
| `advance(dt, input)` | Accumulate and run fixed substeps. Returns `{steps, alpha}`. |
| `setInput(input)` | Normalise `{throttle, brake, steer, handbrake}`. |
| `setSurfaceMesh(mesh)` / `setColliderMeshes(list)` | Hot-swap geometry. |
| `addProp(options)` | Add a dynamic box/sphere prop; returns the body. |
| `applyImpulse(impulse, worldPoint)` | Impulse in world space. |
| `getWheelState()` | `{steer, omega[4], contacts[4], slip[4]}`. |
| `getTelemetry()` | Latest telemetry object. |
| `getSnapshot()` | Deep clone of state, telemetry, config and world stats. |
| `interpolatedState()` | Pose for the render frame. |
| `reset(options)` | Reset pose, velocity, accumulator and layout. |

### 4.2 `PlayCarPhysicsEngine` (unified 3.0)

Adds the legacy surface and the rigid model:

| Method | Description |
| --- | --- |
| `advance/update/step(dt, input)` | Accepts `{gas,brake,left,right,handbrake}` or numeric input. |
| `setConfig(patch)` | Validates and applies 1.8 config keys; rigid config is remapped. |
| `setWeather({wetness, wind})` | Scales tire μ and applies wind to tire forces. |
| `setRigidConfig(patch)` | Deep-merge any 3.0 vehicle config. |
| `setDifferential(type, layout, bias)` | `open|lsd|locked`, `rwd|fwd|awd`. |
| `setSurfaceMesh/setColliderMeshes/addColliderMesh` | Geometry hot-swap. |
| `addProp(options)` | Dynamic prop in the same world. |
| `getTelemetry()` | Legacy telemetry plus 3.0 fields. |
| `getSnapshot()` | Adds `solver`, `rigid`, `rigidConfig`. |
| `reset(position, yaw, up)` | Spawn/reset. |
| `rigidBody` | The underlying `RigidBodyVehicle`. |

### 4.3 Legacy config → 3.0 mapping

| Legacy key | 3.0 effect |
| --- | --- |
| `gravity` | world gravity magnitude |
| `maxSpeed` | speed cap used by the drivetrain |
| `engineAcceleration` | `peakTorque = a · mass · R / (firstGear · finalDrive · efficiency)` |
| `brakeAcceleration` | brake and handbrake torque |
| `tireGrip` | tire μ (multiplied by `1 − 0.42 · wetness`) |
| `handbrakeGrip` | rear handbrake torque scale |
| `maxSteerAngle` | steering max angle |
| `highSpeedSteerScale` | steering reduction with speed |
| `suspensionRestLength` / `suspensionReach` | suspension rest / max |
| `bodyHalfExtents` | chassis collider half extents |
| `aerodynamicDrag` | `dragArea` |
| `maxDownforce` | `downforceArea` |
| `collisionRestitution` / `collisionFriction` | chassis material |
| `fixedTimeStep` / `maxSubsteps` | rigid fixed step / substep cap |

### 4.4 Telemetry schema

```js
{
  tick, contacts, speed, downforce,
  normalLoads: [4], forces: [4], suspension: [4],
  wheelGeometry: [4], rollCenter, bodyTilt, bodyStabilization,
  gear, rpm, drivetrain: { gear, rpm, shiftTimer, gearRatio, wheelTorque },
  absActive, tcsActive, escActive, aids: { abs, tcs, esc },
  antiRoll, loadTransfer: { longitudinal, lateral },
  differential, driveLayout, drivenWheels: [..],
  chassisContacts, collisionImpacts
}
```

The unified facade adds `tireLoads`, `wheelSlip`, `slipAngle`, `tireUtilization`,
`grounded`, `airborne`, `gripState`, `drivenWheels`, `driveLayout`, `differential`,
`antiRoll`, `rollCenter`, `wheelGeometry` and `bodyTilt`.

Suspension entries:

```js
{ wheel, compression, rawLength, springForce, damperForce,
  bumpStopCompression, bumpStopForce, normalLoad, velocity }
```

Tire/force entries include `slipRatio`, `slipAngle`, `normalLoad`, `Fx`, `Fy`,
`utilization`, `surfaceFriction`, `contactPoint`, `wheelCenter` and `spin`.

## 5. Examples

### Drop test with sleeping and materials

```js
const Core = require('./physics/rigidbody-core.js');
const world = new Core.World({ gravity: { x: 0, y: -9.81, z: 0 } });
world.addGroundPlane(0, { friction: 0.9, restitution: 0.02 });
const box = world.addBody(new Core.Body({
  mass: 8, shape: new Core.BoxShape({ x: 0.4, y: 0.4, z: 0.4 }),
  position: { x: 0, y: 2, z: 0 }, friction: 0.7, restitution: 0.05
}));
for (let i = 0; i < 400; i++) world.fixedStep(1 / 120);
console.log(box.position.y, box.sleeping, world.stats.contacts);
```

### Ray cast against a mesh

```js
const hit = world.raycast({ x: 0, y: 10, z: 0 }, { x: 0, y: -1, z: 0 }, 20);
if (hit.hit) console.log(hit.point, hit.body.friction);
```

### Vehicle tuning

```js
car.setRigidConfig({
  suspension: { stiffness: 40000, dampingBump: 5000, dampingRebound: 7000, progressive: 0.6 },
  tire: { mu: 1.15, combined: { peak: 0.92 } },
  aids: { abs: true, tcs: true, esc: true },
  antiRoll: { front: 9000, rear: 7000 },
  differential: { type: 'lsd', layout: 'awd', limitedSlip: 3.2 },
  downforceArea: 1.1
});
```

### Static mesh from a Three.js geometry

```js
const mesh = TriangleMesh.fromBufferGeometry(geometry, matrix.elements, 8);
const walls = new Core.MeshShape(mesh.triangles.map(t => ({ a: t.a, b: t.b, c: t.c, friction: 1 })));
world.addStaticMesh(walls);
```

## 6. Validation and performance

- `npm test` — 125 Node tests (`node --test`), no browser required.
- `npm run smoke` — boots the real `physics/scene3d.js` with the vendored
  Three.js and a stubbed WebGL renderer, drives it for 300 frames, and checks the
  car, geometry, telemetry and scene status.
- `npm run bench` — fixed-step throughput.
- `npm run build` — regenerates the bundle and parses it with `new vm.Script`.

Representative results (Node 22, single core):

| Metric | Result |
| --- | --- |
| Core 60-box stack (160 contacts) | 1.3–1.6 ms/step |
| Vehicle + 40 props | 0.96 ms/step |
| 120 Hz budget | 8.33 ms/frame |
| Box settling | y ≈ 0.5 m, sleeps ≈ 1.7 s |
| Restitution 0.7 | peak ≈ 1.758 m |
| 120 m/s projectile vs wall | no tunnelling |
| μ = 1.1 on 30° slope | holds |
| μ = 0.05 on 30° slope | slides |
| Determinism | bit-identical repeat runs |

The full regression list lives in `tests/` and is run by
`.github/workflows/playcar-physics-tests.yml`, which also rebuilds the bundle and
fails if the committed artifact differs from the sources.

## 7. Limitations

The engine is deliberately scoped to vehicle sandboxes and arcade/simcade stunts:

- dynamic shapes are boxes and spheres; no convex hulls, capsules, compounds or
  triangle-mesh dynamics;
- no joints/constraints beyond contacts (no ragdolls, ropes or hinges);
- dynamic broadphase is a pair loop (O(n²)); only static meshes use a BVH;
- "CCD" is speculative-contact based, not a swept convex solver;
- no soft bodies, fluids, cloth or destruction;
- tire behaviour is a calibrated analytic model, not a measured tire dataset;
- no multithreading or WASM.

These are intentional trade-offs for a dependency-free, inspectable engine that
runs at 120 Hz in a browser tab and remains understandable in a single repository.

## 8. Upgrading from 2.x

- The old kinematic `PlayCarPhysicsEngine` is still available as
  `physics/playcar-physics-engine.js`.
- The bundle's global is the unified 3.0 adapter, which accepts the same
  `advance(dt, input)`, `setWeather`, `setConfig`, `getSnapshot` and
  `TriangleMesh` API.
- `state.position` remains the legacy car origin (`rigid position − bodyOffset`);
  render the car with `rigidBody.interpolatedState()` if you want exact wheel
  placement.
- Snapshots gained `solver`, `rigid` and `rigidConfig`; existing fields are kept.
- Numeric results differ from 2.x because the motion is now rigid-body driven.
  Re-tune `tireGrip`, `engineAcceleration` and suspension values if your old
  presets feel different.
