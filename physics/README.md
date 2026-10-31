# PlayCar Physics 3.0

A dependency-free 3D rigid-body and vehicle-dynamics engine that runs in the browser,
in Node, and inside the interactive 3D lab (`index.html`).

PlayCar 3.0 is a ground-up rigid-body core plus a purpose-built vehicle layer. It is
**not** a wrapper around Bullet, Box2D, PhysX, Rapier or Ammo.js, and the single-file
bundle contains no WASM. The only external dependency is Three.js for rendering the
lab; the physics itself has none.

```
physics/rigidbody-core.js            general rigid-body world (shapes, solver, events)
physics/rigidbody-vehicle-engine.js  suspension, tires, drivetrain, aids, telemetry
physics/playcar-unified.js           3.0 runtime + 1.8-compatible facade
physics/playcar-physics-engine.js    legacy kinematic engine (compatibility only)
physics/scene3d.js                   Three.js driving lab (ES module)
physics/playcar-physics-all-in-one.js generated bundle (do not edit)
tools/build-bundle.js                bundle generator + syntax gate
tools/benchmark.js                   fixed-step throughput benchmark
vendor/three.*                       pinned Three.js 0.180.0 for offline labs
```

## Quick start

```bash
npm run build     # regenerate physics/playcar-physics-all-in-one.js
npm test          # 120+ Node tests (node --test)
npm run bench     # solver/vehicle throughput
npm run smoke     # boot the real 3D scene headlessly and drive it
```

### Browser

```html
<script src="./physics/playcar-physics-all-in-one.js?v=3.0.0"></script>
<script>
  const { PlayCarPhysicsEngine, TriangleMesh } = window.PlayCarPhysicsEngine;

  const surface = TriangleMesh.fromTriangles([
    [{ x: -100, y: 0, z: -100 }, { x: -100, y: 0, z: 100 }, { x: 100, y: 0, z: 100 }, 1.0],
    [{ x: -100, y: 0, z: -100 }, { x: 100, y: 0, z: 100 }, { x: 100, y: 0, z: -100 }, 1.0]
  ]);

  const car = new PlayCarPhysicsEngine({
    surfaceMesh: surface,
    groundPlaneY: 0,
    position: { x: 0, y: 0.08, z: 0 },
    preset: 'simcade'
  });

  function frame(dt, controls) {
    car.advance(dt, {
      throttle: controls.accelerate,  // 0..1
      brake: controls.brake,          // 0..1
      steer: controls.steer,          // -1 left .. +1 right
      handbrake: controls.handbrake   // 0..1
    });
    const pose = car.rigidBody.interpolatedState(); // render with this
    draw(pose, car.getTelemetry());
  }
</script>
```

`gas/left/right` booleans are accepted for compatibility with 1.x input maps
(`{ gas, brake, left, right, handbrake }`).

### Node

```js
const Core = require('./physics/rigidbody-core.js');
const Vehicle = require('./physics/rigidbody-vehicle-engine.js');

const world = new Core.World({ gravity: { x: 0, y: -9.81, z: 0 } });
world.addGroundPlane(0, { friction: 0.9 });
world.addBody(new Core.Body({
  mass: 10,
  shape: new Core.BoxShape({ x: 0.3, y: 0.3, z: 0.3 }),
  position: { x: 0, y: 2, z: 0 },
  ccd: true
}));
for (let i = 0; i < 600; i++) world.fixedStep(1 / 120);
```

## What is in 3.0

### Rigid-body core

- Shapes: oriented box, sphere, infinite plane, indexed triangle mesh with a
  surface-area-heuristic BVH.
- Narrowphase: SAT box/box with face-clipped manifolds, triangle/box
  (vertex-face, vertex-box, edge-edge fallbacks), sphere/box, sphere/mesh and
  box/mesh aggregation with manifold reduction.
- Contacts: speculative contact generation with a velocity-scaled lookahead, so a
  car cannot tunnel through a thin barrier at race speed, plus four-point manifold
  reduction and stable contact IDs for warm starting.
- Solver: sequential impulses with accumulated normal/friction impulses, warm
  starting across frames, restitution, Coulomb friction with a tangent basis,
  Baumgarte bias, a separate non-linear position pass, velocity and position
  iteration counts, and deterministic solve ordering.
- Sleep islands: bodies with low linear/angular velocity for 0.6 s sleep and wake
  on contact, impulse, or force; the vehicle chassis is deliberately excluded.
- Ray casting against all shapes and the BVH mesh, contact begin/end events,
  material friction/restitution (including per-triangle road friction), and
  deterministic fixed-step/accumulator stepping.

### Vehicle layer

- Four independent suspensions with progressive springs, asymmetric bump/rebound
  damping, bump stops, anti-roll bars, and per-wheel normal-load telemetry.
- Longitudinal and lateral load transfer routed proportionally per axle/side, so
  load never becomes negative and the total load is conserved.
- Combined-slip tires: Pacejka-style magic formula, slip-angle and slip-ratio
  curves, load sensitivity, relaxation length, and a friction ellipse that
  prioritises longitudinal demand and clamps the resultant force.
- Drivetrain: 6-speed automatic with shift time, torque converter-ish launch
  behaviour, engine braking, reverse-free arcade forward drive, open / limited-slip
  (TBR) / locked differentials, and RWD / FWD / AWD layouts.
- Aids: ABS with per-wheel slip control, TCS with throttle cut, ESC yaw-rate
  correction. All report state in telemetry.
- Suspension geometry: hardpoint-derived camber/toe, caster, KPI, scrub radius,
  mechanical trail, jacking, roll-centre migration, and anti-dive/anti-squat.
- Aero: quadratic drag and downforce; chassis roll/pitch stabilisation springs for
  a planted simcade feel without arcade auto-alignment.
- Airborne attitude control: pitch/roll/yaw inputs drive a rate-limited PD torque
  while no wheel is in contact, so ramp jumps can be rotated; passive levelling
  resumes when the driver releases the inputs.
- Props: `addProp({ shape, halfExtents, radius, mass, position, friction,
  restitution })` inserts dynamic crates/cones into the same world, so the car can
  knock them around and the props collide with each other.
- Render interpolation (`rigidBody.interpolatedState()`), `getWheelState()`, and a
  deep-cloned `getSnapshot()` for UI/telemetry.

### Unified runtime

`PlayCarPhysicsEngine` in the bundle is the 3.0 adapter. It keeps the 1.8 public
surface (`advance`, `setInput`, `setWeather`, `setConfig`, `getSnapshot`,
`TriangleMesh`, presets) and adds:

```js
car.setRigidConfig({ suspension: { stiffness: 40000 }, tire: { mu: 1.2 }, aids: { esc: true } });
car.setDifferential('lsd', 'awd', 3.4);  // type, layout, limited-slip bias
car.rigidBody;                            // rigid vehicle (world, chassis, telemetry)
car.getWheelState();                      // { steer, omega[4], contacts[4], slip[4] }
car.addProp({ shape: 'cone', mass: 6, position: { x: 3, y: 1, z: 0 } });
```

Legacy config keys are mapped onto the rigid model every time `setConfig` changes
(`tireGrip`, `gravity`, `maxSpeed`, `engineAcceleration`, `brakeAcceleration`,
`suspensionRestLength`, `maxSteerAngle`, `highSpeedSteerScale`,
`bodyHalfExtents`, ...). Weather `wetness` scales the tire μ; wind and
per-triangle friction are applied to tire forces. See
[`RIGIDBODY-ENGINE-3.0.md`](./RIGIDBODY-ENGINE-3.0.md) for the full API, mapping
table, telemetry schema, and design notes.

## The 3D lab

`index.html` loads the single bundle and a pinned Three.js from `vendor/`, with a
CDN fallback. It contains:

- a closed Catmull-Rom circuit with smoothed banking, a wet section with lower
  per-triangle friction, continuous barriers, and a tabletop jump;
- a rigid-body prop set (cones and crates) that the car can hit;
- chase/free camera, skid marks, HUD (speed, gear, RPM, aids, slip, contacts);
- a control center that edits the legacy config and the 3.0 suspension, diff,
  ESC/TCS settings live, and exports JSON/CSV snapshots.

Open the page from a static server, e.g. `npx serve .` or `python3 -m http.server`.
ES modules are used for Three.js, so `file://` will not work.

## Compatibility

- `physics/playcar-physics-engine.js` remains the legacy 1.8 engine and is still
  covered by its own tests. The unified 3.0 adapter is a drop-in replacement for the
  bundle's global.
- The bundle exposes `window.PlayCarRigidBodyCore` (core), `window.PlayCarRigidBody`
  (vehicle layer) and `window.PlayCarPhysicsEngine` (unified 3.0 API).
- In the lab, `window.PlayCarRuntime` is the UI facade
  (`setInput/setWeather/setConfig/setRigidConfig/reset/snapshot/engine`).

## Performance

Measured with `npm run bench` on Node 22 (single core, no JIT warmup beyond 60
steps; your numbers will vary):

| Scenario | Cost | Throughput |
| --- | --- | --- |
| Core: 60-box stack, 160 contacts | 1.3–1.6 ms / step | ~630–760 steps/s |
| Vehicle: 1 car + 40 props | 0.96 ms / step | ~1,040 steps/s |
| 120 Hz frame budget | 8.33 ms | vehicle ≈ 11.6% of a core |

The vehicle runs fixed 1/120 s substeps with a 24-substep cap; rendering can run at
any frame rate and uses interpolated poses.

## Validation

`npm test` runs 120+ tests, including:

- core: box settling and sleeping, 120 m/s thin-wall non-tunnelling, restitution,
  slope friction (μ = 1.1 holds 30°, μ = 0.05 slides), stack stability, raycasts,
  determinism, contact events, wake-on-impact;
- vehicle: suspension bump stops, load transfer conservation, combined-slip
  friction ellipse, ABS on low μ, props settling, hardpoint geometry, caster/KPI,
  roll/pitch stabilisation, differential layouts;
- unified: legacy weather/config mapping, 3.0 telemetry, snapshot shape, runtime
  tuning;
- lab: the real `scene3d.js` booted headlessly with a stubbed WebGL renderer and
  driven for 300 frames (`npm run smoke`), plus end-to-end drives on the same
  circuit geometry (straight, ramp jump, landing, props, wet weather);
- page/build: the generated bundle parses, the lab loads the current bundle
  version, and the single-file build stays in sync.

## Honest limitations

- Dynamic bodies are boxes and spheres; triangle meshes are static geometry. There
  is no convex-hull decomposition, no capsule, no compound body and no joints.
- The dynamic broadphase is an AABB pair loop (O(n²)); static meshes use a BVH. The
  lab's tens of props are fine, thousands are not.
- "CCD" is implemented as speculative contacts with a velocity-expanded lookahead,
  not a swept-box continuous solver. It prevents tunnelling in the tested speed
  range but is not a replacement for a full conservative-advancement pass.
- The tire model is a calibrated Pacejka-style curve, not a thermal/brush model and
  not homologated against measured tire data.
- Determinism is guaranteed for a fixed step sequence on one JS engine; floating
  point differences across engines are possible.
- Three.js is only needed by the lab; physics files run headless.

## History

- 3.0: rigid-body core, vehicle layer, combined-slip tires, aids, props, lab rebuild.
- 2.8: hardpoint suspension geometry, caster/KPI, bump stops.
- 1.8: per-triangle friction and solid ground support.
- 1.7: original simcade kinematic engine.
