# PlayCar 3D Lab — Track and Telemetry

The 3.0 lab (`index.html`) runs the generated bundle and a pinned Three.js
(`vendor/three.module.min.js`, with a CDN fallback). The physics is the rigid-body
stack described in [`physics/RIGIDBODY-ENGINE-3.0.md`](./physics/RIGIDBODY-ENGINE-3.0.md).

## Track

- Closed Catmull-Rom circuit with a start straight through `(0, 0.08, 34)`.
- Curvature-based banking, low-pass filtered over nine segments and clamped to ±0.16
  rad so the closure seam and the tightest corner do not become 20° walls.
- Continuous low barriers on both sides, implemented as vertical triangle colliders
  for the chassis. They are not part of the wheel support surface.
- A wet section (about 55–64 % of the loop) with per-triangle friction 0.55; dry
  road uses 1.0. Shared by the tire support and the chassis contact solver.
- A tabletop ramp on the start straight: 7 m rise to 1.85 m, 5 m crest, 9 m descent.
  The deck is a wheel surface; the chassis does not collide with the thin deck,
  which prevents a long rigid box from high-centring on the crest.
- Cones and crates are dynamic props in the vehicle's own rigid world, so they can be
  knocked into each other and the car.

## Telemetry and recording

The right-hand panel polls `PlayCarRuntime.snapshot()` every 100 ms and streams:

- state (legacy origin, velocity, orientation, angular velocity, grip/collision state);
- telemetry (speed components, slip angle, tire loads/slip/utilisation, suspension
  travel, RPM/gear, ABS/TCS/ESC, differential, driven wheels, load transfer);
- debug (physics milliseconds, substeps, contact count, collision events);
- config, weather and input.

`记录开始/暂停/清空` records a full snapshot every ~100 ms (up to 12,000 samples).
`导出 JSON` writes the snapshot plus the recording; `导出 CSV` flattens the recorded
objects into a header row plus one row per sample; `导出参数配置` writes the live
config and rigid config. All exports are generated from the running session, not
fabricated data.

## Parameter panel

- `全部物理参数` edits the legacy 1.8 config; each change is validated by
  `setConfig`, then remapped onto the rigid model (`tireGrip` → μ, accelerations →
  torque, suspension lengths, body half extents, aero, …).
- `高级车辆动力学` drives the 3.0 model directly: RWD/FWD/AWD, open/LSD/locked
  differential and TBR, suspension stiffness and rebound damping, ESC, TCS.
- `运行自检` verifies the config is finite, the snapshot has the expected sections,
  invalid values are rejected, the rigid body and world are connected, and the 3.0
  tuning API works.

## Validation scope

`npm test` includes a headless boot of the real scene (`npm run smoke`) which
mounts a stubbed canvas, builds the actual track, drives 300 frames, and checks the
car's position, speed, wheel contacts and telemetry. The Node circuit test also
covers the straight, the ramp jump and landing, props and wet grip.

These tests do not replace visual browser testing of the ramp takeoff, barrier
impacts, HUD placement, or touch controls on a real device. Open the page through a
static server (for example `python3 -m http.server`) rather than `file://`, because
the scene uses an ES module for Three.js.
