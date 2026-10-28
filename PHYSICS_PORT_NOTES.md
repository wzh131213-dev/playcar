# PolyTrack movement-logic port notes

## Confirmed facts from the supplied archive
- `realistic.js` explicitly describes itself as a rendering-only mod; it adjusts lighting, materials, shader details and atmospheric effects, not vehicle physics.
- `lib/polytrack_physics.js` is an Emscripten-generated loader for `polytrack_physics.wasm`. It exposes functions including `initializeCarCollisionShape`, `addTrackPartConfiguration`, `createCarModel`, `updateCarModel`, and `testDeterminism`.
- `simulation_worker.bundle.js` initializes collision-shape vertices and track-part vertex data before creating a car model. Its control state includes `up`, `right`, `down`, `left`, and `reset`. Its realtime simulation accumulates elapsed time and advances in 1 ms increments.
- Therefore, the original WASM physics cannot be safely transplanted as a drop-in into playcar's current procedural Three.js track: it expects PolyTrack-specific track-part geometry, collision-shape data, car model state and worker protocol.

## Adaptation now used in playcar
The current `index.html` uses a browser-native vehicle model compatible with playcar's procedural track. The latest revision repairs a malformed physics-script section and uses the PHYS tuning constants for longitudinal force/drag, speed-sensitive steering, track-relative heading correction, lateral slip/grip damping, handbrake drift, edge impact loss, and sampled road-height following. Keyboard and touch inputs are mapped to driving intents. A JavaScript syntax parse check passed after the repair.

## Important distinction
This is a PolyTrack-like behavioral adaptation, not a direct port of the original WASM solver or its exact formulas/constants. The original solver's full behavior has not been reverse-engineered or validated against reference telemetry. A true direct WASM integration would also require adapting the original track geometry and car-state protocol.

## Current PlayCar status (2026-10-28)

The old “track-space dynamics” paragraph above records an earlier intermediate revision and is no longer the current implementation. The feature branch now contains **PlayCar Physics Engine 1.0**, an original JavaScript module in `physics/playcar-physics-engine.js`. It is integrated into `index.html` using the existing Three.js vehicle and road geometry. It implements fixed-step world-space motion, 3D triangle road queries, four wheel-probe contacts, gravity/air controls, landing response, triangle-vs-oriented-box static collision, checkpoints, state interpolation, and an optional physics debug panel.

The regression suite lives in `tests/playcar-physics.test.js` and is run by `.github/workflows/playcar-physics-tests.yml`. The authoritative status is the GitHub Actions result for the latest commit; a real iPad/iPhone browser playtest remains necessary.

The existing procedural course is not a vertical loop or a set of designed jump ramps. A test mesh confirms 3D loop-surface queries only; it does not validate a complete drivable loop. Do not claim full loop-course support until genuine 3D track geometry is authored and tested. No PolyTrack WASM or proprietary implementation is included.
