# PolyTrack movement-logic port notes

## Confirmed facts from the supplied archive
- `realistic.js` explicitly describes itself as a rendering-only mod; it adjusts lighting, materials, shader details and atmospheric effects, not vehicle physics.
- `lib/polytrack_physics.js` is an Emscripten-generated loader for `polytrack_physics.wasm`. It exposes functions including `initializeCarCollisionShape`, `addTrackPartConfiguration`, `createCarModel`, `updateCarModel`, and `testDeterminism`.
- `simulation_worker.bundle.js` initializes collision-shape vertices and track-part vertex data before creating a car model. Its control state includes `up`, `right`, `down`, `left`, and `reset`. Its realtime simulation accumulates elapsed time and advances in 1 ms increments.
- Therefore, the original WASM physics cannot be safely transplanted as a drop-in into playcar's current procedural Three.js track: it expects PolyTrack-specific track-part geometry, collision-shape data, car model state and worker protocol.

## Adaptation now used in playcar
The current `index.html` uses a browser-native arcade vehicle model compatible with playcar's own procedural track. It includes throttle/braking, speed-dependent steering, lateral slip and grip damping, handbrake grip reduction, road-edge response, and sampled road-height following. Keyboard and touch inputs are mapped to equivalent driving intents.

## Important distinction
This is a PolyTrack-like behavioral adaptation, not a direct port of the original WASM solver or its exact formulas/constants. The original solver's full behavior has not been reverse-engineered or validated against reference telemetry. A true direct WASM integration would also require adapting the original track geometry and car-state protocol.
