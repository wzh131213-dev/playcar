# PolyTrack Physics Port Assessment

## Confirmed facts from the supplied archive

- `realistic.js` explicitly says it changes rendering only; it does not change physics or gameplay.
- `lib/polytrack_physics.js` is an Emscripten/WebAssembly loader. It exposes the compiled physics functions.
- `polytrack_physics.wasm` contains the compiled low-level physics implementation; the actual formulas are not readable as ordinary JavaScript source.
- `simulation_worker.bundle.js` initializes the physics engine with a version check for 0.6.3 and configures:
  - car collision-shape vertices and a car mass offset;
  - track-part vertex arrays, IDs, optional detectors, and start offsets;
  - a car model from track part data and a start transform;
  - fixed-step control updates (up/right/down/left/reset);
  - a 227-byte car-state buffer returned after each update.
- The supplied worker expects a serialized PolyTrack track and PolyTrack model/track-part data. It is not a standalone steering function.

## Current playcar architecture

The current `index.html` builds its own procedural Three.js track and a primitive car from boxes and cylinders. Its track is represented as a Catmull-Rom curve and sampled road meshes. It does not currently provide PolyTrack's serialized track format, track-part configuration objects, collision-shape vertex array, or state-buffer decoder.

## Porting decision

Do not copy `realistic.js` as physics, and do not label the current hand-written arcade equations as the original PolyTrack physics.

A genuine engine port needs one of these approaches:

1. **Engine-level port:** bring over the compatible physics loader and WASM, then build a compatibility bridge that supplies valid collision vertices, track-part configuration, start transform, fixed-step controls, and a verified decoder for the 227-byte car-state buffer. This is the path needed to use the original compiled physics engine; it has not been completed or browser-tested.
2. **Behavioral approximation:** keep playcar's procedural track and implement independently tuned acceleration, steering, tire grip, drift, gravity, and collision behavior. This can approximate the feel but is not a transplant of the original physics engine.

## Verification status

- Static archive inspection: completed.
- GitHub playcar source inspection: completed.
- Original WASM integration into playcar: not implemented.
- Real-browser driving test: not run.

This report intentionally separates verified source facts from engineering inference.
