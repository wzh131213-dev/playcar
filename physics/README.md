# PlayCar Physics Engine 1.7.0

A dependency-free, browser-first 3D vehicle handling engine. Rendering, input devices, track authoring, and game rules stay in the host game; this package owns vehicle motion and track-contact queries.

## What it provides

- Fixed-step simulation with render-frame interpolation and a substep budget.
- Vehicle acceleration, braking/reverse, rolling resistance, aerodynamic drag, wind, wet grip, downforce, slip-angle tire forces, and high-speed steering reduction.
- Four wheel probes, road mesh support, chassis/track penetration recovery, and optional triangle-mesh wall colliders.
- Airborne yaw, pitch/roll controls, and passive pitch recovery when the player is not actively pitching.
- Checkpoint crossing, weather settings, collision/contact state, telemetry, and defensive state snapshots.
- Triangle-mesh construction from raw triangles, Three.js-style BufferGeometry position/index data, or a procedural heightfield.
- Integration aliases for common game loops: `advance(dt,input)`, `update(dt,input)`, and `step(dt,input)`.
- Three handling profiles: `arcade`, `simcade` (default), and `simulation`.

This is a configurable **simcade** model, not a validated tire/vehicle homologation package. Realistic feel depends on your world scale, road mesh quality, vehicle dimensions, target handling, and calibration data. No physics engine can be guaranteed to fit every game perfectly without integration and tuning.

## Files

- `physics/playcar-physics-engine.js` — runtime engine (browser global and CommonJS export).
- `tests/playcar-physics.test.js` — engine behavior tests.
- `tests/physics-regression.test.js` — regression tests.
- `tests/physics-test-page.test.js` — checks that the published root page is the physics lab.
- `index.html` — browser test lab, not the racing game.

## Quick start (plain browser)

Load the engine before your game code:

```html
<script src="./physics/playcar-physics-engine.js?v=1.7.0"></script>
<script>
  const { PlayCarPhysicsEngine, TriangleMesh } = window.PlayCarPhysicsEngine;

  // Replace this example with your own road triangles or geometry adapter.
  const road = TriangleMesh.fromTriangles([
    [{x:-100,y:0,z:-100},{x:-100,y:0,z:100},{x:100,y:0,z:100}],
    [{x:-100,y:0,z:-100},{x:100,y:0,z:100},{x:100,y:0,z:-100}]
  ]);

  const vehicle = new PlayCarPhysicsEngine({
    surfaceMesh: road,
    position: {x:0,y:0.12,z:0},
    preset: 'simcade',
    config: {
      // Optional game-specific tuning overrides.
      wheelbase: 2.7,
      maxSpeed: 78
    }
  });

  function gameFrame(deltaSeconds, controls) {
    // Call once per render frame; the engine internally uses a fixed timestep.
    vehicle.update(deltaSeconds, {
      throttle: controls.accelerate, // 0..1
      brake: controls.brake,         // 0..1
      steer: controls.steer,         // -1 left .. +1 right
      handbrake: controls.handbrake  // 0..1
    });

    const pose = vehicle.interpolatedState();
    const telemetry = vehicle.getTelemetry();

    // The host game's renderer owns this part:
    // carObject.position.set(pose.position.x, pose.position.y, pose.position.z);
    // carObject.quaternion.set(pose.orientation.x, pose.orientation.y,
    //   pose.orientation.z, pose.orientation.w);
    // HUD speed = telemetry.speed;
  }
</script>
```

## Presets and calibration

```js
const car = new PlayCarPhysicsEngine({ surfaceMesh: road, preset: 'arcade' });
// preset: 'arcade' | 'simcade' | 'simulation'

// Per-game overrides take precedence over the selected preset:
const tunedCar = new PlayCarPhysicsEngine({
  surfaceMesh: road,
  preset: 'simulation',
  config: {
    gravity: 9.81,
    tireGrip: 1.4,
    maxSteerAngle: 0.38,
    bodyHalfExtents: { x: 0.95, y: 0.65, z: 1.9 }
  }
});
```

All lengths/speeds are **game-world units** unless you deliberately use a metre-based scale. If 1 world unit represents 1 metre, set vehicle dimensions, gravity, velocities, track mesh and camera scale consistently. The included profiles tune handling feel; they are not automatically calibrated against a specific real vehicle.

## Input contract

The engine accepts keyboard booleans and analog axes. Analog values are clamped to safe ranges.

| Property | Range | Meaning |
|---|---:|---|
| `throttle` / `accelerate` / `gas` | 0..1 | Acceleration |
| `brake` | 0..1 | Brake; holding after stopping selects reverse while grounded |
| `steer` / `steering` | -1..1 | Negative left, positive right |
| `left`, `right` | boolean | Digital steering fallback |
| `handbrake` / `hand` | 0..1 | Handbrake/drift input |
| `pitch` | -1..1 | Extra airborne pitch input |
| `roll`, `yaw` | -1..1 | Extra airborne roll/yaw input |

Example with a persistent input source:

```js
car.setInput({ throttle: 0.7, steer: -0.25 });
car.advance(deltaSeconds); // uses the last setInput() value
car.setInput({});          // explicitly release all controls
```

Always release controls when a key/button is released, a touch is cancelled, the page loses focus, the game pauses, or a controller disconnects. The engine does not own your browser events or input devices.

## Track and collider adapters

### Three.js BufferGeometry

Update the geometry's world matrix before passing its position/index attributes. The optional second argument is the geometry's 16-number world matrix elements.

```js
roadMesh.updateWorldMatrix(true, false);
const roadPhysicsMesh = TriangleMesh.fromBufferGeometry(
  roadMesh.geometry,
  roadMesh.matrixWorld.elements,
  6
);
const wallPhysicsMesh = TriangleMesh.fromBufferGeometry(
  wallMesh.geometry,
  wallMesh.matrixWorld.elements,
  6
);
car.setSurfaceMesh(roadPhysicsMesh);
car.setColliderMeshes([wallPhysicsMesh]);
```

### Procedural heightfield

```js
const terrain = TriangleMesh.fromHeightfield({
  width: 300,
  depth: 300,
  segmentsX: 80,
  segmentsZ: 80,
  origin: {x:0,y:0,z:0},
  heightAt: (x,z) => Math.sin(x * 0.025) * Math.cos(z * 0.03) * 2
}, 5);
```

For moving roads or destructible geometry, rebuild/update the corresponding mesh deliberately; this engine does not automatically track renderer transforms or deforming vertices. Avoid passing the same road mesh as a wall collider unless the host intends it to behave as a solid obstacle.

## State, forces, and telemetry

- `getState()` returns a copy of the simulation state so external UI code cannot mutate the live position accidentally.
- `interpolatedState()` returns a render-friendly pose between fixed simulation steps.
- `getTelemetry()` reports speed, longitudinal/lateral speed, slip angle, vertical speed, contact normal and grounded/airborne state.
- `setVelocity({x,y,z})` sets linear velocity after finite-number validation.
- `applyImpulse({x,y,z}, worldPoint?)` adds a velocity impulse; optional world point adds a simplified angular response.
- `setWeather({wind:{x,z},wetness})` configures horizontal wind and wetness from 0 to 1.
- `addCheckpoint({id,position,forward,radius})`, `setCheckpoint(id)`, and `resetToCheckpoint()` manage checkpoint recovery.
- `debugSnapshot()` exposes step timing, tick count, collision counters and dropped simulation time.

## Testing

With Node.js installed:

```bash
node --check physics/playcar-physics-engine.js
node --test tests/*.test.js
```

The root page is a visual/browser test lab and does not include the original racing game. Automated Node tests validate the numerical/API layer; a real browser and target-device run is still needed to verify a specific game's renderer, touch controls, camera, and frame pacing.
