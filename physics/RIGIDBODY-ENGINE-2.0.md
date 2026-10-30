# PlayCar Rigid-Body Vehicle Physics 2.1.0

This package adds an independent rigid-body vehicle solver to the PlayCar physics branch. It is a new engine module; it does not silently replace the existing 1.8.0 simcade engine or change the current 3D lab's runtime selection.

## Solver order (fixed 1/120 s)

1. Four wheel contact probes query the supplied triangle surface and record point, normal, suspension length and compression.
2. Each wheel applies spring, damper and bump-stop force at its chassis lever arm.
3. Tire slip ratio and relaxed slip angle produce longitudinal/lateral force. Lateral force can use a configurable Pacejka-style Magic Formula curve; combined forces are limited by a configurable friction ellipse.
4. Engine torque, RPM, gear ratios, final drive, brake torque, handbrake torque, aerodynamic drag and downforce are evaluated.
5. Forces and lever-arm torques are accumulated on the chassis using mass, centre of mass and diagonal inertia tensor.
6. Semi-implicit Euler advances linear velocity/position; quaternion derivative advances orientation.
7. Contact recovery and swept centre sampling provide a basic CCD safeguard for the supplied track surface.
8. advance(renderDelta,input) retains accumulator remainder; interpolatedState() interpolates position and quaternion for rendering.

## Usage

Load physics/rigidbody-vehicle-engine.js as a classic script, then:

```js
const { RigidBodyVehicle } = window.PlayCarRigidBody;
const vehicle = new RigidBodyVehicle({ surfaceMesh: roadTriangleMesh, groundPlaneY: 0, position: {x:0,y:0.65,z:0} });
function frame(dt, controls) {
  vehicle.advance(dt, { throttle: controls.throttle, brake: controls.brake, steer: controls.steer, handbrake: controls.handbrake });
  const pose = vehicle.interpolatedState();
  // copy pose.position and pose.orientation to the render body
}
```

## Calibration and limitations

Units are SI-like. Defaults: chassis mass 1250 kg, gravity 9.81 m/s², wheelbase 2.65 m, track 1.56 m and dry-road friction coefficient μ=1.08. These are tunable starting values, not vehicle-specific measured data.

This is a dependency-free custom solver, not a full commercial rigid-body SDK. Collision uses triangle-mesh closest-point contact plus swept centre sampling; it is not a full convex-shape manifold solver or mathematically complete continuous collision detection. Inertia is a diagonal tensor in the body frame. Transmission is a simplified sequential automatic gearbox. Real track integration, wheel visual steering/rotation, browser WebGL rendering and iPad touch handling must be validated in the host game before switching the live scene to this engine.

## Test

Run from repository root: node --test tests/rigidbody-vehicle-engine.test.js

## Tire model 2.1.0

The default remains the legacy linear lateral-force model for compatibility. Enable the nonlinear curve per vehicle with:

```js
const vehicle = new RigidBodyVehicle({
  config: {
    tire: {
      pacejka: { enabled: true, B: 0, C: 1.3, E: 0.97, peak: 1 },
      frictionEllipse: { longitudinal: 1, lateral: 1 }
    }
  }
});
```

When `B` is zero, the solver derives it from cornering stiffness, `C`, and the current vertical-load friction limit. `peak` scales the Pacejka peak force. The friction ellipse couples longitudinal and lateral demand; each axis value scales that axis' available grip, and the combined normalized force is kept at or below one. The exported `calculateTireForces` helper is shared by the standalone solver and all-in-one browser bundle.
