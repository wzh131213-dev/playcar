# PlayCar Rigid-Body Vehicle Physics 2.8.0

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

## Suspension bump stops 2.2.0

Suspension contacts retain the unclamped wheel travel length so the end-stop logic can detect travel beyond the minimum ride length. A progressive bump-stop force is added to the spring load:

`F_bump = k_bump · x · (1 + x / range)`

where `x = max(0, minTravel - rawLength)`. Configure `suspension.bump` (N/m) and `suspension.bumpStopRange` (m); setting `bump: 0` disables the bump stop. Per-wheel telemetry is available in `telemetry.suspension`, including raw length, compression, bump-stop compression, bump-stop force, and resulting normal load.


## Longitudinal dynamic load transfer 2.6.0

The solver estimates longitudinal acceleration demand from engine wheel torque and service-brake torque, bounds it by the configured tire friction limit, and transfers normal load between the front and rear axles using the configured centre-of-mass height and wheelbase:

`ΔF = m · aₓ · hCG / wheelbase`

Positive acceleration transfers load rearward; braking transfers load forward. The transfer is applied to the wheel normal loads before tire-force calculation, is limited by the available axle support load to prevent negative wheel loads, and preserves the sum of the four wheel normal loads. Read the signed front-axle gain in `telemetry.loadTransfer.longitudinal` (positive means forward load transfer during braking; negative means rearward transfer during acceleration). This is a force-demand estimate, not a full suspension-geometry or measured-vehicle model.


## Lateral cornering load transfer 2.6.0

The solver estimates lateral acceleration demand from forward speed and front-wheel steering angle using a bicycle-model approximation, then redistributes normal load across the left and right wheel pairs:

`ΔF_lat = m · a_y · hCG / track`

Positive steering while moving forward transfers load to the left/outside wheels; reversing steering reverses the transfer. The signed amount is exposed as `telemetry.loadTransfer.lateral` (positive means left-side gain). The redistribution is applied to wheel support loads before tire-force evaluation, bounded by the available side loads, and conserves total normal load. This simplified model does not yet include a measured roll-centre/roll-stiffness suspension geometry model.


## Chassis roll and pitch stabilization 2.6.0

The rigid body now applies configurable spring-damper torques around the chassis' local roll and pitch axes. The chassis tilt is measured from its rotated up vector; angular velocity is transformed into the body frame before damping, and the resulting local torque is transformed back to world space before entering the same angular-velocity integration used by suspension and tire contact torques.

Default tuning:

| Parameter | Default | Unit |
|---|---:|---|
| `body.rollStiffness` | 9000 | N·m/rad |
| `body.rollDamping` | 2200 | N·m·s/rad |
| `body.pitchStiffness` | 14000 | N·m/rad |
| `body.pitchDamping` | 2800 | N·m·s/rad |

Example:

```js
const vehicle = new RigidBodyVehicle({
  config: {
    body: {
      rollStiffness: 9000,
      rollDamping: 2200,
      pitchStiffness: 14000,
      pitchDamping: 2800
    }
  }
});
```

Set an individual stiffness or damping value to `0` to disable that contribution. Telemetry exposes `bodyTilt` (roll, pitch and body-frame roll/pitch rates) and `bodyStabilization` (active coefficients).

These torques provide chassis attitude spring-damper behavior; they do not replace wheel-contact forces, suspension anti-roll bars, or the separate axle/side normal-load transfer estimates. The current model is a simplified body attitude controller, not a full suspension roll-centre, pitch-centre, or measured chassis compliance model. The defaults require calibration against the actual vehicle and track.


## Suspension geometry and tire contact attitude 2.6.0

The rigid-body solver now carries separate front/rear suspension alignment settings in `config.suspensionGeometry`. Each wheel derives camber, toe, steering direction and wheel-frame orientation from its current suspension compression and the sampled road normal. The tire forward/right axes are projected onto the local road tangent, so they remain orthogonal to the surface on banked and sloped triangles.

Default axle tuning (degrees unless stated):

| Parameter | Front | Rear |
|---|---:|---:|
| Static camber | -0.5° | -1.0° |
| Camber gain | -22°/m | -15°/m |
| Static toe | 0° | 0.08° |
| Toe gain | 0.35°/m | -0.22° |
| Static roll-centre height | 0.12 m | 0.17 m |

Compression-dependent camber/toe are evaluated continuously at each fixed step. The front axle also receives steering input; rear toe remains independently configurable. Camber is exposed in radians and degrees, and a small configurable camber-thrust contribution is added to relaxed tire slip angle through `tire.camberThrustGain` (default 0.08).

Roll-centre height migrates with suspension compression and left/right wheel travel. The lateral load-transfer estimate now uses the effective moment arm `max(0.05, com.y - averageRollCenterHeight)`, rather than treating the centre of mass as the full roll lever arm. This is still a tunable kinematic approximation, not a complete instant-centre solver; measured pickup-point coordinates and anti-dive/anti-squat geometry are not yet modeled.

Per-wheel results are available in `telemetry.wheelGeometry`: axle, camber, toe, roll-centre height, normalized travel, wheel-centre position, road contact normal and tire tangent axes. Axle averages and the effective lateral moment arm are available in `telemetry.rollCenter`.


## Hardpoint-based suspension kinematics 2.8.0

Front and rear axles now have independent, configurable front-view double-wishbone hardpoints under `config.suspensionGeometry.front.hardpoints` and `rear.hardpoints`. Each axle defines upper/lower inner pivots and upper/lower outer ball joints in metres, with outward-positive lateral `x` and upward-positive `y`.

For each suspension compression step the solver:
1. Holds the lower control-arm length and solves the lower ball-joint lateral position at the requested vertical travel.
2. Intersects the upper-arm circle with the upright-length circle to locate the upper ball joint, selecting the solution closest to the static pickup point.
3. Derives camber change from the solved upright angle relative to its zero-travel reference.
4. Intersects the upper and lower control-arm axes to obtain the front-view instant centre, then projects the contact-patch-to-instant-centre line to the vehicle centreline for a roll-centre height estimate.
5. Applies the solved lower-joint lateral migration to the wheel contact lever arm so the contact force location moves with the suspension geometry.

Default front/rear hardpoints are illustrative starting geometry, not measured manufacturer pickup coordinates. The solver is a 2D front-view double-wishbone model: it does not yet solve 3D control-arm sweep, caster, KPI/SAI, steering-axis scrub radius, compliance/bushing deflection, anti-dive, anti-squat, or multi-link/MacPherson strut constraints. If a requested travel position has no valid circle intersection, the solver marks `hardpointValid=false` and falls back to the configured camber/roll-centre approximation without emitting non-finite geometry.

Per-wheel telemetry adds `hardpointValid`, `instantCenter`, `upperOuter`, `lowerOuter`, and `wheelOffsetX`; the existing `wheelGeometry` continues to expose camber, toe, roll-centre height, contact normal and tire tangent axes. Set `hardpoints.enabled=false` per axle to retain the previous parameterized approximation.


## 3D steering-axis geometry 2.8.0

Front and rear axle alignment now includes `casterDeg`, `kingpinInclinationDeg`, `scrubRadius`, and `mechanicalTrail`. Steering and toe rotation are applied about a normalized three-dimensional steering axis built from the contact normal, wheel lateral axis, caster, and side-dependent kingpin inclination. The resulting wheel-forward vector is reprojected onto the road tangent plane before tire forces are evaluated.

Scrub radius and kingpin inclination produce a signed steering jacking displacement `jackingHeight = scrubRadius * sin(steer + toe) * sin(KPI)`; the wheel centre is displaced along the contact normal by this amount. The geometry is exposed per wheel through `steeringAxis`, `casterDeg`, `kingpinInclinationDeg`, `scrubRadius`, `mechanicalTrail`, and `jackingHeight` telemetry.

Default front values are 5.5 degrees caster, 8.5 degrees KPI, 35 mm scrub radius and 45 mm mechanical trail. Rear steering-axis values default to zero. These are generic illustrative tuning values, not measured values for a specific production vehicle. Mechanical trail is exposed as steering geometry telemetry; it is not yet coupled into a separate steering-rack torque model. The control-arm instant-centre solver remains a 2D front-view double-wishbone approximation; this update adds a 3D steering axis but does not yet solve full 3D control-arm sweep, caster gain from hardpoint travel, or compliance.
