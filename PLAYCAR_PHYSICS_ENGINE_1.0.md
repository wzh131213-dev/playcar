# PlayCar Physics Engine 1.0

This document describes the independently written physics module integrated into the PlayCar project. It is a behavioral implementation; it does not contain PolyTrack source, WASM, proprietary assets, or copied solver code.

## Confirmed before implementation

The inspected main/index.html used Three.js 0.160.0, generated a closed Catmull–Rom route from 720 sampled points, built the road as a triangle mesh, and rendered a primitive car from box/cylinder meshes. The prior update(dt) had throttle/brake/drag calculations, a speed-adjusted steering scalar, lateral slip damping, handbrake grip reduction, and a nearest-road-sample clamp. The prior version did not independently integrate vertical velocity, solve a 3D vehicle body against collision triangles, or separate physics cadence from rendering cadence. The file also called an undefined placeCar() at startup.

Those existing calculations were not counted as a complete 3D physics engine.

## Module map

- physics/playcar-physics-engine.js — dependency-free vector/quaternion maths, fixed-step accumulator, vehicle dynamics, triangle-mesh spatial index, closest-point road queries, 3D oriented-box/triangle SAT collision detection, landing/contact response, checkpoint bookkeeping, and state snapshots.
- index.html — keeps the current scene, car meshes, HUD, camera modes, lap flow and touch UI; the vehicle transform now comes from the physics engine's interpolated world position and quaternion. The existing road BufferGeometry is the road-contact surface; marked tree trunks and checkpoint gate bars/posts become triangle colliders.
- tests/playcar-physics.test.js — Node.js regression cases for acceleration, braking, steering, tire grip/handbrake slip, fixed-step frame-rate consistency, leaving a finite road, landing, wall collision, vertical-loop mesh sampling, reset/checkpoint state, and large frame deltas.
- .github/workflows/playcar-physics-tests.yml — runs syntax and regression checks on pushes to main/feature/** and on pull requests targeting main.

## Vehicle model

The engine advances at a fixed 1/120-second step and limits catch-up work per rendered frame. Rendering interpolates the prior and current physics states. Vehicle velocity is world-space; throttle acts along chassis forward, brake removes longitudinal velocity, aerodynamic and rolling resistance attenuate speed, tire grip damps lateral velocity within a grip limit, and steering authority is reduced as speed rises. Handbrake input lowers lateral grip to allow a controlled drift. In air, gravity remains active and steering/throttle/brake inputs apply reduced angular control.

Road contact samples four wheel-probe points against actual 3D road triangles. Contact uses the triangle's authored normal (not a guessed world-up angle), a suspension reach, penetration correction and a landing velocity impulse. Static obstacle collision transforms candidate triangles into the vehicle's oriented-box frame and applies a 3D triangle-vs-box separating-axis test before correcting position and velocity. This is materially different from a 2D distance clamp or a visual lean animation.

## Tuning

Edit the DEFAULTS object in physics/playcar-physics-engine.js:
- fixedTimeStep, maxSubsteps, maxFrameDelta
- gravity, maxSpeed, engineAcceleration, brakeAcceleration
- rollingResistance, aerodynamicDrag
- wheelbase, maxSteerAngle, highSpeedSteerScale
- tireGrip, handbrakeGrip, handbrakeYawMultiplier
- airYawRate, airPitchRate, airRollRate
- wheelProbeY, suspensionRestLength, suspensionReach
- bodyHalfExtents, bodyCenterY, collisionRestitution, collisionFriction

The optional HUD debug panel displays speed, vertical velocity, tire state, grounded status, collision state, last physics-step time, physics tick count, and dropped accumulated time.

## Compatibility and limitations

- Existing keyboard controls are retained. WASD/arrow keys control steering and throttle/brake, Space is handbrake, and R attempts a reset to the most recently crossed checkpoint (the on-screen restart button starts from the start line). Touch controls are hold-to-drive and release on pointer-up/cancel; blur clears all inputs.
- Static collision meshes currently exist for tree trunks and checkpoint gate posts/beams. The scene did not previously contain wall geometry, so the engine supports 3D wall meshes, and the regression suite creates a test wall, but the authored game course itself still has no continuous trackside wall.
- The current course data is a gently undulating Catmull–Rom loop with a mostly horizontal side-vector construction. It does not contain a vertical loop, inverted road, designed ramp jump, or a fully banked track mesh. The solver can query general 3D triangle surfaces and the test fixture samples a vertical loop mesh, but this is not evidence that the current playable course has been validated for high-speed loop driving. The next course-data step is to author genuine 3D/banked loop and ramp geometry, with consistent triangle winding and checkpoint frames, then tune against those exact surfaces.
- A loop contact-sampling fixture validates mesh query continuity, not a complete timed lap around a vertical loop.
- Automated Node tests are not a substitute for a real iPad/iPhone Safari/WebGL playtest. Deployment still needs to be checked after the change lands on the branch served by GitHub Pages.

## Validation status

The source includes runnable tests and a CI workflow. The checked-in status of each test run should be taken from the corresponding GitHub Actions result, not assumed from the existence of the test code. Mobile touch handling, browser loading, lap timing, steering feel, slope takeoff/landing tuning, and complete vertical-loop driving require live browser playtests.
