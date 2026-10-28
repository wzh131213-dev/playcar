# PlayCar 3D Stunt Track and Telemetry

## Track changes

- The main road now uses a three-dimensional road frame. The cross-section is generated from a side vector and a local up vector instead of assuming every road segment is horizontal.
- The main route includes a shaped launch ramp with a rising face, short crest, and descending landing section.
- The road frame applies curvature-based banking on turns.
- A 144-segment vertical loop replaces a section of the original route. Its centerline is a vertical circle with a 13-world-unit radius and a cubic exit connector back to the original route.
- Checkpoint gates were moved away from the loop so the loop does not become a checkpoint plane.

## Driving telemetry

Use the in-game **导出驾驶数据** button after driving. It downloads `playcar-driving-telemetry.csv` with one sample approximately every 0.1 seconds. Columns include elapsed time, physics tick, world position, total and vertical speed, input buttons, grounded state, grip state, collision state, and cumulative collision count. Up to 12,000 samples are kept per run.

The data is collected from the actual browser session; it is not fabricated test telemetry. Reset starts a new sample set. Export after driving the ramp, loop, and several high-speed corners, and use the CSV to tune the engine in a subsequent iteration.

## Parameter baseline and tuning caveat

The existing engine defaults remain the initial baseline: fixed 120 Hz, gravity 24, tire grip 1.65, handbrake grip 0.38, max steering angle 0.48 rad, high-speed steering scale 0.48, suspension rest length 0.09, and suspension reach 0.42 world units.

These are code defaults, not values fitted to a real driver's recorded session. No actual browser telemetry was available at implementation time, and this environment cannot simulate a real iPad Safari driving session. It would be misleading to claim the suspension, grip, or high-speed steering has already been fitted to real driving data. Collect a CSV from a real run first, then compare grounded time, vertical velocity/landing events, speed versus steering input, and collision counts before changing these values.

## Validation scope

The automated CI checks JavaScript syntax and engine regression tests. They do not replace visual/browser testing of loop entry, sustained wheel contact upside-down, ramp takeoff, landing stability, or iPad touch handling.
