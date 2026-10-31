/* PlayCar Physics Checks 1.1.0
 *
 * Self-contained, deterministic validation scenarios for the 3.0 physics stack.
 * The browser test scene (test-scene.html) and the Node test suite run exactly
 * the same code, so the on-page PASS/FAIL table is real evidence, not a mock.
 *
 * Requires the engine modules to be loaded first (rigidbody-core.js,
 * rigidbody-vehicle-engine.js, playcar-physics-engine.js, playcar-unified.js),
 * either standalone in that order or through the all-in-one bundle. Every check
 * builds its own isolated worlds, so running them never touches the visible
 * scene.
 */
(function (root) {
  'use strict';

  const API = root.PlayCarPhysicsEngine;
  const Core = root.PlayCarRigidBodyCore;
  if (!API || !API.PlayCarPhysicsEngine) {
    throw new Error('PlayCarPhysicsChecks: load playcar-physics-all-in-one.js first');
  }
  if (!Core || !Core.World) {
    throw new Error('PlayCarPhysicsChecks: rigid-body core is missing');
  }

  const Engine = API.PlayCarPhysicsEngine;
  const TriangleMesh = API.TriangleMesh;
  const V = Core.Vec3;

  const now = () => (typeof performance !== 'undefined' && typeof performance.now === 'function')
    ? performance.now() : Date.now();

  function assert(condition, message) {
    if (!condition) throw new Error(message);
  }

  function flatSurface(half = 200, friction = 1) {
    return TriangleMesh.fromTriangles([
      [{ x: -half, y: 0, z: half }, { x: -half, y: 0, z: -half }, { x: half, y: 0, z: -half }, friction],
      [{ x: -half, y: 0, z: half }, { x: half, y: 0, z: -half }, { x: half, y: 0, z: half }, friction]
    ], 8);
  }

  function makeVehicle(surface, extra = {}) {
    return new Engine({
      surfaceMesh: surface,
      groundPlaneY: extra.groundPlaneY ?? 0,
      colliderMeshes: extra.colliderMeshes || [],
      position: extra.position || { x: 0, y: 0.08, z: 0 },
      preset: 'simcade',
      config: {
        gravity: 9.81,
        maxSpeed: 60,
        engineAcceleration: 5.5,
        brakeAcceleration: 11,
        tireGrip: 1.0,
        maxSteerAngle: 0.5,
        highSpeedSteerScale: 0.44,
        suspensionRestLength: 0.3,
        suspensionReach: 0.52,
        bodyHalfExtents: { x: 0.84, y: 0.4, z: 1.7 },
        ...(extra.config || {})
      }
    });
  }

  function setSpeed(vehicle, speed) {
    vehicle.rigidBody.state.velocity = { x: 0, y: 0, z: -speed };
    vehicle.rigidBody.state.wheelOmega = [0, 1, 2, 3].map(() => speed / (vehicle.rigidBody.c.wheelRadius || 0.37));
    vehicle.rigidBody.previous.velocity = { ...vehicle.rigidBody.state.velocity };
    vehicle._legacyKey = '';
  }

  function rigidPosition(vehicle) {
    return vehicle.rigidBody.state.position;
  }

  function round(n, digits = 2) {
    return Number.isFinite(n) ? Number(n.toFixed(digits)) : String(n);
  }

  /* ------------------------------------------------------------------ */
  /* Checks                                                             */
  /* ------------------------------------------------------------------ */

  const definitions = [
    {
      id: 'core-rest',
      name: '刚体落地与睡眠',
      note: '10 kg 盒子从 3 m 落到平面，静止高度约 0.5 m 并进入睡眠',
      run() {
        const world = new Core.World({ gravity: { x: 0, y: -9.81, z: 0 } });
        world.addGroundPlane(0, { friction: 0.9, restitution: 0.02 });
        const box = world.addBody(new Core.Body({
          mass: 10,
          shape: new Core.BoxShape({ x: 0.5, y: 0.5, z: 0.5 }),
          position: { x: 0, y: 3, z: 0 },
          friction: 0.8, restitution: 0, allowSleep: true
        }));
        for (let i = 0; i < 300; i++) world.fixedStep(1 / 120);
        assert(box.position.y > 0.47 && box.position.y < 0.56, `rest height ${box.position.y.toFixed(3)}`);
        assert(box.sleeping, 'box did not fall asleep');
        return { restY: round(box.position.y, 3), sleeping: box.sleeping, contacts: world.stats.contacts };
      }
    },
    {
      id: 'core-wall',
      name: '高速防穿透',
      note: '120 m/s 盒子撞向零厚度三角网格墙，必须停在墙前',
      run() {
        const world = new Core.World({ gravity: { x: 0, y: 0, z: 0 } });
        const wall = Core.MeshShape.fromTriangles([
          [{ x: 0, y: -5, z: -5 }, { x: 0, y: 5, z: -5 }, { x: 0, y: 5, z: 5 }],
          [{ x: 0, y: -5, z: -5 }, { x: 0, y: 5, z: 5 }, { x: 0, y: -5, z: 5 }]
        ]);
        world.addStaticMesh(wall, { friction: 0.4 });
        const box = world.addBody(new Core.Body({
          mass: 20,
          shape: new Core.BoxShape({ x: 0.3, y: 0.3, z: 0.3 }),
          position: { x: -2, y: 0, z: 0 },
          velocity: { x: 120, y: 0, z: 0 },
          friction: 0.5, restitution: 0.1, allowSleep: false, gravityScale: 0, ccd: true
        }));
        for (let i = 0; i < 60; i++) world.fixedStep(1 / 120);
        assert(box.position.x < 0.05, `tunnelled to x=${box.position.x.toFixed(2)}`);
        return { x: round(box.position.x, 3), speed: round(V.length(box.linearVelocity), 2), ccdSplits: world.stats.ccdSplits };
      }
    },
    {
      id: 'core-friction',
      name: '坡度摩擦',
      note: '30° 斜面：μ=1.1 应保持，μ=0.05 应明显下滑',
      run() {
        function slopeBox(friction, seconds) {
          const world = new Core.World({ gravity: { x: 0, y: -9.81, z: 0 } });
          const normal = V.normalize({ x: Math.sin(Math.PI / 6), y: Math.cos(Math.PI / 6), z: 0 });
          world.addBody(new Core.Body({ type: 'static', shape: new Core.PlaneShape(normal, 0), friction }));
          const axis = V.normalize(V.cross({ x: 0, y: 1, z: 0 }, normal));
          const angle = Math.acos(V.dot({ x: 0, y: 1, z: 0 }, normal));
          const orientation = Core.Quaternion.axisAngle(axis, angle);
          const body = world.addBody(new Core.Body({
            mass: 10,
            shape: new Core.BoxShape({ x: 0.45, y: 0.45, z: 0.45 }),
            position: V.mul(normal, 0.45), orientation,
            friction, restitution: 0, allowSleep: false
          }));
          const start = { ...body.position };
          for (let i = 0; i < Math.round(seconds * 120); i++) world.fixedStep(1 / 120);
          return V.length(V.sub(body.position, start));
        }
        const held = slopeBox(1.1, 2);
        const slid = slopeBox(0.05, 2);
        assert(held < 0.5, `high friction slid ${held.toFixed(2)} m`);
        assert(slid > 1.0, `low friction only slid ${slid.toFixed(2)} m`);
        return { held: round(held, 2), slid: round(slid, 2) };
      }
    },
    {
      id: 'surface-materials',
      name: '逐三角形路面材质',
      note: '射线命中 μ=0.55 的湿滑三角形时应返回该材质',
      run() {
        const surface = TriangleMesh.fromTriangles([
          [{ x: -10, y: 0, z: 10 }, { x: -10, y: 0, z: -10 }, { x: 10, y: 0, z: -10 }, 0.55],
          [{ x: -10, y: 0, z: 10 }, { x: 10, y: 0, z: -10 }, { x: 10, y: 0, z: 10 }, 0.55]
        ]);
        const world = new Core.World({ gravity: { x: 0, y: -9.81, z: 0 } });
        world.addStaticMesh(new Core.MeshShape(surface.triangles.map(t => ({ a: t.a, b: t.b, c: t.c, friction: t.friction }))), { friction: 1 });
        const hit = world.raycast({ x: 0, y: 5, z: 0 }, { x: 0, y: -1, z: 0 }, 20);
        assert(hit.hit, 'ray missed the surface');
        assert(Math.abs(hit.distance - 5) < 0.01, `distance ${hit.distance}`);
        assert(Math.abs(hit.friction - 0.55) < 1e-9, `friction ${hit.friction}`);
        return { distance: round(hit.distance, 2), friction: hit.friction };
      }
    },
    {
      id: 'vehicle-ride',
      name: '悬挂静态坐姿',
      note: '车辆静止 2 s 后应四轮接触、底盘不擦地，坐姿在合理区间',
      run() {
        const vehicle = makeVehicle(flatSurface());
        for (let i = 0; i < 240; i++) vehicle.advance(1 / 120, {});
        const t = vehicle.getTelemetry();
        const pose = rigidPosition(vehicle);
        assert(t.wheelContacts === 4, `wheel contacts ${t.wheelContacts}`);
        assert(t.chassisContacts === 0, `chassis scraping (${t.chassisContacts} contacts)`);
        assert(pose.y > 0.4 && pose.y < 0.8, `ride height ${pose.y.toFixed(2)}`);
        assert(pose.y > 0.4 && Number.isFinite(pose.y), 'non-finite pose');
        return { rideHeight: round(pose.y, 3), wheelContacts: t.wheelContacts, chassisContacts: t.chassisContacts };
      }
    },
    {
      id: 'braking',
      name: '干/湿/冰制动距离',
      note: '20 m/s 全力制动：μ=1.0、0.6、0.25 的停车距离应递增',
      run() {
        function distance(mu) {
          const vehicle = makeVehicle(flatSurface(), { config: { tireGrip: mu } });
          setSpeed(vehicle, 20);
          let travelled = 0;
          const start = rigidPosition(vehicle).z;
          let stopped = false;
          for (let i = 0; i < 720; i++) {
            vehicle.advance(1 / 120, { brake: 1, throttle: 0, steer: 0 });
            travelled = start - rigidPosition(vehicle).z;
            if (Math.hypot(vehicle.rigidBody.state.velocity.x, vehicle.rigidBody.state.velocity.z) < 0.6) { stopped = true; break; }
          }
          return { distance: travelled, stopped };
        }
        const dry = distance(1.0);
        const wet = distance(0.6);
        const ice = distance(0.25);
        assert(dry.stopped, 'dry run did not stop');
        assert(dry.distance < 30, `dry distance ${dry.distance.toFixed(1)} m`);
        assert(wet.distance > dry.distance * 1.25, `wet ${wet.distance.toFixed(1)} vs dry ${dry.distance.toFixed(1)}`);
        assert(ice.distance > wet.distance * 1.2, `ice ${ice.distance.toFixed(1)} vs wet ${wet.distance.toFixed(1)}`);
        return { dry: round(dry.distance, 1), wet: round(wet.distance, 1), ice: round(ice.distance, 1) };
      }
    },
    {
      id: 'weather-wind',
      name: '天气：侧风与积雪',
      note: '15 m/s 侧风产生下风侧推力与风向标偏航；积雪把附着系数降到 0.68',
      run() {
        function drive(wind) {
          const vehicle = makeVehicle(flatSurface());
          if (wind) vehicle.setWeather({ wind: { x: wind, z: 0 }, wetness: 0 });
          setSpeed(vehicle, 20);
          for (let i = 0; i < 240; i++) vehicle.advance(1 / 120, { throttle: 0.2, steer: 0 });
          const t = vehicle.rigidBody.telemetry;
          return { x: rigidPosition(vehicle).x, sideForce: t.sideForce || 0 };
        }
        const calm = drive(0);
        const windy = drive(15);
        assert(Math.abs(calm.sideForce) < 1, `calm side force ${calm.sideForce}`);
        assert(windy.sideForce > 150, `crosswind side force ${windy.sideForce.toFixed(0)} N`);
        assert(Math.abs(windy.x - calm.x) > 0.05, `lateral response ${(windy.x - calm.x).toFixed(3)} m`);
        const dry = makeVehicle(flatSurface());
        dry.setWeather({ wetness: 0, snow: 0 });
        const snowy = makeVehicle(flatSurface());
        snowy.setWeather({ wetness: 0, snow: 1 });
        const muDry = dry.rigidBody.c.tire.mu;
        const muSnow = snowy.rigidBody.c.tire.mu;
        assert(muSnow < muDry * 0.75, `snow μ ${muSnow.toFixed(2)} vs dry ${muDry.toFixed(2)}`);
        return { sideForce: round(windy.sideForce, 0), drift: round(windy.x - calm.x, 3), muDry: round(muDry, 3), muSnow: round(muSnow, 3) };
      }
    },
    {
      id: 'ramp-jump',
      name: '跳台腾空与落地',
      note: '18 m/s 冲上跳台后应离地、保持直立并重新获得四轮支撑',
      run() {
        const tris = [
          [{ x: -12, y: 0, z: 20 }, { x: -12, y: 0, z: -60 }, { x: 12, y: 0, z: -60 }, 1],
          [{ x: -12, y: 0, z: 20 }, { x: 12, y: 0, z: -60 }, { x: 12, y: 0, z: 20 }, 1]
        ];
        // Ramp: ground at z=-6, lip 1.6 m high at z=-12, deck ends at z=-14.
        const zStart = -6, zLip = -12, zDeck = -14;
        tris.push(
          [{ x: -6, y: 0, z: zStart }, { x: 6, y: 0, z: zStart }, { x: 6, y: 1.6, z: zLip }, 1],
          [{ x: -6, y: 0, z: zStart }, { x: 6, y: 1.6, z: zLip }, { x: -6, y: 1.6, z: zLip }, 1],
          [{ x: -6, y: 1.6, z: zLip }, { x: 6, y: 1.6, z: zLip }, { x: 6, y: 1.6, z: zDeck }, 1],
          [{ x: -6, y: 1.6, z: zLip }, { x: 6, y: 1.6, z: zDeck }, { x: -6, y: 1.6, z: zDeck }, 1]
        );
        const surface = TriangleMesh.fromTriangles(tris, 8);
        const vehicle = makeVehicle(surface, { position: { x: 0, y: 0.08, z: 4 }, groundPlaneY: -5 });
        setSpeed(vehicle, 18);
        let airborne = 0, maxY = 0, landed = false;
        for (let i = 0; i < 480; i++) {
          vehicle.advance(1 / 120, { throttle: 0.6, steer: 0 });
          const p = rigidPosition(vehicle);
          maxY = Math.max(maxY, p.y);
          const t = vehicle.getTelemetry();
          if (t.wheelContacts === 0) airborne++;
          if (airborne > 6 && t.wheelContacts >= 2) landed = true;
          if (landed) break;
        }
        const up = Core.Quaternion.rotate(vehicle.rigidBody.state.orientation, { x: 0, y: 1, z: 0 });
        assert(airborne > 4, `no airborne phase (${airborne} frames)`);
        assert(landed, 'car never landed on its wheels');
        assert(up.y > 0.5, `car landed upside down (up.y ${up.y.toFixed(2)})`);
        return { airborneFrames: airborne, maxY: round(maxY, 2), landed, upY: round(up.y, 2) };
      }
    },
    {
      id: 'prop-impact',
      name: '动态道具碰撞',
      note: '15 m/s 撞飞木箱，木箱应位移且保持有限状态',
      run() {
        const vehicle = makeVehicle(flatSurface());
        const prop = vehicle.addProp({
          shape: 'box', halfExtents: { x: 0.3, y: 0.3, z: 0.3 }, mass: 8,
          position: { x: 0, y: 0.35, z: -8 }, friction: 0.7, restitution: 0.1
        });
        const start = { x: prop.position.x, z: prop.position.z };
        setSpeed(vehicle, 15);
        for (let i = 0; i < 240; i++) vehicle.advance(1 / 120, { throttle: 0.4, steer: 0 });
        const moved = Math.hypot(prop.position.x - start.x, prop.position.z - start.z);
        assert(Number.isFinite(prop.position.x) && Number.isFinite(prop.position.y), 'prop position is not finite');
        assert(moved > 0.5, `prop only moved ${moved.toFixed(2)} m`);
        assert(prop.position.y > -1 && prop.position.y < 10, `prop y ${prop.position.y}`);
        return { moved: round(moved, 2), y: round(prop.position.y, 2) };
      }
    },
    {
      id: 'drivetrain',
      name: '差速器与驱动布局',
      note: 'AWD 四轮驱动，切换 RWD 后仅后轮获得驱动',
      run() {
        const vehicle = makeVehicle(flatSurface());
        vehicle.setDifferential('lsd', 'awd', 3.4);
        vehicle.advance(1 / 120, { throttle: 0.5 });
        const awd = vehicle.getTelemetry().drivenWheels.slice().sort();
        vehicle.setDifferential('lsd', 'rwd', 2.2);
        for (let i = 0; i < 30; i++) vehicle.advance(1 / 120, { throttle: 0.5 });
        const rwd = vehicle.getTelemetry().drivenWheels.slice().sort();
        assert(awd.join(',') === '0,1,2,3', `AWD wheels ${awd}`);
        assert(rwd.join(',') === '2,3', `RWD wheels ${rwd}`);
        return { awd: awd.join('/'), rwd: rwd.join('/') };
      }
    },
    {
      id: 'determinism',
      name: '确定性重复仿真',
      note: '两个相同刚体世界运行 300 步，位置必须完全一致',
      run() {
        function run() {
          const world = new Core.World({ gravity: { x: 0, y: -9.81, z: 0 } });
          world.addGroundPlane(0, { friction: 0.8 });
          for (let i = 0; i < 8; i++) {
            world.addBody(new Core.Body({
              mass: 5,
              shape: new Core.BoxShape({ x: 0.3, y: 0.3, z: 0.3 }),
              position: { x: (i % 4) * 0.9 - 1.35, y: 1 + Math.floor(i / 4) * 0.8, z: (i % 2) * 0.7 - 0.35 },
              friction: 0.7, restitution: 0.05
            }));
          }
          for (let i = 0; i < 300; i++) world.fixedStep(1 / 120);
          return world.bodies.map(b => [b.position.x, b.position.y, b.position.z]);
        }
        const a = run(), b = run();
        let maxDelta = 0;
        for (let i = 0; i < a.length; i++) {
          for (let k = 0; k < 3; k++) maxDelta = Math.max(maxDelta, Math.abs(a[i][k] - b[i][k]));
        }
        assert(maxDelta === 0, `runs diverged by ${maxDelta}`);
        return { bodies: a.length, maxDelta };
      }
    }
  ];

  function runAll() {
    const started = now();
    const results = definitions.map(def => {
      const t0 = now();
      try {
        const detail = def.run();
        return { id: def.id, name: def.name, note: def.note, ok: true, detail, ms: Math.round((now() - t0) * 10) / 10 };
      } catch (error) {
        return {
          id: def.id, name: def.name, note: def.note, ok: false,
          detail: error && error.message ? error.message : String(error),
          ms: Math.round((now() - t0) * 10) / 10
        };
      }
    });
    const passed = results.filter(r => r.ok).length;
    return {
      version: '1.0.0',
      engine: API.version,
      startedAt: new Date().toISOString(),
      ms: Math.round((now() - started) * 10) / 10,
      total: results.length,
      passed,
      failed: results.length - passed,
      ok: passed === results.length,
      results
    };
  }

  root.PlayCarPhysicsChecks = {
    version: '1.0.0',
    engine: API.version,
    runAll,
    ids: definitions.map(d => d.id)
  };
  if (typeof module !== 'undefined' && module.exports) module.exports = root.PlayCarPhysicsChecks;
})(typeof globalThis !== 'undefined' ? globalThis : this);
