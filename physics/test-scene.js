import * as THREE from '../vendor/three.module.min.js';

/* PlayCar 3D Physics Proving Ground
 *
 * A dedicated test scene for the 3.0 rigid-body stack: friction lanes, a
 * 15-degree hill, a washboard, a launch ramp, a solid wall and a prop yard.
 * The same page runs the deterministic check suite in physics/physics-checks.js
 * (PASS/FAIL table) while this module provides the drivable 3D scene.
 */

const EngineAPI = window.PlayCarPhysicsEngine;
const status = document.getElementById('scene-status');
const host = document.getElementById('scene3d');
if (!EngineAPI || !host) {
  if (status) status.textContent = '测试场初始化失败：缺少物理引擎或容器';
  throw new Error('PlayCar test scene prerequisites missing');
}

const { PlayCarPhysicsEngine, TriangleMesh, Vector3: V, Quaternion: Q } = EngineAPI;

host.style.position = 'relative';
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0c1a2a);
scene.fog = new THREE.Fog(0x0c1a2a, 120, 320);
const camera = new THREE.PerspectiveCamera(58, 1, 0.1, 600);
camera.position.set(0, 9, 105);

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
renderer.setPixelRatio(Math.min(devicePixelRatio || 1, 1.75));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;
host.appendChild(renderer.domElement);
renderer.domElement.setAttribute('aria-label', 'PlayCar 物理测试场三维场景');
renderer.domElement.style.touchAction = 'none';

const hemiLight = new THREE.HemisphereLight(0xcfe6ff, 0x1b2634, 1.9);
scene.add(hemiLight);
const sun = new THREE.DirectionalLight(0xffffff, 2.7);
sun.position.set(-45, 75, 40);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
sun.shadow.camera.left = -120; sun.shadow.camera.right = 120;
sun.shadow.camera.top = 140; sun.shadow.camera.bottom = -140;
sun.shadow.camera.far = 320;
scene.add(sun);
const fill = new THREE.DirectionalLight(0x6fb6ff, 0.55);
fill.position.set(50, 30, -70);
scene.add(fill);

const mats = {
  pad: new THREE.MeshStandardMaterial({ color: 0x263440, roughness: 0.96 }),
  runway: new THREE.MeshStandardMaterial({ color: 0x34424e, roughness: 0.9 }),
  ice: new THREE.MeshStandardMaterial({ color: 0x9fd8ef, roughness: 0.22, metalness: 0.05 }),
  wet: new THREE.MeshStandardMaterial({ color: 0x2f6f8f, roughness: 0.35, metalness: 0.1 }),
  dry: new THREE.MeshStandardMaterial({ color: 0x6d7c86, roughness: 0.82 }),
  line: new THREE.MeshStandardMaterial({ color: 0xf2f6fa, roughness: 0.7 }),
  wall: new THREE.MeshStandardMaterial({ color: 0xc8d6df, roughness: 0.72 }),
  wallTop: new THREE.MeshStandardMaterial({ color: 0xff7a45, roughness: 0.45 }),
  ramp: new THREE.MeshStandardMaterial({ color: 0x4e6a86, roughness: 0.6, metalness: 0.08, side: THREE.DoubleSide }),
  hill: new THREE.MeshStandardMaterial({ color: 0x2d4b4f, roughness: 0.9, side: THREE.DoubleSide }),
  curb: new THREE.MeshStandardMaterial({ color: 0xd8c25a, roughness: 0.65 }),
  cone: new THREE.MeshStandardMaterial({ color: 0xff7b2e, roughness: 0.55 }),
  crate: new THREE.MeshStandardMaterial({ color: 0xb08a53, roughness: 0.8 }),
  orange: new THREE.MeshStandardMaterial({ color: 0xff713d, roughness: 0.34, metalness: 0.25 }),
  glass: new THREE.MeshStandardMaterial({ color: 0x173a4f, roughness: 0.16, metalness: 0.4 }),
  tire: new THREE.MeshStandardMaterial({ color: 0x111820, roughness: 0.92 }),
  rim: new THREE.MeshStandardMaterial({ color: 0xa8bdcc, metalness: 0.85, roughness: 0.22 }),
  post: new THREE.MeshStandardMaterial({ color: 0x8fa7b8, roughness: 0.8 })
};

function addTriangle(list, a, b, c, friction) {
  list.push(friction === undefined ? [a, b, c] : [a, b, c, friction]);
}
function addQuad(list, a, b, c, d, friction) {
  addTriangle(list, a, b, c, friction);
  addTriangle(list, a, c, d, friction);
}
function meshFromTriangles(tris, material) {
  const positions = [];
  for (const t of tris) for (const p of t) positions.push(p.x, p.y, p.z);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geo.computeVertexNormals();
  const mesh = new THREE.Mesh(geo, material);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  scene.add(mesh);
  return mesh;
}
function flatQuad(list, x0, x1, z0, z1, y, friction) {
  addQuad(list,
    { x: x0, y, z: z0 }, { x: x0, y, z: z1 },
    { x: x1, y, z: z1 }, { x: x1, y, z: z0 }, friction);
}
function boxMesh(w, h, d, x, y, z, material) {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), material);
  mesh.position.set(x, y, z);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  scene.add(mesh);
  return mesh;
}

/* ------------------------------------------------------------------ */
/* Proving ground                                                     */
/* ------------------------------------------------------------------ */

const surfaceTriangles = [];
const colliderTriangles = [];
const RUNWAY_HALF = 13;
const RUNWAY_Z0 = 96;
const RUNWAY_Z1 = -136;

// Base pad (safety surface everywhere).
flatQuad(surfaceTriangles, -180, 180, 180, -180, 0, 1.0);
const pad = new THREE.Mesh(new THREE.PlaneGeometry(360, 360), mats.pad);
pad.rotation.x = -Math.PI / 2;
pad.receiveShadow = true;
scene.add(pad);

// Asphalt runway from the start line to the wall.
flatQuad(surfaceTriangles, -RUNWAY_HALF, RUNWAY_HALF, RUNWAY_Z0, RUNWAY_Z1, 0.002, 0.98);
const runway = new THREE.Mesh(new THREE.PlaneGeometry(RUNWAY_HALF * 2, RUNWAY_Z0 - RUNWAY_Z1), mats.runway);
runway.rotation.x = -Math.PI / 2;
runway.position.set(0, 0.004, (RUNWAY_Z0 + RUNWAY_Z1) / 2);
runway.receiveShadow = true;
scene.add(runway);

// Friction lanes (x: -9..-3 ice, -3..3 wet, 3..9 dry) at z 26..6.
const LANES = [
  { name: 'ICE', x0: -9, x1: -3, friction: 0.18, color: 0x9fd8ef },
  { name: 'WET', x0: -3, x1: 3, friction: 0.6, color: 0x2f6f8f },
  { name: 'DRY', x0: 3, x1: 9, friction: 1.0, color: 0x6d7c86 }
];
for (const lane of LANES) {
  flatQuad(surfaceTriangles, lane.x0, lane.x1, 26, 6, 0.006, lane.friction);
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(lane.x1 - lane.x0, 20), new THREE.MeshStandardMaterial({
    color: lane.color, roughness: lane.friction < 0.3 ? 0.2 : 0.65, metalness: lane.friction < 0.3 ? 0.05 : 0
  }));
  mesh.rotation.x = -Math.PI / 2;
  mesh.position.set((lane.x0 + lane.x1) / 2, 0.008, 16);
  mesh.receiveShadow = true;
  scene.add(mesh);
}
// Lane separator lines.
for (const x of [-9, -3, 3, 9]) boxMesh(0.12, 0.02, 20, x, 0.012, 16, mats.line);

// Start line and runway edge lines.
for (let i = 0; i < 8; i++) {
  const stripe = boxMesh(1.5, 0.02, 0.8, -10.5 + i * 3, 0.012, 88, i % 2 ? mats.line : mats.wallTop);
  stripe.receiveShadow = false;
}
for (const x of [-RUNWAY_HALF + 0.4, RUNWAY_HALF - 0.4]) {
  boxMesh(0.18, 0.02, RUNWAY_Z0 - RUNWAY_Z1, x, 0.012, (RUNWAY_Z0 + RUNWAY_Z1) / 2, mats.line);
}

// 15-degree hill: up 10 m, 4 m crest, down 10 m.
{
  const h = 3.2, half = 9, z0 = -18, z1 = z0 - 10, z2 = z1 - 4, z3 = z2 - 10;
  const lift = (z, y) => ({ x: 0, y, z });
  const L = p => ({ x: p.x - half, y: p.y, z: p.z });
  const R = p => ({ x: p.x + half, y: p.y, z: p.z });
  const a = lift(z0, 0), b = lift(z1, h), c = lift(z2, h), d = lift(z3, 0);
  const quads = [[a, b], [b, c], [c, d]];
  for (const [p0, p1] of quads) {
    addQuad(surfaceTriangles, L(p0), L(p1), R(p1), R(p0), 1);
    addQuad(colliderTriangles, L(p0), L(p1), R(p1), R(p0), 1);
  }
  // Side skirts close the hill volume for the chassis and for the visuals.
  const hillTris = [];
  for (const [p0, p1] of quads) addQuad(hillTris, L(p0), L(p1), R(p1), R(p0));
  for (const side of [-1, 1]) {
    const s = p => ({ x: p.x + side * half, y: p.y, z: p.z });
    addQuad(colliderTriangles, s(a), s(b), { x: s(b).x, y: 0, z: s(b).z }, { x: s(a).x, y: 0, z: s(a).z }, 1);
    addQuad(colliderTriangles, s(b), s(c), { x: s(c).x, y: 0, z: s(c).z }, { x: s(b).x, y: 0, z: s(b).z }, 1);
    addQuad(colliderTriangles, s(c), s(d), { x: s(d).x, y: 0, z: s(d).z }, { x: s(c).x, y: 0, z: s(c).z }, 1);
    addQuad(hillTris, s(a), s(b), { x: s(b).x, y: 0, z: s(b).z }, { x: s(a).x, y: 0, z: s(a).z });
    addQuad(hillTris, s(b), s(c), { x: s(c).x, y: 0, z: s(c).z }, { x: s(b).x, y: 0, z: s(b).z });
    addQuad(hillTris, s(c), s(d), { x: s(d).x, y: 0, z: s(d).z }, { x: s(c).x, y: 0, z: s(c).z });
  }
  meshFromTriangles(hillTris, mats.hill);
}

// Washboard: six 9 cm cleats across the runway.
for (let i = 0; i < 6; i++) {
  const z = -45 - i * 1.4;
  const tri = [
    { x: -RUNWAY_HALF, y: 0, z: z + 0.22 }, { x: -RUNWAY_HALF, y: 0.09, z: z },
    { x: RUNWAY_HALF, y: 0.09, z: z }, { x: RUNWAY_HALF, y: 0, z: z + 0.22 }
  ];
  addQuad(surfaceTriangles, tri[0], tri[1], tri[2], tri[3], 1);
  addQuad(colliderTriangles, tri[0], tri[1], tri[2], tri[3], 1);
  boxMesh(RUNWAY_HALF * 2, 0.09, 0.22, 0, 0.045, z + 0.11, mats.curb);
}

// Solid wall at the end of the runway: 26 x 2.4 x 0.6 m. It sits past the
// landing zone of the hill jump (~35 m of hang time at full speed).
{
  const z = -126, halfW = 13, thick = 0.3, h = 2.4;
  const x0 = -halfW, x1 = halfW, zf = z + thick, zb = z - thick;
  const nf = [{ x: x0, y: 0, z: zf }, { x: x0, y: h, z: zf }, { x: x1, y: h, z: zf }, { x: x1, y: 0, z: zf }];
  const nb = [{ x: x0, y: 0, z: zb }, { x: x1, y: 0, z: zb }, { x: x1, y: h, z: zb }, { x: x0, y: h, z: zb }];
  addQuad(colliderTriangles, nf[0], nf[1], nf[2], nf[3], 1);
  addQuad(colliderTriangles, nb[0], nb[1], nb[2], nb[3], 1);
  addQuad(colliderTriangles, { x: x0, y: h, z: zf }, { x: x0, y: h, z: zb }, { x: x1, y: h, z: zb }, { x: x1, y: h, z: zf }, 1);
  addQuad(colliderTriangles, { x: x0, y: 0, z: zf }, { x: x0, y: 0, z: zb }, { x: x0, y: h, z: zb }, { x: x0, y: h, z: zf }, 1);
  addQuad(colliderTriangles, { x: x1, y: 0, z: zf }, { x: x1, y: 0, z: zb }, { x: x1, y: h, z: zb }, { x: x1, y: h, z: zf }, 1);
  const wall = boxMesh(halfW * 2, h, thick * 2, 0, h / 2, z, mats.wall);
  wall.receiveShadow = true;
  boxMesh(halfW * 2 + 0.3, 0.12, thick * 2 + 0.3, 0, h + 0.06, z, mats.wallTop);
}

// Launch ramp on the right: 8 m long, 2.2 m lip, 7 m wide.
{
  const x0 = 14, x1 = 21, z0 = 36, z1 = 28, h = 2.2;
  const a = { x: x0, y: 0, z: z0 }, b = { x: x0, y: h, z: z1 };
  const c = { x: x1, y: 0, z: z0 }, d = { x: x1, y: h, z: z1 };
  addQuad(surfaceTriangles, a, b, d, c, 1);
  addQuad(colliderTriangles, a, b, d, c, 1);
  for (const [p, q] of [[a, b], [c, d]]) {
    addQuad(colliderTriangles, p, q, { x: q.x, y: 0, z: q.z }, { x: p.x, y: 0, z: p.z }, 1);
  }
  meshFromTriangles([[a, b, d], [a, d, c]], mats.ramp);
  // Approach lip.
  const lip = new THREE.Mesh(new THREE.BoxGeometry(x1 - x0, 0.12, 0.5), mats.wallTop);
  lip.position.set((x0 + x1) / 2, h, z1 - 0.2);
  scene.add(lip);
}

// Distance markers along the runway.
for (const z of [80, 60, 40, 20, 0, -20, -40, -60, -90, -110]) {
  for (const x of [-RUNWAY_HALF, RUNWAY_HALF]) {
    const post = new THREE.Mesh(new THREE.BoxGeometry(0.18, 1.4, 0.18), mats.post);
    post.position.set(x, 0.7, z);
    post.castShadow = true;
    scene.add(post);
  }
}

const surface = TriangleMesh.fromTriangles(surfaceTriangles, 8);
const walls = TriangleMesh.fromTriangles(colliderTriangles, 8);

/* ------------------------------------------------------------------ */
/* Vehicle                                                            */
/* ------------------------------------------------------------------ */

const vehicle = new PlayCarPhysicsEngine({
  surfaceMesh: surface,
  colliderMeshes: [walls],
  groundPlaneY: 0,
  position: { x: 0, y: 0.08, z: 88 },
  preset: 'simcade',
  config: {
    gravity: 9.81,
    maxSpeed: 62,
    engineAcceleration: 6.4,
    brakeAcceleration: 11,
    tireGrip: 1.05,
    handbrakeGrip: 0.7,
    maxSteerAngle: 0.5,
    highSpeedSteerScale: 0.44,
    suspensionRestLength: 0.3,
    suspensionReach: 0.52,
    bodyHalfExtents: { x: 0.84, y: 0.4, z: 1.7 },
    bodyCenterY: 0.2
  }
});
window.PlayCarTestVehicle = vehicle;
const rigid = vehicle.rigidBody;
rigid.c.aids.esc = true;
rigid.c.suspension.progressive = 0.45;
rigid.c.suspension.dampingBump = 4200;
rigid.c.suspension.dampingRebound = 6200;
rigid.c.tire.relaxationLength = 0.32;

const carRoot = new THREE.Group();
scene.add(carRoot);
const chassis = new THREE.Mesh(new THREE.BoxGeometry(1.72, 0.58, 3.4), mats.orange);
chassis.position.y = 0.02;
chassis.castShadow = true;
chassis.receiveShadow = true;
carRoot.add(chassis);
const cabin = new THREE.Mesh(new THREE.BoxGeometry(1.32, 0.56, 1.4), mats.glass);
cabin.position.set(0, 0.52, 0.15);
cabin.castShadow = true;
carRoot.add(cabin);
const nose = new THREE.Mesh(new THREE.BoxGeometry(1.5, 0.22, 0.72), mats.orange);
nose.position.set(0, -0.1, -1.85);
nose.castShadow = true;
carRoot.add(nose);
const spoiler = new THREE.Mesh(new THREE.BoxGeometry(1.7, 0.08, 0.4), mats.orange);
spoiler.position.set(0, 0.42, 1.68);
carRoot.add(spoiler);
for (const x of [-0.5, 0.5]) {
  const lamp = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.12, 0.08),
    new THREE.MeshStandardMaterial({ color: 0xfff4c4, emissive: 0xffd98b, emissiveIntensity: 1.5 }));
  lamp.position.set(x, -0.03, -1.72);
  carRoot.add(lamp);
}

const wheels = [];
const WHEEL_RADIUS = rigid.c.wheelRadius;
for (const z of [-1.35, 1.35]) {
  for (const x of [-0.78, 0.78]) {
    const pivot = new THREE.Group();
    pivot.position.set(x, -0.3, z);
    carRoot.add(pivot);
    const spinner = new THREE.Group();
    pivot.add(spinner);
    const tire = new THREE.Mesh(new THREE.CylinderGeometry(WHEEL_RADIUS, WHEEL_RADIUS, 0.27, 22), mats.tire);
    tire.rotation.z = Math.PI / 2;
    tire.castShadow = true;
    spinner.add(tire);
    const hub = new THREE.Mesh(new THREE.CylinderGeometry(WHEEL_RADIUS * 0.5, WHEEL_RADIUS * 0.5, 0.29, 16), mats.rim);
    hub.rotation.z = Math.PI / 2;
    spinner.add(hub);
    wheels.push({ pivot, spinner, z });
  }
}

/* Props: cones and crates the car can hit. */
const props = [];
function addPropVisual(kind, body) {
  const group = new THREE.Group();
  if (kind === 'cone') {
    const cone = new THREE.Mesh(new THREE.ConeGeometry(0.22, 0.7, 14), mats.cone);
    cone.position.y = 0.02;
    cone.castShadow = true;
    group.add(cone);
  } else {
    const crate = new THREE.Mesh(new THREE.BoxGeometry(0.62, 0.62, 0.62), mats.crate);
    crate.castShadow = true;
    crate.receiveShadow = true;
    group.add(crate);
  }
  scene.add(group);
  props.push({ group, body });
}
for (let i = 0; i < 6; i++) {
  const body = vehicle.addProp({
    shape: 'box', halfExtents: { x: 0.2, y: 0.35, z: 0.2 }, mass: 7,
    position: { x: -16 + i * 1.1, y: 0.4, z: 14 - i * 3 }, friction: 0.75, restitution: 0.1
  });
  addPropVisual('cone', body);
}
for (let i = 0; i < 4; i++) {
  const body = vehicle.addProp({
    shape: 'box', halfExtents: { x: 0.31, y: 0.31, z: 0.31 }, mass: 16,
    position: { x: 15 + (i % 2) * 0.7, y: 0.36, z: 12 - Math.floor(i / 2) * 0.7 }, friction: 0.8, restitution: 0.08
  });
  addPropVisual('crate', body);
}

/* ------------------------------------------------------------------ */
/* Weather                                                            */
/* ------------------------------------------------------------------ */

const weatherAPI = typeof PlayCarWeather !== 'undefined' ? PlayCarWeather
  : (typeof window !== 'undefined' && window.PlayCarWeather ? window.PlayCarWeather : null);
const weatherSystem = weatherAPI ? new weatherAPI.WeatherSystem({ preset: 'clear' }) : null;
const createWeatherVisuals = typeof createPlayCarWeatherVisuals === 'function' ? createPlayCarWeatherVisuals
  : (typeof window !== 'undefined' && window.createPlayCarWeatherVisuals ? window.createPlayCarWeatherVisuals : null);
const weatherVisuals = (() => {
  if (!createWeatherVisuals) return null;
  try {
    return createWeatherVisuals(THREE, scene, {
      camera, renderer, sun, hemi: hemiLight,
      materials: [mats.pad, mats.runway, mats.dry, mats.wet, mats.ice, mats.line]
    });
  } catch (error) {
    console.warn('PlayCar weather visuals unavailable:', error && error.message);
    return null;
  }
})();
const WEATHER_ORDER = ['clear', 'overcast', 'rain', 'storm', 'fog', 'snow', 'night'];
let weatherIndex = 0;
let weatherApplyTimer = 0;

function setWeatherPreset(name, options = {}) {
  if (!weatherSystem) return null;
  let state;
  try { state = weatherSystem.setPreset(name, options); }
  catch (error) { return null; }
  weatherIndex = Math.max(0, WEATHER_ORDER.indexOf(name));
  weatherSystem.applyTo(vehicle);
  if (!options.silent && typeof window !== 'undefined' && typeof window.dispatchEvent === 'function') {
    try {
      window.dispatchEvent(new CustomEvent('playcar:weather', { detail: weatherSystem.getState() }));
    } catch (_) { /* CustomEvent may be unavailable in headless harnesses */ }
  }
  return state;
}
function cycleWeather(delta) {
  const next = (weatherIndex + delta + WEATHER_ORDER.length) % WEATHER_ORDER.length;
  return setWeatherPreset(WEATHER_ORDER[next]);
}
if (weatherSystem) weatherSystem.applyTo(vehicle);

/* HUD overlay */
const overlay = document.createElement('div');
overlay.style.cssText = 'position:absolute;left:10px;bottom:10px;padding:8px 11px;border-radius:10px;background:rgba(6,14,24,.72);color:#e8f5ff;font:12px/1.55 ui-monospace,monospace;pointer-events:none;backdrop-filter:blur(4px)';
overlay.innerHTML = '<b id="hud-speed">0 km/h</b> · <span id="hud-gear">N</span><br><span id="hud-surface">路面 μ —</span> · <span id="hud-contacts">0 轮接触</span><br><span id="hud-weather">天气 晴朗 · 0% 湿</span>';
host.appendChild(overlay);

/* ------------------------------------------------------------------ */
/* Input                                                              */
/* ------------------------------------------------------------------ */

const input = { gas: false, brake: false, left: false, right: false, handbrake: false };
function release() { for (const key of Object.keys(input)) input[key] = false; }
function bindHold(id, key) {
  const el = document.getElementById(id);
  if (!el) return;
  el.addEventListener('pointerdown', event => {
    event.preventDefault();
    input[key] = true;
    try { el.setPointerCapture(event.pointerId); } catch (_) { /* ignore */ }
  });
  ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(type =>
    el.addEventListener(type, event => { if (event) event.preventDefault(); input[key] = false; }));
}
[['scene-gas', 'gas'], ['scene-brake', 'brake'], ['scene-left', 'left'], ['scene-right', 'right'], ['scene-handbrake', 'handbrake']]
  .forEach(pair => bindHold(...pair));

window.addEventListener('keydown', event => {
  if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', ' '].includes(event.key)) event.preventDefault();
  const key = event.key.toLowerCase();
  if (key === 'arrowup' || key === 'w') input.gas = true;
  if (key === 'arrowdown' || key === 's') input.brake = true;
  if (key === 'arrowleft' || key === 'a') input.left = true;
  if (key === 'arrowright' || key === 'd') input.right = true;
  if (event.key === ' ') input.handbrake = true;
  if (key === 'r') resetVehicle();
  if (event.key === '[') cycleWeather(-1);
  if (event.key === ']') cycleWeather(1);
}, { passive: false });
window.addEventListener('keyup', event => {
  const key = event.key.toLowerCase();
  if (key === 'arrowup' || key === 'w') input.gas = false;
  if (key === 'arrowdown' || key === 's') input.brake = false;
  if (key === 'arrowleft' || key === 'a') input.left = false;
  if (key === 'arrowright' || key === 'd') input.right = false;
  if (event.key === ' ') input.handbrake = false;
});
window.addEventListener('blur', release);

function resetVehicle() {
  vehicle.reset({ x: 0, y: 0.08, z: 88 });
  rigid._layout = null;
  release();
}
document.getElementById('scene-reset')?.addEventListener('click', resetVehicle);
document.getElementById('reset-car')?.addEventListener('click', resetVehicle);

let follow = true;
document.getElementById('scene-follow')?.addEventListener('click', event => {
  follow = !follow;
  event.currentTarget.textContent = follow ? '跟随视角：开' : '跟随视角：关';
});

/* Stable runtime facade for the lab UI and the headless smoke test. */
const runtime = {
  engine: vehicle,
  input,
  setInput(patch = {}) { Object.assign(input, patch); return { ...input }; },
  setWeather(value = {}) { return vehicle.setWeather(value); },
  setConfig(patch = {}) { return vehicle.setConfig(patch); },
  setRigidConfig(patch = {}) { return vehicle.setRigidConfig(patch); },
  setWeatherPreset(name, options) { return setWeatherPreset(name, options); },
  cycleWeather(delta = 1) { return cycleWeather(delta); },
  weather() { return weatherSystem ? weatherSystem.getState() : null; },
  weatherVisuals,
  listWeatherPresets() { return weatherSystem ? weatherSystem.listPresets() : []; },
  reset() { resetVehicle(); return vehicle.getSnapshot(); },
  snapshot() { return vehicle.getSnapshot(); },
  surfaceFriction() {
    const contacts = rigid.lastContacts || [];
    return contacts.length ? Math.min(...contacts.map(c => c.friction)) : null;
  }
};
window.PlayCarRuntime = runtime;
window.dispatchEvent(new CustomEvent('playcar:ready', { detail: { version: EngineAPI.version, scene: 'proving-ground' } }));

/* ------------------------------------------------------------------ */
/* Render loop                                                        */
/* ------------------------------------------------------------------ */

const clock = new THREE.Clock();
let frame = 0;
let lastStatus = 0;
const cameraPosition = new THREE.Vector3(0, 9, 105);
const tmpForward = new THREE.Vector3();
const hudSurface = document.getElementById('hud-surface');
const hudContacts = document.getElementById('hud-contacts');
const hudWeather = document.getElementById('hud-weather');

function resize() {
  const w = Math.max(1, host.clientWidth);
  const h = Math.max(280, host.clientHeight);
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
const observer = new ResizeObserver(resize);
observer.observe(host);
resize();

function render() {
  requestAnimationFrame(render);
  const dt = Math.min(clock.getDelta(), 0.05);

  const level = v => typeof v === 'number' ? V.clamp(v, 0, 1) : (v ? 1 : 0);
  vehicle.advance(dt, {
    throttle: level(input.gas),
    brake: level(input.brake),
    steer: (input.right ? 1 : 0) - (input.left ? 1 : 0),
    handbrake: level(input.handbrake)
  });

  if (weatherSystem) {
    const weather = weatherSystem.update(dt);
    if (weatherVisuals) weatherVisuals.update(dt, weather);
    weatherApplyTimer += dt;
    if (weatherApplyTimer >= 0.1) {
      weatherApplyTimer = 0;
      weatherSystem.applyTo(vehicle);
    }
  }

  const pose = rigid.interpolatedState();
  const wheelState = vehicle.getWheelState();
  carRoot.position.set(pose.position.x, pose.position.y, pose.position.z);
  carRoot.quaternion.set(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

  const suspension = rigid.telemetry.suspension || [];
  for (let i = 0; i < wheels.length; i++) {
    const w = wheels[i];
    const compression = suspension[i] ? suspension[i].compression : 0;
    const length = Math.max(rigid.c.suspension.min, rigid.c.suspension.rest - compression);
    w.pivot.position.y = -length;
    w.pivot.rotation.y = w.z < 0 ? -wheelState.steer : 0;
    w.spinner.rotation.x -= wheelState.omega[i] * dt;
  }

  for (const prop of props) {
    prop.group.position.set(prop.body.position.x, prop.body.position.y, prop.body.position.z);
    prop.group.quaternion.set(prop.body.orientation.x, prop.body.orientation.y, prop.body.orientation.z, prop.body.orientation.w);
  }

  if (follow) {
    const back = new THREE.Vector3(0, 3.4, 8.6).applyQuaternion(carRoot.quaternion);
    const desired = new THREE.Vector3(pose.position.x, pose.position.y, pose.position.z).add(back);
    cameraPosition.lerp(desired, Math.min(1, dt * 6));
    camera.position.copy(cameraPosition);
    const forward = Q.rotate(pose.orientation, { x: 0, y: 0, z: -1 });
    tmpForward.set(forward.x, forward.y, forward.z);
    camera.lookAt(
      pose.position.x + tmpForward.x * 5,
      pose.position.y + 1.0 + tmpForward.y * 2.5,
      pose.position.z + tmpForward.z * 5
    );
  } else {
    camera.lookAt(pose.position.x, pose.position.y + 0.6, pose.position.z);
  }
  camera.updateProjectionMatrix();

  renderer.render(scene, camera);

  if (++frame % 6 === 0) {
    const t = vehicle.getTelemetry();
    const speed = Math.hypot(rigid.state.velocity.x, rigid.state.velocity.z);
    const speedEl = document.getElementById('hud-speed');
    const gearEl = document.getElementById('hud-gear');
    if (speedEl) speedEl.textContent = (speed * 3.6).toFixed(0) + ' km/h';
    if (gearEl) gearEl.textContent = t.rpm ? `${t.gear} 挡 · ${Math.round(t.rpm)} rpm` : 'N';
    if (hudSurface) {
      const contacts = rigid.lastContacts || [];
      const mu = contacts.length ? Math.min(...contacts.map(c => c.friction)) : null;
      hudSurface.textContent = mu === null ? '路面 μ —' : `路面 μ ${mu.toFixed(2)}`;
    }
    if (hudContacts) hudContacts.textContent = `${t.wheelContacts} 轮接触 · ${t.chassisContacts} 底盘接触`;
    if (hudWeather && weatherSystem) {
      const weather = weatherSystem.getState();
      hudWeather.textContent = `天气 ${weather.label} · ${Math.round(weather.wetness * 100)}% 湿 · ${Math.round(weather.temperature)}°C · 风 ${weather.windSpeed.toFixed(0)} m/s`;
    }
    if (status && performance.now() - lastStatus > 1200) {
      lastStatus = performance.now();
      status.textContent = `测试场运行中 · ${rigid.world.stepCount} 子步 · ${(rigid.lastContacts || []).length} 轮接触`;
    }
  }
}

if (status) status.textContent = '测试场已就绪 · 刚体内核 3.0';
render();
