import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js';

const EngineAPI = window.PlayCarPhysicsEngine;
const status = document.getElementById('scene-status');
const host = document.getElementById('scene3d');
if (!EngineAPI || !host) {
  if (status) status.textContent = '3D 场景初始化失败：缺少物理引擎或容器';
  throw new Error('PlayCar 3D scene prerequisites missing');
}

const { PlayCarPhysicsEngine, TriangleMesh, Vector3: V, Quaternion: Q } = EngineAPI;

host.style.position = 'relative';
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x8fb6cf);
scene.fog = new THREE.Fog(0x8fb6cf, 130, 340);
const camera = new THREE.PerspectiveCamera(58, 1, 0.1, 600);
camera.position.set(0, 8, 46);

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
renderer.setPixelRatio(Math.min(devicePixelRatio || 1, 1.75));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.08;
host.appendChild(renderer.domElement);
renderer.domElement.setAttribute('aria-label', '可交互三维赛车物理模拟场景');
renderer.domElement.style.touchAction = 'none';

scene.add(new THREE.HemisphereLight(0xdceeff, 0x344455, 2.0));
const sun = new THREE.DirectionalLight(0xfff1d7, 3.1);
sun.position.set(-55, 80, 35);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
sun.shadow.camera.left = -110; sun.shadow.camera.right = 110;
sun.shadow.camera.top = 110; sun.shadow.camera.bottom = -110;
sun.shadow.camera.far = 300;
scene.add(sun);
const fill = new THREE.DirectionalLight(0x75cfff, 0.7);
fill.position.set(45, 22, -60);
scene.add(fill);

const mats = {
  road: new THREE.MeshStandardMaterial({ color: 0xffffff, vertexColors: true, roughness: 0.9 }),
  ground: new THREE.MeshStandardMaterial({ color: 0x4e7350, roughness: 1 }),
  wall: new THREE.MeshStandardMaterial({ color: 0xdfe9ef, roughness: 0.6 }),
  wallTop: new THREE.MeshStandardMaterial({ color: 0xff713d, roughness: 0.45 }),
  line: new THREE.MeshStandardMaterial({ color: 0xe8f1f5, roughness: 0.7 }),
  cone: new THREE.MeshStandardMaterial({ color: 0xff7b2e, roughness: 0.55 }),
  crate: new THREE.MeshStandardMaterial({ color: 0xb08a53, roughness: 0.8 }),
  orange: new THREE.MeshStandardMaterial({ color: 0xff713d, roughness: 0.34, metalness: 0.25 }),
  glass: new THREE.MeshStandardMaterial({ color: 0x173a4f, roughness: 0.16, metalness: 0.4 }),
  tire: new THREE.MeshStandardMaterial({ color: 0x111820, roughness: 0.92 }),
  rim: new THREE.MeshStandardMaterial({ color: 0xa8bdcc, metalness: 0.85, roughness: 0.22 }),
  skid: new THREE.MeshBasicMaterial({ color: 0x1a2228, transparent: true, opacity: 0.42, depthWrite: false })
};

const world = new THREE.Group();
scene.add(world);

/* ------------------------------------------------------------------ */
/* Track                                                              */
/* ------------------------------------------------------------------ */

const physicsTriangles = [];
const colliderTriangles = [];
const groundTriangles = [];
const ROAD_HALF_WIDTH = 7.2;

function addTriangle(target, a, b, c, friction) {
  target.push(friction === undefined ? [a, b, c] : [a, b, c, friction]);
}
function addQuad(target, a, b, c, d, friction) {
  addTriangle(target, a, b, c, friction);
  addTriangle(target, a, c, d, friction);
}
function meshFromTriangles(tris, material) {
  const positions = [];
  for (const t of tris) for (const p of t) positions.push(p.x, p.y, p.z);
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  g.computeVertexNormals();
  const m = new THREE.Mesh(g, material);
  m.receiveShadow = true;
  m.castShadow = true;
  world.add(m);
  return m;
}

// Closed Catmull-Rom circuit. The start straight includes z = 34 so the
// documented spawn (0, 0.08, 34) sits exactly on the racing surface.
const CONTROL = [
  [0, 60], [0, 20], [0, -20], [0, -62], [16, -84], [44, -88], [66, -72],
  [74, -44], [70, -12], [74, 22], [60, 52], [34, 66], [12, 66]
];
function catmull(p0, p1, p2, p3, t) {
  const t2 = t * t, t3 = t2 * t;
  const f = (a, b, c, d) => 0.5 * ((2 * b) + (-a + c) * t + (2 * a - 5 * b + 4 * c - d) * t2 + (-a + 3 * b - 3 * c + d) * t3);
  return { x: f(p0[0], p1[0], p2[0], p3[0]), z: f(p0[1], p1[1], p2[1], p3[1]) };
}

const centerPoints = [];
const SUB = 8;
for (let i = 0; i < CONTROL.length; i++) {
  const p0 = CONTROL[(i - 1 + CONTROL.length) % CONTROL.length];
  const p1 = CONTROL[i];
  const p2 = CONTROL[(i + 1) % CONTROL.length];
  const p3 = CONTROL[(i + 2) % CONTROL.length];
  for (let s = 0; s < SUB; s++) centerPoints.push(catmull(p0, p1, p2, p3, s / SUB));
}
const N = centerPoints.length;
const track = [];
for (let i = 0; i < N; i++) {
  const prev = centerPoints[(i - 1 + N) % N];
  const cur = centerPoints[i];
  const next = centerPoints[(i + 1) % N];
  const tPrev = V.norm({ x: cur.x - prev.x, y: 0, z: cur.z - prev.z });
  const tNext = V.norm({ x: next.x - cur.x, y: 0, z: next.z - cur.z });
  const tangent = V.norm(V.add(tPrev, tNext));
  const curvature = tPrev.z * tNext.x - tPrev.x * tNext.z;
  const bank = V.clamp(-curvature * 7.5, -0.34, 0.34);
  const up = Q.rotate(Q.axis(tangent, bank), { x: 0, y: 1, z: 0 });
  const right = V.norm(V.cross(tangent, up));
  track.push({ center: { x: cur.x, y: 0, z: cur.z }, tangent, right, up, bank, curvature, u: i / N });
}

// Smooth and clamp the banking. The Catmull-Rom seam and the tightest corner
// can otherwise produce ~20-degree cross-slopes that throw the car off the
// road. A short moving average keeps the circuit banked without wall-like kerbs.
{
  const raw = track.map(f => f.bank);
  for (let i = 0; i < N; i++) {
    let sum = 0;
    for (let k = -4; k <= 4; k++) sum += raw[(i + k + N) % N];
    const bank = V.clamp(sum / 9, -0.16, 0.16);
    const f = track[i];
    f.bank = bank;
    f.up = Q.rotate(Q.axis(f.tangent, bank), { x: 0, y: 1, z: 0 });
    f.right = V.norm(V.cross(f.tangent, f.up));
  }
}

const vertexPositions = [];
const vertexColors = [];
const indices = [];
const wetStart = Math.floor(N * 0.55), wetEnd = Math.floor(N * 0.64);
const dryColor = new THREE.Color(0x3a4b58), wetColor = new THREE.Color(0x27455c);
for (let i = 0; i < N; i++) {
  const f = track[i];
  const wet = i >= wetStart && i < wetEnd;
  const L = V.add(f.center, V.mul(f.right, -ROAD_HALF_WIDTH));
  const R = V.add(f.center, V.mul(f.right, ROAD_HALF_WIDTH));
  vertexPositions.push(L.x, L.y, L.z, R.x, R.y, R.z);
  const color = wet ? wetColor : dryColor;
  vertexColors.push(color.r, color.g, color.b, color.r, color.g, color.b);
  const j = (i + 1) % N;
  indices.push(i * 2, i * 2 + 1, j * 2 + 1, i * 2, j * 2 + 1, j * 2);
}
const roadGeometry = new THREE.BufferGeometry();
roadGeometry.setAttribute('position', new THREE.Float32BufferAttribute(vertexPositions, 3));
roadGeometry.setAttribute('color', new THREE.Float32BufferAttribute(vertexColors, 3));
roadGeometry.setIndex(indices);
roadGeometry.computeVertexNormals();
const roadMesh = new THREE.Mesh(roadGeometry, mats.road);
roadMesh.receiveShadow = true;
roadMesh.castShadow = true;
world.add(roadMesh);

for (let i = 0; i < N; i++) {
  const j = (i + 1) % N;
  const f = track[i], g = track[j];
  const wet = i >= wetStart && i < wetEnd;
  const friction = wet ? 0.55 : 1.0;
  const Li = V.add(f.center, V.mul(f.right, -ROAD_HALF_WIDTH));
  const Ri = V.add(f.center, V.mul(f.right, ROAD_HALF_WIDTH));
  const Lj = V.add(g.center, V.mul(g.right, -ROAD_HALF_WIDTH));
  const Rj = V.add(g.center, V.mul(g.right, ROAD_HALF_WIDTH));
  addTriangle(physicsTriangles, Li, Ri, Rj, friction);
  addTriangle(physicsTriangles, Li, Rj, Lj, friction);

  // Continuous low barriers on both sides, as visual ribbons plus colliders.
  for (const side of [-1, 1]) {
    const ei = V.add(f.center, V.mul(f.right, side * (ROAD_HALF_WIDTH + 0.12)));
    const ej = V.add(g.center, V.mul(g.right, side * (ROAD_HALF_WIDTH + 0.12)));
    const hi = { x: ei.x, y: 1.15, z: ei.z };
    const hj = { x: ej.x, y: 1.15, z: ej.z };
    addQuad(colliderTriangles, ei, ej, hj, hi, 1);
  }
}

// Barrier ribbons (one geometry per side).
for (const side of [-1, 1]) {
  const pos = [], idx = [];
  for (let i = 0; i < N; i++) {
    const f = track[i];
    const e = V.add(f.center, V.mul(f.right, side * (ROAD_HALF_WIDTH + 0.22)));
    pos.push(e.x, 0, e.z, e.x, 1.15, e.z);
    const j = (i + 1) % N;
    idx.push(i * 2, j * 2, j * 2 + 1, i * 2, j * 2 + 1, i * 2 + 1);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setIndex(idx);
  g.computeVertexNormals();
  const wall = new THREE.Mesh(g, mats.wall);
  wall.material.side = THREE.DoubleSide;
  wall.castShadow = true;
  wall.receiveShadow = true;
  world.add(wall);
  const topPoints = [];
  for (let i = 0; i <= N; i++) {
    const f = track[i % N];
    const e = V.add(f.center, V.mul(f.right, side * (ROAD_HALF_WIDTH + 0.22)));
    topPoints.push(new THREE.Vector3(e.x, 1.17, e.z));
  }
  const topGeo = new THREE.BufferGeometry().setFromPoints(topPoints);
  world.add(new THREE.Line(topGeo, new THREE.LineBasicMaterial({ color: 0xff713d })));
}

// Ground: visual lawn plus a solid fallback support plane.
const groundGeo = new THREE.PlaneGeometry(360, 360, 1, 1);
groundGeo.rotateX(-Math.PI / 2);
const ground = new THREE.Mesh(groundGeo, mats.ground);
ground.position.y = -0.18;
ground.receiveShadow = true;
world.add(ground);
addQuad(groundTriangles,
  { x: -180, y: -0.18, z: 180 }, { x: -180, y: -0.18, z: -180 },
  { x: 180, y: -0.18, z: -180 }, { x: 180, y: -0.18, z: 180 }, 0.72);

// Start / finish stripe.
for (let k = -1; k <= 1; k++) {
  const stripe = new THREE.Mesh(new THREE.BoxGeometry(ROAD_HALF_WIDTH * 0.9, 0.02, 0.45), mats.line);
  stripe.position.set(0, 0.012, 34 + k * 0.7);
  world.add(stripe);
}

// A tabletop jump on the start straight.
function createRamp() {
  const idx = track.reduce((best, f, i) => Math.abs(f.center.z + 40) < Math.abs(track[best].center.z + 40) && Math.abs(f.center.x) < 2 ? i : best, 0);
  const f = track[idx];
  const tangent = f.tangent, right = f.right;
  const base = V.add(f.center, V.mul(tangent, -3));
  const height = 1.85;
  const upEnd = V.add(base, V.mul(tangent, 7));
  const flatEnd = V.add(upEnd, V.mul(tangent, 5));
  const downEnd = V.add(flatEnd, V.mul(tangent, 9));
  const halfW = 6.3;
  const L = p => V.add(p, V.mul(right, -halfW));
  const R = p => V.add(p, V.mul(right, halfW));
  const lift = (p, h) => ({ x: p.x, y: h, z: p.z });
  const p0 = { ...base }, p1 = lift(upEnd, height), p2 = lift(flatEnd, height), p3 = { ...downEnd };
  const surface = [
    [L(p0), R(p0), R(p1)], [L(p0), R(p1), L(p1)],
    [L(p1), R(p1), R(p2)], [L(p1), R(p2), L(p2)],
    [L(p2), R(p2), R(p3)], [L(p2), R(p3), L(p3)]
  ];
  // The ramp deck is a wheel surface only. Adding it to the chassis collider
  // mesh makes a long rigid box bridge the crest and high-centre, which stalls
  // the car at full throttle; the wheels still climb and support the car.
  for (const tri of surface) addTriangle(physicsTriangles, tri[0], tri[1], tri[2], 1);
  const side = [L(p0), L(p1), L(p2), L(p3), R(p3), R(p2), R(p1), R(p0)];
  for (let i = 0; i < side.length; i++) {
    const a = side[i], b = side[(i + 1) % side.length];
    addTriangle(colliderTriangles, a, b, { x: b.x, y: 0, z: b.z }, 1);
    addTriangle(colliderTriangles, a, { x: b.x, y: 0, z: b.z }, { x: a.x, y: 0, z: a.z }, 1);
  }
  const positions = [];
  for (const tri of surface) for (const p of tri) positions.push(p.x, p.y, p.z);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geo.computeVertexNormals();
  const ramp = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({
    color: 0x647d90, roughness: 0.62, metalness: 0.08, side: THREE.DoubleSide
  }));
  ramp.receiveShadow = true;
  ramp.castShadow = true;
  world.add(ramp);
  for (let i = 0; i < 6; i++) {
    const stripe = new THREE.Mesh(new THREE.BoxGeometry(halfW * 0.92, 0.05, 0.14), mats.line);
    const p = V.add(V.add(base, V.mul(tangent, 0.8 + i * 1.05)), { x: 0, y: 0.22 + i * 0.29, z: 0 });
    stripe.position.set(p.x, p.y, p.z);
    stripe.rotation.y = -Math.atan2(tangent.x, tangent.z);
    stripe.rotation.z = Math.atan2(height, 7);
    world.add(stripe);
  }
}
createRamp();

function createBarrier(x1, z1, x2, z2, height = 1.15) {
  const a = { x: x1, y: 0, z: z1 }, b = { x: x2, y: 0, z: z2 };
  const c = { x: x2, y: height, z: z2 }, d = { x: x1, y: height, z: z1 };
  addQuad(colliderTriangles, a, b, c, d, 1);
  meshFromTriangles([[a, b, c], [a, c, d]], mats.wall);
}
// Two barrier stubs framing the pit lane next to the start straight.
createBarrier(-9.6, 44, -9.6, 24);
createBarrier(9.6, 44, 9.6, 24);

/* ------------------------------------------------------------------ */
/* Vehicle                                                            */
/* ------------------------------------------------------------------ */

const surface = TriangleMesh.fromTriangles([...physicsTriangles, ...groundTriangles], 8);
const walls = TriangleMesh.fromTriangles(colliderTriangles, 6);

const vehicle = new PlayCarPhysicsEngine({
  surfaceMesh:surface,
  colliderMeshes:[walls],
  groundPlaneY:-.18,
  position: { x: 0, y: 0.08, z: 34 },
  preset: 'simcade',
  config: {
    gravity: 9.81,
    maxSpeed: 62,
    engineAcceleration: 6.4,
    brakeAcceleration: 11,
    tireGrip: 1.08,
    handbrakeGrip: 0.7,
    maxSteerAngle: 0.5,
    highSpeedSteerScale: 0.44,
    suspensionRestLength: 0.3,
    suspensionReach: 0.52,
    bodyHalfExtents: { x: 0.84, y: 0.4, z: 1.7 },
    bodyCenterY: 0.2
  }
});
window.PlayCarLabVehicle = vehicle;

const rigid = vehicle.rigidBody;
rigid.c.aids.esc = true;
rigid.c.suspension.progressive = 0.45;
rigid.c.suspension.dampingBump = 4200;
rigid.c.suspension.dampingRebound = 6200;
rigid.c.tire.relaxationLength = 0.32;

const carRoot = new THREE.Group();
world.add(carRoot);

const chassis = new THREE.Mesh(new THREE.BoxGeometry(1.72, 0.58, 3.4), mats.orange);
chassis.position.y = 0.02;
chassis.castShadow = true;
chassis.receiveShadow = true;
carRoot.add(chassis);
const nose = new THREE.Mesh(new THREE.BoxGeometry(1.5, 0.22, 0.72), mats.orange);
nose.position.set(0, -0.1, -1.85);
nose.castShadow = true;
carRoot.add(nose);
const cabin = new THREE.Mesh(new THREE.BoxGeometry(1.32, 0.56, 1.4), mats.glass);
cabin.position.set(0, 0.52, 0.15);
cabin.castShadow = true;
carRoot.add(cabin);
const spoiler = new THREE.Mesh(new THREE.BoxGeometry(1.7, 0.08, 0.4), mats.orange);
spoiler.position.set(0, 0.42, 1.68);
carRoot.add(spoiler);
for (const z of [-1.3, 1.3]) {
  const brace = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.34, 0.08), mats.orange);
  brace.position.set(0.62, 0.26, z);
  carRoot.add(brace);
  const brace2 = brace.clone();
  brace2.position.x = -0.62;
  carRoot.add(brace2);
}
const headlightMat = new THREE.MeshStandardMaterial({ color: 0xfff4c4, emissive: 0xffd98b, emissiveIntensity: 1.7 });
for (const x of [-0.5, 0.5]) {
  const lamp = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.12, 0.08), headlightMat);
  lamp.position.set(x, -0.03, -1.72);
  carRoot.add(lamp);
}

const wheels = [];
const WHEEL_RADIUS = rigid.c.wheelRadius;
for (const z of [-1.35, 1.35]) {
  for (const x of [-0.78, 0.78]) {
    const pivot = new THREE.Group();
    pivot.position.set(x, -0.28, z);
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
    wheels.push({ pivot, spinner, z, x });
  }
}

/* Dynamic props: cones and crates the rigid-body core can knock around. */
const props = [];
function addPropVisual(kind, body) {
  let mesh;
  if (kind === 'cone') {
    mesh = new THREE.Mesh(new THREE.ConeGeometry(0.22, 0.7, 14), mats.cone);
    mesh.position.y = 0.02;
  } else {
    mesh = new THREE.Mesh(new THREE.BoxGeometry(0.62, 0.62, 0.62), mats.crate);
  }
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  const group = new THREE.Group();
  group.add(mesh);
  world.add(group);
  props.push({ group, body });
}
for (const z of [16, 10, 4, -2, -8]) {
  const body = vehicle.addProp({
    shape: 'box', halfExtents: { x: 0.2, y: 0.35, z: 0.2 }, mass: 7,
    position: { x: -3.6, y: 0.38, z }, friction: 0.75, restitution: 0.1
  });
  addPropVisual('cone', body);
}
for (const [x, z] of [[3.8, 2], [3.8, 1.2], [4.3, 1.6]]) {
  const body = vehicle.addProp({
    shape: 'box', halfExtents: { x: 0.31, y: 0.31, z: 0.31 }, mass: 16,
    position: { x, y: 0.35 + (x === 4.3 ? 0.65 : 0), z }, friction: 0.8, restitution: 0.08
  });
  addPropVisual('crate', body);
}

/* Skid marks. */
const skidPool = [];
const SKID_COUNT = 260;
const skidGeo = new THREE.PlaneGeometry(0.3, 0.75);
skidGeo.rotateX(-Math.PI / 2);
for (let i = 0; i < SKID_COUNT; i++) {
  const m = new THREE.Mesh(skidGeo, mats.skid);
  m.visible = false;
  m.renderOrder = 1;
  world.add(m);
  skidPool.push(m);
}
let skidIndex = 0;

/* HUD overlay */
const overlay = document.createElement('div');
overlay.style.cssText = 'position:absolute;left:10px;bottom:10px;padding:8px 11px;border-radius:10px;background:rgba(7,17,31,.68);color:#e8f5ff;font:12px/1.55 ui-monospace,monospace;pointer-events:none;backdrop-filter:blur(4px)';
overlay.innerHTML = '<b id="hud-speed">0 km/h</b> · <span id="hud-gear">N</span> · <span id="hud-aids"></span><br><span id="hud-slip">grip —</span>';
host.appendChild(overlay);

/* ------------------------------------------------------------------ */
/* Input                                                              */
/* ------------------------------------------------------------------ */

const input = { gas: false, brake: false, left: false, right: false, handbrake: false };
function release() { for (const k of Object.keys(input)) input[k] = false; }
function bindHold(button, key) {
  const el = document.getElementById(button);
  if (!el) return;
  el.addEventListener('pointerdown', e => {
    e.preventDefault(); input[key] = true;
    try { el.setPointerCapture(e.pointerId); } catch (_) { /* ignore */ }
  });
  ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(type =>
    el.addEventListener(type, e => { if (e) e.preventDefault(); input[key] = false; }));
}
[['scene-gas', 'gas'], ['scene-brake', 'brake'], ['scene-left', 'left'], ['scene-right', 'right'], ['scene-handbrake', 'handbrake']]
  .forEach(x => bindHold(...x));

window.addEventListener('keydown', e => {
  if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', ' '].includes(e.key)) e.preventDefault();
  const k = e.key.toLowerCase();
  if (k === 'arrowup' || k === 'w') input.gas = true;
  if (k === 'arrowdown' || k === 's') input.brake = true;
  if (k === 'arrowleft' || k === 'a') { input.left = true; input.steer = null; }
  if (k === 'arrowright' || k === 'd') { input.right = true; input.steer = null; }
  if (e.key === ' ') input.handbrake = true;
  if (k === 'r') resetVehicle();
}, { passive: false });
window.addEventListener('keyup', e => {
  const k = e.key.toLowerCase();
  if (k === 'arrowup' || k === 'w') input.gas = false;
  if (k === 'arrowdown' || k === 's') input.brake = false;
  if (k === 'arrowleft' || k === 'a') input.left = false;
  if (k === 'arrowright' || k === 'd') input.right = false;
  if (e.key === ' ') input.handbrake = false;
});
window.addEventListener('blur', release);

function resetVehicle() {
  vehicle.reset({x:0,y:.08,z:34});
  rigid._layout = null;
  input.steer = null;
  release();
  camera.position.set(0, 7, 45);
}
document.getElementById('scene-reset')?.addEventListener('click', resetVehicle);
document.getElementById('reset-car')?.addEventListener('click', resetVehicle);

let follow = true;
document.getElementById('scene-follow')?.addEventListener('click', e => {
  follow = !follow;
  e.currentTarget.textContent = follow ? '跟随视角：开' : '跟随视角：关';
});

/* Control-center facade: the lab UI only talks to this stable contract. */
const runtime = {
  engine: vehicle,
  input,
  setInput(patch = {}) { Object.assign(input, patch); return { ...input }; },
  setWeather(value = {}) { return vehicle.setWeather(value); },
  setConfig(patch = {}) { return vehicle.setConfig(patch); },
  setRigidConfig(patch = {}) { return vehicle.setRigidConfig(patch); },
  reset() { resetVehicle(); return vehicle.getSnapshot(); },
  snapshot() { return vehicle.getSnapshot(); }
};
window.PlayCarRuntime = runtime;
window.dispatchEvent(new CustomEvent('playcar:ready', { detail: { version: EngineAPI.version } }));

/* ------------------------------------------------------------------ */
/* Render loop                                                        */
/* ------------------------------------------------------------------ */

const clock = new THREE.Clock();
let frame = 0;
let lastStatus = 0;
const cameraTarget = new THREE.Vector3(0, 1, 34);
const cameraPosition = new THREE.Vector3(0, 7, 45);
const tmpForward = new THREE.Vector3();

function resize() {
  const w = Math.max(1, host.clientWidth);
  const h = Math.max(280, host.clientHeight);
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
const ro = new ResizeObserver(resize);
ro.observe(host);
resize();

function render() {
  requestAnimationFrame(render);
  const dt = Math.min(clock.getDelta(), 0.05);

  // Digital keys/buttons and the control center's analog sliders share the
  // same input object. A numeric value is an analog level; a boolean is 0/1.
  const level = v => typeof v === 'number' ? V.clamp(v, 0, 1) : (v ? 1 : 0);
  const steer = Number.isFinite(input.steer)
    ? V.clamp(input.steer, -1, 1)
    : (input.right ? 1 : 0) - (input.left ? 1 : 0);
  vehicle.advance(dt, {
    throttle: level(input.gas),
    brake: level(input.brake),
    steer,
    handbrake: level(input.handbrake)
  });

  const pose = rigid.interpolatedState();
  const wheelState = vehicle.getWheelState();
  carRoot.position.set(pose.position.x, pose.position.y, pose.position.z);
  carRoot.quaternion.set(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

  const suspension = rigid.telemetry.suspension || [];
  const slipRatios = wheelState.slip || [0, 0, 0, 0];
  for (let ix = 0; ix < wheels.length; ix++) {
    const w = wheels[ix];
    const compression = suspension[ix] ? suspension[ix].compression : 0;
    const length = Math.max(rigid.c.suspension.min, rigid.c.suspension.rest - compression);
    w.pivot.position.y = -length;
    w.pivot.rotation.y=w.z<0?-wheelState.steer:0;
    w.spinner.rotation.x -= wheelState.omega[ix] * dt;
  }

  for (const prop of props) {
    prop.group.position.set(prop.body.position.x, prop.body.position.y, prop.body.position.z);
    prop.group.quaternion.set(prop.body.orientation.x, prop.body.orientation.y, prop.body.orientation.z, prop.body.orientation.w);
  }

  // Skid marks at slipping contact patches.
  if (frame % 2 === 0) {
    const contacts = rigid.lastContacts || [];
    for (const c of contacts) {
      const slip = Math.abs(slipRatios[c.index] || 0);
      if (slip < 0.22 || !pose.grounded) continue;
      const mark = skidPool[skidIndex++ % SKID_COUNT];
      mark.position.set(c.point.x, c.point.y + 0.012, c.point.z);
      const f = Q.rotate(pose.orientation, { x: 0, y: 0, z: -1 });
      mark.rotation.y = Math.atan2(f.x, f.z);
      mark.visible = true;
    }
  }

  const speed = V.len(rigid.state.velocity);
  const forward = Q.rotate(pose.orientation, { x: 0, y: 0, z: -1 });
  tmpForward.set(forward.x, forward.y, forward.z);

  if (follow) {
    const back = new THREE.Vector3(0, 3.1, 8.2).applyQuaternion(carRoot.quaternion);
    const desired = new THREE.Vector3(pose.position.x, pose.position.y, pose.position.z).add(back);
    cameraPosition.lerp(desired, Math.min(1, dt * 6));
    camera.position.copy(cameraPosition);
    camera.lookAt(
      pose.position.x + tmpForward.x * 4.5,
      pose.position.y + 1.0 + tmpForward.y * 2.5,
      pose.position.z + tmpForward.z * 4.5
    );
  } else {
    camera.lookAt(pose.position.x, pose.position.y + 0.6, pose.position.z);
  }
  camera.fov = 58 + Math.min(16, speed * 0.5);
  camera.updateProjectionMatrix();

  renderer.render(scene, camera);

  if (++frame % 6 === 0) {
    const t = vehicle.getTelemetry();
    const hudSpeed = document.getElementById('hud-speed');
    const hudGear = document.getElementById('hud-gear');
    const hudAids = document.getElementById('hud-aids');
    const hudSlip = document.getElementById('hud-slip');
    const aids = [];
    if (t.absActive) aids.push('ABS');
    if (t.tcsActive) aids.push('TCS');
    if (t.escActive) aids.push('ESC');
    if (hudSpeed) hudSpeed.textContent = (speed * 3.6).toFixed(0) + ' km/h';
    if (hudGear) hudGear.textContent = t.rpm ? `${t.gear} 挡 · ${Math.round(t.rpm)} rpm` : 'N';
    if (hudAids) hudAids.textContent = aids.join(' ') || '—';
    if (hudSlip) {
      const maxSlip = Math.max(...(t.wheelSlip || [0]));
      hudSlip.textContent = `${t.grounded ? '抓地' : '腾空'} · slip ${maxSlip.toFixed(2)} · ${t.contacts} 轮接触`;
    }
    if (status && performance.now() - lastStatus > 1200) {
      lastStatus = performance.now();
      status.textContent = `3D 场景运行中 · ${rigid.world.stepCount} 物理子步 · ${rigid.world.stats.contacts} 接触点`;
    }
  }
}

if (status) status.textContent = '3D 场景已就绪 · 刚体内核 3.0';
render();
