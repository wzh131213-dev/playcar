#!/usr/bin/env node
/* Headless smoke test for physics/scene3d.js.
 *
 * Node cannot create a WebGL context, so this harness stubs the renderer and the
 * small DOM surface the scene touches, then imports the real Three.js build and
 * the real scene module. Everything else (track construction, the unified 3.0
 * engine, the vehicle, props, telemetry, render loop) is the shipping code.
 *
 * Run: node tools/scene-smoke.mjs
 */
import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import { fileURLToPath } from 'node:url';

const root = path.dirname(path.dirname(fileURLToPath(import.meta.url)));

class FakeElement {
  constructor(tag = 'div', id = '') {
    this.tagName = tag.toUpperCase();
    this.id = id;
    this.children = [];
    this.style = {};
    this.dataset = {};
    this.textContent = '';
    this.innerHTML = '';
    this.className = '';
    this.value = '0';
    this.clientWidth = 960;
    this.clientHeight = 520;
    this.attributes = {};
    this.listeners = new Map();
    this.classList = { add() {}, remove() {}, toggle() {} };
  }
  appendChild(child) { this.children.push(child); child.parentElement = this; return child; }
  append(...nodes) { for (const n of nodes) this.appendChild(typeof n === 'string' ? new FakeElement('span') : n); }
  replaceChildren(...nodes) { this.children = []; for (const n of nodes) this.appendChild(n); }
  remove() {}
  setAttribute(k, v) { this.attributes[k] = String(v); }
  getAttribute(k) { return this.attributes[k] ?? null; }
  addEventListener(type, fn) {
    if (!this.listeners.has(type)) this.listeners.set(type, []);
    this.listeners.get(type).push(fn);
  }
  removeEventListener() {}
  dispatch(type, event = {}) {
    for (const fn of this.listeners.get(type) || []) fn({ preventDefault() {}, currentTarget: this, ...event });
  }
  setPointerCapture() {}
  releasePointerCapture() {}
  getBoundingClientRect() { return { left: 0, top: 0, width: this.clientWidth, height: this.clientHeight }; }
  focus() {}
}

const elements = new Map();
const elementFor = id => {
  if (!elements.has(id)) elements.set(id, new FakeElement('div', id));
  return elements.get(id);
};
const sceneHost = elementFor('scene3d');
const status = elementFor('scene-status');
const canvas = new FakeElement('canvas');

class StubRenderer {
  constructor() {
    this.domElement = canvas;
    this.shadowMap = { enabled: false, type: 0 };
    this.outputColorSpace = '';
    this.toneMapping = 0;
    this.toneMappingExposure = 1;
    this.frames = 0;
  }
  setPixelRatio() {}
  setSize(w, h) { this.width = w; this.height = h; }
  render() { this.frames++; }
  dispose() {}
}

const listeners = new Map();
let fakeNow = 0;
const rafQueue = [];
const win = {
  PlayCarPhysicsEngine: null,
  devicePixelRatio: 1,
  addEventListener(type, fn) {
    if (!listeners.has(type)) listeners.set(type, []);
    listeners.get(type).push(fn);
  },
  dispatchEvent(event) {
    for (const fn of listeners.get(event.type) || []) fn(event);
    return true;
  }
};
const doc = {
  getElementById: id => elementFor(id),
  createElement: tag => new FakeElement(tag),
  createElementNS: (_ns, tag) => new FakeElement(tag),
  addEventListener() {},
  body: new FakeElement('body'),
  documentElement: new FakeElement('html')
};

globalThis.window = win;
globalThis.document = doc;
globalThis.devicePixelRatio = 1;
globalThis.CustomEvent = class CustomEvent {
  constructor(type, init = {}) { this.type = type; this.detail = init.detail; }
};
globalThis.ResizeObserver = class ResizeObserver {
  constructor(cb) { this.cb = cb; }
  observe() {}
  disconnect() {}
};
globalThis.requestAnimationFrame = fn => { rafQueue.push(fn); return rafQueue.length; };
globalThis.cancelAnimationFrame = () => {};
globalThis.performance = { now: () => fakeNow };
globalThis.self = globalThis;

// Load the real engine globals from the generated bundle prefix (everything
// before the scene IIFE).
const bundle = fs.readFileSync(path.join(root, 'physics/playcar-physics-all-in-one.js'), 'utf8');
const prefix = bundle.slice(0, bundle.indexOf('(async function startPlayCar3DScene'));
vm.runInThisContext(prefix, { filename: 'playcar-physics-prefix.js' });
if (!globalThis.PlayCarPhysicsEngine) throw new Error('bundle did not export PlayCarPhysicsEngine');
win.PlayCarPhysicsEngine = globalThis.PlayCarPhysicsEngine;

// Build an importable copy of the scene with the CDN import replaced by the
// vendored Three.js build and the WebGL renderer stubbed.
let scene = fs.readFileSync(path.join(root, 'physics/scene3d.js'), 'utf8');
scene = scene.replace(/^import\s+\*\s+as\s+THREE[^\n]*\n/, '');
const harness = `
import * as REAL from '../vendor/three.module.min.js';
const THREE = new Proxy(REAL, { get(target, key) { return key === 'WebGLRenderer' ? globalThis.__StubRenderer : target[key]; } });
${scene}
`;
const scenePath = path.join(root, 'tools', '.scene-under-test.mjs');
fs.writeFileSync(scenePath, harness, 'utf8');
globalThis.__StubRenderer = StubRenderer;

let error = null;
try {
  await import(`${scenePath}?t=${Date.now()}`);
} catch (e) {
  error = e;
}
fs.rmSync(scenePath, { force: true });
if (error) {
  console.error('SCENE IMPORT FAILED:', error && error.stack || error);
  process.exit(1);
}

const runtime = win.PlayCarRuntime;
if (!runtime || !runtime.engine) {
  console.error('SCENE DID NOT PUBLISH window.PlayCarRuntime');
  process.exit(1);
}
if (!runtime.engine.rigidBody) {
  console.error('RUNTIME HAS NO RIGID BODY');
  process.exit(1);
}

// Drive forward for ~4 simulated seconds (240 frames at 60 Hz).
runtime.setInput({ gas: true, steer: 0, brake: false, handbrake: false });
for (let i = 0; i < 300; i++) {
  fakeNow += 1000 / 60;
  const fn = rafQueue.shift();
  if (!fn) break;
  try { fn(fakeNow); } catch (e) {
    console.error(`SCENE FRAME ${i} FAILED:`, e && e.stack || e);
    process.exit(1);
  }
}
const snap = runtime.snapshot();
const p = snap.rigid ? snap.rigid.state.position : snap.state.position;
const speed = Math.hypot(snap.state.velocity.x, snap.state.velocity.z);
const tele = runtime.engine.getTelemetry();

if (!Number.isFinite(p.x) || !Number.isFinite(p.y) || !Number.isFinite(p.z)) {
  console.error('NON-FINITE VEHICLE POSITION', p); process.exit(1);
}
if (Math.abs(p.z - 34) < 0.5 && Math.abs(p.x) < 0.5) {
  console.error('VEHICLE DID NOT MOVE', p); process.exit(1);
}
if (!Number.isFinite(speed) || speed < 1) {
  console.error('VEHICLE SPEED TOO LOW', speed); process.exit(1);
}
if (!Array.isArray(tele.tireLoads) || tele.tireLoads.some(v => !Number.isFinite(v))) {
  console.error('INVALID TIRE LOADS', tele.tireLoads); process.exit(1);
}
if (!snap.solver || !snap.rigid) {
  console.error('SNAPSHOT MISSING RIGID DIAGNOSTICS'); process.exit(1);
}
if (canvas.parentElement !== sceneHost && !sceneHost.children.includes(canvas)) {
  console.error('RENDERER CANVAS NOT MOUNTED'); process.exit(1);
}
if (sceneHost.children.length === 0) {
  console.error('SCENE HOST EMPTY'); process.exit(1);
}

console.log('RESULT ' + JSON.stringify({
  ok: true,
  version: win.PlayCarPhysicsEngine.version,
  solver: snap.solver,
  frames: 300,
  position: { x: +p.x.toFixed(2), y: +p.y.toFixed(2), z: +p.z.toFixed(2) },
  speed: +speed.toFixed(2),
  gear: tele.gear,
  rpm: Math.round(tele.rpm),
  contacts: tele.contacts,
  wheelContacts: tele.wheelContacts,
  children: sceneHost.children.length,
  status: status.textContent
}));
