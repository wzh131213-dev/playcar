'use strict';
/* Runs the browser check suite headlessly: the same physics/physics-checks.js
 * that test-scene.html loads as a classic script, executed against the real
 * bundle in a fresh VM context. Any failed scenario fails this test.
 */
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const root = path.join(__dirname, '..');

function loadSuite() {
  const bundle = fs.readFileSync(path.join(root, 'physics', 'playcar-physics-all-in-one.js'), 'utf8');
  const marker = bundle.indexOf('(async function startPlayCar3DScene');
  assert.ok(marker > 0, 'bundle scene marker missing');
  const context = { console, performance: globalThis.performance };
  vm.createContext(context);
  vm.runInContext(bundle.slice(0, marker), context, { filename: 'playcar-physics-prefix.js' });
  vm.runInContext(fs.readFileSync(path.join(root, 'physics', 'physics-checks.js'), 'utf8'), context, {
    filename: 'physics-checks.js'
  });
  return context.PlayCarPhysicsChecks;
}

test('physics check suite passes all isolated scenarios', () => {
  const suite = loadSuite();
  assert.ok(suite && typeof suite.runAll === 'function');
  const result = suite.runAll();
  const results = Array.from(result.results);
  const failures = results.filter(r => !r.ok).map(r => `${r.id}: ${r.detail}`);
  assert.equal(failures.length, 0, 'failed checks:\n' + failures.join('\n'));
  assert.equal(result.total, 11);
  assert.equal(result.passed, 11);
  assert.equal(result.failed, 0);
  assert.equal(result.ok, true);
  assert.equal(result.engine, '3.0.0');
  for (const item of results) {
    assert.equal(typeof item.id, 'string');
    assert.equal(typeof item.name, 'string');
    assert.ok(item.name.length > 0);
    assert.ok(item.detail && typeof item.detail === 'object');
    assert.ok(Number.isFinite(item.ms) && item.ms >= 0);
  }
});
