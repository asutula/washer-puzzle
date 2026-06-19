/*
 * Headless smoke test: run the *browser* code (render.js + app.js) in a vm with
 * a stubbed DOM + canvas, boot the app, and fire a few interactions. This won't
 * verify pixels, but it proves the wiring executes without throwing.
 */
const fs = require('fs');
const path = require('path');
const vm = require('vm');

function makeCtx() {
  // Permissive 2D-context stub: every method is a no-op, measureText returns a width.
  return new Proxy({}, {
    get(target, prop) {
      if (prop === 'measureText') return () => ({ width: 12 });
      if (prop in target) return target[prop];
      return () => {};
    },
    set(target, prop, val) { target[prop] = val; return true; },
  });
}

function makeEl(id) {
  const listeners = {};
  return {
    id, value: '', textContent: '', disabled: false,
    min: '0', max: '100', innerHTML: '',
    style: {},
    classList: { add() {}, remove() {}, toggle() {}, contains() { return false; } },
    addEventListener(type, fn) { (listeners[type] = listeners[type] || []).push(fn); },
    removeEventListener() {},
    setPointerCapture() {}, releasePointerCapture() {},
    getBoundingClientRect() { return { width: 900, height: 650, left: 0, top: 0 }; },
    getContext() { return makeCtx(); },
    _fire(type, evt) { (listeners[type] || []).forEach((fn) => fn(Object.assign({
      preventDefault() {}, clientX: 0, clientY: 0, deltaY: 0, pointerId: 1, key: '', shiftKey: false,
    }, evt))); },
  };
}

const elements = {};
function getEl(id) { return elements[id] || (elements[id] = makeEl(id)); }

const winListeners = {};
const sandbox = {
  console,
  devicePixelRatio: 1,
  performance: { now: () => Date.now() },
  requestAnimationFrame: () => 0,   // do not auto-loop in the harness
  cancelAnimationFrame: () => {},
  setTimeout: (cb) => { cb(); return 0; },
  document: {
    getElementById: getEl,
    createElement: () => makeEl('created'),
  },
  addEventListener(type, fn) { (winListeners[type] = winListeners[type] || []).push(fn); },
};
sandbox.window = sandbox;
vm.createContext(sandbox);

function load(file) {
  const src = fs.readFileSync(path.join(__dirname, '..', 'js', file), 'utf8');
  vm.runInContext(src, sandbox, { filename: file });
}

let failed = false;
try {
  ['geometry.js', 'model.js', 'solver.js', 'render.js', 'app.js'].forEach(load);
  console.log('  ✓ all scripts loaded & app booted');

  // Solve (setTimeout is synchronous in the harness, so this runs the search).
  // Use a short (H<=90) washer so this wiring check stays fast; the H>90
  // validated optimizer is covered by test/test.js.
  getEl('nHeight').value = '82'; getEl('nHeight')._fire('input');
  getEl('res').value = 'coarse';
  getEl('btnSolve')._fire('click');
  const res = getEl('result');
  if (!/minimum bay depth/.test(res.innerHTML)) throw new Error('solve produced no result html');
  console.log('  ✓ solve ran and populated result panel');

  // Scrub the timeline (exercises path interpolation + render).
  const tl = getEl('timeline'); tl.value = '500'; tl._fire('input');
  console.log('  ✓ timeline scrub rendered a pose');

  // Drag / wheel / keys.
  const c = getEl('canvas');
  c._fire('pointerdown', { clientX: 450, clientY: 325 });
  c._fire('pointermove', { clientX: 470, clientY: 330 });
  c._fire('pointerup', {});
  c._fire('wheel', { deltaY: 120 });
  (winListeners['keydown'] || []).forEach((fn) => fn({ preventDefault() {}, key: 'q' }));
  console.log('  ✓ drag / wheel / keyboard handlers ran');

  // Param change should invalidate the plan without throwing.
  getEl('hHeight').value = '110'; getEl('hHeight')._fire('input');
  getEl('btnStart')._fire('click');
  getEl('btnGoal')._fire('click');
  getEl('btnReset')._fire('click');
  console.log('  ✓ parameter + preset buttons ran');
} catch (e) {
  failed = true;
  console.error('  ✗ smoke test threw:', e && e.stack || e);
}

console.log(failed ? '\nSMOKE FAILED' : '\nSMOKE OK');
process.exit(failed ? 1 : 0);
