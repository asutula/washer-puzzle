/*
 * app.js — state, interaction and wiring.
 */
(function () {
  'use strict';
  const DEG = Math.PI / 180;

  const RES = {
    coarse: { dx: 2.5, dy: 2.5, daDeg: 2.5 },
    medium: { dx: 1.5, dy: 1.5, daDeg: 1.5 },
    fine:   { dx: 1, dy: 1, daDeg: 1 },
  };

  const canvas = document.getElementById('canvas');
  const renderer = new Renderer(canvas);

  const state = {
    dims: { depth: 60, height: 95 },
    bayDepth: 20,
    bayStart: 68,
    pose: { x: 0, y: 0, angle: 0 },
    env: null,
    plan: null,            // last solve result {depth, path, ...}
    anim: { playing: false, raf: 0, last: 0, p: 0 },
    drag: { active: false, ox: 0, oy: 0 },
  };

  // ---- DOM ----
  const el = (id) => document.getElementById(id);
  const ui = {
    hHeight: el('hHeight'), nHeight: el('nHeight'),
    hDepth: el('hDepth'), nDepth: el('nDepth'),
    hBay: el('hBay'), nBay: el('nBay'),
    hBayStart: el('hBayStart'), nBayStart: el('nBayStart'),
    hAngle: el('hAngle'), angleVal: el('angleVal'),
    status: el('status'), res: el('res'),
    btnStart: el('btnStart'), btnGoal: el('btnGoal'),
    btnSolve: el('btnSolve'), btnPlay: el('btnPlay'),
    btnReset: el('btnReset'),
    timeline: el('timeline'), result: el('result'),
    solving: el('solving'),
  };

  // ---- helpers ----
  function rebuildEnv() { state.env = Model.buildEnvironment(state.bayDepth, state.bayStart); }

  function pointInPoly(pt, poly) {
    let inside = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      const xi = poly[i].x, yi = poly[i].y, xj = poly[j].x, yj = poly[j].y;
      if (((yi > pt.y) !== (yj > pt.y)) &&
          (pt.x < (xj - xi) * (pt.y - yi) / (yj - yi) + xi)) inside = !inside;
    }
    return inside;
  }

  function syncAngleUI() {
    const deg = state.pose.angle / DEG;
    ui.angleVal.textContent = deg.toFixed(0) + '°';
    ui.hAngle.value = Math.max(-95, Math.min(20, deg));
  }

  function render() {
    const env = state.env;
    const report = Model.collisionReport(state.pose, state.dims, env);
    renderer.clear();
    renderer.drawGrid();
    renderer.drawEnvironment(env);
    renderer.drawAnnotations(state.dims, env);
    if (state.plan && state.plan.path) renderer.drawGhostPath(state.plan.path, state.dims);
    renderer.drawHits(state.pose, state.dims, env, report.hits);
    renderer.drawWasher(state.pose, state.dims, report.collides);

    ui.status.className = 'status ' + (report.collides ? 'collision' : 'clear');
    ui.status.textContent = report.collides
      ? 'Collision: ' + report.hits.join(' + ')
      : 'Clear';
    syncAngleUI();
  }

  function reframe() { renderer.resize(); renderer.setView(state.dims, state.bayDepth, state.bayStart); }

  function invalidatePlan() {
    state.plan = null;
    ui.btnPlay.disabled = true;
    ui.timeline.disabled = true;
    ui.timeline.value = 0;
    ui.result.innerHTML = '';
    stopAnim();
  }

  // ---- pose presets ----
  function goStart() { state.pose = Model.startPose(state.dims, state.bayStart); render(); }
  function goGoal() { state.pose = Model.goalPose(state.dims, state.bayDepth); render(); }

  // ---- parameter inputs ----
  function bindPair(range, number, apply) {
    function fromRange() { number.value = range.value; apply(parseFloat(range.value)); }
    function fromNumber() {
      let v = parseFloat(number.value); if (isNaN(v)) return;
      range.value = v; apply(v);
    }
    range.addEventListener('input', fromRange);
    number.addEventListener('input', fromNumber);
  }

  bindPair(ui.hHeight, ui.nHeight, (v) => {
    state.dims.height = v; rebuildEnv(); reframe(); invalidatePlan(); render();
  });
  bindPair(ui.hDepth, ui.nDepth, (v) => {
    state.dims.depth = v; rebuildEnv(); reframe(); invalidatePlan(); render();
  });
  bindPair(ui.hBay, ui.nBay, (v) => {
    state.bayDepth = v; rebuildEnv(); reframe(); invalidatePlan(); render();
  });
  bindPair(ui.hBayStart, ui.nBayStart, (v) => {
    state.bayStart = v; rebuildEnv(); reframe(); invalidatePlan(); render();
  });

  ui.hAngle.addEventListener('input', () => {
    state.pose.angle = parseFloat(ui.hAngle.value) * DEG; render();
  });

  ui.btnStart.addEventListener('click', goStart);
  ui.btnGoal.addEventListener('click', goGoal);
  ui.btnReset.addEventListener('click', () => {
    state.dims = { depth: 60, height: 95 };
    state.bayDepth = 20;
    state.bayStart = 68;
    ui.nHeight.value = ui.hHeight.value = 95;
    ui.nDepth.value = ui.hDepth.value = 60;
    ui.nBay.value = ui.hBay.value = 20;
    ui.nBayStart.value = ui.hBayStart.value = 68;
    rebuildEnv(); reframe(); invalidatePlan(); goStart();
  });

  // ---- dragging ----
  function evtWorld(e) {
    const r = canvas.getBoundingClientRect();
    return renderer.toWorld(e.clientX - r.left, e.clientY - r.top);
  }
  canvas.addEventListener('pointerdown', (e) => {
    const w = evtWorld(e);
    if (pointInPoly(w, Model.washerPoly(state.pose, state.dims))) {
      state.drag.active = true;
      state.drag.ox = state.pose.x - w.x;
      state.drag.oy = state.pose.y - w.y;
      canvas.classList.add('grabbing');
      canvas.setPointerCapture(e.pointerId);
      stopAnim();
    }
  });
  canvas.addEventListener('pointermove', (e) => {
    if (!state.drag.active) return;
    const w = evtWorld(e);
    state.pose.x = w.x + state.drag.ox;
    state.pose.y = w.y + state.drag.oy;
    render();
  });
  function endDrag(e) {
    if (!state.drag.active) return;
    state.drag.active = false;
    canvas.classList.remove('grabbing');
    try { canvas.releasePointerCapture(e.pointerId); } catch (_) {}
  }
  canvas.addEventListener('pointerup', endDrag);
  canvas.addEventListener('pointercancel', endDrag);

  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    state.pose.angle += (e.deltaY > 0 ? 1 : -1) * 2 * DEG;
    render();
  }, { passive: false });

  window.addEventListener('keydown', (e) => {
    const tag = (document.activeElement && document.activeElement.tagName) || '';
    if (tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA') return;
    const big = e.shiftKey ? 5 : 1;
    let handled = true;
    switch (e.key) {
      case 'q': case 'Q': state.pose.angle -= 2 * DEG; break;
      case 'e': case 'E': state.pose.angle += 2 * DEG; break;
      case 'ArrowLeft': state.pose.x -= big; break;
      case 'ArrowRight': state.pose.x += big; break;
      case 'ArrowUp': state.pose.y += big; break;
      case 'ArrowDown': state.pose.y -= big; break;
      default: handled = false;
    }
    if (handled) { e.preventDefault(); render(); }
  });

  // ---- solve ----
  ui.btnSolve.addEventListener('click', () => {
    ui.solving.classList.remove('hidden');
    ui.btnSolve.disabled = true;
    // Defer so the overlay paints before the (synchronous) search runs.
    setTimeout(() => {
      const opts = Object.assign({ refine: true }, RES[ui.res.value] || RES.medium);
      const t0 = performance.now();
      let result;
      try {
        result = Solver.findMinBayDepth(state.dims, state.bayStart, opts);
      } finally {
        ui.solving.classList.add('hidden');
        ui.btnSolve.disabled = false;
      }
      const ms = Math.round(performance.now() - t0);
      showResult(result, ms);
    }, 30);
  });

  function showResult(result, ms) {
    if (!result.feasible || result.depth == null) {
      state.plan = null;
      ui.result.innerHTML =
        '<div class="small">No feasible insertion found within the search range. ' +
        'The washer may be too deep to maneuver under the counter.</div>';
      ui.btnPlay.disabled = true; ui.timeline.disabled = true;
      return;
    }
    const depth = Math.round(result.depth * 10) / 10;
    state.bayDepth = depth;
    ui.nBay.value = depth;
    ui.hBay.value = Math.min(parseFloat(ui.hBay.max), depth);
    rebuildEnv(); reframe();
    state.plan = result;
    state.pose = result.path[0];

    ui.btnPlay.disabled = false;
    ui.timeline.disabled = false;
    ui.timeline.value = 0;

    ui.result.innerHTML =
      '<div><span class="big">' + depth.toFixed(1) + ' cm</span> minimum bay depth</div>' +
      '<table>' +
      row('Static fit floor', result.staticMin.toFixed(1) + ' cm') +
      row('Clearance to maneuver', (depth - result.staticMin).toFixed(1) + ' cm') +
      row('Washer', state.dims.height + ' × ' + state.dims.depth + ' cm') +
      row('Bay start', state.bayStart + ' cm') +
      row('Search time', ms + ' ms') +
      '</table>' +
      '<div class="small" style="margin-top:6px">Press <b>Play insertion</b> or drag the ' +
      'timeline to watch it go in.</div>' +
      (result.validated
        ? '<div class="small" style="margin-top:4px;opacity:.8">✓ Insertion path is ' +
          'collision-validated (dense sub-step checks). Still leave a small real-world ' +
          'margin for hoses, feet and trim.</div>'
        : '<div class="small" style="margin-top:4px;opacity:.8">Approximate (path not fully ' +
          'validated) — add a safety margin.</div>');
    render();
  }
  function row(a, b) { return '<tr><td>' + a + '</td><td>' + b + '</td></tr>'; }

  // ---- animation / timeline ----
  function poseAtParam(p) {
    const path = state.plan.path;
    const f = Math.max(0, Math.min(1, p)) * (path.length - 1);
    const i = Math.floor(f), t = f - i;
    const a = path[i], b = path[Math.min(path.length - 1, i + 1)];
    return {
      x: a.x + (b.x - a.x) * t,
      y: a.y + (b.y - a.y) * t,
      angle: a.angle + (b.angle - a.angle) * t,
    };
  }

  ui.timeline.addEventListener('input', () => {
    if (!state.plan) return;
    stopAnim();
    state.anim.p = parseFloat(ui.timeline.value) / 1000;
    state.pose = poseAtParam(state.anim.p);
    render();
  });

  function stopAnim() {
    if (state.anim.raf) cancelAnimationFrame(state.anim.raf);
    state.anim.raf = 0; state.anim.playing = false;
    ui.btnPlay.textContent = 'Play insertion';
  }

  ui.btnPlay.addEventListener('click', () => {
    if (!state.plan) return;
    if (state.anim.playing) { stopAnim(); return; }
    state.anim.playing = true;
    ui.btnPlay.textContent = 'Pause';
    if (state.anim.p >= 1) state.anim.p = 0;
    state.anim.last = performance.now();
    const DURATION = 4500; // ms for a full insertion
    function frame(now) {
      const dt = now - state.anim.last; state.anim.last = now;
      state.anim.p += dt / DURATION;
      if (state.anim.p >= 1) { state.anim.p = 1; }
      ui.timeline.value = Math.round(state.anim.p * 1000);
      state.pose = poseAtParam(state.anim.p);
      render();
      if (state.anim.p >= 1) { stopAnim(); return; }
      state.anim.raf = requestAnimationFrame(frame);
    }
    state.anim.raf = requestAnimationFrame(frame);
  });

  // ---- boot ----
  window.addEventListener('resize', () => { reframe(); render(); });

  rebuildEnv();
  reframe();
  goStart();
})();
