/*
 * solver.js — motion planning + bay-depth optimisation.
 *
 * The washer has a 3-DOF pose (x, y, angle). "Can it be installed?" is the
 * question of whether a collision-free PATH exists from the upright start pose
 * (on the standard floor) to the upright installed pose (in the bay, under the
 * counter). We answer it with a breadth-first search over a discretised grid of
 * poses (the classic configuration-space approach).
 *
 * The OPTIMAL (shallowest) bay depth is then found by binary search: deeper bays
 * only ever add clearance, so feasibility is monotonic in depth.
 */
(function (root, factory) {
  const isNode = typeof module !== 'undefined' && module.exports;
  const Geo = isNode ? require('./geometry.js') : root.Geo;
  const Model = isNode ? require('./model.js') : root.Model;
  const api = factory(Geo, Model);
  if (isNode) module.exports = api;
  if (root) root.Solver = api;
})(typeof window !== 'undefined' ? window : null, function (Geo, Model) {
  'use strict';

  const DEG = Math.PI / 180;

  const DEFAULTS = {
    dx: 3,          // cm   grid spacing in x
    dy: 3,          // cm   grid spacing in y
    daDeg: 3,       // deg  grid spacing in angle
    angleMinDeg: -93, // tilt top toward the open side (clockwise) ...
    angleMaxDeg: 15,  // ... with a little slack the other way
    maxNodes: 9000000,
    conn: 26,       // configuration-space connectivity (6, 18 or 26)
  };

  // Neighbour offsets in (i,j,k). 26-connectivity lets the search make coupled
  // rotate+translate moves — the actual insertion motion — instead of staircasing
  // one axis at a time, which over-reports the bay depth needed for H > 90.
  // Collision results are cached per pose, so extra neighbours add almost no cost.
  function buildOffsets(conn) {
    const out = [];
    for (let di = -1; di <= 1; di++)
      for (let dj = -1; dj <= 1; dj++)
        for (let dk = -1; dk <= 1; dk++) {
          const m = Math.abs(di) + Math.abs(dj) + Math.abs(dk);
          if (m === 0) continue;            // skip self
          if (conn === 6 && m !== 1) continue;
          if (conn === 18 && m === 3) continue;
          out.push([di, dj, dk]);
        }
    return out;
  }
  const OFFSETS = { 6: buildOffsets(6), 18: buildOffsets(18), 26: buildOffsets(26) };

  /** Build the discretised configuration grid for one (dims, depth, bayStart). */
  function makeGrid(dims, depth, bayStart, opts) {
    const o = Object.assign({}, DEFAULTS, opts);
    const s = (bayStart == null) ? Model.DEFAULT_BAY_START : bayStart;

    // Snap angle bounds so that 0 is always on the grid.
    const daDeg = o.daDeg;
    const aMinDeg = -Math.floor((-o.angleMinDeg) / daDeg) * daDeg;
    const aMaxDeg = Math.ceil(o.angleMaxDeg / daDeg) * daDeg;
    const da = daDeg * DEG;
    const aMin = aMinDeg * DEG;
    const na = Math.round((aMaxDeg - aMinDeg) / daDeg) + 1;

    const xMin = Model.FRONT_EDGE_X - dims.depth - 15;
    const xMax = s + 35 + dims.depth;
    const yMin = -depth - 10;
    const yMax = Math.max(dims.height, Model.COUNTER_BOTTOM) + 25;

    let dx = o.dx, dy = o.dy;
    let nx = Math.round((xMax - xMin) / dx) + 1;
    let ny = Math.round((yMax - yMin) / dy) + 1;

    // Coarsen uniformly if the grid would be too large.
    while (nx * ny * na > o.maxNodes) {
      dx *= 1.5; dy *= 1.5;
      nx = Math.round((xMax - xMin) / dx) + 1;
      ny = Math.round((yMax - yMin) / dy) + 1;
    }

    return { dx, dy, da, xMin, yMin, aMin, nx, ny, na, daDeg, aMinDeg };
  }

  function poseAt(g, i, j, k) {
    return { x: g.xMin + i * g.dx, y: g.yMin + j * g.dy, angle: g.aMin + k * g.da };
  }
  function clamp(v, lo, hi) { return v < lo ? lo : (v > hi ? hi : v); }
  function snap(g, pose) {
    return {
      i: clamp(Math.round((pose.x - g.xMin) / g.dx), 0, g.nx - 1),
      j: clamp(Math.round((pose.y - g.yMin) / g.dy), 0, g.ny - 1),
      k: clamp(Math.round((pose.angle - g.aMin) / g.da), 0, g.na - 1),
    };
  }

  /**
   * Plan an insertion path at a fixed bay depth.
   * @returns {{feasible:boolean, path:Array<pose>, stats:object, env:object}}
   */
  function planAtDepth(dims, depth, bayStart, opts) {
    const env = Model.buildEnvironment(depth, bayStart);
    const g = makeGrid(dims, depth, bayStart, opts);
    const N = g.nx * g.ny * g.na;
    const idx = (i, j, k) => (i * g.ny + j) * g.na + k;

    // free cache: 0 unknown, 1 free, 2 blocked
    const free = new Uint8Array(N);
    function isFree(i, j, k) {
      const id = idx(i, j, k);
      let v = free[id];
      if (v === 0) {
        v = Model.collides(poseAt(g, i, j, k), dims, env) ? 2 : 1;
        free[id] = v;
      }
      return v === 1;
    }

    // Snap start & goal; if the snapped cell is blocked, hunt for a nearby free
    // cell (small box search) so threshold cases still anchor correctly.
    function anchor(pose) {
      const s = snap(g, pose);
      if (isFree(s.i, s.j, s.k)) return s;
      for (let r = 1; r <= 4; r++) {
        for (let di = -r; di <= r; di++)
          for (let dj = -r; dj <= r; dj++)
            for (let dk = -r; dk <= r; dk++) {
              const i = s.i + di, j = s.j + dj, k = s.k + dk;
              if (i < 0 || j < 0 || k < 0 || i >= g.nx || j >= g.ny || k >= g.na) continue;
              if (isFree(i, j, k)) return { i, j, k };
            }
      }
      return null;
    }

    const start = anchor(Model.startPose(dims, bayStart));
    const goal = anchor(Model.goalPose(dims, depth));
    const stats = { nodes: 0, gridSize: N, grid: g };
    if (!start || !goal) {
      return { feasible: false, path: [], stats, env };
    }

    const startId = idx(start.i, start.j, start.k);
    const goalId = idx(goal.i, goal.j, goal.k);

    const visited = new Uint8Array(N);
    const parent = new Int32Array(N).fill(-1);
    const queue = new Int32Array(N);
    let head = 0, tail = 0;

    visited[startId] = 1;
    queue[tail++] = startId;
    let found = startId === goalId;

    const OFF = OFFSETS[(opts && opts.conn) || DEFAULTS.conn] || OFFSETS[26];

    while (head < tail && !found) {
      const cur = queue[head++];
      stats.nodes++;
      const k = cur % g.na;
      const j = ((cur - k) / g.na) % g.ny;
      const i = ((cur - k) / g.na - j) / g.ny;

      for (let n = 0; n < OFF.length; n++) {
        const ni = i + OFF[n][0], nj = j + OFF[n][1], nk = k + OFF[n][2];
        if (ni < 0 || nj < 0 || nk < 0 || ni >= g.nx || nj >= g.ny || nk >= g.na) continue;
        const nId = idx(ni, nj, nk);
        if (visited[nId]) continue;
        if (!isFree(ni, nj, nk)) { visited[nId] = 1; continue; }
        visited[nId] = 1;
        parent[nId] = cur;
        if (nId === goalId) { found = true; break; }
        queue[tail++] = nId;
      }
    }

    if (!found) return { feasible: false, path: [], stats, env };

    // Reconstruct path goal -> start, then reverse.
    const path = [];
    let cur = goalId;
    while (cur !== -1) {
      const k = cur % g.na;
      const j = ((cur - k) / g.na) % g.ny;
      const i = ((cur - k) / g.na - j) / g.ny;
      path.push(poseAt(g, i, j, k));
      if (cur === startId) break;
      cur = parent[cur];
    }
    path.reverse();
    // Pin the exact upright start/goal at the ends for a clean animation.
    path.unshift(Model.startPose(dims, bayStart));
    path.push(Model.goalPose(dims, depth));
    return { feasible: true, path, stats, env };
  }

  function feasible(dims, depth, bayStart, opts) {
    return planAtDepth(dims, depth, bayStart, opts).feasible;
  }

  /**
   * Binary-search the bay-depth feasibility threshold at a fixed resolution.
   * Expands the bracket so that `lo` is infeasible and `hi` feasible, then
   * narrows to `tol`. Returns the shallowest feasible depth and its plan.
   */
  function searchThreshold(dims, bayStart, res, tol, floor, loStart, hiStart) {
    const hardCap = dims.height + 90;
    const grow = Math.max(2, tol * 4);
    let hi = hiStart, g = 0;
    while (!feasible(dims, hi, bayStart, res) && hi < hardCap && g++ < 80) hi += grow;
    if (!feasible(dims, hi, bayStart, res)) return { feasible: false, iters: g };
    let lo = Math.max(floor, loStart);
    g = 0;
    while (lo > floor && feasible(dims, lo, bayStart, res) && g++ < 80) lo -= grow;
    if (lo < floor) lo = floor;
    let plan = planAtDepth(dims, hi, bayStart, res);
    let iters = g;
    while (hi - lo > tol) {
      const mid = (lo + hi) / 2; iters++;
      const p = planAtDepth(dims, mid, bayStart, res);
      if (p.feasible) { hi = mid; plan = p; } else { lo = mid; }
    }
    return { feasible: true, depth: hi, plan, iters };
  }

  /**
   * Find the shallowest bay depth that admits an insertion path, for a given
   * (fixed) bay start distance.
   *
   * With `opts.refine` (set by the UI), the answer is always pinned at a fine
   * resolution: the selected resolution only brackets the threshold cheaply,
   * then a precise pass at 1cm/1deg nails it down. This matters most for H > 90,
   * where the result is dominated by the rotate-under-the-corner maneuver.
   *
   * @returns {{depth:number|null, feasible:boolean, staticMin:number,
   *            path:Array<pose>, iterations:number, tolerance:number}}
   */
  function findMinBayDepth(dims, bayStart, opts) {
    const o = Object.assign({ tolerance: 0.25 }, opts);
    const staticMin = Model.staticMinDepth(dims);
    const fine = { dx: 1, dy: 1, daDeg: 1, conn: o.conn || DEFAULTS.conn, maxNodes: o.maxNodes };
    const hiHint = staticMin + Math.max(12, dims.height * 0.5);

    // H <= 90: the washer slides straight in upright; no bay needed. Check at the
    // fine resolution so the tight slide-in is never missed.
    if (staticMin <= 0 && feasible(dims, 0, bayStart, fine)) {
      const plan = planAtDepth(dims, 0, bayStart, fine);
      return { depth: 0, feasible: true, staticMin, path: plan.path, iterations: 1, tolerance: o.tolerance };
    }

    const selDx = (opts && opts.dx) || DEFAULTS.dx;
    const done = (r, extraIters) => r.feasible
      ? { depth: r.depth, feasible: true, staticMin, path: r.plan.path,
          iterations: r.iters + (extraIters || 0), tolerance: o.tolerance }
      : { depth: null, feasible: false, staticMin, path: [],
          iterations: r.iters + (extraIters || 0), tolerance: o.tolerance };

    // Single precise pass when not refining, or already at fine resolution.
    if (!o.refine || selDx <= fine.dx + 1e-9) {
      const res = o.refine ? fine : o;
      return done(searchThreshold(dims, bayStart, res, o.tolerance, staticMin, staticMin, hiHint));
    }

    // Phase 1: cheap bracket at the selected resolution.
    const bracket = searchThreshold(dims, bayStart, o, 1.5, staticMin, staticMin, hiHint);
    if (!bracket.feasible) return done(bracket);

    // Phase 2: precise refine at fine resolution around the bracket.
    const refined = searchThreshold(dims, bayStart, fine, o.tolerance, staticMin,
                                    bracket.depth - 3, bracket.depth + 1);
    return done(refined.feasible ? refined : bracket, bracket.iters);
  }

  return { makeGrid, planAtDepth, feasible, searchThreshold, findMinBayDepth, DEFAULTS };
});
