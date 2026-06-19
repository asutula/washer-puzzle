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
    maxNodes: 4000000,
  };

  /** Build the discretised configuration grid for one (dims, depth). */
  function makeGrid(dims, depth, opts) {
    const o = Object.assign({}, DEFAULTS, opts);

    // Snap angle bounds so that 0 is always on the grid.
    const daDeg = o.daDeg;
    const aMinDeg = -Math.floor((-o.angleMinDeg) / daDeg) * daDeg;
    const aMaxDeg = Math.ceil(o.angleMaxDeg / daDeg) * daDeg;
    const da = daDeg * DEG;
    const aMin = aMinDeg * DEG;
    const na = Math.round((aMaxDeg - aMinDeg) / daDeg) + 1;

    const xMin = Model.FRONT_EDGE_X - dims.depth - 15;
    const xMax = Model.STEP_X + 35 + dims.depth;
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
  function planAtDepth(dims, depth, opts) {
    const env = Model.buildEnvironment(depth);
    const g = makeGrid(dims, depth, opts);
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

    const start = anchor(Model.startPose(dims));
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

    while (head < tail && !found) {
      const cur = queue[head++];
      stats.nodes++;
      const k = cur % g.na;
      const j = ((cur - k) / g.na) % g.ny;
      const i = ((cur - k) / g.na - j) / g.ny;

      // 6-connected neighbours
      const nb = [
        [i - 1, j, k], [i + 1, j, k],
        [i, j - 1, k], [i, j + 1, k],
        [i, j, k - 1], [i, j, k + 1],
      ];
      for (let n = 0; n < 6; n++) {
        const ni = nb[n][0], nj = nb[n][1], nk = nb[n][2];
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
    path.unshift(Model.startPose(dims));
    path.push(Model.goalPose(dims, depth));
    return { feasible: true, path, stats, env };
  }

  function feasible(dims, depth, opts) {
    return planAtDepth(dims, depth, opts).feasible;
  }

  /**
   * Find the shallowest bay depth that admits an insertion path.
   * @returns {{depth:number|null, feasible:boolean, staticMin:number,
   *            path:Array<pose>, iterations:number, tolerance:number}}
   */
  function findMinBayDepth(dims, opts) {
    const o = Object.assign({ tolerance: 0.5 }, opts);
    const staticMin = Model.staticMinDepth(dims);

    // If the washer fits upright under the counter with no bay, it can simply be
    // slid straight in: no bay required.
    if (staticMin <= 0 && feasible(dims, 0, o)) {
      const plan = planAtDepth(dims, 0, o);
      return { depth: 0, feasible: true, staticMin, path: plan.path, iterations: 1, tolerance: o.tolerance };
    }

    // Establish a feasible upper bound.
    let hi = staticMin + Math.max(12, dims.height * 0.5);
    const hardCap = dims.height + 90;
    let guard = 0;
    while (!feasible(dims, hi, o) && hi < hardCap && guard++ < 12) {
      hi += Math.max(12, dims.height * 0.5);
    }
    if (!feasible(dims, hi, o)) {
      return { depth: null, feasible: false, staticMin, path: [], iterations: guard, tolerance: o.tolerance };
    }

    // lo is infeasible (at/below the static fit there is no room to rotate).
    let lo = staticMin;
    let iterations = 0;
    let bestPlan = planAtDepth(dims, hi, o);
    while (hi - lo > o.tolerance) {
      const mid = (lo + hi) / 2;
      iterations++;
      const plan = planAtDepth(dims, mid, o);
      if (plan.feasible) { hi = mid; bestPlan = plan; }
      else { lo = mid; }
    }

    return {
      depth: hi,
      feasible: true,
      staticMin,
      path: bestPlan.path,
      iterations,
      tolerance: o.tolerance,
    };
  }

  return { makeGrid, planAtDepth, feasible, findMinBayDepth, DEFAULTS };
});
