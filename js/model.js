/*
 * model.js — the physical setup as geometry.
 *
 * Cross-section (side view), units = centimetres, y points UP:
 *
 *            counter slab (solid)
 *     ┌───────────────────────┐  y = COUNTER_TOP
 *     │                       │
 *     │                       │
 *     └───────────────────────┤  y = COUNTER_BOTTOM (90, fixed)
 *     x=WX_LEFT          x=0 (counter front edge)
 *
 *                                      standard floor
 *                              ┌───────────────────  y = 0 (fixed)
 *      bay floor               │
 *   ───────────────────────────┘  step at x = STEP_X (68, fixed)
 *   y = -bayDepth              x=STEP_X
 *
 * The washer lives in the open air between the counter bottom and the floor.
 */
(function (root, factory) {
  const Geo = (typeof module !== 'undefined' && module.exports)
    ? require('./geometry.js') : root.Geo;
  const api = factory(Geo);
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  if (root) root.Model = api;
})(typeof window !== 'undefined' ? window : null, function (Geo) {
  'use strict';

  // ---- Fixed dimensions (cm) -------------------------------------------------
  const COUNTER_BOTTOM = 90;   // bottom face of countertop above standard floor
  const COUNTER_THICKNESS = 10; // countertop slab thickness (visual; top face only)
  const COUNTER_TOP = COUNTER_BOTTOM + COUNTER_THICKNESS;
  const STEP_X = 68;           // step starts 68cm out from counter front edge
  const FRONT_EDGE_X = 0;      // counter front edge defines x = 0

  // ---- World extents used to give the (semi-infinite) solids finite size -----
  // Chosen generously so their far edges never become the binding constraint.
  const WX_LEFT = -120;        // far back, under the counter
  const WX_RIGHT = 320;        // far out onto the standard floor
  const WY_TOP = 220;          // above the countertop
  const WY_BOTTOM = -160;      // below the deepest bay

  /**
   * Build the solid environment for a given bay depth.
   * Returns convex polygons (for collision) plus annotated points for drawing.
   * @param {number} bayDepth depth of the bay below standard floor (>= 0)
   */
  function buildEnvironment(bayDepth) {
    const b = Math.max(0, bayDepth);

    const counter = Geo.rect(
      (WX_LEFT + FRONT_EDGE_X) / 2, (COUNTER_BOTTOM + COUNTER_TOP) / 2,
      (FRONT_EDGE_X - WX_LEFT), (COUNTER_TOP - COUNTER_BOTTOM), 0);

    // Floor solid is an L shape -> two convex blocks.
    const bayBlock = Geo.rect(
      (WX_LEFT + STEP_X) / 2, (WY_BOTTOM + (-b)) / 2,
      (STEP_X - WX_LEFT), ((-b) - WY_BOTTOM), 0);

    const standardBlock = Geo.rect(
      (STEP_X + WX_RIGHT) / 2, (WY_BOTTOM + 0) / 2,
      (WX_RIGHT - STEP_X), (0 - WY_BOTTOM), 0);

    return {
      bayDepth: b,
      // Convex pieces the washer must not penetrate, with labels for highlight.
      solids: [
        { name: 'counter', poly: counter },
        { name: 'bay floor', poly: bayBlock },
        { name: 'standard floor', poly: standardBlock },
      ],
      // Notable points / lines for rendering & reasoning.
      counterCorner: { x: FRONT_EDGE_X, y: COUNTER_BOTTOM },
      stepCorner: { x: STEP_X, y: 0 },
      bayFloorY: -b,
      bounds: { WX_LEFT, WX_RIGHT, WY_TOP, WY_BOTTOM },
    };
  }

  /** Washer rectangle for a pose {x,y,angle} and dims {depth,height}. */
  function washerPoly(pose, dims) {
    return Geo.rect(pose.x, pose.y, dims.depth, dims.height, pose.angle);
  }

  /**
   * Collision report for a washer pose against an environment.
   * @returns {{collides:boolean, hits:string[]}}
   */
  function collisionReport(pose, dims, env) {
    const w = washerPoly(pose, dims);
    const hits = [];
    for (let i = 0; i < env.solids.length; i++) {
      const s = env.solids[i];
      if (Geo.bboxOverlap(w, s.poly) && Geo.convexIntersect(w, s.poly)) {
        hits.push(s.name);
      }
    }
    return { collides: hits.length > 0, hits };
  }

  function collides(pose, dims, env) {
    const w = washerPoly(pose, dims);
    for (let i = 0; i < env.solids.length; i++) {
      const s = env.solids[i];
      if (Geo.bboxOverlap(w, s.poly) && Geo.convexIntersect(w, s.poly)) {
        return true;
      }
    }
    return false;
  }

  /**
   * A clear, upright starting pose: standing on the standard floor, a little to
   * the right of the step.
   */
  function startPose(dims) {
    return {
      x: STEP_X + 25 + dims.depth / 2,
      y: dims.height / 2,
      angle: 0,
    };
  }

  /**
   * Installed (goal) pose: upright, resting on the bay floor, front face flush
   * with the counter front edge (fully tucked under the counter).
   */
  function goalPose(dims, bayDepth) {
    const b = Math.max(0, bayDepth);
    return {
      x: FRONT_EDGE_X - dims.depth / 2,
      y: -b + dims.height / 2,
      angle: 0,
    };
  }

  /** Static lower bound on bay depth: the washer must at least fit upright. */
  function staticMinDepth(dims) {
    return Math.max(0, dims.height - COUNTER_BOTTOM);
  }

  return {
    COUNTER_BOTTOM, COUNTER_THICKNESS, COUNTER_TOP, STEP_X, FRONT_EDGE_X,
    WX_LEFT, WX_RIGHT, WY_TOP, WY_BOTTOM,
    buildEnvironment, washerPoly, collisionReport, collides,
    startPose, goalPose, staticMinDepth,
  };
});
