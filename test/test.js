/* Plain-Node test harness for the geometry / model / solver core. */
const Geo = require('../js/geometry.js');
const Model = require('../js/model.js');
const Solver = require('../js/solver.js');

let pass = 0, fail = 0;
function ok(cond, msg) {
  if (cond) { pass++; }
  else { fail++; console.error('  ✗ FAIL: ' + msg); }
}
function near(a, b, tol, msg) { ok(Math.abs(a - b) <= (tol || 1e-9), msg + ` (got ${a}, want ~${b})`); }

console.log('— geometry —');
{
  const a = Geo.rect(0, 0, 10, 10, 0);
  const overlap = Geo.rect(5, 0, 10, 10, 0);
  const apart = Geo.rect(20, 0, 10, 10, 0);
  const flush = Geo.rect(10, 0, 10, 10, 0);
  ok(Geo.convexIntersect(a, overlap), 'overlapping rects intersect');
  ok(!Geo.convexIntersect(a, apart), 'separated rects do not intersect');
  ok(!Geo.convexIntersect(a, flush), 'flush (edge-touching) rects do not intersect');
  const diamond = Geo.rect(7, 7, 10, 10, Math.PI / 4); // rotated, corner pokes in
  ok(Geo.convexIntersect(a, diamond), 'rotated rect overlapping intersects');
  const diamondOut = Geo.rect(13, 13, 10, 10, Math.PI / 4);
  ok(!Geo.convexIntersect(a, diamondOut), 'rotated rect clear of corner does not intersect');
}

console.log('— model —');
{
  const dims = { depth: 60, height: 95 };
  const env = Model.buildEnvironment(20);

  // Upright on the standard floor, well to the right: clear.
  ok(!Model.collides(Model.startPose(dims), dims, env), 'start pose is collision-free');

  // Centre buried in the floor: collides.
  ok(Model.collides({ x: 150, y: -40, angle: 0 }, dims, env), 'washer sunk into floor collides');

  // Washer overlapping the counter slab (10cm thick, y in [90,100]): collides.
  ok(Model.collides({ x: -30, y: 95, angle: 0 }, dims, env), 'washer overlapping counter slab collides');

  // Goal with enough depth fits (H=95 needs >=5; give 20).
  ok(!Model.collides(Model.goalPose(dims, 20), dims, env), 'goal pose fits with adequate bay');

  // Goal with too-shallow bay clips the counter (needs 5, give 2).
  const shallowEnv = Model.buildEnvironment(2);
  ok(Model.collides(Model.goalPose(dims, 2), dims, shallowEnv), 'goal pose clips counter when bay too shallow');

  near(Model.staticMinDepth(dims), 5, 1e-9, 'static min depth = H - 90');
  near(Model.staticMinDepth({ depth: 60, height: 80 }), 0, 1e-9, 'static min depth clamps at 0 for short washer');
}

console.log('— solver —');
{
  // Short washer: fits under counter, no bay needed.
  const shortDims = { depth: 60, height: 80 };
  const RES = { dx: 4, dy: 4, daDeg: 4 };
  let t = Date.now();
  const r0 = Solver.findMinBayDepth(shortDims, 68, RES);
  console.log(`  short washer -> depth ${r0.depth}cm (${Date.now() - t}ms)`);
  ok(r0.feasible && r0.depth === 0, 'short washer needs no bay (depth 0)');

  // Tall washer: needs a bay deeper than the static minimum to rotate in.
  const tallDims = { depth: 60, height: 95 };
  t = Date.now();
  const r1 = Solver.findMinBayDepth(tallDims, 68, RES);
  console.log(`  tall washer  -> depth ${r1.depth}cm, staticMin ${r1.staticMin}cm, ` +
              `path ${r1.path.length} poses (${Date.now() - t}ms)`);
  ok(r1.feasible, 'tall washer has a feasible insertion');
  ok(r1.depth > r1.staticMin, 'required depth exceeds the static-fit minimum');
  ok(r1.path.length > 2, 'a non-trivial insertion path was returned');

  // Monotonicity: feasible at the solved depth, infeasible a good bit shallower.
  ok(Solver.feasible(tallDims, r1.depth + 1, 68, RES), 'feasible just above solved depth');
  ok(!Solver.feasible(tallDims, r1.staticMin, 68, RES), 'infeasible at the static minimum');

  // Bay-start effect: a deep washer (deeper than a narrow trench) needs more
  // depth than when the trench is wide enough to lower it in upright.
  const deepDims = { depth: 80, height: 100 };
  const wide = Solver.findMinBayDepth(deepDims, 120, RES);
  const narrow = Solver.findMinBayDepth(deepDims, 30, RES);
  console.log(`  deep washer  -> wide-start ${wide.depth}cm vs narrow-start ${narrow.depth}cm`);
  ok(wide.feasible && narrow.feasible, 'deep washer feasible at both bay starts');
  ok(wide.depth < narrow.depth, 'wider bay start needs a shallower bay');
}

console.log(`\n${pass} passed, ${fail} failed`);
process.exit(fail ? 1 : 0);
