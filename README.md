# Washer Bay Puzzle

An interactive 2D model for a real installation problem: a washing machine has to
go **under a counter**, and because it's taller than the gap under the counter, the
concrete floor is dropped into a **bay** so the machine can rotate down into place.

The goal is to find the **shallowest bay depth** that still lets a given washer be
maneuvered into its installed position — digging concrete is expensive, so you want
the least depth that works.

Open **`index.html`** in a browser. No build step, no dependencies.

```
# either just open the file, or serve it (some browsers prefer http://):
python3 -m http.server 8000   # then visit http://localhost:8000
```

## The model

Everything is a side-view cross-section, units in **centimetres**, with **y pointing up**.

```
        counter slab (solid)
   ┌─────────────────────────┐
   │                         │
   └─────────────────────────┤  y = 90  ← counter bottom (FIXED)
   x = back              x = 0  ← counter front edge
                                              standard floor
                                     ┌──────────────────────  y = 0  (FIXED)
        bay floor                    │
   ──────────────────────────────────┘  step, bayStart cm out
   y = −bayDepth                     x = bayStart
```

| Quantity | Value | |
|---|---|---|
| Counter bottom above standard floor | **90 cm** | fixed |
| Standard floor elevation | **0 cm** | fixed (datum) |
| Bay depth | configurable | the thing we optimize |
| Bay start (step distance from counter edge) | configurable, default **68 cm** | also sets the front "trench" width |
| Washer height `H` | configurable | |
| Washer depth `D` | configurable | |

- The **environment** (counter + standard floor + bay floor) is modeled as solid
  rectangles. The L-shaped floor is split into two convex blocks so collision math
  stays simple.
- The **washer** is a rigid rectangle (`D × H`) with a pose `(x, y, angle)`.
- **Collision** is exact via the Separating Axis Theorem (`js/geometry.js`). Flush
  contact (resting on a floor, sliding along a wall) is allowed; overlap is not.

## Finding the optimal depth

"Can this washer be installed at bay depth *B*?" is a motion-planning question: does a
**collision-free path** exist for the washer's pose from the upright **start** (standing
on the standard floor) to the upright **installed** pose (in the bay, tucked under the
counter)?

- `js/solver.js` answers it with a **breadth-first search over a discretized grid of
  poses** (`x`, `y`, `angle`) — the classic configuration-space approach.
- The search is **26-connected**: it can move and rotate *at the same time*, which is how
  you actually pivot a unit under a lip. (A simpler axis-at-a-time search has to
  "staircase" that motion and over-reports the depth needed for `H > 90`.)
- Deeper bays only ever add clearance, so feasibility is **monotonic** in depth. The
  optimum is found by **binary search** on the depth, calling the planner at each step.
- For accuracy the answer is **refined at a fine 1 cm / 1° grid** regardless of the
  selected search speed (the speed setting only controls how fast the threshold is first
  bracketed).

Two reference numbers help interpret the result:

- **Static floor `= max(0, H − 90)`** — the washer must at least *fit* upright under the
  counter once installed. No bay can be shallower than this.
- **Solved depth** — the shallowest depth at which an actual *path* exists. Always ≥ the
  static floor; the gap is the room needed to maneuver it in.

### A result worth knowing

Sweeping washer sizes (at the default **68 cm** bay start) shows a sharp regime change at
**D = bay start** — i.e. the width of the open "trench" in front of the counter:

```
H\D        50     60     68     76     84      (min bay depth, cm)
 88       0.0    0.0    0.0    0.0    0.0
 95       5.6    5.6    5.6    6.9    7.4
100      10.2   10.2   10.2   13.7   15.5
108      18.2   18.2   18.2   25.2   33.2
```

- A washer **as deep as the trench or shallower** can be lowered straight into the open
  area in front of the counter and slid under — bay depth ≈ the static floor.
- A washer **deeper than the trench** can't be lowered upright; it must be tilted and
  rotated under the counter's front corner, which demands a noticeably deeper bay.

Because the trench width *is* the bay-start distance, widening the bay start moves that
threshold out — e.g. an 80 × 100 cm washer needs ~39 cm of depth at a 30 cm bay start,
~14 cm at the default 68 cm, and only ~10 cm (its static floor) once the bay start is
120 cm. Try it with the slider.

If `H ≤ 90`, the washer fits under the counter with no bay at all.

## Controls

- **Sliders / number boxes** — washer height, washer depth, bay depth, bay start.
- **Drag** the washer to move it; **scroll** or **`Q` / `E`** to rotate; **arrow keys** to
  nudge (hold **Shift** for bigger steps).
- **Start pose / Installed pose** — jump the washer to the two reference poses.
- **Find min bay depth** — runs the optimizer; sets the bay to the result and stores the
  insertion path.
- **Play insertion** / **timeline** — animate or scrub the washer along the solved path.
- Live **collision** readout turns red and highlights the offending solid on contact.

## Limitations

- It's a **2D cross-section**. It ignores the washer's left–right width and any 3D
  wiggle. For a straight in-and-down installation that's the binding plane, but a real
  install has more freedom.
- The planner works on a **grid**. Even at the fine refine resolution the reported depth
  is only good to roughly **±1 cm** — pushing finer just trades a lot of time for jitter,
  not real precision. Treat the number as the *idealized* minimum and **add a safety
  margin** for a real install (a rigid box with 0 cm of slack does not actually slide in).
- The model assumes the unit is **rigid** and the bay/counter are exact; it doesn't
  account for hoses, feet, trim, or having to lift over the front lip.

## Project layout

```
index.html        markup + controls
css/style.css     styling
js/geometry.js    vectors, rectangles, SAT collision   (pure, tested)
js/model.js       environment + washer + poses          (pure, tested)
js/solver.js      config-space planner + binary search  (pure, tested)
js/render.js      canvas drawing
js/app.js         state, interaction, animation
test/test.js      unit tests for the core logic
test/smoke.js     headless run of the browser code
```

## Tests

```
npm test          # runs test/test.js and test/smoke.js
# or:
node test/test.js
node test/smoke.js
```
