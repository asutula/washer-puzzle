/*
 * geometry.js — pure 2D geometry helpers.
 *
 * Everything here is framework-free and works both in the browser (attaches to
 * window.Geo) and in Node (module.exports) so it can be unit tested.
 *
 * Conventions:
 *   - A "point" / "vector" is {x, y}.
 *   - A "polygon" is an array of points in order (CW or CCW both fine for SAT).
 *   - Angles are in radians, counter-clockwise positive, with y pointing UP.
 */
(function (root, factory) {
  const api = factory();
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  if (root) root.Geo = api;
})(typeof window !== 'undefined' ? window : null, function () {
  'use strict';

  // Tolerance (in cm) used so that flush contact (resting on a floor, sliding
  // against a wall) counts as "not colliding" rather than a hairline overlap.
  const EPS = 1e-6;

  function sub(a, b) { return { x: a.x - b.x, y: a.y - b.y }; }
  function add(a, b) { return { x: a.x + b.x, y: a.y + b.y }; }
  function dot(a, b) { return a.x * b.x + a.y * b.y; }

  /**
   * Vertices of an axis-aligned-then-rotated rectangle.
   * @param {number} cx centre x
   * @param {number} cy centre y
   * @param {number} w  full width  (x extent at angle 0)
   * @param {number} h  full height (y extent at angle 0)
   * @param {number} angle rotation in radians (CCW)
   * @returns {Array<{x:number,y:number}>} 4 vertices (CCW)
   */
  function rect(cx, cy, w, h, angle) {
    const hw = w / 2, hh = h / 2;
    const c = Math.cos(angle || 0), s = Math.sin(angle || 0);
    const corners = [
      { x: -hw, y: -hh },
      { x:  hw, y: -hh },
      { x:  hw, y:  hh },
      { x: -hw, y:  hh },
    ];
    return corners.map(function (p) {
      return { x: cx + p.x * c - p.y * s, y: cy + p.x * s + p.y * c };
    });
  }

  /** Project a polygon onto an axis, returning [min, max]. */
  function project(poly, axis) {
    let min = Infinity, max = -Infinity;
    for (let i = 0; i < poly.length; i++) {
      const d = poly[i].x * axis.x + poly[i].y * axis.y;
      if (d < min) min = d;
      if (d > max) max = d;
    }
    return [min, max];
  }

  /** Outward edge normals of a polygon (used as SAT separating axes). */
  function axes(poly) {
    const out = [];
    for (let i = 0; i < poly.length; i++) {
      const a = poly[i], b = poly[(i + 1) % poly.length];
      const ex = b.x - a.x, ey = b.y - a.y;
      const len = Math.hypot(ex, ey) || 1;
      out.push({ x: -ey / len, y: ex / len });
    }
    return out;
  }

  /**
   * Separating Axis Theorem test for two CONVEX polygons.
   * Returns true if they overlap with more than EPS penetration.
   * Flush / touching contact returns false (allowed).
   */
  function convexIntersect(a, b) {
    const all = axes(a).concat(axes(b));
    for (let i = 0; i < all.length; i++) {
      const ax = all[i];
      const pa = project(a, ax);
      const pb = project(b, ax);
      // Separated (or merely touching) on this axis -> no collision.
      if (pa[1] <= pb[0] + EPS || pb[1] <= pa[0] + EPS) return false;
    }
    return true;
  }

  /** Axis-aligned bounding box of a polygon: {minX,minY,maxX,maxY}. */
  function bbox(poly) {
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    for (let i = 0; i < poly.length; i++) {
      const p = poly[i];
      if (p.x < minX) minX = p.x;
      if (p.y < minY) minY = p.y;
      if (p.x > maxX) maxX = p.x;
      if (p.y > maxY) maxY = p.y;
    }
    return { minX, minY, maxX, maxY };
  }

  /** Cheap AABB overlap pre-check (with EPS slack). */
  function bboxOverlap(a, b) {
    const ba = bbox(a), bb = bbox(b);
    return !(ba.maxX <= bb.minX + EPS || bb.maxX <= ba.minX + EPS ||
             ba.maxY <= bb.minY + EPS || bb.maxY <= ba.minY + EPS);
  }

  return {
    EPS, sub, add, dot, rect, project, axes,
    convexIntersect, bbox, bboxOverlap,
  };
});
