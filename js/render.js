/*
 * render.js — draws the world onto a 2D canvas.
 *
 * Holds a world<->screen transform (y is flipped so that up is up) and a set of
 * drawing routines for the solids, the washer, dimension annotations and grid.
 */
(function (root) {
  'use strict';
  const Model = root.Model;

  const COLORS = {
    bg: '#f4f1ea',
    grid: '#e6e0d4',
    counter: '#9aa0a6',
    counterEdge: '#5f6368',
    floor: '#bdb5a6',
    floorEdge: '#6b6354',
    hatch: '#a89f8d',
    washer: 'rgba(45, 125, 210, 0.78)',
    washerEdge: '#1a5fa3',
    washerBad: 'rgba(214, 65, 65, 0.80)',
    washerBadEdge: '#a31515',
    hit: '#d64141',
    dim: '#3b3b3b',
    datum: '#7a8aa0',
    label: '#2a2a2a',
    pathGhost: 'rgba(45, 125, 210, 0.10)',
    pathGhostEdge: 'rgba(45, 125, 210, 0.30)',
  };

  function Renderer(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.t = { scale: 1, ox: 0, oy: 0 };
    this.view = { minX: -90, maxX: 200, minY: -60, maxY: 130 };
    this.pad = 46;
  }

  Renderer.prototype.resize = function () {
    const dpr = window.devicePixelRatio || 1;
    const rect = this.canvas.getBoundingClientRect();
    this.canvas.width = Math.max(1, Math.round(rect.width * dpr));
    this.canvas.height = Math.max(1, Math.round(rect.height * dpr));
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.cssW = rect.width;
    this.cssH = rect.height;
  };

  Renderer.prototype.setView = function (dims, bayDepth) {
    // Frame the relevant region with a little breathing room.
    this.view = {
      minX: -90,
      maxX: Model.STEP_X + 45 + dims.depth,
      minY: -(bayDepth + 22),
      maxY: Math.max(Model.COUNTER_BOTTOM + 45, dims.height + 25),
    };
    this._fit();
  };

  Renderer.prototype._fit = function () {
    const v = this.view, pad = this.pad;
    const availW = this.cssW - 2 * pad;
    const availH = this.cssH - 2 * pad;
    const vw = v.maxX - v.minX, vh = v.maxY - v.minY;
    const scale = Math.min(availW / vw, availH / vh);
    const contentW = vw * scale, contentH = vh * scale;
    this.t.scale = scale;
    this.t.ox = pad + (availW - contentW) / 2 - v.minX * scale;
    this.t.oy = pad + (availH - contentH) / 2 + v.maxY * scale;
  };

  Renderer.prototype.toScreen = function (wx, wy) {
    return { x: this.t.ox + wx * this.t.scale, y: this.t.oy - wy * this.t.scale };
  };
  Renderer.prototype.toWorld = function (sx, sy) {
    return { x: (sx - this.t.ox) / this.t.scale, y: (this.t.oy - sy) / this.t.scale };
  };

  Renderer.prototype._poly = function (poly, fill, stroke, lw) {
    const ctx = this.ctx;
    ctx.beginPath();
    for (let i = 0; i < poly.length; i++) {
      const p = this.toScreen(poly[i].x, poly[i].y);
      if (i === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
    }
    ctx.closePath();
    if (fill) { ctx.fillStyle = fill; ctx.fill(); }
    if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = lw || 1.5; ctx.stroke(); }
  };

  Renderer.prototype._line = function (a, b, color, lw, dash) {
    const ctx = this.ctx;
    const p = this.toScreen(a.x, a.y), q = this.toScreen(b.x, b.y);
    ctx.beginPath();
    ctx.setLineDash(dash || []);
    ctx.moveTo(p.x, p.y); ctx.lineTo(q.x, q.y);
    ctx.strokeStyle = color; ctx.lineWidth = lw || 1; ctx.stroke();
    ctx.setLineDash([]);
  };

  Renderer.prototype._text = function (wx, wy, str, opts) {
    const ctx = this.ctx;
    opts = opts || {};
    const p = this.toScreen(wx, wy);
    ctx.font = opts.font || '13px system-ui, sans-serif';
    ctx.fillStyle = opts.color || COLORS.label;
    ctx.textAlign = opts.align || 'left';
    ctx.textBaseline = opts.baseline || 'alphabetic';
    if (opts.bg) {
      const w = ctx.measureText(str).width;
      const padx = 4, h = 16;
      let bx = p.x;
      if (opts.align === 'center') bx = p.x - w / 2;
      else if (opts.align === 'right') bx = p.x - w;
      ctx.fillStyle = opts.bg;
      ctx.fillRect(bx - padx, p.y - 13, w + 2 * padx, h);
      ctx.fillStyle = opts.color || COLORS.label;
    }
    ctx.fillText(str, p.x + (opts.dx || 0), p.y + (opts.dy || 0));
  };

  Renderer.prototype._hatch = function (poly, color) {
    // Diagonal hatching clipped to a polygon, to read as "solid material".
    const ctx = this.ctx;
    ctx.save();
    ctx.beginPath();
    for (let i = 0; i < poly.length; i++) {
      const p = this.toScreen(poly[i].x, poly[i].y);
      if (i === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
    }
    ctx.closePath();
    ctx.clip();
    const b = poly.reduce((acc, pt) => {
      const s = this.toScreen(pt.x, pt.y);
      return {
        minX: Math.min(acc.minX, s.x), minY: Math.min(acc.minY, s.y),
        maxX: Math.max(acc.maxX, s.x), maxY: Math.max(acc.maxY, s.y),
      };
    }, { minX: Infinity, minY: Infinity, maxX: -Infinity, maxY: -Infinity });
    ctx.strokeStyle = color; ctx.lineWidth = 1; ctx.globalAlpha = 0.5;
    const step = 11;
    for (let x = b.minX - (b.maxY - b.minY); x < b.maxX; x += step) {
      ctx.beginPath();
      ctx.moveTo(x, b.maxY);
      ctx.lineTo(x + (b.maxY - b.minY), b.minY);
      ctx.stroke();
    }
    ctx.globalAlpha = 1;
    ctx.restore();
  };

  Renderer.prototype.clear = function () {
    this.ctx.fillStyle = COLORS.bg;
    this.ctx.fillRect(0, 0, this.cssW, this.cssH);
  };

  Renderer.prototype.drawGrid = function () {
    const v = this.view;
    const stepWorld = 20;
    const x0 = Math.ceil(v.minX / stepWorld) * stepWorld;
    for (let x = x0; x <= v.maxX; x += stepWorld)
      this._line({ x, y: v.minY }, { x, y: v.maxY }, COLORS.grid, 1);
    const y0 = Math.ceil(v.minY / stepWorld) * stepWorld;
    for (let y = y0; y <= v.maxY; y += stepWorld)
      this._line({ x: v.minX, y }, { x: v.maxX, y }, COLORS.grid, 1);
  };

  Renderer.prototype.drawEnvironment = function (env) {
    // Floor blocks (concrete) + hatch.
    for (const s of env.solids) {
      if (s.name === 'counter') continue;
      this._poly(s.poly, COLORS.floor, COLORS.floorEdge, 1.5);
      this._hatch(s.poly, COLORS.hatch);
    }
    // Counter slab.
    const counter = env.solids.find(s => s.name === 'counter');
    this._poly(counter.poly, COLORS.counter, COLORS.counterEdge, 1.8);
    this._hatch(counter.poly, '#7d838a');

    // Emphasise the two binding corners.
    this._dot(env.counterCorner, COLORS.counterEdge);
    this._dot(env.stepCorner, COLORS.floorEdge);

    this._text(Model.WX_LEFT + 30, Model.COUNTER_BOTTOM + Model.COUNTER_THICKNESS / 2, 'Counter',
      { color: '#fff', font: '13px system-ui', baseline: 'middle' });
  };

  Renderer.prototype._dot = function (pt, color) {
    const ctx = this.ctx; const p = this.toScreen(pt.x, pt.y);
    ctx.beginPath(); ctx.arc(p.x, p.y, 3.2, 0, Math.PI * 2);
    ctx.fillStyle = color; ctx.fill();
  };

  Renderer.prototype.drawAnnotations = function (dims, env) {
    const C = Model.COUNTER_BOTTOM, S = Model.STEP_X, B = env.bayDepth;

    // datum line at standard floor elevation
    this._line({ x: S, y: 0 }, { x: this.view.maxX, y: 0 }, COLORS.datum, 1, [5, 4]);
    this._line({ x: -90, y: 0 }, { x: S, y: 0 }, COLORS.datum, 1, [2, 5]);

    // 90 cm: counter bottom above standard floor
    this._dimV(-30, 0, C, '90', 'left');
    // 68 cm: step distance from counter front edge
    this._dimH(0, S, 9, '68');
    // bay depth
    if (B > 0.05) this._dimV(40, 0, -B, 'bay ' + B.toFixed(1), 'right');

    this._text(S + 8, 0, 'standard floor', { dy: 16, color: COLORS.dim, font: '12px system-ui' });
    this._text(-85, -B, 'bay floor', { dy: 16, color: COLORS.dim, font: '12px system-ui' });
  };

  // vertical dimension with ticks + label, drawn at world x
  Renderer.prototype._dimV = function (x, y0, y1, label, side) {
    this._line({ x, y: y0 }, { x, y: y1 }, COLORS.dim, 1.2);
    this._tick({ x, y: y0 }, true); this._tick({ x, y: y1 }, true);
    this._text(x, (y0 + y1) / 2, label, {
      align: side === 'right' ? 'left' : 'right',
      dx: side === 'right' ? 6 : -6, dy: 4, color: COLORS.dim,
      font: '12px system-ui', bg: COLORS.bg,
    });
  };
  Renderer.prototype._dimH = function (x0, x1, y, label) {
    this._line({ x: x0, y }, { x: x1, y }, COLORS.dim, 1.2);
    this._tick({ x: x0, y }, false); this._tick({ x: x1, y }, false);
    this._text((x0 + x1) / 2, y, label, {
      align: 'center', dy: -5, color: COLORS.dim, font: '12px system-ui', bg: COLORS.bg,
    });
  };
  Renderer.prototype._tick = function (pt, vertical) {
    const ctx = this.ctx; const p = this.toScreen(pt.x, pt.y); const r = 4;
    ctx.beginPath(); ctx.strokeStyle = COLORS.dim; ctx.lineWidth = 1.2;
    if (vertical) { ctx.moveTo(p.x - r, p.y); ctx.lineTo(p.x + r, p.y); }
    else { ctx.moveTo(p.x, p.y - r); ctx.lineTo(p.x, p.y + r); }
    ctx.stroke();
  };

  Renderer.prototype.drawWasher = function (pose, dims, colliding) {
    const poly = Model.washerPoly(pose, dims);
    this._poly(poly, colliding ? COLORS.washerBad : COLORS.washer,
      colliding ? COLORS.washerBadEdge : COLORS.washerEdge, 2);

    // a little porthole near the "front" (the +depth/2 side) to show orientation
    const c = Math.cos(pose.angle), s = Math.sin(pose.angle);
    const fx = pose.x + (dims.depth * 0.28) * c;
    const fy = pose.y + (dims.depth * 0.28) * s;
    const p = this.toScreen(fx, fy);
    const ctx = this.ctx;
    ctx.beginPath();
    ctx.arc(p.x, p.y, Math.max(5, dims.height * 0.16 * this.t.scale), 0, Math.PI * 2);
    ctx.strokeStyle = colliding ? COLORS.washerBadEdge : COLORS.washerEdge;
    ctx.lineWidth = 2; ctx.stroke();

    this._text(pose.x, pose.y, 'Washer', {
      align: 'center', baseline: 'middle', color: '#fff', font: 'bold 13px system-ui',
    });
  };

  Renderer.prototype.drawGhostPath = function (path, dims, step) {
    if (!path || path.length < 2) return;
    step = step || Math.max(1, Math.floor(path.length / 14));
    for (let i = 0; i < path.length; i += step) {
      this._poly(Model.washerPoly(path[i], dims), COLORS.pathGhost, COLORS.pathGhostEdge, 1);
    }
  };

  Renderer.prototype.drawHits = function (pose, dims, env, hits) {
    if (!hits || !hits.length) return;
    for (const s of env.solids) {
      if (hits.indexOf(s.name) >= 0) this._poly(s.poly, null, COLORS.hit, 3);
    }
  };

  root.Renderer = Renderer;
  root.RENDER_COLORS = COLORS;
})(typeof window !== 'undefined' ? window : this);
