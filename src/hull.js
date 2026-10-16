// Convex hulls: the smallest convex solid around a set of points.
//
// QUICKHULL (Barber, Dobkin and Huhdanpaa, 1996). Start from the biggest
// tetrahedron the points make. Every point outside it belongs to a face it is
// above. Take the point farthest above any face, find every face that point
// can see, cut them away, and close the hole with new faces from its edge
// (the horizon) to the point. The points the cut faces held go to the new
// faces, or are inside now and gone. When no face has a point above it, the
// faces are the hull. Every step is exact operations and square roots, in an
// order fixed by the input, so the same points give the same hull, to the
// bit, on every machine.
//
// FACES, NOT TRIANGLES. Quickhull makes triangles, and a cube's 8 corners
// come out as 12. The collision code wants the cube's 6 square faces: a box
// resting on another touches face to face, with four points, and two
// triangles side by side would be two faces to choose between and to flicker
// on. So triangles lying flat together are merged into one polygon, its
// corners in order round it, and points left on a straight edge or inside a
// face are dropped: the hull keeps only its corners.
//
// FRAME. A hull is built in whatever frame its points are given in. World
// moves it into its body's own frame, centred on the centre of mass and
// turned to the principal axes (shapes.js), with moved(); the solver and the
// narrowphase only ever see that.

import { FLAT } from './mesh.js';

/**
 * Points this far above a face are outside it; nearer, on it. Three times a
 * double's rounding (2^-52) of the coordinates' sizes summed: what one plane
 * test can be off by, the bound Barber et al. and Lloyd's quickhull3d use.
 * Anything smaller is the arithmetic deciding, not the points.
 */
const ROUNDING = 3 / 4503599627370496;   // 3 · 2^-52

export class Hull {
  /** Use build(); this takes its finished arrays. */
  constructor(vertices, start, loops, planes) {
    /** Corners, three numbers each. */
    this.vertices = vertices;
    /** Face f's corners are loops[start[f] .. start[f + 1]), counter-clockwise seen from outside. */
    this.start = start;
    this.loops = loops;
    /** Face f's plane: unit outward normal at 4f, and its distance d from the origin, n·x = d on it. */
    this.planes = planes;
    const F = start.length - 1;
    // Edges, each once: its two corners, and the faces to its left and right
    // seen from outside along it from the first to the second. The faces
    // either side are what decide which edge pairs can touch (collide.js).
    const owner = new Map(), edges = [], sides = [];
    const V = vertices.length / 3;
    for (let f = 0; f < F; f++) {
      for (let i = start[f]; i < start[f + 1]; i++) {
        const u = loops[i], v = loops[i + 1 < start[f + 1] ? i + 1 : start[f]];
        const back = owner.get(v * V + u);
        if (back === undefined) owner.set(u * V + v, f);
        else { edges.push(v, u); sides.push(back, f); }
      }
    }
    this.edges = Int32Array.from(edges);
    this.sides = Int32Array.from(sides);
    // Its box, in its own frame, and how far it reaches from the origin.
    const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
    let reach = 0;
    for (let i = 0; i < V; i++) {
      const x = vertices[3 * i], y = vertices[3 * i + 1], z = vertices[3 * i + 2];
      lo[0] = Math.min(lo[0], x); lo[1] = Math.min(lo[1], y); lo[2] = Math.min(lo[2], z);
      hi[0] = Math.max(hi[0], x); hi[1] = Math.max(hi[1], y); hi[2] = Math.max(hi[2], z);
      reach = Math.max(reach, Math.sqrt(x * x + y * y + z * z));
    }
    this.lo = lo;
    this.hi = hi;
    this.reach = reach;
    // Its thinnest extent: the least of its widths across each face, which
    // is what tolerances scale with, as a box's thinnest side is.
    let size = Infinity;
    for (let f = 0; f < F; f++) {
      const nx = planes[4 * f], ny = planes[4 * f + 1], nz = planes[4 * f + 2];
      let low = Infinity;
      for (let i = 0; i < V; i++) low = Math.min(low, nx * vertices[3 * i] + ny * vertices[3 * i + 1] + nz * vertices[3 * i + 2]);
      size = Math.min(size, planes[4 * f + 3] - low);
    }
    this.size = size;
  }

  get faceCount() { return this.start.length - 1; }
  get vertexCount() { return this.vertices.length / 3; }

  /**
   * This hull in another frame: each point x becomes R (x - c), R the unit
   * quaternion q's rotation as a matrix (r, 9 numbers, row by row), c the new
   * origin. The faces' planes go with it: n becomes R n and d becomes d - n·c.
   */
  moved(c, r) {
    const v = this.vertices, V = v.length / 3, out = new Float64Array(3 * V);
    for (let i = 0; i < V; i++) {
      const x = v[3 * i] - c[0], y = v[3 * i + 1] - c[1], z = v[3 * i + 2] - c[2];
      out[3 * i] = r[0] * x + r[1] * y + r[2] * z;
      out[3 * i + 1] = r[3] * x + r[4] * y + r[5] * z;
      out[3 * i + 2] = r[6] * x + r[7] * y + r[8] * z;
    }
    const p = this.planes, F = p.length / 4, planes = new Float64Array(4 * F);
    for (let f = 0; f < F; f++) {
      const nx = p[4 * f], ny = p[4 * f + 1], nz = p[4 * f + 2];
      planes[4 * f] = r[0] * nx + r[1] * ny + r[2] * nz;
      planes[4 * f + 1] = r[3] * nx + r[4] * ny + r[5] * nz;
      planes[4 * f + 2] = r[6] * nx + r[7] * ny + r[8] * nz;
      planes[4 * f + 3] = p[4 * f + 3] - (nx * c[0] + ny * c[1] + nz * c[2]);
    }
    return new Hull(out, this.start, this.loops, planes);
  }
}

/**
 * The hull of `points` (finite numbers, three to a point), or null if they
 * don't span a solid: fewer than four, or all on one plane, within rounding.
 */
export function build(points) {
  const n = points.length / 3;
  if (n < 4) return null;
  const P = Float64Array.from(points);
  let mx = 0, my = 0, mz = 0;
  for (let i = 0; i < n; i++) {
    mx = Math.max(mx, Math.abs(P[3 * i])); my = Math.max(my, Math.abs(P[3 * i + 1])); mz = Math.max(mz, Math.abs(P[3 * i + 2]));
  }
  const eps = ROUNDING * (mx + my + mz);

  // The first tetrahedron: the two points farthest apart of the six extremes,
  // the point farthest from their line, and the point farthest from their plane.
  const ext = [];
  for (let k = 0; k < 3; k++) {
    let lo = 0, hi = 0;
    for (let i = 1; i < n; i++) {
      if (P[3 * i + k] < P[3 * lo + k]) lo = i;
      if (P[3 * i + k] > P[3 * hi + k]) hi = i;
    }
    ext.push(lo, hi);
  }
  let i0 = 0, i1 = 0, far = -1;
  for (let a = 0; a < 6; a++) {
    for (let b = a + 1; b < 6; b++) {
      const d = dist2(P, ext[a], ext[b]);
      if (d > far) { far = d; i0 = ext[a]; i1 = ext[b]; }
    }
  }
  if (!(Math.sqrt(far) > eps)) return null;
  const ux = P[3 * i1] - P[3 * i0], uy = P[3 * i1 + 1] - P[3 * i0 + 1], uz = P[3 * i1 + 2] - P[3 * i0 + 2];
  const ul = Math.sqrt(ux * ux + uy * uy + uz * uz);
  let i2 = -1;
  far = 0;
  for (let i = 0; i < n; i++) {
    const x = P[3 * i] - P[3 * i0], y = P[3 * i + 1] - P[3 * i0 + 1], z = P[3 * i + 2] - P[3 * i0 + 2];
    const cx = y * uz - z * uy, cy = z * ux - x * uz, cz = x * uy - y * ux;
    const d = Math.sqrt(cx * cx + cy * cy + cz * cz) / ul;
    if (d > far) { far = d; i2 = i; }
  }
  if (!(far > eps)) return null;
  const base = plane(P, i0, i1, i2);
  let i3 = -1;
  far = 0;
  for (let i = 0; i < n; i++) {
    const d = Math.abs(above(P, base, i));
    if (d > far) { far = d; i3 = i; }
  }
  if (!(far > eps)) return null;

  // Faces as triangles while the hull grows: corners, plane, the points
  // above it, and whether it is still part of the hull. Directed edge u→v
  // (key u·n + v) to the face it belongs to, so a face finds its neighbours.
  const faces = [], edge = new Map();
  const add = (a, b, c) => {
    const f = { a, b, c, plane: plane(P, a, b, c), outside: [], alive: true };
    faces.push(f);
    edge.set(a * n + b, f); edge.set(b * n + c, f); edge.set(c * n + a, f);
    return f;
  };
  const kill = (f) => {
    f.alive = false;
    edge.delete(f.a * n + f.b); edge.delete(f.b * n + f.c); edge.delete(f.c * n + f.a);
  };
  // Turned so each face's normal points away from the fourth corner.
  const up = above(P, base, i3) > 0;
  const [a, b, c] = up ? [i0, i2, i1] : [i0, i1, i2];
  add(a, b, c);
  add(a, i3, b); add(b, i3, c); add(c, i3, a);

  /** Hands each point to the face it is farthest above; one above none is inside, and gone. */
  const assign = (list, onto) => {
    for (const i of list) {
      let best = null, most = eps;
      for (const f of onto) {
        const d = above(P, f.plane, i);
        if (d > most) { most = d; best = f; }
      }
      if (best) best.outside.push(i);
    }
  };
  const all = [];
  for (let i = 0; i < n; i++) if (i !== i0 && i !== i1 && i !== i2 && i !== i3) all.push(i);
  assign(all, faces);

  for (let next = 0; next < faces.length;) {
    const f = faces[next];
    if (!f.alive || f.outside.length === 0) { next++; continue; }
    // The point farthest above it; the first of equals.
    let eye = -1, most = -Infinity;
    for (const i of f.outside) {
      const d = above(P, f.plane, i);
      if (d > most) { most = d; eye = i; }
    }
    // Every face it sees, found outward from this one across shared edges,
    // so what is cut away is one piece and its edge one loop.
    const seen = new Set([f]), visit = [f];
    for (let k = 0; k < visit.length; k++) {
      const g = visit[k];
      for (const [u, v] of [[g.a, g.b], [g.b, g.c], [g.c, g.a]]) {
        const h = edge.get(v * n + u);
        if (h && !seen.has(h) && above(P, h.plane, eye) > eps) { seen.add(h); visit.push(h); }
      }
    }
    // The horizon: edges of seen faces whose face across is unseen.
    const horizon = [];
    for (const g of visit) {
      for (const [u, v] of [[g.a, g.b], [g.b, g.c], [g.c, g.a]]) {
        if (!seen.has(edge.get(v * n + u))) horizon.push(u, v);
      }
    }
    const orphans = [];
    for (const g of visit) {
      for (const i of g.outside) if (i !== eye) orphans.push(i);
      kill(g);
    }
    const made = [];
    for (let k = 0; k < horizon.length; k += 2) made.push(add(horizon[k], horizon[k + 1], eye));
    assign(orphans, made);
    next = 0;   // the earliest face with points above it, so the order is the input's alone
  }

  return merge(P, faces.filter((g) => g.alive), edge, n, mx + my + mz);
}

/**
 * The finished hull from its triangles: those lying flat together merged
 * into polygons, points on a straight edge or inside a face dropped, and the
 * rest numbered in the order they were given.
 */
function merge(P, tris, edge, n, scale) {
  // Flat to within FLAT of the hull's size (mesh.js's flatness: two
  // triangles exported from one plane in a float32 file are flat), measured
  // against the first triangle of a face, so a finely cut curve can't merge
  // into one bent face a little at a time.
  const flat = FLAT * scale;
  const group = new Map();
  const groups = [];
  for (const seed of tris) {
    if (group.has(seed)) continue;
    const g = [seed];
    group.set(seed, groups.length);
    const pl = seed.plane;
    for (let k = 0; k < g.length; k++) {
      const t = g[k];
      for (const [u, v] of [[t.a, t.b], [t.b, t.c], [t.c, t.a]]) {
        const h = edge.get(v * n + u);
        if (group.has(h)) continue;
        const dot = h.plane[0] * pl[0] + h.plane[1] * pl[1] + h.plane[2] * pl[2];
        if (dot > 0 && [h.a, h.b, h.c].every((i) => Math.abs(above(P, pl, i)) <= flat)) {
          group.set(h, groups.length);
          g.push(h);
        }
      }
    }
    groups.push(g);
  }
  // Each face's edge, as a loop: the directed edges of its triangles whose
  // reverse belongs to another face, chained end to start.
  const loops = groups.map((g, gi) => {
    const nextOf = new Map();
    for (const t of g) {
      for (const [u, v] of [[t.a, t.b], [t.b, t.c], [t.c, t.a]]) {
        if (group.get(edge.get(v * n + u)) !== gi) nextOf.set(u, v);
      }
    }
    // Starting from its lowest-numbered corner, so the loop doesn't depend on the order triangles were made in.
    let first = Infinity;
    for (const u of nextOf.keys()) first = Math.min(first, u);
    const loop = [first];
    for (let v = nextOf.get(first); v !== first; v = nextOf.get(v)) loop.push(v);
    return loop;
  });
  // A corner on a straight edge in every face it is in lies along an edge of
  // the solid, not at a corner of it, and goes.
  const corner = new Map();
  for (const loop of loops) {
    for (let k = 0; k < loop.length; k++) {
      const p = loop[(k + loop.length - 1) % loop.length], v = loop[k], q = loop[(k + 1) % loop.length];
      const ax = P[3 * v] - P[3 * p], ay = P[3 * v + 1] - P[3 * p + 1], az = P[3 * v + 2] - P[3 * p + 2];
      const bx = P[3 * q] - P[3 * v], by = P[3 * q + 1] - P[3 * v + 1], bz = P[3 * q + 2] - P[3 * v + 2];
      const cx = ay * bz - az * by, cy = az * bx - ax * bz, cz = ax * by - ay * bx;
      // Its turn, as a sine, against flatness: the same FLAT as the faces'.
      const bends = Math.sqrt(cx * cx + cy * cy + cz * cz) > FLAT * Math.sqrt((ax * ax + ay * ay + az * az) * (bx * bx + by * by + bz * bz));
      corner.set(v, (corner.get(v) || false) || bends);
    }
  }
  const kept = [...corner.keys()].filter((v) => corner.get(v)).sort((x, y) => x - y);
  const index = new Map(kept.map((v, i) => [v, i]));
  const vertices = new Float64Array(3 * kept.length);
  kept.forEach((v, i) => { vertices[3 * i] = P[3 * v]; vertices[3 * i + 1] = P[3 * v + 1]; vertices[3 * i + 2] = P[3 * v + 2]; });
  const start = new Int32Array(loops.length + 1), flatLoops = [];
  const planes = new Float64Array(4 * loops.length);
  loops.forEach((loop, f) => {
    const corners = loop.filter((v) => corner.get(v)).map((v) => index.get(v));
    start[f] = flatLoops.length;
    flatLoops.push(...corners);
    // The normal by Newell's method, which averages over the whole polygon,
    // and the plane through its farthest corner of the hull, so every corner
    // is on or under every face: what separating axes rely on.
    let nx = 0, ny = 0, nz = 0;
    for (let k = 0; k < corners.length; k++) {
      const i = corners[k], j = corners[(k + 1) % corners.length];
      nx += (vertices[3 * i + 1] - vertices[3 * j + 1]) * (vertices[3 * i + 2] + vertices[3 * j + 2]);
      ny += (vertices[3 * i + 2] - vertices[3 * j + 2]) * (vertices[3 * i] + vertices[3 * j]);
      nz += (vertices[3 * i] - vertices[3 * j]) * (vertices[3 * i + 1] + vertices[3 * j + 1]);
    }
    const l = Math.sqrt(nx * nx + ny * ny + nz * nz);
    nx /= l; ny /= l; nz /= l;
    let d = -Infinity;
    for (let i = 0; i < kept.length; i++) d = Math.max(d, nx * vertices[3 * i] + ny * vertices[3 * i + 1] + nz * vertices[3 * i + 2]);
    planes[4 * f] = nx; planes[4 * f + 1] = ny; planes[4 * f + 2] = nz; planes[4 * f + 3] = d;
  });
  start[loops.length] = flatLoops.length;
  return new Hull(vertices, start, Int32Array.from(flatLoops), planes);
}

/** The squared distance between points i and j. */
function dist2(P, i, j) {
  const x = P[3 * j] - P[3 * i], y = P[3 * j + 1] - P[3 * i + 1], z = P[3 * j + 2] - P[3 * i + 2];
  return x * x + y * y + z * z;
}

/** The plane through points a, b and c, normal by the right hand round them: [nx, ny, nz, d]. */
function plane(P, a, b, c) {
  const ux = P[3 * b] - P[3 * a], uy = P[3 * b + 1] - P[3 * a + 1], uz = P[3 * b + 2] - P[3 * a + 2];
  const vx = P[3 * c] - P[3 * a], vy = P[3 * c + 1] - P[3 * a + 1], vz = P[3 * c + 2] - P[3 * a + 2];
  let nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
  const l = Math.sqrt(nx * nx + ny * ny + nz * nz);
  nx /= l; ny /= l; nz /= l;
  return [nx, ny, nz, nx * P[3 * a] + ny * P[3 * a + 1] + nz * P[3 * a + 2]];
}

/** How far point i is above plane p. */
function above(P, p, i) {
  return p[0] * P[3 * i] + p[1] * P[3 * i + 1] + p[2] * P[3 * i + 2] - p[3];
}
