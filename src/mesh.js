// Triangle meshes and heightfields: level geometry, as triangles.
//
// Both are sources of triangles in their body's own frame. A mesh keeps
// every triangle and a tree of their boxes (tree.js, the broadphase's), so a
// body touching a floor of 50,000 triangles looks at the few under it. A
// heightfield keeps only its grid of heights, and works out the triangles
// under a box from which cells the box covers: two triangles to a cell, cut
// corner to corner.
//
// SEAMS AND CORNERS. Where two triangles meet flat, or fold inward, their
// shared edge is a seam: nothing can touch the edge itself without touching
// one of the faces first, so a contact that comes out pointing off the edge
// is a ghost, and would catch a box sliding over a flat floor on the seam.
// Where they fold outward, the edge is a corner, and its contacts are real.
// Which is which depends on the side a body is on, since a fold outward seen
// from above is a fold inward seen from below, and triangles here have two
// sides. So each edge carries two answers, one a side, worked out once for a
// mesh and cell by cell for a heightfield. An edge with no triangle across it
// is a corner from either side. triangle.js reads these.
//
// FOLDS. A corner's fold matters too, not only that it is one. Seen across
// an edge, the triangle beside holds everything over it: a contact at the
// edge whose normal leans past the triangle beside's own side of the edge is
// a body over that triangle, which its face holds, and the edge contact is a
// ghost there as at a seam. On ground rippled half a millimetre, every edge
// folds by a few thousandths of a radian, far past flat, so each is a corner;
// a box sliding over it met the next triangle's near edge side-on, by the
// closest points of the two, with a normal along the ground, and stopped as
// at a wall. So each edge also keeps which way the triangle across runs from
// it (fold()), and triangle.js keeps an edge's contact only if its normal
// lies between the two triangles' sides of it.

import { Tree, raycast as walk } from './tree.js';

/**
 * Flat to within a part in 2^20 of an edge's length: sixteen times the
 * rounding of the float32 coordinates a model file stores, so two triangles
 * exported from one plane count as flat, and any fold you could see doesn't.
 */
const FLAT = 1 / 1048576;

/**
 * Flags, per triangle: bit e for edge e a corner seen from the front (the
 * side the normal points to), bit e + 3 from the back. Edge e runs from
 * corner e to corner e + 1. A triangle's corner is a seam from a side when
 * both its edges there are (triangle.js).
 */
const FRONT = 1, BACK = 8;

/**
 * Whether the triangle across an edge, whose far corner is `bend` from this
 * triangle's plane (along its normal), makes the edge a corner from the front
 * and from the back. A fold away from a side is a corner seen from it.
 */
function flags(bend, length) {
  if (Number.isNaN(bend)) return FRONT | BACK;
  const flat = length * FLAT;
  return (bend < -flat ? FRONT : 0) | (bend > flat ? BACK : 0);
}

/**
 * Edge e of triangle p (9 numbers, unit normal n), with the far corner f of
 * the triangle across it: the way into that triangle from the edge, square to
 * the edge, as its parts along this triangle's outward square to the edge (m,
 * in its plane, away from its third corner) and along n, into out[o] and
 * out[o + 1]. A flat neighbour runs along m, (1, 0); a fold away from the
 * front tips it down, a fold towards it up. triangle.js reads these.
 */
function fold(p, n, e, fx, fy, fz, out, o) {
  const a = 3 * e, b = 3 * ((e + 1) % 3), c = 3 * ((e + 2) % 3);
  let ex = p[b] - p[a], ey = p[b + 1] - p[a + 1], ez = p[b + 2] - p[a + 2];
  const el = Math.sqrt(ex * ex + ey * ey + ez * ez);
  ex /= el; ey /= el; ez /= el;
  let mx = ey * n[2] - ez * n[1], my = ez * n[0] - ex * n[2], mz = ex * n[1] - ey * n[0];
  if (mx * (p[c] - p[a]) + my * (p[c + 1] - p[a + 1]) + mz * (p[c + 2] - p[a + 2]) > 0) { mx = -mx; my = -my; mz = -mz; }
  let dx = fx - p[a], dy = fy - p[a + 1], dz = fz - p[a + 2];
  const along = dx * ex + dy * ey + dz * ez;
  dx -= along * ex; dy -= along * ey; dz -= along * ez;
  const dl = Math.sqrt(dx * dx + dy * dy + dz * dz);
  out[o] = (dx * mx + dy * my + dz * mz) / dl;
  out[o + 1] = (dx * n[0] + dy * n[1] + dz * n[2]) / dl;
}

/** Corner positions into out, for vertices (an array of numbers) at indices i, j, k. */
function corners(v, i, j, k, out) {
  out[0] = v[3 * i]; out[1] = v[3 * i + 1]; out[2] = v[3 * i + 2];
  out[3] = v[3 * j]; out[4] = v[3 * j + 1]; out[5] = v[3 * j + 2];
  out[6] = v[3 * k]; out[7] = v[3 * k + 1]; out[8] = v[3 * k + 2];
}

/** The unit normal of triangle p (9 numbers), into out, and its area times two; 0 for none. */
function normal(p, out) {
  const ux = p[3] - p[0], uy = p[4] - p[1], uz = p[5] - p[2];
  const vx = p[6] - p[0], vy = p[7] - p[1], vz = p[8] - p[2];
  const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
  const l = Math.sqrt(nx * nx + ny * ny + nz * nz);
  if (l > 0) { out[0] = nx / l; out[1] = ny / l; out[2] = nz / l; }
  return l;
}

const tri = new Float64Array(9), across = new Float64Array(9), n3 = new Float64Array(3);

/** Whether a triangle has area: its twice-area against its longest edge's square, past rounding. */
function hasArea(p) {
  const l = normal(p, n3);
  let longest = 0;
  for (let e = 0; e < 3; e++) {
    const a = 3 * e, b = 3 * ((e + 1) % 3);
    const dx = p[b] - p[a], dy = p[b + 1] - p[a + 1], dz = p[b + 2] - p[a + 2];
    longest = Math.max(longest, dx * dx + dy * dy + dz * dz);
  }
  return l > longest * FLAT;
}

export class Mesh {
  /**
   * vertices: numbers, three to a vertex; indices: three to a triangle, each
   * below the vertex count (checked by World). Triangles with no area are
   * left out: nothing could touch them.
   */
  constructor(vertices, indices) {
    this.kind = 'mesh';
    this.vertices = Float64Array.from(vertices);
    const v = this.vertices, kept = [];
    for (let t = 0; t < indices.length; t += 3) {
      corners(v, indices[t], indices[t + 1], indices[t + 2], tri);
      if (hasArea(tri)) kept.push(indices[t], indices[t + 1], indices[t + 2]);
    }
    this.indices = Uint32Array.from(kept);
    this.count = this.indices.length / 3;
    // Its box, in its own frame, and how far it reaches from its origin.
    this.lo = [Infinity, Infinity, Infinity];
    this.hi = [-Infinity, -Infinity, -Infinity];
    this.reach = 0;
    for (let i = 0; i < this.count * 3; i++) {
      const k = this.indices[i];
      for (let c = 0; c < 3; c++) {
        this.lo[c] = Math.min(this.lo[c], v[3 * k + c]);
        this.hi[c] = Math.max(this.hi[c], v[3 * k + c]);
      }
      this.reach = Math.max(this.reach, Math.sqrt(v[3 * k] * v[3 * k] + v[3 * k + 1] * v[3 * k + 1] + v[3 * k + 2] * v[3 * k + 2]));
    }
    if (this.count === 0) { this.lo = [0, 0, 0]; this.hi = [0, 0, 0]; }
    this.tree = new Tree();
    for (let t = 0; t < this.count; t++) {
      this.triangle(t, tri);
      this.tree.insert(t,
        Math.min(tri[0], tri[3], tri[6]), Math.min(tri[1], tri[4], tri[7]), Math.min(tri[2], tri[5], tri[8]),
        Math.max(tri[0], tri[3], tri[6]), Math.max(tri[1], tri[4], tri[7]), Math.max(tri[2], tri[5], tri[8]));
    }
    // Each edge's fold (see FOLDS), two numbers an edge, six a triangle.
    // Float32: 2^-24 of a unit direction, under FLAT's 2^-20, the finest
    // fold that counts.
    this.folds = new Float32Array(6 * this.count);
    this.edges = this.seams();
  }

  /** Triangle t's corners, in the mesh's frame, into out. */
  triangle(t, out) {
    const ix = this.indices;
    corners(this.vertices, ix[3 * t], ix[3 * t + 1], ix[3 * t + 2], out);
  }

  /** Calls visit(t) for every triangle whose box overlaps the given one, in the mesh's frame. */
  triangles(x0, y0, z0, x1, y1, z1, visit) {
    this.tree.query(x0, y0, z0, x1, y1, z1, visit);
  }

  /**
   * Calls visit(t) for every triangle whose box, grown by (gx, gy, gz), the
   * ray from (ox, oy, oz) along (dx, dy, dz) passes through before maxT,
   * nearer ones first where it can; visit returns the maxT to carry on with.
   * In the mesh's frame. For rays and shape casts (query.js).
   */
  ray(ox, oy, oz, dx, dy, dz, maxT, gx, gy, gz, visit) {
    walk(this.tree, ox, oy, oz, dx, dy, dz, maxT, gx, gy, gz, visit);
  }

  /** Triangle t's flags: see FRONT and BACK. Its folds into `folds` (six numbers; see FOLDS). */
  edge(t, folds) {
    const f = this.folds, o = 6 * t;
    for (let k = 0; k < 6; k++) folds[k] = f[o + k];
    return this.edges[t];
  }

  /**
   * Every edge's flags. Vertices are matched by position, not by index: a
   * model file repeats a vertex wherever its normal or texture changes, and
   * the two triangles either side of such a seam still meet.
   */
  seams() {
    const v = this.vertices, ix = this.indices, V = v.length / 3, same = new Uint32Array(V);
    // Vertices with the same position: an open-addressed table of first
    // indices, keyed by the coordinates' bits (with -0 as 0, as equal numbers
    // must be). Each vertex is mapped to the first one found at its place,
    // which depends only on the order, not on the hash. A table of strings and
    // a Map of edge lists did this before, 15% of adding a mesh of 131,072
    // triangles and most of its garbage, and the edge key a * 2^32 + b
    // stopped being exact past 2^21 vertices. Measured: see ROADMAP.md,
    // milestone 16.
    const head = new Int32Array(power(2 * V)).fill(-1), vmask = head.length - 1;
    for (let i = 0; i < V; i++) {
      const x = v[3 * i] + 0, y = v[3 * i + 1] + 0, z = v[3 * i + 2] + 0;
      let h = hash3(x, y, z) & vmask;
      for (;;) {
        const k = head[h];
        if (k < 0) { head[h] = i; same[i] = i; break; }
        if (v[3 * k] === x && v[3 * k + 1] === y && v[3 * k + 2] === z) { same[i] = k; break; }
        h = (h + 1) & vmask;
      }
    }
    // Each edge, by its two vertices, lowest first: how many triangles have
    // it (up to 3, which is "more than 2") and the first two. Where each
    // triangle's edges landed is kept, so the next pass needn't look again.
    const E = power(6 * this.count), emask = E - 1;
    const ka = new Int32Array(E).fill(-1), kb = new Int32Array(E), first = new Int32Array(E), second = new Int32Array(E);
    const n = new Uint8Array(E), slot = new Int32Array(3 * this.count);
    for (let t = 0; t < this.count; t++) {
      for (let e = 0; e < 3; e++) {
        let a = same[ix[3 * t + e]], b = same[ix[3 * t + (e + 1) % 3]];
        if (b < a) { const c = a; a = b; b = c; }
        let h = (Math.imul(a, 0x9e3779b1) ^ Math.imul(b, 0x85ebca77)) & emask;
        while (ka[h] >= 0 && (ka[h] !== a || kb[h] !== b)) h = (h + 1) & emask;
        if (ka[h] < 0) { ka[h] = a; kb[h] = b; first[h] = t; n[h] = 1; }
        else if (n[h] === 1) { second[h] = t; n[h] = 2; }
        else n[h] = 3;
        slot[3 * t + e] = h;
      }
    }
    const out = new Uint8Array(this.count);
    for (let t = 0; t < this.count; t++) {
      this.triangle(t, tri);
      normal(tri, n3);
      for (let e = 0; e < 3; e++) {
        const a = same[ix[3 * t + e]], b = same[ix[3 * t + (e + 1) % 3]], h = slot[3 * t + e];
        let bend = NaN;
        // Exactly one triangle across; more makes the edge no seam anyone can trust.
        if (n[h] === 2) {
          const o = first[h] === t ? second[h] : first[h];
          for (let c = 0; c < 3; c++) {
            const far = same[ix[3 * o + c]];
            if (far !== a && far !== b) {
              const fx = v[3 * far] - tri[0], fy = v[3 * far + 1] - tri[1], fz = v[3 * far + 2] - tri[2];
              bend = fx * n3[0] + fy * n3[1] + fz * n3[2];
              fold(tri, n3, e, v[3 * far], v[3 * far + 1], v[3 * far + 2], this.folds, 6 * t + 2 * e);
            }
          }
        }
        const ex = v[3 * b] - v[3 * a], ey = v[3 * b + 1] - v[3 * a + 1], ez = v[3 * b + 2] - v[3 * a + 2];
        const f = flags(bend, Math.sqrt(ex * ex + ey * ey + ez * ez));
        out[t] |= (f & FRONT ? FRONT << e : 0) | (f & BACK ? BACK << e : 0);
      }
    }
    return out;
  }
}

/** The least power of two at least n, and at least 16: a hash table's size. */
function power(n) {
  let c = 16;
  while (c < n) c *= 2;
  return c;
}

// A position's bits, for hash3.
const BITS = new Float64Array(3), WORDS = new Uint32Array(BITS.buffer);

/** A hash of three numbers' bits. Only where entries sit depends on it, never what is found. */
function hash3(x, y, z) {
  BITS[0] = x; BITS[1] = y; BITS[2] = z;
  let h = 0x811c9dc5;
  for (let k = 0; k < 6; k++) h = Math.imul(h ^ WORDS[k], 0x01000193);
  return (h ^ (h >>> 15)) >>> 0;
}

export class Heightfield {
  /** heights: rows of `columns` heights, row by row; spacing: metres between neighbouring samples. */
  constructor(heights, columns, spacing) {
    this.kind = 'heightfield';
    this.heights = Float64Array.from(heights);
    this.columns = columns;
    this.rows = heights.length / columns;
    this.spacing = spacing;
    // Centred on the body: x along columns, z along rows.
    this.x0 = -((columns - 1) * spacing) / 2;
    this.z0 = -((this.rows - 1) * spacing) / 2;
    let lo = Infinity, hi = -Infinity;
    for (const h of this.heights) { lo = Math.min(lo, h); hi = Math.max(hi, h); }
    this.lo = [this.x0, lo, this.z0];
    this.hi = [-this.x0, hi, -this.z0];
    this.reach = Math.sqrt(this.x0 * this.x0 + this.z0 * this.z0 + Math.max(lo * lo, hi * hi));
    this.count = 2 * (columns - 1) * (this.rows - 1);
  }

  /** The grid point (i, j)'s height. */
  h(i, j) {
    return this.heights[j * this.columns + i];
  }

  /**
   * Triangle t's corners, as grid points into g (i, j pairs). Cell (i, j)'s
   * two triangles share its diagonal from (i, j) to (i + 1, j + 1); both
   * face up, +y.
   */
  grid(t, g) {
    const cell = t >> 1, cells = this.columns - 1, i = cell % cells, j = (cell - i) / cells;
    if ((t & 1) === 0) { g[0] = i; g[1] = j; g[2] = i + 1; g[3] = j + 1; g[4] = i + 1; g[5] = j; }
    else { g[0] = i; g[1] = j; g[2] = i; g[3] = j + 1; g[4] = i + 1; g[5] = j + 1; }
  }

  triangle(t, out) {
    this.grid(t, G);
    for (let c = 0; c < 3; c++) {
      const i = G[2 * c], j = G[2 * c + 1];
      out[3 * c] = this.x0 + i * this.spacing;
      out[3 * c + 1] = this.h(i, j);
      out[3 * c + 2] = this.z0 + j * this.spacing;
    }
  }

  /**
   * As Mesh.ray. A ray walks the cells under it in the order it passes over
   * them, stopping at the first cell past maxT; a shape cast (any g) visits
   * every cell under the box it sweeps through the field. ponytail: that box
   * is every cell along a long cast; walk a band of cells as the ray does if
   * long casts across big fields ever measure slow.
   */
  ray(ox, oy, oz, dx, dy, dz, maxT, gx, gy, gz, visit) {
    // The part of the ray over the field's box, grown by g.
    let t0 = 0, t1 = maxT;
    const lo = this.lo, hi = this.hi;
    for (let k = 0; k < 3; k++) {
      const o = k === 0 ? ox : k === 1 ? oy : oz, d = k === 0 ? dx : k === 1 ? dy : dz, g = k === 0 ? gx : k === 1 ? gy : gz;
      if (d === 0) {
        if (o < lo[k] - g || o > hi[k] + g) return;
        continue;
      }
      let near = (lo[k] - g - o) / d, far = (hi[k] + g - o) / d;
      if (near > far) { const s = near; near = far; far = s; }
      if (near > t0) t0 = near;
      if (far < t1) t1 = far;
      if (t0 > t1) return;
    }
    if (gx > 0 || gy > 0 || gz > 0) {
      const ax = ox + t0 * dx, ay = oy + t0 * dy, az = oz + t0 * dz, bx = ox + t1 * dx, by = oy + t1 * dy, bz = oz + t1 * dz;
      this.triangles(Math.min(ax, bx) - gx, Math.min(ay, by) - gy, Math.min(az, bz) - gz,
        Math.max(ax, bx) + gx, Math.max(ay, by) + gy, Math.max(az, bz) + gz, visit);
      return;
    }
    const s = this.spacing, cells = this.columns - 1, rows = this.rows - 1;
    let i = Math.floor((ox + t0 * dx - this.x0) / s), j = Math.floor((oz + t0 * dz - this.z0) / s);
    i = i < 0 ? 0 : i >= cells ? cells - 1 : i;
    j = j < 0 ? 0 : j >= rows ? rows - 1 : j;
    // Where along the ray it next crosses a line of the grid, across x and across z.
    const si = dx > 0 ? 1 : -1, sj = dz > 0 ? 1 : -1;
    let nextX = dx === 0 ? Infinity : (this.x0 + (i + (dx > 0 ? 1 : 0)) * s - ox) / dx;
    let nextZ = dz === 0 ? Infinity : (this.z0 + (j + (dz > 0 ? 1 : 0)) * s - oz) / dz;
    const stepX = dx === 0 ? Infinity : s / Math.abs(dx), stepZ = dz === 0 ? Infinity : s / Math.abs(dz);
    for (;;) {
      const c = 2 * (j * cells + i);
      visit(c);
      t1 = Math.min(t1, visit(c + 1));
      if (nextX <= nextZ) {
        if (nextX > t1) return;
        i += si; nextX += stepX;
        if (i < 0 || i >= cells) return;
      } else {
        if (nextZ > t1) return;
        j += sj; nextZ += stepZ;
        if (j < 0 || j >= rows) return;
      }
    }
  }

  /** Calls visit(t) for both triangles of every cell under the box, if the cell's heights reach it. */
  triangles(x0, y0, z0, x1, y1, z1, visit) {
    const s = this.spacing, cells = this.columns - 1;
    const i0 = Math.max(0, Math.floor((x0 - this.x0) / s)), i1 = Math.min(cells - 1, Math.floor((x1 - this.x0) / s));
    const j0 = Math.max(0, Math.floor((z0 - this.z0) / s)), j1 = Math.min(this.rows - 2, Math.floor((z1 - this.z0) / s));
    for (let j = j0; j <= j1; j++) {
      for (let i = i0; i <= i1; i++) {
        const a = this.h(i, j), b = this.h(i + 1, j), c = this.h(i, j + 1), d = this.h(i + 1, j + 1);
        if (Math.max(a, b, c, d) < y0 || Math.min(a, b, c, d) > y1) continue;
        const t = 2 * (j * cells + i);
        visit(t);
        visit(t + 1);
      }
    }
  }

  /**
   * Triangle t's edge flags, worked out now: for each edge, the triangle
   * across it is in the same cell (the diagonal) or the next cell over, and
   * is whichever of that cell's two holds both of the edge's grid points.
   * Its folds into `folds`, as Mesh.edge.
   */
  edge(t, folds) {
    this.grid(t, G);
    this.triangle(t, tri);
    normal(tri, n3);
    const cells = this.columns - 1;
    let out = 0;
    for (let e = 0; e < 3; e++) {
      const ai = G[2 * e], aj = G[2 * e + 1], bi = G[(2 * e + 2) % 6], bj = G[(2 * e + 3) % 6];
      let bend = NaN;
      folds[2 * e] = 0; folds[2 * e + 1] = 0;
      const cell = t >> 1, ci = cell % cells, cj = (cell - ci) / cells;
      // The cell across: the same one for the diagonal, else one step away from this cell's middle.
      let ni = ci, nj = cj, other = -1;
      if (ai !== bi && aj !== bj) {
        other = t ^ 1;
      } else {
        if (ai === bi) ni = ai === ci ? ci - 1 : ci + 1;
        else nj = aj === cj ? cj - 1 : cj + 1;
        if (ni >= 0 && nj >= 0 && ni < cells && nj < this.rows - 1) {
          for (let k = 0; k < 2; k++) {
            const u = 2 * (nj * cells + ni) + k;
            this.grid(u, H);
            let has = 0;
            for (let c = 0; c < 3; c++) {
              if ((H[2 * c] === ai && H[2 * c + 1] === aj) || (H[2 * c] === bi && H[2 * c + 1] === bj)) has++;
            }
            if (has === 2) other = u;
          }
        }
      }
      if (other >= 0) {
        this.grid(other, H);
        for (let c = 0; c < 3; c++) {
          const fi = H[2 * c], fj = H[2 * c + 1];
          if ((fi === ai && fj === aj) || (fi === bi && fj === bj)) continue;
          const X = this.x0 + fi * this.spacing, Y = this.h(fi, fj), Z = this.z0 + fj * this.spacing;
          const fx = X - tri[0], fy = Y - tri[1], fz = Z - tri[2];
          bend = fx * n3[0] + fy * n3[1] + fz * n3[2];
          fold(tri, n3, e, X, Y, Z, folds, 2 * e);
        }
      }
      const ex = (bi - ai) * this.spacing, ey = this.h(bi, bj) - this.h(ai, aj), ez = (bj - aj) * this.spacing;
      const f = flags(bend, Math.sqrt(ex * ex + ey * ey + ez * ez));
      out |= (f & FRONT ? FRONT << e : 0) | (f & BACK ? BACK << e : 0);
    }
    return out;
  }
}

const G = new Int32Array(6), H = new Int32Array(6);

export { FRONT, BACK, FLAT, normal as triangleNormal };
