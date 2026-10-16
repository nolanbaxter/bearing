// A shape against the triangles of a mesh or heightfield.
//
// touchLevel() finds where a shape touches a mesh or heightfield, as
// manifolds like collide()'s: one for each direction it is pushed in. It asks
// the mesh for the triangles under the shape's box, touches each, and puts
// the points of every triangle that pushes the same way into one manifold, so
// a box resting across a flat floor of many triangles gets one contact of
// four points, its corners, exactly as on one big box.
//
// touch(s, t, flags, margin, tolerance, out, tri) is one triangle, its normal
// from s towards the triangle: a sphere by the closest point on the triangle;
// a capsule by its segment clipped to the triangle, lying on it, or its
// closest point otherwise; a box by separating axes (the triangle's face, the
// box's three faces, and each box edge against each triangle edge) and
// clipping, as boxes against boxes (collide.js); a hull the same way, with
// its own faces and edges in place of the box's. It leaves every clipped
// point in `out`; reducing to four is done once, over the whole manifold.
//
// TWO-SIDED. A triangle is touched from whichever side the shape's centre
// is on, and its face normal is turned to face that side.
//
// SEAMS. `flags` says which of the triangle's edges are corners from each
// side (mesh.js); a corner of the triangle is a seam when both its edges
// there are. A contact that comes out against a seam would push the shape
// off an edge it could never reach without touching a face first: a ghost,
// the bump a box sliding over a flat floor catches on. Such a contact means
// the shape is over the triangle beside, whose face holds it, so it is left
// out; only a ball over this triangle too, to within the tolerance, as one
// rolling along the line between two is, takes this triangle's face instead.
// Never this face carried on past the seam: on a crest, that would pass
// above the ground beside it and hold a ball off it. And a point that
// clipping makes on a seam lies inside the surface the triangles make
// together, not at its edge, so it too is left out: the box's own corners,
// wherever they lie, are what holds it.

import { Shape, Manifold, Placed, SPHERE, CAPSULE, HULL, TRIANGLE, MAX_POINTS } from './collide.js';
import { closest, Closest } from './gjk.js';
import { FLAT, triangleNormal } from './mesh.js';
import { resized } from './core/slots.js';

const near = new Closest();
// The triangle turned to face the shape, and its corners, for the helpers below.
const up = new Float64Array(3), P = new Float64Array(9);
let corners = 0;   // which edges are corners from the shape's side, bit e for edge e
/**
 * Added to the id of a point made by the triangle's own features, so the same
 * feature of two triangles doesn't share an id: (t + 1) * 512, past the 9
 * bits the ids below use, wrapping at 32 bits. A box's own corners keep
 * theirs, so a box sliding from triangle to triangle keeps its impulses.
 */
let mix = 0;

/** The most points one triangle can give: a 4-sided face clipped by 3 planes, or a triangle by 4, has at most 7 sides. */
const MOST = 7;

export function touch(s, t, flags, margin, tolerance, out, tri) {
  out.count = 0;
  mix = (tri + 1) * 512;
  P.set(t.points);
  const nx = t.axes[0], ny = t.axes[1], nz = t.axes[2];
  const front = (s.center[0] - P[0]) * nx + (s.center[1] - P[1]) * ny + (s.center[2] - P[2]) * nz >= 0;
  const k = front ? 1 : -1;
  up[0] = k * nx; up[1] = k * ny; up[2] = k * nz;
  corners = front ? flags & 7 : (flags >> 3) & 7;
  if (s.kind === SPHERE) sphere(s, t, margin, tolerance, out);
  else if (s.kind === CAPSULE) capsule(s, t, margin, tolerance, out);
  else if (s.kind === HULL) hull(s, t, margin, tolerance, out);
  else box(s, t, margin, tolerance, out);
  return out.count;
}

// ---- where on the triangle ----

const FACE = 6;   // region codes: 0, 1, 2 the corners; 3, 4, 5 the edges a→b, b→c, c→a; 6 the face

/** Whether a region of the triangle is a seam from the shape's side (mesh.js decides which are). */
function seam(region) {
  if (region === FACE) return false;
  if (region >= 3) return (corners >> (region - 3) & 1) === 0;
  // Corner v is where edge v (from it) and edge v - 1 (into it) meet.
  return (corners >> region & 1) === 0 && (corners >> ((region + 2) % 3) & 1) === 0;
}

/** Which region of the triangle a point on it lies in, to within the tolerance. */
function region(x, y, z, tolerance) {
  const t2 = tolerance * tolerance;
  for (let v = 0; v < 3; v++) {
    const dx = x - P[3 * v], dy = y - P[3 * v + 1], dz = z - P[3 * v + 2];
    if (dx * dx + dy * dy + dz * dz <= t2) return v;
  }
  for (let e = 0; e < 3; e++) {
    const a = 3 * e, b = 3 * ((e + 1) % 3);
    const ex = P[b] - P[a], ey = P[b + 1] - P[a + 1], ez = P[b + 2] - P[a + 2];
    const px = x - P[a], py = y - P[a + 1], pz = z - P[a + 2];
    const ee = ex * ex + ey * ey + ez * ez, f = (px * ex + py * ey + pz * ez) / ee;
    const qx = px - f * ex, qy = py - f * ey, qz = pz - f * ez;
    if (f >= 0 && f <= 1 && qx * qx + qy * qy + qz * qz <= t2) return 3 + e;
  }
  return FACE;
}

/** The closest point on the triangle to (x, y, z), into Q, and its region: Ericson's, 5.1.5. */
const Q = new Float64Array(3);
function nearest(x, y, z) {
  const ax = P[0], ay = P[1], az = P[2];
  const abx = P[3] - ax, aby = P[4] - ay, abz = P[5] - az, acx = P[6] - ax, acy = P[7] - ay, acz = P[8] - az;
  const apx = x - ax, apy = y - ay, apz = z - az;
  const d1 = abx * apx + aby * apy + abz * apz, d2 = acx * apx + acy * apy + acz * apz;
  const at = (u, v, r) => {
    Q[0] = ax + u * abx + v * acx; Q[1] = ay + u * aby + v * acy; Q[2] = az + u * abz + v * acz;
    return r;
  };
  if (d1 <= 0 && d2 <= 0) return at(0, 0, 0);
  const bpx = x - P[3], bpy = y - P[4], bpz = z - P[5];
  const d3 = abx * bpx + aby * bpy + abz * bpz, d4 = acx * bpx + acy * bpy + acz * bpz;
  if (d3 >= 0 && d4 <= d3) return at(1, 0, 1);
  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) return at(d1 / (d1 - d3), 0, 3);
  const cpx = x - P[6], cpy = y - P[7], cpz = z - P[8];
  const d5 = abx * cpx + aby * cpy + abz * cpz, d6 = acx * cpx + acy * cpy + acz * cpz;
  if (d6 >= 0 && d5 <= d6) return at(0, 1, 2);
  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) return at(0, d2 / (d2 - d6), 5);
  const va = d3 * d6 - d5 * d4;
  if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
    const w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return at(1 - w, w, 4);
  }
  const den = 1 / (va + vb + vc);
  return at(vb * den, vc * den, FACE);
}

/** Height of (x, y, z) above the triangle's plane, on the shape's side. */
const above = (x, y, z) => (x - P[0]) * up[0] + (y - P[1]) * up[1] + (z - P[2]) * up[2];

function add(out, x, y, z, separation, id) {
  if (out.count === out.separations.length) return;
  const i = out.count++;
  out.points[3 * i] = x; out.points[3 * i + 1] = y; out.points[3 * i + 2] = z;
  out.separations[i] = separation;
  out.ids[i] = id;
}

function aim(out, x, y, z) {
  out.normal[0] = x; out.normal[1] = y; out.normal[2] = z;
}

// ---- a sphere ----

function sphere(s, t, margin, tolerance, out) {
  const c = s.center, r = s.half[0];
  let where = nearest(c[0], c[1], c[2]);
  let nx = c[0] - Q[0], ny = c[1] - Q[1], nz = c[2] - Q[2];
  const d = Math.sqrt(nx * nx + ny * ny + nz * nz);
  let sep;
  if (seam(where)) {
    // Nearest at a seam. Over the triangle, to within the tolerance, as a
    // ball rolling along the line between two flat triangles is over both,
    // its face holds it. Otherwise the ball is over another triangle, which
    // holds it, and this one's plane, carried on past the seam, may pass
    // above the ground there, on a crest, and hold the ball off it.
    const h = above(c[0], c[1], c[2]);
    const fx = c[0] - h * up[0] - Q[0], fy = c[1] - h * up[1] - Q[1], fz = c[2] - h * up[2] - Q[2];
    if (fx * fx + fy * fy + fz * fz > tolerance * tolerance) return;
    nx = up[0]; ny = up[1]; nz = up[2];
    sep = h - r;
    where = FACE;
  } else if (d > r / 67108864) {   // past rounding (2^-26 of the radius): a direction of its own
    nx /= d; ny /= d; nz /= d;
    sep = d - r;
  } else {
    // Its centre on the triangle: the face's direction.
    nx = up[0]; ny = up[1]; nz = up[2];
    sep = above(c[0], c[1], c[2]) - r;
  }
  if (sep > margin) return;
  aim(out, -nx, -ny, -nz);
  const m = r + sep / 2;
  add(out, c[0] - nx * m, c[1] - ny * m, c[2] - nz * m, sep, where);
}

// ---- a capsule ----

const seg = new Float64Array(6);
/** Which edge last cut each end of seg back, or -1 for an end that is the capsule's own. */
const cut = new Int8Array(2);

/** Clips the segment in seg to the prism over the triangle; false if none of it is over it. */
function overTriangle() {
  const nx = up[0], ny = up[1], nz = up[2];
  cut[0] = -1; cut[1] = -1;
  for (let e = 0; e < 3; e++) {
    const a = 3 * e, b = 3 * ((e + 1) % 3);
    const ex = P[b] - P[a], ey = P[b + 1] - P[a + 1], ez = P[b + 2] - P[a + 2];
    // Pointing into the triangle, whichever way its corners run around `up`.
    let mx = ny * ez - nz * ey, my = nz * ex - nx * ez, mz = nx * ey - ny * ex;
    const cx = P[3 * ((e + 2) % 3)] - P[a], cy = P[3 * ((e + 2) % 3) + 1] - P[a + 1], cz = P[3 * ((e + 2) % 3) + 2] - P[a + 2];
    if (mx * cx + my * cy + mz * cz < 0) { mx = -mx; my = -my; mz = -mz; }
    const f0 = (seg[0] - P[a]) * mx + (seg[1] - P[a + 1]) * my + (seg[2] - P[a + 2]) * mz;
    const f1 = (seg[3] - P[a]) * mx + (seg[4] - P[a + 1]) * my + (seg[5] - P[a + 2]) * mz;
    if (f0 < 0 && f1 < 0) return false;
    if (f0 < 0 || f1 < 0) {
      const u = f0 / (f0 - f1), into = f0 < 0 ? 0 : 3;
      cut[into / 3] = e;
      seg[into] = seg[0] + u * (seg[3] - seg[0]);
      seg[into + 1] = seg[1] + u * (seg[4] - seg[1]);
      seg[into + 2] = seg[2] + u * (seg[5] - seg[2]);
    }
  }
  return true;
}

function capsule(s, t, margin, tolerance, out) {
  const c = s.center, a = s.axes, r = s.half[0], h = s.half[1];
  // The closest points of the cores: the capsule's segment and the triangle.
  const d = closest(s, t, near);
  let nx, ny, nz, sep, where;
  const apart = d > r / 67108864;
  if (apart) {
    nx = (near.a[0] - near.b[0]) / d; ny = (near.a[1] - near.b[1]) / d; nz = (near.a[2] - near.b[2]) / d;
    sep = d - r;
    where = region(near.b[0], near.b[1], near.b[2], tolerance);
  }
  // Lying over the triangle: its segment, clipped to the prism over the
  // triangle, rests on the face at both ends, as a capsule lies on a box.
  seg[0] = c[0] - h * a[3]; seg[1] = c[1] - h * a[4]; seg[2] = c[2] - h * a[5];
  seg[3] = c[0] + h * a[3]; seg[4] = c[1] + h * a[4]; seg[5] = c[2] + h * a[5];
  const e0 = above(seg[0], seg[1], seg[2]), e1 = above(seg[3], seg[4], seg[5]);
  const over = overTriangle();
  // The side the part over the triangle is on: the centre's, unless all of
  // it is on the other, as for a capsule that crosses the triangle's plane
  // just beside it, centre beyond.
  const h0 = over ? above(seg[0], seg[1], seg[2]) : 0, h1 = over ? above(seg[3], seg[4], seg[5]) : 0;
  const k = h0 < 0 && h1 < 0 ? -1 : 1;
  const s0 = over ? k * h0 - r : Infinity, s1 = over ? k * h1 - r : Infinity;
  const face = Math.min(s0, s1);
  // Apart, the face is taken over the closest points when it is as near, to
  // the tolerance, so a capsule lying on it keeps two points; and only when
  // the whole capsule reaches no deeper along it, or one tilted down past the
  // triangle's edge, nearest the edge, would be pushed the long way out.
  const whole = Math.min(k * e0, k * e1) - r;
  if (over && (!apart || seam(where) || (face <= sep + tolerance && whole >= sep - tolerance))) {
    const ux = k * up[0], uy = k * up[1], uz = k * up[2];
    aim(out, -ux, -uy, -uz);
    for (let e = 0; e < 2; e++) {
      const se = e === 0 ? s0 : s1;
      // An end a seam cut back lies inside the line the capsule rests along,
      // not at its end: the triangle beside carries on from it (see SEAMS).
      if (se > margin || (cut[e] >= 0 && (corners >> cut[e] & 1) === 0)) continue;
      const m = r + se / 2;
      add(out, seg[3 * e] - ux * m, seg[3 * e + 1] - uy * m, seg[3 * e + 2] - uz * m, se, 16 + e);
    }
    return;
  }
  // Nearest at a seam, and not over this triangle: it is over the triangle
  // beside, whose face holds it, as for a box (see closestPoints).
  if (!apart || seam(where) || sep > margin) return;
  aim(out, -nx, -ny, -nz);
  const m = r + sep / 2;
  add(out, near.a[0] - nx * m, near.a[1] - ny * m, near.a[2] - nz * m, sep, where);
}

// ---- a box ----

/** How far a box reaches from its centre along unit (x, y, z). */
function reach(b, x, y, z) {
  const A = b.axes, h = b.half;
  return Math.abs(A[0] * x + A[1] * y + A[2] * z) * h[0] + Math.abs(A[3] * x + A[4] * y + A[5] * z) * h[1]
    + Math.abs(A[6] * x + A[7] * y + A[8] * z) * h[2];
}

/** The triangle's least reach past the box's centre along unit (x, y, z): its nearest corner that way. */
function lowest(c, x, y, z) {
  let m = Infinity;
  for (let v = 0; v < 3; v++) m = Math.min(m, (P[3 * v] - c[0]) * x + (P[3 * v + 1] - c[1]) * y + (P[3 * v + 2] - c[2]) * z);
  return m;
}

/** 2^-27: cross products shorter than this are parallel edges, and give no axis. As collide.js's PARALLEL. */
const PARALLEL = 7.450580596923828e-9;

function box(b, t, margin, tolerance, out) {
  const c = b.center, A = b.axes, h = b.half;
  // The triangle's face, turned to the box.
  const faceT = above(c[0], c[1], c[2]) - reach(b, up[0], up[1], up[2]);
  if (faceT > margin) return;
  // The box's faces, each way along its axis. Both ways: a triangle isn't
  // the same either side of its centre, as a box is, so the side its centre
  // is on needn't be the side it overlaps the box least from.
  let faceB = -Infinity, axisB = 0, signB = 1;
  for (let k = 0; k < 3; k++) {
    const ux = A[3 * k], uy = A[3 * k + 1], uz = A[3 * k + 2];
    const plus = lowest(c, ux, uy, uz) - h[k], minus = lowest(c, -ux, -uy, -uz) - h[k];
    const sk = Math.max(plus, minus);
    if (sk > margin) return;
    if (sk > faceB) { faceB = sk; axisB = k; signB = plus >= minus ? 1 : -1; }
  }
  // Each box edge against each triangle edge.
  let edgeSep = -Infinity, ex = 0, ey = 0, ez = 0, edgeK = -1, edgeE = -1;
  for (let k = 0; k < 3; k++) {
    const ux = A[3 * k], uy = A[3 * k + 1], uz = A[3 * k + 2];
    for (let e = 0; e < 3; e++) {
      const a = 3 * e, q = 3 * ((e + 1) % 3);
      const vx = P[q] - P[a], vy = P[q + 1] - P[a + 1], vz = P[q + 2] - P[a + 2];
      let mx = uy * vz - uz * vy, my = uz * vx - ux * vz, mz = ux * vy - uy * vx;
      const l = Math.sqrt(mx * mx + my * my + mz * mz), lv = Math.sqrt(vx * vx + vy * vy + vz * vz);
      if (!(l > PARALLEL * lv)) continue;
      mx /= l; my /= l; mz /= l;
      const r = reach(b, mx, my, mz), plus = lowest(c, mx, my, mz) - r, minus = lowest(c, -mx, -my, -mz) - r;
      if (minus > plus) { mx = -mx; my = -my; mz = -mz; }
      const se = Math.max(plus, minus);
      if (se > margin) return;
      if (se > edgeSep) { edgeSep = se; ex = mx; ey = my; ez = mz; edgeK = k; edgeE = e; }
    }
  }
  // A face is kept unless an edge, or the box's own face, beats it by more than
  // the tolerance: on a tie the triangle's face, so a box resting on a floor
  // keeps the same contact from step to step.
  if (edgeK >= 0 && edgeSep > Math.max(faceT, faceB) + tolerance) edgeContact(b, edgeK, edgeE, ex, ey, ez, margin, tolerance, out);
  else if (faceB > faceT + tolerance) boxFace(b, axisB, signB, margin, tolerance, out);
  // The triangle's face, chosen, or because the box's face or an edge
  // touched only at seams, where the face beside takes it.
  if (out.count === 0) triangleFace(b, margin, tolerance, out);
  // Apart, and nothing clipped within the margin: nearest at a corner or an
  // edge's end, as boxBoxClosest in collide.js finds it.
  if (out.count === 0 && Math.max(faceT, faceB, edgeSep) > 0) closestPoints(b, t, margin, tolerance, out);
}

// Polygons, as flat lists of points, and the clip between them.
const polyA = new Float64Array(3 * 12), polyB = new Float64Array(3 * 12);
const idA = new Uint32Array(12), idB = new Uint32Array(12);

/** Clips the polygon in (src, srcId, n) to (x - p)·m >= 0, into (dst, dstId); returns its count. */
function clip(src, srcId, n, dst, dstId, px, py, pz, mx, my, mz, plane) {
  let k = 0;
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    const fi = (src[3 * i] - px) * mx + (src[3 * i + 1] - py) * my + (src[3 * i + 2] - pz) * mz;
    const fj = (src[3 * j] - px) * mx + (src[3 * j + 1] - py) * my + (src[3 * j + 2] - pz) * mz;
    if (fi >= 0) {
      dst[3 * k] = src[3 * i]; dst[3 * k + 1] = src[3 * i + 1]; dst[3 * k + 2] = src[3 * i + 2]; dstId[k++] = srcId[i];
    }
    if ((fi >= 0) !== (fj >= 0)) {
      const u = fi / (fi - fj);
      dst[3 * k] = src[3 * i] + u * (src[3 * j] - src[3 * i]);
      dst[3 * k + 1] = src[3 * i + 1] + u * (src[3 * j + 1] - src[3 * i + 1]);
      dst[3 * k + 2] = src[3 * i + 2] + u * (src[3 * j + 2] - src[3 * i + 2]);
      // A new point is named by the plane that made it and the edge it is on.
      dstId[k++] = 256 + 16 * plane + i;
    }
  }
  return k;
}

const CP = [-1, 1, 1, -1], CQ = [-1, -1, 1, 1];   // a face's corners, in order around it

// The clip buffers: what was written is read next. Module scratch, as everything here.
let src = polyA, dst = polyB, srcId = idA, dstId = idB;
function flip() {
  const p = src; src = dst; dst = p;
  const q = srcId; srcId = dstId; dstId = q;
}

/** The triangle's face as the reference: the box's face most against it, clipped to the triangle. */
function triangleFace(b, margin, tolerance, out) {
  const c = b.center, A = b.axes, h = b.half;
  let k = 0, most = 0;
  for (let j = 0; j < 3; j++) {
    const d = A[3 * j] * up[0] + A[3 * j + 1] * up[1] + A[3 * j + 2] * up[2];
    if (Math.abs(d) > Math.abs(most)) { most = d; k = j; }
  }
  // The face whose outward normal points down into the triangle.
  const side = most > 0 ? -1 : 1, p = (k + 1) % 3, q = (k + 2) % 3;
  src = polyA; dst = polyB; srcId = idA; dstId = idB;
  for (let i = 0; i < 4; i++) {
    for (let x = 0; x < 3; x++) {
      src[3 * i + x] = c[x] + side * h[k] * A[3 * k + x] + CP[i] * h[p] * A[3 * p + x] + CQ[i] * h[q] * A[3 * q + x];
    }
    srcId[i] = i;
  }
  let n = 4;
  for (let e = 0; e < 3 && n > 0; e++) {
    const a = 3 * e, q2 = 3 * ((e + 1) % 3), o = 3 * ((e + 2) % 3);
    const vx = P[q2] - P[a], vy = P[q2 + 1] - P[a + 1], vz = P[q2 + 2] - P[a + 2];
    let mx = up[1] * vz - up[2] * vy, my = up[2] * vx - up[0] * vz, mz = up[0] * vy - up[1] * vx;
    if (mx * (P[o] - P[a]) + my * (P[o + 1] - P[a + 1]) + mz * (P[o + 2] - P[a + 2]) < 0) { mx = -mx; my = -my; mz = -mz; }
    n = clip(src, srcId, n, dst, dstId, P[a], P[a + 1], P[a + 2], mx, my, mz, e);
    flip();
  }
  for (let i = 0; i < n; i++) {
    const x = src[3 * i], y = src[3 * i + 1], z = src[3 * i + 2], sep = above(x, y, z);
    if (sep > margin) continue;
    const id = srcId[i];
    // The box's own corners always count; a point clipping made on a seam doesn't (see SEAMS).
    if (id >= 4 && seam(region(x, y, z, tolerance))) continue;
    add(out, x - up[0] * sep / 2, y - up[1] * sep / 2, z - up[2] * sep / 2, sep, id < 4 ? 32 + id : mix + 32 + id);
  }
  if (out.count > 0) aim(out, -up[0], -up[1], -up[2]);
}

/** The box's face k (outward along sign · axis k) as the reference: the triangle clipped to it. */
function boxFace(b, k, sign, margin, tolerance, out) {
  const c = b.center, A = b.axes, h = b.half;
  const ux = sign * A[3 * k], uy = sign * A[3 * k + 1], uz = sign * A[3 * k + 2];
  src = polyA; dst = polyB; srcId = idA; dstId = idB;
  for (let i = 0; i < 9; i++) src[i] = P[i];
  srcId[0] = 0; srcId[1] = 1; srcId[2] = 2;
  let n = 3;
  for (let plane = 0; plane < 4; plane++) {
    // Inside the box's sides: (x - c)·(s axis j) <= h_j, for both other axes j and both signs s.
    const j = (k + 1 + (plane >> 1)) % 3, s = plane & 1 ? -1 : 1;
    const mx = -s * A[3 * j], my = -s * A[3 * j + 1], mz = -s * A[3 * j + 2];
    n = clip(src, srcId, n, dst, dstId, c[0] + s * h[j] * A[3 * j], c[1] + s * h[j] * A[3 * j + 1], c[2] + s * h[j] * A[3 * j + 2], mx, my, mz, plane);
    flip();
    if (n === 0) return false;
  }
  for (let i = 0; i < n; i++) {
    const x = src[3 * i], y = src[3 * i + 1], z = src[3 * i + 2];
    const sep = (x - c[0]) * ux + (y - c[1]) * uy + (z - c[2]) * uz - h[k];
    if (sep > margin || seam(region(x, y, z, tolerance))) continue;
    add(out, x - ux * sep / 2, y - uy * sep / 2, z - uz * sep / 2, sep, mix + 64 + srcId[i]);
  }
  // Touching the box's face only where this triangle has seams: the
  // triangle's face takes it (the caller's fallback).
  if (out.count === 0) return false;
  aim(out, ux, uy, uz);
  return true;
}

/** A box edge against a triangle edge: their closest points, one contact point. */
function edgeContact(b, k, e, nx, ny, nz, margin, tolerance, out) {
  const c = b.center, A = b.axes, h = b.half;
  // The box's edge along axis k that reaches furthest along n.
  let ox = c[0], oy = c[1], oz = c[2];
  for (let j = 0; j < 3; j++) {
    if (j === k) continue;
    const s = A[3 * j] * nx + A[3 * j + 1] * ny + A[3 * j + 2] * nz >= 0 ? 1 : -1;
    ox += s * h[j] * A[3 * j]; oy += s * h[j] * A[3 * j + 1]; oz += s * h[j] * A[3 * j + 2];
  }
  const ux = A[3 * k], uy = A[3 * k + 1], uz = A[3 * k + 2];
  const a = 3 * e, q = 3 * ((e + 1) % 3);
  const vx = P[q] - P[a], vy = P[q + 1] - P[a + 1], vz = P[q + 2] - P[a + 2];
  // Closest points of the segments o + s u, |s| <= h_k, and P_a + w v, w in [0, 1].
  const rx = ox - P[a], ry = oy - P[a + 1], rz = oz - P[a + 2];
  const vv = vx * vx + vy * vy + vz * vz, uv = ux * vx + uy * vy + uz * vz;
  const ur = ux * rx + uy * ry + uz * rz, vr = vx * rx + vy * ry + vz * rz;
  const den = vv - uv * uv;
  let s = den > 0 ? (uv * vr - vv * ur) / den : 0;
  s = Math.max(-h[k], Math.min(h[k], s));
  let w = (vr + s * uv) / vv;
  w = Math.max(0, Math.min(1, w));
  s = Math.max(-h[k], Math.min(h[k], uv * w - ur));
  const bx = ox + s * ux, by = oy + s * uy, bz = oz + s * uz;
  const tx = P[a] + w * vx, ty = P[a + 1] + w * vy, tz = P[a + 2] + w * vz;
  if (seam(region(tx, ty, tz, tolerance))) return false;
  const sep = (tx - bx) * nx + (ty - by) * ny + (tz - bz) * nz;
  if (sep > margin) return false;
  aim(out, nx, ny, nz);
  add(out, (bx + tx) / 2, (by + ty) / 2, (bz + tz) / 2, sep, mix + 128 + 4 * k + e);
  return true;
}

/**
 * Apart within the margin: the true closest points of the box and the
 * triangle, by GJK. None when the triangle's closest point is on a seam:
 * the box is then nearest to the triangle beside, and over it, where that
 * triangle's face finds its corner (triangleFace takes the box's lowest).
 * Turned into a face contact here instead, as a ball's is, it would land in
 * the middle of the box's face, and a box sliding over a seam's end would
 * get a fifth point there that it doesn't have on one big box.
 */
function closestPoints(b, t, margin, tolerance, out) {
  const d = closest(b, t, near);
  if (!(d > 0) || d > margin || seam(region(near.b[0], near.b[1], near.b[2], tolerance))) return;
  const nx = (near.b[0] - near.a[0]) / d, ny = (near.b[1] - near.a[1]) / d, nz = (near.b[2] - near.a[2]) / d;
  aim(out, nx, ny, nz);
  add(out, near.a[0] + nx * d / 2, near.a[1] + ny * d / 2, near.a[2] + nz * d / 2, d, mix + 255);
}

// ---- a hull ----

const H = new Placed();
let hullP = new Float64Array(3 * 16), hullQ = new Float64Array(3 * 16);
let hullPi = new Uint32Array(16), hullQi = new Uint32Array(16);

/** The triangle's least reach along unit (x, y, z), from the origin: its nearest corner that way. */
function low(x, y, z) {
  return Math.min(P[0] * x + P[1] * y + P[2] * z, P[3] * x + P[4] * y + P[5] * z, P[6] * x + P[7] * y + P[8] * z);
}

/**
 * A hull against the triangle, as box(): separating axes over the
 * triangle's face, the hull's faces, and each hull edge crossed with each
 * triangle edge, then clipping. Each axis's gap is measured by how far both
 * shapes reach along it, which is true for any axis, so no pair of edges
 * needs the pruning hulls against hulls use; it costs a pass over the hull's
 * corners for each.
 */
function hull(b, t, margin, tolerance, out) {
  H.set(b);
  const v = H.v, pl = H.planes, h = H.hull, F = h.planes.length / 4;
  // The triangle's face, turned to the hull: the hull's lowest corner above it.
  const faceT = H.lowest(up[0], up[1], up[2]) - (P[0] * up[0] + P[1] * up[1] + P[2] * up[2]);
  if (faceT > margin) return;
  // The hull's faces: the triangle's nearest corner above each.
  let faceB = -Infinity, fb = 0;
  for (let f = 0; f < F; f++) {
    const sb = low(pl[4 * f], pl[4 * f + 1], pl[4 * f + 2]) - pl[4 * f + 3];
    if (sb > margin) return;
    if (sb > faceB) { faceB = sb; fb = f; }
  }
  const e = h.edges, E = e.length / 2;
  let edgeSep = -Infinity, ex = 0, ey = 0, ez = 0, edgeK = -1, edgeE = -1;
  for (let k = 0; k < E; k++) {
    const i = 3 * e[2 * k], j = 3 * e[2 * k + 1];
    const ux = v[j] - v[i], uy = v[j + 1] - v[i + 1], uz = v[j + 2] - v[i + 2];
    const lu = Math.sqrt(ux * ux + uy * uy + uz * uz);
    for (let te = 0; te < 3; te++) {
      const a = 3 * te, q = 3 * ((te + 1) % 3);
      const wx = P[q] - P[a], wy = P[q + 1] - P[a + 1], wz = P[q + 2] - P[a + 2];
      let mx = uy * wz - uz * wy, my = uz * wx - ux * wz, mz = ux * wy - uy * wx;
      const l = Math.sqrt(mx * mx + my * my + mz * mz);
      if (!(l > PARALLEL * lu * Math.sqrt(wx * wx + wy * wy + wz * wz))) continue;
      mx /= l; my /= l; mz /= l;
      // The triangle past the hull along m, or along -m: whichever is apart more, and that way round.
      const plus = low(mx, my, mz) + H.lowest(-mx, -my, -mz), minus = low(-mx, -my, -mz) + H.lowest(mx, my, mz);
      if (minus > plus) { mx = -mx; my = -my; mz = -mz; }
      // Only the edge that reaches farthest towards the triangle: a box has
      // four edges along each axis, and three are behind along it.
      if (-H.lowest(-mx, -my, -mz) - (mx * v[i] + my * v[i + 1] + mz * v[i + 2]) > tolerance) continue;
      const sep = Math.max(plus, minus);
      if (sep > margin) return;
      if (sep > edgeSep) { edgeSep = sep; ex = mx; ey = my; ez = mz; edgeK = k; edgeE = te; }
    }
  }
  // As box(): a face unless an edge, or the hull's own face, beats the
  // triangle's by more than the tolerance; the triangle's face if those touch
  // only at seams; and GJK's closest points if apart with nothing clipped.
  if (edgeK >= 0 && edgeSep > Math.max(faceT, faceB) + tolerance) hullEdge(edgeK, edgeE, ex, ey, ez, margin, tolerance, out);
  else if (faceB > faceT + tolerance) hullFace(fb, margin, tolerance, out);
  if (out.count === 0) triangleFaceHull(margin, tolerance, out);
  if (out.count === 0 && Math.max(faceT, faceB, edgeSep) > 0) closestPoints(b, t, margin, tolerance, out);
}

/** Room for n points in the hull's clip buffers. */
function hullRoom(n) {
  if (hullPi.length >= n) return;
  let c = hullPi.length;
  while (c < n) c *= 2;
  hullP = new Float64Array(3 * c); hullQ = new Float64Array(3 * c);
  hullPi = new Uint32Array(c); hullQi = new Uint32Array(c);
}

/**
 * A hull's corner's id: past the ids a triangle's features take with `mix`
 * (multiples of 512, below 2^30 for a mesh of up to two million triangles),
 * and the same on every triangle, as a box's corners' are, so a hull sliding
 * from triangle to triangle keeps its impulses.
 */
const CORNER = 1 << 30;

/** The triangle's face as the reference: the hull's face most against it, clipped to the triangle. */
function triangleFaceHull(margin, tolerance, out) {
  const h = H.hull, pl = H.planes, v = H.v, F = h.planes.length / 4;
  let f = 0, least = Infinity;
  for (let k = 0; k < F; k++) {
    const d = pl[4 * k] * up[0] + pl[4 * k + 1] * up[1] + pl[4 * k + 2] * up[2];
    if (d < least) { least = d; f = k; }
  }
  const i0 = h.start[f], m = h.start[f + 1] - i0;
  hullRoom(m + 3);
  src = hullP; dst = hullQ; srcId = hullPi; dstId = hullQi;
  for (let k = 0; k < m; k++) {
    const i = 3 * h.loops[i0 + k];
    src[3 * k] = v[i]; src[3 * k + 1] = v[i + 1]; src[3 * k + 2] = v[i + 2];
    srcId[k] = CORNER + h.loops[i0 + k];
  }
  let n = m;
  for (let e = 0; e < 3 && n > 0; e++) {
    const a = 3 * e, q2 = 3 * ((e + 1) % 3), o = 3 * ((e + 2) % 3);
    const vx = P[q2] - P[a], vy = P[q2 + 1] - P[a + 1], vz = P[q2 + 2] - P[a + 2];
    let mx = up[1] * vz - up[2] * vy, my = up[2] * vx - up[0] * vz, mz = up[0] * vy - up[1] * vx;
    if (mx * (P[o] - P[a]) + my * (P[o + 1] - P[a + 1]) + mz * (P[o + 2] - P[a + 2]) < 0) { mx = -mx; my = -my; mz = -mz; }
    n = clip(src, srcId, n, dst, dstId, P[a], P[a + 1], P[a + 2], mx, my, mz, e);
    flip();
  }
  for (let k = 0; k < n; k++) {
    const x = src[3 * k], y = src[3 * k + 1], z = src[3 * k + 2], sep = above(x, y, z);
    if (sep > margin) continue;
    const id = srcId[k], own = id >= CORNER;
    // The hull's own corners always count; a point clipping made on a seam doesn't (see SEAMS).
    if (!own && seam(region(x, y, z, tolerance))) continue;
    add(out, x - up[0] * sep / 2, y - up[1] * sep / 2, z - up[2] * sep / 2, sep, own ? id : mix + 32 + id);
  }
  if (out.count > 0) aim(out, -up[0], -up[1], -up[2]);
}

/** The hull's face f as the reference: the triangle clipped to its sides. */
function hullFace(f, margin, tolerance, out) {
  const h = H.hull, pl = H.planes, v = H.v;
  const nx = pl[4 * f], ny = pl[4 * f + 1], nz = pl[4 * f + 2], d = pl[4 * f + 3];
  const i0 = h.start[f], m = h.start[f + 1] - i0;
  hullRoom(m + 3);
  src = hullP; dst = hullQ; srcId = hullPi; dstId = hullQi;
  for (let i = 0; i < 9; i++) src[i] = P[i];
  srcId[0] = 0; srcId[1] = 1; srcId[2] = 2;
  let n = 3;
  for (let k = 0; k < m && n > 0; k++) {
    const p = 3 * h.loops[i0 + k], q = 3 * h.loops[i0 + (k + 1 < m ? k + 1 : 0)];
    const wx = v[q] - v[p], wy = v[q + 1] - v[p + 1], wz = v[q + 2] - v[p + 2];
    // Inward: the face's corners run counter-clockwise seen from outside, so n × edge points in.
    const mx = ny * wz - nz * wy, my = nz * wx - nx * wz, mz = nx * wy - ny * wx;
    n = clip(src, srcId, n, dst, dstId, v[p], v[p + 1], v[p + 2], mx, my, mz, k & 15);
    flip();
  }
  for (let k = 0; k < n; k++) {
    const x = src[3 * k], y = src[3 * k + 1], z = src[3 * k + 2];
    const sep = x * nx + y * ny + z * nz - d;
    if (sep > margin || seam(region(x, y, z, tolerance))) continue;
    add(out, x - nx * sep / 2, y - ny * sep / 2, z - nz * sep / 2, sep, mix + 64 + srcId[k]);
  }
  if (out.count > 0) aim(out, nx, ny, nz);
}

/** Hull edge k against triangle edge e, along n from the hull to the triangle: one point, where they pass closest. */
function hullEdge(k, e, nx, ny, nz, margin, tolerance, out) {
  const v = H.v, ed = H.hull.edges, i = 3 * ed[2 * k], j = 3 * ed[2 * k + 1];
  const ox = v[i], oy = v[i + 1], oz = v[i + 2];
  const ux = v[j] - ox, uy = v[j + 1] - oy, uz = v[j + 2] - oz;
  const a = 3 * e, q = 3 * ((e + 1) % 3);
  const vx = P[q] - P[a], vy = P[q + 1] - P[a + 1], vz = P[q + 2] - P[a + 2];
  // Closest points of o + s u and P_a + w v, s and w in [0, 1] (Ericson, 5.1.9).
  const rx = ox - P[a], ry = oy - P[a + 1], rz = oz - P[a + 2];
  const uu = ux * ux + uy * uy + uz * uz, vv = vx * vx + vy * vy + vz * vz, uv = ux * vx + uy * vy + uz * vz;
  const ur = ux * rx + uy * ry + uz * rz, vr = vx * rx + vy * ry + vz * rz;
  const den = uu * vv - uv * uv;
  let s = den > 0 ? Math.max(0, Math.min(1, (uv * vr - vv * ur) / den)) : 0;
  let w = (uv * s + vr) / vv;
  if (w < 0) { w = 0; s = Math.max(0, Math.min(1, -ur / uu)); } else if (w > 1) { w = 1; s = Math.max(0, Math.min(1, (uv - ur) / uu)); }
  const bx = ox + s * ux, by = oy + s * uy, bz = oz + s * uz;
  const tx = P[a] + w * vx, ty = P[a + 1] + w * vy, tz = P[a + 2] + w * vz;
  if (seam(region(tx, ty, tz, tolerance))) return;
  const sep = (tx - bx) * nx + (ty - by) * ny + (tz - bz) * nz;
  if (sep > margin) return;
  // Apart and nearest off an edge's end: the closest points take it.
  if (sep > 0 && (s === 0 || s === 1 || w === 0 || w === 1)) return;
  aim(out, nx, ny, nz);
  add(out, (bx + tx) / 2, (by + ty) / 2, (bz + tz) / 2, sep, mix + 384 + ((3 * k + e) & 127));
}

// ---- a whole mesh ----

/**
 * Normals this close count as one direction: within FLAT of each other as an
 * angle, the flatness that makes a seam (mesh.js), since cos θ ≥ 1 - θ²/2.
 */
const SAME = FLAT * FLAT / 2;

/** What touchLevel finds: `count` manifolds, in `list`, grown as needed and reused. */
export class Touches {
  constructor() {
    this.count = 0;
    this.list = [];
  }

  _next() {
    if (this.count === this.list.length) this.list.push(new Manifold());
    return this.list[this.count++];
  }
}

const tri = new Shape();
tri.kind = TRIANGLE;
const local = new Float64Array(9);
const one = {
  count: 0, normal: new Float64Array(3), points: new Float64Array(3 * MOST),
  separations: new Float64Array(MOST), ids: new Uint32Array(MOST),
};
// Every point found, before reducing: grown by doubling, never sized in advance.
let found = 0, groups = 0;
let foundAt = new Float64Array(3 * 16), foundSep = new Float64Array(16), foundId = new Uint32Array(16), foundIn = new Int32Array(16);
let normals = new Float64Array(3 * 4);
// One manifold's points, gathered for reduce().
let gathered = new Float64Array(3 * 16), gatheredSep = new Float64Array(16), gatheredId = new Uint32Array(16);

// What the visitor needs; touchLevel() is never re-entered.
let geometry = null, center = null, axes = null, base = 0, shape = null, margin_ = 0, tolerance_ = 0;

/** Triangle t of the mesh, placed in the world and touched; its points join the manifold that pushes their way. */
function visit(t) {
  geometry.triangle(t, local);
  const c = center, A = axes, o3 = 3 * base, o9 = 9 * base, p = tri.points;
  for (let v = 0; v < 3; v++) {
    const x = local[3 * v], y = local[3 * v + 1], z = local[3 * v + 2];
    p[3 * v] = c[o3] + A[o9] * x + A[o9 + 3] * y + A[o9 + 6] * z;
    p[3 * v + 1] = c[o3 + 1] + A[o9 + 1] * x + A[o9 + 4] * y + A[o9 + 7] * z;
    p[3 * v + 2] = c[o3 + 2] + A[o9 + 2] * x + A[o9 + 5] * y + A[o9 + 8] * z;
  }
  tri.center[0] = (p[0] + p[3] + p[6]) / 3;
  tri.center[1] = (p[1] + p[4] + p[7]) / 3;
  tri.center[2] = (p[2] + p[5] + p[8]) / 3;
  triangleNormal(p, tri.axes);
  if (touch(shape, tri, geometry.edge(t), margin_, tolerance_, one, t) === 0) return;
  const nx = one.normal[0], ny = one.normal[1], nz = one.normal[2];
  let g = 0;
  while (g < groups && normals[3 * g] * nx + normals[3 * g + 1] * ny + normals[3 * g + 2] * nz < 1 - SAME) g++;
  if (g === groups) {
    if (3 * groups === normals.length) normals = resized(normals, 2 * normals.length);
    normals[3 * g] = nx; normals[3 * g + 1] = ny; normals[3 * g + 2] = nz;
    groups++;
  }
  const t2 = tolerance_ * tolerance_;
  for (let k = 0; k < one.count; k++) {
    const x = one.points[3 * k], y = one.points[3 * k + 1], z = one.points[3 * k + 2], sep = one.separations[k];
    // The same point from the triangle beside, as where a ball rests on the
    // edge between two: kept once, the deeper.
    let same = -1;
    for (let i = 0; i < found && same < 0; i++) {
      if (foundIn[i] !== g) continue;
      const dx = foundAt[3 * i] - x, dy = foundAt[3 * i + 1] - y, dz = foundAt[3 * i + 2] - z;
      if (dx * dx + dy * dy + dz * dz <= t2) same = i;
    }
    if (same >= 0) {
      if (sep < foundSep[same]) {
        foundAt[3 * same] = x; foundAt[3 * same + 1] = y; foundAt[3 * same + 2] = z; foundSep[same] = sep;
      }
      // A name from the shape's own features (below 512, unmixed) outlasts
      // one from a triangle's, since it stays the same as the shape crosses
      // from triangle to triangle, and its impulse carries with it.
      if (one.ids[k] < 512 && foundId[same] >= 512) foundId[same] = one.ids[k];
      continue;
    }
    if (found === foundSep.length) {
      foundAt = resized(foundAt, 6 * found); foundSep = resized(foundSep, 2 * found);
      foundId = resized(foundId, 2 * found); foundIn = resized(foundIn, 2 * found);
    }
    foundAt[3 * found] = x; foundAt[3 * found + 1] = y; foundAt[3 * found + 2] = z;
    foundSep[found] = sep; foundId[found] = one.ids[k]; foundIn[found] = g;
    found++;
  }
}

/**
 * Where shape s touches mesh or heightfield g, into `out`: one manifold for
 * each direction it is pushed in, each of up to four points, normals from s
 * towards the mesh. The mesh's body is slot o: its position at c[3o..] and its
 * axes at A[9o..]. (x0, y0, z0)-(x1, y1, z1) is the box to look in, in the
 * mesh's own frame. Returns out.count.
 */
export function touchLevel(g, c, A, o, s, x0, y0, z0, x1, y1, z1, margin, tolerance, out) {
  geometry = g; center = c; axes = A; base = o; shape = s; margin_ = margin; tolerance_ = tolerance;
  found = 0; groups = 0;
  g.triangles(x0, y0, z0, x1, y1, z1, visit);
  out.count = 0;
  for (let k = 0; k < groups; k++) {
    let m = 0;
    for (let i = 0; i < found; i++) {
      if (foundIn[i] !== k) continue;
      if (m === gatheredSep.length) {
        gathered = resized(gathered, 6 * m); gatheredSep = resized(gatheredSep, 2 * m); gatheredId = resized(gatheredId, 2 * m);
      }
      // In order of id, whatever order the triangles came in: a box's corners
      // then come in the order its face lists them, as they do on a box
      // (collide.js), and the solver, which visits points in turn, does the same.
      let j = m++;
      for (; j > 0 && gatheredId[j - 1] > foundId[i]; j--) {
        gathered[3 * j] = gathered[3 * j - 3]; gathered[3 * j + 1] = gathered[3 * j - 2]; gathered[3 * j + 2] = gathered[3 * j - 1];
        gatheredSep[j] = gatheredSep[j - 1]; gatheredId[j] = gatheredId[j - 1];
      }
      gathered[3 * j] = foundAt[3 * i]; gathered[3 * j + 1] = foundAt[3 * i + 1]; gathered[3 * j + 2] = foundAt[3 * i + 2];
      gatheredSep[j] = foundSep[i]; gatheredId[j] = foundId[i];
    }
    const mf = out._next();
    mf.count = 0;
    mf.normal[0] = normals[3 * k]; mf.normal[1] = normals[3 * k + 1]; mf.normal[2] = normals[3 * k + 2];
    reduce(m, mf);
  }
  geometry = null; center = null; axes = null; shape = null;
  return out.count;
}

/**
 * At most four of the m gathered points into out: the deepest; the one
 * farthest from it; the one making the largest triangle with those two; and
 * the one making the largest on the other side. collide.js's choice for
 * boxes (Gregorius), so a box on a mesh holds as it does on a box.
 */
function reduce(m, out) {
  const X = gathered, S = gatheredSep;
  if (m <= MAX_POINTS) {
    for (let k = 0; k < m; k++) keep(out, k);
    return;
  }
  let a = 0;
  for (let k = 1; k < m; k++) if (S[k] < S[a]) a = k;
  let b = -1, far = -1;
  for (let k = 0; k < m; k++) {
    const x = X[3 * k] - X[3 * a], y = X[3 * k + 1] - X[3 * a + 1], z = X[3 * k + 2] - X[3 * a + 2];
    const d = x * x + y * y + z * z;
    if (d > far) { far = d; b = k; }
  }
  const nx = out.normal[0], ny = out.normal[1], nz = out.normal[2];
  let c = -1, d = -1, most = 0, least = 0;
  const abx = X[3 * b] - X[3 * a], aby = X[3 * b + 1] - X[3 * a + 1], abz = X[3 * b + 2] - X[3 * a + 2];
  for (let k = 0; k < m; k++) {
    if (k === a || k === b) continue;
    const x = X[3 * k] - X[3 * a], y = X[3 * k + 1] - X[3 * a + 1], z = X[3 * k + 2] - X[3 * a + 2];
    // Twice the triangle's signed area, measured along the normal.
    const area = (aby * z - abz * y) * nx + (abz * x - abx * z) * ny + (abx * y - aby * x) * nz;
    if (area > most) { most = area; c = k; }
    if (area < least) { least = area; d = k; }
  }
  keep(out, a);
  if (b !== a) keep(out, b);
  if (c >= 0) keep(out, c);
  if (d >= 0) keep(out, d);
}

function keep(out, k) {
  add(out, gathered[3 * k], gathered[3 * k + 1], gathered[3 * k + 2], gatheredSep[k], gatheredId[k]);
}
