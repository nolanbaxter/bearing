// GJK: how far apart two convex shapes are, and their closest points.
//
// Gilbert, Johnson and Keerthi, 1988. The distance between two convex shapes
// A and B is the distance from the origin to their Minkowski difference,
// every point of A minus every point of B, which is convex too. GJK never
// builds that set. It only asks each shape for its support point, the point
// farthest along a direction, and keeps a simplex (a point, a segment, a
// triangle or a tetrahedron) of points of the difference, moved each round
// towards the origin, until no support point gets any closer. A shape needs
// nothing but a support function to take part, which is why the same code
// will serve convex hulls (milestone 14) and continuous collision (13).
//
// CORES. A sphere is a point and a capsule a segment, each with a radius
// around it; a box is its own core, radius 0. GJK runs on the cores, and the
// radii are taken off the distance after, which is exact for these shapes
// and quicker than giving GJK the round surfaces. Box2D does the same.
//
// Closest points on a triangle are Ericson's (Real-Time Collision Detection,
// 5.1.5): which vertex, edge or face of the simplex is nearest the origin,
// read from signs of dot products, with no division until the answer is
// known. Exact operations and square roots only, so the result is the same
// to the bit on every machine.

import { SPHERE, CAPSULE } from './collide.js';

/**
 * A round's progress counts while it brings the squared distance in by more
 * than a part in 2^30 of it: half a double's bits. Smaller than that is the
 * rounding in the dot products deciding, not the shapes, and continuing would
 * only go round in circles.
 */
const PROGRESS = 1 / 1073741824;   // 2^-30, written out: src/ uses no **

/**
 * Rounds at most. The difference of two boxes has 64 vertices, and each round
 * that makes progress moves to a better one, so a run that hasn't settled by
 * then is cycling on rounding: what it has is as close as it will get.
 */
const ROUNDS = 64;

// The simplex: up to four points of the difference, with the points of A and
// B each came from, for the closest points at the end. Module scratch, since
// nothing re-enters this.
const W = new Float64Array(12), PA = new Float64Array(12), PB = new Float64Array(12);
const L = new Float64Array(4);   // barycentric weights of the closest point
const SW = new Float64Array(12), SPA = new Float64Array(12), SPB = new Float64Array(12);
/** A tetrahedron's faces, each with the point opposite it: rising order, as keep() needs. */
const FACES = new Uint8Array([0, 1, 2, 3, 0, 1, 3, 2, 0, 2, 3, 1, 1, 2, 3, 0]);
let count = 0;
const sa = new Float64Array(3), sb = new Float64Array(3);

/** Where to put the result: the core distance and the closest points of each core. */
export class Closest {
  constructor() {
    this.distance = 0;
    this.a = new Float64Array(3);
    this.b = new Float64Array(3);
  }
}

/** The radius around a shape's core: a sphere's or capsule's radius, 0 for a box. */
export function radius(s) {
  return s.kind === SPHERE || s.kind === CAPSULE ? s.half[0] : 0;
}

/** The point of s's core farthest along (x, y, z), into out. */
function support(s, x, y, z, out) {
  const c = s.center, a = s.axes, h = s.half;
  if (s.kind === SPHERE) {
    out[0] = c[0]; out[1] = c[1]; out[2] = c[2];
  } else if (s.kind === CAPSULE) {
    const e = x * a[3] + y * a[4] + z * a[5] >= 0 ? h[1] : -h[1];
    out[0] = c[0] + e * a[3]; out[1] = c[1] + e * a[4]; out[2] = c[2] + e * a[5];
  } else {
    const ex = x * a[0] + y * a[1] + z * a[2] >= 0 ? h[0] : -h[0];
    const ey = x * a[3] + y * a[4] + z * a[5] >= 0 ? h[1] : -h[1];
    const ez = x * a[6] + y * a[7] + z * a[8] >= 0 ? h[2] : -h[2];
    out[0] = c[0] + ex * a[0] + ey * a[3] + ez * a[6];
    out[1] = c[1] + ex * a[1] + ey * a[4] + ez * a[7];
    out[2] = c[2] + ex * a[2] + ey * a[5] + ez * a[8];
  }
}

/**
 * The distance between a's and b's cores, and their closest points, into
 * out; returns the distance. 0 means the cores meet, and then the closest
 * points are a point they share, as near as GJK got.
 */
export function closest(a, b, out) {
  // Start from the difference of the centres, any point of it would do.
  let vx = a.center[0] - b.center[0], vy = a.center[1] - b.center[1], vz = a.center[2] - b.center[2];
  if (vx === 0 && vy === 0 && vz === 0) vx = 1;
  count = 0;
  add(a, b, -vx, -vy, -vz);
  L[0] = 1;
  vx = W[0]; vy = W[1]; vz = W[2];
  for (let round = 0; round < ROUNDS; round++) {
    const vv = vx * vx + vy * vy + vz * vz;
    if (vv === 0) break;
    // The support point of the difference farthest towards the origin.
    support(a, -vx, -vy, -vz, sa);
    support(b, vx, vy, vz, sb);
    const wx = sa[0] - sb[0], wy = sa[1] - sb[1], wz = sa[2] - sb[2];
    if (vv - (vx * wx + vy * wy + vz * wz) <= PROGRESS * vv) break;
    let repeat = false;
    for (let i = 0; i < count; i++) {
      if (W[3 * i] === wx && W[3 * i + 1] === wy && W[3 * i + 2] === wz) repeat = true;
    }
    if (repeat) break;
    const i3 = 3 * count;
    W[i3] = wx; W[i3 + 1] = wy; W[i3 + 2] = wz;
    PA[i3] = sa[0]; PA[i3 + 1] = sa[1]; PA[i3 + 2] = sa[2];
    PB[i3] = sb[0]; PB[i3 + 1] = sb[1]; PB[i3 + 2] = sb[2];
    count++;
    if (!reduce()) { vx = vy = vz = 0; break; }   // the origin is inside: the cores meet
    vx = 0; vy = 0; vz = 0;
    for (let i = 0; i < count; i++) {
      vx += L[i] * W[3 * i]; vy += L[i] * W[3 * i + 1]; vz += L[i] * W[3 * i + 2];
    }
  }
  let ax = 0, ay = 0, az = 0, bx = 0, by = 0, bz = 0;
  for (let i = 0; i < count; i++) {
    ax += L[i] * PA[3 * i]; ay += L[i] * PA[3 * i + 1]; az += L[i] * PA[3 * i + 2];
    bx += L[i] * PB[3 * i]; by += L[i] * PB[3 * i + 1]; bz += L[i] * PB[3 * i + 2];
  }
  out.a[0] = ax; out.a[1] = ay; out.a[2] = az;
  out.b[0] = bx; out.b[1] = by; out.b[2] = bz;
  out.distance = Math.sqrt(vx * vx + vy * vy + vz * vz);
  return out.distance;
}

/** The first simplex point: a's support along (x, y, z) less b's against it. */
function add(a, b, x, y, z) {
  support(a, x, y, z, sa);
  support(b, -x, -y, -z, sb);
  W[0] = sa[0] - sb[0]; W[1] = sa[1] - sb[1]; W[2] = sa[2] - sb[2];
  PA[0] = sa[0]; PA[1] = sa[1]; PA[2] = sa[2];
  PB[0] = sb[0]; PB[1] = sb[1]; PB[2] = sb[2];
  count = 1;
}

/**
 * Shrinks the simplex to the smallest part of it nearest the origin, and sets
 * L to the weights of that nearest point. Returns false if the origin is
 * inside a tetrahedron, which means the shapes' cores overlap.
 */
function reduce() {
  if (count === 2) return segment(0, 1);
  if (count === 3) { triangle(0, 1, 2); return true; }
  return tetrahedron();
}

/**
 * Keeps the first n of the simplex points i, j, k, with weights li, lj, lk.
 * Every caller passes them in rising order, so moving each down into place
 * never overwrites one still to be moved.
 */
function keep(i, j, k, li, lj, lk, n) {
  move(0, i); L[0] = li;
  if (n > 1) { move(1, j); L[1] = lj; }
  if (n > 2) { move(2, k); L[2] = lk; }
  count = n;
}

function move(to, from) {
  if (to === from) return;
  for (let c = 0; c < 3; c++) {
    W[3 * to + c] = W[3 * from + c]; PA[3 * to + c] = PA[3 * from + c]; PB[3 * to + c] = PB[3 * from + c];
  }
}

function segment(i, j) {
  const ax = W[3 * i], ay = W[3 * i + 1], az = W[3 * i + 2];
  const ex = W[3 * j] - ax, ey = W[3 * j + 1] - ay, ez = W[3 * j + 2] - az;
  const ee = ex * ex + ey * ey + ez * ez;
  const t = ee > 0 ? -(ax * ex + ay * ey + az * ez) / ee : 0;
  if (t <= 0) keep(i, i, i, 1, 0, 0, 1);
  else if (t >= 1) keep(j, j, j, 1, 0, 0, 1);
  else keep(i, j, j, 1 - t, t, 0, 2);
  return true;
}

/** Ericson's closest point on triangle (i, j, k) to the origin, reducing the simplex to its region. */
function triangle(i, j, k) {
  const ax = W[3 * i], ay = W[3 * i + 1], az = W[3 * i + 2];
  const bx = W[3 * j], by = W[3 * j + 1], bz = W[3 * j + 2];
  const cx = W[3 * k], cy = W[3 * k + 1], cz = W[3 * k + 2];
  const abx = bx - ax, aby = by - ay, abz = bz - az, acx = cx - ax, acy = cy - ay, acz = cz - az;
  const d1 = -(abx * ax + aby * ay + abz * az), d2 = -(acx * ax + acy * ay + acz * az);
  if (d1 <= 0 && d2 <= 0) return keep(i, i, i, 1, 0, 0, 1);
  const d3 = -(abx * bx + aby * by + abz * bz), d4 = -(acx * bx + acy * by + acz * bz);
  if (d3 >= 0 && d4 <= d3) return keep(j, j, j, 1, 0, 0, 1);
  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) { const v = d1 / (d1 - d3); return keep(i, j, j, 1 - v, v, 0, 2); }
  const d5 = -(abx * cx + aby * cy + abz * cz), d6 = -(acx * cx + acy * cy + acz * cz);
  if (d6 >= 0 && d5 <= d6) return keep(k, k, k, 1, 0, 0, 1);
  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) { const w = d2 / (d2 - d6); return keep(i, k, k, 1 - w, w, 0, 2); }
  const va = d3 * d6 - d5 * d4;
  if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
    const w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return keep(j, k, k, 1 - w, w, 0, 2);
  }
  const s = 1 / (va + vb + vc), v = vb * s, w = vc * s;
  return keep(i, j, k, 1 - v - w, v, w, 3);
}

/**
 * The tetrahedron's nearest face to the origin, or false if the origin is
 * inside it. A face counts only if the origin is on its far side from the
 * fourth point; of those, the one whose nearest point is nearest wins.
 */
function tetrahedron() {
  let best = Infinity, bestFace = -1;
  // Saved, since triangle() rewrites the simplex as it reduces it.
  const w = SW, pa = SPA, pb = SPB;
  w.set(W); pa.set(PA); pb.set(PB);
  let outside = false;
  for (let f = 0; f < 4; f++) {
    const i = FACES[4 * f], j = FACES[4 * f + 1], k = FACES[4 * f + 2], o = FACES[4 * f + 3];
    const ax = w[3 * i], ay = w[3 * i + 1], az = w[3 * i + 2];
    const ux = w[3 * j] - ax, uy = w[3 * j + 1] - ay, uz = w[3 * j + 2] - az;
    const vx = w[3 * k] - ax, vy = w[3 * k + 1] - ay, vz = w[3 * k + 2] - az;
    const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const toOrigin = -(nx * ax + ny * ay + nz * az);
    const toFourth = nx * (w[3 * o] - ax) + ny * (w[3 * o + 1] - ay) + nz * (w[3 * o + 2] - az);
    if (toOrigin * toFourth >= 0) continue;   // the origin is on the fourth point's side
    outside = true;
    W.set(w); PA.set(pa); PB.set(pb);
    count = 4;
    triangle(i, j, k);
    let px = 0, py = 0, pz = 0;
    for (let m = 0; m < count; m++) { px += L[m] * W[3 * m]; py += L[m] * W[3 * m + 1]; pz += L[m] * W[3 * m + 2]; }
    const d = px * px + py * py + pz * pz;
    if (d < best) { best = d; bestFace = f; }
  }
  if (!outside) return false;
  const i = FACES[4 * bestFace], j = FACES[4 * bestFace + 1], k = FACES[4 * bestFace + 2];
  W.set(w); PA.set(pa); PB.set(pb);
  count = 4;
  triangle(i, j, k);
  return true;
}
