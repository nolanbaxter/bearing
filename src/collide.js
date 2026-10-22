// The narrowphase: where two shapes touch, as one normal and up to four points.
//
// collide(a, b, margin, tolerance, out) fills `out`, a Manifold:
//
//   normal       unit, pointing from a towards b
//   points       world positions, each midway between the two surfaces
//   separations  metres along the normal; negative where the shapes overlap
//   ids          which features made each point, so the solver can carry a
//                point's impulse from one step to the next (warm starting)
//
// SPECULATIVE. Points are reported out to `margin` before the shapes touch,
// not only once they overlap. The solver uses them to stop a body short of a
// surface instead of letting it sink in and pushing it back out, which is
// most of what keeps a stack quiet, and most of what stops a fast body
// passing through a thin one. The caller derives the margin from how far the
// pair can close in a step; nothing here picks a distance.
//
// EXACT PER PAIR, not one generic method. Spheres and capsules are points
// and segments with a radius, so their contacts are closest-point problems
// solved in closed form. Boxes use separating axes and clipping (Gregorius,
// "Robust Contact Creation for Physics Simulations", GDC 2015). The generic
// alternative, GJK and EPA as Jolt uses for everything, is iterative and
// approximate where these are exact, and slower for these shapes; GJK comes
// later, for shape casts and convex hulls.
//
// TOLERANCE is the contact tolerance: how far apart two candidate answers
// may be and still count as the same. Where a box's face and one of its edges
// separate the boxes about equally, the face is kept unless the edge is
// better by more than this, so a resting box doesn't flicker between a
// four-point face contact and a one-point edge contact on rounding noise. The
// solver derives it from the bodies' size; the narrowphase only applies it.
//
// HULLS are separating axes too, over their faces and the pairs of their
// edges that can meet, and clipping: see "hulls" below.
//
// Nothing here allocates: everything works in module scratch, which is safe
// because a step never re-enters it. Hulls' scratch grows to the largest
// hull it has seen, once.

import { build } from './hull.js';
import { closest } from './gjk.js';

/** Shape kinds, numbered in the order pairs are dispatched: a pair is always handled lower first. */
export const SPHERE = 0, CAPSULE = 1, BOX = 2, HULL = 3;
/**
 * One triangle of a mesh or heightfield, placed in the world. Not dispatched
 * by collide(): a triangle against a shape is triangle.js's.
 */
export const TRIANGLE = 4;

/** One shape placed in the world, as collide() reads it. */
export class Shape {
  constructor() {
    this.kind = BOX;
    this.center = new Float64Array(3);
    /** The shape's own x, y and z axes in world space: x at 0..2, y at 3..5, z at 6..8. */
    this.axes = new Float64Array([1, 0, 0, 0, 1, 0, 0, 0, 1]);
    /** Box: half its size along each axis. Sphere: radius. Capsule: radius, then half its segment's length. */
    this.half = new Float64Array(3);
    /** Triangle: its corners, a, b and c, in the world. Its centre is their mean, and axes[0..2] its unit normal. */
    this.points = new Float64Array(9);
    /** Hull: its corners and faces in its own frame (hull.js), placed by center and axes. */
    this.hull = null;
    /**
     * A hull body's own Placed, when the shape is that body as the step's
     * contacts load it (contact.js load()), with the step and the World's
     * _epoch it was loaded at; null for any shape placed by hand.
     */
    this.placed = null;
    this.step = 0;
    this.epoch = 0;
  }

  /** Places the shape: `q` is a unit quaternion [x, y, z, w] at q[o..o+3]. */
  place(px, py, pz, q, o = 0) {
    // Placed anywhere but where its body is in the step: not that body's.
    this.placed = null;
    this.center[0] = px; this.center[1] = py; this.center[2] = pz;
    const x = q[o], y = q[o + 1], z = q[o + 2], w = q[o + 3];
    const a = this.axes;
    a[0] = 1 - 2 * (y * y + z * z); a[1] = 2 * (x * y + z * w); a[2] = 2 * (x * z - y * w);
    a[3] = 2 * (x * y - z * w); a[4] = 1 - 2 * (x * x + z * z); a[5] = 2 * (y * z + x * w);
    a[6] = 2 * (x * z + y * w); a[7] = 2 * (y * z - x * w); a[8] = 1 - 2 * (x * x + y * y);
    return this;
  }
}

/**
 * One shape of a compound body (or the one shape of a box, ball or capsule
 * whose centre of mass was moved off its centre), in its body's own frame:
 * the frame the step works in, centred on the centre of mass and turned to
 * the principal axes (World). `half` and `hull` are as Shape's.
 */
export class Part {
  constructor(kind, half, hull, position, rotation) {
    this.kind = kind;
    this.half = Float64Array.from(half);
    this.hull = hull;
    this.position = Float64Array.from(position);
    this.rotation = Float64Array.from(rotation);
    /** How far it reaches from its own centre, and its thinnest size: as a body's _reach and _size. */
    const h = this.half;
    this.reach = kind === BOX ? Math.sqrt(h[0] * h[0] + h[1] * h[1] + h[2] * h[2])
      : kind === SPHERE ? h[0] : kind === CAPSULE ? h[0] + h[1] : hull.reach;
    this.size = kind === BOX ? 2 * Math.min(h[0], h[1], h[2]) : kind === HULL ? hull.size : 2 * h[0];
  }
}

/** A body's parts, and the box round them in its own frame. */
export class Parts {
  constructor(list) {
    this.list = list;
    const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
    const s = new Shape(), corner = new Float64Array(3);
    for (const part of list) {
      setPart(part, s);
      s.place(part.position[0], part.position[1], part.position[2], part.rotation);
      // Its reach along each of the body's axes, from its centre.
      for (let k = 0; k < 3; k++) {
        corner[0] = k === 0 ? 1 : 0; corner[1] = k === 1 ? 1 : 0; corner[2] = k === 2 ? 1 : 0;
        const up = reachAlong(s, corner[0], corner[1], corner[2]), down = reachAlong(s, -corner[0], -corner[1], -corner[2]);
        lo[k] = Math.min(lo[k], s.center[k] - down);
        hi[k] = Math.max(hi[k], s.center[k] + up);
      }
    }
    this.lo = lo;
    this.hi = hi;
  }
}

/** Shape s takes part's kind and size; place() it after. */
export function setPart(part, s) {
  s.kind = part.kind;
  s.half[0] = part.half[0]; s.half[1] = part.half[1]; s.half[2] = part.half[2];
  s.hull = part.hull;
}

const partQ = new Float64Array(4);

/**
 * Part `part` of a body whose centre is at p[po..] and rotation q[qo..],
 * placed in the world as shape s: its centre at p + q·position, turned by
 * q ⊗ rotation.
 */
export function placePart(part, p, po, q, qo, s) {
  setPart(part, s);
  const r = part.rotation;
  const x = q[qo], y = q[qo + 1], z = q[qo + 2], w = q[qo + 3];
  partQ[0] = w * r[0] + x * r[3] + y * r[2] - z * r[1];
  partQ[1] = w * r[1] + y * r[3] + z * r[0] - x * r[2];
  partQ[2] = w * r[2] + z * r[3] + x * r[1] - y * r[0];
  partQ[3] = w * r[3] - x * r[0] - y * r[1] - z * r[2];
  const o = part.position, tx = 2 * (y * o[2] - z * o[1]), ty = 2 * (z * o[0] - x * o[2]), tz = 2 * (x * o[1] - y * o[0]);
  s.place(p[po] + o[0] + w * tx + (y * tz - z * ty), p[po + 1] + o[1] + w * ty + (z * tx - x * tz),
    p[po + 2] + o[2] + w * tz + (x * ty - y * tx), partQ, 0);
  return s;
}

/** How far placed shape s reaches from its centre along the unit (x, y, z). */
export function reachAlong(s, x, y, z) {
  const A = s.axes, h = s.half;
  if (s.kind === SPHERE) return h[0];
  if (s.kind === CAPSULE) return Math.abs(A[3] * x + A[4] * y + A[5] * z) * h[1] + h[0];
  if (s.kind === BOX) return support(A, h, x, y, z);
  const v = s.hull.vertices, lx = A[0] * x + A[1] * y + A[2] * z, ly = A[3] * x + A[4] * y + A[5] * z, lz = A[6] * x + A[7] * y + A[8] * z;
  let most = -Infinity;
  for (let i = 0; i < v.length; i += 3) most = Math.max(most, v[i] * lx + v[i + 1] * ly + v[i + 2] * lz);
  return most;
}

export const MAX_POINTS = 4;

export class Manifold {
  constructor() {
    this.count = 0;
    this.normal = new Float64Array(3);
    this.points = new Float64Array(3 * MAX_POINTS);
    this.separations = new Float64Array(MAX_POINTS);
    this.ids = new Uint32Array(MAX_POINTS);
  }
}

/**
 * Fills `out` with where `a` and `b` touch, or are within `margin` of
 * touching, and returns how many points it found: 0 means they are further
 * apart than the margin.
 */
export function collide(a, b, margin, tolerance, out) {
  out.count = 0;
  if (a.kind > b.kind) {
    // Every pair is written lower kind first; the other order is the same
    // contact seen from the other side, so the normal turns round.
    pair(b, a, margin, tolerance, out);
    const n = out.normal;
    n[0] = -n[0]; n[1] = -n[1]; n[2] = -n[2];
  } else {
    pair(a, b, margin, tolerance, out);
  }
  return out.count;
}

function pair(a, b, margin, tolerance, out) {
  if (a.kind === SPHERE) {
    if (b.kind === SPHERE) sphereSphere(a, b, margin, out);
    else if (b.kind === CAPSULE) sphereCapsule(a, b, margin, out);
    else if (b.kind === BOX) sphereBox(a, b, margin, out);
    else sphereHull(a, b, margin, out);
  } else if (a.kind === CAPSULE) {
    if (b.kind === CAPSULE) capsuleCapsule(a, b, margin, tolerance, out);
    else if (b.kind === BOX) capsuleBox(a, b, margin, tolerance, out);
    else capsuleHull(a, b, margin, tolerance, out);
  } else if (a.kind === BOX && b.kind === BOX) {
    boxBox(a, b, margin, tolerance, out);
  } else {
    // A box against a hull is a hull of eight corners against it: rare
    // enough that its own closed form wouldn't pay for itself.
    hullHull(a, b, margin, tolerance, out);
  }
}

// ---- small helpers ----

const sign = (x) => (x < 0 ? -1 : 1);
const clamp01 = (x) => (x < 0 ? 0 : x > 1 ? 1 : x);

// The unit vector a helper below last produced, and its length before.
const unit = new Float64Array(3);

/**
 * unit = (x, y, z) / |(x, y, z)|, returning the length; 0 leaves unit alone.
 * Scaled by the largest component first, so a vector of 1e-200 or 1e200
 * neither underflows to zero nor overflows to Infinity on the way.
 */
function normalize(x, y, z) {
  const m = Math.max(Math.abs(x), Math.abs(y), Math.abs(z));
  if (m === 0) return 0;
  const sx = x / m, sy = y / m, sz = z / m;
  const l = Math.sqrt(sx * sx + sy * sy + sz * sz);
  unit[0] = sx / l; unit[1] = sy / l; unit[2] = sz / l;
  return l * m;
}

/**
 * Some unit vector at right angles to the unit vector v, the same one every
 * time: v crossed with the world axis it is least along (x before y before z
 * on a tie). For a normal where nothing prefers a direction, such as two
 * capsules whose segments meet.
 */
function perpendicular(vx, vy, vz) {
  const ax = Math.abs(vx), ay = Math.abs(vy), az = Math.abs(vz);
  if (ax <= ay && ax <= az) normalize(0, vz, -vy);        // v × x
  else if (ay <= az) normalize(-vz, 0, vx);               // v × y
  else normalize(vy, -vx, 0);                              // v × z
}

function setNormal(out, x, y, z) {
  out.normal[0] = x; out.normal[1] = y; out.normal[2] = z;
}

function addPoint(out, x, y, z, separation, id) {
  const k = out.count++;
  out.points[3 * k] = x; out.points[3 * k + 1] = y; out.points[3 * k + 2] = z;
  out.separations[k] = separation;
  out.ids[k] = id;
}

/**
 * Two balls: radius ra at (ax, ay, az) and rb at (bx, by, bz). Spheres, and
 * the closest points of capsules, all come down to this. If the centres
 * coincide, the normal is (fx, fy, fz). Returns whether they are within the
 * margin; sets the normal and adds one point if so.
 */
function balls(ax, ay, az, ra, bx, by, bz, rb, fx, fy, fz, margin, out, id) {
  const dist = normalize(bx - ax, by - ay, bz - az);
  const separation = dist - ra - rb;
  if (separation > margin) return false;
  const nx = dist > 0 ? unit[0] : fx, ny = dist > 0 ? unit[1] : fy, nz = dist > 0 ? unit[2] : fz;
  setNormal(out, nx, ny, nz);
  // Midway between a's surface, at a + n ra, and b's, at b - n rb.
  const t = ra + separation * 0.5;
  addPoint(out, ax + nx * t, ay + ny * t, az + nz * t, separation, id);
  return true;
}

// A capsule's segment, from end 0 to end 1 along its own y.
function segment(c, out) {
  const h = c.half[1], y = c.axes;
  out[0] = c.center[0] - y[3] * h; out[1] = c.center[1] - y[4] * h; out[2] = c.center[2] - y[5] * h;
  out[3] = c.center[0] + y[3] * h; out[4] = c.center[1] + y[4] * h; out[5] = c.center[2] + y[5] * h;
}

/** The parameter in [0, 1] of the point on segment p0 + t d nearest to (x, y, z). */
function nearestOnSegment(s, x, y, z) {
  const dx = s[3] - s[0], dy = s[4] - s[1], dz = s[5] - s[2];
  const dd = dx * dx + dy * dy + dz * dz;
  if (dd === 0) return 0;
  return clamp01(((x - s[0]) * dx + (y - s[1]) * dy + (z - s[2]) * dz) / dd);
}

// Parameters of the closest points between two segments, set by closestSegments.
let segS = 0, segT = 0;

/**
 * The closest points between segment p (p0 + s d1) and segment q (q0 + t d2),
 * s and t in [0, 1], into segS and segT. Ericson, Real-Time Collision
 * Detection, 5.1.9. Parallel segments have a line of closest pairs; this
 * takes the one where s is 0, or as near it as the clamp allows.
 */
function closestSegments(p, q) {
  const d1x = p[3] - p[0], d1y = p[4] - p[1], d1z = p[5] - p[2];
  const d2x = q[3] - q[0], d2y = q[4] - q[1], d2z = q[5] - q[2];
  const rx = p[0] - q[0], ry = p[1] - q[1], rz = p[2] - q[2];
  const a = d1x * d1x + d1y * d1y + d1z * d1z;
  const e = d2x * d2x + d2y * d2y + d2z * d2z;
  const f = d2x * rx + d2y * ry + d2z * rz;
  if (a === 0 && e === 0) { segS = 0; segT = 0; return; }
  if (a === 0) { segS = 0; segT = clamp01(f / e); return; }
  const c = d1x * rx + d1y * ry + d1z * rz;
  if (e === 0) { segT = 0; segS = clamp01(-c / a); return; }
  const b = d1x * d2x + d1y * d2y + d1z * d2z;
  const denom = a * e - b * b;
  let s = denom !== 0 ? clamp01((b * f - c * e) / denom) : 0;
  let t = (b * s + f) / e;
  if (t < 0) { t = 0; s = clamp01(-c / a); } else if (t > 1) { t = 1; s = clamp01((b - c) / a); }
  segS = s; segT = t;
}

// ---- sphere against sphere, capsule and box ----

function sphereSphere(a, b, margin, out) {
  const ca = a.center, cb = b.center;
  // Concentric: nothing prefers a direction, so up.
  balls(ca[0], ca[1], ca[2], a.half[0], cb[0], cb[1], cb[2], b.half[0], 0, 1, 0, margin, out, 0);
}

const segA = new Float64Array(6), segB = new Float64Array(6);

function sphereCapsule(a, b, margin, out) {
  const c = a.center;
  segment(b, segB);
  const t = nearestOnSegment(segB, c[0], c[1], c[2]);
  const qx = segB[0] + (segB[3] - segB[0]) * t, qy = segB[1] + (segB[4] - segB[1]) * t, qz = segB[2] + (segB[5] - segB[2]) * t;
  // The centre on the capsule's axis: any direction off the axis will do.
  perpendicular(b.axes[3], b.axes[4], b.axes[5]);
  balls(c[0], c[1], c[2], a.half[0], qx, qy, qz, b.half[0], unit[0], unit[1], unit[2], margin, out, 0);
}

function sphereBox(a, b, margin, out) {
  const ax = b.axes, h = b.half, r = a.half[0];
  const dx = a.center[0] - b.center[0], dy = a.center[1] - b.center[1], dz = a.center[2] - b.center[2];
  // The sphere's centre in the box's frame.
  const lx = dx * ax[0] + dy * ax[1] + dz * ax[2];
  const ly = dx * ax[3] + dy * ax[4] + dz * ax[5];
  const lz = dx * ax[6] + dy * ax[7] + dz * ax[8];
  const inside = Math.abs(lx) <= h[0] && Math.abs(ly) <= h[1] && Math.abs(lz) <= h[2];
  if (!inside) {
    const qx = Math.max(-h[0], Math.min(h[0], lx));
    const qy = Math.max(-h[1], Math.min(h[1], ly));
    const qz = Math.max(-h[2], Math.min(h[2], lz));
    // The nearest point on the box, back in world space, as a ball of radius 0.
    const wx = b.center[0] + ax[0] * qx + ax[3] * qy + ax[6] * qz;
    const wy = b.center[1] + ax[1] * qx + ax[4] * qy + ax[7] * qz;
    const wz = b.center[2] + ax[2] * qx + ax[5] * qy + ax[8] * qz;
    balls(a.center[0], a.center[1], a.center[2], r, wx, wy, wz, 0, 0, 1, 0, margin, out, 0);
    return;
  }
  // The centre is inside: out through the nearest face. On a tie, x before y before z.
  let axis = 0, depth = h[0] - Math.abs(lx);
  if (h[1] - Math.abs(ly) < depth) { axis = 1; depth = h[1] - Math.abs(ly); }
  if (h[2] - Math.abs(lz) < depth) { axis = 2; depth = h[2] - Math.abs(lz); }
  const l = axis === 0 ? lx : axis === 1 ? ly : lz;
  // The face's outward normal points towards the sphere; from sphere to box is the other way.
  const s = -sign(l);
  const nx = ax[3 * axis] * s, ny = ax[3 * axis + 1] * s, nz = ax[3 * axis + 2] * s;
  setNormal(out, nx, ny, nz);
  const separation = -(depth + r);
  // Midway between the sphere's surface, c + n r, and the face, c - n depth.
  const t = r + separation * 0.5;
  addPoint(out, a.center[0] + nx * t, a.center[1] + ny * t, a.center[2] + nz * t, separation, 0);
}

// ---- capsule against capsule ----

/**
 * Closest points first. If those are within the margin and the segments run
 * alongside each other, the stretch of b's segment that lies alongside a's
 * gives two points, one at each end, so two logs lying side by side rest on
 * a line rather than balancing on one point. The closest pair is kept as a
 * third point only where it is deeper than both: segments that cross, or
 * that pass closest beyond the end of a's.
 *
 * An end point's separation is measured along the shared normal, which is
 * what the solver pushes along. That is the true gap only while the two
 * segments run alongside each other; for segments at an angle, an end point
 * sits off to the side of the normal through its partner on a, and the gap
 * along the normal understates the real one. So an end point is kept only
 * while it lies within the tolerance of that normal. Two capsules at any real
 * angle touch at one point, and get one, so they pivot as real logs do; once
 * they settle side by side, they get two and rest.
 */
function capsuleCapsule(a, b, margin, tolerance, out) {
  segment(a, segA);
  segment(b, segB);
  closestSegments(segA, segB);
  const s = segS, t = segT;
  const pax = segA[0] + (segA[3] - segA[0]) * s, pay = segA[1] + (segA[4] - segA[1]) * s, paz = segA[2] + (segA[5] - segA[2]) * s;
  const pbx = segB[0] + (segB[3] - segB[0]) * t, pby = segB[1] + (segB[4] - segB[1]) * t, pbz = segB[2] + (segB[5] - segB[2]) * t;
  const ra = a.half[0], rb = b.half[0];
  // Segments that meet: off both axes if they cross, off a's if they lie along each other.
  const ya = a.axes, yb = b.axes;
  if (normalize(ya[4] * yb[5] - ya[5] * yb[4], ya[5] * yb[3] - ya[3] * yb[5], ya[3] * yb[4] - ya[4] * yb[3]) === 0) {
    perpendicular(ya[3], ya[4], ya[5]);
  }
  const fx = unit[0], fy = unit[1], fz = unit[2];
  // Kept aside: the two-point check below adds its own points first.
  if (!balls(pax, pay, paz, ra, pbx, pby, pbz, rb, fx, fy, fz, margin, out, 0)) return;
  const closest = out.separations[0];
  const cx = out.points[0], cy = out.points[1], cz = out.points[2];
  out.count = 0;

  const h = a.half[1];
  if (h > 0 && b.half[1] > 0) {
    // b(u) = b0 + u (b1 - b0) projects onto a's axis at k0 + u k1, measured from a's centre.
    const k0 = (segB[0] - a.center[0]) * ya[3] + (segB[1] - a.center[1]) * ya[4] + (segB[2] - a.center[2]) * ya[5];
    const k1 = (segB[3] - segB[0]) * ya[3] + (segB[4] - segB[1]) * ya[4] + (segB[5] - segB[2]) * ya[5];
    let u0 = 0, u1 = 1;
    if (k1 !== 0) {
      const ua = (-h - k0) / k1, ub = (h - k0) / k1;
      u0 = Math.max(0, Math.min(ua, ub));
      u1 = Math.min(1, Math.max(ua, ub));
    } else if (k0 < -h || k0 > h) {
      u1 = -1;   // alongside, but past a's end: no stretch
    }
    if (u1 > u0) {
      const n = out.normal;
      let deepest = Infinity;
      for (let e = 0; e < 2; e++) {
        const u = e === 0 ? u0 : u1;
        const bx = segB[0] + (segB[3] - segB[0]) * u, by = segB[1] + (segB[4] - segB[1]) * u, bz = segB[2] + (segB[5] - segB[2]) * u;
        const along = Math.max(-h, Math.min(h, k0 + k1 * u));
        const ax = a.center[0] + ya[3] * along, ay = a.center[1] + ya[4] * along, az = a.center[2] + ya[5] * along;
        const gx = bx - ax, gy = by - ay, gz = bz - az;
        const normalGap = gx * n[0] + gy * n[1] + gz * n[2];
        const separation = normalGap - ra - rb;
        if (separation > margin) continue;
        // How far off to the side of the normal through its partner, squared.
        if (gx * gx + gy * gy + gz * gz - normalGap * normalGap > tolerance * tolerance) continue;
        deepest = Math.min(deepest, separation);
        const m = ra + separation * 0.5;
        addPoint(out, ax + n[0] * m, ay + n[1] * m, az + n[2] * m, separation, 1 + e);
      }
      // The closest pair, wherever it lies, if it is deeper than both ends:
      // segments that cross, or that pass closest beyond a's end.
      if (out.count > 0 && closest < deepest) addPoint(out, cx, cy, cz, closest, 0);
      if (out.count > 0) return;
    }
  }
  addPoint(out, cx, cy, cz, closest, 0);
}

// ---- capsule against box ----

// The capsule's segment in the box's frame.
const local = new Float64Array(6);
const edge = new Float64Array(6);

/**
 * Exact closest points between the capsule's segment and the solid box. If
 * they meet on a face's interior, the capsule lies against that face, and the
 * segment is clipped to the face for two points, so a capsule lying on a box
 * rests on a line. If they meet at an edge or a corner, one point. If the
 * segment passes into the box, separating axes find the shallowest way out.
 */
function capsuleBox(a, b, margin, tolerance, out) {
  segment(a, segA);
  const bx = b.axes, h = b.half, r = a.half[0];
  for (let e = 0; e < 2; e++) {
    const dx = segA[3 * e] - b.center[0], dy = segA[3 * e + 1] - b.center[1], dz = segA[3 * e + 2] - b.center[2];
    local[3 * e] = dx * bx[0] + dy * bx[1] + dz * bx[2];
    local[3 * e + 1] = dx * bx[3] + dy * bx[4] + dz * bx[5];
    local[3 * e + 2] = dx * bx[6] + dy * bx[7] + dz * bx[8];
  }
  if (segmentHitsBox(local, h)) { capsuleBoxDeep(a, b, margin, tolerance, out); return; }

  // Candidates, in a fixed order so a tie always goes the same way: each end
  // against the box, then the segment against each of the 12 edges.
  let best = Infinity, sx = 0, sy = 0, sz = 0, qx = 0, qy = 0, qz = 0;
  for (let e = 0; e < 2; e++) {
    const px = local[3 * e], py = local[3 * e + 1], pz = local[3 * e + 2];
    const cx = Math.max(-h[0], Math.min(h[0], px)), cy = Math.max(-h[1], Math.min(h[1], py)), cz = Math.max(-h[2], Math.min(h[2], pz));
    const d = (px - cx) * (px - cx) + (py - cy) * (py - cy) + (pz - cz) * (pz - cz);
    if (d < best) { best = d; sx = px; sy = py; sz = pz; qx = cx; qy = cy; qz = cz; }
  }
  for (let i = 0; i < 3; i++) {
    const j = (i + 1) % 3, k = (i + 2) % 3;
    for (let c = 0; c < 4; c++) {
      edge[i] = -h[i]; edge[3 + i] = h[i];
      edge[j] = edge[3 + j] = c & 1 ? h[j] : -h[j];
      edge[k] = edge[3 + k] = c & 2 ? h[k] : -h[k];
      closestSegments(local, edge);
      const px = local[0] + (local[3] - local[0]) * segS, py = local[1] + (local[4] - local[1]) * segS, pz = local[2] + (local[5] - local[2]) * segS;
      const ex = edge[0] + (edge[3] - edge[0]) * segT, ey = edge[1] + (edge[4] - edge[1]) * segT, ez = edge[2] + (edge[5] - edge[2]) * segT;
      const d = (px - ex) * (px - ex) + (py - ey) * (py - ey) + (pz - ez) * (pz - ez);
      if (d < best) { best = d; sx = px; sy = py; sz = pz; qx = ex; qy = ey; qz = ez; }
    }
  }
  const dist = normalize(qx - sx, qy - sy, qz - sz);
  if (dist - r > margin) return;
  // From the capsule to the box, in the box's frame and in the world's.
  const nlx = unit[0], nly = unit[1], nlz = unit[2];
  setNormal(out, bx[0] * nlx + bx[3] * nly + bx[6] * nlz, bx[1] * nlx + bx[4] * nly + bx[7] * nlz, bx[2] * nlx + bx[5] * nly + bx[8] * nlz);

  // On a face's interior: exactly one coordinate at the box's surface.
  const onX = Math.abs(qx) === h[0], onY = Math.abs(qy) === h[1], onZ = Math.abs(qz) === h[2];
  if (onX + onY + onZ === 1) {
    const axis = onX ? 0 : onY ? 1 : 2;
    const facing = sign(axis === 0 ? qx : axis === 1 ? qy : qz);
    if (clipToFace(b, axis, facing, r, margin, out, 0)) return;
  }
  const separation = dist - r;
  const m = r + separation * 0.5;
  toWorld(b, sx + nlx * m, sy + nly * m, sz + nlz * m);
  addPoint(out, world[0], world[1], world[2], separation, 0);
}

const world = new Float64Array(3);

/** world = the box's centre plus its axes times (x, y, z): a point in the box's frame, in the world's. */
function toWorld(b, x, y, z) {
  const ax = b.axes;
  world[0] = b.center[0] + ax[0] * x + ax[3] * y + ax[6] * z;
  world[1] = b.center[1] + ax[1] * x + ax[4] * y + ax[7] * z;
  world[2] = b.center[2] + ax[2] * x + ax[5] * y + ax[8] * z;
}

/** Whether the segment in `local` passes through the box of half extents h (slab test). */
function segmentHitsBox(s, h) {
  let t0 = 0, t1 = 1;
  for (let i = 0; i < 3; i++) {
    const p = s[i], d = s[3 + i] - s[i];
    if (d === 0) {
      if (p < -h[i] || p > h[i]) return false;
      continue;
    }
    let ta = (-h[i] - p) / d, tb = (h[i] - p) / d;
    if (ta > tb) { const x = ta; ta = tb; tb = x; }
    if (ta > t0) t0 = ta;
    if (tb < t1) t1 = tb;
    if (t0 > t1) return false;
  }
  return true;
}

/**
 * The segment in `local`, clipped to the face of b on `axis` whose outward
 * normal is `facing` along it: a point at each end of the clipped stretch,
 * with its separation from the face less the radius r. Normal: into the face.
 * Returns whether it added any point.
 */
function clipToFace(b, axis, facing, r, margin, out, idBase) {
  const h = b.half, j = (axis + 1) % 3, k = (axis + 2) % 3;
  let u0 = 0, u1 = 1;
  for (let pass = 0; pass < 2; pass++) {
    const c = pass === 0 ? j : k;
    const p = local[c], d = local[3 + c] - local[c];
    if (d === 0) {
      if (p < -h[c] || p > h[c]) return false;
      continue;
    }
    let ta = (-h[c] - p) / d, tb = (h[c] - p) / d;
    if (ta > tb) { const x = ta; ta = tb; tb = x; }
    if (ta > u0) u0 = ta;
    if (tb < u1) u1 = tb;
  }
  if (u0 > u1) return false;
  const ax = b.axes;
  setNormal(out, -facing * ax[3 * axis], -facing * ax[3 * axis + 1], -facing * ax[3 * axis + 2]);
  const face = 2 * axis + (facing > 0 ? 1 : 0);
  for (let e = 0; e < (u1 > u0 ? 2 : 1); e++) {
    const u = e === 0 ? u0 : u1;
    const px = local[0] + (local[3] - local[0]) * u, py = local[1] + (local[4] - local[1]) * u, pz = local[2] + (local[5] - local[2]) * u;
    const above = facing * (axis === 0 ? px : axis === 1 ? py : pz) - h[axis];
    const separation = above - r;
    if (separation > margin) continue;
    // Midway between the capsule's surface, r below the segment, and the face:
    // separation / 2 out from the face, where the segment is `above` out.
    const off = facing * (separation * 0.5 - above);
    toWorld(b, axis === 0 ? px + off : px, axis === 1 ? py + off : py, axis === 2 ? pz + off : pz);
    addPoint(out, world[0], world[1], world[2], separation, idBase + 4 * face + 1 + e);
  }
  return out.count > 0;
}

/**
 * The segment passes into the box. Separating axes: the box's three face
 * normals, and the segment's direction crossed with each, which is where an
 * edge of the box meets the segment. The axis with the least overlap is the
 * way out; a face axis clips for two points, an edge axis gives one. As
 * between boxes, an edge axis must beat the faces by more than the
 * tolerance: an edge axis can point the very way a face does, and then
 * rounding alone would choose.
 */
function capsuleBoxDeep(a, b, margin, tolerance, out) {
  const h = b.half, r = a.half[0];
  const dx = local[3] - local[0], dy = local[4] - local[1], dz = local[5] - local[2];
  let bestSep = -Infinity, bestAxis = -1, nx = 0, ny = 0, nz = 0;
  for (let c = 0; c < 6; c++) {
    let x = 0, y = 0, z = 0;
    if (c < 3) {
      if (c === 0) x = 1; else if (c === 1) y = 1; else z = 1;
    } else {
      // Segment direction × box axis (c - 3). Skipped where they run parallel:
      // a face axis already tests that direction.
      const e = c - 3;
      const l = normalize(e === 0 ? 0 : e === 1 ? -dz : dy, e === 0 ? dz : e === 1 ? 0 : -dx, e === 0 ? -dy : e === 1 ? dx : 0);
      if (!(l > PARALLEL * Math.sqrt(dx * dx + dy * dy + dz * dz))) continue;
      x = unit[0]; y = unit[1]; z = unit[2];
    }
    const support = h[0] * Math.abs(x) + h[1] * Math.abs(y) + h[2] * Math.abs(z);
    const p0 = local[0] * x + local[1] * y + local[2] * z, p1 = local[3] * x + local[4] * y + local[5] * z;
    const lo = Math.min(p0, p1), hi = Math.max(p0, p1);
    // Out the + side means the capsule ends up past the box along +axis.
    const plus = lo - support, minus = -support - hi;
    const sep = Math.max(plus, minus) - r;
    if (sep > bestSep + (c >= 3 && bestAxis < 3 ? tolerance : 0)) {
      bestSep = sep; bestAxis = c;
      const s = plus >= minus ? -1 : 1;   // from capsule to box
      nx = x * s; ny = y * s; nz = z * s;
    }
  }
  if (bestAxis < 3) {
    // The face of the box towards the capsule.
    const facing = -(bestAxis === 0 ? nx : bestAxis === 1 ? ny : nz);
    clipToFace(b, bestAxis, sign(facing), r, margin, out, 0);
    return;
  }
  // An edge of the box, along the axis the segment crossed, on the side the capsule is.
  const i = bestAxis - 3, j = (i + 1) % 3, k = (i + 2) % 3;
  // Towards the capsule from the box is -n.
  const mj = j === 0 ? -nx : j === 1 ? -ny : -nz, mk = k === 0 ? -nx : k === 1 ? -ny : -nz;
  edge[i] = -h[i]; edge[3 + i] = h[i];
  edge[j] = edge[3 + j] = sign(mj) * h[j];
  edge[k] = edge[3 + k] = sign(mk) * h[k];
  closestSegments(local, edge);
  const px = local[0] + dx * segS, py = local[1] + dy * segS, pz = local[2] + dz * segS;
  const ex = edge[0] + (edge[3] - edge[0]) * segT, ey = edge[1] + (edge[4] - edge[1]) * segT, ez = edge[2] + (edge[5] - edge[2]) * segT;
  const separation = (ex - px) * nx + (ey - py) * ny + (ez - pz) * nz - r;
  const ax = b.axes;
  setNormal(out, ax[0] * nx + ax[3] * ny + ax[6] * nz, ax[1] * nx + ax[4] * ny + ax[7] * nz, ax[2] * nx + ax[5] * ny + ax[8] * nz);
  const mm = r + separation * 0.5;
  toWorld(b, px + nx * mm, py + ny * mm, pz + nz * mm);
  addPoint(out, world[0], world[1], world[2], separation, 32 + i);
}

/**
 * An edge axis built from two directions within this angle (as its sine) of
 * parallel is skipped. Below it, the cross product's direction is decided by
 * rounding in the inputs rather than by the geometry: a direction known to
 * about 2^-53 of its length, crossed with another at sine s, comes out good
 * to 2^-53 / s, and 2^-27 here keeps that to half a double's bits. The face
 * axes, within that angle of the skipped one, already test the same direction.
 */
const PARALLEL = 7.450580596923828e-9;   // 2^-27

// ---- box against box ----

// The incident face, clipped against the reference face's four sides. Each
// vertex carries a feature: which incident vertex it is, or which incident
// edge and which side plane made it, or which two side planes (a corner of
// the reference face). Each edge records the line it lies on, an incident edge
// (0-3) or a side plane (4-7), so a vertex made by clipping knows its feature.
const polyA = new Float64Array(3 * 8), polyB = new Float64Array(3 * 8);
const featA = new Int32Array(8), featB = new Int32Array(8);
const lineA = new Int32Array(8), lineB = new Int32Array(8);
const kept = new Float64Array(3 * 8), keptSep = new Float64Array(8);
const keptId = new Uint32Array(8);
// boxBox()'s |A_i . B_j|, at 3i + j.
const cosines = new Float64Array(9);
// A face's corners going round, as signs along its two in-plane axes.
const CORNER_P = [-1, 1, 1, -1], CORNER_Q = [-1, -1, 1, 1];

/**
 * Separating axes over all 15 candidates: a's three face normals, b's three,
 * and the nine cross products of an edge of each. Any axis separating them by
 * more than the margin ends it. Otherwise the axis of least overlap decides
 * the contact: a face of either box clips the other's nearest face against it
 * for up to eight points, reduced to the four that hold it best; two edges
 * give the one point where they pass closest.
 */
function boxBox(a, b, margin, tolerance, out) {
  const A = a.axes, B = b.axes, ha = a.half, hb = b.half;
  const dx = b.center[0] - a.center[0], dy = b.center[1] - a.center[1], dz = b.center[2] - a.center[2];

  // a's faces. Both boxes' face tests reach the other box through the same
  // nine |A_i . B_j|, the same products summed in the same order (x, y, z), so
  // a's loop works them out a row at a time, as support() would, and b's
  // reads them back: the same bits for half the multiplies. Measured: see
  // ROADMAP.md, milestone 16.
  const R = cosines;
  let faceA = -Infinity, axisA = 0;
  for (let i = 0; i < 3; i++) {
    const nx = A[3 * i], ny = A[3 * i + 1], nz = A[3 * i + 2];
    const r0 = Math.abs(B[0] * nx + B[1] * ny + B[2] * nz);
    const r1 = Math.abs(B[3] * nx + B[4] * ny + B[5] * nz);
    const r2 = Math.abs(B[6] * nx + B[7] * ny + B[8] * nz);
    R[3 * i] = r0; R[3 * i + 1] = r1; R[3 * i + 2] = r2;
    const s = Math.abs(dx * nx + dy * ny + dz * nz) - ha[i] - (hb[0] * r0 + hb[1] * r1 + hb[2] * r2);
    if (s > margin) return;
    if (s > faceA) { faceA = s; axisA = i; }
  }
  // b's faces.
  let faceB = -Infinity, axisB = 0;
  for (let j = 0; j < 3; j++) {
    const nx = B[3 * j], ny = B[3 * j + 1], nz = B[3 * j + 2];
    const s = Math.abs(dx * nx + dy * ny + dz * nz) - hb[j] - (ha[0] * R[j] + ha[1] * R[3 + j] + ha[2] * R[6 + j]);
    if (s > margin) return;
    if (s > faceB) { faceB = s; axisB = j; }
  }
  // Edge pairs.
  let edgeSep = -Infinity, edgeI = -1, edgeJ = -1, enx = 0, eny = 0, enz = 0;
  for (let i = 0; i < 3; i++) {
    const ux = A[3 * i], uy = A[3 * i + 1], uz = A[3 * i + 2];
    for (let j = 0; j < 3; j++) {
      const vx = B[3 * j], vy = B[3 * j + 1], vz = B[3 * j + 2];
      const l = normalize(uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx);
      if (!(l > PARALLEL)) continue;
      let nx = unit[0], ny = unit[1], nz = unit[2];
      const d = dx * nx + dy * ny + dz * nz;
      if (d < 0) { nx = -nx; ny = -ny; nz = -nz; }
      const s = Math.abs(d) - support(A, ha, nx, ny, nz) - support(B, hb, nx, ny, nz);
      if (s > margin) return;
      if (s > edgeSep) { edgeSep = s; edgeI = i; edgeJ = j; enx = nx; eny = ny; enz = nz; }
    }
  }

  // A face is kept unless an edge, or the other box's face, is better by more
  // than the tolerance: on a tie, a's face, so the same pair keeps the same
  // reference from step to step.
  if (edgeI >= 0 && edgeSep > Math.max(faceA, faceB) + tolerance) {
    edgeContact(a, b, edgeI, edgeJ, enx, eny, enz, margin, out);
  } else if (faceB > faceA + tolerance) {
    faceContact(b, a, axisB, true, margin, out);
  } else {
    faceContact(a, b, axisA, false, margin, out);
  }
  // Apart, but within the margin, and neither a face nor a pair of edges gave
  // a point: the boxes are nearest at a corner, or at an edge's end. The
  // separating-axis gap is no contact to use here, since it measures from a
  // face's plane carried on past the face's edge, and would stop a box falling
  // past a corner in thin air. The true closest points are.
  if (out.count === 0 && Math.max(faceA, faceB, edgeSep) > 0) boxBoxClosest(a, b, margin, out);
}

const cornerA = new Float64Array(3 * 8), cornerB = new Float64Array(3 * 8);

/** A box's 8 corners into c, corner k at signs (k & 1, k & 2, k & 4) along its axes. */
function corners(x, c) {
  const X = x.axes, h = x.half;
  for (let k = 0; k < 8; k++) {
    const sx = k & 1 ? h[0] : -h[0], sy = k & 2 ? h[1] : -h[1], sz = k & 4 ? h[2] : -h[2];
    c[3 * k] = x.center[0] + X[0] * sx + X[3] * sy + X[6] * sz;
    c[3 * k + 1] = x.center[1] + X[1] * sx + X[4] * sy + X[7] * sz;
    c[3 * k + 2] = x.center[2] + X[2] * sx + X[5] * sy + X[8] * sz;
  }
}

// The 12 edges of a box as pairs of corners differing in one sign.
const EDGE_FROM = [0, 2, 4, 6, 0, 1, 4, 5, 0, 1, 2, 3];
const EDGE_TO = [1, 3, 5, 7, 2, 3, 6, 7, 4, 5, 6, 7];

/**
 * The true closest points of two boxes that are apart: each corner of one
 * against the other box, and each edge of one against each edge of the
 * other. Exact, and dear (16 corners and 144 edge pairs), which is why it
 * runs only where clipping found nothing. One point, at the midpoint.
 */
function boxBoxClosest(a, b, margin, out) {
  corners(a, cornerA);
  corners(b, cornerB);
  let best = Infinity, ax = 0, ay = 0, az = 0, bx = 0, by = 0, bz = 0;
  for (let side = 0; side < 2; side++) {
    const from = side === 0 ? cornerA : cornerB, box = side === 0 ? b : a;
    const X = box.axes, h = box.half;
    for (let k = 0; k < 8; k++) {
      const dx = from[3 * k] - box.center[0], dy = from[3 * k + 1] - box.center[1], dz = from[3 * k + 2] - box.center[2];
      const lx = Math.max(-h[0], Math.min(h[0], dx * X[0] + dy * X[1] + dz * X[2]));
      const ly = Math.max(-h[1], Math.min(h[1], dx * X[3] + dy * X[4] + dz * X[5]));
      const lz = Math.max(-h[2], Math.min(h[2], dx * X[6] + dy * X[7] + dz * X[8]));
      toWorld(box, lx, ly, lz);
      const ex = world[0] - from[3 * k], ey = world[1] - from[3 * k + 1], ez = world[2] - from[3 * k + 2];
      const d = ex * ex + ey * ey + ez * ez;
      if (d < best) {
        best = d;
        if (side === 0) { ax = from[3 * k]; ay = from[3 * k + 1]; az = from[3 * k + 2]; bx = world[0]; by = world[1]; bz = world[2]; }
        else { bx = from[3 * k]; by = from[3 * k + 1]; bz = from[3 * k + 2]; ax = world[0]; ay = world[1]; az = world[2]; }
      }
    }
  }
  for (let i = 0; i < 12; i++) {
    for (let k = 0; k < 3; k++) { segA[k] = cornerA[3 * EDGE_FROM[i] + k]; segA[3 + k] = cornerA[3 * EDGE_TO[i] + k]; }
    for (let j = 0; j < 12; j++) {
      for (let k = 0; k < 3; k++) { segB[k] = cornerB[3 * EDGE_FROM[j] + k]; segB[3 + k] = cornerB[3 * EDGE_TO[j] + k]; }
      closestSegments(segA, segB);
      const px = segA[0] + (segA[3] - segA[0]) * segS, py = segA[1] + (segA[4] - segA[1]) * segS, pz = segA[2] + (segA[5] - segA[2]) * segS;
      const qx = segB[0] + (segB[3] - segB[0]) * segT, qy = segB[1] + (segB[4] - segB[1]) * segT, qz = segB[2] + (segB[5] - segB[2]) * segT;
      const d = (qx - px) * (qx - px) + (qy - py) * (qy - py) + (qz - pz) * (qz - pz);
      if (d < best) { best = d; ax = px; ay = py; az = pz; bx = qx; by = qy; bz = qz; }
    }
  }
  const dist = normalize(bx - ax, by - ay, bz - az);
  if (dist > margin || dist === 0) return;
  setNormal(out, unit[0], unit[1], unit[2]);
  addPoint(out, (ax + bx) * 0.5, (ay + by) * 0.5, (az + bz) * 0.5, dist, 1 << 14);
}

/** How far a box of axes X and half extents h reaches along the unit (nx, ny, nz). */
function support(X, h, nx, ny, nz) {
  return h[0] * Math.abs(X[0] * nx + X[1] * ny + X[2] * nz)
    + h[1] * Math.abs(X[3] * nx + X[4] * ny + X[5] * nz)
    + h[2] * Math.abs(X[6] * nx + X[7] * ny + X[8] * nz);
}

/**
 * Face `axis` of `ref`, the side facing `inc`, against the face of `inc` most
 * nearly opposite it. `flipped` is true when ref is the collide() call's b, so
 * the normal, which here points from ref to inc, is turned to point from a to b.
 */
function faceContact(ref, inc, axis, flipped, margin, out) {
  const R = ref.axes, I = inc.axes, hr = ref.half, hi = inc.half;
  const dx = inc.center[0] - ref.center[0], dy = inc.center[1] - ref.center[1], dz = inc.center[2] - ref.center[2];
  const s = sign(dx * R[3 * axis] + dy * R[3 * axis + 1] + dz * R[3 * axis + 2]);
  const nx = R[3 * axis] * s, ny = R[3 * axis + 1] * s, nz = R[3 * axis + 2] * s;
  const refFace = 2 * axis + (s > 0 ? 1 : 0);

  // The incident face: the one whose outward normal is most against n.
  let incAxis = 0, best = -1;
  for (let j = 0; j < 3; j++) {
    const d = Math.abs(I[3 * j] * nx + I[3 * j + 1] * ny + I[3 * j + 2] * nz);
    if (d > best) { best = d; incAxis = j; }
  }
  const incSign = -sign(I[3 * incAxis] * nx + I[3 * incAxis + 1] * ny + I[3 * incAxis + 2] * nz);
  const incFace = 2 * incAxis + (incSign > 0 ? 1 : 0);

  // Its four corners, going round, and the incident edge each starts.
  const p = (incAxis + 1) % 3, q = (incAxis + 2) % 3;
  const cx = inc.center[0] + I[3 * incAxis] * incSign * hi[incAxis];
  const cy = inc.center[1] + I[3 * incAxis + 1] * incSign * hi[incAxis];
  const cz = inc.center[2] + I[3 * incAxis + 2] * incSign * hi[incAxis];
  for (let v = 0; v < 4; v++) {
    const sp = CORNER_P[v], sq = CORNER_Q[v];
    polyA[3 * v] = cx + I[3 * p] * sp * hi[p] + I[3 * q] * sq * hi[q];
    polyA[3 * v + 1] = cy + I[3 * p + 1] * sp * hi[p] + I[3 * q + 1] * sq * hi[q];
    polyA[3 * v + 2] = cz + I[3 * p + 2] * sp * hi[p] + I[3 * q + 2] * sq * hi[q];
    featA[v] = v;
    lineA[v] = v;
  }

  // Clip against the reference face's four sides: along its other two axes, within ± their half extents.
  const u = (axis + 1) % 3, w = (axis + 2) % 3;
  let count = 4, src = 0;
  for (let plane = 0; plane < 4 && count > 0; plane++) {
    const c = plane < 2 ? u : w, side = plane % 2 === 0 ? 1 : -1;
    const px = R[3 * c] * side, py = R[3 * c + 1] * side, pz = R[3 * c + 2] * side;
    const offset = px * ref.center[0] + py * ref.center[1] + pz * ref.center[2] + hr[c];
    count = src === 0
      ? clipPolygon(polyA, featA, lineA, count, polyB, featB, lineB, px, py, pz, offset, plane)
      : clipPolygon(polyB, featB, lineB, count, polyA, featA, lineA, px, py, pz, offset, plane);
    src = 1 - src;
  }
  const poly = src === 0 ? polyA : polyB, feat = src === 0 ? featA : featB;

  // Keep what lies within the margin of the reference face, as points midway.
  const fx = ref.center[0] + nx * hr[axis], fy = ref.center[1] + ny * hr[axis], fz = ref.center[2] + nz * hr[axis];
  let n = 0;
  for (let v = 0; v < count; v++) {
    const vx = poly[3 * v], vy = poly[3 * v + 1], vz = poly[3 * v + 2];
    const separation = (vx - fx) * nx + (vy - fy) * ny + (vz - fz) * nz;
    if (separation > margin) continue;
    const m = separation * 0.5;
    kept[3 * n] = vx - nx * m; kept[3 * n + 1] = vy - ny * m; kept[3 * n + 2] = vz - nz * m;
    keptSep[n] = separation;
    keptId[n] = ((flipped ? 1 : 0) << 12) | (refFace << 9) | (incFace << 6) | feat[v];
    n++;
  }
  if (n === 0) return;
  if (flipped) setNormal(out, -nx, -ny, -nz);
  else setNormal(out, nx, ny, nz);
  reduce(n, nx, ny, nz, out);
}

/**
 * Sutherland-Hodgman: the polygon `src` (count vertices) clipped to the half
 * space (x, y, z)·p <= offset, into `dst`. Returns the new count. A vertex
 * made where an edge crosses the plane takes its feature from that edge's
 * line and the plane: incident edge e and plane k is 4 + 4e + k; side planes
 * j and k, a corner of the reference face, is 20 + 4 min + max.
 */
function clipPolygon(src, sf, sl, count, dst, df, dl, px, py, pz, offset, plane) {
  let n = 0;
  for (let v = 0; v < count; v++) {
    const w = (v + 1) % count;
    const ax = src[3 * v], ay = src[3 * v + 1], az = src[3 * v + 2];
    const bx = src[3 * w], by = src[3 * w + 1], bz = src[3 * w + 2];
    const da = ax * px + ay * py + az * pz - offset, db = bx * px + by * py + bz * pz - offset;
    const line = sl[v];   // the line edge v -> w lies on
    if (da <= 0) {
      dst[3 * n] = ax; dst[3 * n + 1] = ay; dst[3 * n + 2] = az;
      df[n] = sf[v];
      // The edge leaving this vertex stays on its line, unless it is cut.
      dl[n] = line;
      n++;
    }
    if ((da <= 0) !== (db <= 0)) {
      const t = da / (da - db);
      dst[3 * n] = ax + (bx - ax) * t; dst[3 * n + 1] = ay + (by - ay) * t; dst[3 * n + 2] = az + (bz - az) * t;
      df[n] = line < 4 ? 4 + 4 * line + plane : 20 + 4 * Math.min(line - 4, plane) + Math.max(line - 4, plane);
      // Leaving the half space, the cut edge runs on along the plane to where
      // the polygon comes back in; entering, it runs on along the old line.
      dl[n] = da <= 0 ? 4 + plane : line;
      n++;
    }
  }
  return n;
}

/**
 * At most four of the n kept points: the deepest; the one farthest from it;
 * the one making the largest triangle with those two; and the one making the
 * largest on the other side (Gregorius). They hold the contact's area, which
 * is what keeps a box from rocking, with the deepest always among them. On
 * any tie, the first point found.
 */
function reduce(n, nx, ny, nz, out) {
  reducePoints(kept, keptSep, keptId, n, nx, ny, nz, out);
}

/** reduce()'s work on any list of n points, with their separations and ids. */
function reducePoints(kept, keptSep, keptId, n, nx, ny, nz, out) {
  if (n <= MAX_POINTS) {
    for (let k = 0; k < n; k++) addPoint(out, kept[3 * k], kept[3 * k + 1], kept[3 * k + 2], keptSep[k], keptId[k]);
    return;
  }
  let a = 0;
  for (let k = 1; k < n; k++) if (keptSep[k] < keptSep[a]) a = k;
  let b = -1, far = -1;
  for (let k = 0; k < n; k++) {
    const x = kept[3 * k] - kept[3 * a], y = kept[3 * k + 1] - kept[3 * a + 1], z = kept[3 * k + 2] - kept[3 * a + 2];
    const d = x * x + y * y + z * z;
    if (d > far) { far = d; b = k; }
  }
  let c = -1, d = -1, most = 0, least = 0;
  const abx = kept[3 * b] - kept[3 * a], aby = kept[3 * b + 1] - kept[3 * a + 1], abz = kept[3 * b + 2] - kept[3 * a + 2];
  for (let k = 0; k < n; k++) {
    if (k === a || k === b) continue;
    const x = kept[3 * k] - kept[3 * a], y = kept[3 * k + 1] - kept[3 * a + 1], z = kept[3 * k + 2] - kept[3 * a + 2];
    // Twice the triangle's signed area, measured along the normal.
    const area = (aby * z - abz * y) * nx + (abz * x - abx * z) * ny + (abx * y - aby * x) * nz;
    if (area > most) { most = area; c = k; }
    if (area < least) { least = area; d = k; }
  }
  addPoint(out, kept[3 * a], kept[3 * a + 1], kept[3 * a + 2], keptSep[a], keptId[a]);
  if (b !== a) addPoint(out, kept[3 * b], kept[3 * b + 1], kept[3 * b + 2], keptSep[b], keptId[b]);
  if (c >= 0) addPoint(out, kept[3 * c], kept[3 * c + 1], kept[3 * c + 2], keptSep[c], keptId[c]);
  if (d >= 0) addPoint(out, kept[3 * d], kept[3 * d + 1], kept[3 * d + 2], keptSep[d], keptId[d]);
}

/**
 * Edge i of a against edge j of b, along the unit (nx, ny, nz), which already
 * points from a to b: the edge of each that reaches furthest towards the
 * other, and the one point where they pass closest.
 */
function edgeContact(a, b, i, j, nx, ny, nz, margin, out) {
  boxEdge(a, i, nx, ny, nz, segA);
  boxEdge(b, j, -nx, -ny, -nz, segB);
  closestSegments(segA, segB);
  const pax = segA[0] + (segA[3] - segA[0]) * segS, pay = segA[1] + (segA[4] - segA[1]) * segS, paz = segA[2] + (segA[5] - segA[2]) * segS;
  const pbx = segB[0] + (segB[3] - segB[0]) * segT, pby = segB[1] + (segB[4] - segB[1]) * segT, pbz = segB[2] + (segB[5] - segB[2]) * segT;
  const separation = (pbx - pax) * nx + (pby - pay) * ny + (pbz - paz) * nz;
  if (separation > margin) return;
  // Apart, and nearest off the end of either edge: the edges don't actually
  // pass each other, so the gap along this axis isn't the gap. Left to
  // boxBoxClosest, as for corners.
  if (separation > 0 && (segS === 0 || segS === 1 || segT === 0 || segT === 1)) return;
  setNormal(out, nx, ny, nz);
  addPoint(out, (pax + pbx) * 0.5, (pay + pby) * 0.5, (paz + pbz) * 0.5, separation, (1 << 13) | (3 * i + j));
}

/** The edge of box x along its axis i that reaches furthest along (nx, ny, nz), as a segment into s. */
function boxEdge(x, i, nx, ny, nz, s) {
  const X = x.axes, h = x.half;
  let cx = x.center[0], cy = x.center[1], cz = x.center[2];
  for (let k = 0; k < 3; k++) {
    if (k === i) continue;
    const t = sign(X[3 * k] * nx + X[3 * k + 1] * ny + X[3 * k + 2] * nz) * h[k];
    cx += X[3 * k] * t; cy += X[3 * k + 1] * t; cz += X[3 * k + 2] * t;
  }
  const ex = X[3 * i] * h[i], ey = X[3 * i + 1] * h[i], ez = X[3 * i + 2] * h[i];
  s[0] = cx - ex; s[1] = cy - ey; s[2] = cz - ez;
  s[3] = cx + ex; s[4] = cy + ey; s[5] = cz + ez;
}

// ---- hulls ----
//
// Separating axes between convex polyhedra (Gregorius, "The Separating Axis
// Test between Convex Polyhedra", GDC 2013). The candidates are each hull's
// face normals, and the cross product of an edge of each, but only for the
// pairs of edges that make a face of the two hulls' Minkowski difference:
// those whose arcs cross on the sphere of directions, an edge's arc running
// between the normals of the faces either side of it. Any other pair can't
// be where the hulls are nearest, and its axis would say nothing true. A face
// is kept unless an edge beats it by more than the tolerance, as between
// boxes; the other hull's face most against it is then clipped to its sides,
// for up to four points reduced as boxes' are, and two edges give one point.
// Apart within the margin with nothing clipped, the hulls are nearest at a
// corner or an edge's end, and GJK finds where.

/** A unit cube's hull: a box against a hull is placed as this, scaled. */
const CUBE = build([-0.5, -0.5, -0.5, 0.5, -0.5, -0.5, -0.5, 0.5, -0.5, 0.5, 0.5, -0.5,
  -0.5, -0.5, 0.5, 0.5, -0.5, 0.5, -0.5, 0.5, 0.5, 0.5, 0.5, 0.5]);

/** The next power of two at least n: scratch grows by doubling, once per larger hull. */
function grown(n) {
  let c = 16;
  while (c < n) c *= 2;
  return c;
}

/** A hull placed in the world for one call: its corners, its faces' planes, and each edge's arc. */
export class Placed {
  constructor() {
    this.hull = null;
    this.v = new Float64Array(24);
    this.planes = new Float64Array(24);
    /** Per edge, the normal of the face on its second side crossed with the first's: the arc's axis. */
    this.arcs = new Float64Array(36);
    this.center = new Float64Array(3);
    // For a body's own (see fresh()): the step and epoch it was placed at.
    this.step = -1;
    this.epoch = -1;
  }

  /**
   * Shape s placed as set() would, but kept: a hull body is placed once a
   * step and shared by all its pairs, not placed again for each one, 6 to 10
   * times a step in a pile. A body doesn't move while the step finds its
   * contacts, and anything that moves it between steps changes the World's
   * epoch, so the same step and epoch and hull mean the same placement, to
   * the bit. Measured: see ROADMAP.md, milestone 16.
   */
  fresh(s) {
    if (this.hull === s.hull && this.step === s.step && this.epoch === s.epoch) return this;
    this.set(s);
    this.step = s.step;
    this.epoch = s.epoch;
    return this;
  }

  /** Shape s, a hull or a box as CUBE scaled, placed in the world. */
  set(s) {
    const box = s.kind === BOX, h = box ? CUBE : s.hull, hv = h.vertices, hp = h.planes;
    const V = hv.length / 3, F = hp.length / 4, E = h.edges.length / 2;
    this.hull = h;
    if (this.v.length < 3 * V) this.v = new Float64Array(grown(3 * V));
    if (this.planes.length < 4 * F) this.planes = new Float64Array(grown(4 * F));
    if (this.arcs.length < 3 * E) this.arcs = new Float64Array(grown(3 * E));
    const A = s.axes, c = s.center, half = s.half, v = this.v, p = this.planes;
    this.center[0] = c[0]; this.center[1] = c[1]; this.center[2] = c[2];
    for (let i = 0; i < V; i++) {
      let x = hv[3 * i], y = hv[3 * i + 1], z = hv[3 * i + 2];
      if (box) { x *= 2 * half[0]; y *= 2 * half[1]; z *= 2 * half[2]; }
      v[3 * i] = c[0] + A[0] * x + A[3] * y + A[6] * z;
      v[3 * i + 1] = c[1] + A[1] * x + A[4] * y + A[7] * z;
      v[3 * i + 2] = c[2] + A[2] * x + A[5] * y + A[8] * z;
    }
    for (let f = 0; f < F; f++) {
      const x = hp[4 * f], y = hp[4 * f + 1], z = hp[4 * f + 2];
      const nx = A[0] * x + A[3] * y + A[6] * z, ny = A[1] * x + A[4] * y + A[7] * z, nz = A[2] * x + A[5] * y + A[8] * z;
      // A cube's faces are along its axes, so a box's are as far out as its half size that way.
      const d = box ? Math.abs(x) * half[0] + Math.abs(y) * half[1] + Math.abs(z) * half[2] : hp[4 * f + 3];
      p[4 * f] = nx; p[4 * f + 1] = ny; p[4 * f + 2] = nz;
      p[4 * f + 3] = d + nx * c[0] + ny * c[1] + nz * c[2];
    }
    const sd = h.sides, arc = this.arcs;
    for (let e = 0; e < E; e++) {
      const l = 4 * sd[2 * e], r = 4 * sd[2 * e + 1];
      arc[3 * e] = p[r + 1] * p[l + 2] - p[r + 2] * p[l + 1];
      arc[3 * e + 1] = p[r + 2] * p[l] - p[r] * p[l + 2];
      arc[3 * e + 2] = p[r] * p[l + 1] - p[r + 1] * p[l];
    }
    return this;
  }

  /** The least of n·x over the corners. */
  lowest(nx, ny, nz) {
    const v = this.v, V = this.hull.vertices.length / 3;
    let m = Infinity;
    for (let i = 0; i < V; i++) {
      const d = v[3 * i] * nx + v[3 * i + 1] * ny + v[3 * i + 2] * nz;
      if (d < m) m = d;
    }
    return m;
  }
}

const placedA = new Placed(), placedB = new Placed();
// For hullHull's edge loop: each of b's faces' normals against one of a's arcs.
let sideB = new Float64Array(16);
// GJK's answer, for the hulls' fallbacks: the shape gjk.js's Closest has,
// made here so this module needs nothing of gjk.js while it loads.
const near = { distance: 0, a: new Float64Array(3), b: new Float64Array(3) };

/** A contact point's id from the features that made it: mixed, so any number of faces fits in 32 bits. */
function mix(kind, i, j, k) {
  return (Math.imul(kind + 1, 0x9e3779b1) ^ Math.imul(i + 1, 0x85ebca77) ^ Math.imul(j + 1, 0xc2b2ae3d)
    ^ Math.imul(k + 1, 0x27d4eb2f)) >>> 0;
}

function hullHull(a, b, margin, tolerance, out) {
  const A = a.placed !== null ? a.placed.fresh(a) : placedA.set(a);
  const B = b.placed !== null ? b.placed.fresh(b) : placedB.set(b);
  const ha = A.hull, hb = B.hull, pa = A.planes, pb = B.planes;
  const FA = ha.planes.length / 4, FB = hb.planes.length / 4;
  // a's faces: how far b's lowest corner is above each.
  let faceA = -Infinity, fa = 0;
  for (let f = 0; f < FA; f++) {
    const s = B.lowest(pa[4 * f], pa[4 * f + 1], pa[4 * f + 2]) - pa[4 * f + 3];
    if (s > margin) return;
    if (s > faceA) { faceA = s; fa = f; }
  }
  let faceB = -Infinity, fb = 0;
  for (let f = 0; f < FB; f++) {
    const s = A.lowest(pb[4 * f], pb[4 * f + 1], pb[4 * f + 2]) - pb[4 * f + 3];
    if (s > margin) return;
    if (s > faceB) { faceB = s; fb = f; }
  }
  // Pairs of edges whose arcs cross. b's normals are turned round, since the
  // difference is a minus b; the test is Gregorius's with that folded in.
  const ea = ha.edges, sa = ha.sides, eb = hb.edges, sb = hb.sides, va = A.v, vb = B.v, arcA = A.arcs, arcB = B.arcs;
  const EA = ea.length / 2, EB = eb.length / 2;
  if (sideB.length < FB) sideB = new Float64Array(grown(FB));
  const side = sideB;
  let edgeSep = -Infinity, ei = -1, ej = -1, enx = 0, eny = 0, enz = 0;
  for (let i = 0; i < EA; i++) {
    const l = 4 * sa[2 * i], r = 4 * sa[2 * i + 1];
    const ax = arcA[3 * i], ay = arcA[3 * i + 1], az = arcA[3 * i + 2];
    const u = 3 * ea[2 * i], w = 3 * ea[2 * i + 1];
    const ux = va[w] - va[u], uy = va[w + 1] - va[u + 1], uz = va[w + 2] - va[u + 2];
    const ul = Math.sqrt(ux * ux + uy * uy + uz * uz);
    // The first test reads only b's two faces at an edge against this arc, so
    // each face's is worked out once here, not twice for every edge of b it
    // borders: F dot products where it was 2E (12 corners: 20 against 60).
    for (let g = 0; g < FB; g++) side[g] = pb[4 * g] * ax + pb[4 * g + 1] * ay + pb[4 * g + 2] * az;
    for (let j = 0; j < EB; j++) {
      const cba = side[sb[2 * j]], dba = side[sb[2 * j + 1]];
      if (!(cba * dba < 0)) continue;
      const bx = arcB[3 * j], by = arcB[3 * j + 1], bz = arcB[3 * j + 2];
      const adc = pa[l] * bx + pa[l + 1] * by + pa[l + 2] * bz, bdc = pa[r] * bx + pa[r + 1] * by + pa[r + 2] * bz;
      if (!(adc * bdc < 0 && cba * bdc < 0)) continue;
      const p = 3 * eb[2 * j], q = 3 * eb[2 * j + 1];
      const tx = vb[q] - vb[p], ty = vb[q + 1] - vb[p + 1], tz = vb[q + 2] - vb[p + 2];
      const len = normalize(uy * tz - uz * ty, uz * tx - ux * tz, ux * ty - uy * tx);
      if (!(len > PARALLEL * ul * Math.sqrt(tx * tx + ty * ty + tz * tz))) continue;
      let nx = unit[0], ny = unit[1], nz = unit[2];
      // Away from a: the edge is on a's surface, its centre inside.
      if (nx * (va[u] - A.center[0]) + ny * (va[u + 1] - A.center[1]) + nz * (va[u + 2] - A.center[2]) < 0) {
        nx = -nx; ny = -ny; nz = -nz;
      }
      const s = nx * (vb[p] - va[u]) + ny * (vb[p + 1] - va[u + 1]) + nz * (vb[p + 2] - va[u + 2]);
      if (s > margin) return;
      if (s > edgeSep) { edgeSep = s; ei = i; ej = j; enx = nx; eny = ny; enz = nz; }
    }
  }
  if (ei >= 0 && edgeSep > Math.max(faceA, faceB) + tolerance) hullEdges(A, ei, B, ej, enx, eny, enz, margin, out);
  else if (faceB > faceA + tolerance) hullFace(B, fb, A, true, margin, out);
  else hullFace(A, fa, B, false, margin, out);
  if (out.count === 0 && Math.max(faceA, faceB, edgeSep) > 0) nearest(a, b, 0, margin, out);
}

// The incident face, clipped to the reference face's sides: its points, the
// feature that made each, and the line each edge leaving a point lies on (an
// incident edge, or a side plane past CUT).
let clipP = new Float64Array(48), clipQ = new Float64Array(48);
let featP = new Int32Array(16), featQ = new Int32Array(16), lineP = new Int32Array(16), lineQ = new Int32Array(16);
let keptP = new Float64Array(48), keptS = new Float64Array(16), keptI = new Uint32Array(16);
const CUT = 1 << 20;

/**
 * Face rf of placed hull R as the reference, against the face of I most
 * nearly opposite it. `flipped`: R is the collide() call's b, and the normal,
 * which points from R to I here, is turned to point from a to b.
 */
function hullFace(R, rf, I, flipped, margin, out) {
  const rp = R.planes, ip = I.planes, rh = R.hull, ih = I.hull;
  const nx = rp[4 * rf], ny = rp[4 * rf + 1], nz = rp[4 * rf + 2], d = rp[4 * rf + 3];
  let inc = 0, least = Infinity;
  for (let f = 0; f < ih.planes.length / 4; f++) {
    const dot = ip[4 * f] * nx + ip[4 * f + 1] * ny + ip[4 * f + 2] * nz;
    if (dot < least) { least = dot; inc = f; }
  }
  const i0 = ih.start[inc], m = ih.start[inc + 1] - i0, r0 = rh.start[rf], k = rh.start[rf + 1] - r0;
  if (clipP.length < 3 * (m + k)) {
    const c = grown(m + k);
    clipP = new Float64Array(3 * c); clipQ = new Float64Array(3 * c);
    featP = new Int32Array(c); featQ = new Int32Array(c); lineP = new Int32Array(c); lineQ = new Int32Array(c);
    keptP = new Float64Array(3 * c); keptS = new Float64Array(c); keptI = new Uint32Array(c);
  }
  const iv = I.v, rv = R.v;
  for (let j = 0; j < m; j++) {
    const v = 3 * ih.loops[i0 + j];
    clipP[3 * j] = iv[v]; clipP[3 * j + 1] = iv[v + 1]; clipP[3 * j + 2] = iv[v + 2];
    featP[j] = j; lineP[j] = j;
  }
  // Inside each side: the side's outward normal is its edge crossed with n,
  // since the face's corners run counter-clockwise seen from outside.
  let count = m, src = 0;
  for (let j = 0; j < k && count > 0; j++) {
    const p = 3 * rh.loops[r0 + j], q = 3 * rh.loops[r0 + (j + 1 < k ? j + 1 : 0)];
    const ex = rv[q] - rv[p], ey = rv[q + 1] - rv[p + 1], ez = rv[q + 2] - rv[p + 2];
    const mx = ey * nz - ez * ny, my = ez * nx - ex * nz, mz = ex * ny - ey * nx;
    const offset = mx * rv[p] + my * rv[p + 1] + mz * rv[p + 2];
    count = src === 0
      ? clipSides(clipP, featP, lineP, count, clipQ, featQ, lineQ, mx, my, mz, offset, j)
      : clipSides(clipQ, featQ, lineQ, count, clipP, featP, lineP, mx, my, mz, offset, j);
    src = 1 - src;
  }
  const poly = src === 0 ? clipP : clipQ, feat = src === 0 ? featP : featQ;
  let n = 0;
  for (let j = 0; j < count; j++) {
    const x = poly[3 * j], y = poly[3 * j + 1], z = poly[3 * j + 2];
    const separation = x * nx + y * ny + z * nz - d;
    if (separation > margin) continue;
    const h = separation * 0.5;
    keptP[3 * n] = x - nx * h; keptP[3 * n + 1] = y - ny * h; keptP[3 * n + 2] = z - nz * h;
    keptS[n] = separation;
    keptI[n] = mix(flipped ? 1 : 0, rf, inc, feat[j]);
    n++;
  }
  if (n === 0) return;
  if (flipped) setNormal(out, -nx, -ny, -nz);
  else setNormal(out, nx, ny, nz);
  reducePoints(keptP, keptS, keptI, n, nx, ny, nz, out);
}

/**
 * clipPolygon's work for faces of any number of sides: the polygon in src
 * clipped to (x, y, z)·m <= offset, into dst; returns the count. A point
 * made by the cut is named by the line it was on and the plane, or the two
 * planes for a corner of the reference face.
 */
function clipSides(src, sf, sl, count, dst, df, dl, mx, my, mz, offset, plane) {
  let n = 0;
  for (let v = 0; v < count; v++) {
    const w = v + 1 < count ? v + 1 : 0;
    const ax = src[3 * v], ay = src[3 * v + 1], az = src[3 * v + 2];
    const bx = src[3 * w], by = src[3 * w + 1], bz = src[3 * w + 2];
    const da = ax * mx + ay * my + az * mz - offset, db = bx * mx + by * my + bz * mz - offset;
    const line = sl[v];
    if (da <= 0) {
      dst[3 * n] = ax; dst[3 * n + 1] = ay; dst[3 * n + 2] = az;
      df[n] = sf[v]; dl[n] = line;
      n++;
    }
    if ((da <= 0) !== (db <= 0)) {
      const t = da / (da - db);
      dst[3 * n] = ax + (bx - ax) * t; dst[3 * n + 1] = ay + (by - ay) * t; dst[3 * n + 2] = az + (bz - az) * t;
      df[n] = line < CUT ? CUT + 2 * (line * 4096 + plane) : 2 * CUT + 2 * (Math.min(line - CUT, plane) * 4096 + Math.max(line - CUT, plane));
      dl[n] = da <= 0 ? CUT + plane : line;
      n++;
    }
  }
  return n;
}

/** Edge i of A against edge j of B along n, which points from A to B: the one point where they pass closest. */
function hullEdges(A, i, B, j, nx, ny, nz, margin, out) {
  const ea = A.hull.edges, eb = B.hull.edges, va = A.v, vb = B.v;
  const u = 3 * ea[2 * i], w = 3 * ea[2 * i + 1], p = 3 * eb[2 * j], q = 3 * eb[2 * j + 1];
  for (let k = 0; k < 3; k++) {
    segA[k] = va[u + k]; segA[3 + k] = va[w + k];
    segB[k] = vb[p + k]; segB[3 + k] = vb[q + k];
  }
  closestSegments(segA, segB);
  const pax = segA[0] + (segA[3] - segA[0]) * segS, pay = segA[1] + (segA[4] - segA[1]) * segS, paz = segA[2] + (segA[5] - segA[2]) * segS;
  const pbx = segB[0] + (segB[3] - segB[0]) * segT, pby = segB[1] + (segB[4] - segB[1]) * segT, pbz = segB[2] + (segB[5] - segB[2]) * segT;
  const separation = (pbx - pax) * nx + (pby - pay) * ny + (pbz - paz) * nz;
  if (separation > margin) return;
  // Apart and nearest off an edge's end: left to GJK, as boxes leave it to boxBoxClosest.
  if (separation > 0 && (segS === 0 || segS === 1 || segT === 0 || segT === 1)) return;
  setNormal(out, nx, ny, nz);
  addPoint(out, (pax + pbx) * 0.5, (pay + pby) * 0.5, (paz + pbz) * 0.5, separation, mix(2, i, j, 0));
}

/**
 * Apart: the true closest points of a and b's cores, by GJK, as one point
 * midway between their surfaces. ra is a's radius round its core (a ball's
 * or a capsule's), b's being 0. Returns the distance between the cores.
 */
function nearest(a, b, ra, margin, out) {
  const d = closest(a, b, near);
  if (!(d > 0) || d - ra > margin) return d;
  const nx = (near.b[0] - near.a[0]) / d, ny = (near.b[1] - near.a[1]) / d, nz = (near.b[2] - near.a[2]) / d;
  const separation = d - ra, t = ra + separation * 0.5;
  setNormal(out, nx, ny, nz);
  addPoint(out, near.a[0] + nx * t, near.a[1] + ny * t, near.a[2] + nz * t, separation, mix(3, 0, 0, 0));
  return d;
}

/**
 * A ball against a hull. Its centre outside: the nearest point of the hull,
 * by GJK. Inside: out through the face it is least deep under, which for a
 * point inside a convex solid is always the nearest way out.
 */
function sphereHull(a, b, margin, out) {
  const B = b.placed !== null ? b.placed.fresh(b) : placedB.set(b), p = B.planes, F = B.hull.planes.length / 4, c = a.center, r = a.half[0];
  let most = -Infinity, f = 0;
  for (let k = 0; k < F; k++) {
    const s = p[4 * k] * c[0] + p[4 * k + 1] * c[1] + p[4 * k + 2] * c[2] - p[4 * k + 3];
    if (s > most) { most = s; f = k; }
  }
  if (most - r > margin) return;
  // Outside, unless GJK finds the centre on the surface to rounding.
  if (most > 0 && nearest(a, b, r, margin, out) > 0) return;
  // From the ball into the face: against the face's outward normal.
  const nx = -p[4 * f], ny = -p[4 * f + 1], nz = -p[4 * f + 2];
  const separation = most - r, t = r + separation * 0.5;
  setNormal(out, nx, ny, nz);
  addPoint(out, c[0] + nx * t, c[1] + ny * t, c[2] + nz * t, separation, mix(4, f, 0, 0));
}

/**
 * A capsule against a hull. Apart: GJK's closest points, and where those are
 * on a face's inside, the segment clipped to that face for a point at each
 * end of what lies over it, so a capsule lying on a hull rests on a line, as
 * on a box. Its segment into the hull: separating axes over the hull's faces
 * and its edges crossed with the segment, as capsuleBoxDeep.
 */
function capsuleHull(a, b, margin, tolerance, out) {
  const B = b.placed !== null ? b.placed.fresh(b) : placedB.set(b), p = B.planes, h = B.hull, F = h.planes.length / 4, r = a.half[0];
  segment(a, segA);
  const d = closest(a, b, near);
  if (d > 0) {
    if (d - r > margin) return;
    const nx = (near.b[0] - near.a[0]) / d, ny = (near.b[1] - near.a[1]) / d, nz = (near.b[2] - near.a[2]) / d;
    // The face most against the way to the hull: nearest on its inside when
    // the way is its normal, to within PARALLEL as an angle.
    let f = 0, most = -Infinity;
    for (let k = 0; k < F; k++) {
      const dot = -(p[4 * k] * nx + p[4 * k + 1] * ny + p[4 * k + 2] * nz);
      if (dot > most) { most = dot; f = k; }
    }
    if (1 - most <= PARALLEL && clipToHullFace(B, f, r, margin, out)) return;
    nearest(a, b, r, margin, out);
    return;
  }
  const dx = segA[3] - segA[0], dy = segA[4] - segA[1], dz = segA[5] - segA[2];
  const dl = Math.sqrt(dx * dx + dy * dy + dz * dz);
  let bestSep = -Infinity, bestFace = -1, edgeI = -1, nx = 0, ny = 0, nz = 0;
  for (let k = 0; k < F; k++) {
    const mx = p[4 * k], my = p[4 * k + 1], mz = p[4 * k + 2];
    const s = Math.min(segA[0] * mx + segA[1] * my + segA[2] * mz, segA[3] * mx + segA[4] * my + segA[5] * mz) - p[4 * k + 3] - r;
    if (s > bestSep) { bestSep = s; bestFace = k; }
  }
  const e = h.edges, v = B.v, E = e.length / 2;
  for (let i = 0; i < E; i++) {
    const u = 3 * e[2 * i], w = 3 * e[2 * i + 1];
    const tx = v[w] - v[u], ty = v[w + 1] - v[u + 1], tz = v[w + 2] - v[u + 2];
    const l = normalize(dy * tz - dz * ty, dz * tx - dx * tz, dx * ty - dy * tx);
    if (!(l > PARALLEL * dl * Math.sqrt(tx * tx + ty * ty + tz * tz))) continue;
    let mx = unit[0], my = unit[1], mz = unit[2];
    // Out of the hull through this edge: away from its centre.
    if (mx * (v[u] - B.center[0]) + my * (v[u + 1] - B.center[1]) + mz * (v[u + 2] - B.center[2]) < 0) { mx = -mx; my = -my; mz = -mz; }
    const top = -B.lowest(-mx, -my, -mz);
    // Only the edge that reaches farthest out that way: a box has four edges
    // along each axis, and three of them are inside along it.
    if (top - (mx * v[u] + my * v[u + 1] + mz * v[u + 2]) > tolerance) continue;
    const s = Math.min(segA[0] * mx + segA[1] * my + segA[2] * mz, segA[3] * mx + segA[4] * my + segA[5] * mz) - r - top;
    if (s > bestSep + tolerance) { bestSep = s; edgeI = i; nx = mx; ny = my; nz = mz; }
  }
  if (bestSep > margin) return;
  if (edgeI < 0) {
    clipToHullFace(B, bestFace, r, margin, out);
    return;
  }
  // The hull's edge and the segment, where they pass closest: from the capsule into the hull is -m.
  const u = 3 * e[2 * edgeI], w = 3 * e[2 * edgeI + 1];
  for (let k = 0; k < 3; k++) { segB[k] = v[u + k]; segB[3 + k] = v[w + k]; }
  closestSegments(segA, segB);
  const px = segA[0] + dx * segS, py = segA[1] + dy * segS, pz = segA[2] + dz * segS;
  const qx = segB[0] + (segB[3] - segB[0]) * segT, qy = segB[1] + (segB[4] - segB[1]) * segT, qz = segB[2] + (segB[5] - segB[2]) * segT;
  const separation = (px - qx) * nx + (py - qy) * ny + (pz - qz) * nz - r;
  setNormal(out, -nx, -ny, -nz);
  const t = r + separation * 0.5;
  addPoint(out, px - nx * t, py - ny * t, pz - nz * t, separation, mix(5, edgeI, 0, 0));
}

/**
 * The capsule's segment (segA) clipped to the sides of face f of placed hull
 * B: a point at each end of what lies over the face, each with its own
 * separation less the radius r. Normal: into the face. Returns whether it
 * added any point.
 */
function clipToHullFace(B, f, r, margin, out) {
  const p = B.planes, h = B.hull, v = B.v;
  const nx = p[4 * f], ny = p[4 * f + 1], nz = p[4 * f + 2];
  const i0 = h.start[f], k = h.start[f + 1] - i0;
  let u0 = 0, u1 = 1;
  for (let j = 0; j < k; j++) {
    const s = 3 * h.loops[i0 + j], q = 3 * h.loops[i0 + (j + 1 < k ? j + 1 : 0)];
    const ex = v[q] - v[s], ey = v[q + 1] - v[s + 1], ez = v[q + 2] - v[s + 2];
    const mx = ey * nz - ez * ny, my = ez * nx - ex * nz, mz = ex * ny - ey * nx;
    const offset = mx * v[s] + my * v[s + 1] + mz * v[s + 2];
    const d0 = segA[0] * mx + segA[1] * my + segA[2] * mz - offset, d1 = segA[3] * mx + segA[4] * my + segA[5] * mz - offset;
    if (d0 > 0 && d1 > 0) return false;
    if (d0 > 0) u0 = Math.max(u0, d0 / (d0 - d1));
    else if (d1 > 0) u1 = Math.min(u1, d0 / (d0 - d1));
  }
  if (u0 > u1) return false;
  setNormal(out, -nx, -ny, -nz);
  for (let e = 0; e < (u1 > u0 ? 2 : 1); e++) {
    const u = e === 0 ? u0 : u1;
    const x = segA[0] + (segA[3] - segA[0]) * u, y = segA[1] + (segA[4] - segA[1]) * u, z = segA[2] + (segA[5] - segA[2]) * u;
    const above = x * nx + y * ny + z * nz - p[4 * f + 3], separation = above - r;
    if (separation > margin) continue;
    // Midway between the capsule's surface, r below the segment, and the face, `above` below it.
    const t = (above + r) * 0.5;
    addPoint(out, x - nx * t, y - ny * t, z - nz * t, separation, mix(6, f, e, 0));
  }
  return out.count > 0;
}
