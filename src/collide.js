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
// Nothing here allocates: everything works in module scratch, which is safe
// because a step never re-enters it.

/** Shape kinds, numbered in the order pairs are dispatched: a pair is always handled lower first. */
export const SPHERE = 0, CAPSULE = 1, BOX = 2;
/**
 * One triangle of a mesh or heightfield, placed in the world. Not dispatched
 * by collide(): a triangle against a shape is triangle.js's.
 */
export const TRIANGLE = 3;

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
  }

  /** Places the shape: `q` is a unit quaternion [x, y, z, w] at q[o..o+3]. */
  place(px, py, pz, q, o = 0) {
    this.center[0] = px; this.center[1] = py; this.center[2] = pz;
    const x = q[o], y = q[o + 1], z = q[o + 2], w = q[o + 3];
    const a = this.axes;
    a[0] = 1 - 2 * (y * y + z * z); a[1] = 2 * (x * y + z * w); a[2] = 2 * (x * z - y * w);
    a[3] = 2 * (x * y - z * w); a[4] = 1 - 2 * (x * x + z * z); a[5] = 2 * (y * z + x * w);
    a[6] = 2 * (x * z + y * w); a[7] = 2 * (y * z - x * w); a[8] = 1 - 2 * (x * x + y * y);
    return this;
  }
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
    else sphereBox(a, b, margin, out);
  } else if (a.kind === CAPSULE) {
    if (b.kind === CAPSULE) capsuleCapsule(a, b, margin, tolerance, out);
    else capsuleBox(a, b, margin, tolerance, out);
  } else {
    boxBox(a, b, margin, tolerance, out);
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

  // a's faces.
  let faceA = -Infinity, axisA = 0;
  for (let i = 0; i < 3; i++) {
    const nx = A[3 * i], ny = A[3 * i + 1], nz = A[3 * i + 2];
    const s = Math.abs(dx * nx + dy * ny + dz * nz) - ha[i] - support(B, hb, nx, ny, nz);
    if (s > margin) return;
    if (s > faceA) { faceA = s; axisA = i; }
  }
  // b's faces.
  let faceB = -Infinity, axisB = 0;
  for (let j = 0; j < 3; j++) {
    const nx = B[3 * j], ny = B[3 * j + 1], nz = B[3 * j + 2];
    const s = Math.abs(dx * nx + dy * ny + dz * nz) - hb[j] - support(A, ha, nx, ny, nz);
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
  keep(out, a);
  if (b !== a) keep(out, b);
  if (c >= 0) keep(out, c);
  if (d >= 0) keep(out, d);
}

function keep(out, k) {
  addPoint(out, kept[3 * k], kept[3 * k + 1], kept[3 * k + 2], keptSep[k], keptId[k]);
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
