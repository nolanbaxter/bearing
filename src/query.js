// Queries: rays, shape casts and overlap tests, against the world as it is.
//
// Each walks the broadphase's two trees (broadphase.js), static and moving,
// and tests only the bodies whose fat boxes it reaches, exactly, against the
// body's shape where it is now: its current position and rotation, not the
// ones the last step started from, so a query between steps, or just after a
// teleport, sees what is there.
//
// A body can, rarely, end a step outside its fat box: struck hard enough
// mid-step to move further than the box's margins. The next step re-files it,
// but a query before then would walk past it, so the first query after each
// step lists such bodies once and tests them directly. That reads the tree
// and changes nothing, so asking questions never changes what the world does.
//
// Queries run outside the step, when the user calls them, and return fresh
// objects; they cost nothing until called.

import { Shape, Manifold, collide, SPHERE, CAPSULE } from './collide.js';
import { load, PIXEL } from './contact.js';
import { raycast as walk } from './tree.js';
import { closest, Closest, radius } from './gjk.js';

const STATIC = 1, MOVING_TREE = 1;   // World's static type code; Broadphase's moving tree

/**
 * Conservative advancement stops once the gap is under the contact tolerance,
 * about a pixel; each advance lands short of the touch, never past it, so a
 * cast that hasn't closed to within that in this many advances is grazing
 * past, and is counted a miss. Measured over 2,000 random casts of every
 * shape, any turn, into scenes of 60 bodies: a hit took at most 6 advances,
 * most 2 or 3. This is ten times that.
 */
const ADVANCES = 64;

// Scratch, since queries aren't re-entered.
const body = new Shape(), manifold = new Manifold(), near = new Closest();
const normal = new Float64Array(3), point = new Float64Array(3);

/** Slot s's shape where it is now, into out. */
function place(world, s, out) {
  load(world, s, out);   // its kind and size; the axes load copies are the step's, so replaced:
  const p = world._position, s3 = 3 * s;
  return out.place(p[s3], p[s3 + 1], p[s3 + 2], world._rotation, 4 * s);
}

/** Half a placed shape's extent along each world axis, into out. */
function extents(s, out) {
  const a = s.axes, h = s.half;
  if (s.kind === SPHERE) {
    out[0] = out[1] = out[2] = h[0];
  } else if (s.kind === CAPSULE) {
    for (let k = 0; k < 3; k++) out[k] = Math.abs(a[3 + k]) * h[1] + h[0];
  } else {
    for (let k = 0; k < 3; k++) out[k] = Math.abs(a[k]) * h[0] + Math.abs(a[3 + k]) * h[1] + Math.abs(a[6 + k]) * h[2];
  }
  return out;
}

const ext = new Float64Array(3);

/**
 * Bodies that ended the last step outside their fat boxes, listed once a step
 * and only when a query asks. Only awake dynamic bodies can: static and
 * sleeping ones don't move, and a kinematic one moves only at the speed its
 * box was grown by.
 */
function strays(world) {
  // Kept until the next step, or anything that adds, removes or moves a body by hand.
  if (world._straysAt === world._stepCount && world._straysEpoch === world._epoch) return world._strays;
  const list = world._strays;
  list.length = 0;
  const bp = world._broadphase, fat = bp.trees[MOVING_TREE].box, type = world._type;
  for (let i = 0; i < world._awakeCount; i++) {
    const s = world._awakeList[i];
    if (type[s] !== 0) continue;
    extents(place(world, s, body), ext);
    const c = body.center, o = 6 * bp.leaf[s];
    if (c[0] - ext[0] < fat[o] || c[1] - ext[1] < fat[o + 1] || c[2] - ext[2] < fat[o + 2]
      || c[0] + ext[0] > fat[o + 3] || c[1] + ext[1] > fat[o + 4] || c[2] + ext[2] > fat[o + 5]) list.push(s);
  }
  world._straysAt = world._stepCount;
  world._straysEpoch = world._epoch;
  return list;
}

/** Whether a query with this mask and ignore looks at slot s. */
const wanted = (world, s, mask, ignore) => s !== ignore && (world._layers[s] & mask) !== 0;

// ---- rays ----

/**
 * Where the ray from (ox, oy, oz) along the unit (dx, dy, dz) first meets
 * shape s, before maxT: the distance, with the surface's outward normal in
 * `normal`; or -1. A ray starting inside meets it at 0, its normal back
 * along the ray.
 */
export function rayShape(s, ox, oy, oz, dx, dy, dz, maxT) {
  const c = s.center, a = s.axes, h = s.half;
  // Into the shape's own frame.
  const rx = ox - c[0], ry = oy - c[1], rz = oz - c[2];
  const px = rx * a[0] + ry * a[1] + rz * a[2], py = rx * a[3] + ry * a[4] + rz * a[5], pz = rx * a[6] + ry * a[7] + rz * a[8];
  const ux = dx * a[0] + dy * a[1] + dz * a[2], uy = dx * a[3] + dy * a[4] + dz * a[5], uz = dx * a[6] + dy * a[7] + dz * a[8];
  let t = -1, nx = 0, ny = 0, nz = 0;
  if (s.kind === SPHERE) {
    const r = h[0], b = px * ux + py * uy + pz * uz, cc = px * px + py * py + pz * pz - r * r;
    if (cc <= 0) return inside(dx, dy, dz);
    const disc = b * b - cc;
    if (b >= 0 || disc < 0) return -1;
    t = -b - Math.sqrt(disc);
    nx = (px + t * ux) / r; ny = (py + t * uy) / r; nz = (pz + t * uz) / r;
  } else if (s.kind === CAPSULE) {
    const r = h[0], half = h[1];
    const cy = py < -half ? -half : py > half ? half : py;
    if (px * px + (py - cy) * (py - cy) + pz * pz <= r * r) return inside(dx, dy, dz);
    // The first way in is the earliest of: the side, a cylinder about y
    // between the ends, and the two end spheres, which cover its flat ends.
    let best = Infinity;
    const A = ux * ux + uz * uz;
    if (A > 0) {
      const B = px * ux + pz * uz, C = px * px + pz * pz - r * r, disc = B * B - A * C;
      if (disc >= 0) {
        const at = (-B - Math.sqrt(disc)) / A, y = py + at * uy;
        if (at >= 0 && y >= -half && y <= half) { best = at; nx = (px + at * ux) / r; ny = 0; nz = (pz + at * uz) / r; }
      }
    }
    for (const e of ENDS) {
      const qy = py - e * half, b = px * ux + qy * uy + pz * uz, cc = px * px + qy * qy + pz * pz - r * r, disc = b * b - cc;
      if (b >= 0 || disc < 0) continue;
      const at = -b - Math.sqrt(disc);
      if (at < best) { best = at; nx = (px + at * ux) / r; ny = (qy + at * uy) / r; nz = (pz + at * uz) / r; }
    }
    if (best === Infinity) return -1;
    t = best;
  } else {
    if (Math.abs(px) <= h[0] && Math.abs(py) <= h[1] && Math.abs(pz) <= h[2]) return inside(dx, dy, dz);
    // The slabs: the ray enters the box where it has entered all three.
    let enter = -Infinity, exit = Infinity, axis = -1, side = 0;
    const p = [px, py, pz], u = [ux, uy, uz];
    for (let k = 0; k < 3; k++) {
      if (u[k] === 0) {
        if (p[k] < -h[k] || p[k] > h[k]) return -1;
        continue;
      }
      let t0 = (-h[k] - p[k]) / u[k], t1 = (h[k] - p[k]) / u[k];
      if (t0 > t1) { const swap = t0; t0 = t1; t1 = swap; }
      if (t0 > enter) { enter = t0; axis = k; side = u[k] > 0 ? -1 : 1; }
      if (t1 < exit) exit = t1;
      if (enter > exit) return -1;
    }
    if (exit < 0 || enter < 0) return -1;
    t = enter;
    if (axis === 0) nx = side; else if (axis === 1) ny = side; else nz = side;
  }
  if (t >= maxT) return -1;
  // The normal back into the world.
  normal[0] = nx * a[0] + ny * a[3] + nz * a[6];
  normal[1] = nx * a[1] + ny * a[4] + nz * a[7];
  normal[2] = nx * a[2] + ny * a[5] + nz * a[8];
  return t;
}

const ENDS = [-1, 1];

function inside(dx, dy, dz) {
  normal[0] = -dx; normal[1] = -dy; normal[2] = -dz;
  return 0;
}

/**
 * Every body the ray meets before maxT, each through hit(slot, distance,
 * normal): nearest returns the distance to carry on with, so only nearer hits
 * are looked for after the first, and all keeps maxT to see everything.
 */
export function ray(world, ox, oy, oz, dx, dy, dz, maxT, mask, ignore, hit, nearest) {
  const bp = world._broadphase, seen = new Set();
  const visit = (s) => {
    // A stray can be reached through the tree as well; it counts once.
    if (seen.has(s) || !wanted(world, s, mask, ignore)) return maxT;
    seen.add(s);
    const t = rayShape(place(world, s, body), ox, oy, oz, dx, dy, dz, maxT);
    if (t < 0) return maxT;
    hit(s, t, normal);
    if (nearest) maxT = t;
    return maxT;
  };
  for (const tree of bp.trees) maxT = walk(tree, ox, oy, oz, dx, dy, dz, maxT, 0, 0, 0, visit);
  for (const s of strays(world)) visit(s);
}

// ---- shape casts ----

/**
 * How far `cast`, moving from where it is along the unit (dx, dy, dz), goes
 * before it touches body slot s, under maxT, by conservative advancement: the
 * gap between the two, over how fast the move closes it, is as far as the
 * shape can surely go; go that far, and look again. The distance between two
 * convex shapes, one sliding in a straight line, is a convex function of how
 * far it has slid, so each such step lands short of the touch, never past
 * it. Returns -1 for no touch, with the normal (from the body towards the
 * cast) in `normal` and the point on the body in `point` otherwise.
 */
function advance(world, cast, s, ox, oy, oz, dx, dy, dz, maxT) {
  const b = place(world, s, body);
  const ra = radius(cast), rb = radius(b);
  const scale = Math.min(world._size[s], size(cast)), tolerance = scale * PIXEL;
  // Cores closer than this are meeting: GJK's closest points are rounding
  // there, half a double's bits of the shapes' size, and give no direction.
  const meeting = scale / 67108864;   // 2^-26
  let t = 0;
  for (let i = 0; i < ADVANCES; i++) {
    cast.center[0] = ox + t * dx; cast.center[1] = oy + t * dy; cast.center[2] = oz + t * dz;
    const d = closest(cast, b, near), gap = d - ra - rb;
    if (gap < tolerance) {
      if (d > meeting) {
        // Apart, or touching: the normal runs from the body's core to the cast's.
        const nx = (near.a[0] - near.b[0]) / d, ny = (near.a[1] - near.b[1]) / d, nz = (near.a[2] - near.b[2]) / d;
        normal[0] = nx; normal[1] = ny; normal[2] = nz;
        point[0] = near.b[0] + nx * rb; point[1] = near.b[1] + ny * rb; point[2] = near.b[2] + nz * rb;
      } else if (collide(cast, b, 0, tolerance, manifold) > 0) {
        // The cores meet: the shapes overlap, deeply, where it started. The
        // narrowphase knows which way out; its normal runs from the cast to the body.
        normal[0] = -manifold.normal[0]; normal[1] = -manifold.normal[1]; normal[2] = -manifold.normal[2];
        point[0] = manifold.points[0]; point[1] = manifold.points[1]; point[2] = manifold.points[2];
      } else {
        normal[0] = -dx; normal[1] = -dy; normal[2] = -dz;
        point[0] = cast.center[0]; point[1] = cast.center[1]; point[2] = cast.center[2];
      }
      return t;
    }
    const closing = -((near.a[0] - near.b[0]) * dx + (near.a[1] - near.b[1]) * dy + (near.a[2] - near.b[2]) * dz) / d;
    if (closing <= 0) return -1;   // moving apart, or alongside: the gap only grows from here
    t += gap / closing;
    if (t >= maxT) return -1;
  }
  return -1;
}

/** A shape's thinnest size, as a body's _size: what its tolerance scales with. */
function size(s) {
  if (s.kind === SPHERE || s.kind === CAPSULE) return 2 * s.half[0];
  return 2 * Math.min(s.half[0], s.half[1], s.half[2]);
}

/**
 * The first body `cast`, placed at its start, touches moving along the unit
 * (dx, dy, dz) before maxT, through hit(slot, distance, normal, point), or
 * nothing.
 */
export function shapeCast(world, cast, dx, dy, dz, maxT, mask, ignore, hit) {
  const ox = cast.center[0], oy = cast.center[1], oz = cast.center[2];
  extents(cast, ext);
  const gx = ext[0], gy = ext[1], gz = ext[2];
  let best = -1;
  const visit = (s) => {
    if (!wanted(world, s, mask, ignore)) return maxT;
    const t = advance(world, cast, s, ox, oy, oz, dx, dy, dz, maxT);
    if (t < 0) return maxT;
    best = s;
    maxT = t;
    hit(s, t, normal, point);
    return maxT;
  };
  for (const tree of world._broadphase.trees) maxT = walk(tree, ox, oy, oz, dx, dy, dz, maxT, gx, gy, gz, visit);
  for (const s of strays(world)) visit(s);
  cast.center[0] = ox; cast.center[1] = oy; cast.center[2] = oz;
  return best;
}

// ---- overlap ----

/** Every body `shape`, placed, overlaps or touches, through hit(slot). */
export function overlap(world, shape, mask, ignore, hit) {
  extents(shape, ext);
  const c = shape.center;
  const visit = (s) => {
    if (!wanted(world, s, mask, ignore)) return;
    const b = place(world, s, body);
    const tolerance = Math.min(world._size[s], size(shape)) * PIXEL;
    if (collide(shape, b, 0, tolerance, manifold) > 0) hit(s);
  };
  const seen = new Set();
  const once = (s) => { if (!seen.has(s)) { seen.add(s); visit(s); } };
  for (const tree of world._broadphase.trees) {
    tree.query(c[0] - ext[0], c[1] - ext[1], c[2] - ext[2], c[0] + ext[0], c[1] + ext[1], c[2] + ext[2], once);
  }
  for (const s of strays(world)) once(s);
}

