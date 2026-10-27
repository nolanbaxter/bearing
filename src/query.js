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
//
// A mesh or heightfield is asked about its triangles: the query goes into
// the body's own frame, and the mesh's tree (or the heightfield's grid) says
// which triangles to test. Every triangle has two sides, as it does for
// contacts, and seams make no difference to a query: a ray hits where the
// triangles are. A compound is asked about each of its parts, and the
// nearest, or any, answers for the body.

import { Shape, Manifold, collide, Parts, placePart, reachAlong, SPHERE, CAPSULE, BOX, HULL as HULL_KIND, TRIANGLE } from './collide.js';
import { load, PIXEL } from './contact.js';
import { raycast as walk } from './tree.js';
import { closest, Closest, radius } from './gjk.js';
import { touch } from './triangle.js';
import { triangleNormal } from './mesh.js';
import { rotate } from './core/math.js';
import { apart } from './joints.js';

const STATIC = 1, MOVING_TREE = 1;   // World's static type code; Broadphase's moving tree
const MESH = 5;                      // World's shape code; shapes from it on are triangles
/** Every edge and corner a corner from both sides (mesh.js): pure geometry, with no seams. */
const BARE = 63;

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
const tri = new Shape(), local = new Float64Array(3), dir = new Float64Array(3);
tri.kind = TRIANGLE;

/** Triangle t of slot s's mesh or heightfield, placed in the world, into tri. */
function triangle(world, s, t) {
  const g = world._geometry[s], p = tri.points, pos = world._position, s3 = 3 * s;
  g.triangle(t, p);
  for (let v = 0; v < 9; v += 3) {
    rotate(world._rotation, 4 * s, p[v], p[v + 1], p[v + 2], false, local);
    p[v] = pos[s3] + local[0]; p[v + 1] = pos[s3 + 1] + local[1]; p[v + 2] = pos[s3 + 2] + local[2];
  }
  for (let k = 0; k < 3; k++) tri.center[k] = (p[k] + p[k + 3] + p[k + 6]) / 3;
  triangleNormal(p, tri.axes);
  return tri;
}

/**
 * A box's half-extents (gx, gy, gz) along the world's axes, as half-extents
 * along slot s's own axes: each of its axes reaches |that axis·world axis|
 * of each. From the rotation now, not the axes the last step worked with.
 */
function extentsIn(world, s, gx, gy, gz, out = [0, 0, 0]) {
  out[0] = 0; out[1] = 0; out[2] = 0;
  for (let j = 0; j < 3; j++) {
    const g = j === 0 ? gx : j === 1 ? gy : gz;
    rotate(world._rotation, 4 * s, j === 0 ? 1 : 0, j === 1 ? 1 : 0, j === 2 ? 1 : 0, true, dir);
    out[0] += Math.abs(dir[0]) * g; out[1] += Math.abs(dir[1]) * g; out[2] += Math.abs(dir[2]) * g;
  }
  return out;
}

/** (x, y, z) into slot s's own frame, into out: a point, or a direction when `point` is false. */
function inward(world, s, x, y, z, isPoint, out) {
  const p = world._position, s3 = 3 * s;
  if (isPoint) { x -= p[s3]; y -= p[s3 + 1]; z -= p[s3 + 2]; }
  return rotate(world._rotation, 4 * s, x, y, z, true, out);
}

/** Slot s's shape where it is now, into out. Not for a body with parts: see shapeOf(). */
export function place(world, s, out) {
  load(world, s, out);   // its kind and size; the axes load copies are the step's, so replaced:
  const p = world._position, s3 = 3 * s;
  return out.place(p[s3], p[s3 + 1], p[s3 + 2], world._rotation, 4 * s);
}

/** How many shapes slot s is made of: its parts, or 1, itself. */
function shapes(world, s) {
  const g = world._inner[s];
  return g instanceof Parts ? g.list.length : 1;
}

/** Slot s's shape k (a part, or the body itself) where it is now, into out. */
function shapeOf(world, s, k, out) {
  const g = world._inner[s];
  return g instanceof Parts ? placePart(g.list[k], world._position, 3 * s, world._rotation, 4 * s, out) : place(world, s, out);
}

/**
 * Half a placed shape's extent along each world axis, into out: a box
 * centred on its centre that holds it. A hull isn't symmetric about its
 * centre, so it gets its farther reach each way.
 */
function extents(s, out) {
  const a = s.axes, h = s.half;
  if (s.kind === SPHERE) {
    out[0] = out[1] = out[2] = h[0];
  } else if (s.kind === CAPSULE) {
    for (let k = 0; k < 3; k++) out[k] = Math.abs(a[3 + k]) * h[1] + h[0];
  } else if (s.kind === BOX) {
    for (let k = 0; k < 3; k++) out[k] = Math.abs(a[k]) * h[0] + Math.abs(a[3 + k]) * h[1] + Math.abs(a[6 + k]) * h[2];
  } else {
    for (let k = 0; k < 3; k++) {
      out[k] = Math.max(reachAlong(s, k === 0 ? 1 : 0, k === 1 ? 1 : 0, k === 2 ? 1 : 0), reachAlong(s, k === 0 ? -1 : 0, k === 1 ? -1 : 0, k === 2 ? -1 : 0));
    }
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
    const o = 6 * bp.leaf[s];
    for (let k = 0; k < shapes(world, s); k++) {
      extents(shapeOf(world, s, k, body), ext);
      const c = body.center;
      if (c[0] - ext[0] < fat[o] || c[1] - ext[1] < fat[o + 1] || c[2] - ext[2] < fat[o + 2]
        || c[0] + ext[0] > fat[o + 3] || c[1] + ext[1] > fat[o + 4] || c[2] + ext[2] > fat[o + 5]) {
        list.push(s);
        break;
      }
    }
  }
  world._straysAt = world._stepCount;
  world._straysEpoch = world._epoch;
  return list;
}

/**
 * Whether a query with this mask and ignore looks at slot s. Never a sensor:
 * a bullet's ray shouldn't stop at an invisible trigger zone, which is the
 * trap Unity's opposite default sets. What is in a zone is for events and
 * world.touching().
 */
const wanted = (world, s, mask, ignore) => s !== ignore && (world._layers[s] & mask) !== 0 && world._sensor[s] === 0;

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
  } else if (s.kind === HULL_KIND) {
    // Its faces' half spaces: the ray is inside the hull where it is inside
    // every one, so it enters at the last face it crosses going in, if that
    // comes before the first it crosses going out.
    const P = s.hull.planes;
    let enter = -Infinity, exit = maxT, face = -1, within = true;
    for (let f = 0; f < P.length; f += 4) {
      const above = P[f] * px + P[f + 1] * py + P[f + 2] * pz - P[f + 3], along = P[f] * ux + P[f + 1] * uy + P[f + 2] * uz;
      if (above > 0) within = false;
      if (along === 0) {
        if (above > 0) return -1;
        continue;
      }
      const at = -above / along;
      if (along < 0) {
        if (at > enter) { enter = at; face = f; }
      } else if (at < exit) exit = at;
    }
    if (within) return inside(dx, dy, dz);
    if (face < 0 || enter < 0 || enter > exit) return -1;
    t = enter;
    nx = P[face]; ny = P[face + 1]; nz = P[face + 2];
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

/**
 * Where the ray meets the triangle in p, from either side, before maxT, with
 * its unit normal turned back at the ray into `normal`; or -1. Edges count
 * as the triangle's own, so a ray down the line between two triangles hits.
 * Möller and Trumbore's test.
 */
function rayTriangle(p, ox, oy, oz, dx, dy, dz, maxT) {
  const e1x = p[3] - p[0], e1y = p[4] - p[1], e1z = p[5] - p[2], e2x = p[6] - p[0], e2y = p[7] - p[1], e2z = p[8] - p[2];
  const hx = dy * e2z - dz * e2y, hy = dz * e2x - dx * e2z, hz = dx * e2y - dy * e2x;
  const a = e1x * hx + e1y * hy + e1z * hz;
  if (a === 0) return -1;   // along its plane: it passes the triangle edge-on
  const f = 1 / a, sx = ox - p[0], sy = oy - p[1], sz = oz - p[2];
  const u = f * (sx * hx + sy * hy + sz * hz);
  if (u < 0 || u > 1) return -1;
  const qx = sy * e1z - sz * e1y, qy = sz * e1x - sx * e1z, qz = sx * e1y - sy * e1x;
  const v = f * (dx * qx + dy * qy + dz * qz);
  if (v < 0 || u + v > 1) return -1;
  const t = f * (e2x * qx + e2y * qy + e2z * qz);
  if (t < 0 || t >= maxT) return -1;
  let nx = e1y * e2z - e1z * e2y, ny = e1z * e2x - e1x * e2z, nz = e1x * e2y - e1y * e2x;
  const l = Math.sqrt(nx * nx + ny * ny + nz * nz) * (nx * dx + ny * dy + nz * dz > 0 ? -1 : 1);
  normal[0] = nx / l; normal[1] = ny / l; normal[2] = nz / l;
  return t;
}

/**
 * Where the ray first meets slot s's mesh or heightfield before maxT, with
 * the world normal in `normal`; or -1. Tested in the body's own frame.
 */
function rayLevel(world, s, ox, oy, oz, dx, dy, dz, maxT) {
  const g = world._geometry[s], p = tri.points;
  const [lx, ly, lz] = inward(world, s, ox, oy, oz, true, local);
  const [ux, uy, uz] = inward(world, s, dx, dy, dz, false, dir);
  let best = -1, nx = 0, ny = 0, nz = 0;
  g.ray(lx, ly, lz, ux, uy, uz, maxT, 0, 0, 0, (t) => {
    g.triangle(t, p);
    const at = rayTriangle(p, lx, ly, lz, ux, uy, uz, maxT);
    if (at >= 0) { best = maxT = at; nx = normal[0]; ny = normal[1]; nz = normal[2]; }
    return maxT;
  });
  if (best >= 0) rotate(world._rotation, 4 * s, nx, ny, nz, false, normal);
  return best;
}

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
  // A stray can be reached through the tree as well; it counts once. Strays
  // are rare (bodies that left their fat boxes within the last step), so the
  // set telling them apart is only made when there are some.
  const bp = world._broadphase, extra = strays(world), seen = extra.length > 0 ? new Set() : null;
  const visit = (s) => {
    if (!wanted(world, s, mask, ignore)) return maxT;
    if (seen !== null) {
      if (seen.has(s)) return maxT;
      seen.add(s);
    }
    let t = -1;
    if (world._shape[s] >= MESH) t = rayLevel(world, s, ox, oy, oz, dx, dy, dz, maxT);
    // The nearest of its parts: each one tried only out to the nearest so far, so a hit is nearer.
    else for (let k = 0; k < shapes(world, s); k++) {
      const at = rayShape(shapeOf(world, s, k, body), ox, oy, oz, dx, dy, dz, t < 0 ? maxT : t);
      if (at >= 0) t = at;
    }
    if (t < 0) return maxT;
    hit(s, t, normal);
    if (nearest) maxT = t;
    return maxT;
  };
  for (const tree of bp.trees) maxT = walk(tree, ox, oy, oz, dx, dy, dz, maxT, 0, 0, 0, visit);
  for (const s of extra) visit(s);
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
function advance(cast, b, scale, ox, oy, oz, dx, dy, dz, maxT, sliding = false) {
  const ra = radius(cast), rb = radius(b), tolerance = scale * PIXEL;
  // Cores closer than this are meeting: GJK's closest points are rounding
  // there, half a double's bits of the shapes' size, and give no direction.
  const meeting = scale / 67108864;   // 2^-26
  let t = 0;
  for (let i = 0; i < ADVANCES; i++) {
    cast.center[0] = ox + t * dx; cast.center[1] = oy + t * dy; cast.center[2] = oz + t * dz;
    const d = closest(cast, b, near), gap = d - ra - rb;
    if (gap < tolerance) {
      // Sliding (a character, see sweep()): already in contact at the start,
      // it is stopped only if the move could take it further in than it is,
      // or than touching, by more than rounding. The gap between two convex
      // shapes, one sliding straight, never closes faster than it starts to,
      // so that rate over the whole move is the furthest in it can get. So
      // moving off the body doesn't stop it, nor moving along it, even with
      // rounding's tilt into it (a floor's normal 2e-14 off up) from rounding's
      // depth in it (1e-16).
      if (sliding && i === 0 && d > meeting) {
        const opening = ((near.a[0] - near.b[0]) * dx + (near.a[1] - near.b[1]) * dy + (near.a[2] - near.b[2]) * dz) / d;
        if (opening >= 0 || gap + opening * maxT >= Math.min(gap, 0) - meeting) return -1;
      }
      if (d > meeting) {
        // Apart, or touching: the normal runs from the body's core to the cast's.
        const nx = (near.a[0] - near.b[0]) / d, ny = (near.a[1] - near.b[1]) / d, nz = (near.a[2] - near.b[2]) / d;
        normal[0] = nx; normal[1] = ny; normal[2] = nz;
        point[0] = near.b[0] + nx * rb; point[1] = near.b[1] + ny * rb; point[2] = near.b[2] + nz * rb;
      } else if ((b.kind === TRIANGLE ? touch(cast, b, BARE, null, 0, tolerance, manifold, 0) : collide(cast, b, 0, tolerance, manifold)) > 0) {
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
  if (s.kind === HULL_KIND) return s.hull.size;
  return 2 * Math.min(s.half[0], s.half[1], s.half[2]);
}

/** advance() against each shape of slot s, the nearest touch: its distance, or -1, with `normal` and `point` its. */
function advanceBody(world, cast, s, ox, oy, oz, dx, dy, dz, maxT, sliding) {
  const scale = scaleOf(world, s, cast);
  let t = -1;
  for (let k = 0; k < shapes(world, s); k++) {
    const at = advance(cast, shapeOf(world, s, k, body), scale, ox, oy, oz, dx, dy, dz, t < 0 ? maxT : t, sliding);
    if (at >= 0) t = at;
  }
  return t;
}

/** Whether `shape` touches slot s, any of its shapes, within the tolerance; the last contact found into `manifold`. */
function collideBody(world, shape, s, tolerance) {
  for (let k = 0; k < shapes(world, s); k++) {
    if (collide(shape, shapeOf(world, s, k, body), 0, tolerance, manifold) > 0) return true;
  }
  return false;
}

/** The size a query against slot s scales its tolerance by: the smaller, as for a contact; a mesh has none of its own. */
function scaleOf(world, s, shape) {
  const own = world._size[s];
  return own > 0 ? Math.min(own, size(shape)) : size(shape);
}

/**
 * Where `cast` first touches slot s's mesh or heightfield, as advance()
 * does for one shape, testing each triangle its sweep can reach.
 */
function castLevel(world, cast, s, ox, oy, oz, dx, dy, dz, maxT, gx, gy, gz) {
  const g = world._geometry[s], scale = size(cast);
  const [lx, ly, lz] = inward(world, s, ox, oy, oz, true, local);
  const [ux, uy, uz] = inward(world, s, dx, dy, dz, false, dir);
  const [hx, hy, hz] = extentsIn(world, s, gx, gy, gz);
  let best = -1;
  const n = [0, 0, 0], at = [0, 0, 0];
  g.ray(lx, ly, lz, ux, uy, uz, maxT, hx, hy, hz, (t) => {
    const hit = advance(cast, triangle(world, s, t), scale, ox, oy, oz, dx, dy, dz, maxT);
    if (hit >= 0) {
      best = maxT = hit;
      n[0] = normal[0]; n[1] = normal[1]; n[2] = normal[2];
      at[0] = point[0]; at[1] = point[1]; at[2] = point[2];
    }
    return maxT;
  });
  normal.set(n); point.set(at);
  return best;
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
    const t = world._shape[s] >= MESH ? castLevel(world, cast, s, ox, oy, oz, dx, dy, dz, maxT, gx, gy, gz)
      : advanceBody(world, cast, s, ox, oy, oz, dx, dy, dz, maxT, false);
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
    const tolerance = scaleOf(world, s, shape) * PIXEL;
    if (world._shape[s] < MESH) {
      if (collideBody(world, shape, s, tolerance)) hit(s);
      return;
    }
    // The shape's box in the body's own frame; any triangle in it that it touches.
    const [lx, ly, lz] = inward(world, s, c[0], c[1], c[2], true, local);
    const [hx, hy, hz] = extentsIn(world, s, ext[0], ext[1], ext[2]);
    let touched = false;
    world._geometry[s].triangles(lx - hx, ly - hy, lz - hz, lx + hx, ly + hy, lz + hz, (t) => {
      if (!touched && touch(shape, triangle(world, s, t), BARE, null, 0, tolerance, manifold, t) > 0) touched = true;
    });
    if (touched) hit(s);
  };
  const seen = new Set();
  const once = (s) => { if (!seen.has(s)) { seen.add(s); visit(s); } };
  for (const tree of world._broadphase.trees) {
    tree.query(c[0] - ext[0], c[1] - ext[1], c[2] - ext[2], c[0] + ext[0], c[1] + ext[1], c[2] + ext[2], once);
  }
  for (const s of strays(world)) once(s);
}


// ---- sensors ----

const sensed = new Shape(), half = new Float64Array(3);
// What senseTriangle needs; sense() is never re-entered.
let senseWorld = null, senseShape = null, senseSlot = 0, senseTolerance = 0, senseOut = null;

/**
 * Whether slots a and b, a pair with a sensor in it, are within the tolerance
 * of touching where they are now, at the end of a step: the contact into
 * `out`, its normal from a towards b. Two meshes or heightfields never meet,
 * here as in contacts. It runs inside the step, so it allocates nothing.
 */
export function sense(world, a, b, tolerance, out) {
  out.count = 0;
  const levelA = world._shape[a] >= MESH, levelB = world._shape[b] >= MESH;
  if (levelA && levelB) return 0;
  if (levelA) {
    sense(world, b, a, tolerance, out);
    const n = out.normal;
    n[0] = -n[0]; n[1] = -n[1]; n[2] = -n[2];
    return out.count;
  }
  if (!levelB) {
    // The first of a's shapes and b's that touch.
    for (let i = 0; i < shapes(world, a); i++) {
      for (let j = 0; j < shapes(world, b); j++) {
        if (collide(shapeOf(world, a, i, sensed), shapeOf(world, b, j, body), tolerance, tolerance, out) > 0) return out.count;
      }
    }
    return 0;
  }
  // b's triangles under each of a's shapes' boxes, in b's own frame, as overlap() finds them.
  for (let i = 0; i < shapes(world, a) && out.count === 0; i++) senseLevel(world, shapeOf(world, a, i, sensed), b, tolerance, out);
  return out.count;
}

/** sense() for shape s against the triangles of level slot b. */
function senseLevel(world, s, b, tolerance, out) {
  extents(s, ext);
  inward(world, b, s.center[0], s.center[1], s.center[2], true, local);
  const lx = local[0], ly = local[1], lz = local[2];
  extentsIn(world, b, ext[0] + tolerance, ext[1] + tolerance, ext[2] + tolerance, half);
  senseWorld = world; senseShape = s; senseSlot = b; senseTolerance = tolerance; senseOut = out;
  world._geometry[b].triangles(lx - half[0], ly - half[1], lz - half[2], lx + half[0], ly + half[1], lz + half[2], senseTriangle);
  senseWorld = null; senseShape = null; senseOut = null;
}

function senseTriangle(t) {
  if (senseOut.count > 0) return;
  touch(senseShape, triangle(senseWorld, senseSlot, t), BARE, null, senseTolerance, senseTolerance, senseOut, t);
}

// ---- for character bodies (character.js) ----
//
// The same casts as shapeCast, but run inside the step, so with no closures
// and nothing allocated: module state, and named visitors. A character is
// blocked by what it would collide with, by its own layers and mask both
// ways, and never by a sensor or itself. And a cast that starts in contact
// with a body and moves off it, or along it, isn't stopped by it (advance's
// `sliding`): a character walking on a floor is always within a pixel of it.

let sweepWorld = null, sweepShape = null, sweepSelf = -1, sweepOther = -1, sweepBest = -1, sweepSkip = null;
let sweepOx = 0, sweepOy = 0, sweepOz = 0, sweepDx = 0, sweepDy = 0, sweepDz = 0, sweepMax = 0;
let sweepGx = 0, sweepGy = 0, sweepGz = 0;
const sweepNormal = new Float64Array(3), sweepPoint = new Float64Array(3);
let levelSlot = -1, levelBest = -1, levelMax = 0;
const levelNormal = new Float64Array(3), levelPoint = new Float64Array(3), levelHalf = new Float64Array(3);

/**
 * Whether character slot s is blocked by slot o: as the two would collide,
 * both ways, and o isn't `other`. A body a joint holds to it, as a lantern on
 * a pole, doesn't block it unless the joint lets them collide (joints.js).
 */
function blocks(world, s, o, other) {
  return o !== s && o !== other && world._sensor[o] === 0
    && (world._layers[s] & world._mask[o]) !== 0 && (world._layers[o] & world._mask[s]) !== 0
    && !(world._jointHead[s] !== 0 && world._jointHead[o] !== 0 && apart(world, s, o));
}

/**
 * Slides `cast` (slot self's shape, placed where the move starts) along the
 * unit (dx, dy, dz) for up to maxT, and returns the first slot it would
 * touch, or -1. Into out: [distance, normal (from the body towards the cast),
 * point]. Leaves out slot `other` as well as self, or none if -1, and any slot
 * `skip` (a column of flags, or null) marks: `skipped` of them, and when that
 * is every body in the moving tree, the tree isn't walked at all.
 */
export function sweep(world, cast, self, other, dx, dy, dz, maxT, out, skip = null, skipped = 0) {
  sweepWorld = world; sweepShape = cast; sweepSelf = self; sweepOther = other; sweepBest = -1; sweepSkip = skip;
  sweepOx = cast.center[0]; sweepOy = cast.center[1]; sweepOz = cast.center[2];
  sweepDx = dx; sweepDy = dy; sweepDz = dz; sweepMax = maxT;
  extents(cast, ext);
  sweepGx = ext[0]; sweepGy = ext[1]; sweepGz = ext[2];
  const trees = world._broadphase.trees;
  for (let k = 0; k < trees.length; k++) {
    if (k === MOVING_TREE && skip !== null && trees[k].leaves === skipped) continue;
    sweepMax = walk(trees[k], sweepOx, sweepOy, sweepOz, dx, dy, dz, sweepMax, sweepGx, sweepGy, sweepGz, sweepVisit);
  }
  const loose = strays(world);
  for (let i = 0; i < loose.length; i++) sweepVisit(loose[i]);
  cast.center[0] = sweepOx; cast.center[1] = sweepOy; cast.center[2] = sweepOz;
  if (sweepBest >= 0) {
    out[0] = sweepMax;
    out[1] = sweepNormal[0]; out[2] = sweepNormal[1]; out[3] = sweepNormal[2];
    out[4] = sweepPoint[0]; out[5] = sweepPoint[1]; out[6] = sweepPoint[2];
  }
  sweepWorld = null; sweepShape = null; sweepSkip = null;
  return sweepBest;
}

function sweepVisit(s) {
  const world = sweepWorld;
  if ((sweepSkip !== null && sweepSkip[s] !== 0) || !blocks(world, sweepSelf, s, sweepOther)) return sweepMax;
  const t = world._shape[s] >= MESH ? sweepLevel(s)
    : advanceBody(world, sweepShape, s, sweepOx, sweepOy, sweepOz, sweepDx, sweepDy, sweepDz, sweepMax, true);
  if (t < 0) return sweepMax;
  sweepBest = s;
  sweepMax = t;
  sweepNormal.set(normal);
  sweepPoint.set(point);
  return sweepMax;
}

/** sweepVisit's cast against a mesh or heightfield in slot s: castLevel, with nothing allocated. */
function sweepLevel(s) {
  const world = sweepWorld;
  inward(world, s, sweepOx, sweepOy, sweepOz, true, local);
  const lx = local[0], ly = local[1], lz = local[2];
  inward(world, s, sweepDx, sweepDy, sweepDz, false, dir);
  const ux = dir[0], uy = dir[1], uz = dir[2];
  extentsIn(world, s, sweepGx, sweepGy, sweepGz, levelHalf);
  levelSlot = s; levelBest = -1; levelMax = sweepMax;
  world._geometry[s].ray(lx, ly, lz, ux, uy, uz, sweepMax, levelHalf[0], levelHalf[1], levelHalf[2], sweepTriangle);
  if (levelBest < 0) return -1;
  normal.set(levelNormal);
  point.set(levelPoint);
  return levelMax;
}

function sweepTriangle(t) {
  const at = advance(sweepShape, triangle(sweepWorld, levelSlot, t), size(sweepShape), sweepOx, sweepOy, sweepOz, sweepDx, sweepDy, sweepDz, levelMax, true);
  if (at >= 0) {
    levelBest = t; levelMax = at;
    levelNormal.set(normal); levelPoint.set(point);
  }
  return levelMax;
}

let deepWorld = null, deepShape = null, deepSelf = -1, deepSlot = -1, deepSep = 0, deepTolerance = 0, deepLevel = -1;
let deepGx = 0, deepGy = 0, deepGz = 0;   // the shape's half-extents: ext is strays()' scratch too
const deepNormal = new Float64Array(3), deepFolds = new Float64Array(6);   // deepFolds: a triangle's folds (mesh.js)

/**
 * The deepest overlap of `shape` (slot self's, placed) with anything that
 * blocks it: into out [separation, normal], the separation negative and the
 * normal from the shape towards the body, as collide() gives them; returns
 * the slot, or -1 if it overlaps nothing. A mesh's seams count as seams, so
 * a character overlapping a flat floor is pushed up, not off an edge.
 */
export function deepest(world, shape, self, out) {
  deepWorld = world; deepShape = shape; deepSelf = self; deepSlot = -1; deepSep = 0;
  extents(shape, ext);
  deepGx = ext[0]; deepGy = ext[1]; deepGz = ext[2];
  const c = shape.center, trees = world._broadphase.trees;
  for (let k = 0; k < trees.length; k++) {
    trees[k].query(c[0] - deepGx, c[1] - deepGy, c[2] - deepGz, c[0] + deepGx, c[1] + deepGy, c[2] + deepGz, deepVisit);
  }
  const loose = strays(world);
  for (let i = 0; i < loose.length; i++) deepVisit(loose[i]);
  if (deepSlot >= 0) { out[0] = deepSep; out[1] = deepNormal[0]; out[2] = deepNormal[1]; out[3] = deepNormal[2]; }
  deepWorld = null; deepShape = null;
  return deepSlot;
}

function deepVisit(s) {
  const world = deepWorld;
  if (!blocks(world, deepSelf, s, -1)) return;
  deepTolerance = scaleOf(world, s, deepShape) * PIXEL;
  if (world._shape[s] < MESH) {
    for (let k = 0; k < shapes(world, s); k++) {
      if (collide(deepShape, shapeOf(world, s, k, body), 0, deepTolerance, manifold) > 0) deepKeep(s);
    }
    return;
  }
  const c = deepShape.center;
  inward(world, s, c[0], c[1], c[2], true, local);
  const lx = local[0], ly = local[1], lz = local[2];
  extentsIn(world, s, deepGx, deepGy, deepGz, levelHalf);
  deepLevel = s;
  world._geometry[s].triangles(lx - levelHalf[0], ly - levelHalf[1], lz - levelHalf[2],
    lx + levelHalf[0], ly + levelHalf[1], lz + levelHalf[2], deepTriangle);
}

function deepTriangle(t) {
  const g = deepWorld._geometry[deepLevel];
  if (touch(deepShape, triangle(deepWorld, deepLevel, t), g.edge(t, deepFolds), deepFolds, 0, deepTolerance, manifold, t) > 0) deepKeep(deepLevel);
}

/** The manifold just found, if deeper than the deepest so far. */
function deepKeep(s) {
  let least = 0;
  for (let k = 0; k < manifold.count; k++) if (manifold.separations[k] < least) least = manifold.separations[k];
  if (least >= deepSep) return;
  deepSep = least;
  deepSlot = s;
  deepNormal.set(manifold.normal);
}
