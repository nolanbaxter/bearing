// Contacts: which bodies touch, and the solver that keeps them apart.
//
// A step's contact work, in order:
//
//   narrow    every pair in the pair list (pairs.js, kept by the broadphase)
//             is handed to the narrowphase (collide.js), and each pair that
//             touches, or will within a step, becomes a manifold: a normal
//             and up to four points
//   prepare   each point gets what the solver needs and doesn't change during
//             the step: where it is on each body, how hard it is to push
//             (its effective mass), and how fast the bodies met there
//   solve     impulses at each point, several passes, so the bodies stop
//             closing, slide no more than friction allows, and come apart
//             from any overlap
//   relax     the same, without the push-apart term, so the speed used to
//             correct overlap doesn't stay in the bodies as motion
//   restitution  the bounce, last, from the speed the bodies met at
//
// SOFT CONTACTS. A contact isn't a rigid wall here but a very stiff, heavily
// damped spring (Catto, "Solver2D", 2024, and Box2D 3). Its stiffness is the
// most the integrator can follow at the step size, so an overlap is pushed out
// over a few substeps instead of in one jolt, and the push carries no energy
// in with it. The solver runs `substeps` small steps of `iterations` passes
// each; see World for what those are.
//
// PATCHES, NOT POINTS. A contact of one or two points can't stop a body
// turning about them: a ball spinning about the vertical turns about its one
// point, and a ball or a capsule rolling turns about the point or the line it
// rests on, so neither friction nor the normal push can slow it. Real contacts
// are small flattened patches, and a patch resists both: friction across its
// width resists spinning, and its push, leaning toward the front as the body
// rolls over it, resists rolling. Here each contact is a patch PIXEL of the
// rolling body's size across, the same pixel every tolerance is built from.
// The most it can resist rolling is the push N leaning to the patch's edge,
// N times its radius; the most it can resist spinning is friction over a
// uniformly pressed disc, 2/3 mu N times its radius. For a ball or a capsule
// of radius r, the patch is 2r/200, so its rolling resistance coefficient,
// patch over radius, is 0.01 at every size: rubber on concrete, a billiard
// ball on cloth. Contacts of three or four points already resist both through
// their spread, and get neither. Nothing here is tuned, and there is nothing
// to set.
//
// WARM STARTING. Each point starts the step with the impulse it ended the
// last one with, matched by the narrowphase's feature ids. A resting stack
// needs the same impulses step after step, so it starts at the answer. The
// impulses are kept on the pair, which lasts as long as the two bodies' fat
// boxes overlap, so nothing has to be looked up: store() writes them after
// the solve, and the next step's carry() reads them back. A patch's rolling
// and spinning impulse is kept the same way, as one angular impulse.
//
// MESHES. A body against a mesh or a heightfield is one pair, but may touch
// it along several directions at once: a box in a valley is pushed by both
// slopes. triangle.js gives one manifold per direction, and the pair keeps a
// Memory of each (pairs.js); a manifold starts from the remembered one whose
// normal is nearest its own, so a box sliding from triangle to triangle of a
// flat floor keeps its impulses all the way across.
//
// TOUCHING. After the solve, each pair is touching or not, and a change is
// an event (events.js): see touched(). A sensor's pairs get no manifold here,
// since a sensor pushes nothing; World asks about them after the step, where
// the bodies end up.

import { collide, Shape, Manifold, SPHERE, CAPSULE, BOX as BOX_KIND } from './collide.js';
import { touchLevel, Touches } from './triangle.js';
import { rotate } from './core/math.js';
import { resized } from './core/slots.js';

const DYNAMIC = 0, STATIC = 1;
const BOX = 0, SPHERE_SHAPE = 1, CAPSULE_SHAPE = 2, MESH = 3;   // World's shape codes; a heightfield is 4

/**
 * A contact may sink this much of the smaller body's thinnest size, and the
 * same fraction sets the narrowphase's tolerance, the speculative margin's
 * floor, the bounce cut-off and how fast an overlap is pushed out. A body seen
 * whole on screen is a few hundred pixels across, so a two-hundredth of it is
 * about a pixel: as much as a contact can be off by without being seen. For a
 * 1 m box that is 5 mm (Box2D's fixed slop is the same); for a 1 cm marble,
 * 0.05 mm; for a 50 m building, 25 cm. One rule, at every scale (rule 31).
 */
export const PIXEL = 1 / 200;

// Points per manifold, and per-point numbers kept.
const P = 4;

/**
 * The torque friction can exert across a uniformly pressed disc of radius R,
 * as a fraction of mu N R: the integral of mu p r over the disc, with
 * p = N / (pi R^2), is 2/3 mu N R. Derived, not chosen.
 */
const SPIN = 2 / 3;

/** Manifolds for one step: grown by doubling, never sized in advance. */
export class Contacts {
  constructor() {
    this.count = 0;
    this.capacity = 0;
    this._alloc(0);
  }

  _alloc(capacity) {
    const m = capacity, p = capacity * P;
    const grow = (name, Type, n) => { this[name] = this[name] ? resized(this[name], n) : new Type(n); };
    grow('a', Int32Array, m); grow('b', Int32Array, m);
    grow('pair', Int32Array, m);   // which pair in the pair list this manifold is
    grow('sub', Int32Array, m);    // a mesh pair's: which of its Memory's manifolds this one is kept as; else -1
    grow('points', Uint8Array, m);
    grow('normal', Float64Array, 3 * m); grow('t1', Float64Array, 3 * m); grow('t2', Float64Array, 3 * m);
    grow('friction', Float64Array, m); grow('restitution', Float64Array, m); grow('tolerance', Float64Array, m);
    // Rolling and spinning (contacts of one or two points; see PATCHES above):
    grow('patch', Float64Array, m);                       // the patch's radius, m
    grow('rollMass1', Float64Array, m); grow('rollMass2', Float64Array, m); grow('spinMass', Float64Array, m);
    grow('rollImpulse1', Float64Array, m); grow('rollImpulse2', Float64Array, m); grow('spinImpulse', Float64Array, m);
    grow('pid', Uint32Array, p);
    grow('separation', Float64Array, p);
    grow('point', Float64Array, 3 * p);
    grow('ra', Float64Array, 3 * p); grow('rb', Float64Array, 3 * p);
    grow('la', Float64Array, 3 * p); grow('lb', Float64Array, 3 * p);
    grow('adjusted', Float64Array, p);
    grow('gap', Float64Array, m);   // its points' least separation as the last pass saw it: at the step's end
    grow('normalMass', Float64Array, p); grow('tangentMass1', Float64Array, p); grow('tangentMass2', Float64Array, p);
    grow('normalImpulse', Float64Array, p); grow('tangentImpulse1', Float64Array, p); grow('tangentImpulse2', Float64Array, p);
    grow('maxNormalImpulse', Float64Array, p); grow('relativeVelocity', Float64Array, p);
    this.capacity = capacity;
  }

  ensure(count) {
    if (count <= this.capacity) return;
    let c = Math.max(this.capacity, 1);
    while (c < count) c *= 2;
    this._alloc(c);
  }

  /**
   * Gives storage back once a step's manifolds fill a quarter of it or less,
   * shrinking to the power of two growth would have picked: the rule the
   * world's own columns follow (see World._remove), so a burst of contacts
   * leaves nothing behind and growing and shrinking by turns can't thrash.
   */
  trim() {
    if (this.count > this.capacity / 4) return;
    let c = this.count === 0 ? 0 : 1;
    while (c < this.count) c *= 2;
    if (c < this.capacity) this._alloc(c);
  }
}

// Each point's closing-speed target and softness for one pass, worked out
// once before the sweeps over it.
const target = new Float64Array(P), massScale = new Float64Array(P), impulseScale = new Float64Array(P);

// ---- find ----

const shapeA = new Shape(), shapeB = new Shape();
const manifold = new Manifold();
const touches = new Touches();
const v3 = new Float64Array(3);

/**
 * Adds to `next` a manifold for each awake pair from `from` on whose shapes
 * touch, or will within the step. A pair that no longer holds (a body gone,
 * filtered apart, or fat boxes parted) is dropped from the list here. Each
 * manifold starts from the impulses its pair remembers. A pair with a sensor
 * in it gets none: it is listed for World to ask about after the step.
 *
 * A touch between an awake body and a sleeping one wakes the sleeper's
 * island. Its other pairs then join the awake ones, past where this pass
 * stops, and the world runs this again from there (see World.step), so a
 * woken stack is held up in the very step it wakes.
 */
export function narrow(world, pairs, broadphase, next, from) {
  const box = world._box, reach = world._motion, awake = world._awake, shape = world._shape, sensor = world._sensor;
  const sensing = world._sensorCount > 0;
  for (let p = from; p < pairs.active;) {
    if (!pairs.holds(p, broadphase)) {
      pairs.remove(p);   // the last pair moves into p, so p is looked at again
      continue;
    }
    const a = pairs.a[p], b = pairs.b[p], a6 = a * 6, b6 = b * 6;
    // Fat boxes overlap, but their actual boxes, grown only by this step's
    // motion, may not: nothing can touch this step, and nothing is carried.
    if (box[a6] > box[b6 + 3] || box[b6] > box[a6 + 3] || box[a6 + 1] > box[b6 + 4]
      || box[b6 + 1] > box[a6 + 4] || box[a6 + 2] > box[b6 + 5] || box[b6 + 2] > box[a6 + 5]) {
      pairs.forget(p);
      p++;
      continue;
    }
    if (sensing && (sensor[a] === 1 || sensor[b] === 1)) {
      world._sense(p);
      p++;
      continue;
    }
    if (shape[a] >= MESH || shape[b] >= MESH) {
      level(world, pairs, p, a, b, next);
      p++;
      continue;
    }
    load(world, a, shapeA);
    load(world, b, shapeB);
    const tolerance = Math.min(world._size[a], world._size[b]) * PIXEL;
    const margin = reach[a] + reach[b] + tolerance;
    if (collide(shapeA, shapeB, margin, tolerance, manifold) === 0) {
      pairs.forget(p);
      p++;
      continue;
    }

    // Static bodies are never awake, and _wake leaves them be.
    if (awake[a] !== awake[b]) world._wake(awake[a] ? b : a);
    const m = emit(world, next, p, a, b, manifold, 1, tolerance, -1);
    carry(pairs, p, next, m);
    p++;
  }
  return next.count;
}

/**
 * Pair p, one of whose bodies is a mesh or a heightfield (never both: neither
 * is ever dynamic): a manifold for each direction the other body touches it
 * along, each started from the remembered manifold nearest its direction.
 */
function level(world, pairs, p, a, b, next) {
  const turned = world._shape[a] >= MESH, g = turned ? a : b, o = turned ? b : a;
  load(world, o, shapeA);
  // A mesh is a surface, with no thickness of its own: the tolerance is the other body's.
  const tolerance = world._size[o] * PIXEL, reach = world._motion;
  const margin = reach[a] + reach[b] + tolerance;
  // The other body's box, already grown by its motion, into the mesh's
  // frame, and grown by the mesh's own motion: where its triangles can be met.
  const box = world._box, o6 = 6 * o, g3 = 3 * g, g9 = 9 * g, A = world._axes, pos = world._position;
  const cx = (box[o6] + box[o6 + 3]) * 0.5 - pos[g3], cy = (box[o6 + 1] + box[o6 + 4]) * 0.5 - pos[g3 + 1];
  const cz = (box[o6 + 2] + box[o6 + 5]) * 0.5 - pos[g3 + 2];
  const hx = (box[o6 + 3] - box[o6]) * 0.5, hy = (box[o6 + 4] - box[o6 + 1]) * 0.5, hz = (box[o6 + 5] - box[o6 + 2]) * 0.5;
  const lx = A[g9] * cx + A[g9 + 1] * cy + A[g9 + 2] * cz;
  const ly = A[g9 + 3] * cx + A[g9 + 4] * cy + A[g9 + 5] * cz;
  const lz = A[g9 + 6] * cx + A[g9 + 7] * cy + A[g9 + 8] * cz;
  const ex = Math.abs(A[g9]) * hx + Math.abs(A[g9 + 1]) * hy + Math.abs(A[g9 + 2]) * hz + reach[g];
  const ey = Math.abs(A[g9 + 3]) * hx + Math.abs(A[g9 + 4]) * hy + Math.abs(A[g9 + 5]) * hz + reach[g];
  const ez = Math.abs(A[g9 + 6]) * hx + Math.abs(A[g9 + 7]) * hy + Math.abs(A[g9 + 8]) * hz + reach[g];
  const n = touchLevel(world._geometry[g], pos, A, g, shapeA, lx - ex, ly - ey, lz - ez, lx + ex, ly + ey, lz + ez,
    margin, tolerance, touches);
  if (n === 0) {
    pairs.forget(p);
    return;
  }
  const awake = world._awake;
  if (awake[a] !== awake[b]) world._wake(awake[a] ? b : a);
  const kept = pairs.memoryOf(p);
  kept.taken.fill(0, 0, kept.count);
  for (let k = 0; k < n; k++) {
    // Normals come out from the other body; a pair's point from a to b.
    const m = emit(world, next, p, a, b, touches.list[k], turned ? -1 : 1, tolerance, k);
    let best = -1, most = 0;
    for (let j = 0; j < kept.count; j++) {
      if (kept.taken[j]) continue;
      const d = kept.normal[3 * j] * next.normal[3 * m] + kept.normal[3 * j + 1] * next.normal[3 * m + 1]
        + kept.normal[3 * j + 2] * next.normal[3 * m + 2];
      if (d > most) { most = d; best = j; }
    }
    if (best < 0) continue;
    kept.taken[best] = 1;
    carry(kept, best, next, m);
  }
}

/**
 * Manifold mf of pair p, between slots a and b, as the next manifold in
 * next, its normal times sign; returns its index. sub is which of a mesh
 * pair's manifolds it is, or -1.
 */
function emit(world, next, p, a, b, mf, sign, tolerance, sub) {
  const m = next.count++;
  next.ensure(next.count);
  next.a[m] = a; next.b[m] = b; next.pair[m] = p; next.sub[m] = sub;
  next.points[m] = mf.count;
  for (let k = 0; k < 3; k++) next.normal[3 * m + k] = sign * mf.normal[k];
  // Friction mixes as a geometric mean, so ice is slippery against
  // anything; restitution takes the larger, so a bouncy ball bounces off
  // anything. Box2D's rules.
  next.friction[m] = Math.sqrt(world._friction[a] * world._friction[b]);
  next.restitution[m] = Math.max(world._restitution[a], world._restitution[b]);
  next.tolerance[m] = tolerance;
  // The patch is the rolling body's: a ball's is the same on any floor,
  // however thin. Between two dynamic bodies, the smaller one's.
  const type = world._type;
  next.patch[m] = PIXEL * (type[a] !== DYNAMIC ? world._size[b] : type[b] !== DYNAMIC ? world._size[a]
    : Math.min(world._size[a], world._size[b]));
  next.rollImpulse1[m] = 0; next.rollImpulse2[m] = 0; next.spinImpulse[m] = 0;
  for (let k = 0; k < mf.count; k++) {
    const i = m * P + k;
    next.pid[i] = mf.ids[k];
    next.separation[i] = mf.separations[k];
    for (let c = 0; c < 3; c++) next.point[3 * i + c] = mf.points[3 * k + c];
    next.normalImpulse[i] = 0; next.tangentImpulse1[i] = 0; next.tangentImpulse2[i] = 0;
  }
  basis(next, m);
  return m;
}

/**
 * What "from" remembers at p (the pair list for pair p, or a mesh pair's
 * Memory for its manifold p) onto this step's points with the same feature
 * id. Friction is remembered as a vector in world space and laid onto this
 * step's tangents, since the tangents turn with the normal from step to step.
 */
function carry(from, p, next, m) {
  const remembered = from.points[p];
  if (remembered === 0) return;
  if (next.points[m] <= 2) {
    const p3 = 3 * p, m3 = 3 * m;
    const lx = from.twist[p3], ly = from.twist[p3 + 1], lz = from.twist[p3 + 2];
    next.rollImpulse1[m] = lx * next.t1[m3] + ly * next.t1[m3 + 1] + lz * next.t1[m3 + 2];
    next.rollImpulse2[m] = lx * next.t2[m3] + ly * next.t2[m3 + 1] + lz * next.t2[m3 + 2];
    next.spinImpulse[m] = lx * next.normal[m3] + ly * next.normal[m3 + 1] + lz * next.normal[m3 + 2];
  }
  for (let k = 0; k < next.points[m]; k++) {
    const i = m * P + k, id = next.pid[i];
    for (let j = 0; j < remembered; j++) {
      const o = p * P + j;
      if (from.pid[o] !== id) continue;
      next.normalImpulse[i] = from.normalImpulse[o];
      const fx = from.friction[3 * o], fy = from.friction[3 * o + 1], fz = from.friction[3 * o + 2];
      next.tangentImpulse1[i] = fx * next.t1[3 * m] + fy * next.t1[3 * m + 1] + fz * next.t1[3 * m + 2];
      next.tangentImpulse2[i] = fx * next.t2[3 * m] + fy * next.t2[3 * m + 1] + fz * next.t2[3 * m + 2];
      break;
    }
  }
}

/**
 * After the solve: each pair remembers its points' ids and impulses, for the
 * next step; a mesh pair, each of its manifolds, with its normal, in its Memory.
 */
export function store(pairs, c) {
  for (let m = 0; m < c.count; m++) {
    const p = c.pair[m], sub = c.sub[m];
    if (sub < 0) {
      keep(pairs, p, c, m);
      continue;
    }
    // A mesh pair's manifolds come in order, so the last one sets the count.
    const kept = pairs.memoryOf(p);
    kept.ensure(sub + 1);
    kept.count = sub + 1;
    for (let k = 0; k < 3; k++) kept.normal[3 * sub + k] = c.normal[3 * m + k];
    keep(kept, sub, c, m);
  }
}

/** Manifold m's ids and impulses into "to" (the pair list, or a Memory) at p. */
function keep(to, p, c, m) {
  const count = c.points[m], m3 = 3 * m, p3 = 3 * p;
  to.points[p] = count;
  const r1 = c.rollImpulse1[m], r2 = c.rollImpulse2[m], sp = c.spinImpulse[m];
  to.twist[p3] = c.t1[m3] * r1 + c.t2[m3] * r2 + c.normal[m3] * sp;
  to.twist[p3 + 1] = c.t1[m3 + 1] * r1 + c.t2[m3 + 1] * r2 + c.normal[m3 + 1] * sp;
  to.twist[p3 + 2] = c.t1[m3 + 2] * r1 + c.t2[m3 + 2] * r2 + c.normal[m3 + 2] * sp;
  for (let k = 0; k < count; k++) {
    const i = m * P + k, o = p * P + k;
    to.pid[o] = c.pid[i];
    to.normalImpulse[o] = c.normalImpulse[i];
    const l1 = c.tangentImpulse1[i], l2 = c.tangentImpulse2[i];
    to.friction[3 * o] = c.t1[m3] * l1 + c.t2[m3] * l2;
    to.friction[3 * o + 1] = c.t1[m3 + 1] * l1 + c.t2[m3 + 1] * l2;
    to.friction[3 * o + 2] = c.t1[m3 + 2] * l1 + c.t2[m3 + 2] * l2;
  }
}

/**
 * The anchor that tracks a contact through the step, in slot s's own frame,
 * into out[o..]: the contact's offset (x, y, z) from the body's centre, moved
 * onto the shape's core. A ball's surface touches the floor at its bottom
 * however it turns, so the point that touched at the start of a step mustn't
 * turn with it: rolled up and away, it would read as the ball lifting off,
 * and the ball would drop and be caught again every step. Measured before
 * this: a 1 cm marble rolled at 1 m/s hopped a third of its radius and lost
 * 9% of its speed in 5 s. So a ball's anchor is its centre and a capsule's
 * the nearest point of its axis, which move with it as they should, and a
 * box's is the point itself, since a box's surface does turn with it, as a
 * mesh's does.
 * Leaves the anchor's world offset at the start in v3: for a box, the
 * offset it was given, unchanged; a ball's, zero.
 */
function core(world, s, x, y, z, out, o) {
  const shape = world._shape[s];
  if (shape === SPHERE_SHAPE) {
    out[o] = 0; out[o + 1] = 0; out[o + 2] = 0;
    v3[0] = 0; v3[1] = 0; v3[2] = 0;
    return;
  }
  rotate(world._rotation, s * 4, x, y, z, true, v3);
  if (shape !== CAPSULE_SHAPE) {
    out[o] = v3[0]; out[o + 1] = v3[1]; out[o + 2] = v3[2];
    v3[0] = x; v3[1] = y; v3[2] = z;
    return;
  }
  const d = world._dims, half = (d[s * 3 + 1] - 2 * d[s * 3]) * 0.5;
  out[o] = 0; out[o + 1] = v3[1] < -half ? -half : v3[1] > half ? half : v3[1]; out[o + 2] = 0;
  rotate(world._rotation, s * 4, 0, out[o + 1], 0, false, v3);
}

/** A body's shape, placed, into s. Not for a mesh or a heightfield, which are triangles (triangle.js). */
export function load(world, slot, s) {
  const shape = world._shape[slot], d = world._dims, s3 = slot * 3;
  if (shape === BOX) {
    s.kind = BOX_KIND;
    s.half[0] = d[s3] * 0.5; s.half[1] = d[s3 + 1] * 0.5; s.half[2] = d[s3 + 2] * 0.5;
  } else if (shape === SPHERE_SHAPE) {
    s.kind = SPHERE;
    s.half[0] = d[s3];
  } else {
    s.kind = CAPSULE;
    s.half[0] = d[s3];
    s.half[1] = (d[s3 + 1] - 2 * d[s3]) * 0.5;
  }
  s.center[0] = world._position[s3]; s.center[1] = world._position[s3 + 1]; s.center[2] = world._position[s3 + 2];
  const ax = world._axes, a9 = slot * 9;
  for (let k = 0; k < 9; k++) s.axes[k] = ax[a9 + k];
}

/**
 * Two tangents at right angles to the normal, the same pair for the same
 * normal every time: the first is the normal crossed with the world axis it is
 * least along. Friction is solved along these two.
 */
function basis(c, m) {
  const nx = c.normal[3 * m], ny = c.normal[3 * m + 1], nz = c.normal[3 * m + 2];
  const ax = Math.abs(nx), ay = Math.abs(ny), az = Math.abs(nz);
  let tx, ty, tz;
  if (ax <= ay && ax <= az) { tx = 0; ty = nz; tz = -ny; } else if (ay <= az) { tx = -nz; ty = 0; tz = nx; } else { tx = ny; ty = -nx; tz = 0; }
  const l = 1 / Math.sqrt(tx * tx + ty * ty + tz * tz);
  tx *= l; ty *= l; tz *= l;
  c.t1[3 * m] = tx; c.t1[3 * m + 1] = ty; c.t1[3 * m + 2] = tz;
  c.t2[3 * m] = ny * tz - nz * ty; c.t2[3 * m + 1] = nz * tx - nx * tz; c.t2[3 * m + 2] = nx * ty - ny * tx;
}


// ---- prepare ----

/**
 * What the solver needs that holds for the whole step. Each body's world
 * inverse inertia, R diag(1/I) Rᵀ, goes into world._invI as its six distinct
 * entries; then per point the anchors (from each body's centre, in world
 * space and in the body's own frame), the effective masses along the normal
 * and both tangents, and the speed the bodies were closing at.
 */
export function prepare(world, c) {
  const invI = world._invI, invMass = world._invMass;
  // Every body in a contact is awake by now, or static, whose stays zero.
  const list = world._awakeList, n = world._awakeCount;
  for (let i = 0; i < n; i++) inverseInertia(world, list[i]);
  const p = world._position, q = world._rotation, v = world._velocity, w = world._angular;
  for (let m = 0; m < c.count; m++) {
    const a = c.a[m], b = c.b[m], a3 = a * 3, b3 = b * 3;
    const ima = invMass[a], imb = invMass[b];
    const nx = c.normal[3 * m], ny = c.normal[3 * m + 1], nz = c.normal[3 * m + 2];
    if (c.points[m] <= 2) {
      const m3 = 3 * m;
      c.rollMass1[m] = turning(invI, a, b, c.t1[m3], c.t1[m3 + 1], c.t1[m3 + 2]);
      c.rollMass2[m] = turning(invI, a, b, c.t2[m3], c.t2[m3 + 1], c.t2[m3 + 2]);
      c.spinMass[m] = turning(invI, a, b, nx, ny, nz);
    }
    for (let k = 0; k < c.points[m]; k++) {
      const i = m * P + k, i3 = 3 * i;
      const px = c.point[i3], py = c.point[i3 + 1], pz = c.point[i3 + 2];
      const rax = px - p[a3], ray = py - p[a3 + 1], raz = pz - p[a3 + 2];
      const rbx = px - p[b3], rby = py - p[b3 + 1], rbz = pz - p[b3 + 2];
      c.ra[i3] = rax; c.ra[i3 + 1] = ray; c.ra[i3 + 2] = raz;
      c.rb[i3] = rbx; c.rb[i3 + 1] = rby; c.rb[i3 + 2] = rbz;
      // Separation as the bodies move during the step is the starting
      // separation plus how far the two anchors have moved apart along the
      // normal; this is the part of that sum fixed at the start.
      core(world, a, rax, ray, raz, c.la, i3);
      const cax = v3[0], cay = v3[1], caz = v3[2];
      core(world, b, rbx, rby, rbz, c.lb, i3);
      c.adjusted[i] = c.separation[i] - ((v3[0] - cax) * nx + (v3[1] - cay) * ny + (v3[2] - caz) * nz);
      c.normalMass[i] = mass(invI, ima, imb, a, b, rax, ray, raz, rbx, rby, rbz, nx, ny, nz);
      c.tangentMass1[i] = mass(invI, ima, imb, a, b, rax, ray, raz, rbx, rby, rbz, c.t1[3 * m], c.t1[3 * m + 1], c.t1[3 * m + 2]);
      c.tangentMass2[i] = mass(invI, ima, imb, a, b, rax, ray, raz, rbx, rby, rbz, c.t2[3 * m], c.t2[3 * m + 1], c.t2[3 * m + 2]);
      c.maxNormalImpulse[i] = 0;
      // Closing speed along the normal, before anything this step: what a bounce is measured from.
      const dvx = v[b3] + w[b3 + 1] * rbz - w[b3 + 2] * rby - v[a3] - w[a3 + 1] * raz + w[a3 + 2] * ray;
      const dvy = v[b3 + 1] + w[b3 + 2] * rbx - w[b3] * rbz - v[a3 + 1] - w[a3 + 2] * rax + w[a3] * raz;
      const dvz = v[b3 + 2] + w[b3] * rby - w[b3 + 1] * rbx - v[a3 + 2] - w[a3] * ray + w[a3 + 1] * rax;
      c.relativeVelocity[i] = dvx * nx + dvy * ny + dvz * nz;
    }
  }
}

/** world._invI for slot s: R diag(1/I) Rᵀ for a dynamic body, zero for anything else. */
function inverseInertia(world, s) {
  const o = s * 6, out = world._invI;
  if (world._type[s] !== DYNAMIC) { out.fill(0, o, o + 6); return; }
  const ax = world._axes, a9 = s * 9, inv = world._invInertia, s3 = s * 3;
  const i0 = inv[s3], i1 = inv[s3 + 1], i2 = inv[s3 + 2];
  // Columns of R are the body's axes; entry (r, c) is Σ_k R_rk R_ck / I_k.
  const x0 = ax[a9], x1 = ax[a9 + 1], x2 = ax[a9 + 2];
  const y0 = ax[a9 + 3], y1 = ax[a9 + 4], y2 = ax[a9 + 5];
  const z0 = ax[a9 + 6], z1 = ax[a9 + 7], z2 = ax[a9 + 8];
  out[o] = x0 * x0 * i0 + y0 * y0 * i1 + z0 * z0 * i2;       // xx
  out[o + 1] = x0 * x1 * i0 + y0 * y1 * i1 + z0 * z1 * i2;   // xy
  out[o + 2] = x0 * x2 * i0 + y0 * y2 * i1 + z0 * z2 * i2;   // xz
  out[o + 3] = x1 * x1 * i0 + y1 * y1 * i1 + z1 * z1 * i2;   // yy
  out[o + 4] = x1 * x2 * i0 + y1 * y2 * i1 + z1 * z2 * i2;   // yz
  out[o + 5] = x2 * x2 * i0 + y2 * y2 * i1 + z2 * z2 * i2;   // zz
}

/**
 * The effective mass along unit d at a contact: 1 / (1/ma + 1/mb +
 * (ra × d)·Ia⁻¹(ra × d) + (rb × d)·Ib⁻¹(rb × d)). How much impulse it takes to
 * change the bodies' relative speed there by 1 m/s, turning included.
 */
function mass(invI, ima, imb, a, b, rax, ray, raz, rbx, rby, rbz, dx, dy, dz) {
  const cax = ray * dz - raz * dy, cay = raz * dx - rax * dz, caz = rax * dy - ray * dx;
  const cbx = rby * dz - rbz * dy, cby = rbz * dx - rbx * dz, cbz = rbx * dy - rby * dx;
  const k = ima + imb + quadratic(invI, a * 6, cax, cay, caz) + quadratic(invI, b * 6, cbx, cby, cbz);
  return k > 0 ? 1 / k : 0;
}

/** x·M x for the symmetric M at m[o..o+5]. */
function quadratic(m, o, x, y, z) {
  return x * (m[o] * x + m[o + 1] * y + m[o + 2] * z)
    + y * (m[o + 1] * x + m[o + 3] * y + m[o + 4] * z)
    + z * (m[o + 2] * x + m[o + 4] * y + m[o + 5] * z);
}

// ---- solve ----

/**
 * How soft a contact is, as Box2D 3 makes it from a frequency and a damping
 * ratio over the substep h: biasRate turns an overlap into a closing-speed
 * target, massScale and impulseScale blend the spring into the impulse.
 */
export const soft = { biasRate: 0, massScale: 1, impulseScale: 0 };
export function makeSoft(hertz, zeta, h) {
  const omega = 2 * Math.PI * hertz;
  const a1 = 2 * zeta + h * omega, a2 = h * omega * a1, a3 = 1 / (1 + a2);
  soft.biasRate = omega / a1;
  soft.massScale = a2 * a3;
  soft.impulseScale = a3;
}

/**
 * Applies every point's accumulated impulse, at the start of each substep:
 * the warm start, which begins each substep at last time's answer.
 */
export function warmStart(world, c) {
  const v = world._velocity, w = world._angular, invI = world._invI, invMass = world._invMass;
  for (let m = 0; m < c.count; m++) {
    const a = c.a[m], b = c.b[m];
    for (let k = 0; k < c.points[m]; k++) {
      const i = m * P + k, i3 = 3 * i, m3 = 3 * m;
      const ln = c.normalImpulse[i], l1 = c.tangentImpulse1[i], l2 = c.tangentImpulse2[i];
      const px = c.normal[m3] * ln + c.t1[m3] * l1 + c.t2[m3] * l2;
      const py = c.normal[m3 + 1] * ln + c.t1[m3 + 1] * l1 + c.t2[m3 + 1] * l2;
      const pz = c.normal[m3 + 2] * ln + c.t1[m3 + 2] * l1 + c.t2[m3 + 2] * l2;
      push(v, w, invI, invMass, a, b, c.ra, c.rb, i3, px, py, pz);
    }
    if (c.points[m] <= 2) {
      const m3 = 3 * m, r1 = c.rollImpulse1[m], r2 = c.rollImpulse2[m], sp = c.spinImpulse[m];
      twist(w, invI, a, b, c.t1[m3] * r1 + c.t2[m3] * r2 + c.normal[m3] * sp,
        c.t1[m3 + 1] * r1 + c.t2[m3 + 1] * r2 + c.normal[m3 + 1] * sp,
        c.t1[m3 + 2] * r1 + c.t2[m3 + 2] * r2 + c.normal[m3 + 2] * sp);
    }
  }
}

/** Equal and opposite angular impulse (lx, ly, lz): -L on a, +L on b. */
function twist(w, invI, a, b, lx, ly, lz) {
  turn(w, invI, a, -lx, -ly, -lz);
  turn(w, invI, b, lx, ly, lz);
}

/** 1 / (axis · (Ia⁻¹ + Ib⁻¹) axis): how hard the pair is to turn against each other about a unit axis. */
function turning(invI, a, b, x, y, z) {
  const k = along(invI, a * 6, x, y, z) + along(invI, b * 6, x, y, z);
  return k > 0 ? 1 / k : 0;
}

/** axis · I⁻¹ axis, for the inverse inertia at invI[o..]. */
function along(invI, o, x, y, z) {
  return invI[o] * x * x + invI[o + 3] * y * y + invI[o + 5] * z * z
    + 2 * (invI[o + 1] * x * y + invI[o + 2] * x * z + invI[o + 4] * y * z);
}

/** Equal and opposite impulse (px, py, pz) at point i3: -P on a, +P on b. */
function push(v, w, invI, invMass, a, b, ra, rb, i3, px, py, pz) {
  const ima = invMass[a], imb = invMass[b], a3 = a * 3, b3 = b * 3;
  v[a3] -= ima * px; v[a3 + 1] -= ima * py; v[a3 + 2] -= ima * pz;
  v[b3] += imb * px; v[b3 + 1] += imb * py; v[b3 + 2] += imb * pz;
  const rax = ra[i3], ray = ra[i3 + 1], raz = ra[i3 + 2], rbx = rb[i3], rby = rb[i3 + 1], rbz = rb[i3 + 2];
  turn(w, invI, a, -(ray * pz - raz * py), -(raz * px - rax * pz), -(rax * py - ray * px));
  turn(w, invI, b, rby * pz - rbz * py, rbz * px - rbx * pz, rbx * py - rby * px);
}

/** ω += I⁻¹ L for slot s. */
function turn(w, invI, s, lx, ly, lz) {
  const o = s * 6, s3 = s * 3;
  w[s3] += invI[o] * lx + invI[o + 1] * ly + invI[o + 2] * lz;
  w[s3 + 1] += invI[o + 1] * lx + invI[o + 3] * ly + invI[o + 4] * lz;
  w[s3 + 2] += invI[o + 2] * lx + invI[o + 4] * ly + invI[o + 5] * lz;
}

/**
 * One pass over every contact: the normal at each point, then friction.
 * `bias` is true while solving, false while relaxing. h is the substep.
 */
export function solve(world, c, h, bias) {
  const v = world._velocity, w = world._angular, invI = world._invI, invMass = world._invMass;
  const p = world._position, start = world._start, q = world._rotation;
  const inv = 1 / h;
  for (let m = 0; m < c.count; m++) {
    const a = c.a[m], b = c.b[m], a3 = a * 3, b3 = b * 3, m3 = 3 * m;
    const nx = c.normal[m3], ny = c.normal[m3 + 1], nz = c.normal[m3 + 2];
    // How far each body has moved since the step began.
    const dx = (p[b3] - start[b3]) - (p[a3] - start[a3]);
    const dy = (p[b3 + 1] - start[b3 + 1]) - (p[a3 + 1] - start[a3 + 1]);
    const dz = (p[b3 + 2] - start[b3 + 2]) - (p[a3 + 2] - start[a3 + 2]);
    // An overlap is pushed out no faster than a pixel (the tolerance) a substep.
    const maxBias = c.tolerance[m] * inv;
    const count = c.points[m];
    let least = Infinity;
    for (let k = 0; k < count; k++) {
      const i = m * P + k, i3 = 3 * i;
      // The separation now: the anchors where the bodies' current rotations put
      // them. It can't change within a pass, since nothing moves until _move.
      rotate(q, a * 4, c.la[i3], c.la[i3 + 1], c.la[i3 + 2], false, v3);
      const cax = v3[0], cay = v3[1], caz = v3[2];
      rotate(q, b * 4, c.lb[i3], c.lb[i3 + 1], c.lb[i3 + 2], false, v3);
      const s = (dx + v3[0] - cax) * nx + (dy + v3[1] - cay) * ny + (dz + v3[2] - caz) * nz + c.adjusted[i];
      if (s < least) least = s;
      if (s > 0) {
        // Not touching yet: allowed to close exactly the gap this substep, no more.
        target[k] = s * inv; massScale[k] = 1; impulseScale[k] = 0;
      } else if (bias) {
        target[k] = Math.max(soft.biasRate * s, -maxBias);
        massScale[k] = soft.massScale; impulseScale[k] = soft.impulseScale;
      } else {
        target[k] = 0; massScale[k] = 1; impulseScale[k] = 0;
      }
    }
    // The last relaxing pass comes after the last move, so this is where the step leaves them.
    c.gap[m] = least;
    // One sweep per point. The points of a face contact are coupled: four
    // points on a box's face hold only three things (its height and its two
    // tilts), and a single sweep visits them in an order, so the first stops
    // the whole fall on its own and tips the box, and friction turns the tip
    // into a slide. Sweeping the manifold's own points as many times as it
    // has points lets each one's correction reach every other. Measured, a 1 m
    // box dropped flat 2.5 m at 4 substeps: 7.6 mm of slide with one sweep,
    // 0.04 mm with two, none to a micrometre with four. A one-point contact
    // gets one sweep, which already solves it exactly.
    for (let sweep = 0; sweep < count; sweep++) {
      for (let k = 0; k < count; k++) {
        const i = m * P + k, i3 = 3 * i;
        const vn = closing(v, w, a3, b3, c.ra, c.rb, i3, nx, ny, nz);
        const old = c.normalImpulse[i];
        let next = old - c.normalMass[i] * massScale[k] * (vn + target[k]) - impulseScale[k] * old;
        if (next < 0) next = 0;
        const d = next - old;
        c.normalImpulse[i] = next;
        // The first push at a point this step is the moment of impact: the
        // speed it closes at now is what a bounce sends back. Taken at the
        // start of the step instead, it would miss what gravity added before
        // the bodies met, a twentieth of a slow bounce at 60 Hz.
        if (d > 0 && c.maxNormalImpulse[i] === 0) c.relativeVelocity[i] = vn;
        if (d > c.maxNormalImpulse[i]) c.maxNormalImpulse[i] = d;
        push(v, w, invI, invMass, a, b, c.ra, c.rb, i3, nx * d, ny * d, nz * d);
      }
    }
    // Friction: each point resists sliding along both tangents, and the two
    // together may not exceed friction times the normal impulse: a cone, so a
    // box sliding diagonally slows exactly as one sliding straight.
    const mu = c.friction[m];
    const t1x = c.t1[m3], t1y = c.t1[m3 + 1], t1z = c.t1[m3 + 2];
    const t2x = c.t2[m3], t2y = c.t2[m3 + 1], t2z = c.t2[m3 + 2];
    for (let k = 0; k < count; k++) {
      const i = m * P + k, i3 = 3 * i;
      const v1 = closing(v, w, a3, b3, c.ra, c.rb, i3, t1x, t1y, t1z);
      const v2 = closing(v, w, a3, b3, c.ra, c.rb, i3, t2x, t2y, t2z);
      const o1 = c.tangentImpulse1[i], o2 = c.tangentImpulse2[i];
      let n1 = o1 - c.tangentMass1[i] * v1, n2 = o2 - c.tangentMass2[i] * v2;
      const limit = mu * c.normalImpulse[i];
      const l2 = n1 * n1 + n2 * n2;
      if (l2 > limit * limit) {
        const scale = limit / Math.sqrt(l2);
        n1 *= scale; n2 *= scale;
      }
      c.tangentImpulse1[i] = n1; c.tangentImpulse2[i] = n2;
      const d1 = n1 - o1, d2 = n2 - o2;
      push(v, w, invI, invMass, a, b, c.ra, c.rb, i3, t1x * d1 + t2x * d2, t1y * d1 + t2y * d2, t1z * d1 + t2z * d2);
    }
    if (count > 2) continue;
    // Rolling and spinning (see PATCHES): the turning of b against a about
    // the two tangents, held within the patch's push leaning to its edge, a
    // disc as friction's cone is; then about the normal, within friction
    // across the patch, for one point (two already resist it with their spread).
    let pressed = 0;
    for (let k = 0; k < count; k++) pressed += c.normalImpulse[m * P + k];
    const most = c.patch[m] * pressed;
    let ux = w[b3] - w[a3], uy = w[b3 + 1] - w[a3 + 1], uz = w[b3 + 2] - w[a3 + 2];
    const r1 = c.rollImpulse1[m], r2 = c.rollImpulse2[m];
    let n1 = r1 - c.rollMass1[m] * (ux * t1x + uy * t1y + uz * t1z);
    let n2 = r2 - c.rollMass2[m] * (ux * t2x + uy * t2y + uz * t2z);
    const l2 = n1 * n1 + n2 * n2;
    if (l2 > most * most) {
      const scale = most / Math.sqrt(l2);
      n1 *= scale; n2 *= scale;
    }
    c.rollImpulse1[m] = n1; c.rollImpulse2[m] = n2;
    const d1 = n1 - r1, d2 = n2 - r2;
    twist(w, invI, a, b, t1x * d1 + t2x * d2, t1y * d1 + t2y * d2, t1z * d1 + t2z * d2);
    if (count > 1) continue;
    ux = w[b3] - w[a3]; uy = w[b3 + 1] - w[a3 + 1]; uz = w[b3 + 2] - w[a3 + 2];
    const s0 = c.spinImpulse[m], limit = SPIN * mu * most;
    let s1 = s0 - c.spinMass[m] * (ux * nx + uy * ny + uz * nz);
    if (s1 > limit) s1 = limit; else if (s1 < -limit) s1 = -limit;
    c.spinImpulse[m] = s1;
    const ds = s1 - s0;
    twist(w, invI, a, b, nx * ds, ny * ds, nz * ds);
  }
}

/**
 * After the solve: which pairs touched in this step, and an event for each
 * that started or stopped. A pair touched if the solver pushed it apart
 * anywhere, or it ends the step within the tolerance of touching: the push
 * catches a ball that hits and leaves within one step, and the distance a box
 * resting against a wall with nothing pressing it. A begin carries the
 * deepest point, moved on by half of what the two bodies moved this step, so
 * it lands where they met rather than where the step started them; the
 * pair's normal; and the fastest closing speed among its points at impact.
 */
export function touched(world, pairs, c) {
  const events = world._events, p3 = world._position, start = world._start;
  for (let m = 0; m < c.count;) {
    const p = c.pair[m];
    // A mesh pair's manifolds come together; any one of them touching will do.
    let hit = -1;
    for (; m < c.count && c.pair[m] === p; m++) {
      if (hit >= 0) continue;
      let meets = c.gap[m] <= c.tolerance[m];
      for (let k = 0; k < c.points[m] && !meets; k++) meets = c.maxNormalImpulse[m * P + k] > 0;
      if (meets) hit = m;
    }
    const now = hit >= 0 ? 1 : 0;
    if (now === pairs.touching[p]) continue;
    pairs.touching[p] = now;
    if (now === 0) {
      events.end(pairs.idA[p], pairs.idB[p]);
      continue;
    }
    let deepest = hit * P, speed = 0;
    for (let k = 0; k < c.points[hit]; k++) {
      const i = hit * P + k;
      if (c.separation[i] < c.separation[deepest]) deepest = i;
      if (-c.relativeVelocity[i] > speed) speed = -c.relativeVelocity[i];
    }
    const a3 = 3 * c.a[hit], b3 = 3 * c.b[hit], d3 = 3 * deepest, h3 = 3 * hit;
    events.begin(pairs.idA[p], pairs.idB[p],
      c.point[d3] + (p3[a3] - start[a3] + p3[b3] - start[b3]) * 0.5,
      c.point[d3 + 1] + (p3[a3 + 1] - start[a3 + 1] + p3[b3 + 1] - start[b3 + 1]) * 0.5,
      c.point[d3 + 2] + (p3[a3 + 2] - start[a3 + 2] + p3[b3 + 2] - start[b3 + 2]) * 0.5,
      c.normal[h3], c.normal[h3 + 1], c.normal[h3 + 2], speed);
  }
}

/** Relative velocity of b against a at point i3, along unit (dx, dy, dz). */
function closing(v, w, a3, b3, ra, rb, i3, dx, dy, dz) {
  const rax = ra[i3], ray = ra[i3 + 1], raz = ra[i3 + 2], rbx = rb[i3], rby = rb[i3 + 1], rbz = rb[i3 + 2];
  const x = v[b3] + w[b3 + 1] * rbz - w[b3 + 2] * rby - v[a3] - w[a3 + 1] * raz + w[a3 + 2] * ray;
  const y = v[b3 + 1] + w[b3 + 2] * rbx - w[b3] * rbz - v[a3 + 1] - w[a3 + 2] * rax + w[a3] * raz;
  const z = v[b3 + 2] + w[b3] * rby - w[b3 + 1] * rbx - v[a3 + 2] - w[a3] * ray + w[a3 + 1] * rax;
  return x * dx + y * dy + z * dz;
}

/**
 * The bounce, once the substeps are done: each point that met faster than
 * the cut-off, and actually pushed, is sent apart at its restitution times
 * the speed it met at. The cut-off is the speed of a bounce too small to see,
 * one that would rise no higher than the tolerance: √(2 g tolerance). Below
 * it, a body settles instead of bouncing for ever on rounding.
 */
export function restitution(world, c, gravity) {
  const v = world._velocity, w = world._angular, invI = world._invI, invMass = world._invMass;
  for (let m = 0; m < c.count; m++) {
    const e = c.restitution[m];
    if (e === 0) continue;
    const threshold = Math.sqrt(2 * gravity * c.tolerance[m]);
    const a = c.a[m], b = c.b[m], a3 = a * 3, b3 = b * 3, m3 = 3 * m;
    const nx = c.normal[m3], ny = c.normal[m3 + 1], nz = c.normal[m3 + 2];
    for (let k = 0; k < c.points[m]; k++) {
      const i = m * P + k, i3 = 3 * i;
      if (c.relativeVelocity[i] > -threshold || c.maxNormalImpulse[i] === 0) continue;
      const vn = closing(v, w, a3, b3, c.ra, c.rb, i3, nx, ny, nz);
      const old = c.normalImpulse[i];
      let next = old - c.normalMass[i] * (vn + e * c.relativeVelocity[i]);
      if (next < 0) next = 0;
      const d = next - old;
      c.normalImpulse[i] = next;
      push(v, w, invI, invMass, a, b, c.ra, c.rb, i3, nx * d, ny * d, nz * d);
    }
  }
}
