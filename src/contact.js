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
// ball on cloth. Contacts of three or four points already resist rolling
// through their spread, and get none. Nothing here is tuned, and there is
// nothing to set.
//
// FRICTION, ONE ANCHOR A MANIFOLD. Friction isn't solved at each point, but
// once for the whole manifold, as Box3D, Jolt and Rapier's default do: the
// two tangents together, as a 2x2 block, at one anchor, held within a circle
// of mu times the manifold's whole normal impulse; then a twist about the
// normal, held within mu times each point's normal impulse times its distance
// from the anchor (Box3D's rule), which is what the points' friction resisted
// spinning with when each had its own. A box face's four corners resist to
// the same limit as before, with three rows where there were eight. One point
// has no spread, so its twist is the patch's spinning friction above.
//
// The anchor is the points' centre, each point weighed by how near it is to
// touching: fully within the tolerance, fading to nothing at twice it, as
// Box3D weighs by its speculative distance. A point further off only matters
// if the bodies close on it, and an anchor pulled toward it, say a tipping
// box's lifted edge, would put friction where nothing presses. Weighing by
// normal impulse instead was Dirk Gregorius's suggestion, which Box3D turned
// down: the impulses aren't known until the solve, and as points take turns
// pressing, a rocking box's, the anchor would jump from step to step and could
// feed the rocking. A manifold whose points are all further than twice the
// tolerance is weighed evenly: it holds nothing until it closes anyway.
//
// WARM STARTING. Each point starts the step with the normal impulse it ended
// the last one with, matched by the narrowphase's feature ids; each manifold
// its friction and twist, which belong to no point. A resting stack needs the
// same impulses step after step, so it starts at the answer. The impulses
// are kept on the pair, which lasts as long as the two bodies' fat boxes
// overlap, so nothing has to be looked up: store() writes them after the
// solve, and the next step's carry() reads them back. Friction is kept as a
// world vector, and rolling and twist as one angular impulse.
//
// MESHES. A body against a mesh or a heightfield is one pair, but may touch
// it along several directions at once: a box in a valley is pushed by both
// slopes. triangle.js gives one manifold per direction, and the pair keeps a
// Memory of each (pairs.js); a manifold starts from the remembered one whose
// normal is nearest its own, so a box sliding from triangle to triangle of a
// flat floor keeps its impulses all the way across.
//
// PARTS. A compound's parts (and the one part of a box, ball or capsule whose
// centre of mass was moved) each touch as a shape of their own: one manifold
// for each pair of parts that touches, kept in the pair's Memory under the
// two parts' numbers, so the same two parts find their impulses again.
//
// TOUCHING. After the solve, each pair is touching or not, and a change is
// an event (events.js): see touched(). A sensor's pairs get no manifold here,
// since a sensor pushes nothing; World asks about them after the step, where
// the bodies end up.

import { collide, Shape, Manifold, Parts, Placed, placePart, SPHERE, CAPSULE, BOX as BOX_KIND, HULL } from './collide.js';
import { touchLevel, Touches } from './triangle.js';
import { rotate, atan2 } from './core/math.js';
import { resized } from './core/slots.js';
import { impact, posed, UNSURE } from './impact.js';

const DYNAMIC = 0, STATIC = 1;
const BOX = 0, SPHERE_SHAPE = 1, CAPSULE_SHAPE = 2, HULL_SHAPE = 3, MESH = 5;   // World's shape codes; a heightfield is 6

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

// J: the numbers a point keeps for its normal, 12: a's lever crossed with the
// normal, b's, then each of those turned by its body's inverse inertia. How
// fast the pair closes along it is then three dot products, and a push along
// it three multiply-adds per body, where working them out from the levers and
// the inverse inertias at every push cost 15 multiplies a body. Levers and
// inverse inertias hold for the whole step, so these do too. Measured in the
// solver, which a box face sweeps four points four times a pass: see
// ROADMAP.md, milestone 16. A manifold keeps the same 12 for each of its two
// tangents at its friction anchor (FJ; see FRICTION).
const J = 12, FJ = 24;

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
    grow('sub', Int32Array, m);    // a mesh or parts pair's: which of its Memory's manifolds this one is kept as; else -1
    grow('key', Int32Array, m);    // which parts made it, as its Memory knows it again
    grow('partA', Int32Array, m); grow('partB', Int32Array, m);   // the part of a and of b it is on, or -1
    grow('points', Uint8Array, m);
    grow('normal', Float64Array, 3 * m); grow('t1', Float64Array, 3 * m); grow('t2', Float64Array, 3 * m);
    grow('friction', Float64Array, m); grow('restitution', Float64Array, m); grow('tolerance', Float64Array, m);
    // Rolling and spinning (contacts of one or two points; see PATCHES above):
    grow('patch', Float64Array, m);                       // the patch's radius, m
    grow('rollMass1', Float64Array, m); grow('rollMass2', Float64Array, m); grow('spinMass', Float64Array, m);
    grow('rollImpulse1', Float64Array, m); grow('rollImpulse2', Float64Array, m); grow('spinImpulse', Float64Array, m);
    // Friction, one anchor a manifold (see FRICTION): the anchor's offset from
    // each body's centre, its turning terms along t1 and t2 (FJ), the inverse of
    // the 2x2 block's effective mass as xx xy yy, the impulse along each
    // tangent, the slip carried in along each (see store), and whether friction
    // was at its limit in the last pass.
    grow('anchorA', Float64Array, 3 * m); grow('anchorB', Float64Array, 3 * m);
    grow('fjac', Float64Array, FJ * m);
    grow('tangentMass', Float64Array, 3 * m);
    grow('tangentImpulse1', Float64Array, m); grow('tangentImpulse2', Float64Array, m);
    grow('drift1', Float64Array, m); grow('drift2', Float64Array, m);
    grow('sliding', Uint8Array, m);
    grow('pid', Uint32Array, p);
    grow('separation', Float64Array, p);
    grow('point', Float64Array, 3 * p);
    grow('ra', Float64Array, 3 * p); grow('rb', Float64Array, 3 * p);
    // From a contact's point to b's end of it. A contact found where the step
    // starts has its point midway between the two surfaces, a hair apart, and
    // each body reaches it, so this is 0; one found at the moment of impact
    // (impact.js) joins two points that start the step apart, maybe metres,
    // so its point is a's end and this the way to b's, and each body's lever
    // reaches its own end.
    grow('shift', Float64Array, 3 * p);
    grow('la', Float64Array, 3 * p); grow('lb', Float64Array, 3 * p);
    grow('adjusted', Float64Array, p);
    grow('gap', Float64Array, m);   // its points' least separation as the last pass saw it: at the step's end
    grow('normalMass', Float64Array, p);
    grow('jac', Float64Array, J * p);   // each point's turning terms, worked out once a step (see J)
    grow('normalImpulse', Float64Array, p);
    grow('maxNormalImpulse', Float64Array, p); grow('relativeVelocity', Float64Array, p);
    grow('lever', Float64Array, p);   // the point's distance from the friction anchor, across the normal: its twist lever
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
  const inner = world._inner;
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
    const tolerance = Math.min(world._size[a], world._size[b]) * PIXEL;
    const margin = reach[a] + reach[b] + tolerance;
    if (inner[a] instanceof Parts || inner[b] instanceof Parts) {
      parted(world, pairs, p, a, b, next, tolerance, margin);
      p++;
      continue;
    }
    const hull = shape[a] === HULL_SHAPE || shape[b] === HULL_SHAPE;
    if (!hull || !reused(world, pairs, p, a, b, tolerance)) {
      load(world, a, shapeA);
      load(world, b, shapeB);
      if (!meet(world, a, b, null, null, tolerance, margin)) {
        pairs.forget(p);
        p++;
        continue;
      }
      if (hull) remember(world, pairs, p, a, b);
    }
    // Static bodies are never awake, and _wake leaves them be.
    if (awake[a] !== awake[b]) world._wake(awake[a] ? b : a);
    const m = emit(world, next, p, a, b, manifold, 1, tolerance, -1, -1, -1, 0);
    carry(pairs, p, next, m);
    p++;
  }
  return next.count;
}

/**
 * REUSE. A hull pair (one body a hull, the other a box, a ball, a capsule
 * or a hull; neither a mesh, a heightfield or one with parts) whose two
 * bodies have barely moved against each other since the narrowphase last
 * looked at it gets that manifold again, rebuilt from where
 * its points sit on each body, instead of being found afresh: Jolt's body pair
 * cache, and parry's try_update_contacts. Most of a settling pile's pairs, and
 * every resting one's, are such pairs. Each point keeps its end on a, in a's
 * frame, and its end on b, in b's frame; rebuilt, the two ends are placed
 * where their bodies are now, the point is midway between them and its
 * separation is how far apart they are along the normal, exactly as collide()
 * reports one, and the normal turns with a. A ball's or a capsule's end is
 * kept as its core (its centre, or the nearest point of its axis) and put
 * back on its surface along the normal, for core()'s reason: a ball's surface
 * point turns away as it rolls, and a contact point carried off to the side
 * would push off-centre, a torque the true contact never has. A manifold
 * found at the moment of impact isn't kept, and one whose bodies may have
 * been reshaped, refiltered or joined since (World._pairEpoch) is dropped
 * (pairs.js holds). A body moved by hand needs nothing of its own: the pose
 * below is read from where the bodies are now, so a teleport is a drift like
 * any other, and one kinematic body set by hand every step costs only its
 * own pairs their kept manifolds, not every pair in the world.
 *
 * Only hull pairs keep one. Measured with every plain pair kept: 1,000
 * hulls, whose collide() is separating axes over faces and edges, ran
 * 27% faster, while boxes, a pyramid and terrain, whose collide() is about
 * as cheap as the rebuild, ran no faster and stored about 300 bytes more a pair.
 * Rebuilt or found, a manifold reaches emit() and carry() the same way, so
 * the solver's anchor, twist, slip and warm start see no difference.
 *
 * How far it may have moved, and why. The drift is the most any point of the
 * smaller body (in reach) has moved against the larger since the manifold was
 * found: its centre's move in the larger's frame, plus the chord the turn
 * between them sweeps at its reach, 2 sin(θ/2) r, read exactly off the turn's
 * quaternion. Two rules, both from the tolerance, the pair's PIXEL that the
 * narrowphase itself already decides within (collide.js, TOLERANCE):
 *
 *   2 drift ≤ tolerance. A kept point's two ends each moved with their bodies,
 *   so its separation is off from the true one by at most the drift of each
 *   end, twice the drift: within what the narrowphase lets a face win over a
 *   nearly-as-good edge. Its place, the lever the solver pushes at, is off by
 *   at most the drift, half a pixel.
 *
 *   drift + moving ≤ tolerance. A point the kept manifold lacks was further
 *   than the margin when it was found, and the margin is at least the
 *   tolerance, so it can come no nearer than tolerance - drift by the time the
 *   step starts, and the step closes the pair by no more than moving() (the
 *   same bound that asks for a time of impact): such a point still can't
 *   touch by the step's end, so leaving it out loses nothing. A pair moving
 *   fast enough to need a time of impact (moving > tolerance) never gets here.
 *
 *   loss + 2 drift ≤ tolerance, which holds the first rule in it. collide()
 *   lets a hull's manifold fall short of the exact answer by up to the
 *   tolerance (Manifold.loss): a face kept on a tie separates the hulls that
 *   much less than the best axis does, and its deepest point is that much
 *   shallower than the deepest corner under the face. Drift moves every
 *   axis's and every corner's separation by up to 2 drift (each body's end of
 *   it, as for a point), so a kept manifold falls short by loss + 2 drift: at
 *   most the tolerance, the most a fresh collide() would. Without the loss, a
 *   kept normal fell behind the best axis by up to 1.25 tolerances on rock
 *   heaps (measured against brute-force separating axes over every face and
 *   edge pair), and on heaps of rocks and boxes a kept manifold's deepest
 *   point was up to 1.09 tolerances shallower than a fresh one's, missing a
 *   corner a fresh collide() adds; with it, 0.80 and 0.99.
 *
 * The rules are measured against the pose the manifold was found at, not the
 * last step's, so a slow creep can't add up past them unseen.
 */
function reused(world, pairs, p, a, b, tolerance) {
  if (pairs.cache[p] === 0) return false;
  const r = pairs.cache[p] - 1, count = pairs.cachePoints[r];
  const g = relative(world, a, b), pose = pairs.cachePose, p7 = 7 * r;
  const ex = POSE[0] - pose[p7], ey = POSE[1] - pose[p7 + 1], ez = POSE[2] - pose[p7 + 2];
  // The turn since, conj(then) ⊗ now: its vector part is sin(θ/2) along the axis, either way round.
  const cx = -pose[p7 + 3], cy = -pose[p7 + 4], cz = -pose[p7 + 5], cw = pose[p7 + 6];
  const rx = POSE[3], ry = POSE[4], rz = POSE[5], rw = POSE[6];
  const ux = cw * rx + cx * rw + cy * rz - cz * ry;
  const uy = cw * ry + cy * rw + cz * rx - cx * rz;
  const uz = cw * rz + cz * rw + cx * ry - cy * rx;
  const drift = Math.sqrt(ex * ex + ey * ey + ez * ez) + 2 * Math.sqrt(ux * ux + uy * uy + uz * uz) * world._reach[g];
  if (pairs.cacheLoss[r] + 2 * drift > tolerance || drift + moving(world, a, b) > tolerance) return false;
  const pos = world._position, A = world._axes, a3 = 3 * a, b3 = 3 * b, a9 = 9 * a, b9 = 9 * b;
  const local = pairs.cacheLocal, ln = pairs.cacheNormal, p3 = 3 * r;
  const n = manifold.normal, lx = ln[p3], ly = ln[p3 + 1], lz = ln[p3 + 2];
  const nx = A[a9] * lx + A[a9 + 3] * ly + A[a9 + 6] * lz;
  const ny = A[a9 + 1] * lx + A[a9 + 4] * ly + A[a9 + 7] * lz;
  const nz = A[a9 + 2] * lx + A[a9 + 5] * ly + A[a9 + 8] * lz;
  n[0] = nx; n[1] = ny; n[2] = nz;
  // A round end's core, put back on its surface: a's toward b, b's toward a.
  const ra = rounded(world, a), rb = -rounded(world, b);
  manifold.count = count;
  for (let k = 0; k < count; k++) {
    const o = 6 * (P * r + k), k3 = 3 * k;
    const ax = local[o], ay = local[o + 1], az = local[o + 2], bx = local[o + 3], by = local[o + 4], bz = local[o + 5];
    const pax = pos[a3] + A[a9] * ax + A[a9 + 3] * ay + A[a9 + 6] * az + nx * ra;
    const pay = pos[a3 + 1] + A[a9 + 1] * ax + A[a9 + 4] * ay + A[a9 + 7] * az + ny * ra;
    const paz = pos[a3 + 2] + A[a9 + 2] * ax + A[a9 + 5] * ay + A[a9 + 8] * az + nz * ra;
    const pbx = pos[b3] + A[b9] * bx + A[b9 + 3] * by + A[b9 + 6] * bz + nx * rb;
    const pby = pos[b3 + 1] + A[b9 + 1] * bx + A[b9 + 4] * by + A[b9 + 7] * bz + ny * rb;
    const pbz = pos[b3 + 2] + A[b9 + 2] * bx + A[b9 + 5] * by + A[b9 + 8] * bz + nz * rb;
    const s = (pbx - pax) * nx + (pby - pay) * ny + (pbz - paz) * nz, half = 0.5 * s;
    manifold.separations[k] = s;
    // Midway, on the line through a round end's core along the normal, where
    // collide() puts a ball's or a capsule's points: the other end, kept on
    // its own body, falls behind as a ball rolls on, and a point midway to it
    // would push off-centre too.
    if (rb !== 0) {
      manifold.points[k3] = pbx - nx * half; manifold.points[k3 + 1] = pby - ny * half; manifold.points[k3 + 2] = pbz - nz * half;
    } else if (ra !== 0) {
      manifold.points[k3] = pax + nx * half; manifold.points[k3 + 1] = pay + ny * half; manifold.points[k3 + 2] = paz + nz * half;
    } else {
      manifold.points[k3] = (pax + pbx) * 0.5; manifold.points[k3 + 1] = (pay + pby) * 0.5; manifold.points[k3 + 2] = (paz + pbz) * 0.5;
    }
    manifold.ids[k] = pairs.cacheId[P * r + k];
  }
  shifted = false;
  return true;
}

/** Keeps the manifold just found for pair p, for reused(); not one found at the moment of impact. */
function remember(world, pairs, p, a, b) {
  if (shifted) { if (pairs.cache[p] !== 0) pairs.uncache(p); return; }
  const count = manifold.count, r = pairs.cacheOf(p);
  pairs.cachePoints[r] = count;
  relative(world, a, b);
  const pose = pairs.cachePose, p7 = 7 * r;
  for (let k = 0; k < 7; k++) pose[p7 + k] = POSE[k];
  pairs.cacheLoss[r] = manifold.loss;
  // Into a body's frame: the transpose of its axes, each axis dotted in turn.
  const pos = world._position, A = world._axes, a3 = 3 * a, b3 = 3 * b, a9 = 9 * a, b9 = 9 * b;
  const n = manifold.normal, nx = n[0], ny = n[1], nz = n[2], ln = pairs.cacheNormal, p3 = 3 * r;
  ln[p3] = A[a9] * nx + A[a9 + 1] * ny + A[a9 + 2] * nz;
  ln[p3 + 1] = A[a9 + 3] * nx + A[a9 + 4] * ny + A[a9 + 5] * nz;
  ln[p3 + 2] = A[a9 + 6] * nx + A[a9 + 7] * ny + A[a9 + 8] * nz;
  const local = pairs.cacheLocal;
  for (let k = 0; k < count; k++) {
    const o = 6 * (P * r + k), k3 = 3 * k, half = 0.5 * manifold.separations[k];
    const px = manifold.points[k3], py = manifold.points[k3 + 1], pz = manifold.points[k3 + 2];
    const ax = px - nx * half - pos[a3], ay = py - ny * half - pos[a3 + 1], az = pz - nz * half - pos[a3 + 2];
    const bx = px + nx * half - pos[b3], by = py + ny * half - pos[b3 + 1], bz = pz + nz * half - pos[b3 + 2];
    local[o] = A[a9] * ax + A[a9 + 1] * ay + A[a9 + 2] * az;
    local[o + 1] = A[a9 + 3] * ax + A[a9 + 4] * ay + A[a9 + 5] * az;
    local[o + 2] = A[a9 + 6] * ax + A[a9 + 7] * ay + A[a9 + 8] * az;
    local[o + 3] = A[b9] * bx + A[b9 + 1] * by + A[b9 + 2] * bz;
    local[o + 4] = A[b9 + 3] * bx + A[b9 + 4] * by + A[b9 + 5] * bz;
    local[o + 5] = A[b9 + 6] * bx + A[b9 + 7] * by + A[b9 + 8] * bz;
    cored(world, a, local, o);
    cored(world, b, local, o + 3);
    pairs.cacheId[P * r + k] = manifold.ids[k];
  }
}

/** A ball's or a capsule's radius, the depth of its surface over its core; 0 for any other shape. */
function rounded(world, s) {
  const shape = world._shape[s];
  return shape === SPHERE_SHAPE || shape === CAPSULE_SHAPE ? world._dims[3 * s] : 0;
}

/** A kept end at local[o..o+2], in slot s's frame, moved onto its core if it is round (see core()). */
function cored(world, s, local, o) {
  const shape = world._shape[s];
  if (shape === SPHERE_SHAPE) { local[o] = 0; local[o + 1] = 0; local[o + 2] = 0; return; }
  if (shape !== CAPSULE_SHAPE) return;
  const half = (world._dims[3 * s + 1] - 2 * world._dims[3 * s]) * 0.5, y = local[o + 1];
  local[o] = 0; local[o + 1] = y < -half ? -half : y > half ? half : y; local[o + 2] = 0;
}

/**
 * The pose of a and b against each other, into POSE: the smaller body's
 * centre in the larger's frame, then the larger's rotation undone from the
 * smaller's, conj(qh) ⊗ qg. Which is which depends only on their reach, which
 * only setBody changes, and that moves World._pairEpoch on, dropping the kept
 * manifold. Measured in the larger one's frame, a point's drift is bounded
 * by the smaller one's reach: a box on a 400 m floor is bounded by the box.
 * Returns the smaller one.
 */
function relative(world, a, b) {
  const h = world._reach[a] >= world._reach[b] ? a : b, g = h === a ? b : a;
  const pos = world._position, A = world._axes, q = world._rotation, h3 = 3 * h, g3 = 3 * g, h9 = 9 * h, h4 = 4 * h, g4 = 4 * g;
  const dx = pos[g3] - pos[h3], dy = pos[g3 + 1] - pos[h3 + 1], dz = pos[g3 + 2] - pos[h3 + 2];
  POSE[0] = A[h9] * dx + A[h9 + 1] * dy + A[h9 + 2] * dz;
  POSE[1] = A[h9 + 3] * dx + A[h9 + 4] * dy + A[h9 + 5] * dz;
  POSE[2] = A[h9 + 6] * dx + A[h9 + 7] * dy + A[h9 + 8] * dz;
  const ax = -q[h4], ay = -q[h4 + 1], az = -q[h4 + 2], aw = q[h4 + 3];
  const bx = q[g4], by = q[g4 + 1], bz = q[g4 + 2], bw = q[g4 + 3];
  POSE[3] = aw * bx + ax * bw + ay * bz - az * by;
  POSE[4] = aw * by + ay * bw + az * bx - ax * bz;
  POSE[5] = aw * bz + az * bw + ax * by - ay * bx;
  POSE[6] = aw * bw - ax * bx - ay * by - az * bz;
  return g;
}
const POSE = new Float64Array(7);

/**
 * Whether shapeA and shapeB (slots a and b, or parts partA and partB of
 * them; null for a whole body) touch within the step, into `manifold`.
 *
 * Not touching yet, and moving against each other by more than a pixel: do
 * they meet this step, and where? (impact.js.) A contact found where the step
 * starts is wrong by no more than how far the pair moves against itself in
 * the step, so within a pixel there is nothing to ask. If they meet, the
 * manifold is found where they meet, with every point within reach of the
 * rest of the step, so a rod striking a wall with one end has its other
 * end's contact too, before it swings in: points of the faces that meet, so
 * no plane off a corner (see impact.js). Leaves the shapes wherever impact()
 * left them.
 */
function meet(world, a, b, partA, partB, tolerance, margin) {
  shifted = false;
  if (collide(shapeA, shapeB, margin, tolerance, manifold) === 0) return false;
  const moves = least(manifold) > tolerance ? moving(world, a, b) : 0;
  if (moves > tolerance) {
    const dt = world._stepDt, t = impact(world, a, b, shapeA, shapeB, dt, tolerance, partA, partB);
    world._impacts++;
    const ahead = t >= 0 ? moves * (1 - t / dt) + 2 * tolerance : 0;
    if (t === -1 || (t !== UNSURE && collide(shapeA, shapeB, ahead, tolerance, manifold) === 0)) return false;
    if (t !== UNSURE) back(world, a, b, t, manifold, partA, partB);
  }
  return true;
}

/**
 * Pair p, one or both of whose bodies have parts: each part of one against
 * each part of the other whose balls (its reach round its centre) come
 * within the margin, as a pair of its own with its own time of impact; a
 * manifold for each that touches, started from what the same two parts
 * remembered. ponytail: every part against every part, n × m, culled by
 * those balls; a tree of each body's parts when compounds have dozens.
 */
function parted(world, pairs, p, a, b, next, tolerance, margin) {
  const inner = world._inner, pos = world._position, rot = world._rotation, reach = world._reach, awake = world._awake;
  const la = inner[a] instanceof Parts ? inner[a].list : null, lb = inner[b] instanceof Parts ? inner[b].list : null;
  const na = la === null ? 1 : la.length, nb = lb === null ? 1 : lb.length;
  const kept = pairs.memoryOf(p);
  kept.taken.fill(0, 0, kept.count);
  let sub = 0;
  for (let i = 0; i < na; i++) {
    const partA = la === null ? null : la[i], ra = la === null ? reach[a] : partA.reach;
    for (let j = 0; j < nb; j++) {
      const partB = lb === null ? null : lb[j], rb = lb === null ? reach[b] : partB.reach;
      // Placed afresh for each pair: a time of impact leaves them moved.
      if (partA === null) load(world, a, shapeA); else placePart(partA, pos, 3 * a, rot, 4 * a, shapeA);
      if (partB === null) load(world, b, shapeB); else placePart(partB, pos, 3 * b, rot, 4 * b, shapeB);
      const dx = shapeB.center[0] - shapeA.center[0], dy = shapeB.center[1] - shapeA.center[1], dz = shapeB.center[2] - shapeA.center[2];
      const r = ra + rb + margin;
      if (dx * dx + dy * dy + dz * dz > r * r) continue;
      if (!meet(world, a, b, partA, partB, tolerance, margin)) continue;
      if (awake[a] !== awake[b]) world._wake(awake[a] ? b : a);
      const key = i * nb + j, m = emit(world, next, p, a, b, manifold, 1, tolerance, sub++, la === null ? -1 : i, lb === null ? -1 : j, key);
      for (let k = 0; k < kept.count; k++) {
        if (kept.taken[k] || kept.key[k] !== key) continue;
        kept.taken[k] = 1;
        carry(kept, k, next, m);
        break;
      }
    }
  }
  if (sub === 0) pairs.forget(p);
}

/** The least separation of a manifold's points. */
function least(mf) {
  let s = Infinity;
  for (let k = 0; k < mf.count; k++) if (mf.separations[k] < s) s = mf.separations[k];
  return s;
}

/**
 * How far slots a and b can move against each other in the step, at most:
 * their relative speed, what gravity adds, and the fastest any point of
 * either can move by turning, over the step.
 */
function moving(world, a, b) {
  const v = world._velocity, w = world._angular, g = world._gravity, dt = world._stepDt, a3 = 3 * a, b3 = 3 * b;
  const ga = world._type[a] === DYNAMIC ? world._gravityScale[a] : 0, gb = world._type[b] === DYNAMIC ? world._gravityScale[b] : 0;
  const dx = v[b3] - v[a3], dy = v[b3 + 1] - v[a3 + 1], dz = v[b3 + 2] - v[a3 + 2];
  const gs = gb - ga, gl = Math.sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]) * Math.abs(gs);
  const turning = Math.sqrt(w[a3] * w[a3] + w[a3 + 1] * w[a3 + 1] + w[a3 + 2] * w[a3 + 2]) * world._reach[a]
    + Math.sqrt(w[b3] * w[b3] + w[b3 + 1] * w[b3 + 1] + w[b3 + 2] * w[b3 + 2]) * world._reach[b];
  return (Math.sqrt(dx * dx + dy * dy + dz * dz) + 0.5 * gl * dt + turning) * dt;
}

/**
 * Manifold mf, found where the bodies are at time t of the step, back to the
 * step's start: each point's two ends, a's and b's, carried back with their
 * bodies; the point becomes a's end, SHIFT the way from it to b's, and the
 * separation how far apart they are along the normal. So the solver, closing
 * the pair as it moves, lets it close to exactly the impact.
 */
function back(world, a, b, t, mf, partA, partB) {
  posed(0, t, CA, QA);
  posed(3, t, CB, QB);
  const p = world._position, q = world._rotation, a3 = 3 * a, b3 = 3 * b;
  const nx = mf.normal[0], ny = mf.normal[1], nz = mf.normal[2];
  for (let k = 0; k < mf.count; k++) {
    const k3 = 3 * k, half = 0.5 * mf.separations[k];
    const px = mf.points[k3], py = mf.points[k3 + 1], pz = mf.points[k3 + 2];
    carried(world, a, partA, QA, px - nx * half - CA[0], py - ny * half - CA[1], pz - nz * half - CA[2]);
    const ax = p[a3] + v3[0], ay = p[a3 + 1] + v3[1], az = p[a3 + 2] + v3[2];
    carried(world, b, partB, QB, px + nx * half - CB[0], py + ny * half - CB[1], pz + nz * half - CB[2]);
    const bx = p[b3] + v3[0] - ax, by = p[b3 + 1] + v3[1] - ay, bz = p[b3 + 2] + v3[2] - az;
    mf.separations[k] = bx * nx + by * ny + bz * nz;
    mf.points[k3] = ax; mf.points[k3 + 1] = ay; mf.points[k3 + 2] = az;
    SHIFT[k3] = bx; SHIFT[k3 + 1] = by; SHIFT[k3 + 2] = bz;
  }
  shifted = true;
}

/**
 * A point of slot s, offset (x, y, z) from its centre where it is at time t
 * (turned by qt), carried back to where s starts the step, as an offset into
 * v3: turned back as the solver's anchor turns (core()). A box's point turns
 * with it. A ball's anchor is its centre, so its point doesn't turn at all,
 * and a capsule's is on its axis, so only that part turns. Turned back whole,
 * the point of a ball spinning fast was carried round its side, its gap read
 * 4 cm wider than the solver held, and two balls met with nothing between.
 */
function carried(world, s, part, qt, x, y, z) {
  if (part !== null) {
    carriedPart(world, s, part, qt, x, y, z);
    return;
  }
  const shape = world._shape[s];
  if (shape === SPHERE_SHAPE) { v3[0] = x; v3[1] = y; v3[2] = z; return; }
  rotate(qt, 0, x, y, z, true, v3);
  if (shape !== CAPSULE_SHAPE) { rotate(world._rotation, 4 * s, v3[0], v3[1], v3[2], false, v3); return; }
  const half = (world._dims[3 * s + 1] - 2 * world._dims[3 * s]) * 0.5;
  const c = v3[1] < -half ? -half : v3[1] > half ? half : v3[1];
  // The radial part, from the axis to the point, as it is in the world at t.
  rotate(qt, 0, 0, c, 0, false, v3);
  const rx = x - v3[0], ry = y - v3[1], rz = z - v3[2];
  rotate(world._rotation, 4 * s, 0, c, 0, false, v3);
  v3[0] += rx; v3[1] += ry; v3[2] += rz;
}

/**
 * carried() for a part: the same rule about the part's own core. A ball
 * part's point is carried with its centre, which orbits the body's, and
 * doesn't turn about it; a capsule part's turns only along its axis.
 */
function carriedPart(world, s, part, qt, x, y, z) {
  if (part.kind === SPHERE) {
    rotate(qt, 0, part.position[0], part.position[1], part.position[2], false, C3);
    const rx = x - C3[0], ry = y - C3[1], rz = z - C3[2];
    rotate(world._rotation, 4 * s, part.position[0], part.position[1], part.position[2], false, v3);
    v3[0] += rx; v3[1] += ry; v3[2] += rz;
    return;
  }
  rotate(qt, 0, x, y, z, true, C3);
  if (part.kind !== CAPSULE) {
    rotate(world._rotation, 4 * s, C3[0], C3[1], C3[2], false, v3);
    return;
  }
  axisPoint(part, C3[0], C3[1], C3[2], C3);
  rotate(qt, 0, C3[0], C3[1], C3[2], false, v3);
  const rx = x - v3[0], ry = y - v3[1], rz = z - v3[2];
  rotate(world._rotation, 4 * s, C3[0], C3[1], C3[2], false, v3);
  v3[0] += rx; v3[1] += ry; v3[2] += rz;
}

/** The point of capsule part's axis nearest (x, y, z), all in its body's frame, into out. */
function axisPoint(part, x, y, z, out) {
  const o = part.position, h = part.half[1];
  rotate(part.rotation, 0, 0, 1, 0, false, AXIS);
  let t = (x - o[0]) * AXIS[0] + (y - o[1]) * AXIS[1] + (z - o[2]) * AXIS[2];
  t = t < -h ? -h : t > h ? h : t;
  out[0] = o[0] + AXIS[0] * t; out[1] = o[1] + AXIS[1] * t; out[2] = o[2] + AXIS[2] * t;
}
const C3 = new Float64Array(3), AXIS = new Float64Array(3);

const CA = new Float64Array(3), CB = new Float64Array(3), QA = new Float64Array(4), QB = new Float64Array(4);
/** back()'s shifts, for emit() to keep; `shifted` says the manifold has them. */
const SHIFT = new Float64Array(12);
let shifted = false;

/**
 * Pair p, one of whose bodies is a mesh or a heightfield (never both: neither
 * is ever dynamic): a manifold for each direction the other body touches it
 * along, each started from the remembered manifold nearest its direction.
 */
function level(world, pairs, p, a, b, next) {
  const turned = world._shape[a] >= MESH, g = turned ? a : b, o = turned ? b : a;
  const parts = world._inner[o] instanceof Parts ? world._inner[o].list : null;
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
  const awake = world._awake, kept = pairs.memoryOf(p), rot = world._rotation;
  kept.taken.fill(0, 0, kept.count);
  // Never carried back (impact() isn't asked of a mesh), so no shifts: left
  // as the last pair meet() saw had them, emit() gave this pair's points that
  // pair's, and a world began with the last world's.
  shifted = false;
  let sub = 0;
  for (let i = 0; i < (parts === null ? 1 : parts.length); i++) {
    if (parts === null) load(world, o, shapeA);
    else placePart(parts[i], pos, 3 * o, rot, 4 * o, shapeA);
    const n = touchLevel(world._geometry[g], pos, A, g, shapeA, lx - ex, ly - ey, lz - ez, lx + ex, ly + ey, lz + ez,
      margin, tolerance, touches);
    if (n > 0 && awake[a] !== awake[b]) world._wake(awake[a] ? b : a);
    const part = parts === null ? -1 : i;
    for (let k = 0; k < n; k++) {
      // Normals come out from the other body; a pair's point from a to b.
      const m = emit(world, next, p, a, b, touches.list[k], turned ? -1 : 1, tolerance, sub++,
        turned ? -1 : part, turned ? part : -1, i);
      let best = -1, most = 0;
      for (let j = 0; j < kept.count; j++) {
        if (kept.taken[j] || kept.key[j] !== i) continue;
        const d = kept.normal[3 * j] * next.normal[3 * m] + kept.normal[3 * j + 1] * next.normal[3 * m + 1]
          + kept.normal[3 * j + 2] * next.normal[3 * m + 2];
        if (d > most) { most = d; best = j; }
      }
      if (best < 0) continue;
      kept.taken[best] = 1;
      carry(kept, best, next, m);
    }
  }
  if (sub === 0) pairs.forget(p);
}

/**
 * Manifold mf of pair p, between slots a and b, as the next manifold in
 * next, its normal times sign; returns its index. sub is which of a mesh or
 * parts pair's manifolds it is, or -1; partA and partB the parts it is on,
 * or -1; key what its Memory knows it by.
 */
function emit(world, next, p, a, b, mf, sign, tolerance, sub, partA, partB, key) {
  const m = next.count++;
  next.ensure(next.count);
  next.a[m] = a; next.b[m] = b; next.pair[m] = p; next.sub[m] = sub;
  next.partA[m] = partA; next.partB[m] = partB; next.key[m] = key;
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
  next.tangentImpulse1[m] = 0; next.tangentImpulse2[m] = 0; next.drift1[m] = 0; next.drift2[m] = 0;
  for (let k = 0; k < mf.count; k++) {
    const i = m * P + k;
    next.pid[i] = mf.ids[k];
    next.separation[i] = mf.separations[k];
    for (let c = 0; c < 3; c++) next.point[3 * i + c] = mf.points[3 * k + c];
    for (let c = 0; c < 3; c++) next.shift[3 * i + c] = shifted ? SHIFT[3 * k + c] : 0;
    next.normalImpulse[i] = 0;
  }
  basis(next, m);
  return m;
}

/**
 * What "from" remembers at p (the pair list for pair p, or a mesh pair's
 * Memory for its manifold p): normal impulses onto this step's points with
 * the same feature id, friction and twist onto the manifold. Friction is
 * remembered as a vector in world space and laid onto this step's tangents,
 * since the tangents turn with the normal from step to step. Slip is carried
 * only if some point is the same one: a box tipped onto another face has
 * slipped nowhere on it yet.
 */
function carry(from, p, next, m) {
  const remembered = from.points[p];
  if (remembered === 0) return;
  // The manifold's axes are read once: the stores below go to other arrays,
  // which the compiler can't be sure don't alias them. The id search stays a
  // plain one from the first point: ids can repeat within a manifold (a mesh
  // manifold's ids are mixed from triangles' features), and the first match
  // is the one taken.
  const m3 = 3 * m;
  const t1x = next.t1[m3], t1y = next.t1[m3 + 1], t1z = next.t1[m3 + 2];
  const t2x = next.t2[m3], t2y = next.t2[m3 + 1], t2z = next.t2[m3 + 2];
  const count = next.points[m], p3 = 3 * p;
  const lx = from.twist[p3], ly = from.twist[p3 + 1], lz = from.twist[p3 + 2];
  if (count <= 2) {
    next.rollImpulse1[m] = lx * t1x + ly * t1y + lz * t1z;
    next.rollImpulse2[m] = lx * t2x + ly * t2y + lz * t2z;
  }
  next.spinImpulse[m] = lx * next.normal[m3] + ly * next.normal[m3 + 1] + lz * next.normal[m3 + 2];
  const fx = from.friction[p3], fy = from.friction[p3 + 1], fz = from.friction[p3 + 2];
  next.tangentImpulse1[m] = fx * t1x + fy * t1y + fz * t1z;
  next.tangentImpulse2[m] = fx * t2x + fy * t2y + fz * t2z;
  let same = false;
  for (let k = 0; k < count; k++) {
    const i = m * P + k, id = next.pid[i];
    for (let j = 0; j < remembered; j++) {
      const o = p * P + j;
      if (from.pid[o] !== id) continue;
      next.normalImpulse[i] = from.normalImpulse[o];
      same = true;
      break;
    }
  }
  if (same) {
    const sx = from.slip[p3], sy = from.slip[p3 + 1], sz = from.slip[p3 + 2];
    next.drift1[m] = sx * t1x + sy * t1y + sz * t1z;
    next.drift2[m] = sx * t2x + sy * t2y + sz * t2z;
  }
}

/**
 * After the solve: each pair remembers its points' ids and impulses and its
 * manifold's friction, twist and slip, for the next step; a mesh pair, each
 * of its manifolds, with its normal, in its Memory.
 *
 * Slip is how far the two surfaces have slid against each other at the
 * friction anchor while friction held it, and friction's solving pass pulls
 * it back, as the normal's pulls back an overlap. Friction alone works on
 * speeds: it stops a slide, but can't undo one, and a solver that visits
 * each contact once a pass leaves a little error, which in a heap leans
 * the same way step after step. Measured on the benchmark's 1,000 boxes:
 * settled, their contacts held
 * at a third to a half of friction's limit, yet slid 0.5 to 1.5 mm/s, and a
 * box crept 4 cm in 10 s, so the heap never slept. PhysX's friction anchors
 * do this job; this measures the slip itself rather than fixing a spot on
 * each body, so a rolling ball, whose surfaces don't slide, has none.
 *
 * Measured on the benchmark's pile, five starts, sleeping off from 30 s to
 * 40 s: the fastest box crept 1 to 1.5 mm/s with this, up to 8.9 without.
 * Gentler versions were tried and measured worse on that pile: pulling back at
 * half the rate, and letting go of a whole contact when any point slid (a
 * rocking box's points take turns sliding). See also the sleep rule in
 * world.js, which this works with.
 *
 * It is let go where friction was at its limit in the last pass, since the
 * manifold is truly sliding and pulling it back would be friction it doesn't
 * have, or where nothing presses, since there's no friction there to hold.
 * A manifold with a point found at the moment of impact (shift) starts afresh.
 */
export function store(world, c) {
  const pairs = world._pairs;
  for (let m = 0; m < c.count; m++) {
    slipped(world, c, m);
    const p = c.pair[m], sub = c.sub[m];
    if (sub < 0) {
      keep(pairs, p, c, m);
      continue;
    }
    // A mesh or parts pair's manifolds come in order, so the last one sets the count.
    const kept = pairs.memoryOf(p);
    kept.ensure(sub + 1);
    kept.count = sub + 1;
    for (let k = 0; k < 3; k++) kept.normal[3 * sub + k] = c.normal[3 * m + k];
    kept.key[sub] = c.key[m];
    keep(kept, sub, c, m);
  }
}

/** Manifold m's ids and impulses into "to" (the pair list, or a Memory) at p. */
function keep(to, p, c, m) {
  const count = c.points[m], m3 = 3 * m, p3 = 3 * p;
  to.points[p] = count;
  // The manifold's axes are read once: the stores below go to other arrays,
  // which the compiler can't be sure don't alias them.
  const t1x = c.t1[m3], t1y = c.t1[m3 + 1], t1z = c.t1[m3 + 2];
  const t2x = c.t2[m3], t2y = c.t2[m3 + 1], t2z = c.t2[m3 + 2];
  const r1 = c.rollImpulse1[m], r2 = c.rollImpulse2[m], sp = c.spinImpulse[m];
  to.twist[p3] = t1x * r1 + t2x * r2 + c.normal[m3] * sp;
  to.twist[p3 + 1] = t1y * r1 + t2y * r2 + c.normal[m3 + 1] * sp;
  to.twist[p3 + 2] = t1z * r1 + t2z * r2 + c.normal[m3 + 2] * sp;
  const l1 = c.tangentImpulse1[m], l2 = c.tangentImpulse2[m];
  to.friction[p3] = t1x * l1 + t2x * l2;
  to.friction[p3 + 1] = t1y * l1 + t2y * l2;
  to.friction[p3 + 2] = t1z * l1 + t2z * l2;
  const d1 = c.drift1[m], d2 = c.drift2[m];
  to.slip[p3] = t1x * d1 + t2x * d2;
  to.slip[p3 + 1] = t1y * d1 + t2y * d2;
  to.slip[p3 + 2] = t1z * d1 + t2z * d2;
  for (let k = 0; k < count; k++) {
    const i = m * P + k, o = p * P + k;
    to.pid[o] = c.pid[i];
    to.normalImpulse[o] = c.normalImpulse[i];
  }
}

/**
 * Manifold m's slip after the step, into drift1 and drift2: what it carried
 * in, plus how far b's surface moved against a's at the friction anchor along
 * the tangents, from where each body started the step to where it ends it.
 */
function slipped(world, c, m) {
  const a = c.a[m], b = c.b[m], m3 = 3 * m, awake = world._awake;
  let pressed = 0, impact = false;
  for (let k = 0; k < c.points[m]; k++) {
    const i = m * P + k, i3 = 3 * i;
    pressed += c.normalImpulse[i];
    if (c.shift[i3] !== 0 || c.shift[i3 + 1] !== 0 || c.shift[i3 + 2] !== 0) impact = true;
  }
  if (c.sliding[m] !== 0 || pressed === 0 || impact) {
    c.drift1[m] = 0; c.drift2[m] = 0;
    return;
  }
  // Only an awake body moves in a step; a static one's start rotation was never kept.
  const T = turns;
  let dx = 0, dy = 0, dz = 0;
  if (awake[b] !== 0) {
    turned(world, b, T, 0);
    const rx = c.anchorB[m3], ry = c.anchorB[m3 + 1], rz = c.anchorB[m3 + 2];
    dx = T[0] + T[4] * rz - T[5] * ry;
    dy = T[1] + T[5] * rx - T[3] * rz;
    dz = T[2] + T[3] * ry - T[4] * rx;
  }
  if (awake[a] !== 0) {
    turned(world, a, T, 6);
    const rx = c.anchorA[m3], ry = c.anchorA[m3 + 1], rz = c.anchorA[m3 + 2];
    dx -= T[6] + T[10] * rz - T[11] * ry;
    dy -= T[7] + T[11] * rx - T[9] * rz;
    dz -= T[8] + T[9] * ry - T[10] * rx;
  }
  c.drift1[m] += dx * c.t1[m3] + dy * c.t1[m3 + 1] + dz * c.t1[m3 + 2];
  c.drift2[m] += dx * c.t2[m3] + dy * c.t2[m3 + 1] + dz * c.t2[m3 + 2];
}

// slipped()'s two bodies, each as (centre's move, turn vector): b at 0, a at 6.
const turns = new Float64Array(12);

/**
 * How slot s moved in the step, into T[o..o+5]: its centre's move, then its
 * turn as an angle along the axis, θ; a point at offset r from its centre
 * moved by the first plus θ × r. Not where that bit of surface ended up: a
 * 1 cm marble rolling at 1 m/s turns 1.7 radians in a step, and the bit that
 * touched at the start swings forward 0.7 of its radius over that turn, which
 * would read as slip where a rolling ball has none. Measured, it took 3% off
 * how far the marble rolled. θ × r is exactly the rolling ball's centre's
 * move, so it reads 0.
 */
function turned(world, s, T, o) {
  const p = world._position, st = world._start, q = world._rotation, q0 = world._startQ, s3 = s * 3, s4 = s * 4;
  // The step's turn, q q0⁻¹: its vector part is sin(θ/2) along the axis.
  const ax = q[s4], ay = q[s4 + 1], az = q[s4 + 2], aw = q[s4 + 3];
  const bx = -q0[s4], by = -q0[s4 + 1], bz = -q0[s4 + 2], bw = q0[s4 + 3];
  let ux = aw * bx + ax * bw + ay * bz - az * by;
  let uy = aw * by + ay * bw + az * bx - ax * bz;
  let uz = aw * bz + az * bw + ax * by - ay * bx;
  let uw = aw * bw - ax * bx - ay * by - az * bz;
  if (uw < 0) { ux = -ux; uy = -uy; uz = -uz; uw = -uw; }
  const sine = Math.sqrt(ux * ux + uy * uy + uz * uz);
  const k = sine === 0 ? 0 : 2 * atan2(sine, uw) / sine;
  T[o] = p[s3] - st[s3]; T[o + 1] = p[s3 + 1] - st[s3 + 1]; T[o + 2] = p[s3 + 2] - st[s3 + 2];
  T[o + 3] = ux * k; T[o + 4] = uy * k; T[o + 5] = uz * k;
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
function core(world, s, part, x, y, z, out, o) {
  if (part !== null) {
    corePart(world, s, part, x, y, z, out, o);
    return;
  }
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

/** core() for a part of slot s: its anchor on the part's own core, in the body's frame. */
function corePart(world, s, part, x, y, z, out, o) {
  const q = world._rotation, s4 = 4 * s;
  if (part.kind === SPHERE) {
    out[o] = part.position[0]; out[o + 1] = part.position[1]; out[o + 2] = part.position[2];
    rotate(q, s4, out[o], out[o + 1], out[o + 2], false, v3);
    return;
  }
  rotate(q, s4, x, y, z, true, v3);
  if (part.kind !== CAPSULE) {
    out[o] = v3[0]; out[o + 1] = v3[1]; out[o + 2] = v3[2];
    v3[0] = x; v3[1] = y; v3[2] = z;
    return;
  }
  axisPoint(part, v3[0], v3[1], v3[2], C3);
  out[o] = C3[0]; out[o + 1] = C3[1]; out[o + 2] = C3[2];
  rotate(q, s4, C3[0], C3[1], C3[2], false, v3);
}

/**
 * A body's shape, placed, into s. Not for a mesh or a heightfield, which are
 * triangles (triangle.js), nor a body with parts, which placePart() places.
 */
export function load(world, slot, s) {
  const shape = world._shape[slot], d = world._dims, s3 = slot * 3;
  if (shape === BOX) {
    s.kind = BOX_KIND;
    s.half[0] = d[s3] * 0.5; s.half[1] = d[s3 + 1] * 0.5; s.half[2] = d[s3 + 2] * 0.5;
  } else if (shape === SPHERE_SHAPE) {
    s.kind = SPHERE;
    s.half[0] = d[s3];
  } else if (shape === HULL_SHAPE) {
    s.kind = HULL;
    s.hull = world._inner[slot];
  } else {
    s.kind = CAPSULE;
    s.half[0] = d[s3];
    s.half[1] = (d[s3 + 1] - 2 * d[s3]) * 0.5;
  }
  // A hull keeps its placement for the step (Placed.fresh); made the first
  // time it is loaded, and kept until the body goes (World._remove).
  if (shape === HULL_SHAPE) {
    s.placed = world._placed[slot] || (world._placed[slot] = new Placed());
    s.step = world._stepCount;
    s.epoch = world._epoch;
  } else {
    s.placed = null;
  }
  s.center[0] = world._position[s3]; s.center[1] = world._position[s3 + 1]; s.center[2] = world._position[s3 + 2];
  // Its weight, for collide.js reducePoints (HELD UP): what gravity pulls a
  // dynamic body with, as _accelerate applies it; nothing for any other.
  const g = world._gravity, scale = world._type[slot] === DYNAMIC ? world._gravityScale[slot] : 0;
  s.weight[0] = g[0] * scale; s.weight[1] = g[1] * scale; s.weight[2] = g[2] * scale;
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
 * Each awake body's world inverse inertia, R diag(1/I) Rᵀ, into world._invI
 * as its six distinct entries, for the step's contacts and joints. Every body
 * in either is awake by now, or static, whose stays zero.
 */
export function inertias(world) {
  const list = world._awakeList, n = world._awakeCount;
  for (let i = 0; i < n; i++) inverseInertia(world, list[i]);
}

/**
 * What the solver needs that holds for the whole step, per point: the anchors
 * (from each body's centre, in world space and in the body's own frame), the
 * effective masses along the normal and both tangents, and the speed the
 * bodies were closing at. Reads the inverse inertias inertias() worked out.
 */
export function prepare(world, c) {
  const invI = world._invI, invMass = world._invMass;
  const p = world._position, q = world._rotation, v = world._velocity, w = world._angular;
  for (let m = 0; m < c.count; m++) {
    const a = c.a[m], b = c.b[m], a3 = a * 3, b3 = b * 3;
    const ima = invMass[a], imb = invMass[b];
    const nx = c.normal[3 * m], ny = c.normal[3 * m + 1], nz = c.normal[3 * m + 2];
    const partA = c.partA[m] < 0 ? null : world._inner[a].list[c.partA[m]];
    const partB = c.partB[m] < 0 ? null : world._inner[b].list[c.partB[m]];
    const m3 = 3 * m;
    if (c.points[m] <= 2) {
      c.rollMass1[m] = turning(invI, a, b, c.t1[m3], c.t1[m3 + 1], c.t1[m3 + 2]);
      c.rollMass2[m] = turning(invI, a, b, c.t2[m3], c.t2[m3 + 1], c.t2[m3 + 2]);
    }
    c.spinMass[m] = turning(invI, a, b, nx, ny, nz);
    // The friction anchor's sums (see FRICTION): each point weighed 1 within
    // the tolerance of touching, down to 0 at twice it; evenly if none is near.
    const tol = c.tolerance[m];
    let weight = 0, ax = 0, ay = 0, az = 0, bx = 0, by = 0, bz = 0;
    let ex = 0, ey = 0, ez = 0, fx = 0, fy = 0, fz = 0;
    for (let k = 0; k < c.points[m]; k++) {
      const i = m * P + k, i3 = 3 * i;
      const px = c.point[i3], py = c.point[i3 + 1], pz = c.point[i3 + 2];
      const rax = px - p[a3], ray = py - p[a3 + 1], raz = pz - p[a3 + 2];
      const rbx = px + c.shift[i3] - p[b3], rby = py + c.shift[i3 + 1] - p[b3 + 1], rbz = pz + c.shift[i3 + 2] - p[b3 + 2];
      c.ra[i3] = rax; c.ra[i3 + 1] = ray; c.ra[i3 + 2] = raz;
      c.rb[i3] = rbx; c.rb[i3 + 1] = rby; c.rb[i3 + 2] = rbz;
      // Separation as the bodies move during the step is the starting
      // separation plus how far the two anchors have moved apart along the
      // normal; this is the part of that sum fixed at the start.
      core(world, a, partA, rax, ray, raz, c.la, i3);
      const cax = v3[0], cay = v3[1], caz = v3[2];
      core(world, b, partB, rbx, rby, rbz, c.lb, i3);
      c.adjusted[i] = c.separation[i] - ((v3[0] - cax) * nx + (v3[1] - cay) * ny + (v3[2] - caz) * nz);
      c.normalMass[i] = lever(c.jac, J * i, invI, ima, imb, a, b, rax, ray, raz, rbx, rby, rbz, nx, ny, nz);
      c.maxNormalImpulse[i] = 0;
      const s = c.separation[i] / tol, near = s <= 1 ? 1 : s >= 2 ? 0 : 2 - s;
      weight += near;
      ax += near * rax; ay += near * ray; az += near * raz; bx += near * rbx; by += near * rby; bz += near * rbz;
      ex += rax; ey += ray; ez += raz; fx += rbx; fy += rby; fz += rbz;
      // Closing speed along the normal, before anything this step: what a bounce is measured from.
      const dvx = v[b3] + w[b3 + 1] * rbz - w[b3 + 2] * rby - v[a3] - w[a3 + 1] * raz + w[a3 + 2] * ray;
      const dvy = v[b3 + 1] + w[b3 + 2] * rbx - w[b3] * rbz - v[a3 + 1] - w[a3 + 2] * rax + w[a3] * raz;
      const dvz = v[b3 + 2] + w[b3] * rby - w[b3 + 1] * rbx - v[a3 + 2] - w[a3] * ray + w[a3 + 1] * rax;
      c.relativeVelocity[i] = dvx * nx + dvy * ny + dvz * nz;
    }
    if (weight === 0) { weight = c.points[m]; ax = ex; ay = ey; az = ez; bx = fx; by = fy; bz = fz; }
    ax /= weight; ay /= weight; az /= weight; bx /= weight; by /= weight; bz /= weight;
    c.anchorA[m3] = ax; c.anchorA[m3 + 1] = ay; c.anchorA[m3 + 2] = az;
    c.anchorB[m3] = bx; c.anchorB[m3 + 1] = by; c.anchorB[m3 + 2] = bz;
    // Each point's twist lever: its distance from the anchor across the normal.
    for (let k = 0; k < c.points[m]; k++) {
      const i3 = 3 * (m * P + k);
      let dx = c.ra[i3] - ax, dy = c.ra[i3 + 1] - ay, dz = c.ra[i3 + 2] - az;
      const along = dx * nx + dy * ny + dz * nz;
      dx -= along * nx; dy -= along * ny; dz -= along * nz;
      c.lever[m * P + k] = Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
    // The 2x2 block along t1 and t2 at the anchor, and its inverse. The
    // tangents are at right angles, so the masses add nothing off the diagonal.
    const f = FJ * m, jac = c.fjac;
    lever(jac, f, invI, ima, imb, a, b, ax, ay, az, bx, by, bz, c.t1[m3], c.t1[m3 + 1], c.t1[m3 + 2]);
    lever(jac, f + 12, invI, ima, imb, a, b, ax, ay, az, bx, by, bz, c.t2[m3], c.t2[m3 + 1], c.t2[m3 + 2]);
    const k11 = ima + imb + jac[f] * jac[f + 6] + jac[f + 1] * jac[f + 7] + jac[f + 2] * jac[f + 8]
      + jac[f + 3] * jac[f + 9] + jac[f + 4] * jac[f + 10] + jac[f + 5] * jac[f + 11];
    const k22 = ima + imb + jac[f + 12] * jac[f + 18] + jac[f + 13] * jac[f + 19] + jac[f + 14] * jac[f + 20]
      + jac[f + 15] * jac[f + 21] + jac[f + 16] * jac[f + 22] + jac[f + 17] * jac[f + 23];
    const k12 = jac[f + 12] * jac[f + 6] + jac[f + 13] * jac[f + 7] + jac[f + 14] * jac[f + 8]
      + jac[f + 15] * jac[f + 9] + jac[f + 16] * jac[f + 10] + jac[f + 17] * jac[f + 11];
    const det = k11 * k22 - k12 * k12;
    if (det > 0) {
      c.tangentMass[m3] = k22 / det; c.tangentMass[m3 + 1] = -k12 / det; c.tangentMass[m3 + 2] = k11 / det;
    } else {
      c.tangentMass[m3] = 0; c.tangentMass[m3 + 1] = 0; c.tangentMass[m3 + 2] = 0;
    }
  }
}

/** world._invI for slot s: R diag(1/I) Rᵀ for a dynamic body, zero for anything else. */
function inverseInertia(world, s) {
  const o = s * 6, out = world._invI;
  if (world._type[s] !== DYNAMIC) { out.fill(0, o, o + 6); return; }
  if (world._lock[s] !== 0) { lockedInverse(world._axes, s * 9, world._inertia, s * 3, world._lock[s], out, o); return; }
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
 * The world inverse inertia of a body that may not turn about some world
 * axes (bit k of `lock` for axis k), into out[o..o+5] as xx xy xz yy yz zz.
 * A locked axis is one no torque turns it about: as if its inertia there were
 * endless. So this is the inverse of its world inertia taken on the free axes
 * alone, with nothing on the locked ones. Cutting the locked rows out of the
 * free inverse instead would be right only when the free axes are the body's
 * own principal axes. The body's axes are at ax[a9..a9+8], its principal
 * moments at I[i3..i3+2].
 */
export function lockedInverse(ax, a9, I, i3, lock, out, o) {
  const i0 = I[i3], i1 = I[i3 + 1], i2 = I[i3 + 2];
  const x0 = ax[a9], x1 = ax[a9 + 1], x2 = ax[a9 + 2];
  const y0 = ax[a9 + 3], y1 = ax[a9 + 4], y2 = ax[a9 + 5];
  const z0 = ax[a9 + 6], z1 = ax[a9 + 7], z2 = ax[a9 + 8];
  // R diag(I) Rᵀ, entry (r, c) = Σ_k R_rk R_ck I_k: the world inertia.
  M6[0] = x0 * x0 * i0 + y0 * y0 * i1 + z0 * z0 * i2;
  M6[1] = x0 * x1 * i0 + y0 * y1 * i1 + z0 * z1 * i2;
  M6[2] = x0 * x2 * i0 + y0 * y2 * i1 + z0 * z2 * i2;
  M6[3] = x1 * x1 * i0 + y1 * y1 * i1 + z1 * z1 * i2;
  M6[4] = x1 * x2 * i0 + y1 * y2 * i1 + z1 * z2 * i2;
  M6[5] = x2 * x2 * i0 + y2 * y2 * i1 + z2 * z2 * i2;
  out.fill(0, o, o + 6);
  let first = -1, second = -1;
  for (let k = 0; k < 3; k++) {
    if (lock >> k & 1) continue;
    if (first < 0) first = k; else second = k;
  }
  if (first < 0) return;
  const ff = SYM[4 * first];
  if (second < 0) {
    if (M6[ff] > 0) out[o + ff] = 1 / M6[ff];
    return;
  }
  // Two free axes: the 2x2 inverse.
  const ss = SYM[4 * second], fs = SYM[3 * first + second];
  const det = M6[ff] * M6[ss] - M6[fs] * M6[fs];
  if (!(det > 0)) return;
  out[o + ff] = M6[ss] / det; out[o + ss] = M6[ff] / det; out[o + fs] = -M6[fs] / det;
}

const M6 = new Float64Array(6);
/** Where entry (r, c) of a symmetric 3x3 sits in xx xy xz yy yz zz: SYM[3r + c]. */
const SYM = new Int8Array([0, 1, 2, 1, 3, 4, 2, 4, 5]);

/**
 * The effective mass along unit d at a contact: 1 / (1/ma + 1/mb +
 * (ra × d)·Ia⁻¹(ra × d) + (rb × d)·Ib⁻¹(rb × d)). How much impulse it takes to
 * change the bodies' relative speed there by 1 m/s, turning included.
 */
/**
 * One direction's turning terms into jac[j..j+11] (see J): ra x d, rb x d,
 * and each turned by its body's inverse inertia. Returns the pair's effective
 * mass along d, 1 / (1/ma + 1/mb + (ra x d)·Ia⁻¹(ra x d) + (rb x d)·Ib⁻¹(rb x d)).
 */
function lever(jac, j, invI, ima, imb, a, b, rax, ray, raz, rbx, rby, rbz, dx, dy, dz) {
  const cax = ray * dz - raz * dy, cay = raz * dx - rax * dz, caz = rax * dy - ray * dx;
  const cbx = rby * dz - rbz * dy, cby = rbz * dx - rbx * dz, cbz = rbx * dy - rby * dx;
  const oa = a * 6, ob = b * 6;
  const iax = invI[oa] * cax + invI[oa + 1] * cay + invI[oa + 2] * caz;
  const iay = invI[oa + 1] * cax + invI[oa + 3] * cay + invI[oa + 4] * caz;
  const iaz = invI[oa + 2] * cax + invI[oa + 4] * cay + invI[oa + 5] * caz;
  const ibx = invI[ob] * cbx + invI[ob + 1] * cby + invI[ob + 2] * cbz;
  const iby = invI[ob + 1] * cbx + invI[ob + 3] * cby + invI[ob + 4] * cbz;
  const ibz = invI[ob + 2] * cbx + invI[ob + 4] * cby + invI[ob + 5] * cbz;
  jac[j] = cax; jac[j + 1] = cay; jac[j + 2] = caz;
  jac[j + 3] = cbx; jac[j + 4] = cby; jac[j + 5] = cbz;
  jac[j + 6] = iax; jac[j + 7] = iay; jac[j + 8] = iaz;
  jac[j + 9] = ibx; jac[j + 10] = iby; jac[j + 11] = ibz;
  const k = ima + imb + cax * iax + cay * iay + caz * iaz + cbx * ibx + cby * iby + cbz * ibz;
  return k > 0 ? 1 / k : 0;
}

// ---- solve ----

/**
 * How soft a contact is, as Box2D 3 makes it from a frequency and a damping
 * ratio over the substep h: biasRate turns an overlap into a closing-speed
 * target, massScale and impulseScale blend the spring into the impulse.
 */
export const soft = { biasRate: 0, massScale: 1, impulseScale: 0 };
export function makeSoft(hertz, zeta, h, out = soft) {
  const omega = 2 * Math.PI * hertz;
  const a1 = 2 * zeta + h * omega, a2 = h * omega * a1, a3 = 1 / (1 + a2);
  out.biasRate = omega / a1;
  out.massScale = a2 * a3;
  out.impulseScale = a3;
}

/**
 * Applies every point's accumulated impulse, at the start of each substep:
 * the warm start, which begins each substep at last time's answer.
 */
export function warmStart(world, c) {
  const v = world._velocity, w = world._angular, invI = world._invI, invMass = world._invMass, jac = c.jac;
  for (let m = 0; m < c.count; m++) {
    const a = c.a[m], b = c.b[m], a3 = a * 3, b3 = b * 3, m3 = 3 * m;
    const ima = invMass[a], imb = invMass[b];
    const nx = c.normal[m3], ny = c.normal[m3 + 1], nz = c.normal[m3 + 2];
    const t1x = c.t1[m3], t1y = c.t1[m3 + 1], t1z = c.t1[m3 + 2];
    const t2x = c.t2[m3], t2y = c.t2[m3 + 1], t2z = c.t2[m3 + 2];
    // The pair's velocities in locals for the manifold, as in solve().
    let vax = v[a3], vay = v[a3 + 1], vaz = v[a3 + 2], wax = w[a3], way = w[a3 + 1], waz = w[a3 + 2];
    let vbx = v[b3], vby = v[b3 + 1], vbz = v[b3 + 2], wbx = w[b3], wby = w[b3 + 1], wbz = w[b3 + 2];
    for (let k = 0; k < c.points[m]; k++) {
      const i = m * P + k, j = J * i;
      const ln = c.normalImpulse[i];
      const px = nx * ln, py = ny * ln, pz = nz * ln;
      vax -= ima * px; vay -= ima * py; vaz -= ima * pz;
      vbx += imb * px; vby += imb * py; vbz += imb * pz;
      wax -= jac[j + 6] * ln; way -= jac[j + 7] * ln; waz -= jac[j + 8] * ln;
      wbx += jac[j + 9] * ln; wby += jac[j + 10] * ln; wbz += jac[j + 11] * ln;
    }
    {
      // Friction at the anchor.
      const f = FJ * m, fj = c.fjac, l1 = c.tangentImpulse1[m], l2 = c.tangentImpulse2[m];
      const px = t1x * l1 + t2x * l2, py = t1y * l1 + t2y * l2, pz = t1z * l1 + t2z * l2;
      vax -= ima * px; vay -= ima * py; vaz -= ima * pz;
      vbx += imb * px; vby += imb * py; vbz += imb * pz;
      wax -= fj[f + 6] * l1 + fj[f + 18] * l2; way -= fj[f + 7] * l1 + fj[f + 19] * l2; waz -= fj[f + 8] * l1 + fj[f + 20] * l2;
      wbx += fj[f + 9] * l1 + fj[f + 21] * l2; wby += fj[f + 10] * l1 + fj[f + 22] * l2; wbz += fj[f + 11] * l1 + fj[f + 23] * l2;
    }
    {
      // Rolling (none past two points: carry() and solve() leave it 0) and twist.
      const r1 = c.rollImpulse1[m], r2 = c.rollImpulse2[m], sp = c.spinImpulse[m], oa = a * 6, ob = b * 6;
      const lx = t1x * r1 + t2x * r2 + nx * sp, ly = t1y * r1 + t2y * r2 + ny * sp, lz = t1z * r1 + t2z * r2 + nz * sp;
      wax -= invI[oa] * lx + invI[oa + 1] * ly + invI[oa + 2] * lz;
      way -= invI[oa + 1] * lx + invI[oa + 3] * ly + invI[oa + 4] * lz;
      waz -= invI[oa + 2] * lx + invI[oa + 4] * ly + invI[oa + 5] * lz;
      wbx += invI[ob] * lx + invI[ob + 1] * ly + invI[ob + 2] * lz;
      wby += invI[ob + 1] * lx + invI[ob + 3] * ly + invI[ob + 4] * lz;
      wbz += invI[ob + 2] * lx + invI[ob + 4] * ly + invI[ob + 5] * lz;
    }
    v[a3] = vax; v[a3 + 1] = vay; v[a3 + 2] = vaz; w[a3] = wax; w[a3 + 1] = way; w[a3 + 2] = waz;
    v[b3] = vbx; v[b3 + 1] = vby; v[b3 + 2] = vbz; w[b3] = wbx; w[b3 + 1] = wby; w[b3 + 2] = wbz;
  }
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

/**
 * Equal and opposite impulse l along the unit direction d, whose turning
 * terms are at jac[j..] (see J): -l d on a, +l d on b.
 */
function push(v, w, ima, imb, a3, b3, jac, j, dx, dy, dz, l) {
  const px = dx * l, py = dy * l, pz = dz * l;
  v[a3] -= ima * px; v[a3 + 1] -= ima * py; v[a3 + 2] -= ima * pz;
  v[b3] += imb * px; v[b3 + 1] += imb * py; v[b3 + 2] += imb * pz;
  w[a3] -= jac[j + 6] * l; w[a3 + 1] -= jac[j + 7] * l; w[a3 + 2] -= jac[j + 8] * l;
  w[b3] += jac[j + 9] * l; w[b3 + 1] += jac[j + 10] * l; w[b3 + 2] += jac[j + 11] * l;
}

/**
 * Manifold m's least separation where its bodies are now, as the solve
 * measures it: for World._catch, which moves a body after the last pass.
 */
export function gapOf(world, c, m) {
  const p = world._position, start = world._start, q = world._rotation;
  const a = c.a[m], b = c.b[m], a3 = a * 3, b3 = b * 3, m3 = 3 * m;
  const nx = c.normal[m3], ny = c.normal[m3 + 1], nz = c.normal[m3 + 2];
  const dx = (p[b3] - start[b3]) - (p[a3] - start[a3]);
  const dy = (p[b3 + 1] - start[b3 + 1]) - (p[a3 + 1] - start[a3 + 1]);
  const dz = (p[b3 + 2] - start[b3 + 2]) - (p[a3 + 2] - start[a3 + 2]);
  let least = Infinity;
  for (let k = 0; k < c.points[m]; k++) {
    const i = m * P + k, i3 = 3 * i;
    rotate(q, a * 4, c.la[i3], c.la[i3 + 1], c.la[i3 + 2], false, v3);
    const cax = v3[0], cay = v3[1], caz = v3[2];
    rotate(q, b * 4, c.lb[i3], c.lb[i3 + 1], c.lb[i3 + 2], false, v3);
    const s = (dx + v3[0] - cax) * nx + (dy + v3[1] - cay) * ny + (dz + v3[2] - caz) * nz + c.adjusted[i];
    if (s < least) least = s;
  }
  return least;
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
    // The pair's velocities, held here for the whole manifold and written back
    // once at its end: every sweep and friction reads and writes them, and
    // reading and writing the shared columns at each push cost more than the
    // arithmetic. Measured: see ROADMAP.md, milestone 16.
    const jac = c.jac, ima = invMass[a], imb = invMass[b];
    let vax = v[a3], vay = v[a3 + 1], vaz = v[a3 + 2], wax = w[a3], way = w[a3 + 1], waz = w[a3 + 2];
    let vbx = v[b3], vby = v[b3 + 1], vbz = v[b3 + 2], wbx = w[b3], wby = w[b3 + 1], wbz = w[b3 + 2];
    // One sweep per point. The points of a face contact are coupled: four
    // points on a box's face hold only three things (its height and its two
    // tilts), and a single sweep visits them in an order, so the first stops
    // the whole fall on its own and tips the box, and friction turns the tip
    // into a slide. Sweeping the manifold's own points as many times as it
    // has points lets each one's correction reach every other. Measured, a 1 m
    // box dropped flat 2.5 m at 4 substeps: 7.6 mm of slide with one sweep,
    // 0.04 mm with two, none to a micrometre with four. A one-point contact
    // gets one sweep, which already solves it exactly. Measured again at 8
    // substeps (milestone 16): one sweep slid it 1.5 mm, two 0.01 mm and let
    // the pyramid stray 2.4 mm where four keep it to 1.0, for 9 to 16% of a
    // step saved; kept at four.
    for (let sweep = 0; sweep < count; sweep++) {
      for (let k = 0; k < count; k++) {
        const i = m * P + k, j = J * i;
        const vn = (vbx - vax) * nx + (vby - vay) * ny + (vbz - vaz) * nz
          + wbx * jac[j + 3] + wby * jac[j + 4] + wbz * jac[j + 5] - wax * jac[j] - way * jac[j + 1] - waz * jac[j + 2];
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
        const px = nx * d, py = ny * d, pz = nz * d;
        vax -= ima * px; vay -= ima * py; vaz -= ima * pz;
        vbx += imb * px; vby += imb * py; vbz += imb * pz;
        wax -= jac[j + 6] * d; way -= jac[j + 7] * d; waz -= jac[j + 8] * d;
        wbx += jac[j + 9] * d; wby += jac[j + 10] * d; wbz += jac[j + 11] * d;
      }
    }
    // Friction, once for the manifold at its anchor (see FRICTION): both
    // tangents as one 2x2 block, held within a circle of mu times the whole
    // normal impulse, so a box sliding diagonally slows exactly as one
    // sliding straight.
    const mu = c.friction[m], oa = a * 6, ob = b * 6;
    const t1x = c.t1[m3], t1y = c.t1[m3 + 1], t1z = c.t1[m3 + 2];
    const t2x = c.t2[m3], t2y = c.t2[m3 + 1], t2z = c.t2[m3 + 2];
    let pressed = 0, twisted = 0;
    for (let k = 0; k < count; k++) {
      const i = m * P + k;
      pressed += c.normalImpulse[i];
      twisted += c.lever[i] * c.normalImpulse[i];
    }
    {
      const f = FJ * m, fj = c.fjac, g = f + 12;
      const dvx = vbx - vax, dvy = vby - vay, dvz = vbz - vaz;
      let v1 = dvx * t1x + dvy * t1y + dvz * t1z
        + wbx * fj[f + 3] + wby * fj[f + 4] + wbz * fj[f + 5] - wax * fj[f] - way * fj[f + 1] - waz * fj[f + 2];
      let v2 = dvx * t2x + dvy * t2y + dvz * t2z
        + wbx * fj[g + 3] + wby * fj[g + 4] + wbz * fj[g + 5] - wax * fj[g] - way * fj[g + 1] - waz * fj[g + 2];
      // The solving pass also pulls back the slip carried in (see store), as
      // fast as an overlap is pushed out, and no faster than a pixel a substep.
      const s1 = c.drift1[m], s2 = c.drift2[m];
      if (bias && (s1 !== 0 || s2 !== 0)) {
        let b1 = soft.biasRate * s1, b2 = soft.biasRate * s2;
        const bb = b1 * b1 + b2 * b2;
        if (bb > maxBias * maxBias) {
          const scale = maxBias / Math.sqrt(bb);
          b1 *= scale; b2 *= scale;
        }
        v1 += b1; v2 += b2;
      }
      const tm = c.tangentMass, o1 = c.tangentImpulse1[m], o2 = c.tangentImpulse2[m];
      let n1 = o1 - (tm[m3] * v1 + tm[m3 + 1] * v2), n2 = o2 - (tm[m3 + 1] * v1 + tm[m3 + 2] * v2);
      const limit = mu * pressed, l2 = n1 * n1 + n2 * n2;
      if (l2 > limit * limit) {
        const scale = limit / Math.sqrt(l2);
        n1 *= scale; n2 *= scale;
        c.sliding[m] = 1;
      } else {
        c.sliding[m] = 0;
      }
      c.tangentImpulse1[m] = n1; c.tangentImpulse2[m] = n2;
      const d1 = n1 - o1, d2 = n2 - o2;
      const px = t1x * d1 + t2x * d2, py = t1y * d1 + t2y * d2, pz = t1z * d1 + t2z * d2;
      vax -= ima * px; vay -= ima * py; vaz -= ima * pz;
      vbx += imb * px; vby += imb * py; vbz += imb * pz;
      wax -= fj[f + 6] * d1 + fj[g + 6] * d2; way -= fj[f + 7] * d1 + fj[g + 7] * d2; waz -= fj[f + 8] * d1 + fj[g + 8] * d2;
      wbx += fj[f + 9] * d1 + fj[g + 9] * d2; wby += fj[f + 10] * d1 + fj[g + 10] * d2; wbz += fj[f + 11] * d1 + fj[g + 11] * d2;
    }
    if (count <= 2) {
      // Rolling (see PATCHES): the turning of b against a about the two
      // tangents, held within the patch's push leaning to its edge, a disc as
      // friction's is.
      const most = c.patch[m] * pressed;
      const ux = wbx - wax, uy = wby - way, uz = wbz - waz;
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
      // An angular impulse L: -L on a, +L on b, each through its inverse inertia.
      const lx = t1x * d1 + t2x * d2, ly = t1y * d1 + t2y * d2, lz = t1z * d1 + t2z * d2;
      wax -= invI[oa] * lx + invI[oa + 1] * ly + invI[oa + 2] * lz;
      way -= invI[oa + 1] * lx + invI[oa + 3] * ly + invI[oa + 4] * lz;
      waz -= invI[oa + 2] * lx + invI[oa + 4] * ly + invI[oa + 5] * lz;
      wbx += invI[ob] * lx + invI[ob + 1] * ly + invI[ob + 2] * lz;
      wby += invI[ob + 1] * lx + invI[ob + 3] * ly + invI[ob + 4] * lz;
      wbz += invI[ob + 2] * lx + invI[ob + 4] * ly + invI[ob + 5] * lz;
    }
    {
      // Twist about the normal: within mu times each point's normal impulse
      // times its lever from the anchor; one point has no lever, so its
      // patch's spinning friction (see PATCHES).
      const limit = count === 1 ? SPIN * mu * c.patch[m] * pressed : mu * twisted;
      const s0 = c.spinImpulse[m];
      let s1 = s0 - c.spinMass[m] * ((wbx - wax) * nx + (wby - way) * ny + (wbz - waz) * nz);
      if (s1 > limit) s1 = limit; else if (s1 < -limit) s1 = -limit;
      c.spinImpulse[m] = s1;
      const ds = s1 - s0, lx = nx * ds, ly = ny * ds, lz = nz * ds;
      wax -= invI[oa] * lx + invI[oa + 1] * ly + invI[oa + 2] * lz;
      way -= invI[oa + 1] * lx + invI[oa + 3] * ly + invI[oa + 4] * lz;
      waz -= invI[oa + 2] * lx + invI[oa + 4] * ly + invI[oa + 5] * lz;
      wbx += invI[ob] * lx + invI[ob + 1] * ly + invI[ob + 2] * lz;
      wby += invI[ob + 1] * lx + invI[ob + 3] * ly + invI[ob + 4] * lz;
      wbz += invI[ob + 2] * lx + invI[ob + 4] * ly + invI[ob + 5] * lz;
    }
    v[a3] = vax; v[a3 + 1] = vay; v[a3 + 2] = vaz; w[a3] = wax; w[a3 + 1] = way; w[a3 + 2] = waz;
    v[b3] = vbx; v[b3 + 1] = vby; v[b3 + 2] = vbz; w[b3] = wbx; w[b3 + 1] = wby; w[b3 + 2] = wbz;
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
      c.point[d3] + (c.shift[d3] + p3[a3] - start[a3] + p3[b3] - start[b3]) * 0.5,
      c.point[d3 + 1] + (c.shift[d3 + 1] + p3[a3 + 1] - start[a3 + 1] + p3[b3 + 1] - start[b3 + 1]) * 0.5,
      c.point[d3 + 2] + (c.shift[d3 + 2] + p3[a3 + 2] - start[a3 + 2] + p3[b3 + 2] - start[b3 + 2]) * 0.5,
      c.normal[h3], c.normal[h3 + 1], c.normal[h3 + 2], speed);
  }
}

/** How fast b moves against a at a point, along the unit d whose turning terms are at jac[j..] (see J). */
function closing(v, w, a3, b3, jac, j, dx, dy, dz) {
  return (v[b3] - v[a3]) * dx + (v[b3 + 1] - v[a3 + 1]) * dy + (v[b3 + 2] - v[a3 + 2]) * dz
    + w[b3] * jac[j + 3] + w[b3 + 1] * jac[j + 4] + w[b3 + 2] * jac[j + 5]
    - w[a3] * jac[j] - w[a3 + 1] * jac[j + 1] - w[a3 + 2] * jac[j + 2];
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
      const vn = closing(v, w, a3, b3, c.jac, J * i, nx, ny, nz);
      const old = c.normalImpulse[i];
      let next = old - c.normalMass[i] * (vn + e * c.relativeVelocity[i]);
      if (next < 0) next = 0;
      const d = next - old;
      c.normalImpulse[i] = next;
      push(v, w, invMass[a], invMass[b], a3, b3, c.jac, J * i, nx, ny, nz, d);
    }
  }
}
