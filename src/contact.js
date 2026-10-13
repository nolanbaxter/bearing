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
// WARM STARTING. Each point starts the step with the impulse it ended the
// last one with, matched by the narrowphase's feature ids. A resting stack
// needs the same impulses step after step, so it starts at the answer. The
// impulses are kept on the pair, which lasts as long as the two bodies' fat
// boxes overlap, so nothing has to be looked up: store() writes them after
// the solve, and the next step's carry() reads them back.

import { collide, Shape, Manifold, SPHERE, CAPSULE, BOX as BOX_KIND } from './collide.js';
import { rotate } from './core/math.js';
import { resized } from './core/slots.js';

const DYNAMIC = 0, STATIC = 1;
const BOX = 0, SPHERE_SHAPE = 1;   // World's shape codes

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
    grow('points', Uint8Array, m);
    grow('normal', Float64Array, 3 * m); grow('t1', Float64Array, 3 * m); grow('t2', Float64Array, 3 * m);
    grow('friction', Float64Array, m); grow('restitution', Float64Array, m); grow('tolerance', Float64Array, m);
    grow('pid', Uint32Array, p);
    grow('separation', Float64Array, p);
    grow('point', Float64Array, 3 * p);
    grow('ra', Float64Array, 3 * p); grow('rb', Float64Array, 3 * p);
    grow('la', Float64Array, 3 * p); grow('lb', Float64Array, 3 * p);
    grow('adjusted', Float64Array, p);
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
const v3 = new Float64Array(3);

/**
 * Fills `next` with this step's manifolds, one for each pair in the pair
 * list whose shapes touch, or will within the step. A pair that no longer
 * holds (a body gone, filtered apart, or fat boxes parted) is dropped from
 * the list here. Each manifold starts from the impulses its pair remembers.
 */
export function narrow(world, pairs, broadphase, next) {
  const box = world._box, reach = world._motion;
  next.count = 0;
  for (let p = 0; p < pairs.count;) {
    if (!pairs.holds(p, broadphase)) {
      pairs.remove(p);   // the last pair moves into p, so p is looked at again
      continue;
    }
    const a = pairs.a[p], b = pairs.b[p], a6 = a * 6, b6 = b * 6;
    // Fat boxes overlap, but their actual boxes, grown only by this step's
    // motion, may not: nothing can touch this step, and nothing is carried.
    if (box[a6] > box[b6 + 3] || box[b6] > box[a6 + 3] || box[a6 + 1] > box[b6 + 4]
      || box[b6 + 1] > box[a6 + 4] || box[a6 + 2] > box[b6 + 5] || box[b6 + 2] > box[a6 + 5]) {
      pairs.points[p] = 0;
      p++;
      continue;
    }
    load(world, a, shapeA);
    load(world, b, shapeB);
    const tolerance = Math.min(world._size[a], world._size[b]) * PIXEL;
    const margin = reach[a] + reach[b] + tolerance;
    if (collide(shapeA, shapeB, margin, tolerance, manifold) === 0) {
      pairs.points[p] = 0;
      p++;
      continue;
    }

    const m = next.count++;
    next.ensure(next.count);
    next.a[m] = a; next.b[m] = b; next.pair[m] = p;
    next.points[m] = manifold.count;
    for (let k = 0; k < 3; k++) next.normal[3 * m + k] = manifold.normal[k];
    // Friction mixes as a geometric mean, so ice is slippery against
    // anything; restitution takes the larger, so a bouncy ball bounces off
    // anything. Box2D's rules.
    next.friction[m] = Math.sqrt(world._friction[a] * world._friction[b]);
    next.restitution[m] = Math.max(world._restitution[a], world._restitution[b]);
    next.tolerance[m] = tolerance;
    for (let k = 0; k < manifold.count; k++) {
      const i = m * P + k;
      next.pid[i] = manifold.ids[k];
      next.separation[i] = manifold.separations[k];
      for (let c = 0; c < 3; c++) next.point[3 * i + c] = manifold.points[3 * k + c];
      next.normalImpulse[i] = 0; next.tangentImpulse1[i] = 0; next.tangentImpulse2[i] = 0;
    }
    basis(next, m);
    carry(pairs, p, next, m);
    p++;
  }
  return next.count;
}

/**
 * The pair's remembered impulses onto this step's points with the same
 * feature id. Friction is remembered as a vector in world space and laid onto
 * this step's tangents, since the tangents turn with the normal from step to step.
 */
function carry(pairs, p, next, m) {
  const remembered = pairs.points[p];
  if (remembered === 0) return;
  for (let k = 0; k < next.points[m]; k++) {
    const i = m * P + k, id = next.pid[i];
    for (let j = 0; j < remembered; j++) {
      const o = p * P + j;
      if (pairs.pid[o] !== id) continue;
      next.normalImpulse[i] = pairs.normalImpulse[o];
      const fx = pairs.friction[3 * o], fy = pairs.friction[3 * o + 1], fz = pairs.friction[3 * o + 2];
      next.tangentImpulse1[i] = fx * next.t1[3 * m] + fy * next.t1[3 * m + 1] + fz * next.t1[3 * m + 2];
      next.tangentImpulse2[i] = fx * next.t2[3 * m] + fy * next.t2[3 * m + 1] + fz * next.t2[3 * m + 2];
      break;
    }
  }
}

/** After the solve: each pair remembers its points' ids and impulses, for the next step. */
export function store(pairs, c) {
  for (let m = 0; m < c.count; m++) {
    const p = c.pair[m], count = c.points[m];
    pairs.points[p] = count;
    for (let k = 0; k < count; k++) {
      const i = m * P + k, o = p * P + k, m3 = 3 * m;
      pairs.pid[o] = c.pid[i];
      pairs.normalImpulse[o] = c.normalImpulse[i];
      const l1 = c.tangentImpulse1[i], l2 = c.tangentImpulse2[i];
      pairs.friction[3 * o] = c.t1[m3] * l1 + c.t2[m3] * l2;
      pairs.friction[3 * o + 1] = c.t1[m3 + 1] * l1 + c.t2[m3 + 1] * l2;
      pairs.friction[3 * o + 2] = c.t1[m3 + 2] * l1 + c.t2[m3 + 2] * l2;
    }
  }
}

/** A body's shape, placed, into s. */
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
  const n = world._slots.top, state = world._slots.state;
  for (let s = 0; s < n; s++) if (state[s] === 1) inverseInertia(world, s);
  const p = world._position, q = world._rotation, v = world._velocity, w = world._angular;
  for (let m = 0; m < c.count; m++) {
    const a = c.a[m], b = c.b[m], a3 = a * 3, b3 = b * 3;
    const ima = invMass[a], imb = invMass[b];
    const nx = c.normal[3 * m], ny = c.normal[3 * m + 1], nz = c.normal[3 * m + 2];
    for (let k = 0; k < c.points[m]; k++) {
      const i = m * P + k, i3 = 3 * i;
      const px = c.point[i3], py = c.point[i3 + 1], pz = c.point[i3 + 2];
      const rax = px - p[a3], ray = py - p[a3 + 1], raz = pz - p[a3 + 2];
      const rbx = px - p[b3], rby = py - p[b3 + 1], rbz = pz - p[b3 + 2];
      c.ra[i3] = rax; c.ra[i3 + 1] = ray; c.ra[i3 + 2] = raz;
      c.rb[i3] = rbx; c.rb[i3 + 1] = rby; c.rb[i3 + 2] = rbz;
      rotate(q, a * 4, rax, ray, raz, true, v3);
      c.la[i3] = v3[0]; c.la[i3 + 1] = v3[1]; c.la[i3 + 2] = v3[2];
      rotate(q, b * 4, rbx, rby, rbz, true, v3);
      c.lb[i3] = v3[0]; c.lb[i3 + 1] = v3[1]; c.lb[i3 + 2] = v3[2];
      // Separation as the bodies move during the step is the starting
      // separation plus how far the two anchor points have moved apart along
      // the normal; this is the part of that sum fixed at the start.
      c.adjusted[i] = c.separation[i] - ((rbx - rax) * nx + (rby - ray) * ny + (rbz - raz) * nz);
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
  }
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
    for (let k = 0; k < count; k++) {
      const i = m * P + k, i3 = 3 * i;
      // The separation now: the anchors where the bodies' current rotations put
      // them. It can't change within a pass, since nothing moves until _move.
      rotate(q, a * 4, c.la[i3], c.la[i3 + 1], c.la[i3 + 2], false, v3);
      const cax = v3[0], cay = v3[1], caz = v3[2];
      rotate(q, b * 4, c.lb[i3], c.lb[i3 + 1], c.lb[i3 + 2], false, v3);
      const s = (dx + v3[0] - cax) * nx + (dy + v3[1] - cay) * ny + (dz + v3[2] - caz) * nz + c.adjusted[i];
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
