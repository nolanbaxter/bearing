// Time of impact: when, within a step, two moving shapes first touch.
//
// WHY. Contacts start before bodies touch (collide.js, SPECULATIVE): every
// pair close enough to meet within the step gets its contact at the step's
// start, and the solver lets it close only the gap. That is what stops a fast
// body passing through a thin one. But the contact is a plane, fixed where the
// step starts, and a body crossing that plane is stopped whether or not the
// shapes would meet: a ball heading past a box's corner, about to miss it,
// was slowed or turned by as much as 73% of its speed at 5 m/s with 1 cm to
// spare, and every time at 100 m/s with 20 cm to spare; a ball aimed through
// a gap 10 cm wider than it bounced back. And a plane fixed at the start
// can't follow a body that turns within the step: a 10 cm rod at 200 m/s,
// spinning at 30 to 80 rad/s, went through a 2 cm wall 28 times in 50.
//
// So a pair that isn't touching yet, and moves against itself by more than a
// pixel in the step, is first asked when it meets. Not this step: it gets no
// contact. At time t: its contact is made from where both bodies will be
// then, with the real normal and points of impact, and the solver lets it
// close to exactly there. Fast bodies are still stopped by anything, dynamic
// bodies included, with nothing to flag, and keep the rest of their step's
// motion.
//
// CONSERVATIVE ADVANCEMENT (Mirtich, 1996). Measure the gap between the two
// shapes where they are at time t (GJK, gjk.js), bound how fast it can close
// from there, and move t on by the gap over that rate: the shapes can't have
// met before it. Repeat until the gap is within the contact tolerance, or the
// step runs out. Each body follows the path the step would give it with
// nothing in the way: its velocity, gravity, and its spin, turned exactly.

import { closest, Closest, radius } from './gjk.js';
import { placePart } from './collide.js';
import { sin, cos } from './core/math.js';

const DYNAMIC = 0, SPHERE_SHAPE = 1;   // World's codes
/** World's SPIN, the most a body other than a sphere turns in a step: π. */
const SPIN = Math.PI;

/**
 * Rounds at most. A round that doesn't finish moves t on by the gap over the
 * fastest the gap can close, and the gap is over a pixel until the last; a
 * pair still unsure after 32 is a fast-spinning body skimming past another,
 * and keeps the contact found at the step's start, as before this existed.
 */
const ROUNDS = 32;

/** The answer for a pair that ran out of rounds: keep the step's own contact. */
export const UNSURE = -2;

const near = new Closest();
// Each body's path over the step, set by start(): position, velocity, the
// gravity it falls by, rotation, and spin. Module scratch: never re-entered.
const P = new Float64Array(6), V = new Float64Array(6), G = new Float64Array(6);
const Q = new Float64Array(8), WV = new Float64Array(6);
const R = new Float64Array(4);

/**
 * When slots a and b, as shapes sa and sb (placed where the step starts, as
 * contact.js loads them; parts partA and partB of them, or null for the
 * whole body), first come within `tolerance` of touching within `dt`: the
 * time, -1 if they don't, or UNSURE. Leaves sa and sb placed where they are
 * at the time it returns (or at the step's end).
 */
export function impact(world, a, b, sa, sb, dt, tolerance, partA, partB) {
  start(world, a, 0, dt);
  start(world, b, 3, dt);
  const ra = radius(sa), rb = radius(sb);
  const reach = world._reach;
  const spin = Math.sqrt(WV[0] * WV[0] + WV[1] * WV[1] + WV[2] * WV[2]) * reach[a]
    + Math.sqrt(WV[3] * WV[3] + WV[4] * WV[4] + WV[5] * WV[5]) * reach[b];
  const gx = G[3] - G[0], gy = G[4] - G[1], gz = G[5] - G[2], gl = Math.sqrt(gx * gx + gy * gy + gz * gz);
  let t = 0;
  for (let round = 0; round < ROUNDS; round++) {
    pose(sa, 0, t, partA);
    pose(sb, 3, t, partB);
    const d = closest(sa, sb, near), gap = d - ra - rb;
    if (gap <= tolerance) return t;
    // The gap closes no faster than the cores' closing speed along the line
    // between them, plus what gravity can add in the rest of the step, plus
    // the fastest any point of either can move by turning.
    const nx = (near.b[0] - near.a[0]) / d, ny = (near.b[1] - near.a[1]) / d, nz = (near.b[2] - near.a[2]) / d;
    const vx = V[3] - V[0] + gx * t, vy = V[4] - V[1] + gy * t, vz = V[5] - V[2] + gz * t;
    const rate = -(vx * nx + vy * ny + vz * nz) + gl * (dt - t) + spin;
    if (!(rate > 0)) return -1;   // parting, and nothing can turn them back within the step
    t += gap / rate;
    if (t >= dt) return -1;
  }
  return UNSURE;
}

/** Slot s's path over the step, into the scratch at o (0 for a, 3 for b). */
function start(world, s, o, dt) {
  const p = world._position, v = world._velocity, w = world._angular, q = world._rotation, s3 = 3 * s;
  const dynamic = world._type[s] === DYNAMIC, scale = dynamic ? world._gravityScale[s] : 0, g = world._gravity;
  for (let k = 0; k < 3; k++) {
    P[o + k] = p[s3 + k];
    V[o + k] = v[s3 + k];
    G[o + k] = g[k] * scale;
    WV[o + k] = w[s3 + k];
  }
  // Its spin as the step will turn it: capped as World._accelerate caps it.
  if (dynamic && (world._shape[s] !== SPHERE_SHAPE || world._framed[s] === 1)) {
    const most = SPIN / dt, w2 = WV[o] * WV[o] + WV[o + 1] * WV[o + 1] + WV[o + 2] * WV[o + 2];
    if (w2 > most * most) {
      const k = most / Math.sqrt(w2);
      WV[o] *= k; WV[o + 1] *= k; WV[o + 2] *= k;
    }
  }
  const q4 = o === 0 ? 0 : 4;
  for (let k = 0; k < 4; k++) Q[q4 + k] = q[4 * s + k];
}

/** Places shape s (or its body's part) where its body (o: 0 for a, 3 for b) is at time t on its path. */
function pose(s, o, t, part) {
  posed(o, t, C, R);
  if (part === null) s.place(C[0], C[1], C[2], R, 0);
  else placePart(part, C, 0, R, 0, s);
}
const C = new Float64Array(3);

/**
 * Where body a (o = 0) or b (o = 3) of the last impact() is at time t on its
 * path, its centre into c and its rotation into q: moved by its velocity and
 * gravity, p + v t + g t² / 2, and turned by its spin, ω t, an exact turn
 * about ω (Bearing's own sine and cosine, so the same everywhere).
 */
export function posed(o, t, c, q) {
  c[0] = P[o] + (V[o] + 0.5 * G[o] * t) * t;
  c[1] = P[o + 1] + (V[o + 1] + 0.5 * G[o + 1] * t) * t;
  c[2] = P[o + 2] + (V[o + 2] + 0.5 * G[o + 2] * t) * t;
  const q4 = o === 0 ? 0 : 4;
  const wx = WV[o], wy = WV[o + 1], wz = WV[o + 2], w = Math.sqrt(wx * wx + wy * wy + wz * wz);
  if (w === 0 || t === 0) {
    for (let k = 0; k < 4; k++) q[k] = Q[q4 + k];
    return;
  }
  const half = 0.5 * w * t, k = sin(half) / w;
  turned(wx * k, wy * k, wz * k, cos(half), Q, q4, q);
}

/** out = (x, y, z, w) ⊗ q[o..], normalized. */
function turned(x, y, z, w, q, o, out) {
  const bx = q[o], by = q[o + 1], bz = q[o + 2], bw = q[o + 3];
  const rx = w * bx + x * bw + y * bz - z * by;
  const ry = w * by + y * bw + z * bx - x * bz;
  const rz = w * bz + z * bw + x * by - y * bx;
  const rw = w * bw - x * bx - y * by - z * bz;
  const l = 1 / Math.sqrt(rx * rx + ry * ry + rz * rz + rw * rw);
  out[0] = rx * l; out[1] = ry * l; out[2] = rz * l; out[3] = rw * l;
}
