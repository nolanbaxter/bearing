// Trigonometry that gives the same bits on every machine.
//
// Bearing promises that the same inputs give bit-identical results on every
// CPU, browser and runtime. JavaScript makes most of that free: it forbids
// fused multiply-add and extended precision, so + - * / and Math.sqrt round
// the same everywhere. Math.sin and Math.cos are the exception. The spec calls
// them "implementation-approximated", and V8, SpiderMonkey and JavaScriptCore
// may differ in the last bit, which is enough for two machines' simulations
// to part ways. A rotation set with setAxisAngle on one machine has to be the
// same rotation, to the bit, on another.
//
// So these use only exactly-rounded operations. The method is fdlibm's: reduce
// x by the nearest multiple of pi/2, with pi/2 split into three 33-bit parts
// so each product with the multiple is exact, then evaluate fdlibm's minimax
// polynomials for sine and cosine on [-pi/4, pi/4]. They agree with a
// correctly rounded result to about one unit in the last place while
// |x| < 2^19 * pi/2 (823,550 radians), where the multiple times the first part
// is still exact. Past that they lose accuracy, but never determinism.
//
// The polynomial coefficients and the split of pi/2 are fdlibm's:
//   Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
//   Developed at SunSoft, a Sun Microsystems, Inc. business.
//   Permission to use, copy, modify, and distribute this software is freely
//   granted, provided that this notice is preserved.

const INV_PIO2 = 6.36619772367581382433e-01;   // 2 / pi
const PIO2_1 = 1.57079632673412561417e+00;     // first 33 bits of pi/2
const PIO2_1T = 6.07710050650619224932e-11;    // pi/2 - PIO2_1
const PIO2_2 = 6.07710050630396597660e-11;     // second 33 bits
const PIO2_2T = 2.02226624879595063154e-21;    // pi/2 - (PIO2_1 + PIO2_2)
const PIO2_3 = 2.02226624871116645580e-21;     // third 33 bits
const PIO2_3T = 8.47842766036889956997e-32;    // pi/2 - (PIO2_1 + PIO2_2 + PIO2_3)

// When to take the second and third rounds. fdlibm takes them when the first
// result's exponent is more than 16, then 49, below x's: that many bits
// cancelled, and the next 33 bits of pi/2 are needed. Comparing magnitudes is
// the same test without reading exponent bits, to within a factor of 2 in
// where it cuts over. The second round is set to cut over early, which only
// adds accuracy: it carries its own rounding error forward. The third is set
// late, because it drops the second's rounding error, which is zero only once
// the cancellation is that deep; skipping it at a gap of exactly 50 bits still
// leaves 118 - 50 = 68 good bits, more than a double holds.
const ROUND_2 = 3.0517578125e-05;       // 2^-15
const ROUND_3 = 8.881784197001252e-16;  // 2^-50

const S1 = -1.66666666666666324348e-01, S2 = 8.33333333332248946124e-03;
const S3 = -1.98412698298579493134e-04, S4 = 2.75573137070700676789e-06;
const S5 = -2.50507602534068634195e-08, S6 = 1.58969099521155010221e-10;

const C1 = 4.16666666666666019037e-02, C2 = -1.38888888888741095749e-03;
const C3 = 2.48015872894767294178e-05, C4 = -2.75573143513906633035e-07;
const C5 = 2.08757232129817482790e-09, C6 = -1.13596475577881948265e-11;

// The reduced argument, as a head and a tail: x - n pi/2 = head + tail.
// Module scratch: nothing here is re-entered.
let head = 0, tail = 0;

/** Sets head and tail, and returns n, the multiple of pi/2 taken off. */
function reduce(x) {
  const n = Math.round(x * INV_PIO2);
  // fdlibm's __ieee754_rem_pio2, medium case: each round takes off the next
  // 33 bits of n pi/2 (n times 33 bits is exact for |n| < 2^20) and keeps what
  // the subtraction rounded away in w.
  let r = x - n * PIO2_1;
  let w = n * PIO2_1T;
  head = r - w;
  const limit = Math.abs(x);
  if (Math.abs(head) < limit * ROUND_2) {
    let t = r;
    w = n * PIO2_2;
    r = t - w;
    w = n * PIO2_2T - ((t - r) - w);
    head = r - w;
    if (Math.abs(head) < limit * ROUND_3) {
      t = r;
      w = n * PIO2_3;
      r = t - w;
      w = n * PIO2_3T - ((t - r) - w);
      head = r - w;
    }
  }
  tail = (r - head) - w;
  return n;
}

/** sin(x + y) for |x + y| <= pi/4, y the tail of x. */
function kernelSin(x, y) {
  const z = x * x, v = z * x;
  const r = S2 + z * (S3 + z * (S4 + z * (S5 + z * S6)));
  return x - ((z * (0.5 * y - v * r) - y) - v * S1);
}

/** cos(x + y) for |x + y| <= pi/4. */
function kernelCos(x, y) {
  const z = x * x;
  const r = z * (C1 + z * (C2 + z * (C3 + z * (C4 + z * (C5 + z * C6)))));
  const hz = 0.5 * z, w = 1 - hz;
  return w + (((1 - w) - hz) + (z * r - x * y));
}

export function sin(x) {
  const n = reduce(x);
  switch (n & 3) {
    case 0: return kernelSin(head, tail);
    case 1: return kernelCos(head, tail);
    case 2: return -kernelSin(head, tail);
    default: return -kernelCos(head, tail);
  }
}

export function cos(x) {
  const n = reduce(x);
  switch (n & 3) {
    case 0: return kernelCos(head, tail);
    case 1: return -kernelSin(head, tail);
    case 2: return -kernelCos(head, tail);
    default: return kernelSin(head, tail);
  }
}

/** The rotation of `radians` about a unit `axis`, as [x, y, z, w]. */
export function quatAxisAngle(axis, radians) {
  const half = radians * 0.5, s = sin(half);
  return [axis[0] * s, axis[1] * s, axis[2] * s, cos(half)];
}

/**
 * Yaw about Y, then pitch about X, then roll about Z (YXZ), in radians: the
 * same order and the same formula as Winding's quatFromEuler, so one set of
 * angles means one rotation in both.
 */
export function quatEuler(yaw, pitch, roll) {
  const cy = cos(yaw * 0.5), sy = sin(yaw * 0.5);
  const cx = cos(pitch * 0.5), sx = sin(pitch * 0.5);
  const cz = cos(roll * 0.5), sz = sin(roll * 0.5);
  const x = cy * sx * cz + sy * cx * sz;
  const y = sy * cx * cz - cy * sx * sz;
  const z = cy * cx * sz - sy * sx * cz;
  const w = cy * cx * cz + sy * sx * sz;
  // Unit to rounding already; normalizing keeps the integrator's input exact.
  const s = 1 / Math.sqrt(x * x + y * y + z * z + w * w);
  return [x * s, y * s, z * s, w * s];
}
