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
// The polynomial coefficients and the split of pi/2 are fdlibm's, as are
// atan and atan2 below:
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

// Arctangent, fdlibm's s_atan.c and e_atan2.c, for a joint's angles. The
// argument is brought to |t| < 7/16 about one of 0, 0.5, 1, 1.5 and infinity,
// whose arctangents are kept in two parts, then an odd minimax polynomial.
// fdlibm picks the interval by the high word of x; these thresholds' low
// words are zero, so comparing the values themselves picks the same one.
const ATAN_HI = [4.63647609000806093515e-01, 7.85398163397448278999e-01, 9.82793723247329054082e-01, 1.57079632679489655800e+00];
const ATAN_LO = [2.26987774529616870924e-17, 3.06161699786838301793e-17, 1.39033110312309984516e-17, 6.12323399573676603587e-17];
const AT0 = 3.33333333333329318027e-01, AT1 = -1.99999999998764832476e-01, AT2 = 1.42857142725034663711e-01;
const AT3 = -1.11111104054623557880e-01, AT4 = 9.09088713343650656196e-02, AT5 = -7.69187620504482999495e-02;
const AT6 = 6.66107313738753120669e-02, AT7 = -5.83357013379057348645e-02, AT8 = 4.97687799461593236017e-02;
const AT9 = -3.65315727442169155270e-02, AT10 = 1.62858201153657823623e-02;
const PI = 3.1415926535897931160e+00, PI_LO = 1.2246467991473531772e-16;
const PI_O_2 = 1.5707963267948965580e+00, PI_O_4 = 7.8539816339744827900e-01;
const TWO_66 = 7.378697629483821e+19, TWO_M27 = 7.450580596923828e-09;
const TWO_60 = 1.152921504606847e+18, TWO_M60 = 8.673617379884035e-19;

export function atan(x) {
  if (x !== x) return x;
  const ax = Math.abs(x);
  if (ax >= TWO_66) return x > 0 ? ATAN_HI[3] + ATAN_LO[3] : -ATAN_HI[3] - ATAN_LO[3];
  let id, t;
  if (ax < 0.4375) {
    if (ax < TWO_M27) return x;
    id = -1; t = x;
  } else if (ax < 0.6875) {
    id = 0; t = (2 * ax - 1) / (2 + ax);
  } else if (ax < 1.1875) {
    id = 1; t = (ax - 1) / (ax + 1);
  } else if (ax < 2.4375) {
    id = 2; t = (ax - 1.5) / (1 + 1.5 * ax);
  } else {
    id = 3; t = -1 / ax;
  }
  const z = t * t, w = z * z;
  const s1 = z * (AT0 + w * (AT2 + w * (AT4 + w * (AT6 + w * (AT8 + w * AT10)))));
  const s2 = w * (AT1 + w * (AT3 + w * (AT5 + w * (AT7 + w * AT9))));
  if (id < 0) return t - t * (s1 + s2);
  const r = ATAN_HI[id] - ((t * (s1 + s2) - ATAN_LO[id]) - t);
  return x < 0 ? -r : r;
}

/** Whether v's sign bit is set, -0 included. */
const negative = (v) => v < 0 || (v === 0 && 1 / v < 0);

/**
 * The angle of (x, y) from the x axis, from -π to π. fdlibm reads the
 * exponents' difference to see |y/x| past 2^60 or under 2^-60; comparing
 * the magnitudes, scaled by those powers of two exactly, is the same test
 * to within a factor of 2 in where it cuts over, and the answer either way
 * is right to the last bit there.
 */
export function atan2(y, x) {
  if (x !== x || y !== y) return x + y;
  if (x === 1) return atan(y);
  let m = (negative(y) ? 1 : 0) | (negative(x) ? 2 : 0);
  if (y === 0) return m < 2 ? y : m === 2 ? PI : -PI;
  if (x === 0) return y < 0 ? -PI_O_2 : PI_O_2;
  const ax = Math.abs(x), ay = Math.abs(y);
  if (ax === Infinity) {
    if (ay === Infinity) return [PI_O_4, -PI_O_4, 3 * PI_O_4, -3 * PI_O_4][m];
    return [0, -0, PI, -PI][m];
  }
  if (ay === Infinity) return y < 0 ? -PI_O_2 : PI_O_2;
  let z;
  if (ay > ax * TWO_60) {
    z = PI_O_2 + 0.5 * PI_LO;
    m &= 1;
  } else if (x < 0 && ay < ax * TWO_M60) {
    z = 0;
  } else {
    z = atan(ay / ax);
  }
  switch (m) {
    case 0: return z;
    case 1: return -z;
    case 2: return PI - (z - PI_LO);
    default: return (z - PI_LO) - PI;
  }
}

/**
 * Rotates (x, y, z) by the unit quaternion at q[o..o+3], into out[0..2]. With
 * `inverse`, by its conjugate: world to body instead of body to world. Takes
 * the quaternion by offset and writes into a caller's array, so the step can
 * use it on its columns without allocating.
 *
 * v' = v + w t + u × t, with u the quaternion's vector part and t = 2 u × v:
 * two cross products, cheaper than building the matrix.
 */
export function rotate(q, o, x, y, z, inverse, out) {
  const s = inverse ? -1 : 1;
  const ux = q[o] * s, uy = q[o + 1] * s, uz = q[o + 2] * s, w = q[o + 3];
  const tx = 2 * (uy * z - uz * y), ty = 2 * (uz * x - ux * z), tz = 2 * (ux * y - uy * x);
  out[0] = x + w * tx + (uy * tz - uz * ty);
  out[1] = y + w * ty + (uz * tx - ux * tz);
  out[2] = z + w * tz + (ux * ty - uy * tx);
  return out;
}

/** out[oo..] = a[ao..] ⊗ b[bo..], quaternions [x, y, z, w]: b's turn, then a's. out may not be a or b. */
export function quatMultiply(a, ao, b, bo, out, oo) {
  const ax = a[ao], ay = a[ao + 1], az = a[ao + 2], aw = a[ao + 3];
  const bx = b[bo], by = b[bo + 1], bz = b[bo + 2], bw = b[bo + 3];
  out[oo] = aw * bx + ax * bw + ay * bz - az * by;
  out[oo + 1] = aw * by + ay * bw + az * bx - ax * bz;
  out[oo + 2] = aw * bz + az * bw + ax * by - ay * bx;
  out[oo + 3] = aw * bw - ax * bx - ay * by - az * bz;
  return out;
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
