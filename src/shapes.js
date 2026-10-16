// What a shape weighs and how it turns: volume, and the moments of inertia
// about its own axes.
//
// A box, ball or capsule is symmetric about its own x, y and z axes, with its
// centre of mass at its centre, so its inertia tensor is diagonal in its own
// frame: three numbers, the principal moments, rather than a 3x3 matrix. A
// hull or a compound isn't. Its centre of mass is wherever its shape puts it,
// and its tensor is full, so hullMass() and compoundMass() give the whole
// tensor about the centre of mass, and principal() finds the axes it is
// diagonal along. World turns the body's own frame to those axes, so the
// solver still sees three numbers.

/** m³. A mesh or heightfield is a surface, and encloses none. */
export function volume(spec) {
  if (spec.shape === 'mesh' || spec.shape === 'heightfield') return 0;
  const r = spec.radius;
  if (spec.shape === 'box') return spec.size[0] * spec.size[1] * spec.size[2];
  const ball = (4 / 3) * Math.PI * r * r * r;
  if (spec.shape === 'sphere') return ball;
  return Math.PI * r * r * (spec.height - 2 * r) + ball;
}

/**
 * The principal moments, kg·m², of a solid shape of mass `mass`, about its
 * own x, y and z axes through its centre. Into `out`.
 *
 * A capsule is a cylinder of length L = height - 2r along y, capped by two
 * hemispheres, its mass shared between them by volume. About y, the cylinder
 * gives m r²/2 and the two caps together a sphere's 2/5 m r². About x (and z,
 * the same by symmetry) the cylinder gives m (L²/12 + r²/4), and each cap is a
 * hemisphere whose own centre of mass sits 3r/8 from its flat face: moved by
 * the parallel-axis theorem out to L/2 + 3r/8 from the capsule's centre, the
 * two give m (2r²/5 + L²/4 + 3Lr/8). test/motion.test.js checks this by
 * slicing the capsule into discs and integrating, so it isn't the formula
 * checking itself.
 */
export function inertia(spec, mass, out) {
  if (spec.shape === 'box') {
    const [w, h, d] = spec.size;
    out[0] = mass * (h * h + d * d) / 12;
    out[1] = mass * (w * w + d * d) / 12;
    out[2] = mass * (w * w + h * h) / 12;
  } else if (spec.shape === 'sphere') {
    const r = spec.radius;
    out[0] = out[1] = out[2] = 0.4 * mass * r * r;
  } else if (spec.shape === 'mesh' || spec.shape === 'heightfield') {
    out[0] = out[1] = out[2] = 0;
  } else {
    const r = spec.radius, L = spec.height - 2 * r;
    const cylinder = Math.PI * r * r * L, ball = (4 / 3) * Math.PI * r * r * r;
    const mc = mass * cylinder / (cylinder + ball), ms = mass * ball / (cylinder + ball);
    out[1] = mc * r * r / 2 + ms * 0.4 * r * r;
    out[0] = out[2] = mc * (L * L / 12 + r * r / 4) + ms * (0.4 * r * r + L * L / 4 + 3 * L * r / 8);
  }
  return out;
}

// ---- hulls, compounds and principal axes ----
//
// A tensor here is six numbers, the symmetric 3x3 matrix's xx xy xz yy yz zz.

/**
 * A hull's volume, centre of mass and inertia tensor about it, for a density
 * of 1. The hull is cut into tetrahedra, one from a point inside to each
 * triangle of each face, and summed exactly. A tetrahedron with one corner at
 * the origin and the others at a, b and c (det = a·(b × c), six times its
 * volume) has ∫ x xᵀ dV = det / 120 · (a aᵀ + b bᵀ + c cᵀ + s sᵀ), s = a + b + c
 * (Tonon, 2004); from that "covariance" C about a point, the inertia is
 * tr(C) I - C, moved to the centre of mass by taking off V c cᵀ.
 */
export function hullMass(hull) {
  const v = hull.vertices, V = v.length / 3;
  // From the mean of the corners, so the sums stay small however far from
  // the origin the hull is given.
  let rx = 0, ry = 0, rz = 0;
  for (let i = 0; i < V; i++) { rx += v[3 * i]; ry += v[3 * i + 1]; rz += v[3 * i + 2]; }
  rx /= V; ry /= V; rz /= V;
  let vol = 0, cx = 0, cy = 0, cz = 0;
  const C = [0, 0, 0, 0, 0, 0];
  const { start, loops } = hull;
  for (let f = 0; f + 1 < start.length; f++) {
    const o = loops[start[f]];
    for (let k = start[f] + 1; k + 1 < start[f + 1]; k++) {
      const i = loops[k], j = loops[k + 1];
      const ax = v[3 * o] - rx, ay = v[3 * o + 1] - ry, az = v[3 * o + 2] - rz;
      const bx = v[3 * i] - rx, by = v[3 * i + 1] - ry, bz = v[3 * i + 2] - rz;
      const qx = v[3 * j] - rx, qy = v[3 * j + 1] - ry, qz = v[3 * j + 2] - rz;
      const det = ax * (by * qz - bz * qy) + ay * (bz * qx - bx * qz) + az * (bx * qy - by * qx);
      vol += det / 6;
      const sx = ax + bx + qx, sy = ay + by + qy, sz = az + bz + qz;
      cx += det / 24 * sx; cy += det / 24 * sy; cz += det / 24 * sz;
      const k6 = det / 120;
      C[0] += k6 * (ax * ax + bx * bx + qx * qx + sx * sx);
      C[1] += k6 * (ax * ay + bx * by + qx * qy + sx * sy);
      C[2] += k6 * (ax * az + bx * bz + qx * qz + sx * sz);
      C[3] += k6 * (ay * ay + by * by + qy * qy + sy * sy);
      C[4] += k6 * (ay * az + by * bz + qy * qz + sy * sz);
      C[5] += k6 * (az * az + bz * bz + qz * qz + sz * sz);
    }
  }
  cx /= vol; cy /= vol; cz /= vol;
  // About the centre of mass.
  C[0] -= vol * cx * cx; C[1] -= vol * cx * cy; C[2] -= vol * cx * cz;
  C[3] -= vol * cy * cy; C[4] -= vol * cy * cz; C[5] -= vol * cz * cz;
  return { volume: vol, center: [rx + cx, ry + cy, rz + cz], tensor: inertiaOf(C) };
}

/** The inertia tensor from a covariance C: tr(C) I - C. */
function inertiaOf(C) {
  const t = C[0] + C[3] + C[5];
  return [t - C[0], -C[1], -C[2], t - C[3], -C[4], t - C[5]];
}

/**
 * Tensor T about a centre of mass, moved to a point `d` from it, for mass m:
 * the parallel-axis theorem, T + m (|d|² I - d dᵀ).
 */
export function shifted(T, m, d) {
  const dd = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
  return [
    T[0] + m * (dd - d[0] * d[0]), T[1] - m * d[0] * d[1], T[2] - m * d[0] * d[2],
    T[3] + m * (dd - d[1] * d[1]), T[4] - m * d[1] * d[2], T[5] + m * (dd - d[2] * d[2]),
  ];
}

/** R T Rᵀ, for R a rotation as a matrix (9 numbers, row by row). */
export function turnedTensor(T, R) {
  const M = [T[0], T[1], T[2], T[1], T[3], T[4], T[2], T[4], T[5]];
  const RM = new Array(9).fill(0), out = new Array(9).fill(0);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) for (let k = 0; k < 3; k++) RM[3 * i + j] += R[3 * i + k] * M[3 * k + j];
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) for (let k = 0; k < 3; k++) out[3 * i + j] += RM[3 * i + k] * R[3 * j + k];
  return [out[0], out[1], out[2], out[4], out[5], out[8]];
}

/** The rotation matrix (9 numbers, row by row) of unit quaternion [x, y, z, w]. */
export function matrixOf(q) {
  const [x, y, z, w] = q;
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
  ];
}

/**
 * The principal moments of tensor T, and the unit quaternion q whose
 * rotation's columns are the axes they are about: T = R diag(moments) Rᵀ.
 *
 * Jacobi's method: turn the tensor about one axis at a time by the angle that
 * clears one off-diagonal entry, until none is left. The angle's tangent comes
 * from a square root (Numerical Recipes, 11.1), never an arctangent, so the
 * axes are the same on every machine. An entry is cleared once it is under a
 * double's rounding of the two moments it joins: past that, a turn is the
 * rounding deciding, and a cube's equal moments would be turned 45° on noise.
 * Each sweep clears all three; for a 3x3 tensor, a handful settle it.
 */
export function principal(T) {
  const A = [[T[0], T[1], T[2]], [T[1], T[3], T[4]], [T[2], T[4], T[5]]];
  const R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  for (let sweep = 0; sweep < 64; sweep++) {
    let turned = false;
    for (const [p, q] of [[0, 1], [0, 2], [1, 2]]) {
      const apq = A[p][q];
      if (!(Math.abs(apq) > EPSILON * (Math.abs(A[p][p]) + Math.abs(A[q][q])))) { A[p][q] = A[q][p] = 0; continue; }
      turned = true;
      const theta = (A[q][q] - A[p][p]) / (2 * apq);
      const t = (theta >= 0 ? 1 : -1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
      const c = 1 / Math.sqrt(t * t + 1), s = t * c;
      for (let k = 0; k < 3; k++) {
        const akp = A[k][p], akq = A[k][q];
        A[k][p] = c * akp - s * akq; A[k][q] = s * akp + c * akq;
      }
      for (let k = 0; k < 3; k++) {
        const apk = A[p][k], aqk = A[q][k];
        A[p][k] = c * apk - s * aqk; A[q][k] = s * apk + c * aqk;
      }
      for (let k = 0; k < 3; k++) {
        const rkp = R[k][p], rkq = R[k][q];
        R[k][p] = c * rkp - s * rkq; R[k][q] = s * rkp + c * rkq;
      }
    }
    if (!turned) break;
  }
  return { moments: [A[0][0], A[1][1], A[2][2]], rotation: quaternionOf(R) };
}

/** A double's rounding, 2^-52. */
const EPSILON = 1 / 4503599627370496;

/**
 * The unit quaternion of rotation matrix R (rows), by Shepperd's method: from
 * whichever of w, x, y, z is largest, so no square root is of a number near
 * zero. Jacobi's turns are rotations, so R's determinant is +1.
 */
function quaternionOf(R) {
  const t = R[0][0] + R[1][1] + R[2][2];
  let x, y, z, w;
  if (t >= R[0][0] && t >= R[1][1] && t >= R[2][2]) {
    const s = 2 * Math.sqrt(1 + t);
    w = s / 4; x = (R[2][1] - R[1][2]) / s; y = (R[0][2] - R[2][0]) / s; z = (R[1][0] - R[0][1]) / s;
  } else if (R[0][0] >= R[1][1] && R[0][0] >= R[2][2]) {
    const s = 2 * Math.sqrt(1 + R[0][0] - R[1][1] - R[2][2]);
    x = s / 4; w = (R[2][1] - R[1][2]) / s; y = (R[0][1] + R[1][0]) / s; z = (R[0][2] + R[2][0]) / s;
  } else if (R[1][1] >= R[2][2]) {
    const s = 2 * Math.sqrt(1 + R[1][1] - R[0][0] - R[2][2]);
    y = s / 4; w = (R[0][2] - R[2][0]) / s; x = (R[0][1] + R[1][0]) / s; z = (R[1][2] + R[2][1]) / s;
  } else {
    const s = 2 * Math.sqrt(1 + R[2][2] - R[0][0] - R[1][1]);
    z = s / 4; w = (R[1][0] - R[0][1]) / s; x = (R[0][2] + R[2][0]) / s; y = (R[1][2] + R[2][1]) / s;
  }
  const l = Math.sqrt(x * x + y * y + z * z + w * w);
  return [x / l, y / l, z / l, w / l];
}
