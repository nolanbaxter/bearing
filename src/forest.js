// The forest: joints that join bodies into trees, solved exactly, all at once.
//
// WHY. Solved one joint at a time, a chain is slow to pull tight: each pass
// carries the weight of the links below up only one joint, so a 100-link
// chain falls and stretches before its joints have the load between them, and
// bounces on them like a bungee cord. Measured with joints.js's rows solved
// that way: 2% longer at once and still bobbing at 30 s, and correcting its
// position harder only made it blow up. Soft joints, Box2D 3's answer, hold
// it, but sag under load: 1.8% for a 10 m chain of 100 links, 18% for a 1 m
// one, since the sag is the same length at any scale.
//
// But the joints of a chain, a ragdoll or a bridge of planks form a tree:
// remove any one and the bodies fall into two groups. A tree's joints can be
// solved exactly, together, in time proportional to their number (Baraff,
// "Linear-Time Dynamics using Lagrange Multipliers", 1996): the equations
// for every body's motion and every joint's rows form one matrix, and taken
// leaves first, eliminating a leaf leaves the rest of the tree as it was, so
// the work is a small dense block per body and per joint. Each pass then
// holds every joint in the tree exactly, however long the chain and however
// heavy the weight on it.
//
// Static, kinematic and character bodies, and points in the world, are not
// in the tree, since nothing moves them: a joint to one is a leaf. So a
// bridge fixed at both ends is a tree too, a path between its two ends. A
// joint that closes a loop among moving bodies (a ladder's rungs, a four-bar
// linkage) is solved one at a time as before, as are the rows that may give
// (limits, motors, springs).
//
// THE MATRIX. For bodies with inverse mass matrices W (6x6: its inverse mass
// on the diagonal for moving, its world inverse inertia for turning) and
// joints with row Jacobians J, the velocity changes u and the joints'
// impulses λ satisfy
//
//   M u - Jᵀ λ = 0     (a body moves by the impulses on it)
//   J u = r            (each row changes speed by what it asks, r)
//
// a symmetric matrix with a block per body and per joint, nonzero off the
// diagonal only where a joint holds a body: the tree itself. Its LDLᵀ
// factor, children before parents, has no blocks the tree doesn't. A body's
// block is kept as its inverse, worked out from W, so a body that can't turn
// about some axis (lockRotation), whose M is infinite there, needs no M.
//
// GIVE. A joint's block is what its child bodies let it move. Where they
// can't move along some row at all (the end of a bridge, held to the world;
// a body that can't turn), that block has nothing on that row, and leaves
// first can't divide by it, though its parent could still move that way. So
// every joint gives by COMPLIANCE of its own effective inverse mass: its
// block is never empty, a row its children can't move is held by its parent
// (as a stiff spring a million times the joint's own), and the speed it lets
// through is a millionth of its push. Rows that repeat one another (a door on
// two hinges) share the push between them, rather than dividing by nothing.

import { resized } from './core/slots.js';

const DYNAMIC = 0;

/**
 * How much each joint gives, as a fraction of its own effective inverse mass
 * (the sum of its rows' couplings with themselves, over its rows). 2^-20,
 * about a millionth: the speed a joint lets through is that fraction of what
 * its push would make it, too small to see, and the stiffest block it adds,
 * 2^20 times the joint's own, still leaves 30 of a double's 53 bits.
 * Measured against a dense solve: within 2^-20 of exact where the exact
 * solution exists.
 */
const COMPLIANCE = 9.5367431640625e-7;   // 2^-20

/** Numbers per node in Forest.vector: 6 for a body, up to 6 for a joint. */
export const NODE = 6;

/** Storage for one step's forest, per world, sized to its joints and shrinking with them. */
export class Forest {
  constructor() {
    this.capacity = 0;
    this.count = 0;        // joints in the forest this step
    this.bodies = 0;       // bodies in it
    this.nodes = 0;
    this._alloc(0);
  }

  _alloc(n) {
    const grow = (name, Type, per) => { this[name] = this[name] ? resized(this[name], n * per) : new Type(n * per); };
    // Per forest joint t:
    grow('joint', Int32Array, 1);      // its slot
    grow('rows', Uint8Array, 1);       // how many rows it holds, 1 to 6
    grow('row', Float64Array, 54);     // up to 6 rows of 9 (see joints.js)
    grow('rhs', Float64Array, 6);      // the speed change each row asks for
    grow('scale', Float64Array, 1);    // the sum of its rows' couplings with themselves
    grow('side', Int32Array, 2);       // its body nodes, a's and b's; -1 for one that doesn't move
    grow('jointInverse', Float64Array, 36);  // its block's inverse, m x m
    grow('jac', Float64Array, 72);     // its Jacobian at a, then at b: m rows of 6 each (see shape())
    grow('inTree', Uint8Array, 1);     // by joint slot: 1 while the joint is in the forest this step
    // Per body node k, at most two per joint:
    grow('body', Int32Array, 2);       // its slot
    grow('inverse', Float64Array, 2 * 36);   // its block's inverse, 6x6
    grow('first', Int32Array, 2);      // its first forest joint, as an edge 2t + side, plus 1
    grow('next', Int32Array, 2);       // per edge (2t + side), the next at the same body, plus 1
    grow('union', Int32Array, 2);      // union-find while trees are found
    // The elimination: every node (joints 0..count-1, bodies count..), parents before children.
    grow('order', Int32Array, 3);
    grow('parent', Int32Array, 3);
    grow('vector', Float64Array, 3 * NODE);   // per node, the solve's working values
    this.capacity = n;
  }

  /** Room for n joints; shrinks by the world's quarter rule when the joints in use fall that low. */
  fit(n) {
    if (n > this.capacity) {
      let c = Math.max(this.capacity, 1);
      while (c < n) c *= 2;
      this._alloc(c);
    } else if (n < this.capacity && n <= this.capacity / 4) {
      let c = n === 0 ? 0 : 1;
      while (c < n) c *= 2;
      this._alloc(c);
    }
  }
}

/**
 * Finds this step's trees among the joints `holds` says have rows that must
 * hold, in list order, so which joints close loops depends only on the
 * inputs. world._node is a per-body column, the body's node plus 1, left
 * zeroed again.
 */
export function plant(world, list, n, holds) {
  const F = world._joints.forest, J = world._joints, type = world._type, node = world._node;
  F.fit(J.slots.capacity);
  F.count = 0;
  F.bodies = 0;
  F.inTree.fill(0);
  for (let i = 0; i < n; i++) {
    const j = list[i];
    if (!holds(j)) continue;
    const a = J.a[j], b = J.b[j];
    const ka = type[a] === DYNAMIC ? take(F, node, a) : -1;
    const kb = b >= 0 && type[b] === DYNAMIC ? take(F, node, b) : -1;
    if (ka >= 0 && kb >= 0) {
      const ra = find(F.union, ka), rb = find(F.union, kb);
      if (ra === rb) continue;   // a loop: solved one at a time
      F.union[rb] = ra;
    }
    const t = F.count++;
    F.joint[t] = j;
    F.inTree[j] = 1;
    F.side[2 * t] = ka;
    F.side[2 * t + 1] = kb;
    if (ka >= 0) { F.next[2 * t] = F.first[ka]; F.first[ka] = 2 * t + 1; }
    if (kb >= 0) { F.next[2 * t + 1] = F.first[kb]; F.first[kb] = 2 * t + 2; }
  }
  // Each tree, breadth first from its first body, so every node comes after
  // its parent; eliminated in reverse, children come first.
  const T = F.count, seen = F.vector;   // free until the solve: a mark per node
  seen.fill(0, 0, T + F.bodies);
  let length = 0;
  for (let k = 0; k < F.bodies; k++) {
    if (seen[T + k] === 0) length = branch(F, T + k, length, seen);
  }
  F.nodes = length;
  for (let k = 0; k < F.bodies; k++) node[F.body[k]] = 0;
  return T;
}

/** Body slot s's node, made if it has none. */
function take(F, node, s) {
  if (node[s] !== 0) return node[s] - 1;
  const k = F.bodies++;
  node[s] = k + 1;
  F.body[k] = s;
  F.first[k] = 0;
  F.union[k] = k;
  return k;
}

function find(union, k) {
  while (union[k] !== k) {
    union[k] = union[union[k]];
    k = union[k];
  }
  return k;
}

/** Breadth first from body node `root`, appending to the order. */
function branch(F, root, length, seen) {
  const T = F.count, order = F.order, parent = F.parent;
  let head = length;
  order[length++] = root;
  parent[root] = -1;
  seen[root] = 1;
  while (head < length) {
    const x = order[head++];
    if (x < T) {
      for (let side = 0; side < 2; side++) {
        const k = F.side[2 * x + side];
        if (k < 0 || seen[T + k] !== 0) continue;
        seen[T + k] = 1;
        parent[T + k] = x;
        order[length++] = T + k;
      }
    } else {
      for (let e = F.first[x - T] - 1; e >= 0; e = F.next[e] - 1) {
        const t = e >> 1;
        if (seen[t] !== 0) continue;
        seen[t] = 1;
        parent[t] = x;
        order[length++] = t;
      }
    }
  }
  return length;
}

// Scratch for the solve: none of it is re-entered.
const W = new Float64Array(36), S = new Float64Array(36), E = new Float64Array(36), X = new Float64Array(36);
const L = new Float64Array(36), Y = new Float64Array(6), PIVOT = new Uint8Array(6);

/**
 * Factors the forest's matrix from its joints' rows at F.row: each node's
 * block's inverse, children first. Once a substep (see joints.js).
 *
 * A body's block D = M - S (S = Σ Jᵀ D⁻¹ J over its child joints) is
 * symmetric and positive definite, so it is inverted through its Cholesky
 * factor, a sixth of the work of general elimination; a joint's, -D, the
 * same. A body that can't turn about some axis has no finite M there, and
 * its block is kept as (I - W S)⁻¹ W instead, from its inverse mass matrix W.
 */
export function factorForest(world) {
  const F = world._joints.forest, T = F.count, n = F.nodes, order = F.order, parent = F.parent;
  const invMass = world._invMass, invI = world._invI, lock = world._lock;
  for (let t = 0; t < T; t++) shape(F, t);
  for (let i = n - 1; i >= 0; i--) {
    const x = order[i];
    if (x >= T) {
      const k = x - T, s = F.body[k], out = F.inverse, o = 36 * k;
      S.fill(0);
      let any = false;
      for (let e = F.first[k] - 1; e >= 0; e = F.next[e] - 1) {
        const t = e >> 1;
        if (parent[t] !== x) continue;
        any = true;
        // S += Jᵀ D⁻¹ J, J this joint's rows at this body.
        const m = F.rows[t], jo = 72 * t + 36 * (e & 1), D = F.jointInverse, d = 36 * t, jac = F.jac;
        for (let r = 0; r < m; r++) {
          for (let c = 0; c < 6; c++) {
            let sum = 0;
            for (let q = 0; q < m; q++) sum += D[d + r * m + q] * jac[jo + q * 6 + c];
            X[r * 6 + c] = sum;
          }
        }
        for (let r = 0; r < 6; r++) {
          for (let c = r; c < 6; c++) {
            let sum = 0;
            for (let q = 0; q < m; q++) sum += jac[jo + q * 6 + r] * X[q * 6 + c];
            S[r * 6 + c] += sum;
          }
        }
      }
      for (let r = 1; r < 6; r++) for (let c = 0; c < r; c++) S[r * 6 + c] = S[c * 6 + r];
      if (!any) {
        inverseMass(invMass[s], invI, 6 * s, W);
        out.set(W, o);
        continue;
      }
      if (lock[s] === 0) {
        mass(world, s, E);
        for (let q = 0; q < 36; q++) E[q] -= S[q];
        if (inverseSPD(E, 6, X)) {
          out.set(X, o);
          continue;
        }
      }
      inverseMass(invMass[s], invI, 6 * s, W);
      product(W, 0, 6, 6, S, 0, 6, X);
      for (let q = 0; q < 36; q++) E[q] = (q % 7 === 0 ? 1 : 0) - X[q];
      invert(E, 6, X);
      product(X, 0, 6, 6, W, 0, 6, S);
      out.set(S, o);
    } else {
      // A joint's block: D = -(give + Σ J D⁻¹ Jᵀ) over its child bodies.
      const t = x, m = F.rows[t], give = COMPLIANCE * F.scale[t] / m, jac = F.jac;
      E.fill(0, 0, m * m);
      for (let r = 0; r < m; r++) E[r * m + r] = give;
      for (let side = 0; side < 2; side++) {
        const k = F.side[2 * t + side];
        if (k < 0 || parent[T + k] !== t) continue;
        const jo = 72 * t + 36 * side, inv = F.inverse, io = 36 * k;
        for (let r = 0; r < m; r++) {
          for (let c = 0; c < 6; c++) {
            let sum = 0;
            for (let q = 0; q < 6; q++) sum += jac[jo + r * 6 + q] * inv[io + q * 6 + c];
            X[r * 6 + c] = sum;
          }
        }
        for (let r = 0; r < m; r++) {
          for (let c = r; c < m; c++) {
            let sum = 0;
            for (let q = 0; q < 6; q++) sum += X[r * 6 + q] * jac[jo + c * 6 + q];
            E[r * m + c] += sum;
          }
        }
      }
      for (let r = 1; r < m; r++) for (let c = 0; c < r; c++) E[r * m + c] = E[c * m + r];
      if (!inverseSPD(E, m, X)) invert(E, m, X);
      const D = F.jointInverse, d = 36 * t;
      for (let q = 0; q < m * m; q++) D[d + q] = -X[q];
    }
  }
}

/** Joint t's Jacobian at each side, m rows of 6 (linear, then angular), into F.jac. A row (n, cA, cB) acts as -n, -cA on a and n, cB on b. */
function shape(F, t) {
  const row = F.row, jac = F.jac, o = 54 * t, ja = 72 * t, jb = ja + 36;
  for (let r = 0; r < F.rows[t]; r++) {
    const q = o + 9 * r, a = ja + 6 * r, b = jb + 6 * r;
    jac[a] = -row[q]; jac[a + 1] = -row[q + 1]; jac[a + 2] = -row[q + 2];
    jac[a + 3] = -row[q + 3]; jac[a + 4] = -row[q + 4]; jac[a + 5] = -row[q + 5];
    jac[b] = row[q]; jac[b + 1] = row[q + 1]; jac[b + 2] = row[q + 2];
    jac[b + 3] = row[q + 6]; jac[b + 4] = row[q + 7]; jac[b + 5] = row[q + 8];
  }
}

/** Body slot s's mass matrix, m on the diagonal for moving and its world inertia R diag(I) Rᵀ for turning, into out. */
function mass(world, s, out) {
  out.fill(0);
  const m = 1 / world._invMass[s];
  out[0] = m; out[7] = m; out[14] = m;
  const ax = world._axes, a9 = 9 * s, I = world._inertia, i0 = I[3 * s], i1 = I[3 * s + 1], i2 = I[3 * s + 2];
  for (let r = 0; r < 3; r++) {
    for (let c = r; c < 3; c++) {
      const v = ax[a9 + r] * ax[a9 + c] * i0 + ax[a9 + 3 + r] * ax[a9 + 3 + c] * i1 + ax[a9 + 6 + r] * ax[a9 + 6 + c] * i2;
      out[21 + r * 6 + c] = v;
      out[21 + c * 6 + r] = v;
    }
  }
}

/**
 * The inverse of the symmetric positive definite n x n matrix A (row-major,
 * overwritten) into out, through its Cholesky factor: A = L Lᵀ, A⁻¹ = L⁻ᵀ L⁻¹.
 * false if A turns out not to be positive definite, for the caller to fall
 * back on general elimination.
 */
export function inverseSPD(A, n, out) {
  for (let c = 0; c < n; c++) {
    let d = A[c * n + c];
    for (let q = 0; q < c; q++) d -= L[c * n + q] * L[c * n + q];
    if (!(d > 0)) return false;
    const l = Math.sqrt(d);
    L[c * n + c] = l;
    for (let r = c + 1; r < n; r++) {
      let sum = A[r * n + c];
      for (let q = 0; q < c; q++) sum -= L[r * n + q] * L[c * n + q];
      L[r * n + c] = sum / l;
    }
  }
  // L⁻¹, lower triangular, into A's lower half.
  for (let c = 0; c < n; c++) {
    A[c * n + c] = 1 / L[c * n + c];
    for (let r = c + 1; r < n; r++) {
      let sum = 0;
      for (let q = c; q < r; q++) sum -= L[r * n + q] * A[q * n + c];
      A[r * n + c] = sum / L[r * n + r];
    }
  }
  // A⁻¹ = (L⁻¹)ᵀ L⁻¹: entry (r, c) = Σ_{q ≥ max(r, c)} L⁻¹[q][r] L⁻¹[q][c].
  for (let r = 0; r < n; r++) {
    for (let c = r; c < n; c++) {
      let sum = 0;
      for (let q = c; q < n; q++) sum += A[q * n + r] * A[q * n + c];
      out[r * n + c] = sum;
      out[c * n + r] = sum;
    }
  }
  return true;
}

/**
 * Solves the factored forest for the targets at F.rhs: every forest joint's
 * rows held exactly together. Leaves each body's velocity change, and each
 * joint's impulses, at its node in F.vector, for the caller to apply.
 */
export function solveForest(world) {
  const F = world._joints.forest, T = F.count, n = F.nodes, order = F.order, parent = F.parent, vec = F.vector;
  const jac = F.jac, inv = F.inverse, D = F.jointInverse;
  // Forward, children first: w = b - Σ H D⁻¹ w over the children. A body's
  // b is 0, a joint's -rhs, and H between them -J (or -Jᵀ).
  for (let i = n - 1; i >= 0; i--) {
    const x = order[i], v = NODE * x;
    if (x >= T) {
      const k = x - T;
      vec.fill(0, v, v + 6);
      for (let e = F.first[k] - 1; e >= 0; e = F.next[e] - 1) {
        const t = e >> 1;
        if (parent[t] !== x) continue;
        // w_k += Jᵀ (D_t⁻¹ w_t)
        const m = F.rows[t], jo = 72 * t + 36 * (e & 1), d = 36 * t, w = NODE * t;
        for (let r = 0; r < m; r++) {
          let sum = 0;
          for (let q = 0; q < m; q++) sum += D[d + r * m + q] * vec[w + q];
          Y[r] = sum;
        }
        for (let c = 0; c < 6; c++) {
          let sum = 0;
          for (let r = 0; r < m; r++) sum += jac[jo + r * 6 + c] * Y[r];
          vec[v + c] += sum;
        }
      }
    } else {
      const t = x, m = F.rows[t];
      for (let r = 0; r < m; r++) vec[v + r] = -F.rhs[6 * t + r];
      for (let side = 0; side < 2; side++) {
        const k = F.side[2 * t + side];
        if (k < 0 || parent[T + k] !== t) continue;
        // w_t += J (D_k⁻¹ w_k)
        const jo = 72 * t + 36 * side, io = 36 * k, w = NODE * (T + k);
        for (let r = 0; r < 6; r++) {
          let sum = 0;
          for (let q = 0; q < 6; q++) sum += inv[io + r * 6 + q] * vec[w + q];
          Y[r] = sum;
        }
        for (let r = 0; r < m; r++) {
          let sum = 0;
          for (let c = 0; c < 6; c++) sum += jac[jo + r * 6 + c] * Y[c];
          vec[v + r] += sum;
        }
      }
    }
  }
  // Back, parents first: z = D⁻¹ (w - H z_parent).
  for (let i = 0; i < n; i++) {
    const x = order[i], v = NODE * x, p = parent[x];
    if (x >= T) {
      const k = x - T, io = 36 * k;
      if (p >= 0) {
        // + Jᵀ z_p, from its parent joint
        const m = F.rows[p], jo = 72 * p + 36 * (F.side[2 * p] === k ? 0 : 1), z = NODE * p;
        for (let c = 0; c < 6; c++) {
          let sum = 0;
          for (let r = 0; r < m; r++) sum += jac[jo + r * 6 + c] * vec[z + r];
          vec[v + c] += sum;
        }
      }
      for (let r = 0; r < 6; r++) {
        let sum = 0;
        for (let q = 0; q < 6; q++) sum += inv[io + r * 6 + q] * vec[v + q];
        Y[r] = sum;
      }
      for (let r = 0; r < 6; r++) vec[v + r] = Y[r];
    } else {
      // + J z_p, from its parent body
      const t = x, m = F.rows[t], k = p - T, jo = 72 * t + 36 * (F.side[2 * t] === k ? 0 : 1), z = NODE * p, d = 36 * t;
      for (let r = 0; r < m; r++) {
        let sum = 0;
        for (let c = 0; c < 6; c++) sum += jac[jo + r * 6 + c] * vec[z + c];
        vec[v + r] += sum;
      }
      for (let r = 0; r < m; r++) {
        let sum = 0;
        for (let q = 0; q < m; q++) sum += D[d + r * m + q] * vec[v + q];
        Y[r] = sum;
      }
      for (let r = 0; r < m; r++) vec[v + r] = Y[r];
    }
  }
}

/** out = A B, for A rows x inner at a[ao..] and B inner x cols at b[bo..], both row-major. */
function product(a, ao, rows, inner, b, bo, cols, out) {
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      let sum = 0;
      for (let q = 0; q < inner; q++) sum += a[ao + r * inner + q] * b[bo + q * cols + c];
      out[r * cols + c] = sum;
    }
  }
}

/** A body's inverse mass matrix: 1/m on the diagonal for moving, its world inverse inertia for turning. */
function inverseMass(im, invI, o, out) {
  out.fill(0);
  out[0] = im; out[7] = im; out[14] = im;
  out[21] = invI[o]; out[22] = invI[o + 1]; out[23] = invI[o + 2];
  out[27] = invI[o + 1]; out[28] = invI[o + 3]; out[29] = invI[o + 4];
  out[33] = invI[o + 2]; out[34] = invI[o + 4]; out[35] = invI[o + 5];
}

/**
 * The inverse of the n x n matrix A (row-major, overwritten) into out, by
 * Gauss-Jordan elimination with partial pivoting. With every joint giving a
 * little (COMPLIANCE), no block is singular; a pivot of exactly nothing, a
 * joint with no row at all (a distance joint whose anchors meet), takes no
 * part rather than dividing by zero.
 */
function invert(A, n, out) {
  out.fill(0, 0, n * n);
  for (let i = 0; i < n; i++) out[i * n + i] = 1;
  for (let c = 0; c < n; c++) {
    PIVOT[c] = 1;
    let best = c, most = Math.abs(A[c * n + c]);
    for (let r = c + 1; r < n; r++) if (Math.abs(A[r * n + c]) > most) { most = Math.abs(A[r * n + c]); best = r; }
    if (most === 0) {
      PIVOT[c] = 0;
      continue;
    }
    if (best !== c) {
      for (let k = 0; k < n; k++) {
        let s = A[c * n + k]; A[c * n + k] = A[best * n + k]; A[best * n + k] = s;
        s = out[c * n + k]; out[c * n + k] = out[best * n + k]; out[best * n + k] = s;
      }
    }
    const p = 1 / A[c * n + c];
    for (let k = 0; k < n; k++) { A[c * n + k] *= p; out[c * n + k] *= p; }
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = A[r * n + c];
      if (f === 0) continue;
      for (let k = 0; k < n; k++) { A[r * n + k] -= f * A[c * n + k]; out[r * n + k] -= f * out[c * n + k]; }
    }
  }
  for (let c = 0; c < n; c++) {
    if (PIVOT[c]) continue;
    for (let k = 0; k < n; k++) { out[c * n + k] = 0; out[k * n + c] = 0; }
  }
}
