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
import { axesOf } from './core/math.js';

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
export const COMPLIANCE = 9.5367431640625e-7;   // 2^-20

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
    // The factor's products the forward solve needs again, kept as they are
    // made: D⁻¹ J toward its parent body (m x 6), and J D_k⁻¹ from each child
    // body k (m x 6 a side). The forward pass then multiplies once where it
    // multiplied twice: measured, see ROADMAP.md, milestone 16.
    grow('up', Float64Array, 36);
    grow('down', Float64Array, 72);
    grow('fresh', Uint8Array, 3);      // by node: 1 if its tree is factored at this solve pass (joints.js, age())
    grow('inTree', Uint8Array, 1);     // by joint slot: 1 while the joint is in the forest this step; 2 or 3 in a tree held down (see plant())
    // Per body node k, at most two per joint:
    grow('body', Int32Array, 2);       // its slot
    grow('inverse', Float64Array, 2 * 36);   // its block's inverse, 6x6
    grow('first', Int32Array, 2);      // its first forest joint, as an edge 2t + side, plus 1
    grow('next', Int32Array, 2);       // per edge (2t + side), the next at the same body, plus 1
    grow('union', Int32Array, 2);      // union-find while trees are found
    grow('since', Float64Array, 2 * 4);  // its rotation when its tree was last factored (joints.js, age())
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
 * inputs; `point` says whether a joint holds a point, and so can hold its
 * tree down (below). world._node is a per-body column, the body's node plus
 * 1, left zeroed again.
 */
export function plant(world, list, n, holds, point) {
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
  // A tree with a joint to something the solve doesn't move (a static,
  // kinematic or character body, or a point in the world) is held down: its
  // joints are marked 2, and its bodies aren't caught alone (World._catch).
  // One held down with a body in this step's contacts is marked 3, and its
  // impulses aren't replayed (joints.js, warmStartJoints()). Flags per tree
  // root in seen, free again now, 1 for held down and 2 for in contact;
  // gather() lists no joint without a dynamic body, so a side of -1 has a
  // node opposite. A joint that holds no point (a rotation joint) doesn't
  // hold its tree down: a body it holds upright, swept back along its path
  // by World._catch, turns back along it too, and opens nothing.
  seen.fill(0, 0, F.bodies);
  for (let t = 0; t < T; t++) {
    if (!point(F.joint[t])) continue;
    if (F.side[2 * t] < 0) seen[find(F.union, F.side[2 * t + 1])] = 1;
    else if (F.side[2 * t + 1] < 0) seen[find(F.union, F.side[2 * t])] = 1;
  }
  const c = world._contacts;
  for (let m = 0; m < c.count; m++) {
    const a = node[c.a[m]], b = node[c.b[m]];
    if (a !== 0) { const r = find(F.union, a - 1); if (seen[r] === 1) seen[r] = 3; }
    if (b !== 0) { const r = find(F.union, b - 1); if (seen[r] === 1) seen[r] = 3; }
  }
  for (let t = 0; t < T; t++) {
    const k = F.side[2 * t] >= 0 ? F.side[2 * t] : F.side[2 * t + 1], f = seen[find(F.union, k)];
    if (f !== 0) F.inTree[F.joint[t]] = f === 1 ? 2 : 3;
  }
  for (let k = 0; k < F.bodies; k++) node[F.body[k]] = 0;
  return T;
}

/**
 * Turns 1 to 2 in `mark`, a column by body slot, for every body in a tree
 * the last plant() found held down. Each body node has a forest joint, its
 * first, whose mark is its tree's.
 */
export function anchored(world, mark) {
  const F = world._joints.forest;
  for (let k = 0; k < F.bodies; k++) {
    const s = F.body[k];
    if (mark[s] === 1 && F.inTree[F.joint[(F.first[k] - 1) >> 1]] >= 2) mark[s] = 2;
  }
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
/** A body's axes where it has turned to, for mass(). */
const AX9 = new Float64Array(9);

/**
 * Factors the forest's matrix from its joints' rows at F.row: each node's
 * block's inverse, children first, for the trees F.fresh marks: every tree at
 * a step's first substep, and after that only one a body of which has turned
 * too far since (see joints.js, age()).
 *
 * A body's block D = M - S (S = Σ Jᵀ D⁻¹ J over its child joints) is
 * symmetric and positive definite, so it is inverted through its Cholesky
 * factor, a sixth of the work of general elimination; a joint's, -D, the
 * same. A body that can't turn about some axis has no finite M there, and
 * its block is kept as (I - W S)⁻¹ W instead, from its inverse mass matrix W.
 */
export function factorForest(world) {
  const F = world._joints.forest, T = F.count, n = F.nodes, order = F.order, parent = F.parent;
  const invMass = world._invMass, invI = world._invIt, lock = world._lock;
  const first = F.first, next = F.next, rows = F.rows, side = F.side, jac = F.jac;
  const D = F.jointInverse, inv = F.inverse, up = F.up, down = F.down, fresh = F.fresh;
  for (let t = 0; t < T; t++) if (fresh[t] !== 0) shape(F, t);
  // The loops below take every sum's terms in the order a plain loop over
  // q would (from 0, and from a 0 start, which keeps a -0 term's sign as
  // the loop did), so the factor is the same to the bit; only the index
  // arithmetic is hoisted, and sums over 6 unrolled. Measured: see
  // ROADMAP.md, milestone 16.
  for (let i = n - 1; i >= 0; i--) {
    const x = order[i];
    if (fresh[x] === 0) continue;   // a tree whose factor is kept
    if (x >= T) {
      const k = x - T, s = F.body[k], o = 36 * k;
      S.fill(0);
      let any = false;
      for (let e = first[k] - 1; e >= 0; e = next[e] - 1) {
        const t = e >> 1;
        if (parent[t] !== x) continue;
        any = true;
        // S += Jᵀ D⁻¹ J, J this joint's rows at this body, with D⁻¹ J made
        // straight into F.up (kept for the forward solve) and read from there.
        const m = rows[t], jo = 72 * t + 36 * (e & 1), d = 36 * t, u = 36 * t;
        for (let r = 0; r < m; r++) {
          const dr = d + r * m, ur = u + 6 * r;
          for (let c = 0; c < 6; c++) {
            let sum = 0;
            for (let q = 0, jq = jo + c; q < m; q++, jq += 6) sum += D[dr + q] * jac[jq];
            up[ur + c] = sum;
          }
        }
        for (let r = 0; r < 6; r++) {
          for (let c = r; c < 6; c++) {
            let sum = 0;
            for (let q = 0, jq = jo + r, uq = u + c; q < m; q++, jq += 6, uq += 6) sum += jac[jq] * up[uq];
            S[6 * r + c] += sum;
          }
        }
      }
      // Each body's inertia is taken where it has turned to now, as its
      // joints' rows are: see turnedInertias() in contact.js.
      if (!any) {
        inverseMass(invMass[s], invI, 6 * s, inv, o);
        continue;
      }
      for (let r = 1; r < 6; r++) for (let c = 0; c < r; c++) S[6 * r + c] = S[6 * c + r];
      if (lock[s] === 0) {
        mass(world, s, E);
        for (let q = 0; q < 36; q++) E[q] -= S[q];
        if (inverse6(E, inv, o)) continue;
      }
      inverseMass(invMass[s], invI, 6 * s, W, 0);
      product(W, 0, 6, 6, S, 0, 6, X);
      for (let q = 0; q < 36; q++) E[q] = (q % 7 === 0 ? 1 : 0) - X[q];
      invert(E, 6, X);
      product(X, 0, 6, 6, W, 0, 6, S);
      inv.set(S, o);
    } else {
      // A joint's block: D = -(give + Σ J D⁻¹ Jᵀ) over its child bodies.
      const t = x, m = rows[t], give = COMPLIANCE * F.scale[t] / m;
      E.fill(0, 0, m * m);
      for (let r = 0; r < m; r++) E[r * m + r] = give;
      for (let sd = 0; sd < 2; sd++) {
        const k = side[2 * t + sd];
        if (k < 0 || parent[T + k] !== t) continue;
        // J D_k⁻¹, made straight into F.down (kept for the forward solve),
        // then E += (J D_k⁻¹) Jᵀ, each from a row held in locals.
        const jo = 72 * t + 36 * sd, io = 36 * k, dn = jo;
        for (let r = 0; r < m; r++) {
          const jr = jo + 6 * r, dr = dn + 6 * r;
          const j0 = jac[jr], j1 = jac[jr + 1], j2 = jac[jr + 2], j3 = jac[jr + 3], j4 = jac[jr + 4], j5 = jac[jr + 5];
          for (let c = 0; c < 6; c++) {
            const ic = io + c;
            down[dr + c] = 0 + j0 * inv[ic] + j1 * inv[ic + 6] + j2 * inv[ic + 12] + j3 * inv[ic + 18] + j4 * inv[ic + 24] + j5 * inv[ic + 30];
          }
        }
        for (let r = 0; r < m; r++) {
          const dr = dn + 6 * r, rm = r * m;
          const x0 = down[dr], x1 = down[dr + 1], x2 = down[dr + 2], x3 = down[dr + 3], x4 = down[dr + 4], x5 = down[dr + 5];
          for (let c = r, jc = jo + 6 * r; c < m; c++, jc += 6) {
            E[rm + c] += 0 + x0 * jac[jc] + x1 * jac[jc + 1] + x2 * jac[jc + 2] + x3 * jac[jc + 3] + x4 * jac[jc + 4] + x5 * jac[jc + 5];
          }
        }
      }
      for (let r = 1; r < m; r++) for (let c = 0; c < r; c++) E[r * m + c] = E[c * m + r];
      if (!inverseSPD(E, m, X)) invert(E, m, X);
      const d = 36 * t;
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

/** Body slot s's mass matrix, m on the diagonal for moving and its world inertia R diag(I) Rᵀ for turning, where it has turned to now, into out. */
function mass(world, s, out) {
  out.fill(0);
  const m = 1 / world._invMass[s];
  out[0] = m; out[7] = m; out[14] = m;
  const ax = AX9, a9 = 0, I = world._inertia, i0 = I[3 * s], i1 = I[3 * s + 1], i2 = I[3 * s + 2];
  axesOf(world._rotation, 4 * s, AX9, 0);
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
  if (n === 6) return inverse6(A, out, 0);
  for (let c = 0; c < n; c++) {
    const cn = c * n;
    let d = A[cn + c];
    for (let q = 0; q < c; q++) d -= L[cn + q] * L[cn + q];
    if (!(d > 0)) return false;
    const l = Math.sqrt(d);
    L[cn + c] = l;
    for (let r = c + 1, rn = r * n; r < n; r++, rn += n) {
      let sum = A[rn + c];
      for (let q = 0; q < c; q++) sum -= L[rn + q] * L[cn + q];
      L[rn + c] = sum / l;
    }
  }
  // L⁻¹, lower triangular, into A's lower half.
  for (let c = 0; c < n; c++) {
    A[c * n + c] = 1 / L[c * n + c];
    for (let r = c + 1, rn = r * n; r < n; r++, rn += n) {
      let sum = 0;
      for (let q = c, qc = c * n + c; q < r; q++, qc += n) sum -= L[rn + q] * A[qc];
      A[rn + c] = sum / L[rn + r];
    }
  }
  // A⁻¹ = (L⁻¹)ᵀ L⁻¹: entry (r, c) = Σ_{q ≥ max(r, c)} L⁻¹[q][r] L⁻¹[q][c].
  for (let r = 0; r < n; r++) {
    for (let c = r; c < n; c++) {
      let sum = 0;
      for (let q = c, qn = c * n; q < n; q++, qn += n) sum += A[qn + r] * A[qn + c];
      out[r * n + c] = sum;
      out[c * n + r] = sum;
    }
  }
  return true;
}

/**
 * inverseSPD for n = 6, every body's block, into out at p: the same
 * operations in the same order, unrolled, with the factor and its inverse in
 * locals rather than in L and A. A is only read, so it may be out, and is
 * left as it was when this returns false. Measured: see ROADMAP.md,
 * milestone 16.
 */
function inverse6(A, out, p) {
  const d0 = A[0];
  if (!(d0 > 0)) return false;
  const l00 = Math.sqrt(d0);
  const l10 = (A[6]) / l00;
  const l20 = (A[12]) / l00;
  const l30 = (A[18]) / l00;
  const l40 = (A[24]) / l00;
  const l50 = (A[30]) / l00;
  const d1 = A[7] - l10 * l10;
  if (!(d1 > 0)) return false;
  const l11 = Math.sqrt(d1);
  const l21 = (A[13] - l20 * l10) / l11;
  const l31 = (A[19] - l30 * l10) / l11;
  const l41 = (A[25] - l40 * l10) / l11;
  const l51 = (A[31] - l50 * l10) / l11;
  const d2 = A[14] - l20 * l20 - l21 * l21;
  if (!(d2 > 0)) return false;
  const l22 = Math.sqrt(d2);
  const l32 = (A[20] - l30 * l20 - l31 * l21) / l22;
  const l42 = (A[26] - l40 * l20 - l41 * l21) / l22;
  const l52 = (A[32] - l50 * l20 - l51 * l21) / l22;
  const d3 = A[21] - l30 * l30 - l31 * l31 - l32 * l32;
  if (!(d3 > 0)) return false;
  const l33 = Math.sqrt(d3);
  const l43 = (A[27] - l40 * l30 - l41 * l31 - l42 * l32) / l33;
  const l53 = (A[33] - l50 * l30 - l51 * l31 - l52 * l32) / l33;
  const d4 = A[28] - l40 * l40 - l41 * l41 - l42 * l42 - l43 * l43;
  if (!(d4 > 0)) return false;
  const l44 = Math.sqrt(d4);
  const l54 = (A[34] - l50 * l40 - l51 * l41 - l52 * l42 - l53 * l43) / l44;
  const d5 = A[35] - l50 * l50 - l51 * l51 - l52 * l52 - l53 * l53 - l54 * l54;
  if (!(d5 > 0)) return false;
  const l55 = Math.sqrt(d5);
  const i00 = 1 / l00;
  const i10 = (0 - l10 * i00) / l11;
  const i20 = (0 - l20 * i00 - l21 * i10) / l22;
  const i30 = (0 - l30 * i00 - l31 * i10 - l32 * i20) / l33;
  const i40 = (0 - l40 * i00 - l41 * i10 - l42 * i20 - l43 * i30) / l44;
  const i50 = (0 - l50 * i00 - l51 * i10 - l52 * i20 - l53 * i30 - l54 * i40) / l55;
  const i11 = 1 / l11;
  const i21 = (0 - l21 * i11) / l22;
  const i31 = (0 - l31 * i11 - l32 * i21) / l33;
  const i41 = (0 - l41 * i11 - l42 * i21 - l43 * i31) / l44;
  const i51 = (0 - l51 * i11 - l52 * i21 - l53 * i31 - l54 * i41) / l55;
  const i22 = 1 / l22;
  const i32 = (0 - l32 * i22) / l33;
  const i42 = (0 - l42 * i22 - l43 * i32) / l44;
  const i52 = (0 - l52 * i22 - l53 * i32 - l54 * i42) / l55;
  const i33 = 1 / l33;
  const i43 = (0 - l43 * i33) / l44;
  const i53 = (0 - l53 * i33 - l54 * i43) / l55;
  const i44 = 1 / l44;
  const i54 = (0 - l54 * i44) / l55;
  const i55 = 1 / l55;
  out[p + 0] = 0 + i00 * i00 + i10 * i10 + i20 * i20 + i30 * i30 + i40 * i40 + i50 * i50;
  out[p + 1] = out[p + 6] = 0 + i10 * i11 + i20 * i21 + i30 * i31 + i40 * i41 + i50 * i51;
  out[p + 2] = out[p + 12] = 0 + i20 * i22 + i30 * i32 + i40 * i42 + i50 * i52;
  out[p + 3] = out[p + 18] = 0 + i30 * i33 + i40 * i43 + i50 * i53;
  out[p + 4] = out[p + 24] = 0 + i40 * i44 + i50 * i54;
  out[p + 5] = out[p + 30] = 0 + i50 * i55;
  out[p + 7] = 0 + i11 * i11 + i21 * i21 + i31 * i31 + i41 * i41 + i51 * i51;
  out[p + 8] = out[p + 13] = 0 + i21 * i22 + i31 * i32 + i41 * i42 + i51 * i52;
  out[p + 9] = out[p + 19] = 0 + i31 * i33 + i41 * i43 + i51 * i53;
  out[p + 10] = out[p + 25] = 0 + i41 * i44 + i51 * i54;
  out[p + 11] = out[p + 31] = 0 + i51 * i55;
  out[p + 14] = 0 + i22 * i22 + i32 * i32 + i42 * i42 + i52 * i52;
  out[p + 15] = out[p + 20] = 0 + i32 * i33 + i42 * i43 + i52 * i53;
  out[p + 16] = out[p + 26] = 0 + i42 * i44 + i52 * i54;
  out[p + 17] = out[p + 32] = 0 + i52 * i55;
  out[p + 21] = 0 + i33 * i33 + i43 * i43 + i53 * i53;
  out[p + 22] = out[p + 27] = 0 + i43 * i44 + i53 * i54;
  out[p + 23] = out[p + 33] = 0 + i53 * i55;
  out[p + 28] = 0 + i44 * i44 + i54 * i54;
  out[p + 29] = out[p + 34] = 0 + i54 * i55;
  out[p + 35] = 0 + i55 * i55;
  return true;
}

/**
 * Solves the factored forest for the targets at F.rhs: every forest joint's
 * rows held exactly together. Leaves each body's velocity change, and each
 * joint's impulses, at its node in F.vector, for the caller to apply.
 */
export function solveForest(world) {
  const F = world._joints.forest, T = F.count, n = F.nodes, order = F.order, parent = F.parent, vec = F.vector;
  const jac = F.jac, inv = F.inverse, D = F.jointInverse, up = F.up, down = F.down, rhs = F.rhs;
  const first = F.first, next = F.next, rows = F.rows, side = F.side;
  // As in factorForest, every sum keeps the plain loop's order and 0 start;
  // only the index arithmetic is hoisted, and sums over 6 unrolled.
  // Forward, children first: w = b - Σ H D⁻¹ w over the children. A body's
  // b is 0, a joint's -rhs, and H between them -J (or -Jᵀ).
  for (let i = n - 1; i >= 0; i--) {
    const x = order[i], v = NODE * x;
    if (x >= T) {
      const k = x - T;
      vec.fill(0, v, v + 6);
      for (let e = first[k] - 1; e >= 0; e = next[e] - 1) {
        const t = e >> 1;
        if (parent[t] !== x) continue;
        // w_k += Jᵀ (D_t⁻¹ w_t), which is (D_t⁻¹ J)ᵀ w_t, D_t being symmetric
        const m = rows[t], u = 36 * t, w = NODE * t;
        for (let c = 0; c < 6; c++) {
          let sum = 0;
          for (let r = 0, ur = u + c; r < m; r++, ur += 6) sum += up[ur] * vec[w + r];
          vec[v + c] += sum;
        }
      }
    } else {
      const t = x, m = rows[t];
      for (let r = 0; r < m; r++) vec[v + r] = -rhs[6 * t + r];
      for (let sd = 0; sd < 2; sd++) {
        const k = side[2 * t + sd];
        if (k < 0 || parent[T + k] !== t) continue;
        // w_t += J (D_k⁻¹ w_k), which is (J D_k⁻¹) w_k
        const w = NODE * (T + k);
        const w0 = vec[w], w1 = vec[w + 1], w2 = vec[w + 2], w3 = vec[w + 3], w4 = vec[w + 4], w5 = vec[w + 5];
        for (let r = 0, dr = 72 * t + 36 * sd; r < m; r++, dr += 6) {
          vec[v + r] += 0 + down[dr] * w0 + down[dr + 1] * w1 + down[dr + 2] * w2 + down[dr + 3] * w3 + down[dr + 4] * w4 + down[dr + 5] * w5;
        }
      }
    }
  }
  // Back, parents first: z = D⁻¹ (w - H z_parent).
  for (let i = 0; i < n; i++) {
    const x = order[i], v = NODE * x, p = parent[x];
    if (x >= T) {
      const k = x - T;
      if (p >= 0) {
        // + Jᵀ z_p, from its parent joint
        const m = rows[p], jo = 72 * p + 36 * (side[2 * p] === k ? 0 : 1), z = NODE * p;
        for (let c = 0; c < 6; c++) {
          let sum = 0;
          for (let r = 0, jr = jo + c; r < m; r++, jr += 6) sum += jac[jr] * vec[z + r];
          vec[v + c] += sum;
        }
      }
      // Its block's inverse times it, read whole first, so written in place.
      const v0 = vec[v], v1 = vec[v + 1], v2 = vec[v + 2], v3 = vec[v + 3], v4 = vec[v + 4], v5 = vec[v + 5];
      for (let r = 0, a = 36 * k; r < 6; r++, a += 6) {
        vec[v + r] = 0 + inv[a] * v0 + inv[a + 1] * v1 + inv[a + 2] * v2 + inv[a + 3] * v3 + inv[a + 4] * v4 + inv[a + 5] * v5;
      }
    } else {
      // + J z_p, from its parent body
      const t = x, m = rows[t], k = p - T, z = NODE * p;
      const z0 = vec[z], z1 = vec[z + 1], z2 = vec[z + 2], z3 = vec[z + 3], z4 = vec[z + 4], z5 = vec[z + 5];
      for (let r = 0, jr = 72 * t + 36 * (side[2 * t] === k ? 0 : 1); r < m; r++, jr += 6) {
        Y[r] = vec[v + r] + (0 + jac[jr] * z0 + jac[jr + 1] * z1 + jac[jr + 2] * z2 + jac[jr + 3] * z3 + jac[jr + 4] * z4 + jac[jr + 5] * z5);
      }
      for (let r = 0, dr = 36 * t; r < m; r++, dr += m) {
        let sum = 0;
        for (let q = 0; q < m; q++) sum += D[dr + q] * Y[q];
        vec[v + r] = sum;
      }
    }
  }
}

/** out = A B, for A rows x inner at a[ao..] and B inner x cols at b[bo..], both row-major. */
function product(a, ao, rows, inner, b, bo, cols, out) {
  for (let r = 0; r < rows; r++) {
    const ar = ao + r * inner, or = r * cols;
    for (let c = 0; c < cols; c++) {
      let sum = 0;
      for (let q = 0, bq = bo + c; q < inner; q++, bq += cols) sum += a[ar + q] * b[bq];
      out[or + c] = sum;
    }
  }
}

/** A body's inverse mass matrix, into out at p: 1/m on the diagonal for moving, its world inverse inertia for turning. */
function inverseMass(im, invI, o, out, p) {
  out.fill(0, p, p + 36);
  out[p] = im; out[p + 7] = im; out[p + 14] = im;
  out[p + 21] = invI[o]; out[p + 22] = invI[o + 1]; out[p + 23] = invI[o + 2];
  out[p + 27] = invI[o + 1]; out[p + 28] = invI[o + 3]; out[p + 29] = invI[o + 4];
  out[p + 33] = invI[o + 2]; out[p + 34] = invI[o + 4]; out[p + 35] = invI[o + 5];
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
