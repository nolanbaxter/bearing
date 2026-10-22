// A dynamic AABB tree: boxes nested in boxes, so a query can skip whole
// regions at once.
//
// Every leaf holds one item's box. Every inner node holds the box around its
// two children. A query starts at the root and only goes down into a node
// whose box overlaps what it is looking for: if a big box doesn't overlap, nothing
// inside it can, so most of the tree is never visited. Bodies are leaves; the
// broadphase finds what each moving body might touch by querying with its
// box, and later the same tree answers raycasts and shape casts.
//
// The tree changes as bodies move, which is what "dynamic" means: a leaf is
// taken out and put back where it now fits. Where it goes is chosen by the
// surface area heuristic (Goldsmith and Salmon, 1987; Box2D and Bullet both use
// it): the cost of a query is roughly proportional to the surface area of the
// boxes it has to open, so each insertion picks the place that grows the total
// area least. Then each node on the way back up is rebalanced by a rotation if
// one side has grown two levels taller than the other (Box2D 2's AVL balance),
// so no query has to walk a long thin branch.
//
// Everything is in typed arrays, indexed by node, grown by doubling, with
// freed nodes kept on a list for reuse: no object per node, nothing allocated
// once the tree has reached its size.

import { resized } from './core/slots.js';

const NULL = -1;

export class Tree {
  constructor() {
    this.capacity = 0;
    this.root = NULL;
    this.count = 0;      // nodes in use, leaves and inner
    this.leaves = 0;
    this.free = NULL;    // first free node, chained through child1
    this.box = new Float64Array(0);      // minX, minY, minZ, maxX, maxY, maxZ
    this.parent = new Int32Array(0);
    this.child1 = new Int32Array(0);
    this.child2 = new Int32Array(0);
    this.height = new Int32Array(0);     // 0 for a leaf, -1 for a free node
    this.item = new Int32Array(0);       // a leaf's item; NULL for an inner node
    this.stack = new Int32Array(64);     // for queries; grows if a tree is deeper
    this.at = new Float64Array(64);      // for raycast(): where the ray enters each node on the stack
  }

  _alloc() {
    if (this.free === NULL) {
      const old = this.capacity, capacity = Math.max(old * 2, 16);
      this.box = resized(this.box, capacity * 6);
      this.parent = resized(this.parent, capacity);
      this.child1 = resized(this.child1, capacity);
      this.child2 = resized(this.child2, capacity);
      this.height = resized(this.height, capacity);
      this.item = resized(this.item, capacity);
      for (let i = old; i < capacity; i++) {
        this.child1[i] = i + 1 < capacity ? i + 1 : NULL;
        this.height[i] = -1;
      }
      this.free = old;
      this.capacity = capacity;
    }
    const n = this.free;
    this.free = this.child1[n];
    this.parent[n] = NULL; this.child1[n] = NULL; this.child2[n] = NULL;
    this.height[n] = 0; this.item[n] = NULL;
    this.count++;
    return n;
  }

  _release(n) {
    this.child1[n] = this.free;
    this.height[n] = -1;
    this.free = n;
    this.count--;
  }

  /**
   * Shrinks the node arrays once the tree uses a quarter of them or less, by
   * rebuilding it into fresh, smaller arrays: the quarter rule the world's own
   * storage follows, so a burst of bodies leaves nothing behind.
   */
  trim() {
    // At the smallest size it would only rebuild into the same 16 nodes, on every removal.
    if (this.capacity <= 16 || this.count > this.capacity / 4) return;
    const items = [], boxes = [];
    for (let n = 0; n < this.capacity; n++) {
      if (this.height[n] === 0) { items.push(this.item[n]); boxes.push(this.box.slice(6 * n, 6 * n + 6)); }
    }
    const handles = new Map();
    this.capacity = 0; this.root = NULL; this.count = 0; this.leaves = 0; this.free = NULL;
    for (const name of ['box', 'parent', 'child1', 'child2', 'height', 'item']) this[name] = new this[name].constructor(0);
    for (let i = 0; i < items.length; i++) {
      const b = boxes[i];
      handles.set(items[i], this.insert(items[i], b[0], b[1], b[2], b[3], b[4], b[5]));
    }
    return handles;
  }

  /** Adds an item with its box; returns the leaf's node, which names it from now on. */
  insert(item, x0, y0, z0, x1, y1, z1) {
    const leaf = this._alloc();
    const b = this.box, o = 6 * leaf;
    b[o] = x0; b[o + 1] = y0; b[o + 2] = z0; b[o + 3] = x1; b[o + 4] = y1; b[o + 5] = z1;
    this.item[leaf] = item;
    this.leaves++;
    this._insertLeaf(leaf);
    return leaf;
  }

  remove(leaf) {
    this._removeLeaf(leaf);
    this._release(leaf);
    this.leaves--;
  }

  /** Gives a leaf a new box, and moves it to where that box now fits best. */
  move(leaf, x0, y0, z0, x1, y1, z1) {
    this._removeLeaf(leaf);
    const b = this.box, o = 6 * leaf;
    b[o] = x0; b[o + 1] = y0; b[o + 2] = z0; b[o + 3] = x1; b[o + 4] = y1; b[o + 5] = z1;
    this._insertLeaf(leaf);
  }

  _insertLeaf(leaf) {
    if (this.root === NULL) {
      this.root = leaf;
      this.parent[leaf] = NULL;
      return;
    }
    const b = this.box, l = 6 * leaf;
    const lx0 = b[l], ly0 = b[l + 1], lz0 = b[l + 2], lx1 = b[l + 3], ly1 = b[l + 4], lz1 = b[l + 5];

    // Walk down to the best sibling. At each inner node: making the leaf this
    // node's sibling costs the area of their union (a new parent), and every
    // ancestor grows by however much the union is bigger than the node; going
    // down into a child costs what that child would grow by, plus the same.
    let index = this.root;
    while (this.height[index] > 0) {
      const o = 6 * index;
      const area = surface(b[o], b[o + 1], b[o + 2], b[o + 3], b[o + 4], b[o + 5]);
      const combined = surface(Math.min(b[o], lx0), Math.min(b[o + 1], ly0), Math.min(b[o + 2], lz0),
        Math.max(b[o + 3], lx1), Math.max(b[o + 4], ly1), Math.max(b[o + 5], lz1));
      const cost = 2 * combined;
      const inherited = 2 * (combined - area);
      const c1 = this.child1[index], c2 = this.child2[index];
      const cost1 = this._descend(c1, lx0, ly0, lz0, lx1, ly1, lz1) + inherited;
      const cost2 = this._descend(c2, lx0, ly0, lz0, lx1, ly1, lz1) + inherited;
      if (cost < cost1 && cost < cost2) break;
      index = cost1 <= cost2 ? c1 : c2;
    }

    const sibling = index, oldParent = this.parent[sibling];
    const parent = this._alloc();
    this.parent[parent] = oldParent;
    this._fit(parent, sibling, leaf);
    this.height[parent] = this.height[sibling] + 1;
    if (oldParent !== NULL) {
      if (this.child1[oldParent] === sibling) this.child1[oldParent] = parent;
      else this.child2[oldParent] = parent;
    } else {
      this.root = parent;
    }
    this.child1[parent] = sibling; this.child2[parent] = leaf;
    this.parent[sibling] = parent; this.parent[leaf] = parent;
    this._refit(this.parent[leaf]);
  }

  /** What putting the leaf under child c would add, for _insertLeaf. */
  _descend(c, lx0, ly0, lz0, lx1, ly1, lz1) {
    const b = this.box, o = 6 * c;
    const union = surface(Math.min(b[o], lx0), Math.min(b[o + 1], ly0), Math.min(b[o + 2], lz0),
      Math.max(b[o + 3], lx1), Math.max(b[o + 4], ly1), Math.max(b[o + 5], lz1));
    if (this.height[c] === 0) return union;
    return union - surface(b[o], b[o + 1], b[o + 2], b[o + 3], b[o + 4], b[o + 5]);
  }

  _removeLeaf(leaf) {
    if (leaf === this.root) {
      this.root = NULL;
      return;
    }
    const parent = this.parent[leaf], grand = this.parent[parent];
    const sibling = this.child1[parent] === leaf ? this.child2[parent] : this.child1[parent];
    if (grand !== NULL) {
      if (this.child1[grand] === parent) this.child1[grand] = sibling;
      else this.child2[grand] = sibling;
      this.parent[sibling] = grand;
      this._release(parent);
      this._refit(grand);
    } else {
      this.root = sibling;
      this.parent[sibling] = NULL;
      this._release(parent);
    }
  }

  /** From node up to the root: rebalance, then refit each box and height. */
  _refit(index) {
    while (index !== NULL) {
      index = this._balance(index);
      const c1 = this.child1[index], c2 = this.child2[index];
      this.height[index] = 1 + Math.max(this.height[c1], this.height[c2]);
      this._fit(index, c1, c2);
      index = this.parent[index];
    }
  }

  /** The box of node n is the union of nodes p's and q's. */
  _fit(n, p, q) {
    const b = this.box, o = 6 * n, i = 6 * p, j = 6 * q;
    b[o] = Math.min(b[i], b[j]); b[o + 1] = Math.min(b[i + 1], b[j + 1]); b[o + 2] = Math.min(b[i + 2], b[j + 2]);
    b[o + 3] = Math.max(b[i + 3], b[j + 3]); b[o + 4] = Math.max(b[i + 4], b[j + 4]); b[o + 5] = Math.max(b[i + 5], b[j + 5]);
  }

  /**
   * If one child of a is two levels taller than the other, rotate the taller
   * one up into a's place, and its taller child with it (Box2D 2's Balance).
   * Returns the node now at a's place.
   */
  _balance(a) {
    if (this.height[a] < 2) return a;
    const b = this.child1[a], c = this.child2[a];
    const balance = this.height[c] - this.height[b];
    if (balance > 1) return this._rotate(a, c, b, false);
    if (balance < -1) return this._rotate(a, b, c, true);
    return a;
  }

  /** Lifts `up`, a child of a, into a's place; `other` is a's other child. */
  _rotate(a, up, other, upIsFirst) {
    const f = this.child1[up], g = this.child2[up];
    // up takes a's place.
    const top = this.parent[a];
    this.child1[up] = a;
    this.parent[up] = top;
    this.parent[a] = up;
    if (top !== NULL) {
      if (this.child1[top] === a) this.child1[top] = up;
      else this.child2[top] = up;
    } else {
      this.root = up;
    }
    // The taller of up's children stays with it; the shorter goes to a.
    const tall = this.height[f] > this.height[g] ? f : g, short = tall === f ? g : f;
    this.child2[up] = tall;
    if (upIsFirst) this.child1[a] = short; else this.child2[a] = short;
    this.parent[short] = a;
    this._fit(a, other, short);
    this._fit(up, a, tall);
    this.height[a] = 1 + Math.max(this.height[other], this.height[short]);
    this.height[up] = 1 + Math.max(this.height[a], this.height[tall]);
    return up;
  }

  /**
   * Calls visit(item, leaf) for every leaf whose box overlaps the given one.
   * visit may return false to stop early. Depth-first, child1 before child2,
   * so the order depends only on the tree, which depends only on the inputs.
   */
  query(x0, y0, z0, x1, y1, z1, visit) {
    if (this.root === NULL) return;
    const b = this.box;
    let stack = this.stack, top = 0;
    stack[top++] = this.root;
    while (top > 0) {
      const n = stack[--top], o = 6 * n;
      if (b[o] > x1 || b[o + 3] < x0 || b[o + 1] > y1 || b[o + 4] < y0 || b[o + 2] > z1 || b[o + 5] < z0) continue;
      if (this.height[n] === 0) {
        if (visit(this.item[n], n) === false) return;
        continue;
      }
      if (top + 2 > stack.length) stack = this.stack = resized(stack, stack.length * 2);
      stack[top++] = this.child2[n];
      stack[top++] = this.child1[n];
    }
  }
}

/**
 * Calls visit(item, leaf) for every leaf whose box, grown by (gx, gy, gz) on
 * every side, the ray from (ox, oy, oz) along (dx, dy, dz) passes through
 * between 0 and maxT. visit returns the maxT to carry on with: a nearer hit
 * returns its own distance, and from then on nothing beyond it is opened,
 * which is how a nearest-hit query leaves most of the tree unvisited. The
 * growth is for a shape cast: a node's box grown by the moving shape's
 * half-extents is everywhere its centre could be and still touch the box.
 * Returns the last maxT.
 */
export function raycast(tree, ox, oy, oz, dx, dy, dz, maxT, gx, gy, gz, visit) {
  if (tree.root === NULL) return maxT;
  const b = tree.box;
  // A zero component would give 0 * Infinity = NaN on a slab's edge, so it
  // is tested as "inside the slab or not" instead.
  const ix = 1 / dx, iy = 1 / dy, iz = 1 / dz;
  // Nearest first: both children are tested as their parent is opened, and
  // the one the ray enters first is opened next, with where it enters kept
  // beside it. A nearest-hit query then meets a near hit early, and anything
  // entered past the nearest hit so far is skipped when its turn comes. Opened
  // child1 before child2 instead, measured on 10,000 rays into the benchmark's
  // pile (bench/compare.html), the walk was 62% of a raycast: see ROADMAP.md,
  // milestone 16. Ties go to child1, so the order depends only on the tree.
  let stack = tree.stack, at = tree.at, top = 0;
  const t = enter(b, 6 * tree.root, ox, oy, oz, dx, dy, dz, ix, iy, iz, gx, gy, gz, maxT);
  if (t < 0) return maxT;
  stack[0] = tree.root; at[0] = t; top = 1;
  while (top > 0) {
    const n = stack[--top];
    if (at[top] > maxT) continue;
    if (tree.height[n] === 0) {
      maxT = visit(tree.item[n], n);
      continue;
    }
    const c1 = tree.child1[n], c2 = tree.child2[n];
    const t1 = enter(b, 6 * c1, ox, oy, oz, dx, dy, dz, ix, iy, iz, gx, gy, gz, maxT);
    const t2 = enter(b, 6 * c2, ox, oy, oz, dx, dy, dz, ix, iy, iz, gx, gy, gz, maxT);
    if (top + 2 > stack.length) {
      stack = tree.stack = resized(stack, stack.length * 2);
      at = tree.at = resized(at, stack.length);
    }
    // Pushed farther first, so the nearer comes off the stack next.
    if (t1 >= 0 && t2 >= 0) {
      if (t2 < t1) { stack[top] = c1; at[top++] = t1; stack[top] = c2; at[top++] = t2; }
      else { stack[top] = c2; at[top++] = t2; stack[top] = c1; at[top++] = t1; }
    } else if (t1 >= 0) {
      stack[top] = c1; at[top++] = t1;
    } else if (t2 >= 0) {
      stack[top] = c2; at[top++] = t2;
    }
  }
  return maxT;
}

/**
 * Where the ray enters the box at b[o..o+5] grown by (gx, gy, gz), from 0 to
 * maxT; -1 if it misses it there.
 */
function enter(b, o, ox, oy, oz, dx, dy, dz, ix, iy, iz, gx, gy, gz, maxT) {
  let t0 = 0, t1 = maxT;
  if (dx === 0) {
    if (ox < b[o] - gx || ox > b[o + 3] + gx) return -1;
  } else {
    let near = (b[o] - gx - ox) * ix, far = (b[o + 3] + gx - ox) * ix;
    if (near > far) { const s = near; near = far; far = s; }
    if (near > t0) t0 = near;
    if (far < t1) t1 = far;
    if (t0 > t1) return -1;
  }
  if (dy === 0) {
    if (oy < b[o + 1] - gy || oy > b[o + 4] + gy) return -1;
  } else {
    let near = (b[o + 1] - gy - oy) * iy, far = (b[o + 4] + gy - oy) * iy;
    if (near > far) { const s = near; near = far; far = s; }
    if (near > t0) t0 = near;
    if (far < t1) t1 = far;
    if (t0 > t1) return -1;
  }
  if (dz === 0) {
    if (oz < b[o + 2] - gz || oz > b[o + 5] + gz) return -1;
  } else {
    let near = (b[o + 2] - gz - oz) * iz, far = (b[o + 5] + gz - oz) * iz;
    if (near > far) { const s = near; near = far; far = s; }
    if (near > t0) t0 = near;
    if (far < t1) t1 = far;
    if (t0 > t1) return -1;
  }
  return t0;
}

/** Half the surface area of a box: what matters is how areas compare, so the factor of 2 is left out. */
function surface(x0, y0, z0, x1, y1, z1) {
  const x = x1 - x0, y = y1 - y0, z = z1 - z0;
  return x * y + y * z + z * x;
}
