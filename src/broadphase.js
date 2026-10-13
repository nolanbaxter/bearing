// The broadphase: which bodies might touch, without looking at every pair.
//
// Each body sits in a tree (tree.js) inside a "fat" box: its box, grown by a
// margin. While it stays inside that fat box, the tree isn't touched. Only a
// body that leaves its fat box is re-filed, and only a re-filed body asks the
// trees what it now overlaps. A pile at rest costs nothing here; a body that
// moves costs a removal, an insertion and a query, each a walk down a tree.
//
// Static bodies live in one tree and everything that moves in another, as in
// Box2D 3, so static against static is never even asked, and a level that
// never moves is never re-filed. Static and kinematic bodies are never paired
// with each other either, since neither would respond.
//
// What the broadphase finds are candidate pairs. The pair list (pairs.js)
// keeps each one while the two fat boxes overlap, so a pair found once is not
// looked for again, and the narrowphase only ever runs on pairs in that list.

import { Tree } from './tree.js';
import { resized } from './core/slots.js';

const STATIC_TREE = 0, MOVING_TREE = 1;
const STATIC = 1;   // World's type code

/**
 * How far a moving body's fat box reaches past its box, as a fraction of the
 * body's thinnest size: the room a body jostling in place has before it is
 * re-filed. Measured with bench/broadphase.js (i7-1260P, 10,000 bodies): 1/8
 * re-filed more and lost the fliers and the level by 13-15%; 1/2 made boxes
 * standing a body's width apart overlap, and a field of resting towers cost
 * 13 times as much.
 */
export const FAT = 1 / 4;

/**
 * How many steps ahead a moving body's fat box reaches along its velocity,
 * so a body moving steadily stays inside it for about that many steps instead
 * of being re-filed every step (Box2D 2 does the same, at 4). Re-filing is
 * what a moving tree costs: measured with bench/broadphase.js (i7-1260P,
 * 10,000 bodies), 16 against 8 cut the fliers' step from 6.9 to 4.3 ms and
 * the level's from 2.5 to 1.8 ms; with none, the level took 15.4 ms, behind a
 * one-axis sort and sweep's 5.9. More than 16 wasn't tried; the bench's
 * default run checks 32 next. Re-measure if re-filing gets cheaper (milestone 16).
 */
export const AHEAD = 16;

export class Broadphase {
  /** margin and ahead: FAT and AHEAD, unless measuring others. */
  constructor(margin = FAT, ahead = AHEAD) {
    this.margin = margin;
    this.ahead = ahead;
    this.trees = [new Tree(), new Tree()];
    this.capacity = 0;
    this.leaf = new Int32Array(0);      // the body's leaf in its tree
    this.which = new Int8Array(0);      // which tree: STATIC_TREE, MOVING_TREE, or -1 for none
    this.moved = new Int32Array(0);     // bodies re-filed this step, to query with
    this.movedCount = 0;
    this.isMoved = new Uint8Array(0);
    // The query visitor, made once: a fresh closure per moving body per step
    // would be the allocation in step() that rule 76 forbids.
    this._a = -1;
    this._pairs = null;
    this._visit = (b) => { if (b !== this._a) this._pairs.found(this._a, b); };
  }

  resize(capacity) {
    this.leaf = resized(this.leaf, capacity);
    const which = new Int8Array(capacity).fill(-1);
    which.set(this.which.subarray(0, Math.min(this.which.length, capacity)));
    this.which = which;
    // A shrink only drops removed slots, but they can still be on the moved
    // list anywhere, so the list is rebuilt rather than cut short.
    const moved = new Int32Array(capacity);
    let kept = 0;
    for (let i = 0; i < this.movedCount; i++) if (this.moved[i] < capacity) moved[kept++] = this.moved[i];
    this.moved = moved;
    this.movedCount = kept;
    this.isMoved = resized(this.isMoved, capacity);
    this.capacity = capacity;
  }

  /** Files slot s, of type `type`, with the world's current box for it. */
  add(world, s) {
    const t = world._type[s] === STATIC ? STATIC_TREE : MOVING_TREE;
    fat(world, s, t, this.margin, 0);
    this.leaf[s] = this.trees[t].insert(s, out[0], out[1], out[2], out[3], out[4], out[5]);
    this.which[s] = t;
    this._mark(s);
  }

  remove(s) {
    const t = this.which[s];
    if (t < 0) return;
    this.trees[t].remove(this.leaf[s]);
    // Left on the moved list, if it is there: candidates() skips a slot in no
    // tree, and queries for whatever body takes the slot before the step.
    // Clearing isMoved here would let a refile list the slot twice.
    this.which[s] = -1;
  }

  /** The body changed by hand (teleported, reshaped, retyped, refiltered): file it afresh. */
  refile(world, s) {
    this.remove(s);
    this.add(world, s);
  }

  _mark(s) {
    if (this.isMoved[s]) return;
    this.isMoved[s] = 1;
    this.moved[this.movedCount++] = s;
  }

  /**
   * Re-files every moving body whose box has left its fat box. The world's
   * _box is each body's box grown by how far it can move this step, so a body
   * is re-filed before it could reach anything its old fat box didn't cover.
   */
  update(world, dt) {
    const n = world._slots.top, state = world._slots.state, box = world._box;
    const tree = this.trees[MOVING_TREE], fb = tree.box;
    for (let s = 0; s < n; s++) {
      if (state[s] !== 1 || this.which[s] !== MOVING_TREE) continue;
      const o = 6 * this.leaf[s], b = 6 * s;
      if (box[b] >= fb[o] && box[b + 1] >= fb[o + 1] && box[b + 2] >= fb[o + 2]
          && box[b + 3] <= fb[o + 3] && box[b + 4] <= fb[o + 4] && box[b + 5] <= fb[o + 5]) continue;
      fat(world, s, MOVING_TREE, this.margin, this.ahead * dt);
      tree.move(this.leaf[s], out[0], out[1], out[2], out[3], out[4], out[5]);
      this._mark(s);
    }
  }

  /**
   * Calls pairs.found(a, b) for every body a re-filed this step and every body b
   * whose fat box now overlaps a's, in both trees, then forgets who moved.
   * The same pair may be offered twice, once from each end, when both moved;
   * the pair list keeps it once.
   */
  candidates(world, pairs) {
    this._pairs = pairs;
    const type = world._type;
    for (let i = 0; i < this.movedCount; i++) {
      const a = this.moved[i];
      this.isMoved[a] = 0;
      const t = this.which[a];
      if (t < 0) continue;   // removed since it moved
      const fb = this.trees[t].box, o = 6 * this.leaf[a];
      const x0 = fb[o], y0 = fb[o + 1], z0 = fb[o + 2], x1 = fb[o + 3], y1 = fb[o + 4], z1 = fb[o + 5];
      const aStatic = type[a] === STATIC;
      // A static body only meets moving ones; a moving one meets both.
      this._a = a;
      if (!aStatic) this.trees[STATIC_TREE].query(x0, y0, z0, x1, y1, z1, this._visit);
      this.trees[MOVING_TREE].query(x0, y0, z0, x1, y1, z1, this._visit);
    }
    this.movedCount = 0;
  }

  /** Whether the fat boxes of a and b overlap: how long a pair is kept. */
  overlap(a, b) {
    const fa = this.trees[this.which[a]].box, fb = this.trees[this.which[b]].box;
    const i = 6 * this.leaf[a], j = 6 * this.leaf[b];
    return fa[i] <= fb[j + 3] && fa[i + 3] >= fb[j] && fa[i + 1] <= fb[j + 4] && fa[i + 4] >= fb[j + 1]
      && fa[i + 2] <= fb[j + 5] && fa[i + 5] >= fb[j + 2];
  }

  /** Gives storage back after a burst; the trees renumber their leaves, so the handles are updated. */
  trim() {
    for (let t = 0; t < 2; t++) {
      const handles = this.trees[t].trim();
      if (handles) for (const [s, leaf] of handles) this.leaf[s] = leaf;
    }
  }
}

// One fat box at a time; the step never re-enters this.
const out = [0, 0, 0, 0, 0, 0];

/**
 * Slot s's box from the world, grown by the fat margin for a moving body,
 * and stretched by its velocity times `time` on the side it is heading. A
 * static body's box is exact: it doesn't move, so a margin would only add
 * pairs that can't happen.
 */
function fat(world, s, tree, margin, time) {
  const box = world._box, b = 6 * s;
  if (tree !== MOVING_TREE) {
    for (let k = 0; k < 6; k++) out[k] = box[b + k];
    return out;
  }
  const m = world._size[s] * margin, v = world._velocity;
  for (let k = 0; k < 3; k++) {
    const d = v[3 * s + k] * time;
    out[k] = box[b + k] - m + (d < 0 ? d : 0);
    out[k + 3] = box[b + k + 3] + m + (d > 0 ? d : 0);
  }
  return out;
}
