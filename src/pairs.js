// The pair list: every pair of bodies whose fat boxes overlap, kept from step
// to step for as long as they do.
//
// The broadphase offers a pair when one of its bodies is re-filed near the
// other; the pair is kept until its fat boxes part, either body is removed,
// or filtering rules it out. Each step the narrowphase runs on every pair in
// the list, and nothing else. A pair also remembers its contact points'
// impulses from the last step, by feature id, which is what warm starting
// reads, and what the begin, stay and end events of milestone 10 will read.
//
// Pairs sit in a dense array, in two runs: first the pairs with an awake body,
// which the narrowphase looks at every step, then the pairs whose bodies are
// all asleep (or static), which nothing looks at until one of them wakes. A
// sleeping pile of 10,000 bodies keeps about 170,000 pairs, and a step that
// walked them all would cost milliseconds for nothing. A pair moves between
// the runs by trading places with the pair at the boundary, so no pair below
// the boundary moves while the narrowphase is working through it. That order
// depends only on the inputs, which keeps every step deterministic. A small
// hash table, keyed by the two slots, says whether a pair is already listed.

import { resized } from './core/slots.js';

const SLOT_SPAN = 16777216;   // 2^24: a pair's key is a * 2^24 + b, exact below 2^53
const STATIC = 1, DYNAMIC = 0;
const P = 4;                  // points a pair remembers

export class Pairs {
  constructor(world) {
    this.world = world;
    this.count = 0;
    /** Pairs [0, active) have an awake body; [active, count) are asleep. */
    this.active = 0;
    this.capacity = 0;
    this.a = new Int32Array(0); this.b = new Int32Array(0);
    this.idA = new Float64Array(0); this.idB = new Float64Array(0);
    // What the last step's contact left behind, for warm starting.
    this.points = new Uint8Array(0);
    this.pid = new Uint32Array(0);
    this.normalImpulse = new Float64Array(0);
    this.friction = new Float64Array(0);   // the friction impulse at each point, as a world vector
    this.twist = new Float64Array(0);      // the pair's rolling and spinning friction, an angular impulse, world vector
    // The hash table: open addressing, linear probing, keys -1 where empty.
    this.slots = 0;
    this.keys = new Float64Array(0);
    this.index = new Int32Array(0);
  }

  _grow(count) {
    if (count <= this.capacity) return;
    let c = Math.max(this.capacity, 16);
    while (c < count) c *= 2;
    this._alloc(c);
  }

  _alloc(c) {
    this.a = resized(this.a, c); this.b = resized(this.b, c);
    this.idA = resized(this.idA, c); this.idB = resized(this.idB, c);
    this.points = resized(this.points, c);
    this.pid = resized(this.pid, c * P);
    this.normalImpulse = resized(this.normalImpulse, c * P);
    this.friction = resized(this.friction, c * P * 3);
    this.twist = resized(this.twist, c * 3);
    this.capacity = c;
  }

  // ---- the hash table ----

  _home(a, b) {
    return (Math.imul(a, 0x9e3779b1) ^ Math.imul(b, 0x85ebca77)) & (this.slots - 1);
  }

  _find(a, b) {
    if (this.slots === 0) return -1;
    const key = a * SLOT_SPAN + b, mask = this.slots - 1;
    for (let i = this._home(a, b); ; i = (i + 1) & mask) {
      const k = this.keys[i];
      if (k === key) return i;
      if (k === -1) return -1;
    }
  }

  _put(a, b, p) {
    // Kept at most half full, so a search ends soon after its home.
    if (2 * (this.count + 1) > this.slots) this._rehash(Math.max(16, this.slots * 2));
    const key = a * SLOT_SPAN + b, mask = this.slots - 1;
    let i = this._home(a, b);
    while (this.keys[i] !== -1) i = (i + 1) & mask;
    this.keys[i] = key;
    this.index[i] = p;
  }

  /**
   * Takes the key at slot i out, and moves later keys of the same run back
   * into the gap, so a search never stops early at it (linear probing's
   * backward-shift deletion, needing no tombstones).
   */
  _delete(i) {
    const mask = this.slots - 1;
    let gap = i;
    for (let j = (gap + 1) & mask; this.keys[j] !== -1; j = (j + 1) & mask) {
      const key = this.keys[j], a = Math.floor(key / SLOT_SPAN), home = this._home(a, key - a * SLOT_SPAN);
      // Moves back unless its home lies cyclically in (gap, j].
      const between = gap <= j ? (home > gap && home <= j) : (home > gap || home <= j);
      if (between) continue;
      this.keys[gap] = key;
      this.index[gap] = this.index[j];
      gap = j;
    }
    this.keys[gap] = -1;
  }

  _rehash(slots) {
    this.slots = slots;
    this.keys = new Float64Array(slots).fill(-1);
    this.index = new Int32Array(slots);
    const mask = slots - 1;
    for (let p = 0; p < this.count; p++) {
      let i = this._home(this.a[p], this.b[p]);
      while (this.keys[i] !== -1) i = (i + 1) & mask;
      this.keys[i] = this.a[p] * SLOT_SPAN + this.b[p];
      this.index[i] = p;
    }
  }

  // ---- what the broadphase calls ----

  /**
   * Offered by the broadphase: bodies x and y might touch. Listed unless they
   * can't collide (neither dynamic, or filtered apart) or already are.
   */
  found(x, y) {
    const w = this.world, type = w._type;
    if (type[x] !== DYNAMIC && type[y] !== DYNAMIC) return;
    if (!filtered(w, x, y)) return;
    // Only a body that was re-filed asks, and a sleeping one never is; a static
    // one is when it is added or moved by hand. One that now overlaps a
    // sleeping body could leave it buried, so the sleeper wakes. (An awake
    // body coming near only wakes it by touching it: see narrow().)
    if (type[y] === STATIC) w._wake(x);
    else if (type[x] === STATIC) w._wake(y);
    const a = x < y ? x : y, b = x < y ? y : x;
    const ida = w._slots.id(a), idb = w._slots.id(b);
    const at = this._find(a, b);
    if (at >= 0) {
      const p = this.index[at];
      if (this.idA[p] === ida && this.idB[p] === idb) return;
      // The same slots under new bodies: the old pair is stale, so it starts
      // over, among the awake pairs, since a new body is awake.
      this.idA[p] = ida; this.idB[p] = idb; this.points[p] = 0;
      if (p >= this.active) this.wake(p);
      return;
    }
    const p = this.count;
    this._grow(p + 1);
    this.a[p] = a; this.b[p] = b; this.idA[p] = ida; this.idB[p] = idb; this.points[p] = 0;
    this._put(a, b, p);
    this.count++;
    this.wake(p);
  }

  /** Takes pair p off the list; the pairs that fill its place come from the end of each run. */
  remove(p) {
    this._delete(this._find(this.a[p], this.b[p]));
    if (p < this.active) {
      const last = --this.active;
      if (p !== last) this._move(last, p);
      p = last;
    }
    const end = --this.count;
    if (p !== end) this._move(end, p);
  }

  /** Moves asleep pair p into the awake run. Pairs below the boundary stay where they are. */
  wake(p) {
    this._swap(p, this.active);
    this.active++;
  }

  /** Moves awake pair p into the asleep run; the last awake pair takes its place. */
  sleep(p) {
    this.active--;
    this._swap(p, this.active);
  }

  _move(from, to) {
    this.a[to] = this.a[from]; this.b[to] = this.b[from];
    this.idA[to] = this.idA[from]; this.idB[to] = this.idB[from];
    this.points[to] = this.points[from];
    this.pid.copyWithin(to * P, from * P, from * P + P);
    this.normalImpulse.copyWithin(to * P, from * P, from * P + P);
    this.friction.copyWithin(to * P * 3, from * P * 3, from * P * 3 + P * 3);
    this.twist.copyWithin(to * 3, from * 3, from * 3 + 3);
    this.index[this._find(this.a[to], this.b[to])] = to;
  }

  _swap(i, j) {
    if (i === j) return;
    swap(this.a, i, j, 1); swap(this.b, i, j, 1);
    swap(this.idA, i, j, 1); swap(this.idB, i, j, 1);
    swap(this.points, i, j, 1);
    swap(this.pid, i, j, P); swap(this.normalImpulse, i, j, P); swap(this.friction, i, j, 3 * P); swap(this.twist, i, j, 3);
    this.index[this._find(this.a[i], this.b[i])] = i;
    this.index[this._find(this.a[j], this.b[j])] = j;
  }

  /**
   * Whether pair p still holds: both bodies alive as the same bodies, still
   * able to collide, and their fat boxes still overlapping.
   */
  holds(p, broadphase) {
    const w = this.world, a = this.a[p], b = this.b[p];
    if (w._slots.slotOf(this.idA[p]) !== a || w._slots.slotOf(this.idB[p]) !== b) return false;
    if (w._type[a] !== DYNAMIC && w._type[b] !== DYNAMIC) return false;
    if (!filtered(w, a, b)) return false;
    return broadphase.overlap(a, b);
  }

  /** Gives storage back once a quarter or less is in use: the world's rule. */
  trim() {
    if (this.capacity > 16 && this.count <= this.capacity / 4) {
      let c = 16;
      while (c < this.count) c *= 2;
      this._alloc(c);
    }
    if (this.slots > 16 && 8 * this.count <= this.slots) {
      let s = 16;
      while (s < 2 * this.count) s *= 2;
      this._rehash(s);
    }
  }
}

/** Swaps the n values at i and j. */
function swap(array, i, j, n) {
  for (let k = 0; k < n; k++) {
    const t = array[i * n + k];
    array[i * n + k] = array[j * n + k];
    array[j * n + k] = t;
  }
}

/** Each body's layers must be in the other's mask (Box2D's rule, and Rapier's and Jolt's). */
function filtered(w, a, b) {
  return (w._layers[a] & w._mask[b]) !== 0 && (w._layers[b] & w._mask[a]) !== 0;
}
