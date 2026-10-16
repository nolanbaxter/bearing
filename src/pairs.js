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
//
// A pair with a mesh or a heightfield can touch along several directions at
// once, a box in a valley pushed by both slopes, so it can have several
// manifolds a step, and remembers each in a Memory of its own, matched from
// step to step by direction (contact.js). Memories are reused, not made
// afresh, as pairs come and go.
//
// A pair also knows whether its bodies were touching after the last step it
// was looked at (contact.js decides). When that changes, or a touching pair
// is dropped or taken over by new bodies in the same slots, an event is
// written (events.js). Asleep pairs aren't looked at, so they keep touching,
// and write nothing.

import { resized } from './core/slots.js';
import { apart } from './joints.js';

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
    this.slip = new Float64Array(0);       // how far each point has slipped while friction held it, a world vector (contact.js, store)
    this.twist = new Float64Array(0);      // the pair's rolling and spinning friction, an angular impulse, world vector
    this.touching = new Uint8Array(0);     // 1 while its bodies touch, as of the last step that looked
    /** A mesh pair's Memory, by pair; null for any other pair. */
    this.memory = [];
    this.spare = [];
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
    this.touching = resized(this.touching, c);
    this.pid = resized(this.pid, c * P);
    this.normalImpulse = resized(this.normalImpulse, c * P);
    this.friction = resized(this.friction, c * P * 3);
    this.slip = resized(this.slip, c * P * 3);
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
    if (!pairable(w, x, y)) return;
    // Only a body that was re-filed asks, and a sleeping one never is; a static
    // one is when it is added or moved by hand. One that now overlaps a
    // sleeping body could leave it buried, so the sleeper wakes. (An awake
    // body coming near only wakes it by touching it: see narrow().) A sensor
    // buries nothing, and wakes nothing.
    if (w._sensor[x] === 0 && w._sensor[y] === 0) {
      if (type[y] === STATIC) w._wake(x);
      else if (type[x] === STATIC) w._wake(y);
    }
    const a = x < y ? x : y, b = x < y ? y : x;
    const ida = w._slots.id(a), idb = w._slots.id(b);
    const at = this._find(a, b);
    if (at >= 0) {
      const p = this.index[at];
      if (this.idA[p] === ida && this.idB[p] === idb) return;
      // The same slots under new bodies: the old pair is stale, so it ends,
      // and starts over, among the awake pairs, since a new body is awake.
      this.forget(p);
      this.idA[p] = ida; this.idB[p] = idb;
      if (p >= this.active) this.wake(p);
      return;
    }
    const p = this.count;
    this._grow(p + 1);
    this.a[p] = a; this.b[p] = b; this.idA[p] = ida; this.idB[p] = idb; this.points[p] = 0; this.touching[p] = 0;
    this.memory[p] = null;
    this._put(a, b, p);
    this.count++;
    this.wake(p);
  }

  /** Takes pair p off the list; the pairs that fill its place come from the end of each run. */
  remove(p) {
    this.part(p);
    this._delete(this._find(this.a[p], this.b[p]));
    const kept = this.memory[p];
    if (kept) { kept.count = 0; this.spare.push(kept); }
    if (p < this.active) {
      const last = --this.active;
      if (p !== last) this._move(last, p);
      p = last;
    }
    const end = --this.count;
    if (p !== end) this._move(end, p);
    this.memory[end] = null;
  }

  /** Pair p touched nothing this step: nothing to start the next from, and no longer touching. */
  forget(p) {
    this.points[p] = 0;
    const kept = this.memory[p];
    if (kept) kept.count = 0;
    this.part(p);
  }

  /** Pair p is no longer touching: its end, if it was. */
  part(p) {
    if (this.touching[p] === 0) return;
    this.touching[p] = 0;
    this.world._events.end(this.idA[p], this.idB[p]);
  }

  /** Pair p's Memory, taken when it first needs one: a pair can become a mesh pair when a body is reshaped. */
  memoryOf(p) {
    return this.memory[p] || (this.memory[p] = this.spare.pop() || new Memory());
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
    this.touching[to] = this.touching[from];
    this.pid.copyWithin(to * P, from * P, from * P + P);
    this.normalImpulse.copyWithin(to * P, from * P, from * P + P);
    this.friction.copyWithin(to * P * 3, from * P * 3, from * P * 3 + P * 3);
    this.slip.copyWithin(to * P * 3, from * P * 3, from * P * 3 + P * 3);
    this.twist.copyWithin(to * 3, from * 3, from * 3 + 3);
    this.memory[to] = this.memory[from];
    this.index[this._find(this.a[to], this.b[to])] = to;
  }

  _swap(i, j) {
    if (i === j) return;
    swap(this.a, i, j, 1); swap(this.b, i, j, 1);
    swap(this.idA, i, j, 1); swap(this.idB, i, j, 1);
    swap(this.points, i, j, 1); swap(this.touching, i, j, 1);
    swap(this.pid, i, j, P); swap(this.normalImpulse, i, j, P); swap(this.friction, i, j, 3 * P); swap(this.slip, i, j, 3 * P); swap(this.twist, i, j, 3);
    const kept = this.memory[i];
    this.memory[i] = this.memory[j];
    this.memory[j] = kept;
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
    if (!pairable(w, a, b)) return false;
    return broadphase.overlap(a, b);
  }

  /** Gives storage back once a quarter or less is in use: the world's rule. */
  trim() {
    if (this.capacity > 16 && this.count <= this.capacity / 4) {
      let c = 16;
      while (c < this.count) c *= 2;
      this._alloc(c);
      this.memory.length = Math.min(this.memory.length, c);
      this.spare.length = 0;
    }
    if (this.slots > 16 && 8 * this.count <= this.slots) {
      let s = 16;
      while (s < 2 * this.count) s *= 2;
      this._rehash(s);
    }
  }
}

/**
 * What a mesh pair's manifolds left behind, for warm starting: per manifold,
 * its normal and what Pairs keeps for one, grown by doubling.
 */
export class Memory {
  constructor() {
    this.count = 0;
    this.capacity = 0;
    this.normal = new Float64Array(0);
    this.taken = new Uint8Array(0);   // matched already this step
    this.key = new Int32Array(0);     // a manifold's parts, matched first: a part's number, or two parts' as i * nb + j
    this.points = new Uint8Array(0);
    this.pid = new Uint32Array(0);
    this.normalImpulse = new Float64Array(0);
    this.friction = new Float64Array(0);
    this.slip = new Float64Array(0);
    this.twist = new Float64Array(0);
    this.ensure(1);
  }

  ensure(n) {
    if (n <= this.capacity) return;
    let c = Math.max(this.capacity, 1);
    while (c < n) c *= 2;
    this.normal = resized(this.normal, 3 * c);
    this.taken = resized(this.taken, c);
    this.key = resized(this.key, c);
    this.points = resized(this.points, c);
    this.pid = resized(this.pid, c * P);
    this.normalImpulse = resized(this.normalImpulse, c * P);
    this.friction = resized(this.friction, c * P * 3);
    this.slip = resized(this.slip, c * P * 3);
    this.twist = resized(this.twist, c * 3);
    this.capacity = c;
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

/**
 * Whether bodies a and b are ever paired. Two bodies collide if one is
 * dynamic: a static or kinematic one moves nothing it touches, so two of
 * them have nothing to settle. A sensor sees anything that isn't a sensor,
 * kinematic included, so a zone sees a character walk in, but never another
 * sensor (Box2D 3's rule: rarely wanted, and it costs pairs), and never when
 * both are static, since neither can come or go.
 */
function pairable(w, a, b) {
  const type = w._type, sa = w._sensor[a], sb = w._sensor[b];
  if (sa === 0 && sb === 0) {
    if (type[a] !== DYNAMIC && type[b] !== DYNAMIC) return false;
  } else if ((sa === 1 && sb === 1) || (type[a] === STATIC && type[b] === STATIC)) {
    return false;
  }
  // Two bodies a joint holds don't collide unless it says they do (joints.js);
  // a body with no joints costs one read here.
  if (w._jointHead[a] !== 0 && w._jointHead[b] !== 0 && apart(w, a, b)) return false;
  return filtered(w, a, b);
}

/** Each body's layers must be in the other's mask (Box2D's rule, and Rapier's and Jolt's). */
function filtered(w, a, b) {
  return (w._layers[a] & w._mask[b]) !== 0 && (w._layers[b] & w._mask[a]) !== 0;
}
