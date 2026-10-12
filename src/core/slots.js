// Slots and ids: where a body's data lives, and the number that names it.
//
//   id = generation * 2^24 + slot
//
// A body's data lives at its slot in every column, so the slot is the index
// and lookups need no indirection. When a body is removed its slot is reused,
// and the generation, a count of how many times that slot has been handed
// out, tells an old id from the new body's: an old handle stops matching the
// moment its body is removed, instead of quietly reaching whatever came next.
//
// WHY 53 BITS. An id has to be one plain number, so that `a.id === b.id`, Map
// keys and JSON all simply work, and a JavaScript number holds whole numbers
// exactly only up to 2^53. So 24 bits of slot (16,777,216 bodies: the same
// limit as Winding's entities, derived the same way, from the field) and 29 of
// generation, 536,870,911 reuses of one slot. Winding's handles are 32 bits,
// to fit a Uint32Array, which leaves 8 for the generation: a slot retires
// after 255 reuses, so a world spawning and removing bodies all day would
// retire slots all day and never get its slot count back. At 29 bits, a slot
// reused every step at 60 Hz lasts 280 years. JavaScript's bitwise operators
// work on 32 bits, so `id & 0xffffff` still gives the slot (the low 24 of the
// low 32), and the generation is one exact division.
//
// A slot whose generation runs out is retired: marked used by nobody, never
// handed out again, so no old id can match anything after it.

export const SLOT_BITS = 24;
export const SLOT_SPAN = 16777216;                // 2^24
export const MAX_SLOTS = SLOT_SPAN;
export const MAX_GENERATION = 536870911;          // 2^29 - 1
export const MAX_ID = MAX_GENERATION * SLOT_SPAN + (SLOT_SPAN - 1);   // 2^53 - 1

const FREE = 0, USED = 1, RETIRED = 2;

/**
 * Hands out slots and generations, and tracks how far up the slots in use
 * reach (`top`), which is how far the columns must reach.
 *
 * Freed slots go on a stack and are reused first, newest first. A slot at or
 * above `top` needs no entry: everything from `top` up is free by definition.
 * So when the highest body is removed and `top` falls, entries left on the
 * stack for slots above it are stale, and are skipped when popped. That can't
 * hand a slot out twice: a fresh slot is only taken from `top` once the stack
 * is empty, so no stale entry for it can be left behind, and an entry is only
 * used if its slot is still free and below `top`.
 */
export class Slots {
  constructor() {
    this.capacity = 0;
    this.top = 0;
    this.count = 0;
    /** The most slots there can be. A field, not the constant, so a test can lower it. */
    this.limit = MAX_SLOTS;
    /** FREE, USED or RETIRED per slot. */
    this.state = new Uint8Array(0);
    /** A used slot's generation; a free slot's next one. 0 means never used. */
    this.generations = new Uint32Array(0);
    this.free = new Int32Array(0);
    this.freeCount = 0;
    /**
     * The generation a slot starts at when it is first used: always above every
     * generation any slot dropped by a shrink ever had. A dropped slot's
     * generations are gone with it, so without this, a regrown slot would start
     * again at 1 and an id from before the shrink could come back to life.
     */
    this.floor = 1;
  }

  /** A free slot, or -1 if every slot below `capacity` is taken and the columns must grow first. */
  take() {
    while (this.freeCount > 0) {
      const s = this.free[--this.freeCount];
      if (s < this.top && this.state[s] === FREE) return this._use(s);
    }
    if (this.top === this.capacity) return -1;
    return this._use(this.top++);
  }

  _use(s) {
    this.state[s] = USED;
    if (this.generations[s] < this.floor) this.generations[s] = this.floor;
    this.count++;
    return s;
  }

  release(s) {
    this.state[s] = FREE;
    this.count--;
    const next = this.generations[s] + 1;
    if (next > MAX_GENERATION) {
      this.state[s] = RETIRED;
    } else {
      this.generations[s] = next;
      this.free[this.freeCount++] = s;
    }
    while (this.top > 0 && this.state[this.top - 1] === FREE) this.top--;
  }

  id(s) {
    return this.generations[s] * SLOT_SPAN + s;
  }

  /** The slot of a live id, or -1. `id` must already be a whole number from 0 to MAX_ID. */
  slotOf(id) {
    const s = id & 0xffffff;
    if (s >= this.top || this.state[s] !== USED) return -1;
    return this.generations[s] === (id - s) / SLOT_SPAN ? s : -1;
  }

  /**
   * Room for `capacity` slots. Growing keeps everything; shrinking may only cut
   * free slots from above `top`, and raises `floor` past their generations.
   */
  resize(capacity) {
    if (capacity < this.capacity) {
      for (let s = capacity; s < this.capacity; s++) {
        if (this.generations[s] > this.floor) this.floor = this.generations[s];
      }
      // Entries at or above top are stale; keep only the live ones.
      let kept = 0;
      for (let i = 0; i < this.freeCount; i++) {
        const s = this.free[i];
        if (s < this.top && this.state[s] === FREE) this.free[kept++] = s;
      }
      this.freeCount = kept;
    }
    this.state = resized(this.state, capacity);
    this.generations = resized(this.generations, capacity);
    this.free = resized(this.free, capacity);
    this.capacity = capacity;
  }
}

/** A copy of `array` with room for `length` items: longer is zero-filled, shorter is cut. */
export function resized(array, length) {
  const next = new array.constructor(length);
  next.set(length < array.length ? array.subarray(0, length) : array);
  return next;
}
