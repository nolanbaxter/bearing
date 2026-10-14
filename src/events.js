// Contact events: which pairs began touching in the last step, and which
// ended, kept as numbers until world.contacts() asks for them.
//
// The step writes here only when a pair changes: a pile at rest writes
// nothing, and a step allocates nothing, since the columns are reused and
// grow by doubling. A begin holds where the two met, which way, and how fast;
// an end holds only who. Bodies are kept by id, not slot, so an end for a
// body removed since still names it, and its handle reads as dead.
//
// Read after the step, never called back from inside it (rule 65): a
// callback that removed a body mid-solve would pull it out of arrays the
// solver is halfway through.

import { resized } from './core/slots.js';

export const BEGIN = 0, END = 1;

export class Events {
  constructor() {
    this.count = 0;
    this.capacity = 0;
    this._alloc(0);
  }

  _alloc(c) {
    const grow = (name, Type, n) => { this[name] = this[name] ? resized(this[name], n) : new Type(n); };
    grow('type', Uint8Array, c);
    grow('idA', Float64Array, c); grow('idB', Float64Array, c);
    grow('point', Float64Array, 3 * c); grow('normal', Float64Array, 3 * c);
    grow('speed', Float64Array, c);
    this.capacity = c;
  }

  _add(type, idA, idB) {
    if (this.count === this.capacity) this._alloc(Math.max(16, 2 * this.capacity));
    const e = this.count++;
    this.type[e] = type; this.idA[e] = idA; this.idB[e] = idB;
    return e;
  }

  /** Bodies idA and idB began touching at (px, py, pz), the normal from A to B, meeting at `speed` m/s. */
  begin(idA, idB, px, py, pz, nx, ny, nz, speed) {
    const e = this._add(BEGIN, idA, idB), e3 = 3 * e;
    this.point[e3] = px; this.point[e3 + 1] = py; this.point[e3 + 2] = pz;
    this.normal[e3] = nx; this.normal[e3 + 1] = ny; this.normal[e3 + 2] = nz;
    this.speed[e] = speed;
  }

  end(idA, idB) {
    this._add(END, idA, idB);
  }

  /**
   * Gives storage back once a step's events fill a quarter of it or less,
   * down to the power of two growth would have picked: the world's rule, so a
   * pile landing all at once leaves nothing behind.
   */
  trim() {
    if (this.capacity <= 16 || this.count > this.capacity / 4) return;
    let c = 16;
    while (c < this.count) c *= 2;
    this._alloc(c);
  }
}
