// Which broadphase, and how fat its boxes: measured on recorded motion.
// Run: node bench/broadphase.js           (1,000, 10,000 and 50,000 bodies; several minutes)
//      node bench/broadphase.js --quick   (1,000 and 10,000, fewer rounds)
//      node bench/broadphase.js --only=1000
//      node bench/broadphase.js --all     (every margin and reach, as when FAT and AHEAD were chosen)
//
// By default the tree runs as shipped (FAT, AHEAD) and with each of the two
// halved and doubled, against sap and sweep: enough to say whether the choice
// still holds after a change, and which way to move if it doesn't.
//
// WHY RECORDED MOTION. A broadphase timed inside whole steps is timed with
// the solver around it, and the solver will change: it is most of a step
// today and will be made faster later, which would shift any comparison
// made now. So each scene is run once in Bearing as it is, and every body's
// box is recorded at every step, exactly as the broadphase is handed it. Each
// candidate then replays the same boxes on its own. The motion is real (piles
// settle, bodies bounce off each other), but nothing outside the broadphase
// is timed, and a later change to the solver changes the recording only as
// much as it changes how things actually move.
//
// WHAT IS TIMED. Everything between "here are this step's boxes" and "here
// are the pairs the narrowphase should look at": re-filing, finding new
// pairs, keeping the persistent pair list (pairs.js, the same for every
// candidate, since warm starting needs pairs to last from step to step), and
// the loop that drops pairs that no longer hold and checks the rest against
// this step's boxes. The output of every candidate is the same list of pairs,
// checked against the answer of testing every pair before anything is timed.
//
// THE CANDIDATES, each written with the same care:
//   tree      src/broadphase.js as shipped: a dynamic AABB tree with fat
//             boxes, static bodies in a tree of their own (Box2D 3's design).
//             Run at several fat margins to choose FAT, and with the fat box
//             reaching 2 or 4 steps ahead along the velocity (+2v, +4v) to
//             choose AHEAD.
//   sap       incremental sweep and prune on all three axes (Bullet's
//             btAxisSweep3): every box's ends kept sorted on each axis, a
//             pair added or dropped as two ends pass each other.
//   sweep     sort and sweep on one axis each step (cannon-es and Box2D 2.0):
//             re-sort by insertion (the order barely changes from a step to
//             the next), then pair whatever overlaps along it.
// And, for raycasts and shape casts to come (milestone 8), box queries against
// each: the tree walks itself; the sorted ones scan along their axis.
//
// THE SCENES, chosen for what is coming as much as for what is here:
//   pile      mixed shapes dropped in a heap: dense, all moving, then settling
//   fliers    no gravity, bodies flying about in a closed room: sparse, fast
//   towers    stacks of ten boxes standing still: what sleeping bodies (milestone 6) look like
//   level     half the bodies static, scattered like a level, the rest falling through it (milestone 9)
// 50,000 bodies is the 10,000 recording tiled five times side by side, since
// recording 50,000 in Bearing takes too long; the local motion is the same.
//
// RESULTS (i7-1260P, Node 22, 2026-10-13), the runs that chose the broadphase.
// µs per step at 10,000 bodies, median of 3 rounds (the no-reach column, of 5):
//
//               tree as chosen     tree 1/4,      sap       sweep
//               (1/4, 16 ahead)    no reach
//   pile            10,893          22,244       31,878    22,105
//   fliers           4,324          18,799        6,498     8,563
//   towers             271             258        4,916     6,268
//   level            1,776          15,390       22,272     5,933
//   one query      0.7-11 µs                   44-64 µs  15-28 µs
//
// The tree as chosen won every scene at 10,000, at rest by 18 to 23 times, and
// answered queries 1.4 to 63 times faster (least against sweep in the dense
// pile). Without reaching ahead it only tied sweep in the pile and was nearly 9
// times slower in the level: re-filing is what a moving tree costs. At 1,000
// bodies it won everything but the pile, where sap was 20% faster (552 against
// 663 µs). Margin 1/8 won the pile by 7% but lost the fliers and the level by
// 13-15%; 1/2 lost the towers by 13 times, as fat boxes a body's width apart
// overlap. Reach 16 beat 8, and 8 beat 4, in the fliers and the level at both
// sizes and in the 10,000 pile, and was within 5% elsewhere; reaches of 0 and 2
// lost every moving scene at 1,000.
// The 50,000 run was stopped, by choice, partway: the tree's lead grew from
// 1,000 to 10,000, so it could only have confirmed. Not yet run on the Ryzen.

import os from 'node:os';
import { World } from '../src/bearing.js';
import { Broadphase, FAT, AHEAD } from '../src/broadphase.js';
import { Pairs } from '../src/pairs.js';
import { Slots } from '../src/core/slots.js';

const quick = process.argv.includes('--quick'), all = process.argv.includes('--all');
const only = process.argv.find((a) => a.startsWith('--only='));
const STEPS = 120, DT = 1 / 60;
const SCALES = only ? [Number(only.slice(7))] : quick ? [1000, 10000] : [1000, 10000, 50000];
const ROUNDS = quick ? 3 : 5;          // measured rounds, after one warm-up round that isn't
// --all: the grid FAT and AHEAD were chosen from. A margin of 0 or 1 and a
// reach of 2 steps were each beaten everywhere at 1,000 bodies (see RESULTS).
const MARGINS = [1 / 16, 1 / 8, 1 / 4, 1 / 2];
const AHEADS = [0, 4, 8, 16];          // steps of velocity the fat box reaches ahead
const DYNAMIC = 0, STATIC = 1;

/** A seeded generator, so every run records the same scenes. */
function random(seed) {
  let s = seed >>> 0;
  return () => {
    s = (Math.imul(s, 1664525) + 1013904223) >>> 0;
    return s / 4294967296;
  };
}

// ---- the scenes ----

const shapeOf = (rand, scale = 1) => {
  const k = rand(), s = scale * (0.6 + rand() * 0.4);
  if (k < 0.5) return { size: [s, s * (0.6 + rand() * 0.4), s] };
  if (k < 0.8) return { shape: 'sphere', radius: s / 2 };
  return { shape: 'capsule', radius: s / 4, height: s };
};
const turn = (rand) => [rand() - 0.5, rand() - 0.5, rand() - 0.5, rand() - 0.5];

const SCENES = {
  pile(n, rand) {
    const world = new World();
    const side = Math.ceil(Math.sqrt(n / 8));
    world.addBody({ type: 'static', size: [side * 4, 1, side * 4], position: [0, -0.5, 0] });
    for (let i = 0; i < n; i++) {
      const x = i % side, z = Math.floor(i / side) % side, y = Math.floor(i / (side * side));
      world.addBody({ ...shapeOf(rand), rotation: turn(rand), position: [(x - side / 2) * 1.2, 1 + y * 1.2, (z - side / 2) * 1.2] });
    }
    return world;
  },
  fliers(n, rand) {
    const world = new World({ gravity: [0, 0, 0] });
    const room = Math.cbrt(n) * 4, h = room / 2;
    for (const [axis, sign] of [[0, -1], [0, 1], [1, -1], [1, 1], [2, -1], [2, 1]]) {
      const size = [room + 2, room + 2, room + 2], position = [0, 0, 0];
      size[axis] = 1; position[axis] = sign * (h + 0.5);
      world.addBody({ type: 'static', size, position });
    }
    for (let i = 0; i < n; i++) {
      world.addBody({
        ...shapeOf(rand), rotation: turn(rand), restitution: 0.5,
        position: [(rand() - 0.5) * (room - 2), (rand() - 0.5) * (room - 2), (rand() - 0.5) * (room - 2)],
        velocity: [(rand() - 0.5) * 6, (rand() - 0.5) * 6, (rand() - 0.5) * 6],
        angularVelocity: [(rand() - 0.5) * 4, (rand() - 0.5) * 4, (rand() - 0.5) * 4],
      });
    }
    return world;
  },
  towers(n, rand) {
    const world = new World();
    const stacks = Math.ceil(n / 10), side = Math.ceil(Math.sqrt(stacks));
    world.addBody({ type: 'static', size: [side * 3, 1, side * 3], position: [0, -0.5, 0] });
    for (let i = 0; i < n; i++) {
      const t = Math.floor(i / 10), x = t % side, z = Math.floor(t / side);
      world.addBody({ position: [(x - side / 2) * 2, 0.5 + (i % 10), (z - side / 2) * 2] });
    }
    rand();
    return world;
  },
  level(n, rand) {
    const world = new World();
    const statics = Math.floor(n / 2), side = Math.sqrt(statics) * 3;
    world.addBody({ type: 'static', size: [side + 4, 1, side + 4], position: [0, -0.5, 0] });
    for (let i = 1; i < statics; i++) {
      world.addBody({
        type: 'static', size: [1 + rand() * 3, 0.3 + rand(), 1 + rand() * 3], rotation: turn(rand),
        position: [(rand() - 0.5) * side, rand() * 12, (rand() - 0.5) * side],
      });
    }
    for (let i = statics; i < n; i++) {
      world.addBody({ ...shapeOf(rand), rotation: turn(rand), position: [(rand() - 0.5) * side, 14 + rand() * 10, (rand() - 0.5) * side] });
    }
    return world;
  },
};

/**
 * Runs a scene in Bearing and keeps every body's box at every step, as
 * step() hands them to the broadphase, in Float32 to keep 10,000 bodies'
 * recording to 30 MB a scene. Every candidate replays the same rounded
 * numbers, so the rounding favours none.
 */
function record(name, n) {
  const world = SCENES[name](n, random(n * 31 + name.length));
  const count = world._slots.top;
  const boxes = new Float32Array(STEPS * count * 6), velocities = new Float32Array(STEPS * count * 3);
  const bounds = world._bounds;
  let step = 0;
  world._bounds = function (dt) {
    bounds.call(this, dt);
    boxes.set(this._box.subarray(0, count * 6), step * count * 6);
    velocities.set(this._velocity.subarray(0, count * 3), step * count * 3);
    step++;
  };
  for (let s = 0; s < STEPS; s++) world.step(DT);
  let x0 = Infinity, x1 = -Infinity, z0 = Infinity, z1 = -Infinity;
  for (let i = 0; i < boxes.length; i += 6) {
    x0 = Math.min(x0, boxes[i]); x1 = Math.max(x1, boxes[i + 3]);
    z0 = Math.min(z0, boxes[i + 2]); z1 = Math.max(z1, boxes[i + 5]);
  }
  return {
    name, n: count, boxes, velocities, width: x1 - x0 + 10, depth: z1 - z0 + 10,
    type: world._type.slice(0, count), size: world._size.slice(0, count),
  };
}

/**
 * What a candidate sees of the world: the same fields, filled from a
 * recording, tiled side by side `tiles` times.
 */
class Stage {
  constructor(trace, tiles) {
    const per = trace.n, n = per * tiles;
    this.trace = trace;
    this.tiles = tiles;
    this.n = n;
    this._slots = new Slots();
    this._slots.resize(n);
    for (let i = 0; i < n; i++) this._slots.take();
    this._type = new Uint8Array(n);
    this._size = new Float64Array(n);
    this._layers = new Uint32Array(n).fill(1);
    this._mask = new Uint32Array(n).fill(0xffffffff);
    // What the pair list also asks of a world since milestones 10 and 12: no
    // sensors here, and no joints (a 0 head is a body with none).
    this._sensor = new Uint8Array(n);
    this._jointHead = new Int32Array(n);
    this._box = new Float64Array(n * 6);
    this._velocity = new Float64Array(n * 3);
    this.offsets = [];
    // Every moving body awake, as while a scene is being recorded: the
    // broadphase walks the awake list, and nothing here sleeps.
    this._awakeList = new Int32Array(n);
    this._awakeCount = 0;
    const columns = Math.ceil(Math.sqrt(tiles));
    for (let t = 0; t < tiles; t++) {
      this._type.set(trace.type, t * per);
      this._size.set(trace.size, t * per);
      this.offsets.push([(t % columns) * trace.width, Math.floor(t / columns) * trace.depth]);
    }
    for (let s = 0; s < n; s++) if (this._type[s] !== STATIC) this._awakeList[this._awakeCount++] = s;
  }

  /** Nothing sleeps here, so there is nothing to wake. */
  _wake() {}

  load(step) {
    const per = this.trace.n * 6, from = this.trace.boxes, box = this._box;
    for (let t = 0; t < this.tiles; t++) {
      const [dx, dz] = this.offsets[t], o = t * per, f = step * per;
      for (let i = 0; i < per; i += 6) {
        box[o + i] = from[f + i] + dx; box[o + i + 1] = from[f + i + 1]; box[o + i + 2] = from[f + i + 2] + dz;
        box[o + i + 3] = from[f + i + 3] + dx; box[o + i + 4] = from[f + i + 4]; box[o + i + 5] = from[f + i + 5] + dz;
      }
      const vs = this.trace.n * 3;
      for (let i = 0; i < vs; i++) this._velocity[t * vs + i] = this.trace.velocities[step * vs + i];
    }
  }
}

const overlap = (box, a, b) => {
  const i = 6 * a, j = 6 * b;
  return box[i] <= box[j + 3] && box[i + 3] >= box[j] && box[i + 1] <= box[j + 4]
    && box[i + 4] >= box[j + 1] && box[i + 2] <= box[j + 5] && box[i + 5] >= box[j + 2];
};

/** Removes pair (x, y) from the list if it is there. */
function drop(pairs, x, y) {
  const at = pairs._find(x < y ? x : y, x < y ? y : x);
  if (at >= 0) pairs.remove(pairs.index[at]);
}

// ---- the candidates ----

class TreeCandidate {
  constructor(stage, margin, ahead) {
    this.broadphase = new Broadphase(margin, ahead);
    this.broadphase.resize(stage.n);
    this.pairs = new Pairs(stage);
    this.refiled = 0;
  }

  build(stage) {
    for (let s = 0; s < stage.n; s++) this.broadphase.add(stage, s);
    this.broadphase.candidates(stage, this.pairs);
  }

  step(stage) {
    const bp = this.broadphase, pairs = this.pairs, box = stage._box;
    bp.update(stage, DT);
    this.refiled += bp.movedCount;
    bp.candidates(stage, pairs);
    // narrow()'s loop, up to where it would call the narrowphase.
    let exact = 0;
    for (let p = 0; p < pairs.count;) {
      if (!pairs.holds(p, bp)) { pairs.remove(p); continue; }
      if (overlap(box, pairs.a[p], pairs.b[p])) exact++;
      p++;
    }
    return exact;
  }

  query(x0, y0, z0, x1, y1, z1, visit) {
    for (const tree of this.broadphase.trees) tree.query(x0, y0, z0, x1, y1, z1, visit);
  }
}

/**
 * Incremental sweep and prune on three axes. Each axis keeps both ends of
 * every box in one sorted array: value, and owner = body * 2 + (1 for a max
 * end). When a min end passes below another body's max end on one axis, the
 * two now overlap on that axis, and become a pair if they already overlap on
 * the other two (compared by where their ends sit in those axes' arrays);
 * when a max end passes below another's min, they part. At equal values a
 * min sorts first, so touching boxes overlap, as everywhere else.
 */
class SapCandidate {
  constructor(stage) {
    const n = stage.n;
    this.pairs = new Pairs(stage);
    this.value = [0, 1, 2].map(() => new Float64Array(2 * n));
    this.owner = [0, 1, 2].map(() => new Int32Array(2 * n));
    this.lo = [0, 1, 2].map(() => new Int32Array(n));
    this.hi = [0, 1, 2].map(() => new Int32Array(n));
    this.type = stage._type;
    this.length = 2 * n;
  }

  build(stage) {
    const n = stage.n, box = stage._box;
    for (let k = 0; k < 3; k++) {
      const ends = new Int32Array(2 * n);
      for (let i = 0; i < 2 * n; i++) ends[i] = i;
      const at = (e) => box[6 * (e >> 1) + k + 3 * (e & 1)];
      ends.sort((a, b) => at(a) - at(b) || (a & 1) - (b & 1));
      for (let i = 0; i < 2 * n; i++) {
        const e = ends[i];
        this.value[k][i] = at(e);
        this.owner[k][i] = e;
        if (e & 1) this.hi[k][e >> 1] = i; else this.lo[k][e >> 1] = i;
      }
    }
    // The first pairs: one sweep along x, testing y and z by position.
    const active = new Int32Array(n), place = new Int32Array(n), owner = this.owner[0];
    let count = 0;
    for (let i = 0; i < 2 * n; i++) {
      const h = owner[i] >> 1;
      if (owner[i] & 1) {
        const last = active[--count];
        active[place[h]] = last;
        place[last] = place[h];
        continue;
      }
      for (let j = 0; j < count; j++) {
        const o = active[j];
        if (this.overlap2(0, h, o)) this.found(h, o);
      }
      place[h] = count;
      active[count++] = h;
    }
  }

  found(h, o) {
    if (this.type[h] !== DYNAMIC && this.type[o] !== DYNAMIC) return;
    this.pairs.found(h, o);
  }

  lost(h, o) {
    if (this.type[h] !== DYNAMIC && this.type[o] !== DYNAMIC) return;
    drop(this.pairs, h, o);
  }

  /** Whether h and o overlap on the two axes other than k, by where their ends sit. */
  overlap2(k, h, o) {
    const j = k === 0 ? 1 : 0, l = k === 2 ? 1 : 2;
    return this.lo[j][h] < this.hi[j][o] && this.lo[j][o] < this.hi[j][h]
      && this.lo[l][h] < this.hi[l][o] && this.lo[l][o] < this.hi[l][h];
  }

  step(stage) {
    const n = stage.n, box = stage._box, type = this.type, pairs = this.pairs;
    for (let h = 0; h < n; h++) {
      if (type[h] === STATIC) continue;
      for (let k = 0; k < 3; k++) {
        const v = this.value[k], lo = this.lo[k][h], hi = this.hi[k][h];
        const min = box[6 * h + k], max = box[6 * h + k + 3];
        const dmin = min - v[lo], dmax = max - v[hi];
        v[lo] = min; v[hi] = max;
        // Growing first, then shrinking, as Bullet does, so an end never
        // has to pass its own body's other end.
        if (dmin < 0) this.minDown(k, h);
        if (dmax > 0) this.maxUp(k, h);
        if (dmin > 0) this.minUp(k, h);
        if (dmax < 0) this.maxDown(k, h);
      }
    }
    let exact = 0;
    for (let p = 0; p < pairs.count; p++) if (overlap(box, pairs.a[p], pairs.b[p])) exact++;
    return exact;
  }

  minDown(k, h) {
    const v = this.value[k], w = this.owner[k], lo = this.lo[k], hi = this.hi[k];
    let i = lo[h];
    const val = v[i];
    while (i > 0) {
      const pv = v[i - 1], pw = w[i - 1];
      if (!(pv > val || (pv === val && (pw & 1)))) break;
      const o = pw >> 1;
      if (pw & 1) {
        if (this.overlap2(k, h, o)) this.found(h, o);
        hi[o]++;
      } else {
        lo[o]++;
      }
      v[i] = pv; w[i] = pw; i--;
    }
    v[i] = val; w[i] = 2 * h; lo[h] = i;
  }

  minUp(k, h) {
    const v = this.value[k], w = this.owner[k], lo = this.lo[k], hi = this.hi[k], end = this.length;
    let i = lo[h];
    const val = v[i];
    while (i + 1 < end) {
      const nv = v[i + 1], nw = w[i + 1];
      if (!(nv < val)) break;
      const o = nw >> 1;
      if (nw & 1) {
        this.lost(h, o);
        hi[o]--;
      } else {
        lo[o]--;
      }
      v[i] = nv; w[i] = nw; i++;
    }
    v[i] = val; w[i] = 2 * h; lo[h] = i;
  }

  maxUp(k, h) {
    const v = this.value[k], w = this.owner[k], lo = this.lo[k], hi = this.hi[k], end = this.length;
    let i = hi[h];
    const val = v[i];
    while (i + 1 < end) {
      const nv = v[i + 1], nw = w[i + 1];
      if (!(nv < val || (nv === val && !(nw & 1)))) break;
      const o = nw >> 1;
      if (nw & 1) {
        hi[o]--;
      } else {
        if (this.overlap2(k, h, o)) this.found(h, o);
        lo[o]--;
      }
      v[i] = nv; w[i] = nw; i++;
    }
    v[i] = val; w[i] = 2 * h + 1; hi[h] = i;
  }

  maxDown(k, h) {
    const v = this.value[k], w = this.owner[k], lo = this.lo[k], hi = this.hi[k];
    let i = hi[h];
    const val = v[i];
    while (i > 0) {
      const pv = v[i - 1], pw = w[i - 1];
      if (!(pv > val)) break;
      const o = pw >> 1;
      if (pw & 1) {
        hi[o]++;
      } else {
        this.lost(h, o);
        lo[o]++;
      }
      v[i] = pv; w[i] = pw; i--;
    }
    v[i] = val; w[i] = 2 * h + 1; hi[h] = i;
  }

  /** Every body whose min on x is at most the query's max, by binary search, then checked. */
  query(x0, y0, z0, x1, y1, z1, visit) {
    const v = this.value[0], w = this.owner[0];
    let a = 0, b = this.length;
    while (a < b) { const m = (a + b) >> 1; if (v[m] <= x1) a = m + 1; else b = m; }
    const vy = this.value[1], vz = this.value[2];
    for (let i = 0; i < a; i++) {
      if (w[i] & 1) continue;
      const h = w[i] >> 1;
      if (v[this.hi[0][h]] < x0) continue;
      if (vy[this.lo[1][h]] > y1 || vy[this.hi[1][h]] < y0 || vz[this.lo[2][h]] > z1 || vz[this.hi[2][h]] < z0) continue;
      visit(h);
    }
  }
}

/**
 * Sort and sweep on one axis, chosen once as the one the bodies spread
 * along most. Every step: insertion sort by the min end (nearly sorted
 * already, so close to one pass), then for each body, every later body whose
 * min is below its max, tested on the other two axes.
 */
class SweepCandidate {
  constructor(stage) {
    this.box = stage._box;
    this.pairs = new Pairs(stage);
    this.order = new Int32Array(stage.n);
    this.axis = 0;
  }

  build(stage) {
    const n = stage.n, box = stage._box;
    let best = -1;
    for (let k = 0; k < 3; k++) {
      let sum = 0, sum2 = 0;
      for (let s = 0; s < n; s++) { const c = box[6 * s + k] + box[6 * s + k + 3]; sum += c; sum2 += c * c; }
      const variance = sum2 / n - (sum / n) * (sum / n);
      if (variance > best) { best = variance; this.axis = k; }
    }
    for (let s = 0; s < n; s++) this.order[s] = s;
    const k = this.axis;
    this.order.sort((a, b) => box[6 * a + k] - box[6 * b + k]);
    this.step(stage);
  }

  step(stage) {
    const n = stage.n, box = stage._box, order = this.order, k = this.axis, type = stage._type, pairs = this.pairs;
    for (let i = 1; i < n; i++) {
      const s = order[i], v = box[6 * s + k];
      let j = i - 1;
      while (j >= 0 && box[6 * order[j] + k] > v) { order[j + 1] = order[j]; j--; }
      order[j + 1] = s;
    }
    const j1 = k === 0 ? 1 : 0, j2 = k === 2 ? 1 : 2;
    for (let i = 0; i < n; i++) {
      const s = order[i], max = box[6 * s + k + 3], a = 6 * s;
      for (let j = i + 1; j < n; j++) {
        const t = order[j], b = 6 * t;
        if (box[b + k] > max) break;
        if (type[s] !== DYNAMIC && type[t] !== DYNAMIC) continue;
        if (box[a + j1] > box[b + j1 + 3] || box[b + j1] > box[a + j1 + 3]
          || box[a + j2] > box[b + j2 + 3] || box[b + j2] > box[a + j2 + 3]) continue;
        pairs.found(s, t);
      }
    }
    // Pairs found in an earlier step that no longer overlap are dropped.
    let exact = 0;
    for (let p = 0; p < pairs.count;) {
      if (!overlap(box, pairs.a[p], pairs.b[p])) { pairs.remove(p); continue; }
      exact++;
      p++;
    }
    return exact;
  }

  query(x0, y0, z0, x1, y1, z1, visit) {
    const box = this.box, order = this.order, k = this.axis, upper = k === 0 ? x1 : k === 1 ? y1 : z1;
    let a = 0, b = order.length;
    while (a < b) { const m = (a + b) >> 1; if (box[6 * order[m] + k] <= upper) a = m + 1; else b = m; }
    for (let i = 0; i < a; i++) {
      const o = 6 * order[i];
      if (box[o] > x1 || box[o + 3] < x0 || box[o + 1] > y1 || box[o + 4] < y0 || box[o + 2] > z1 || box[o + 5] < z0) continue;
      visit(order[i]);
    }
  }
}

const tree = (m, ahead) => ({
  name: `tree ${fraction(m)}${ahead ? ` +${ahead}v` : ''}`, margin: m,
  make: (stage) => new TreeCandidate(stage, m, ahead),
});
const neighbours = [tree(FAT, AHEAD), tree(FAT / 2, AHEAD), tree(FAT * 2, AHEAD), tree(FAT, AHEAD / 2), tree(FAT, AHEAD * 2)];
const CANDIDATES = [
  ...(all ? AHEADS.flatMap((ahead) => MARGINS.map((m) => tree(m, ahead))) : neighbours)
    .filter((c, i, list) => list.findIndex((d) => d.name === c.name) === i),
  { name: 'sap', make: (stage) => new SapCandidate(stage) },
  { name: 'sweep', make: (stage) => new SweepCandidate(stage) },
];

function fraction(m) {
  return m === 0 ? '0' : m === 1 ? '1' : `1/${1 / m}`;
}

// ---- checking, then timing ----

/**
 * Every pair that should be handed on, by testing every pair: the answer each
 * candidate must give. Tiles stand 10 m apart, so no two bodies in different
 * tiles can touch, and the answer is the first tile's, repeated: 50 million
 * tests a step at 50,000 bodies instead of 1.25 billion. A candidate that paired
 * bodies across tiles would be wrong, and would fail here.
 */
function brute(stage) {
  const per = stage.trace.n, type = stage._type, box = stage._box, found = [];
  for (let a = 0; a < per; a++) {
    for (let b = a + 1; b < per; b++) {
      if (type[a] !== DYNAMIC && type[b] !== DYNAMIC) continue;
      if (overlap(box, a, b)) for (let t = 0; t < stage.tiles; t++) found.push((a + t * per) * 16777216 + b + t * per);
    }
  }
  return found.sort((x, y) => x - y);
}

function handedOn(candidate, stage) {
  const pairs = candidate.pairs, found = [];
  for (let p = 0; p < pairs.count; p++) {
    if (overlap(stage._box, pairs.a[p], pairs.b[p])) found.push(pairs.a[p] * 16777216 + pairs.b[p]);
  }
  return found.sort((x, y) => x - y);
}

/** Replays a recording through every candidate, checking its pairs at the given steps. */
function check(stage, at) {
  // Worked out once a step and shared by every candidate.
  const expected = new Map();
  for (const c of CANDIDATES) {
    stage.load(0);
    const candidate = c.make(stage);
    candidate.build(stage);
    for (let step = 1; step < STEPS; step++) {
      stage.load(step);
      candidate.step(stage);
      if (!at(step)) continue;
      if (!expected.has(step)) expected.set(step, brute(stage));
      const got = handedOn(candidate, stage), want = expected.get(step);
      if (got.length !== want.length || got.some((x, i) => x !== want[i])) {
        throw new Error(`${c.name} on ${stage.trace.name} (${stage.n}) is wrong at step ${step}: ${got.length} pairs, should be ${want.length}`);
      }
    }
  }
}

/** One replay: build time, then the time and output of every step. */
function run(c, stage) {
  stage.load(0);
  const candidate = c.make(stage);
  let t = performance.now();
  candidate.build(stage);
  const build = performance.now() - t;
  let total = 0, listed = 0, exact = 0;
  for (let step = 1; step < STEPS; step++) {
    stage.load(step);
    t = performance.now();
    exact += candidate.step(stage);
    total += performance.now() - t;
    listed += candidate.pairs.count;
  }
  const steps = STEPS - 1;
  return { build, step: total / steps, listed: listed / steps, exact: exact / steps, refiled: (candidate.refiled ?? 0) / steps, candidate };
}

/** Box queries the size of a body, at bodies' own places, on the last step's state. */
function queries(candidate, stage, count) {
  const box = stage._box, rand = random(7);
  let hits = 0;
  const visit = () => { hits++; };
  const t = performance.now();
  for (let q = 0; q < count; q++) {
    const s = Math.floor(rand() * stage.n), o = 6 * s;
    candidate.query(box[o], box[o + 1], box[o + 2], box[o + 3], box[o + 4], box[o + 5], visit);
  }
  return { time: ((performance.now() - t) * 1000) / count, hits };
}

const median = (xs) => [...xs].sort((a, b) => a - b)[xs.length >> 1];
const pad = (s, n) => String(s).padStart(n);

console.log(`Broadphase: ${os.cpus()[0].model.trim()}, Node ${process.versions.node}, ${STEPS} steps at 60 Hz, median of ${ROUNDS} rounds\n`);

const traces = {};
for (const n of new Set(SCALES.map((s) => Math.min(s, 10000)))) {
  for (const name of Object.keys(SCENES)) {
    const t = performance.now();
    traces[`${name} ${n}`] = record(name, n);
    process.stderr.write(`recorded ${name} at ${n} in ${((performance.now() - t) / 1000).toFixed(1)} s\n`);
  }
}

for (const n of SCALES) {
  const per = Math.min(n, 10000), tiles = n / per;
  console.log(`${n.toLocaleString('en')} bodies${tiles > 1 ? ` (the 10,000 recording, ${tiles} times)` : ''}`);
  console.log(`  scene    candidate        µs/step   build ms   pairs listed   to narrowphase   re-filed/step   µs/query`);
  for (const name of Object.keys(SCENES)) {
    const stage = new Stage(traces[`${name} ${per}`], tiles);
    // Correct first: every step at 1,000; a few steps above that, since testing every pair is n².
    check(stage, n <= 1000 ? () => true : (step) => step === 1 || step === 60 || step === STEPS - 1);
    const times = new Map(CANDIDATES.map((c) => [c.name, []]));
    let last = new Map();
    for (let round = 0; round <= ROUNDS; round++) {
      // Taking turns, starting with a different one each round, so a
      // machine warming up or slowing down favours none.
      for (let i = 0; i < CANDIDATES.length; i++) {
        const c = CANDIDATES[(i + round) % CANDIDATES.length];
        const r = run(c, stage);
        if (round > 0) times.get(c.name).push(r);
        last.set(c.name, r);
      }
    }
    for (const c of CANDIDATES) {
      const rs = times.get(c.name), r = last.get(c.name);
      const q = queries(r.candidate, stage, 2000);
      console.log(`  ${name.padEnd(8)} ${c.name.padEnd(14)} ${pad(median(rs.map((x) => x.step * 1000)).toFixed(0), 9)} ${pad(median(rs.map((x) => x.build)).toFixed(1), 10)}`
        + ` ${pad(r.listed.toFixed(0), 14)} ${pad(r.exact.toFixed(0), 16)} ${pad(c.margin === undefined ? '' : r.refiled.toFixed(0), 15)} ${pad(q.time.toFixed(2), 10)}`);
    }
    last = null;
  }
  console.log('');
}
