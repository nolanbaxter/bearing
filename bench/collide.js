// What a contact costs: microseconds per pair, for every pair of shapes.
// Run: node bench/collide.js
//
// Two cases a pair: touching (overlapping by a little, as resting contacts
// do), and apart but within the margin, where a speculative contact is made.
// Box pairs apart also take the corner fallback some of the time, which is
// why they cost more there. 2,000 random placements a case, cycled until each
// timing block lasts long enough for the timer not to matter, five blocks,
// median reported.

import { collide, Shape, Manifold, SPHERE, CAPSULE, BOX } from '../src/collide.js';

let seed = 1;
const random = () => (seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648;

function shape(kind) {
  const s = new Shape();
  s.kind = kind;
  if (kind === BOX) s.half.set([0.5, 0.5, 0.5]);
  else if (kind === SPHERE) s.half[0] = 0.5;
  else { s.half[0] = 0.3; s.half[1] = 0.6; }
  const q = [random() - 0.5, random() - 0.5, random() - 0.5, random() - 0.5];
  const l = Math.hypot(...q);
  return s.place(0, 0, 0, q.map((x) => x / l));
}

/** A placement of b against a whose deepest separation lands near `target`, found by moving b along a random direction. */
function placed(ka, kb, target) {
  const m = new Manifold();
  for (;;) {
    const a = shape(ka), b = shape(kb);
    const d = [random() - 0.5, random() - 0.5, random() - 0.5];
    const l = Math.hypot(...d);
    let lo = 0, hi = 4;
    for (let i = 0; i < 50; i++) {
      const t = (lo + hi) / 2;
      b.center.set(d.map((x) => (x / l) * t));
      collide(a, b, 10, 0.005, m);
      const deepest = Math.min(...m.separations.subarray(0, m.count));
      if (deepest < target) lo = t; else hi = t;
    }
    b.center.set(d.map((x) => (x / l) * hi));
    if (collide(a, b, 0.02, 0.005, m) > 0) return [a, b];
  }
}

const NAMES = ['sphere', 'capsule', 'box'];
const out = new Manifold();
console.log(`collide(), µs per pair (median of 5 blocks), ${typeof process !== 'undefined' ? `Node ${process.version}` : ''}`);
console.log('pair               touching   apart, within margin');
for (let i = 0; i < 3; i++) {
  for (let j = i; j < 3; j++) {
    const row = [];
    for (const target of [-0.01, 0.01]) {
      const pairs = Array.from({ length: 2000 }, () => placed(i, j, target));
      for (let w = 0; w < 3; w++) for (const [a, b] of pairs) collide(a, b, 0.02, 0.005, out);
      const times = [];
      for (let block = 0; block < 5; block++) {
        let n = 0;
        const t0 = performance.now();
        while (performance.now() - t0 < 100) {
          for (const [a, b] of pairs) collide(a, b, 0.02, 0.005, out);
          n += pairs.length;
        }
        times.push(((performance.now() - t0) * 1000) / n);
      }
      times.sort((x, y) => x - y);
      row.push(times[2].toFixed(3).padStart(8));
    }
    console.log(`${`${NAMES[i]}-${NAMES[j]}`.padEnd(18)} ${row[0]}   ${row[1]}`);
  }
}
