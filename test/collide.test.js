// The narrowphase: hand-worked cases for every pair, then thousands of random
// pairs held to a reference that shares no code with src/collide.js.
// Run: node test/collide.test.js

import assert from 'node:assert/strict';
import { collide, Shape, Manifold, SPHERE, CAPSULE, BOX } from '../src/collide.js';
import { quatAxisAngle } from '../src/core/math.js';
import { suite, test } from './test.js';

suite('collide');

/** A shape from addBody-style options. */
function make({ shape = 'box', size = [1, 1, 1], radius = 0.5, height = 2, position = [0, 0, 0], rotation = [0, 0, 0, 1] }) {
  const s = new Shape();
  if (shape === 'box') {
    s.kind = BOX;
    s.half.set([size[0] / 2, size[1] / 2, size[2] / 2]);
  } else if (shape === 'sphere') {
    s.kind = SPHERE;
    s.half[0] = radius;
  } else {
    s.kind = CAPSULE;
    s.half[0] = radius;
    s.half[1] = (height - 2 * radius) / 2;
  }
  return s.place(position[0], position[1], position[2], rotation);
}
const turn = (axis, radians) => {
  const l = Math.hypot(...axis);
  return quatAxisAngle(axis.map((x) => x / l), radians);
};

function run(a, b, margin = 0.2, tolerance = 0.005) {
  const m = new Manifold();
  collide(a, b, margin, tolerance, m);
  const points = [];
  for (let k = 0; k < m.count; k++) {
    points.push({ p: [...m.points.subarray(3 * k, 3 * k + 3)], s: m.separations[k], id: m.ids[k] });
  }
  return { count: m.count, normal: [...m.normal], points };
}

const EPS = 1e-9;
function near(a, b, what, eps = EPS) {
  for (let i = 0; i < a.length; i++) assert.ok(Math.abs(a[i] - b[i]) <= eps, `${what}[${i}]: ${a[i]} vs ${b[i]}`);
}
/** The expected points, in any order, each matched once, position and separation. */
function samePoints(got, expected, what) {
  assert.equal(got.length, expected.length, `${what}: ${got.length} points, expected ${expected.length}`);
  const left = [...got];
  for (const [p, s] of expected) {
    const i = left.findIndex((g) => Math.hypot(g.p[0] - p[0], g.p[1] - p[1], g.p[2] - p[2]) < 1e-9);
    assert.ok(i >= 0, `${what}: no point at ${p}; got ${got.map((g) => g.p.map((x) => x.toFixed(4)))}`);
    near([left[i].s], [s], `${what} separation at ${p}`);
    left.splice(i, 1);
  }
}

// ---- hand-worked cases ----

test('sphere and sphere: apart, within the margin, overlapping, concentric', () => {
  const a = make({ shape: 'sphere', radius: 1 });
  let r = run(a, make({ shape: 'sphere', radius: 0.5, position: [2, 0, 0] }), 1);
  near(r.normal, [1, 0, 0], 'normal');
  samePoints(r.points, [[[1.25, 0, 0], 0.5]], 'within margin');
  assert.equal(run(a, make({ shape: 'sphere', radius: 0.5, position: [3, 0, 0] }), 0.5).count, 0);
  r = run(a, make({ shape: 'sphere', radius: 0.5, position: [1.2, 0, 0] }));
  samePoints(r.points, [[[0.85, 0, 0], -0.3]], 'overlapping');
  // Nothing prefers a direction: up.
  r = run(a, make({ shape: 'sphere', radius: 0.5 }));
  near(r.normal, [0, 1, 0], 'concentric normal');
  samePoints(r.points, [[[0, 0.25, 0], -1.5]], 'concentric');
});

test('sphere and box: over a face, an edge and a corner, and with its centre inside', () => {
  const box = make({ size: [2, 2, 2] });
  let r = run(make({ shape: 'sphere', position: [0, 1.6, 0] }), box);
  near(r.normal, [0, -1, 0], 'face normal');
  samePoints(r.points, [[[0, 1.05, 0], 0.1]], 'face');
  r = run(make({ shape: 'sphere', position: [1.3, 1.3, 0] }), box);
  near(r.normal, [-Math.SQRT1_2, -Math.SQRT1_2, 0], 'edge normal');
  const d = 0.3 * Math.SQRT2;
  near([r.points[0].s], [d - 0.5], 'edge separation');
  r = run(make({ shape: 'sphere', position: [1.2, 1.2, 1.2] }), box);
  near([r.points[0].s], [0.2 * Math.sqrt(3) - 0.5], 'corner separation');
  // Inside, nearest the top face: out through it.
  r = run(make({ shape: 'sphere', position: [0, 0.9, 0] }), box);
  near(r.normal, [0, -1, 0], 'inside normal');
  samePoints(r.points, [[[0, 0.7, 0], -0.6]], 'inside');
  // The other way round, the normal turns round and nothing else changes.
  r = run(box, make({ shape: 'sphere', position: [0, 1.6, 0] }));
  near(r.normal, [0, 1, 0], 'box first');
  samePoints(r.points, [[[0, 1.05, 0], 0.1]], 'box first');
  // A turned box is the same box.
  r = run(make({ shape: 'sphere', position: [0, 1.6, 0] }), make({ size: [2, 2, 2], rotation: turn([0, 1, 0], 0.7) }));
  samePoints(r.points, [[[0, 1.05, 0], 0.1]], 'turned box');
});

test('sphere and capsule: beside it, past its end, and on its axis', () => {
  const capsule = make({ shape: 'capsule', radius: 0.5, height: 2 });   // segment y in [-0.5, 0.5]
  let r = run(make({ shape: 'sphere', position: [1.2, 0, 0] }), capsule);
  near(r.normal, [-1, 0, 0], 'beside normal');
  samePoints(r.points, [[[0.6, 0, 0], 0.2]], 'beside');
  r = run(make({ shape: 'sphere', position: [0, 1.8, 0] }), capsule, 0.5);
  near(r.normal, [0, -1, 0], 'past the end normal');
  samePoints(r.points, [[[0, 1.15, 0], 0.3]], 'past the end');
  // Exactly on the axis: any way off it will do, and it is always the same one.
  // (Elsewhere on the axis, rounding usually leaves a direction of its own.)
  r = run(make({ shape: 'sphere', position: [0, 0, 0] }), capsule);
  near(r.normal, [0, 0, -1], 'on the axis normal');
  near([r.points[0].s], [-1], 'on the axis separation');
});

test('capsule and capsule: side by side, crossed, end to end, half alongside', () => {
  const a = make({ shape: 'capsule' });   // segment y in [-0.5, 0.5]
  let r = run(a, make({ shape: 'capsule', position: [1.1, 0, 0] }));
  near(r.normal, [1, 0, 0], 'side by side normal');
  samePoints(r.points, [[[0.55, -0.5, 0], 0.1], [[0.55, 0.5, 0], 0.1]], 'side by side');
  r = run(a, make({ shape: 'capsule', position: [1.1, 0, 0], rotation: turn([1, 0, 0], Math.PI / 2) }));
  samePoints(r.points, [[[0.55, 0, 0], 0.1]], 'crossed');
  r = run(a, make({ shape: 'capsule', position: [0, 2.1, 0] }));
  near(r.normal, [0, 1, 0], 'end to end normal');
  samePoints(r.points, [[[0, 1.05, 0], 0.1]], 'end to end');
  r = run(a, make({ shape: 'capsule', position: [1.1, 0.6, 0] }));   // b's segment y in [0.1, 1.1]
  samePoints(r.points, [[[0.55, 0.1, 0], 0.1], [[0.55, 0.5, 0], 0.1]], 'half alongside');
});

test('capsule and box: lying on a face, standing on it, across an edge, and through it', () => {
  const box = make({ size: [4, 1, 4] });   // top face at y = 0.5
  const lying = turn([0, 0, 1], -Math.PI / 2);   // the capsule's axis along x
  let r = run(make({ shape: 'capsule', position: [0, 1.1, 0], rotation: lying }), box);
  near(r.normal, [0, -1, 0], 'lying normal');
  samePoints(r.points, [[[-0.5, 0.55, 0], 0.1], [[0.5, 0.55, 0], 0.1]], 'lying');
  r = run(make({ shape: 'capsule', position: [0, 1.6, 0] }), box);
  samePoints(r.points, [[[0, 0.55, 0], 0.1]], 'standing');
  // Along z, just over the edge where the top meets the +x side.
  r = run(make({ shape: 'capsule', position: [2.3, 0.8, 0], rotation: turn([1, 0, 0], Math.PI / 2) }), box);
  near(r.normal, [-Math.SQRT1_2, -Math.SQRT1_2, 0], 'edge normal');
  assert.equal(r.count, 1);
  near([r.points[0].s], [0.3 * Math.SQRT2 - 0.5], 'edge separation');
  // A long thin capsule through the box, 0.2 below its top: out through the top.
  r = run(make({ shape: 'capsule', radius: 0.3, height: 6, position: [0, 0.3, 0], rotation: lying }), box);
  near(r.normal, [0, -1, 0], 'through normal');
  samePoints(r.points, [[[-2, 0.25, 0], -0.5], [[2, 0.25, 0], -0.5]], 'through');
});

test('box and box: stacked, small on large, turned 45°, edge across edge, corner down', () => {
  const a = make({ size: [2, 2, 2] });
  let r = run(a, make({ size: [2, 2, 2], position: [0, 2.1, 0] }));
  near(r.normal, [0, 1, 0], 'stacked normal');
  samePoints(r.points, [[[-1, 1.05, -1], 0.1], [[1, 1.05, -1], 0.1], [[1, 1.05, 1], 0.1], [[-1, 1.05, 1], 0.1]], 'stacked');

  const big = make({ size: [4, 1, 4] }), small = make({ size: [1, 1, 1], position: [0, 1, 0] });
  const corners = [[-0.5, 0.5, -0.5], [0.5, 0.5, -0.5], [0.5, 0.5, 0.5], [-0.5, 0.5, 0.5]];
  r = run(big, small);
  near(r.normal, [0, 1, 0], 'small on large');
  samePoints(r.points, corners.map((p) => [p, 0]), 'small on large');
  r = run(small, big);
  near(r.normal, [0, -1, 0], 'large under small');
  samePoints(r.points, corners.map((p) => [p, 0]), 'large under small');

  // Turned 45° about y, 0.05 above a box its size: the faces overlap in an
  // octagon, kept as 4 of its corners.
  r = run(a, make({ size: [2, 2, 2], position: [0, 2.05, 0], rotation: turn([0, 1, 0], Math.PI / 4) }));
  near(r.normal, [0, 1, 0], 'turned normal');
  assert.equal(r.count, 4);
  for (const p of r.points) {
    near([p.s], [0.05], 'turned separation');
    assert.ok(Math.abs(p.p[0]) <= 1 + EPS && Math.abs(p.p[2]) <= 1 + EPS, 'turned: inside the face');
  }

  // A's top edge runs along z, b's bottom edge along x: they cross at one point.
  const ridge = Math.SQRT2;
  r = run(make({ size: [2, 2, 2], rotation: turn([0, 0, 1], Math.PI / 4) }),
    make({ size: [2, 2, 2], position: [0, 2.8, 0], rotation: turn([1, 0, 0], Math.PI / 4) }));
  near(r.normal, [0, 1, 0], 'edges normal');
  samePoints(r.points, [[[0, 1.4, 0], 2.8 - 2 * ridge]], 'edges');

  // A unit cube balanced on a corner, 0.01 above the large box's top.
  const up = [1, 1, 1].map((x) => x / Math.sqrt(3));
  const corner = turn([up[2], 0, -up[0]], Math.acos(up[1]) + Math.PI);   // takes (1,1,1) to straight down
  r = run(big, make({ size: [1, 1, 1], position: [0, 0.5 + Math.sqrt(3) / 2 + 0.01, 0], rotation: corner }), 0.05);
  assert.equal(r.count, 1, 'corner down: one point');
  near([r.points[0].s], [0.01], 'corner down separation', 1e-12);
  near(r.points[0].p, [0, 0.505, 0], 'corner down point', 1e-12);
});

test('box and box: deep, in the same place, and apart', () => {
  const a = make({ size: [2, 2, 2] });
  let r = run(a, make({ size: [2, 2, 2], position: [0, 0.5, 0] }));
  near(r.normal, [0, 1, 0], 'deep normal');
  assert.equal(r.count, 4);
  for (const p of r.points) near([p.s], [-1.5], 'deep separation');
  // Exactly the same place: every face ties, so a's x, the first.
  r = run(a, make({ size: [2, 2, 2] }));
  near(r.normal, [1, 0, 0], 'same place normal');
  assert.equal(r.count, 4);
  assert.equal(run(a, make({ size: [2, 2, 2], position: [0, 2.3, 0] }), 0.2).count, 0);
});

// ---- the reference ----

/** How far the shape reaches along the unit n: its support function. */
function reach(s, n) {
  const c = s.center, x = s.axes;
  let r = c[0] * n[0] + c[1] * n[1] + c[2] * n[2];
  if (s.kind === SPHERE) return r + s.half[0];
  if (s.kind === CAPSULE) return r + s.half[0] + s.half[1] * Math.abs(x[3] * n[0] + x[4] * n[1] + x[5] * n[2]);
  for (let i = 0; i < 3; i++) r += s.half[i] * Math.abs(x[3 * i] * n[0] + x[3 * i + 1] * n[1] + x[3 * i + 2] * n[2]);
  return r;
}
/** The gap between a and b along the unit n, from a towards b: negative where their extents overlap. */
const gap = (a, b, n) => -reach(b, n.map((x) => -x)) - reach(a, n);

/** The axes along which a shape's support function has a kink: |axis · n| switches sign there. */
function kinks(s) {
  const x = s.axes;
  if (s.kind === SPHERE) return [];
  if (s.kind === CAPSULE) return [[x[3], x[4], x[5]]];
  return [[x[0], x[1], x[2]], [x[3], x[4], x[5]], [x[6], x[7], x[8]]];
}
const cross = (u, v) => [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
const unitOf = (v) => { const l = Math.hypot(...v); return v.map((x) => x / l); };

/**
 * The signed distance between two convex shapes, by brute force: the largest
 * gap along any direction. For shapes apart it is the distance between them;
 * for shapes overlapping, minus how far one must move to clear the other.
 *
 * The gap is smooth except on great circles where some shape axis is at right
 * angles to the direction (the |axis · n| in a support function changes sign
 * there), and the best direction can lie exactly on one, or where two cross,
 * where a random search stalls. So all three are searched: the whole sphere,
 * each kink circle on its own, and each crossing of two. It uses nothing from
 * src/collide.js but the shapes.
 */
function reference(a, b) {
  let best = -Infinity, dir = null;
  const consider = (n) => {
    const g = gap(a, b, n);
    if (g > best) { best = g; dir = n; }
    return g;
  };
  // The whole sphere: 2,000 even directions, then random steps around the best.
  const N = 2000, golden = Math.PI * (3 - Math.sqrt(5));
  for (let i = 0; i < N; i++) {
    const y = 1 - (2 * i + 1) / N, r = Math.sqrt(1 - y * y), t = golden * i;
    consider([Math.cos(t) * r, y, Math.sin(t) * r]);
  }
  let seed = 99;
  const random = () => (seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648 - 0.5;
  for (let step = 0.05; step > 1e-10; step *= 0.5) {
    for (let k = 0; k < 40; k++) consider(unitOf(dir.map((x) => x + step * random())));
  }
  const axes = [...kinks(a), ...kinks(b)];
  // Each kink circle: directions at right angles to one axis, a search in one angle.
  for (const k of axes) {
    const u = unitOf(Math.abs(k[0]) < 0.9 ? cross(k, [1, 0, 0]) : cross(k, [0, 1, 0])), v = cross(k, u);
    const at = (t) => [u[0] * Math.cos(t) + v[0] * Math.sin(t), u[1] * Math.cos(t) + v[1] * Math.sin(t), u[2] * Math.cos(t) + v[2] * Math.sin(t)];
    let bt = 0, bg = -Infinity;
    for (let i = 0; i < 720; i++) {
      const g = consider(at((i / 720) * 2 * Math.PI));
      if (g > bg) { bg = g; bt = (i / 720) * 2 * Math.PI; }
    }
    for (let step = 0.01; step > 1e-13; step *= 0.5) {
      for (let moved = true; moved;) {
        moved = false;
        for (const t of [bt - step, bt + step]) {
          const g = consider(at(t));
          if (g > bg) { bg = g; bt = t; moved = true; }
        }
      }
    }
  }
  // Each crossing of two kink circles: one direction, and its opposite.
  for (let i = 0; i < axes.length; i++) {
    for (let j = i + 1; j < axes.length; j++) {
      const c = cross(axes[i], axes[j]);
      if (Math.hypot(...c) < 1e-9) continue;
      const n = unitOf(c);
      consider(n);
      consider(n.map((x) => -x));
    }
  }
  return best;
}

// ---- random pairs ----

let seed = 2026;
const random = () => (seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648;
function randomShape(kind) {
  const q = [random() - 0.5, random() - 0.5, random() - 0.5, random() - 0.5];
  const l = Math.hypot(...q);
  const position = [random() * 2 - 1, random() * 2 - 1, random() * 2 - 1];
  const rotation = q.map((x) => x / l);
  if (kind === 'box') return make({ shape: 'box', size: [0.3 + random() * 1.5, 0.3 + random() * 1.5, 0.3 + random() * 1.5], position, rotation });
  if (kind === 'sphere') return make({ shape: 'sphere', radius: 0.15 + random() * 0.6, position, rotation });
  const radius = 0.15 + random() * 0.4;
  return make({ shape: 'capsule', radius, height: 2 * radius + random() * 1.5, position, rotation });
}

const KINDS = ['sphere', 'capsule', 'box'];
const PAIRS = [];
for (let i = 0; i < 3; i++) for (let j = i; j < 3; j++) PAIRS.push([KINDS[i], KINDS[j]]);
const MARGIN = 0.02, TOLERANCE = 0.005;
// How closely the reference finds the best gap, on shapes about a metre across.
const REF = 1e-7;

test('random pairs agree with the brute-force reference', () => {
  for (const [ka, kb] of PAIRS) {
    let touching = 0, apart = 0;
    for (let k = 0; k < 600; k++) {
      const a = randomShape(ka), b = randomShape(kb);
      const truth = reference(a, b);
      const r = run(a, b, MARGIN, TOLERANCE);
      const what = `${ka}-${kb} #${k}`;
      if (r.count === 0) {
        // Nothing reported means nothing within the margin, ever: no contact is missed.
        assert.ok(truth > MARGIN - REF, `${what}: missed a contact at ${truth}`);
        apart++;
        continue;
      }
      touching++;
      near([Math.hypot(...r.normal)], [1], `${what}: unit normal`, 1e-12);
      const deepest = Math.min(...r.points.map((p) => p.s));
      for (const p of r.points) assert.ok(p.s <= MARGIN + 1e-12, `${what}: a point past the margin`);
      // The normal is a best way apart, to within the tolerance that lets a
      // box's face win a near tie with an edge.
      assert.ok(gap(a, b, r.normal) >= truth - TOLERANCE - REF, `${what}: normal ${gap(a, b, r.normal)} vs ${truth}`);
      // No point is deeper than the shapes overlap.
      assert.ok(deepest >= truth - TOLERANCE - REF, `${what}: deepest ${deepest} vs ${truth}`);
      // Separating axes, used between boxes and for a capsule whose segment is
      // inside a box, are exact, less the tolerance a face may win a tie by.
      const axes = (ka === 'box' && kb === 'box') || (ka === 'capsule' && kb === 'box' && truth <= -a.half[0]);
      if (!axes && (ka !== 'box' || kb !== 'box')) {
        // Closest points: exact, so the deepest point is the distance.
        near([deepest], [truth], `${what}: deepest vs reference`, REF);
      } else if (truth <= 0) {
        near([deepest], [truth], `${what}: deepest vs reference`, TOLERANCE + REF);
      } else {
        // Apart and nearest at their corners, separating axes see less than
        // the true gap: a contact a little early, never late.
        assert.ok(deepest <= truth + REF, `${what}: reported apart further than they are`);
      }
    }
    // Enough of both kinds to mean something.
    assert.ok(touching > 50 && apart > 50, `${ka}-${kb}: ${touching} touching, ${apart} apart`);
  }
});

/** Signed distance from p to the shape's surface: negative inside. Exact, and independent of src/collide.js. */
function sd(s, p) {
  const x = s.axes, d = [p[0] - s.center[0], p[1] - s.center[1], p[2] - s.center[2]];
  const local = [0, 1, 2].map((i) => d[0] * x[3 * i] + d[1] * x[3 * i + 1] + d[2] * x[3 * i + 2]);
  if (s.kind === SPHERE) return Math.hypot(...local) - s.half[0];
  if (s.kind === CAPSULE) {
    const y = Math.max(-s.half[1], Math.min(s.half[1], local[1]));
    return Math.hypot(local[0], local[1] - y, local[2]) - s.half[0];
  }
  const q = local.map((v, i) => Math.abs(v) - s.half[i]);
  return Math.hypot(...q.map((v) => Math.max(v, 0))) + Math.min(Math.max(...q), 0);
}

test('every point sits midway between the two surfaces, never out in the open', () => {
  // Half the separation back along the normal lands on a's surface, half on
  // lands on b's: never outside either, which would be a contact with
  // nothing, and on the surface wherever the method says so exactly. Two
  // allowances. A capsule pair's end points may sit off to the side by up to
  // the tolerance, which is what decides whether they're kept. And a capsule
  // lying tilted on a box's face keeps the ball around each end of its
  // clipped segment off the face: a true constraint, since the ball is part of
  // the capsule, but that ball's lowest point is inside the capsule, not on it.
  // A box pair's points may sit off the incident box by up to the tolerance:
  // a corner past the reference face's side by no more than that is snapped
  // onto the side, keeping its id, rather than cut (collide.js clipPolygon).
  for (const [ka, kb] of PAIRS) {
    for (let k = 0; k < 400; k++) {
      const a = randomShape(ka), b = randomShape(kb);
      const r = run(a, b, MARGIN, TOLERANCE);
      const slack = (ka === 'capsule' && kb === 'capsule') || (ka === 'box' && kb === 'box') ? TOLERANCE : 0;
      const ballOfA = ka === 'capsule' && kb === 'box';
      for (const { p, s } of r.points) {
        const onA = sd(a, p.map((v, i) => v - r.normal[i] * s / 2));
        const onB = sd(b, p.map((v, i) => v + r.normal[i] * s / 2));
        assert.ok(onA <= slack + 1e-9, `${ka}-${kb} #${k}: ${onA} outside a`);
        assert.ok(onB <= slack + 1e-9, `${ka}-${kb} #${k}: ${onB} outside b`);
        if (!ballOfA) assert.ok(onA >= -slack - 1e-9, `${ka}-${kb} #${k}: ${onA} inside a, off its surface`);
        if (ballOfA) assert.ok(onA >= -a.half[0] - 1e-9, `${ka}-${kb} #${k}: ${onA} deeper in a than its radius`);
        assert.ok(onB >= -slack - 1e-9, `${ka}-${kb} #${k}: ${onB} inside b, off its surface`);
      }
    }
  }
});

test('swapping a and b turns the normal round and keeps the contact', () => {
  // Two same-kind shapes are clipped one against the other, so which is a
  // decides which points stand for the contact; both answers are right, and
  // the solver always passes a pair in the same order (by slot), which is what
  // keeps a contact steady from step to step. So a swapped capsule pair keeps
  // its normal and its deepest point, and a swapped box pair its deepest point,
  // to within the tolerance a face may win a near tie by. Mixed pairs are
  // always worked the same way round, so they match exactly.
  const byValue = (x, y) => x - y;
  for (const [ka, kb] of PAIRS) {
    for (let k = 0; k < 300; k++) {
      const a = randomShape(ka), b = randomShape(kb);
      const r = run(a, b, MARGIN, TOLERANCE), s = run(b, a, MARGIN, TOLERANCE);
      assert.equal(s.count > 0, r.count > 0, `${ka}-${kb} #${k}: touching one way only`);
      if (r.count === 0) continue;
      const da = Math.min(...r.points.map((p) => p.s)), db = Math.min(...s.points.map((p) => p.s));
      if (ka === 'box' && kb === 'box') {
        near([da], [db], `${ka}-${kb} #${k}: deepest`, TOLERANCE + 1e-12);
        continue;
      }
      near(s.normal, r.normal.map((x) => -x), `${ka}-${kb} #${k}: normal`, 1e-12);
      if (ka === kb) near([da], [db], `${ka}-${kb} #${k}: deepest`, 1e-12);
      else near(s.points.map((p) => p.s).sort(byValue), r.points.map((p) => p.s).sort(byValue), `${ka}-${kb} #${k}: separations`, 1e-12);
    }
  }
});

test('a box point keeps its id whichever face is the reference', () => {
  // A ragdoll's forearm under its thigh, faces flat on each other: tilted
  // 1 mrad, the forearm's face is the reference; tilted 3 mrad, the thigh's
  // wins by more than the tolerance. The same four points come out, the
  // thigh's edge across the forearm's sides and the forearm's two corners
  // under the thigh, and each keeps its id, so its warm start carries over
  // (SWAPPED in collide.js). Named by reference and incident, all four changed.
  const a = make({ size: [0.26, 0.08, 0.08] });
  const at = (t) => run(a, make({ size: [0.14, 0.14, 0.44], position: [0.1, 0.108, 0.01], rotation: turn([1, 0, 0], t) }), 0.01, 0.0004);
  const r = at(0.001), s = at(0.003);
  near(r.normal, [0, 1, 0], "the forearm's face", 0);
  assert.ok(s.normal[2] > 0.002, `the thigh's face: normal ${s.normal}`);
  assert.equal(s.count, 4);
  assert.deepEqual(s.points.map((p) => p.id).sort(), r.points.map((p) => p.id).sort());
  for (const p of r.points) near(s.points.find((q) => q.id === p.id).p, p.p, `point ${p.id}`, 0.001);
});

test('moving a shape by d changes the deepest separation by at most d', () => {
  // Continuity: a contact that jumps when a shape barely moves is a contact
  // the solver can't rest on. Box pairs may also switch between a face and an
  // edge, or between the two boxes' faces, within the tolerance.
  const d = 1e-3;
  for (const [ka, kb] of PAIRS) {
    for (let k = 0; k < 300; k++) {
      const a = randomShape(ka), b = randomShape(kb);
      const r = run(a, b, MARGIN, TOLERANCE);
      if (r.count === 0) continue;
      const dir = [random() - 0.5, random() - 0.5, random() - 0.5];
      const l = Math.hypot(...dir);
      b.center[0] += d * dir[0] / l; b.center[1] += d * dir[1] / l; b.center[2] += d * dir[2] / l;
      const s = run(a, b, MARGIN, TOLERANCE);
      if (s.count === 0) continue;   // moved past the margin
      const before = Math.min(...r.points.map((p) => p.s)), after = Math.min(...s.points.map((p) => p.s));
      const slack = ka === 'box' && kb === 'box' ? TOLERANCE : 0;
      assert.ok(Math.abs(after - before) <= d + slack + 1e-12, `${ka}-${kb} #${k}: jumped ${after - before}`);
    }
  }
});

test('the same pair gives the same contact, to the bit', () => {
  for (const [ka, kb] of PAIRS) {
    for (let k = 0; k < 100; k++) {
      const a = randomShape(ka), b = randomShape(kb);
      const r = JSON.stringify(run(a, b, MARGIN, TOLERANCE)), s = JSON.stringify(run(a, b, MARGIN, TOLERANCE));
      assert.equal(r, s);
    }
  }
});
