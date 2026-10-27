// Triangle meshes and heightfields: every shape against a triangle held to a
// brute-force reference, a box sliding over a floor of many triangles as over
// one box, balls rolling over heightfields, .glb files read byte for byte,
// and queries checked against answers worked out another way.
// Run: node test/mesh.test.js

import assert from 'node:assert/strict';
import { World, readGlb } from '../src/bearing.js';
import { Shape, Manifold, Placed, SPHERE, CAPSULE, BOX, HULL, TRIANGLE } from '../src/collide.js';
import { build } from '../src/hull.js';
import { touch } from '../src/triangle.js';
import { Mesh, Heightfield, triangleNormal } from '../src/mesh.js';
import { closest, Closest, radius } from '../src/gjk.js';
import { sin, cos } from '../src/core/math.js';
import { suite, test, thrown } from './test.js';

suite('mesh');

const steps = (world, seconds, each) => {
  for (let i = 0; i < Math.round(seconds * 60); i++) { world.step(1 / 60); if (each) each(i); }
};
const apart = (a, b) => Math.hypot(...a.map((x, k) => x - b[k]));

/** A flat floor of n by n squares of `size` metres, two triangles each, centred on the origin. */
function floor(n, size) {
  const vertices = [], indices = [], h = (n * size) / 2;
  for (let j = 0; j <= n; j++) for (let i = 0; i <= n; i++) vertices.push(i * size - h, 0, j * size - h);
  for (let j = 0; j < n; j++) {
    for (let i = 0; i < n; i++) {
      const a = j * (n + 1) + i, b = a + 1, c = a + n + 1, d = c + 1;
      indices.push(a, c, d, a, d, b);
    }
  }
  return { shape: 'mesh', vertices, indices };
}

// ---- every shape against one triangle ----

/** A shape from addBody-style options, or a triangle from three corners. */
function make({ shape = 'box', size = [1, 1, 1], radius: r = 0.5, height = 2, position = [0, 0, 0], rotation = [0, 0, 0, 1], corners }) {
  const s = new Shape();
  if (shape === 'triangle') {
    s.kind = TRIANGLE;
    s.points.set(corners);
    for (let k = 0; k < 3; k++) s.center[k] = (corners[k] + corners[k + 3] + corners[k + 6]) / 3;
    triangleNormal(s.points, s.axes);
    return s;
  }
  if (shape === 'box') { s.kind = BOX; s.half.set([size[0] / 2, size[1] / 2, size[2] / 2]); }
  else if (shape === 'sphere') { s.kind = SPHERE; s.half[0] = r; }
  else { s.kind = CAPSULE; s.half[0] = r; s.half[1] = (height - 2 * r) / 2; }
  return s.place(position[0], position[1], position[2], rotation);
}

/** Every edge and corner a corner from both sides: a lone triangle, with no seams and no folds asked of. */
const BARE = 63;

function run(s, t, margin, tolerance) {
  const out = { count: 0, normal: new Float64Array(3), points: new Float64Array(21), separations: new Float64Array(7), ids: new Uint32Array(7) };
  touch(s, t, BARE, null, margin, tolerance, out, 0);
  const points = [];
  for (let k = 0; k < out.count; k++) points.push({ p: [...out.points.subarray(3 * k, 3 * k + 3)], s: out.separations[k] });
  return { count: out.count, normal: [...out.normal], points };
}

// The reference: the same as test/collide.test.js's, with a triangle's
// support function. The signed distance between two convex shapes is the
// largest gap along any direction; it uses nothing from src/triangle.js.

/** How far the shape reaches along the unit n. */
function reach(s, n) {
  if (s.kind === TRIANGLE) {
    const p = s.points;
    return Math.max(p[0] * n[0] + p[1] * n[1] + p[2] * n[2], p[3] * n[0] + p[4] * n[1] + p[5] * n[2], p[6] * n[0] + p[7] * n[1] + p[8] * n[2]);
  }
  const c = s.center, x = s.axes;
  let r = c[0] * n[0] + c[1] * n[1] + c[2] * n[2];
  if (s.kind === SPHERE) return r + s.half[0];
  if (s.kind === CAPSULE) return r + s.half[0] + s.half[1] * Math.abs(x[3] * n[0] + x[4] * n[1] + x[5] * n[2]);
  for (let i = 0; i < 3; i++) r += s.half[i] * Math.abs(x[3 * i] * n[0] + x[3 * i + 1] * n[1] + x[3 * i + 2] * n[2]);
  return r;
}
const gap = (a, b, n) => -reach(b, n.map((x) => -x)) - reach(a, n);
const cross = (u, v) => [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
const unitOf = (v) => { const l = Math.hypot(...v); return v.map((x) => x / l); };

/** Directions where a shape's support function has a kink: at right angles to these. A triangle's are its edges. */
function kinks(s) {
  const x = s.axes;
  if (s.kind === SPHERE) return [];
  if (s.kind === CAPSULE) return [[x[3], x[4], x[5]]];
  if (s.kind === TRIANGLE) {
    const p = s.points;
    return [0, 1, 2].map((e) => { const a = 3 * e, b = 3 * ((e + 1) % 3); return unitOf([p[b] - p[a], p[b + 1] - p[a + 1], p[b + 2] - p[a + 2]]); });
  }
  return [[x[0], x[1], x[2]], [x[3], x[4], x[5]], [x[6], x[7], x[8]]];
}

function reference(a, b) {
  let best = -Infinity, dir = null;
  const consider = (n) => {
    const g = gap(a, b, n);
    if (g > best) { best = g; dir = n; }
    return g;
  };
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
  for (let i = 0; i < axes.length; i++) {
    for (let j = i + 1; j < axes.length; j++) {
      const c = cross(axes[i], axes[j]);
      if (Math.hypot(...c) < 1e-9) continue;
      consider(unitOf(c));
      consider(unitOf(c).map((x) => -x));
    }
  }
  return best;
}

let seed = 2027;
const random = () => (seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648;
const turnRandom = () => unitOf([random() - 0.5, random() - 0.5, random() - 0.5, random() - 0.5]);
function randomShape(kind) {
  const position = [random() * 2 - 1, random() * 2 - 1, random() * 2 - 1], rotation = turnRandom();
  if (kind === 'box') return make({ shape: 'box', size: [0.3 + random() * 1.5, 0.3 + random() * 1.5, 0.3 + random() * 1.5], position, rotation });
  if (kind === 'sphere') return make({ shape: 'sphere', radius: 0.15 + random() * 0.6, position });
  const r = 0.15 + random() * 0.4;
  return make({ shape: 'capsule', radius: r, height: 2 * r + random() * 1.5, position, rotation });
}
function randomTriangle() {
  for (;;) {
    const corners = Array.from({ length: 9 }, () => random() * 3 - 1.5);
    const t = make({ shape: 'triangle', corners });
    // Not a sliver: a triangle that thin isn't what this is checking.
    const e = kinks(t), area = Math.hypot(...cross(
      [corners[3] - corners[0], corners[4] - corners[1], corners[5] - corners[2]],
      [corners[6] - corners[0], corners[7] - corners[1], corners[8] - corners[2]]));
    if (area > 0.3 && e.every((v) => v.every(Number.isFinite))) return t;
  }
}

const MARGIN = 0.02, TOLERANCE = 0.005, REF = 1e-7;

test('every shape against a triangle agrees with the brute-force reference', () => {
  for (const kind of ['sphere', 'capsule', 'box']) {
    let touching = 0, away = 0;
    for (let k = 0; k < 400; k++) {
      const s = randomShape(kind), t = randomTriangle();
      const truth = reference(s, t), r = run(s, t, MARGIN, TOLERANCE);
      const what = `${kind} #${k}`;
      // Deep enough that a capsule's segment, or the middle of a box, goes
      // through the triangle: pushed out along the face, by design, not the
      // shortest way, so only checked for not missing it.
      const deep = kind === 'capsule' ? truth <= -s.half[0] : kind === 'box' ? truth <= -Math.min(...s.half) : false;
      if (r.count === 0) {
        assert.ok(truth > MARGIN - REF, `${what}: missed a contact at ${truth}`);
        away++;
        continue;
      }
      touching++;
      assert.ok(Math.abs(Math.hypot(...r.normal) - 1) < 1e-12, `${what}: unit normal`);
      const deepest = Math.min(...r.points.map((p) => p.s));
      for (const p of r.points) assert.ok(p.s <= MARGIN + 1e-12, `${what}: a point past the margin`);
      if (deep) continue;
      assert.ok(gap(s, t, r.normal) >= truth - TOLERANCE - REF, `${what}: normal ${gap(s, t, r.normal)} vs ${truth}`);
      assert.ok(deepest >= truth - TOLERANCE - REF, `${what}: deepest ${deepest} vs ${truth}`);
      if (kind === 'sphere') assert.ok(Math.abs(deepest - truth) <= REF, `${what}: deepest ${deepest} vs ${truth}`);
      else if (kind === 'capsule' || truth <= 0) assert.ok(Math.abs(deepest - truth) <= TOLERANCE + REF, `${what}: deepest ${deepest} vs ${truth}`);
      // Apart and nearest at their corners, separating axes see less than the true gap: early, never late.
      else assert.ok(deepest <= truth + REF, `${what}: reported apart further than they are`);
    }
    assert.ok(touching > 50 && away > 50, `${kind}: ${touching} touching, ${away} apart`);
  }
});

test('a triangle has two sides: turned over, it gives the same contact', () => {
  for (const kind of ['sphere', 'capsule', 'box']) {
    for (let k = 0; k < 200; k++) {
      const s = randomShape(kind), t = randomTriangle();
      const p = t.points, flipped = make({ shape: 'triangle', corners: [p[0], p[1], p[2], p[6], p[7], p[8], p[3], p[4], p[5]] });
      const a = run(s, t, MARGIN, TOLERANCE), b = run(s, flipped, MARGIN, TOLERANCE);
      assert.equal(a.count, b.count, `${kind} #${k}: count`);
      if (a.count === 0) continue;
      assert.ok(apart(a.normal, b.normal) < 1e-9, `${kind} #${k}: normal`);
      assert.ok(Math.abs(Math.min(...a.points.map((q) => q.s)) - Math.min(...b.points.map((q) => q.s))) < 1e-9, `${kind} #${k}: depth`);
    }
  }
});

// ---- on a mesh as on a box ----

test('a box slides across a flat mesh of 800 triangles exactly as across one box', () => {
  const slide = (mesh, from, velocity, rotation) => {
    const world = new World();
    if (mesh) world.addBody(floor(20, 2));
    else world.addBody({ type: 'static', size: [40, 1, 40], position: [0, -0.5, 0] });
    const box = world.addBody({ position: from, velocity, rotation });
    const path = [];
    steps(world, 4, () => path.push([...box.position, ...box.rotation]));
    return path;
  };
  const turned = [0, sin(Math.PI / 12), 0, cos(Math.PI / 12)];
  // Straight, diagonal and turned, and with its corners riding exactly along the lines between triangles.
  for (const [from, velocity, rotation] of [
    [[-6.3, 0.5, 0.7], [6, 0, 0], [0, 0, 0, 1]], [[-6.3, 0.5, 0.7], [5, 0, 3], [0, 0, 0, 1]],
    [[-6.3, 0.5, 0.7], [4, 0, 2], turned], [[-6, 0.5, 0.5], [6, 0, 0], [0, 0, 0, 1]], [[-6, 0.5, 0.5], [4, 0, 2], turned],
  ]) {
    const box = slide(false, from, velocity, rotation), mesh = slide(true, from, velocity, rotation);
    let worst = 0;
    for (let i = 0; i < box.length; i++) worst = Math.max(worst, apart(box[i], mesh[i]));
    assert.ok(worst < 1e-12, `from ${from} at ${velocity}: ${worst} apart`);
    // It did slide, and stopped where friction says.
    assert.ok(apart(box.at(-1).slice(0, 3), from) > 1, `${from}: moved ${apart(box.at(-1).slice(0, 3), from)}`);
  }
});

test('a ball and a capsule roll down a heightfield slope as down a tilted box', () => {
  const angle = 0.2, q = [0, 0, -sin(angle / 2), cos(angle / 2)];
  // The box's top face, from its rotation as Shape.place works it out, so both slopes are the same plane.
  const ny = [2 * (q[0] * q[1] - q[2] * q[3]), 1 - 2 * (q[0] * q[0] + q[2] * q[2]), 2 * (q[1] * q[2] + q[0] * q[3])];
  const columns = 41, heights = [];
  for (let j = 0; j < columns; j++) for (let i = 0; i < columns; i++) heights.push(-((i - 20) * ny[0]) / ny[1]);
  const roll = (field, shape) => {
    const world = new World();
    if (field) world.addBody({ shape: 'heightfield', heights, columns });
    else world.addBody({ type: 'static', size: [60, 1, 60], position: ny.map((x) => -0.5 * x), rotation: q });
    const x = -10, y = -(x * ny[0]) / ny[1] + 0.5 / ny[1];
    const body = world.addBody({ ...shape, position: [x, y + 0.001, 0.3] });
    const path = [];
    steps(world, 2, () => path.push(body.position));
    return path;
  };
  for (const shape of [{ shape: 'sphere' }, { shape: 'capsule', rotation: [Math.SQRT1_2, 0, 0, Math.SQRT1_2] }]) {
    const box = roll(false, shape), field = roll(true, shape);
    let worst = 0;
    for (let i = 0; i < box.length; i++) worst = Math.max(worst, apart(box[i], field[i]));
    // A landing ends at a gap of exactly zero, give or take rounding, and
    // which side of zero it falls decides whether the next substep counts it
    // as touching. The box's surface is worked out from a centre half a metre
    // below it, the field's from its own origin, so they round differently,
    // and a ball can part by micrometres (7.6 µm measured); never more.
    assert.ok(worst < 1e-5, `${shape.shape}: ${worst} apart`);
    assert.ok(field.at(-1)[0] > -8, `${shape.shape} rolled to ${field.at(-1)[0]}`);
  }
});

test('a ball rolls over waves without leaving the ground', () => {
  // Waves 0.3 m high, 8 m long, sampled every 0.5 m; the folds between
  // triangles are what a ball can catch on, or be held off the ground by.
  const columns = 121, rows = 13, heights = [];
  for (let j = 0; j < rows; j++) for (let i = 0; i < columns; i++) heights.push(0.3 * Math.sin((i * 0.5 * 2 * Math.PI) / 8));
  for (const [r, speed] of [[0.5, 4], [0.5, 2], [0.1, 3], [0.02, 1]]) {
    const world = new World();
    world.addBody({ shape: 'heightfield', heights, columns, spacing: 0.5 });
    const ball = world.addBody({ shape: 'sphere', radius: r, position: [-25, 1, 0.1], velocity: [speed, 0, 0] });
    const g = world._geometry[0], tri = make({ shape: 'triangle', corners: [0, 0, 0, 1, 0, 0, 0, 0, 1] }), core = make({ shape: 'sphere', radius: 0 }), near = new Closest();
    // The gap: the nearest triangle's distance from the centre, less the radius.
    const gapNow = () => {
      core.center.set(ball.position);
      let d = Infinity;
      for (let t = 0; t < g.count; t++) { g.triangle(t, tri.points); d = Math.min(d, closest(core, tri, near)); }
      return d - r;
    };
    let landed = false, highest = 0, travelled = 0, last = ball.position;
    steps(world, 10, () => {
      const gp = gapNow();
      if (landed) highest = Math.max(highest, gp);
      // From the step after it first touches: a landing stops a ball just short.
      if (gp < r * 0.01) landed = true;
      travelled += apart(ball.position, last);
      last = ball.position;
    });
    assert.ok(landed, `radius ${r}: never landed`);
    // Within a contact's tolerance (a pixel of its size) of the ground, all the way.
    assert.ok(highest <= 2 * r / 200, `radius ${r} at ${speed} m/s rose ${highest} m off the ground`);
    assert.ok(travelled > 3, `radius ${r}: rolled ${travelled} m`);
  }
});

test('bodies come to rest and sleep on a mesh and on a heightfield, and wake when hit', () => {
  for (const level of [floor(10, 2), { shape: 'heightfield', heights: new Array(21 * 21).fill(0), columns: 21, spacing: 1 }]) {
    const world = new World();
    world.addBody(level);
    const bodies = [
      world.addBody({ position: [0.3, 2, 0.7] }),
      world.addBody({ shape: 'sphere', position: [3.3, 2, 0.1] }),
      world.addBody({ shape: 'capsule', position: [-3.3, 2, 0.1], rotation: [0, 0, Math.SQRT1_2, Math.SQRT1_2] }),
    ];
    steps(world, 4);
    for (const b of bodies) {
      assert.ok(b.asleep, `${level.shape}: a body didn't fall asleep`);
      assert.ok(Math.abs(b.position[1] - 0.5) < 0.005, `${level.shape}: rests at ${b.position[1]}`);
    }
    world.addBody({ shape: 'sphere', radius: 0.2, position: [0.3, 3, 0.7], velocity: [0, -5, 0] });
    steps(world, 0.5);
    assert.ok(!bodies[0].asleep, `${level.shape}: the box slept through being hit`);
  }
});

test('a mesh or a heightfield is static unless made kinematic, and a kinematic one carries what rests on it', () => {
  const world = new World();
  const platform = world.addBody({ ...floor(4, 1), type: 'kinematic', velocity: [1, 0, 0] });
  // Moving with it from the start; set down still, it would slip until friction caught it up.
  const box = world.addBody({ position: [0, 0.5, 0], velocity: [1, 0, 0] });
  steps(world, 2);
  assert.ok(Math.abs(platform.position[0] - 2) < 1e-9, `the platform moved to ${platform.position[0]}`);
  assert.ok(Math.abs(box.position[0] - 2) < 0.01, `the box rode to ${box.position[0]}`);
  const still = world.addBody(floor(2, 1));
  assert.equal(world.bodyOf(still).type, 'static');
  assert.equal(still.mass, 0);
});

test('a slow kinematic body never falls asleep: it moves exactly as it is told', () => {
  // A 10 m platform at 1 cm/s strays less than a pixel of its size in a
  // second; measured by distance, it would fall asleep and stop.
  const world = new World();
  const platform = world.addBody({ type: 'kinematic', size: [10, 1, 10], velocity: [0.01, 0, 0] });
  steps(world, 5);
  assert.ok(!platform.asleep);
  assert.ok(Math.abs(platform.position[0] - 0.05) < 1e-12, `moved ${platform.position[0]}`);
  platform.setVelocity(0, 0, 0);
  steps(world, 2);
  assert.ok(platform.asleep, 'stopped, it does sleep');
});

test('bodyOf gives back a mesh and a heightfield that addBody copies, and setBody changes them', () => {
  const world = new World();
  const mesh = world.addBody({ ...floor(2, 1), position: [1, 2, 3], friction: 0.2 });
  const copy = world.bodyOf(mesh);
  assert.deepEqual(Object.keys(copy), ['type', 'shape', 'vertices', 'indices', 'friction', 'restitution', 'damping',
    'angularDamping', 'gravityScale', 'layers', 'mask', 'sensor', 'lockRotation', 'position', 'rotation']);
  const again = world.addBody(copy);
  assert.deepEqual(world.bodyOf(again), copy);
  // A triangle with no area is left out: nothing could touch it.
  const flat = world.addBody({ shape: 'mesh', vertices: [0, 0, 0, 1, 0, 0, 2, 0, 0, 0, 0, 1], indices: [0, 1, 2, 0, 1, 3] });
  assert.deepEqual(world.bodyOf(flat).indices, [0, 1, 3]);
  const field = world.addBody({ shape: 'heightfield', heights: [0, 1, 2, 3, 4, 5], columns: 3, spacing: 2 });
  assert.deepEqual(world.bodyOf(field).heights, [0, 1, 2, 3, 4, 5]);
  world.setBody(field, { spacing: 0.5 });
  assert.equal(world.bodyOf(field).spacing, 0.5);
  assert.equal(world.bodyOf(field).columns, 3);
  // Left out, each takes its default: an empty mesh, and a flat 1 m square.
  assert.deepEqual(world.bodyOf(world.addBody({ shape: 'mesh' })).vertices, []);
  assert.deepEqual(world.bodyOf(world.addBody({ shape: 'heightfield' })).heights, [0, 0, 0, 0]);
});

test('seams are found however high a mesh numbers its vertices', () => {
  // Edges were once keyed a * 2^32 + b, exact only below 2^21 vertices:
  // past that, (a, a + 1) and (a, a + 2) took one key, and two lone edges
  // read as one shared seam. Two triangles numbered past 2^21 get exactly
  // the flags of the same two numbered from 0. The filler vertices all sit
  // at one point, so they cost one table entry.
  const tri = [1, 2, 0, 2, 2, 0, 1, 3, 0, 1, 2, 1, 2, 2.5, 0.3];   // corners a, a + 1, a + 2, c, d
  const small = new Mesh(tri, [0, 1, 3, 0, 2, 4]);
  const high = 2 ** 21, vertices = new Float64Array(3 * (high + 5));
  vertices.set(tri, 3 * high);
  const big = new Mesh(vertices, [high, high + 1, high + 3, high, high + 2, high + 4]);
  assert.deepEqual([...big.edges], [...small.edges]);
});

// ---- .glb ----

/** A .glb, built byte by byte: the header, a JSON chunk padded with spaces, and a binary chunk. */
function glb(json, bin) {
  let text = JSON.stringify(json);
  while (text.length % 4) text += ' ';
  const j = new TextEncoder().encode(text), pad = (4 - (bin.length % 4)) % 4;
  const out = new Uint8Array(12 + 8 + j.length + 8 + bin.length + pad), v = new DataView(out.buffer);
  v.setUint32(0, 0x46546c67, true); v.setUint32(4, 2, true); v.setUint32(8, out.length, true);
  v.setUint32(12, j.length, true); v.setUint32(16, 0x4e4f534a, true); out.set(j, 20);
  const at = 20 + j.length;
  v.setUint32(at, bin.length + pad, true); v.setUint32(at + 4, 0x004e4942, true); out.set(bin, at + 8);
  return out;
}
const bytes = (...arrays) => {
  const out = new Uint8Array(arrays.reduce((n, a) => n + a.byteLength, 0));
  let at = 0;
  for (const a of arrays) { out.set(new Uint8Array(a.buffer, a.byteOffset, a.byteLength), at); at += a.byteLength; }
  return out;
};

/**
 * A level: a parent node moved 10 m along x and turned a quarter about y,
 * with two children. One, 'floor', scaled by 2, shows a square as two
 * indexed triangles; the other, 'ramp', placed by a matrix that lifts it 1 m,
 * shows a triangle fan and a strip, unindexed, and a line, which has no triangles.
 */
function level(extra = {}) {
  const square = new Float32Array([0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1]), order = new Uint16Array([0, 1, 2, 0, 2, 3]);
  const fan = new Float32Array([0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0]);
  const bin = bytes(square, order, fan);
  return glb({
    asset: { version: '2.0' }, scene: 0, scenes: [{ nodes: [0] }],
    nodes: [
      { name: 'level', translation: [10, 0, 0], rotation: [0, Math.SQRT1_2, 0, Math.SQRT1_2], children: [1, 2] },
      { name: 'floor', scale: [2, 2, 2], mesh: 0 },
      { name: 'ramp', matrix: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1], mesh: 1 },
    ],
    meshes: [
      { primitives: [{ attributes: { POSITION: 0 }, indices: 1 }] },
      { primitives: [{ attributes: { POSITION: 2 }, mode: 6 }, { attributes: { POSITION: 2 }, mode: 5 }, { attributes: { POSITION: 2 }, mode: 1 }] },
    ],
    accessors: [
      { bufferView: 0, componentType: 5126, count: 4, type: 'VEC3' },
      { bufferView: 1, componentType: 5123, count: 6, type: 'SCALAR' },
      { bufferView: 2, componentType: 5126, count: 4, type: 'VEC3' },
    ],
    bufferViews: [
      { buffer: 0, byteOffset: 0, byteLength: 48 }, { buffer: 0, byteOffset: 48, byteLength: 12 }, { buffer: 0, byteOffset: 60, byteLength: 48 },
    ],
    buffers: [{ byteLength: bin.length }],
    ...extra,
  }, bin);
}

test('a box or hull landing on, or set down on, a floor of small triangles keeps its corners as its contact points', () => {
  // Each point's id carries its impulse into the next step (warm starting).
  // A face held flat over triangles smaller than it touches the ones wholly
  // under it only at seams. Falling within the margin, those seam points
  // were taken for points on the face, since where they lay was asked of
  // the point on the shape's face, above the triangle by more than the
  // tolerance; and set down exactly on the ground, apart by rounding, each
  // such triangle gave GJK's closest point, any point of it, with a normal
  // from rounding. Either way they joined the corners under the triangles'
  // ids and went the step after: a hull landing lost 2 or 3 ids, and one set
  // down on a turned heightfield 33 in its first step.
  const cube = []; for (const z of [-0.5, 0.5]) for (const y of [-0.5, 0.5]) for (const x of [-0.5, 0.5]) cube.push(x, y, z);
  const N = 101, at = [0.37, 0, -0.21], q = [0, sin(0.2), 0, cos(0.2)];
  for (const shape of [{}, { shape: 'hull', vertices: cube }]) {
    for (const ground of ['heightfield', 'mesh', 'turned']) {
      const world = new World({ sleeping: false });
      if (ground === 'mesh') world.addBody(floor(100, 0.1));
      else world.addBody({ shape: 'heightfield', heights: new Array(N * N).fill(ground === 'turned' ? 0.3 : 0), columns: N, spacing: 0.1, ...(ground === 'turned' ? { position: at, rotation: q } : {}) });
      if (ground === 'turned') world.addBody({ ...shape, position: [at[0], 0.8, at[2]], rotation: q });
      else world.addBody({ ...shape, position: [0, 2, 0] });
      const pairs = world._pairs, before = new Map();
      let lost = 0;
      steps(world, 0.75, () => {
        for (let p = 0; p < pairs.count; p++) {
          if (!pairs.touching[p]) continue;
          const m = pairs.memory[p], ids = [];
          for (let s = 0; s < m.count; s++) ids.push(...m.pid.subarray(4 * s, 4 * s + m.points[s]));
          const key = `${pairs.idA[p]}:${pairs.idB[p]}`;
          for (const id of before.get(key) ?? []) if (!ids.includes(id)) lost++;
          before.set(key, ids);
        }
      });
      assert.equal(lost, 0, `${shape.shape ?? 'box'} on ${ground}`);
    }
  }
});

test('a hull resting a corner into a triangle touches at least as deep as it overlaps', () => {
  // Separating axes give the true depth: the least overlap over the
  // triangle's normal, the hull's faces, and each hull edge crossed with each
  // triangle edge. The hull's face clipped against the triangle was the one
  // most against it of all its faces, often not one at its deepest corner:
  // rocks on a heightfield got points 1 to 3 cm above how deep they were, or
  // none, as between hulls before (collide.js).
  let seed = 7;
  const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
  const H = new Placed(), T = new Float64Array(9);
  const out = { count: 0, normal: new Float64Array(3), points: new Float64Array(21), separations: new Float64Array(7), ids: new Uint32Array(7) };
  const rock = () => {
    const pts = [];
    for (let k = 0; k < 12; k++) pts.push(rnd() - 0.5, (rnd() - 0.5) * 0.8, rnd() - 0.5);
    const h = build(pts), v = h.vertices, c = [0, 0, 0];
    for (let i = 0; i < v.length; i++) c[i % 3] += v[i] / (v.length / 3);
    return build(Array.from(v, (x, i) => x - c[i % 3]));
  };
  const along = (x, y, z) => {
    let ha = -Infinity, la = Infinity, ht = -Infinity, lt = Infinity;
    for (let i = 0; i < H.hull.vertices.length; i += 3) {
      const d = H.v[i] * x + H.v[i + 1] * y + H.v[i + 2] * z;
      ha = Math.max(ha, d); la = Math.min(la, d);
    }
    for (let i = 0; i < 9; i += 3) {
      const d = T[i] * x + T[i + 1] * y + T[i + 2] * z;
      ht = Math.max(ht, d); lt = Math.min(lt, d);
    }
    return Math.min(ha - lt, ht - la);
  };
  const depth = () => {
    let least = Infinity;
    const axis = (x, y, z) => {
      const l = Math.sqrt(x * x + y * y + z * z);
      if (l > 1e-9) least = Math.min(least, along(x / l, y / l, z / l));
    };
    const ux = T[3] - T[0], uy = T[4] - T[1], uz = T[5] - T[2], vx = T[6] - T[0], vy = T[7] - T[1], vz = T[8] - T[2];
    axis(uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx);
    for (let f = 0; f < H.hull.planes.length; f += 4) axis(H.planes[f], H.planes[f + 1], H.planes[f + 2]);
    const e = H.hull.edges;
    for (let i = 0; i < e.length; i += 2) {
      const a = 3 * e[i], b = 3 * e[i + 1], ex = H.v[b] - H.v[a], ey = H.v[b + 1] - H.v[a + 1], ez = H.v[b + 2] - H.v[a + 2];
      for (let k = 0; k < 3; k++) {
        const p = 3 * k, q = 3 * ((k + 1) % 3), tx = T[q] - T[p], ty = T[q + 1] - T[p + 1], tz = T[q + 2] - T[p + 2];
        axis(ey * tz - ez * ty, ez * tx - ex * tz, ex * ty - ey * tx);
      }
    }
    return least;
  };
  let tried = 0;
  for (let k = 0; k < 3000; k++) {
    const a = new Shape();
    a.kind = HULL; a.hull = rock();
    // A triangle 4 m across, tilted up to 0.3 rad, and the rock's lowest corner put up to 5 cm into it.
    const tx = (rnd() - 0.5) * 0.6, tz = (rnd() - 0.5) * 0.6;
    const corners = [-2, -2, 2, -2, 0, 2];
    for (let c = 0; c < 3; c++) { const x = corners[2 * c], z = corners[2 * c + 1]; T[3 * c] = x; T[3 * c + 1] = x * tx + z * tz; T[3 * c + 2] = z; }
    const t = make({ shape: 'triangle', corners: T }), n = t.axes;
    const q = [rnd() - 0.5, rnd() - 0.5, rnd() - 0.5, rnd() - 0.5], l = Math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2);
    for (let i = 0; i < 4; i++) q[i] /= l;
    a.place(0, 0, 0, q);
    H.set(a);
    let low = Infinity;
    for (let i = 0; i < H.hull.vertices.length; i += 3) low = Math.min(low, H.v[i] * n[0] + H.v[i + 1] * n[1] + H.v[i + 2] * n[2]);
    const x = rnd() - 0.5, z = rnd() - 0.5, y = x * tx + z * tz, d = -low - rnd() * 0.05;
    a.place(x + n[0] * d, y + n[1] * d, z + n[2] * d, q);
    H.set(a);
    const D = depth(), tolerance = 0.002;
    if (!(D > tolerance && D < 0.1)) continue;
    tried++;
    const count = touch(a, t, BARE, null, tolerance, tolerance, out, 0);
    assert.ok(count > 0, `pose ${k}: ${D} m into the triangle and no contact`);
    const deepest = Math.min(...out.separations.subarray(0, count));
    assert.ok(deepest <= -D + tolerance, `pose ${k}: ${D} m into the triangle, deepest point ${deepest}`);
  }
  assert.ok(tried > 2000, `${tried} poses overlapped`);
});

test('a box resting or rocking on a crest of a bumpy mesh comes to rest', () => {
  // Boxes that rocked on a crest for good, each the only body awake in a heap
  // on this mesh (x8: sin(0.2 x + 0.1 z) * 1.2 over 1 m squares), taken after
  // 60 s with their velocities, and set down alone on the 6 by 6 squares
  // around them. The first three rest on the crest's vertex under their middle
  // and one point near an end, with a dozen points of the triangles under them
  // a few millimetres off: when the four kept left out the vertex, the box sank
  // there until a triangle's face caught it, the overlap pushed it back, and
  // none slept in 30 s. The fourth rocks over a crest: when the four kept
  // points millimetres off over the one it was about to land on, it sank into
  // that one and the push back out sent it over again. Kept, as the points
  // they rest and land on, each sleeps in under 3 s.
  const poses = [
    { position: [-6.073344952135551, 1.4972435765772525, -5.942770277230397], rotation: [-0.9176497312529144, -0.0077677484037579, 0.3972393776279659, -0.007714251612180011],
      velocity: [-0.020518035514507288, -0.0010561825041234423, -0.020528659491018644], angularVelocity: [-0.06826740017987869, -0.0008457217263349291, 0.06826283019535023] },
    { position: [-7.004805436209794, 1.539142381696568, -2.163981120190655], rotation: [0.6449986499425362, -0.2998002781747299, 0.27469359156188833, 0.6470239296451799],
      velocity: [0.0049616227008843486, 0.00007081404332452805, -0.0003143079226092131], angularVelocity: [-0.0008299996880677495, 0.00019284609397242667, -0.01425346545793042] },
    { position: [-6.209266565583694, 1.4897660449167345, -3.883362369324237], rotation: [-0.8741367423385727, 0.0065729339975939185, 0.48554207197608246, 0.0095209544450027],
      velocity: [-0.00876422021164889, 0.005640495232909539, -0.0056925909236064266], angularVelocity: [-0.01873807741970775, 0.0008922152569492375, 0.02824402593850031] },
    { position: [-7.962760856780643, 1.5333731564384638, 0.12084973047342118], rotation: [0.3641268606983299, -0.5967360866782836, -0.5971228905876719, -0.39339792285940023],
      velocity: [-0.0013924039592255646, 0.00014570587686701958, 0.0001644439492931325], angularVelocity: [0.0005184371193879654, -0.00010767286169366176, 0.0038352332746354025] },
  ];
  for (const pose of poses) {
    const world = new World(), vertices = [], indices = [], n = 6;
    const x0 = Math.floor(pose.position[0]) - n / 2, z0 = Math.floor(pose.position[2]) - n / 2;
    for (let j = 0; j <= n; j++) {
      for (let i = 0; i <= n; i++) vertices.push(x0 + i, Math.sin((x0 + i + 32) * 0.2 + (z0 + j + 32) * 0.1) * 1.2, z0 + j);
    }
    for (let j = 0; j < n; j++) {
      for (let i = 0; i < n; i++) { const a = j * (n + 1) + i; indices.push(a, a + n + 1, a + 1, a + 1, a + n + 1, a + n + 2); }
    }
    world.addBody({ shape: 'mesh', vertices, indices });
    const box = world.addBody({ size: [0.9, 0.6, 0.7], ...pose });
    let t = 0;
    while (t < 180 && !box.asleep) { world.step(1 / 60); t++; }
    assert.ok(box.asleep, `the box at ${pose.position.map((x) => x.toFixed(2))} still awake after 3 s`);
    assert.ok(apart(box.position, pose.position) < 0.01, `moved ${apart(box.position, pose.position)} m`);
  }
});

test('a long box lying on one slope of a ridge, an end past the crest, rests there, from either side', () => {
  // A ridge of two slopes of 0.15 meeting at x = 0, as a mesh and as a
  // heightfield; a 2 m box lying along the left slope, its end 0.25 m past
  // the crest. Its centre is below the right slope's plane, carried back
  // under it, so the right slope's triangles were touched from below: the
  // box's top face clipped 0.3 m "into" them, pressing it down into the left
  // slope with 500 N·s a step. It was wedged and pushed about, moved 15 to 23
  // cm, and took 4.5 s to sleep on the mesh and never did on the
  // heightfield. Upside down, the ridge a valley overhead and gravity up, the
  // box meets every triangle from its other side, and must do the same.
  const c = 1 / Math.sqrt(1 + 0.15 * 0.15), s = Math.sqrt((1 - c) / 2), w = Math.sqrt((1 + c) / 2);
  for (const ground of ['mesh', 'heightfield']) {
    for (const k of [1, -1]) {
      const world = new World(), n = 16, ridge = (x) => -k * Math.abs(x - 8) * 0.15;
      world.gravity = [0, -k * 9.81, 0];
      if (ground === 'mesh') {
        const vertices = [], indices = [];
        for (let z = 0; z <= n; z++) for (let x = 0; x <= n; x++) vertices.push(x - 8, ridge(x), z - 8);
        for (let z = 0; z < n; z++) {
          for (let x = 0; x < n; x++) { const a = z * (n + 1) + x; indices.push(a, a + n + 1, a + 1, a + 1, a + n + 1, a + n + 2); }
        }
        world.addBody({ shape: 'mesh', vertices, indices });
      } else {
        const heights = [];
        for (let z = 0; z <= n; z++) for (let x = 0; x <= n; x++) heights.push(ridge(x));
        world.addBody({ shape: 'heightfield', heights, columns: n + 1, spacing: 1 });
      }
      const from = [-0.75, k * 0.04, -1.07], box = world.addBody({ size: [2, 0.3, 0.6], position: from, rotation: [0, 0, k * s, w] });
      let t = 0;
      while (t < 120 && !box.asleep) { world.step(1 / 60); t++; }
      const what = `${ground}${k < 0 ? ', upside down' : ''}`;
      assert.ok(box.asleep, `${what}: still awake after 2 s`);
      assert.ok(apart(box.position, from) < 0.01, `${what}: moved ${apart(box.position, from)} m`);
    }
  }
});

test("a box slides over ground rippled a millimetre or two as the slope says, and holds below friction's hold", () => {
  // Ground rippled a + sin(1.3 i) cos(0.9 j) over 0.25 m squares, mesh and
  // heightfield, tipped to a slope; a 1 m box set down on it, friction 0.5.
  // Every edge folds a few thousandths of a radian, far past flat, so each is
  // a corner, not a seam. A box sliding 2 degrees past friction's hold met the
  // next triangle's near edge side-on (its closest points, an edge against an
  // edge, or its own front face), with a normal along the ground, and stopped
  // as at a wall: 0.9 m in 4 s on 0.5 mm ripples where the slope gives 3.06
  // m. With those left out (triangle.js, FOLDS), friction leaned off the
  // slide at the one-corner manifolds the ripples make, slowed it 3% short of
  // mu N, and a box half a degree below the hold slid on for good, 0.1 to 0.25
  // m. A 1 m box over 0.5 mm ripples, 1.2 m long, climbs and drops at most a
  // millimetre a ripple: 0.01 J/kg against the 1.2 J/kg it slides with after
  // 4 s, under 1%, and its normal tilts a few thousandths either way, which
  // averages out; held to 5% of the slope's figure (the drop it is set down
  // with adds 2% on flat ground).
  const hold = Math.atan(0.5), n = 64, c = 0.25;
  for (const kind of ['mesh', 'heightfield']) {
    for (const ripple of [0.0005, 0.002]) {
      const height = (i, j) => ripple * Math.sin(i * 1.3) * Math.cos(j * 0.9);
      for (const past of [-0.5, 2]) {
        const a = hold + past * Math.PI / 180, rotation = [0, 0, sin(a / 2), cos(a / 2)], world = new World({ sleeping: false });
        if (kind === 'mesh') {
          const vertices = [], indices = [];
          for (let j = 0; j <= n; j++) for (let i = 0; i <= n; i++) vertices.push((i - n / 2) * c, height(i, j), (j - n / 2) * c);
          for (let j = 0; j < n; j++) {
            for (let i = 0; i < n; i++) { const k = j * (n + 1) + i; indices.push(k, k + n + 1, k + 1, k + 1, k + n + 1, k + n + 2); }
          }
          world.addBody({ shape: 'mesh', vertices, indices, rotation });
        } else {
          const heights = [];
          for (let j = 0; j <= n; j++) for (let i = 0; i <= n; i++) heights.push(height(i, j));
          world.addBody({ shape: 'heightfield', heights, columns: n + 1, spacing: c, rotation });
        }
        const box = world.addBody({ rotation, position: [-0.5 * sin(a), 0.5 * cos(a) + 0.003, 0] });
        const start = box.position;
        steps(world, 1.5);
        const landed = box.position;
        steps(world, 2.5);
        const what = `${kind}, ${ripple * 1000} mm ripples, ${past} degrees past the hold`;
        if (past < 0) {
          assert.ok(apart(box.position, landed) < 0.001, `${what}: slid ${apart(box.position, landed)} m after landing`);
        } else {
          const slope = 0.5 * 9.81 * (Math.sin(a) - 0.5 * Math.cos(a)) * 16;
          assert.ok(Math.abs(apart(box.position, start) / slope - 1) < 0.05, `${what}: slid ${apart(box.position, start)} m of ${slope}`);
        }
      }
    }
  }
});

test('a body on a mesh steps the same whatever the world before it did', () => {
  // A point carried back from where a pair met in the step (impact.js) keeps
  // a shift, and the manifold being built was told so by a flag meet() set;
  // mesh pairs never ask meet() and went on with the flag the last pair
  // left, the last world's in a new one, and took that pair's shifts as
  // their own: a box resting on a mesh came out 6 mm from where it did
  // alone, and two copies of one heap on a heightfield parted.
  const before = (fast) => {
    const world = new World();
    world.addBody({ type: 'static', size: [10, 1, 10], position: [0, -0.5, 0] });
    // Fast, it meets the floor partway through the step, and its contact is carried back.
    world.addBody(fast ? { position: [0, 0.6, 0], velocity: [0, -12, 0] } : { position: [0, 0.5, 0] });
    world.step(1 / 60);
  };
  const resting = () => {
    const world = new World();
    world.addBody({ shape: 'mesh', vertices: [-5, 0, -5, 5, 0, -5, 5, 0, 5, -5, 0, 5], indices: [0, 2, 1, 0, 3, 2] });
    const box = world.addBody({ position: [0.2, 0.5, 0.1], rotation: [0.05, 0, 0.03, 0.998] });
    steps(world, 0.5);
    return [...box.position, ...box.rotation];
  };
  before(false);
  const calm = resting();
  before(true);
  assert.deepEqual(resting(), calm);
});

test('a .glb built byte by byte reads back exactly, every node placed as in the scene', () => {
  const { vertices, indices } = readGlb(level());
  // The quarter turn about y takes (x, y, z) to (z, y, -x), worked by hand.
  const r = (x, y, z) => [10 + z, y, -x];
  const expected = [
    ...r(0, 0, 0), ...r(2, 0, 0), ...r(2, 0, 2), ...r(0, 0, 2),              // the floor, scaled by 2
    ...r(0, 1, 0), ...r(1, 1, 0), ...r(1, 2, 0), ...r(0, 2, 0),              // the fan, lifted 1 m
    ...r(0, 1, 0), ...r(1, 1, 0), ...r(1, 2, 0), ...r(0, 2, 0),              // the strip; the line adds nothing
  ];
  assert.equal(vertices.length, expected.length);
  for (let i = 0; i < expected.length; i++) assert.ok(Math.abs(vertices[i] - expected[i]) < 1e-12, `vertex number ${i}: ${vertices[i]} vs ${expected[i]}`);
  assert.deepEqual(Array.from(indices), [0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7, 8, 9, 10, 9, 10, 11]);
  // One node, still placed by its parent; an ArrayBuffer is read the same as a typed array.
  const ramp = readGlb(level().buffer, { node: 'ramp' });
  assert.deepEqual(Array.from(ramp.vertices).map((v) => +v.toFixed(12)), expected.slice(12).map((v) => +v.toFixed(12)));
  assert.deepEqual(Array.from(ramp.indices), [0, 1, 2, 0, 2, 3, 4, 5, 6, 5, 6, 7]);
  // Straight into a body, which a ray then finds where the floor is.
  const world = new World();
  world.addBody({ shape: 'mesh', ...readGlb(level(), { node: 'floor' }) });
  const hit = world.raycast([11, 5, -1], [0, -1, 0]);
  assert.ok(hit && Math.abs(hit.distance - 5) < 1e-12 && apart(hit.normal, [0, 1, 0]) < 1e-12);
});

test('readGlb names what it can\'t read, rather than reading it wrong', () => {
  assert.match(thrown(() => readGlb(level({ extensionsRequired: ['KHR_draco_mesh_compression'] }))),
    /needs KHR_draco_mesh_compression, which Bearing doesn't read; export it without that/);
  // Materials and other extensions say nothing about shape, and don't stop it.
  assert.equal(readGlb(level({ extensionsRequired: ['KHR_materials_unlit'] })).indices.length, 18);
  const file = level();
  new DataView(file.buffer).setUint32(4, 1, true);
  assert.match(thrown(() => readGlb(file)), /got glTF version 1/);
  assert.match(thrown(() => readGlb(new Uint8Array(40))), /doesn't start with glTF/);
  assert.match(thrown(() => readGlb(level().subarray(0, 300))), /the file is damaged/);
});

// ---- queries ----

let qseed = 11;
const q = () => ((qseed = (Math.imul(qseed, 1664525) + 1013904223) >>> 0) / 4294967296);
const qturn = () => unitOf([q() - 0.5, q() - 0.5, q() - 0.5, q() - 0.5]);
/** v turned by the inverse of unit quaternion r. */
function unrotate(r, v) {
  const [x, y, z, w] = [-r[0], -r[1], -r[2], r[3]];
  const tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
  return [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)];
}

test('a ray meets a mesh of a box\'s 12 triangles exactly where it meets the box', () => {
  const [hx, hy, hz] = [1, 0.5, 1.5], vertices = [];
  for (const x of [-hx, hx]) for (const y of [-hy, hy]) for (const z of [-hz, hz]) vertices.push(x, y, z);
  const indices = [0, 1, 3, 0, 3, 2, 4, 6, 7, 4, 7, 5, 0, 4, 5, 0, 5, 1, 2, 3, 7, 2, 7, 6, 0, 2, 6, 0, 6, 4, 1, 5, 7, 1, 7, 3];
  const rotation = qturn(), position = [1, 2, 3];
  const box = new World(), mesh = new World();
  box.addBody({ type: 'static', size: [2 * hx, 2 * hy, 2 * hz], position, rotation });
  mesh.addBody({ shape: 'mesh', vertices, indices, position, rotation });
  let hits = 0;
  for (let i = 0; i < 4000; i++) {
    const o = position.map((c) => c + (q() - 0.5) * 8), d = [q() - 0.5, q() - 0.5, q() - 0.5];
    const a = box.raycast(o, d), b = mesh.raycast(o, d);
    // From inside, a box is met at 0; a mesh is a surface, met on the way out.
    if (a && a.distance === 0) continue;
    assert.equal(a === null, b === null, `ray ${i}`);
    if (!a) continue;
    hits++;
    assert.ok(Math.abs(a.distance - b.distance) < 1e-12 && apart(a.normal, b.normal) < 1e-12 && apart(a.point, b.point) < 1e-12, `ray ${i}`);
  }
  assert.ok(hits > 100, `${hits} hits`);
});

test('a ray meets a heightfield where marching down it first finds the ground', () => {
  const columns = 17, rows = 13, spacing = 0.75, heights = [];
  for (let j = 0; j < rows; j++) for (let i = 0; i < columns; i++) heights.push(Math.sin(i * 0.7) * Math.cos(j * 0.5) * 1.2);
  const position = [-1, 0.5, 2], rotation = unitOf([0.1, 0.4, -0.2, 0.9]);
  const world = new World();
  world.addBody({ shape: 'heightfield', heights, columns, spacing, position, rotation });
  // The ground's height under (x, z), in the field's frame, from its two
  // triangles a cell, worked out here: cut corner to corner, from (i, j) to (i + 1, j + 1).
  const x0 = -((columns - 1) * spacing) / 2, z0 = -((rows - 1) * spacing) / 2, h = (i, j) => heights[j * columns + i];
  const ground = (x, z) => {
    const fx = (x - x0) / spacing, fz = (z - z0) / spacing;
    if (fx < 0 || fz < 0 || fx > columns - 1 || fz > rows - 1) return null;
    const i = Math.min(columns - 2, Math.floor(fx)), j = Math.min(rows - 2, Math.floor(fz)), u = fx - i, v = fz - j;
    return u >= v ? h(i, j) + u * (h(i + 1, j) - h(i, j)) + v * (h(i + 1, j + 1) - h(i + 1, j))
      : h(i, j) + v * (h(i, j + 1) - h(i, j)) + u * (h(i + 1, j + 1) - h(i, j + 1));
  };
  const below = (p) => {
    const l = unrotate(rotation, p.map((c, k) => c - position[k])), g = ground(l[0], l[2]);
    return g !== null && l[1] < g;
  };
  let hits = 0;
  for (let i = 0; i < 300; i++) {
    // From above the field, in its own frame, down at a point on it.
    const from = unrotate(rotation.map((c, k) => (k < 3 ? -c : c)), [(q() - 0.5) * 14, 3, (q() - 0.5) * 10]).map((c, k) => c + position[k]);
    const to = unrotate(rotation.map((c, k) => (k < 3 ? -c : c)), [(q() - 0.5) * 12, -2, (q() - 0.5) * 8]).map((c, k) => c + position[k]);
    const d = unitOf(to.map((c, k) => c - from[k]));
    const hit = world.raycast(from, d, { maxDistance: 20 });
    let lo = 0, hi = null;
    for (let t = 0; t < 20; t += 0.002) if (below(from.map((c, k) => c + t * d[k]))) { hi = t; break; }
    if (hi === null) { assert.equal(hit, null, `ray ${i} hit what marching never reached`); continue; }
    lo = hi - 0.002;
    while (hi - lo > 1e-12) { const m = (lo + hi) / 2; if (below(from.map((c, k) => c + m * d[k]))) hi = m; else lo = m; }
    assert.ok(hit && Math.abs(hit.distance - hi) < 1e-9, `ray ${i}: ${hit && hit.distance}, marching says ${hi}`);
    hits++;
  }
  assert.ok(hits > 100, `${hits} hits`);
});

test('shape casts and overlap tests against a mesh agree with every triangle tested alone', () => {
  const columns = 9, heights = [];
  for (let j = 0; j < columns; j++) for (let i = 0; i < columns; i++) heights.push(Math.sin(i * 0.9 + j * 0.4) * 0.8);
  const position = [0.5, -1, 0.25], rotation = unitOf([0.05, 0.3, 0.1, 0.95]);
  for (const kind of ['heightfield', 'mesh']) {
    const world = new World();
    const field = new Heightfield(heights, columns, 1), vertices = [], indices = [], p9 = new Float64Array(9);
    for (let t = 0; t < field.count; t++) { field.triangle(t, p9); vertices.push(...p9); indices.push(3 * t, 3 * t + 1, 3 * t + 2); }
    const options = kind === 'heightfield' ? { shape: 'heightfield', heights, columns } : { shape: 'mesh', vertices, indices };
    world.addBody({ ...options, position, rotation });
    // Every triangle, placed in the world, for the reference.
    const tris = [];
    for (let t = 0; t < field.count; t++) {
      field.triangle(t, p9);
      const c = [];
      for (let v = 0; v < 3; v++) {
        const w = unrotate(rotation.map((x, k) => (k < 3 ? -x : x)), [p9[3 * v], p9[3 * v + 1], p9[3 * v + 2]]);
        c.push(w[0] + position[0], w[1] + position[1], w[2] + position[2]);
      }
      tris.push(make({ shape: 'triangle', corners: c }));
    }
    const near = new Closest();
    const nearest = (s) => Math.min(...tris.map((t) => closest(s, t, near))) - radius(s);
    let hits = 0, overlaps = 0;
    for (let i = 0; i < 60; i++) {
      const k = i % 3, size = 0.2 + q() * 0.6, turn = qturn();
      const desc = k === 0 ? { shape: 'sphere', radius: size / 2 } : k === 1 ? { shape: 'capsule', radius: size / 3, height: size * 1.5 } : { shape: 'box', size: [size, size * 0.7, size * 1.2] };
      const from = [(q() - 0.5) * 6, 2 + q() * 2, (q() - 0.5) * 6], d = unitOf([(q() - 0.5) * 2, -1, (q() - 0.5) * 2]);
      const hit = world.shapeCast(desc, from, d, { rotation: turn, maxDistance: 8 });
      const probe = make({ ...desc, position: from, rotation: turn });
      const at = (t) => { probe.place(from[0] + t * d[0], from[1] + t * d[1], from[2] + t * d[2], turn); return nearest(probe); };
      // Sampled finer than the shape is thin, so no triangle slips between samples.
      const step = size / 8;
      if (!hit) {
        for (let t = 0; t <= 8; t += step) assert.ok(at(t) > 0, `${kind} cast ${i}: missed a touch at ${t}`);
        continue;
      }
      hits++;
      for (let t = 0; t < hit.distance - step; t += step) assert.ok(at(t) > 0, `${kind} cast ${i}: passed a touch at ${t}`);
      const there = at(hit.distance);
      assert.ok(there >= -1e-9 && there <= size * 2 / 200 + 1e-9, `${kind} cast ${i}: ${there} from touching where it stopped`);
      // Overlap: placed where the cast stopped and pushed a little further in, or held back.
      for (const t of [hit.distance + size / 4, hit.distance - size / 4]) {
        const g = at(Math.max(0, t));
        if (Math.abs(g) < 0.01) continue;
        const found = world.overlap(desc, probe.center, { rotation: turn }).length > 0;
        assert.equal(found, g < 0, `${kind} overlap ${i}: found ${found}, gap ${g}`);
        if (found) overlaps++;
      }
    }
    assert.ok(hits > 30 && overlaps > 20, `${kind}: ${hits} hits, ${overlaps} overlaps`);
  }
});

// ---- size ----

test('a mesh of 100,000 triangles builds once, and a body on it looks only at the triangles under it', () => {
  const n = 224;   // 224 x 224 squares: 100,352 triangles
  const start = performance.now();
  const world = new World();
  const big = world.addBody(floor(n, 0.5));
  const built = performance.now() - start;
  const mesh = world._geometry[0];
  assert.equal(mesh.count, 2 * n * n);
  let looked = 0;
  const triangles = mesh.triangles.bind(mesh);
  mesh.triangles = (x0, y0, z0, x1, y1, z1, visit) => triangles(x0, y0, z0, x1, y1, z1, (t) => { looked++; visit(t); });
  const box = world.addBody({ position: [3.1, 0.6, -7.3], velocity: [2, 0, 1] });
  steps(world, 1);
  const perStep = looked / 60;
  console.log(`      built in ${built.toFixed(0)} ms; ${perStep.toFixed(1)} triangles looked at a step, of ${mesh.count}`);
  // A 1 m box over 0.5 m squares covers about 2 by 2 of them, 8 triangles, and a few more as it moves.
  assert.ok(perStep < 40, `${perStep} triangles a step`);
  assert.ok(Math.abs(box.position[1] - 0.5) < 0.005);
  world.remove(big);
});

test('errors name the mesh option at fault, and where in a long list', () => {
  const world = new World();
  assert.equal(thrown(() => world.addBody({ shape: 'mesh', vertices: [0, 0, NaN] })),
    'addBody: vertices must be finite numbers, three to a vertex, got NaN at index 2');
  assert.equal(thrown(() => world.addBody({ shape: 'mesh', vertices: [0, 0, 0, 1], indices: [] })),
    'addBody: vertices must be finite numbers, three to a vertex, got 4 numbers');
  assert.equal(thrown(() => world.addBody({ shape: 'mesh', vertices: [0, 0, 0], indices: [0, 0, 1] })),
    'addBody: indices must be whole numbers below the vertex count (1), three to a triangle, got 1 at index 2');
  assert.equal(thrown(() => world.addBody({ shape: 'heightfield', heights: [0, 0, 0], columns: 3 })),
    'addBody: heights must be finite numbers, rows of columns (3), 2 rows or more, got 3 numbers');
  assert.equal(thrown(() => world.addBody({ shape: 'heightfield', columns: 1 })),
    'addBody: columns must be a whole number, 2 or more, got 1');
  assert.equal(thrown(() => world.addBody({ shape: 'mesh', type: 'dynamic' })),
    "addBody: type must be 'static' or 'kinematic' for a mesh, got 'dynamic'");
  assert.equal(thrown(() => world.addBody({ shape: 'heightfield', mass: 5 })), 'addBody: a heightfield has no mass');
  assert.equal(thrown(() => world.addBody({ shape: 'mesh', radius: 5 })), 'addBody: a mesh has no radius');
  assert.equal(thrown(() => world.addBody({ heights: [1] })), 'addBody: a box has no heights');
  const box = world.addBody({});
  assert.equal(thrown(() => world.setBody(box, { shape: 'mesh' })), "setBody: type must be 'static' or 'kinematic' for a mesh, got 'dynamic'");
  assert.equal(thrown(() => world.shapeCast({ shape: 'mesh' }, [0, 0, 0], [1, 0, 0])),
    "shapeCast: shape must be 'box', 'sphere' or 'capsule', got 'mesh'");
});
