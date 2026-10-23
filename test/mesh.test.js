// Triangle meshes and heightfields: every shape against a triangle held to a
// brute-force reference, a box sliding over a floor of many triangles as over
// one box, balls rolling over heightfields, .glb files read byte for byte,
// and queries checked against answers worked out another way.
// Run: node test/mesh.test.js

import assert from 'node:assert/strict';
import { World, readGlb } from '../src/bearing.js';
import { Shape, Manifold, SPHERE, CAPSULE, BOX, TRIANGLE } from '../src/collide.js';
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

/** Every edge and corner a corner from both sides: a lone triangle, with no seams. */
const BARE = 63;

function run(s, t, margin, tolerance) {
  const out = { count: 0, normal: new Float64Array(3), points: new Float64Array(21), separations: new Float64Array(7), ids: new Uint32Array(7) };
  touch(s, t, BARE, margin, tolerance, out, 0);
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
