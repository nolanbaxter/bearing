// Queries: rays, shape casts and overlap tests, each held to an answer worked
// out without the code it checks.
// Run: node test/query.test.js

import assert from 'node:assert/strict';
import { World } from '../src/bearing.js';
import { Shape, Manifold, collide, SPHERE, CAPSULE, BOX } from '../src/collide.js';
import { closest, Closest, radius } from '../src/gjk.js';
import { suite, test } from './test.js';

suite('query');

let seed = 7;
const rand = () => ((seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0) / 4294967296);
const between = (a, b) => a + rand() * (b - a);
const unit = (v) => { const l = Math.hypot(...v); return v.map((x) => x / l); };
const turn = () => unit([rand() - 0.5, rand() - 0.5, rand() - 0.5, rand() - 0.5]);

/** v turned by the inverse of unit quaternion q: into a body's own frame. */
function unrotate(q, v) {
  const [x, y, z, w] = [-q[0], -q[1], -q[2], q[3]];
  const tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
  return [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)];
}

/** Whether point p is inside a body, from its options alone: none of the code under test. */
function inside(desc, p) {
  const l = unrotate(desc.rotation, [p[0] - desc.position[0], p[1] - desc.position[1], p[2] - desc.position[2]]);
  if (desc.shape === 'sphere') return Math.hypot(...l) <= desc.radius;
  if (desc.shape === 'capsule') {
    const half = desc.height / 2 - desc.radius, y = Math.max(-half, Math.min(half, l[1]));
    return Math.hypot(l[0], l[1] - y, l[2]) <= desc.radius;
  }
  return Math.abs(l[0]) <= desc.size[0] / 2 && Math.abs(l[1]) <= desc.size[1] / 2 && Math.abs(l[2]) <= desc.size[2] / 2;
}

/** A random body's options: any shape, any turn, 0.2 to 2 m. */
function randomBody(spread) {
  const kind = rand(), s = between(0.2, 2);
  const desc = { position: [between(-spread, spread), between(-spread, spread), between(-spread, spread)], rotation: turn() };
  if (kind < 0.33) return { ...desc, shape: 'sphere', radius: s / 2 };
  if (kind < 0.66) return { ...desc, shape: 'capsule', radius: s / 4, height: s / 2 + between(0, s) };
  return { ...desc, shape: 'box', size: [s, between(0.2, 2), between(0.2, 2)] };
}

test('a ray meets every shape where marching along it first finds the inside', () => {
  // The reference walks the ray in steps of a 4,000th of the body's size,
  // testing the inside by the body's options, then halves the last step
  // down to rounding: slow, but sharing nothing with the code it checks.
  for (let i = 0; i < 300; i++) {
    const desc = randomBody(0);
    const world = new World({ gravity: [0, 0, 0] });
    world.addBody({ ...desc, type: 'static' });
    // From well outside, aimed at a point inside, so the ray does go in.
    const target = desc.position.map((c) => c + between(-0.05, 0.05));
    const from = unit([rand() - 0.5, rand() - 0.5, rand() - 0.5]).map((x, k) => desc.position[k] + x * 5);
    const d = unit(target.map((x, k) => x - from[k]));
    const hit = world.raycast(from, d);
    assert.ok(hit, `case ${i}: missed`);
    const step = 1 / 4000;
    let t = 0;
    while (!inside(desc, from.map((x, k) => x + t * d[k]))) t += step;
    let lo = t - step, hi = t;
    for (let k = 0; k < 60; k++) {
      const mid = (lo + hi) / 2;
      if (inside(desc, from.map((x, j) => x + mid * d[j]))) hi = mid; else lo = mid;
    }
    assert.ok(Math.abs(hit.distance - hi) < 1e-9, `case ${i} (${desc.shape}): ${hit.distance}, marching says ${hi}`);
    assert.ok(Math.abs(Math.hypot(...hit.normal) - 1) < 1e-12, `case ${i}: normal not unit`);
    // The normal faces back at the ray, and the point is on the ray.
    assert.ok(hit.normal[0] * d[0] + hit.normal[1] * d[1] + hit.normal[2] * d[2] < 0, `case ${i}: normal faces away`);
    for (let k = 0; k < 3; k++) assert.ok(Math.abs(hit.point[k] - (from[k] + hit.distance * d[k])) < 1e-9);
  }
});

test('a ray starting inside a body meets it at once, its normal back along the ray', () => {
  const world = new World();
  const box = world.addBody({ type: 'static', size: [2, 2, 2] });
  const hit = world.raycast([0.3, 0.2, 0.1], [1, 0, 0]);
  assert.equal(hit.body.id, box.id);
  assert.equal(hit.distance, 0);
  assert.deepEqual(hit.normal, [-1, -0, -0]);
});

/** A scene of every shape and type, some asleep, stepped so bodies have moved and turned. */
function scene(n = 60) {
  const world = new World();
  world.addBody({ type: 'static', size: [40, 1, 40], position: [0, -0.5, 0] });
  const bodies = [];
  for (let i = 0; i < n; i++) {
    const desc = randomBody(8);
    desc.position[1] = Math.abs(desc.position[1]) + 1;
    const type = rand() < 0.2 ? 'static' : 'dynamic';
    bodies.push(world.addBody({ ...desc, type, layers: 1 << Math.floor(rand() * 3) }));
  }
  for (let i = 0; i < 90; i++) world.step(1 / 60);
  return { world, bodies };
}

/** Every body's ray distance, tested one body at a time in a world of its own: no tree, no shared walk. */
function alone(world, from, d) {
  const out = [];
  for (let s = 0; s < world._slots.top; s++) {
    if (world._slots.state[s] !== 1) continue;
    const b = world.body(world._slots.id(s)), desc = world.bodyOf(b);
    const solo = new World();
    solo.addBody({ ...desc, type: 'static', velocity: undefined, angularVelocity: undefined, position: b.position, rotation: b.rotation });
    const hit = solo.raycast(from, d);
    if (hit) out.push({ slot: s, distance: hit.distance, layers: desc.layers });
  }
  return out.sort((a, b) => a.distance - b.distance || a.slot - b.slot);
}

test('raycast and raycastAll find what testing every body alone finds, through mask, ignore and maxDistance', () => {
  const { world, bodies } = scene();
  for (let i = 0; i < 150; i++) {
    const from = [between(-12, 12), between(0.5, 15), between(-12, 12)];
    const d = unit([rand() - 0.5, rand() - 0.7, rand() - 0.5]);
    const mask = rand() < 0.3 ? 1 << Math.floor(rand() * 3) : 0xffffffff;
    const ignore = rand() < 0.3 ? bodies[Math.floor(rand() * bodies.length)] : null;
    const maxDistance = rand() < 0.3 ? between(1, 20) : Infinity;
    const expected = alone(world, from, d)
      .filter((h) => (h.layers & mask) !== 0 && h.slot !== ignore?.slot && h.distance < maxDistance);
    const all = world.raycastAll(from, d, { mask, ignore, maxDistance });
    assert.deepEqual(all.map((h) => h.body.slot), expected.map((h) => h.slot), `ray ${i}`);
    all.forEach((h, k) => assert.ok(Math.abs(h.distance - expected[k].distance) < 1e-12));
    const nearest = world.raycast(from, d, { mask, ignore, maxDistance });
    assert.equal(nearest?.body.slot ?? null, expected[0]?.slot ?? null, `ray ${i}: nearest`);
  }
});

/** The exact distance from point p to a body's surface, from its options alone (positive outside). */
function away(desc, p) {
  const l = unrotate(desc.rotation, [p[0] - desc.position[0], p[1] - desc.position[1], p[2] - desc.position[2]]);
  if (desc.shape === 'sphere') return Math.hypot(...l) - desc.radius;
  if (desc.shape === 'capsule') {
    const half = desc.height / 2 - desc.radius, y = Math.max(-half, Math.min(half, l[1]));
    return Math.hypot(l[0], l[1] - y, l[2]) - desc.radius;
  }
  const q = l.map((x, k) => Math.max(Math.abs(x) - desc.size[k] / 2, 0));
  return Math.hypot(...q);
}

/** v turned by unit quaternion q: out of a body's frame. */
function rotate(q, v) {
  const [x, y, z, w] = q;
  const tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
  return [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)];
}

/**
 * A body's core, sampled finely, with its radius: a sphere's centre; 2,000
 * points along a capsule's segment; a 100 x 100 grid on each of a box's faces.
 */
function core(desc) {
  const at = (l) => rotate(desc.rotation, l).map((x, k) => x + desc.position[k]);
  if (desc.shape === 'sphere') return { points: [desc.position], r: desc.radius };
  if (desc.shape === 'capsule') {
    const half = desc.height / 2 - desc.radius, points = [];
    for (let i = 0; i <= 2000; i++) points.push(at([0, -half + (2 * half * i) / 2000, 0]));
    return { points, r: desc.radius };
  }
  const h = desc.size.map((x) => x / 2), points = [];
  for (let axis = 0; axis < 3; axis++) for (const side of [-1, 1]) for (let i = 0; i <= 100; i++) for (let j = 0; j <= 100; j++) {
    const l = [0, 0, 0], u = (axis + 1) % 3, v = (axis + 2) % 3;
    l[axis] = side * h[axis]; l[u] = -h[u] + (2 * h[u] * i) / 100; l[v] = -h[v] + (2 * h[v] * j) / 100;
    points.push(at(l));
  }
  return { points, r: 0 };
}

test('GJK gives the distance between shapes that sampling one core against the exact distance to the other gives', () => {
  const near = new Closest(), m = new Manifold();
  const shape = (desc) => {
    const s = new Shape();
    if (desc.shape === 'sphere') { s.kind = SPHERE; s.half[0] = desc.radius; }
    else if (desc.shape === 'capsule') { s.kind = CAPSULE; s.half[0] = desc.radius; s.half[1] = desc.height / 2 - desc.radius; }
    else { s.kind = BOX; s.half.set(desc.size.map((x) => x / 2)); }
    return s.place(...desc.position, desc.rotation);
  };
  let checked = 0;
  for (let i = 0; i < 200 && checked < 60; i++) {
    const a = randomBody(1.5), b = randomBody(1.5);
    const A = shape(a), B = shape(b);
    if (collide(A, B, 0, 1e-9, m) > 0) continue;   // overlapping: no distance to check
    const gap = closest(A, B, near) - radius(A) - radius(B);
    // The nearest sample's distance is the gap, from above, to within the
    // samples' spacing: a millimetre along a segment, 2 cm across a face.
    const { points, r } = core(a);
    let best = Infinity;
    for (const p of points) best = Math.min(best, away(b, p) - r);
    assert.ok(gap <= best + 1e-9, `case ${i} (${a.shape}, ${b.shape}): GJK ${gap} past the sampled ${best}`);
    assert.ok(best - gap < 0.02, `case ${i} (${a.shape}, ${b.shape}): GJK ${gap}, sampling ${best}`);
    checked++;
  }
  assert.ok(checked >= 40, `only ${checked} separated pairs`);
});

test('a shape cast stops where marching the shape forward first finds it touching, and never past it', () => {
  const m = new Manifold();
  const { world } = scene(30);
  let checked = 0;
  for (let i = 0; i < 60; i++) {
    const desc = randomBody(0);
    const cast = { shape: desc.shape, size: desc.size, radius: desc.radius, height: desc.height };
    const from = [between(-10, 10), between(4, 10), between(-10, 10)];
    const d = unit([rand() - 0.5, -0.5 - rand(), rand() - 0.5]);
    const hit = world.shapeCast(cast, from, d, { rotation: desc.rotation });
    // March the shape along in steps of a 400th of a metre, testing it
    // against every body with the narrowphase, until something touches.
    const probe = new Shape();
    if (desc.shape === 'sphere') { probe.kind = SPHERE; probe.half[0] = desc.radius; }
    else if (desc.shape === 'capsule') { probe.kind = CAPSULE; probe.half[0] = desc.radius; probe.half[1] = desc.height / 2 - desc.radius; }
    else { probe.kind = BOX; probe.half.set(desc.size.map((x) => x / 2)); }
    const step = 1 / 400;
    // Every body placed once, with the radius of a sphere around it, so the
    // march only asks the narrowphase about bodies near enough to touch.
    const reach = (bd) => (bd.shape === 'sphere' ? bd.radius : bd.shape === 'capsule' ? bd.height / 2 : Math.hypot(...bd.size) / 2);
    const others = [];
    for (let s = 0; s < world._slots.top; s++) {
      if (world._slots.state[s] !== 1) continue;
      const b = world.body(world._slots.id(s)), bd = world.bodyOf(b);
      const other = new Shape();
      if (bd.shape === 'sphere') { other.kind = SPHERE; other.half[0] = bd.radius; }
      else if (bd.shape === 'capsule') { other.kind = CAPSULE; other.half[0] = bd.radius; other.half[1] = bd.height / 2 - bd.radius; }
      else { other.kind = BOX; other.half.set(bd.size.map((x) => x / 2)); }
      other.place(...b.position, b.rotation);
      others.push({ shape: other, at: b.position, reach: reach(bd) });
    }
    const own = reach(desc);
    let marched = null;
    for (let t = 0; t < 40 && marched === null; t += step) {
      const c = [from[0] + t * d[0], from[1] + t * d[1], from[2] + t * d[2]];
      probe.place(c[0], c[1], c[2], desc.rotation);
      for (const o of others) {
        if (Math.hypot(c[0] - o.at[0], c[1] - o.at[1], c[2] - o.at[2]) > own + o.reach) continue;
        if (collide(probe, o.shape, 0, 1e-9, m) > 0) { marched = t; break; }
      }
    }
    if (marched === null) { assert.equal(hit, null, `cast ${i}: hit nothing marching finds`); continue; }
    assert.ok(hit, `cast ${i}: missed what marching hit at ${marched}`);
    // Stopped within a pixel of touching, before it, and never past where marching found contact.
    assert.ok(hit.distance <= marched + 1e-9, `cast ${i}: went past, ${hit.distance} against ${marched}`);
    assert.ok(hit.distance >= marched - step - 0.02, `cast ${i}: stopped short, ${hit.distance} against ${marched}`);
    checked++;
  }
  assert.ok(checked >= 20, `only ${checked} casts hit`);
});

test('a small shape cast from far off stops at a thin wall, not past it', () => {
  const world = new World();
  world.addBody({ type: 'static', size: [0.01, 4, 4], position: [50, 0, 0] });
  const hit = world.shapeCast({ shape: 'sphere', radius: 0.05 }, [0, 0, 0], [1, 0, 0]);
  assert.ok(hit);
  assert.ok(Math.abs(hit.distance - (50 - 0.005 - 0.05)) < 0.05 / 200, `${hit.distance}`);
  assert.deepEqual(hit.normal.map((x) => Math.round(x)), [-1, 0, 0]);
});

test('overlap finds what the narrowphase finds against every body', () => {
  const m = new Manifold();
  const { world } = scene(40);
  for (let i = 0; i < 100; i++) {
    const desc = randomBody(8);
    desc.position[1] = Math.abs(desc.position[1]) + 0.5;
    const shape = { shape: desc.shape, size: desc.size, radius: desc.radius, height: desc.height };
    const got = world.overlap(shape, desc.position, { rotation: desc.rotation }).map((b) => b.slot).sort((a, b) => a - b);
    const probe = new Shape(), other = new Shape();
    const fill = (s, bd) => {
      if (bd.shape === 'sphere') { s.kind = SPHERE; s.half[0] = bd.radius; }
      else if (bd.shape === 'capsule') { s.kind = CAPSULE; s.half[0] = bd.radius; s.half[1] = bd.height / 2 - bd.radius; }
      else { s.kind = BOX; s.half.set(bd.size.map((x) => x / 2)); }
    };
    fill(probe, desc);
    probe.place(...desc.position, desc.rotation);
    const want = [];
    for (let s = 0; s < world._slots.top; s++) {
      if (world._slots.state[s] !== 1) continue;
      const b = world.body(world._slots.id(s));
      fill(other, world.bodyOf(b));
      other.place(...b.position, b.rotation);
      if (collide(probe, other, 0, 1e-9, m) > 0) want.push(s);
    }
    assert.deepEqual(got, want, `overlap ${i}`);
  }
});

test('queries see the world as it is: after a step, a teleport, an add and a remove', () => {
  const world = new World();
  const ball = world.addBody({ shape: 'sphere', radius: 0.5, position: [0, 10, 0] });
  for (let i = 0; i < 30; i++) world.step(1 / 60);
  const fell = world.raycast([0, 20, 0], [0, -1, 0]);
  assert.ok(Math.abs(fell.distance - (20 - ball.position[1] - 0.5)) < 1e-12, 'after falling');
  ball.setPosition(5, 2, 0);
  assert.equal(world.raycast([0, 20, 0], [0, -1, 0]), null, 'still found where it was');
  assert.ok(Math.abs(world.raycast([5, 20, 0], [0, -1, 0]).distance - 17.5) < 1e-12, 'not found where it went');
  const box = world.addBody({ type: 'static', position: [0, 0, 0] });
  assert.equal(world.raycast([0, 20, 0], [0, -1, 0]).body.id, box.id);
  world.remove(box);
  assert.equal(world.raycast([0, 20, 0], [0, -1, 0]), null);
});

test('a body struck so hard it leaves its fat box in a step is still found before the next step', () => {
  const world = new World();
  world.addBody({ type: 'static', size: [100, 1, 100], position: [0, -0.5, 0] });
  const marble = world.addBody({ shape: 'sphere', radius: 0.01, position: [0, 0.01, 0] });
  for (let i = 0; i < 30; i++) world.step(1 / 60);
  // A heavy ball, fast, into the marble from the side, at its own height: a
  // ball much bigger would wedge it into the floor (see docs/API.md's Contacts).
  const striker = world.addBody({ shape: 'sphere', radius: 0.01, density: 50000, position: [-0.1, 0.01, 0], velocity: [30, 0, 0] });
  world.step(1 / 60);
  const p = marble.position;
  const fat = world._broadphase.trees[1].box, o = 6 * world._broadphase.leaf[marble.slot];
  assert.ok(p[0] - 0.01 > fat[o + 3], `the marble stayed in its fat box (x ${p[0]}); the test needs it out`);
  const hit = world.raycast([p[0], 5, p[2]], [0, -1, 0], { ignore: striker });
  assert.equal(hit?.body.id, marble.id);
});

test('maxDistance leaves out a hit exactly at it or beyond', () => {
  const world = new World();
  world.addBody({ type: 'static', position: [0, 0, 0] });
  assert.equal(world.raycast([0, 10, 0], [0, -1, 0], { maxDistance: 9.5 }), null);
  assert.ok(world.raycast([0, 10, 0], [0, -1, 0], { maxDistance: 9.5000001 }));
});
