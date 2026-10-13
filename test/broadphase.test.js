// Finding who might touch: the tree, the broadphase and the pair list, each
// checked against the brute-force answer it replaces, and filtering by layers.
// Run: node test/broadphase.test.js

import assert from 'node:assert/strict';
import { World } from '../src/bearing.js';
import { Tree } from '../src/tree.js';
import { load, PIXEL } from '../src/contact.js';
import { Shape, Manifold, collide } from '../src/collide.js';
import { suite, test } from './test.js';

suite('broadphase');

/** A seeded generator, so a failure can be run again exactly. */
function random(seed) {
  let s = seed >>> 0;
  return () => {
    s = (Math.imul(s, 1664525) + 1013904223) >>> 0;
    return s / 4294967296;
  };
}

const overlaps = (a, i, b, j) => a[i] <= b[j + 3] && a[i + 3] >= b[j] && a[i + 1] <= b[j + 4]
  && a[i + 4] >= b[j + 1] && a[i + 2] <= b[j + 5] && a[i + 5] >= b[j + 2];

test('the tree answers every query as checking every box would, through inserts, moves and removals', () => {
  const rand = random(1), tree = new Tree(), boxes = new Map();   // item -> [leaf, box]
  const box = () => {
    const x = rand() * 100, y = rand() * 100, z = rand() * 100, s = 0.1 + rand() * 5;
    return [x, y, z, x + s, y + s * rand(), z + s];
  };
  for (let op = 0; op < 5000; op++) {
    const r = rand();
    if (r < 0.5 || boxes.size < 10) {
      const b = box();
      boxes.set(op, [tree.insert(op, ...b), b]);
    } else if (r < 0.8) {
      const items = [...boxes.keys()], item = items[Math.floor(rand() * items.length)];
      const b = box();
      tree.move(boxes.get(item)[0], ...b);
      boxes.get(item)[1] = b;
    } else {
      const items = [...boxes.keys()], item = items[Math.floor(rand() * items.length)];
      tree.remove(boxes.get(item)[0]);
      boxes.delete(item);
    }
    if (op % 50 !== 0) continue;
    const q = box(), found = [];
    tree.query(...q, (item) => { found.push(item); });
    const expected = [...boxes].filter(([, [, b]]) => overlaps(q, 0, b, 0)).map(([item]) => item);
    assert.deepEqual(found.sort((a, b) => a - b), expected.sort((a, b) => a - b), `op ${op}`);
  }
  // Every inner node's box holds its children's, its height is one more than
  // the taller child's, and no node leans by more than one level.
  const b = tree.box;
  for (let n = 0; n < tree.capacity; n++) {
    if (tree.height[n] <= 0) continue;
    const c1 = tree.child1[n], c2 = tree.child2[n];
    for (const c of [c1, c2]) {
      assert.equal(tree.parent[c], n);
      for (let k = 0; k < 3; k++) {
        assert.ok(b[6 * n + k] <= b[6 * c + k] && b[6 * n + 3 + k] >= b[6 * c + 3 + k], `node ${n} doesn't hold ${c}`);
      }
    }
    assert.equal(tree.height[n], 1 + Math.max(tree.height[c1], tree.height[c2]));
    assert.ok(Math.abs(tree.height[c1] - tree.height[c2]) <= 1, `node ${n} leans`);
  }
  assert.equal(tree.leaves, boxes.size);
  assert.equal(tree.count, 2 * boxes.size - 1);
});

/**
 * Every pair a step should touch, worked out the slow way: every pair of
 * bodies, through the narrowphase, with the same bounds and margins step()
 * is about to use.
 */
function brute(world, dt) {
  world._bounds(dt);
  const n = world._slots.top, state = world._slots.state, type = world._type;
  const A = new Shape(), B = new Shape(), m = new Manifold(), found = [];
  for (let a = 0; a < n; a++) {
    if (state[a] !== 1) continue;
    for (let b = a + 1; b < n; b++) {
      if (state[b] !== 1 || (type[a] !== 0 && type[b] !== 0)) continue;
      if ((world._layers[a] & world._mask[b]) === 0 || (world._layers[b] & world._mask[a]) === 0) continue;
      load(world, a, A);
      load(world, b, B);
      const tolerance = Math.min(world._size[a], world._size[b]) * PIXEL;
      if (collide(A, B, world._motion[a] + world._motion[b] + tolerance, tolerance, m) > 0) found.push(`${a}-${b}`);
    }
  }
  return found.sort();
}

const touching = (world) => {
  const c = world._contacts, found = [];
  for (let m = 0; m < c.count; m++) found.push(`${c.a[m]}-${c.b[m]}`);
  return found.sort();
};

/** Bodies of every shape and type flung about over a floor, some filtered apart, some removed and added as it runs. */
function scene(seed, steps, check) {
  const rand = random(seed), world = new World();
  world.addBody({ type: 'static', size: [40, 1, 40], position: [0, -0.5, 0] });
  const add = () => {
    const kind = rand(), size = 0.2 + rand() * 1.8;
    const shape = kind < 0.4 ? { size: [size, size * (0.5 + rand()), size] }
      : kind < 0.7 ? { shape: 'sphere', radius: size / 2 } : { shape: 'capsule', radius: size / 4, height: size };
    const type = rand() < 0.1 ? 'static' : rand() < 0.1 ? 'kinematic' : 'dynamic';
    const moving = type === 'static' ? {} : {
      velocity: [(rand() - 0.5) * 20, (rand() - 0.5) * 20, (rand() - 0.5) * 20],
      angularVelocity: [(rand() - 0.5) * 10, (rand() - 0.5) * 10, (rand() - 0.5) * 10],
    };
    return world.addBody({
      type, ...shape, ...moving,
      position: [(rand() - 0.5) * 20, rand() * 15, (rand() - 0.5) * 20],
      rotation: [rand() - 0.5, rand() - 0.5, rand() - 0.5, rand() - 0.5],
      layers: 1 << Math.floor(rand() * 3), mask: rand() < 0.2 ? ~(1 << Math.floor(rand() * 3)) : 0xffffffff,
    });
  };
  const bodies = [];
  for (let i = 0; i < 300; i++) bodies.push(add());
  for (let s = 0; s < steps; s++) {
    if (s % 10 === 5) {
      for (let k = 0; k < 10; k++) world.remove(bodies.splice(Math.floor(rand() * bodies.length), 1)[0]);
      for (let k = 0; k < 10; k++) bodies.push(add());
      // Teleported, retyped and refiltered by hand, between steps.
      bodies[Math.floor(rand() * bodies.length)].setPosition((rand() - 0.5) * 20, rand() * 10, 0);
      world.setBody(bodies[Math.floor(rand() * bodies.length)], { mask: rand() < 0.5 ? 0 : 0xffffffff });
    }
    check?.(world);
    world.step(1 / 60);
  }
  return world;
}

test('the pairs a step touches are exactly those checking every pair would find', () => {
  let checked = 0, touched = 0;
  scene(2, 120, (world) => {
    const expected = brute(world, 1 / 60);
    world.step(1 / 60);
    const found = touching(world);
    assert.deepEqual(found, expected, `step ${checked}`);
    checked++;
    touched += found.length;
  });
  // The scene has to have something in it for this to mean anything.
  assert.ok(touched > 1000, `only ${touched} contacts in all`);
});

test('every listed pair\'s fat boxes overlap, and every pair whose step boxes do is listed', () => {
  scene(3, 120, (world) => {
    world.step(1 / 60);
    const bp = world._broadphase, pairs = world._pairs, box = world._box;
    const listed = new Set();
    for (let p = 0; p < pairs.count; p++) {
      assert.ok(bp.overlap(pairs.a[p], pairs.b[p]), `pair ${pairs.a[p]}-${pairs.b[p]} kept apart`);
      listed.add(`${pairs.a[p]}-${pairs.b[p]}`);
    }
    // After the step, bodies have moved; bounds for the next step are what
    // the next narrowing sees, so check against those.
    world._bounds(1 / 60);
    bp.update(world);
    bp.candidates(world, pairs);
    for (let p = 0; p < pairs.count; p++) listed.add(`${pairs.a[p]}-${pairs.b[p]}`);
    const n = world._slots.top, state = world._slots.state, type = world._type;
    for (let a = 0; a < n; a++) {
      if (state[a] !== 1) continue;
      for (let b = a + 1; b < n; b++) {
        if (state[b] !== 1 || (type[a] !== 0 && type[b] !== 0)) continue;
        if ((world._layers[a] & world._mask[b]) === 0 || (world._layers[b] & world._mask[a]) === 0) continue;
        if (overlaps(box, 6 * a, box, 6 * b)) assert.ok(listed.has(`${a}-${b}`), `${a}-${b} missed`);
      }
    }
  });
});

test('the same scene twice gives the same bits, with bodies added and removed as it runs', () => {
  const a = scene(4, 120), b = scene(4, 120);
  assert.deepEqual(a.transforms, b.transforms);
});

const ground = (world, extra = {}) =>
  world.addBody({ type: 'static', size: [100, 1, 100], position: [0, -0.5, 0], ...extra });
const steps = (world, seconds) => { for (let i = 0; i < Math.round(seconds * 60); i++) world.step(1 / 60); };

test('a body whose mask leaves out the floor\'s layer falls through it, and so does one the floor\'s mask leaves out', () => {
  for (const [floor, box] of [[{}, { mask: ~1 }], [{ mask: ~2 }, { layers: 2 }]]) {
    const world = new World();
    ground(world, floor);
    const body = world.addBody({ position: [0, 2, 0], ...box });
    steps(world, 2);
    assert.ok(body.position[1] < -5, `stopped at ${body.position[1]}`);
  }
});

test('a change of mask takes effect at the next step, both ways', () => {
  const world = new World();
  ground(world);
  const body = world.addBody({ position: [0, 3, 0], mask: 0 });
  steps(world, 0.25);   // falls, but not yet to the floor
  world.setBody(body, { mask: 0xffffffff });
  steps(world, 3);
  assert.ok(Math.abs(body.position[1] - 0.5) < 0.005, `rests at ${body.position[1]}`);
  world.setBody(body, { mask: 0 });
  steps(world, 1);
  assert.ok(body.position[1] < -1, `still held at ${body.position[1]}`);
});

test('layers and masks take negative 32-bit numbers as the same bits, and give them back unsigned', () => {
  const world = new World();
  const body = world.addBody({ layers: 1 << 31, mask: ~2 });
  assert.equal(world.bodyOf(body).layers, 2147483648);
  assert.equal(world.bodyOf(body).mask, 4294967293);
});

test('off means off: nothing moving means no tree is touched, and a pile at rest is re-filed and asked about nothing', () => {
  const count = (world) => {
    const calls = { query: 0, move: 0 };
    for (const tree of world._broadphase.trees) {
      const query = tree.query, move = tree.move;
      tree.query = (...args) => { calls.query++; return query.apply(tree, args); };
      tree.move = (...args) => { calls.move++; return move.apply(tree, args); };
    }
    return calls;
  };

  const level = new World();
  for (let i = 0; i < 1000; i++) level.addBody({ type: 'static', position: [i % 32, Math.floor(i / 32), 0] });
  level.step(1 / 60);
  const quiet = count(level);
  steps(level, 1);
  assert.deepEqual(quiet, { query: 0, move: 0 });
  assert.equal(level._pairs.count, 0);

  const pile = new World();
  ground(pile);
  for (let i = 0; i < 100; i++) pile.addBody({ position: [(i % 10) * 1.01, 0.5 + Math.floor(i / 10), 0] });
  steps(pile, 3);
  const resting = count(pile);
  steps(pile, 1);
  assert.deepEqual(resting, { query: 0, move: 0 });
});
