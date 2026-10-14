// Nothing grows forever: the permanent leak test.
// Run: node --expose-gc test/soak.test.js
//
// Adds and removes 10,000 bodies over and over, past the 255 reuses that
// would retire a slot under Winding's 8-bit generations, and checks that the
// world's storage and memory come back to where they started each time. Then
// checks, with WeakRef, that removed bodies' handles and destroyed worlds are
// collected rather than held by something inside.

import assert from 'node:assert/strict';
import { World } from '../src/bearing.js';
import { suite, testAsync as test } from './test.js';

if (typeof globalThis.gc !== 'function') {
  throw new Error('run with node --expose-gc test/soak.test.js');
}

suite('soak');

const tick = () => new Promise((resolve) => setTimeout(resolve, 0));
/** A full collection, after the current job ends so WeakRefs can be cleared. */
async function collect() {
  for (let i = 0; i < 3; i++) {
    await tick();
    globalThis.gc();
  }
}
const heap = () => process.memoryUsage().heapUsed + process.memoryUsage().arrayBuffers;
/** Rounds run before memory is measured: see the colliding pile's test. */
const WARM = 10;

/** Everything the broadphase holds: tree nodes in use and allotted, and pairs listed. */
const held = (world) => {
  const [statics, moving] = world._broadphase.trees;
  return {
    nodes: statics.count + moving.count, treeCapacity: statics.capacity + moving.capacity,
    pairs: world._pairs.count, pairCapacity: world._pairs.capacity,
  };
};

await test('10,000 bodies added and removed 300 times: slots return, storage never drifts', async () => {
  const world = new World();
  const resident = [];
  // Floating, so they stay 3 m apart for the whole test and never pair.
  for (let i = 0; i < 100; i++) resident.push(world.addBody({ position: [3 * i, 0, 0], gravityScale: 0 }));
  world.step(1 / 60);
  const start = { top: world._slots.top, count: world.bodyCount, capacity: world.transforms.length / 7 };
  const startHeld = held(world);
  const burst = new Array(10000);
  const after = [], treeAfter = [];
  for (let round = 0; round < 300; round++) {
    // A grid 2 m apart, so the step finds no pairs among them, but files and
    // queries all 10,000 in the broadphase's trees.
    for (let i = 0; i < burst.length; i++) {
      burst[i] = world.addBody({ shape: 'sphere', position: [(i % 100) * 2, 10 + Math.floor(i / 100) * 2, 5] });
    }
    world.step(1 / 60);
    // Oldest first, then newest first: the highest slot goes last, so storage
    // falls in one go, or first, so it falls a step at a time.
    if (round % 2) burst.reverse();
    for (const b of burst) world.remove(b);
    assert.equal(world.bodyCount, start.count, `round ${round}: count`);
    assert.equal(world._slots.top, start.top, `round ${round}: slots in use`);
    after.push(world.transforms.length / 7);
    world.step(1 / 60);
    const now = held(world);
    assert.equal(now.nodes, startHeld.nodes, `round ${round}: tree nodes kept`);
    assert.equal(now.pairs, 0, `round ${round}: pairs kept`);
    assert.equal(now.pairCapacity, startHeld.pairCapacity, `round ${round}: pair storage kept`);
    treeAfter.push(now.treeCapacity);
  }
  // Removed all at once, storage comes back to exactly where it started.
  // Removed a step at a time, it stops shrinking inside the no-thrash band,
  // under four times the slots in use. Either way, every round of a kind
  // ends where the first did: nothing accumulates.
  assert.equal(after[0], start.capacity);
  for (let r = 0; r < after.length; r++) {
    assert.equal(after[r], after[r % 2], `round ${r}: storage drifted`);
    assert.ok(after[r] < 4 * start.top, `round ${r}: ${after[r]} slots of storage for ${start.top} in use`);
    // The trees follow the same rule, each on its own.
    assert.equal(treeAfter[r], treeAfter[r % 2], `round ${r}: tree storage drifted`);
    assert.ok(treeAfter[r] <= 4 * startHeld.nodes + 32, `round ${r}: ${treeAfter[r]} tree nodes for ${startHeld.nodes} in use`);
  }
  for (const b of resident) assert.ok(b.alive);
});

await test('memory comes back after a burst', async () => {
  const world = new World();
  for (let i = 0; i < 100; i++) world.addBody({ position: [3 * i, 0, 0] });
  world.step(1 / 60);
  await collect();
  const before = heap();
  for (let round = 0; round < 20; round++) {
    const burst = [];
    for (let i = 0; i < 10000; i++) burst.push(world.addBody({}));
    for (const b of burst) world.remove(b);
  }
  await collect();
  const grown = heap() - before;
  // One 10,000-body burst holds about 4 MB of columns; anything kept would
  // show as at least that. 256 KB covers V8's own housekeeping.
  assert.ok(grown < 256 * 1024, `grew ${(grown / 1024).toFixed(0)} KB`);
  assert.equal(world.bodyCount, 100);
});

await test('a colliding pile built and cleared 50 times: contact storage and memory come back', async () => {
  const world = new World();
  world.addBody({ type: 'static', size: [100, 1, 100], position: [0, -0.5, 0] });
  world.step(1 / 60);
  // The first rounds are a warm-up and aren't measured: V8 compiles the
  // narrowphase, the solver and the sleep pass, then recompiles the hottest
  // of them as they warm, over several rounds, and compiled code lives in the
  // heap too. Measured from round 1, growth wandered from 422 KB at 25 rounds
  // to 17 KB at 100 and below zero at 200: code, not a leak, which would
  // grow with every round.
  let before = 0;
  for (let round = 0; round < 50; round++) {
    if (round === WARM) {
      await collect();
      before = heap();
    }
    const pile = [];
    for (let k = 0; k < 200; k++) pile.push(world.addBody({ position: [(k % 10) * 1.01, 0.5 + Math.floor(k / 10), (round % 3) * 0.1] }));
    for (let s = 0; s < 10; s++) world.step(1 / 60);
    assert.ok(world._contacts.count > 100, `round ${round}: only ${world._contacts.count} contacts`);
    for (const b of pile) world.remove(b);
    // One step finds every pair gone and trims what held them, down to the
    // smallest size each keeps: 16 nodes a tree, 16 pairs.
    world.step(1 / 60);
    assert.equal(world._contacts.capacity, 0, `round ${round}: contact storage kept`);
    assert.deepEqual(held(world), { nodes: 1, treeCapacity: 32, pairs: 0, pairCapacity: 16 }, `round ${round}: broadphase storage kept`);
  }
  await collect();
  const grown = heap() - before;
  assert.ok(grown < 256 * 1024, `grew ${(grown / 1024).toFixed(0)} KB`);
});

await test('stacks put to sleep, woken and cleared 30 times: the awake list, the asleep pairs and the moved log come back', async () => {
  const world = new World();
  const floor = world.addBody({ type: 'static', size: [100, 1, 100], position: [0, -0.5, 0] });
  world.step(1 / 60);
  let before = 0;
  for (let round = 0; round < 30; round++) {
    if (round === WARM) {
      await collect();
      before = heap();
    }
    const boxes = [];
    for (let k = 0; k < 60; k++) boxes.push(world.addBody({ position: [(k % 12) * 2 - 12, 0.5 + Math.floor(k / 12), round % 2] }));
    let n = 0;
    while (world._awakeCount > 0 && n++ < 600) world.step(1 / 60);
    assert.equal(world._awakeCount, 0, `round ${round}: never slept`);
    assert.ok(world._pairs.count > world._pairs.active, `round ${round}: no pair asleep`);
    world.moved(world.stepCount);
    // Woken from beneath, then cleared away.
    floor.setPosition(0, -0.6, round % 2);
    world.step(1 / 60);
    assert.equal(world._awakeCount, 60, `round ${round}: not all woken`);
    for (const b of boxes) world.remove(b);
    world.step(1 / 60);
    assert.equal(world._awakeCount, 0);
    assert.deepEqual(held(world), { nodes: 1, treeCapacity: 32, pairs: 0, pairCapacity: 16 }, `round ${round}: broadphase storage kept`);
    assert.ok(world._logSlots.length <= 16, `round ${round}: a moved log of ${world._logSlots.length} kept`);
    floor.setPosition(0, -0.5, 0);
  }
  await collect();
  const grown = heap() - before;
  assert.ok(grown < 256 * 1024, `grew ${(grown / 1024).toFixed(0)} KB`);
});

await test('meshes and heightfields made, landed on and removed 30 times: their triangles and pair memories go', async () => {
  const world = new World();
  world.step(1 / 60);
  // A floor of 800 triangles, as test/mesh.test.js builds it.
  const vertices = [], indices = [];
  for (let j = 0; j <= 20; j++) for (let i = 0; i <= 20; i++) vertices.push(i - 10, 0, j - 10);
  for (let j = 0; j < 20; j++) for (let i = 0; i < 20; i++) { const a = j * 21 + i; indices.push(a, a + 21, a + 22, a, a + 22, a + 1); }
  let before = 0, mesh = null;
  for (let round = 0; round < 30; round++) {
    if (round === WARM) {
      await collect();
      before = heap();
    }
    const level = [
      world.addBody({ shape: 'mesh', vertices, indices }),
      world.addBody({ shape: 'heightfield', heights: new Array(21 * 21).fill(0), columns: 21, position: [30, 0, 0] }),
    ];
    if (round === WARM) mesh = new WeakRef(world._geometry[level[0].slot]);
    const bodies = [];
    for (let k = 0; k < 40; k++) {
      bodies.push(world.addBody({ shape: k % 2 ? 'sphere' : 'box', position: [(k % 20) - 9.5 + 30 * (k >= 20), 1, round % 3] }));
    }
    for (let s = 0; s < 30; s++) world.step(1 / 60);
    assert.ok(world._contacts.count >= 40, `round ${round}: only ${world._contacts.count} contacts`);
    for (const b of [...bodies, ...level]) world.remove(b);
    world.step(1 / 60);
    assert.equal(world._contacts.capacity, 0, `round ${round}: contact storage kept`);
    assert.equal(world._pairs.count, 0, `round ${round}: pairs kept`);
    assert.ok(world._geometry.every((g) => g === null), `round ${round}: triangles kept`);
    assert.ok(world._pairs.memory.length <= world._pairs.capacity && world._pairs.spare.length <= 40, `round ${round}: pair memories kept`);
  }
  await collect();
  assert.equal(mesh.deref(), undefined, 'a removed mesh\'s triangles are still held');
  const grown = heap() - before;
  assert.ok(grown < 256 * 1024, `grew ${(grown / 1024).toFixed(0)} KB`);
});

await test('removed bodies\' handles and destroyed worlds are collected', async () => {
  const world = new World();
  let handle = world.addBody({});
  const body = new WeakRef(handle);
  world.remove(handle);
  handle = null;

  let other = new World();
  for (let i = 0; i < 1000; i++) other.addBody({});
  const destroyed = new WeakRef(other);
  other.destroy();
  other = null;

  await collect();
  assert.equal(body.deref(), undefined, 'a removed body\'s handle is still held');
  assert.equal(destroyed.deref(), undefined, 'a destroyed world is still held');
  assert.equal(world.bodyCount, 0);
});

