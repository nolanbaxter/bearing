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
  // The first round is a warm-up and isn't measured: it is when V8 first
  // compiles the narrowphase and the solver, and compiled code lives in the
  // heap too. A leak grows with every round after it; compiled code doesn't.
  let before = 0;
  for (let round = 0; round < 50; round++) {
    if (round === 1) {
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

