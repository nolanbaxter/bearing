// Nothing grows forever: the permanent leak test.
// Run: node --expose-gc test/soak.test.js
//
// Adds and removes 10,000 bodies over and over, past the 255 reuses that
// would retire a slot under Winding's 8-bit generations, and checks that the
// world's storage and memory come back to where they started each time. Then
// checks, with WeakRef, that removed bodies' handles and destroyed worlds are
// collected rather than held by something inside.

import assert from 'node:assert/strict';
import { World, Joint } from '../src/bearing.js';
import { suite, testAsync as test } from './test.js';
import { mixed } from './scenes.js';

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

await test('a step makes no new arrays or views, whatever the world holds', async () => {
  // Rule 76. Memory can't tell this apart: V8 boxes decimals passed between
  // functions it doesn't inline, short-lived heap numbers it clears almost
  // for free (measured, milestone 16). So count the calls that make a new
  // array or a view of one, in a world of every kind of body, joint, sensor
  // and character. Before milestone 16, three copies made a subarray() view
  // each: the sleep rule's, and two in the joint solver.
  const { world } = mixed({ sleeping: false });
  for (let n = 0; n < 60; n++) world.step(1 / 60);
  const typed = Object.getPrototypeOf(Int8Array.prototype);
  const watched = [[typed, 'subarray'], [typed, 'slice'], [typed, 'map'], [typed, 'filter'],
    [Array.prototype, 'slice'], [Array.prototype, 'map'], [Array.prototype, 'filter'], [Array.prototype, 'concat']];
  const calls = new Map();
  const saved = watched.map(([o, k]) => {
    const f = o[k];
    o[k] = function (...args) { calls.set(k, (calls.get(k) || 0) + 1); return f.apply(this, args); };
    return f;
  });
  try {
    for (let n = 0; n < 300; n++) world.step(1 / 60);
  } finally {
    watched.forEach(([o, k], i) => { o[k] = saved[i]; });
  }
  assert.deepEqual([...calls], []);
});

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

await test('hulls and compounds made, piled on a mesh and removed 30 times: their shapes and frames go', async () => {
  const world = new World();
  world.step(1 / 60);
  let seed = 7;
  const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
  let before = 0, hull = null;
  for (let round = 0; round < 30; round++) {
    if (round === WARM) {
      await collect();
      before = heap();
    }
    const floor = world.addBody({ shape: 'mesh', vertices: [-10, 0, -10, 10, 0, -10, 10, 0, 10, -10, 0, 10], indices: [0, 2, 1, 0, 3, 2] });
    const bodies = [];
    for (let k = 0; k < 40; k++) {
      const pts = [];
      for (let i = 0; i < 12; i++) pts.push(rnd() - 0.5, rnd() - 0.5, rnd() - 0.5);
      bodies.push(world.addBody(k % 2
        ? { shape: 'hull', vertices: pts, position: [(k % 10) * 1.5 - 7, 1 + Math.floor(k / 10), 0] }
        : { shape: 'compound', parts: [{ size: [0.6, 0.3, 0.3] }, { shape: 'hull', vertices: pts, position: [0.5, 0, 0] }], position: [(k % 10) * 1.5 - 7, 1 + Math.floor(k / 10), 3] }));
    }
    if (round === WARM) hull = new WeakRef(world._inner[bodies[1].slot]);
    for (let s = 0; s < 60; s++) world.step(1 / 60);
    // Landed and touching: some may already sleep, and sleeping pairs hold no contacts.
    assert.ok(world._contacts.count >= 20, `round ${round}: only ${world._contacts.count} contacts`);
    for (const b of [...bodies, floor]) world.remove(b);
    world.step(1 / 60);
    assert.equal(world._framedCount, 0, `round ${round}: frames kept`);
    assert.ok(world._inner.every((g) => g === null) && world._geometry.every((g) => g === null), `round ${round}: shapes kept`);
    assert.equal(world._contacts.capacity, 0, `round ${round}: contact storage kept`);
    assert.ok(world._pairs.memory.length <= world._pairs.capacity && world._pairs.spare.length <= 80, `round ${round}: pair memories kept`);
  }
  await collect();
  assert.equal(hull.deref(), undefined, 'a removed hull is still held');
  const grown = heap() - before;
  // Measured on Node 22: with --jitless it grew 329 KB by round 30 and held
  // there to round 70 (scratch grown once to the largest hull), and with the
  // JIT 452 KB by round 30. One round's bodies hold 374 KB, so even a third
  // of a round kept each round would show as 2.5 MB over the 20 measured
  // rounds: 1 MB tells the two apart.
  assert.ok(grown < 1024 * 1024, `grew ${(grown / 1024).toFixed(0)} KB`);
});

await test('a world saved, loaded and drawn 30 times: the loaded worlds go, and the drawing keeps one array', async () => {
  const world = new World();
  world.addBody({ type: 'static', size: [20, 1, 20], position: [0, -0.5, 0] });
  for (let i = 0; i < 60; i++) world.addBody({ shape: i % 2 ? 'sphere' : 'box', position: [(i % 6) - 3, 1 + Math.floor(i / 6), 0] });
  let before = 0, loaded = null, lines = null;
  for (let round = 0; round < 30; round++) {
    if (round === WARM) {
      await collect();
      before = heap();
    }
    for (let s = 0; s < 5; s++) world.step(1 / 60);
    const copy = World.load(JSON.parse(JSON.stringify(world.save())));
    for (let s = 0; s < 5; s++) copy.step(1 / 60);
    if (round === WARM) loaded = new WeakRef(copy);
    if (round % 2) copy.destroy();
    const drawn = world.debugLines({ boxes: true });
    if (round > WARM) assert.equal(drawn.buffer, lines, `round ${round}: a new array for the same drawing`);
    lines = drawn.buffer;
  }
  await collect();
  assert.equal(loaded.deref(), undefined, 'a loaded world is still held');
  const grown = heap() - before;
  // One loaded world holds 371 KB, measured, so one kept each round would be
  // 7.4 MB over the 20 measured rounds: 1 MB tells them apart, as V8's
  // warm-up, a few hundred KB in the other tests, doesn't reach.
  assert.ok(grown < 1024 * 1024, `grew ${(grown / 1024).toFixed(0)} KB`);
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
  // V8 goes on compiling the mesh paths for far longer than the other tests'
  // warm-up: measured on Node 22, growth climbed to about 450 KB by round 90
  // and then held, and with --jitless it was 27 KB after 150 rounds. Node 20
  // reached 278 KB here. One mesh and heightfield hold 166 KB, measured, so a
  // level kept each round would show as 3.3 MB over the 20 measured rounds,
  // and a third of one as 1.1 MB: 1 MB tells the two apart.
  assert.ok(grown < 1024 * 1024, `grew ${(grown / 1024).toFixed(0)} KB`);
});

await test('a pile landing past sensors and cleared 30 times: event storage, sensor pairs and memory come back', async () => {
  const world = new World();
  world.addBody({ type: 'static', size: [40, 1, 40], position: [0, -0.5, 0] });
  world.step(1 / 60);
  let before = 0;
  for (let round = 0; round < 30; round++) {
    if (round === WARM) {
      await collect();
      before = heap();
    }
    // Zones the pile falls through, and a pile that lands all at once: a
    // burst of begins, then, cleared, a burst of ends.
    const zones = [], pile = [];
    for (let k = 0; k < 8; k++) zones.push(world.addBody({ type: 'static', size: [3, 1, 3], position: [(k % 4) * 3 - 4.5, 3, Math.floor(k / 4) * 3 - 1.5], sensor: true }));
    for (let k = 0; k < 200; k++) pile.push(world.addBody({ position: [(k % 10) * 1.01 - 5, 0.5 + Math.floor(k / 20) * 1.01, (Math.floor(k / 10) % 2) * 1.01 + (round % 3) * 0.1] }));
    let begun = 0, ended = 0;
    for (let s = 0; s < 40; s++) {
      world.step(1 / 60);
      for (const e of world.contacts()) e.type === 'begin' ? begun++ : ended++;
    }
    assert.ok(begun > 200, `round ${round}: only ${begun} begins`);
    for (const b of [...pile, ...zones]) world.remove(b);
    world.step(1 / 60);
    const cleared = world.contacts().length;
    assert.ok(cleared > 200, `round ${round}: only ${cleared} ends`);
    world.step(1 / 60);
    // Back to the smallest room it keeps, as the pair list does: 16.
    assert.ok(world._events.capacity <= 16, `round ${round}: room for ${world._events.capacity} events kept`);
    assert.ok(world._sensed.length <= 16, `round ${round}: ${world._sensed.length} sensor pairs kept room for`);
    assert.equal(world._sensorCount, 0);
  }
  await collect();
  const grown = heap() - before;
  // As the mesh test's: V8 compiling this scene's paths, not Bearing's data.
  // Measured on Node 22, growth climbed to about 285 KB by round 70 and then
  // held, and with --jitless it was 5 KB after 50 rounds; Node 20 reached
  // 303 KB here. A round's handles and events, kept, hold about 130 KB, so a
  // kept round would show as 2.6 MB over the 20 measured rounds: 1 MB tells
  // the two apart. Its storage is held to its size above, round by round.
  assert.ok(grown < 1024 * 1024, `grew ${(grown / 1024).toFixed(0)} KB`);
});

await test('chains and ragdoll-like trees made, swung and removed 30 times: joint storage, trees and memory come back', async () => {
  const world = new World();
  world.addBody({ type: 'static', size: [40, 1, 40], position: [0, -0.5, 0] });
  world.step(1 / 60);
  const start = world.bodyCount;
  let before = 0, joint = null;
  for (let round = 0; round < 30; round++) {
    if (round === WARM) {
      await collect();
      before = heap();
    }
    // 20 chains of 10 links, hung from points and swung, half with limits
    // and motors, and a loop in each, so every part of the solver runs.
    const bodies = [];
    for (let c = 0; c < 20; c++) {
      const x = (c % 5) * 2 - 4, z = Math.floor(c / 5) * 2 - 3;
      let last = null;
      for (let i = 0; i < 10; i++) {
        const link = world.addBody({ size: [0.04, 0.2, 0.04], position: [x, 5 - 0.2 * (i + 0.5), z], velocity: [i * 0.1, 0, 0] });
        const type = c % 2 ? 'hinge' : 'ball';
        const j = world.addJoint(last
          ? { type, a: link, b: last, anchor: [x, 5 - 0.2 * i, z], ...(c % 2 ? { min: -1, max: 1, motorForce: 1 } : { cone: 1 }) }
          : { type, a: link, anchor: [x, 5, z] });
        if (round === WARM && !joint) joint = new WeakRef(j);
        bodies.push(link);
        last = link;
      }
      world.addJoint({ type: 'distance', a: bodies[bodies.length - 1], b: bodies[bodies.length - 4], min: 0 });
    }
    for (let s = 0; s < 30; s++) world.step(1 / 60);
    assert.equal(world.jointCount, 220, `round ${round}: joints`);
    assert.ok(world._joints.forest.count > 0, `round ${round}: no trees solved`);
    // Every other round the joints go first, by hand; else with their bodies.
    if (round % 2) {
      for (let j = world._joints.slots.top - 1; j >= 0; j--) {
        if (world._joints.slots.state[j] === 1) world.remove(new Joint(world, world._joints.slots.id(j)));
      }
    }
    for (const b of bodies) world.remove(b);
    world.step(1 / 60);
    assert.equal(world.jointCount, 0, `round ${round}: joints left`);
    assert.equal(world.bodyCount, start, `round ${round}: bodies left`);
    assert.equal(world._joints.slots.capacity, 0, `round ${round}: joint storage kept`);
    assert.equal(world._joints.forest.capacity, 0, `round ${round}: tree storage kept`);
    assert.ok(world._jointHead.every((x) => x === 0) && world._jointTally.every((x) => x === 0), `round ${round}: a body's list of joints kept`);
  }
  await collect();
  assert.equal(joint.deref(), undefined, "a removed joint's handle is still held");
  const grown = heap() - before;
  // V8's compiling, not a leak: measured with --jitless it held at 326 KB
  // from round 30 to round 50, and Node 20 in CI reached 265 KB here. One
  // round's chains hold 889 KB, so even a quarter of one kept each round
  // would show as 4.4 MB over the 20 measured rounds: 1 MB tells them apart.
  assert.ok(grown < 1024 * 1024, `grew ${(grown / 1024).toFixed(0)} KB`);
});

await test('characters made, walked about a level and removed 30 times: nothing of them stays', async () => {
  const world = new World();
  world.addBody({ type: 'static', size: [40, 1, 40], position: [0, -0.5, 0] });
  for (let k = 0; k < 4; k++) world.addBody({ type: 'static', size: [1, 0.15 * (k + 1), 6], position: [2 + k, 0.075 * (k + 1), 0] });
  const lift = world.addBody({ type: 'kinematic', size: [3, 0.4, 3], position: [-4, 0.2, 0], velocity: [0, 0.5, 0] });
  world.step(1 / 60);
  let before = 0;
  for (let round = 0; round < 30; round++) {
    if (round === WARM) {
      await collect();
      before = heap();
    }
    const walkers = [];
    for (let k = 0; k < 20; k++) {
      walkers.push(world.addBody({ type: 'character', shape: 'capsule', radius: 0.3, height: 1.8, position: [(k % 5) * 1.5 - 4, 1.5, Math.floor(k / 5) * 1.5 - 2] }));
    }
    for (let s = 0; s < 40; s++) {
      for (const c of walkers) c.setVelocity(s < 20 ? 2 : -2, c.velocity[1], 0.3);
      world.step(1 / 60);
    }
    if (round % 2) lift.setVelocity(0, -0.5, 0); else lift.setVelocity(0, 0.5, 0);
    for (const c of walkers) world.remove(c);
    world.step(1 / 60);
    assert.equal(world._characterCount, 0, `round ${round}: characters counted`);
    assert.equal(world.bodyCount, 6, `round ${round}: bodies left`);
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

