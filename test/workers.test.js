// Workers: the same bits with any number of them, the parallel path really
// taken, and nothing left running.
// Run: node --expose-gc test/workers.test.js

import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { World } from '../src/bearing.js';
import { cores, sharedMemory, SLEEPING } from '../src/core/jobs.js';
import { suite, testAsync as test } from './test.js';
import { mixed } from './scenes.js';

if (typeof globalThis.gc !== 'function') {
  throw new Error('run with node --expose-gc test/workers.test.js');
}

suite('workers');

// Node has shared memory, and CI's machines more than one thread. Where a
// world can have no workers, the tests that need them say so and are skipped:
// the suite still runs, on one thread, as Bearing does there.
const able = sharedMemory && cores() > 1;
/** A test that needs workers, skipped out loud where there can be none. */
const withWorkers = able ? test : async (name) => console.log(`  --  ${name}: skipped, no workers on this machine`);

const DT = 1 / 60;
const BEARING = new URL('../src/bearing.js', import.meta.url).href;
/** Everything a step reads, as save() writes it: equal strings are equal worlds, to the bit. */
const state = (world) => JSON.stringify(world.save());
/** Steps until the world's workers have started and reported ready; false if they never did. */
async function started(world, most = 600) {
  for (let n = 0; n < most && world._jobs.control === null; n++) world.step(DT);
  return world._jobs.ready();
}
/** A promise for each of a world's workers ending. */
const ends = (world) => world._jobs.workers.map((w) => new Promise((resolve) => w.once('exit', resolve)));

/** The benchmark's pile: 1,000 boxes dropped turned, in 7 layers. */
function pile(workers) {
  const world = new World({ workers });
  world.addBody({ type: 'static', size: [200, 1, 200], position: [0, -0.5, 0] });
  for (let i = 0; i < 1000; i++) {
    const t = ((i * 0.618034) % 1) * Math.PI, a = [Math.sin(i * 1.3), Math.cos(i * 0.7), Math.sin(i * 2.1) + 0.1];
    const l = Math.hypot(...a), s = Math.sin(t / 2) / l;
    world.addBody({
      position: [(i % 12 - 6) * 1.8, 1 + Math.floor(i / 144) * 1.8, (Math.floor(i / 12) % 12 - 6) * 1.8],
      rotation: [a[0] * s, a[1] * s, a[2] * s, Math.cos(t / 2)],
    });
  }
  return world;
}

await withWorkers('a world of everything steps to the same bits with 0, 2 and 4 workers', async () => {
  // Every colour handed to the workers, however small, so the parallel path
  // meets every kind of contact the world has: hulls, a compound (whose
  // steps prepare on one thread), a mesh, a heightfield, joints, a
  // character, a sensor and a kinematic platform.
  const worlds = [0, 2, 4].map((workers) => mixed({ workers }).world);
  for (const w of worlds.slice(1)) w._jobs.least = [1, 1, 1, 1];
  // In step, until the workers have started and are ready.
  for (const w of worlds) w.step(DT);
  for (const w of worlds.slice(1)) assert.ok(await w._jobs.ready(), 'the workers never reported ready');
  for (let n = 0; n < 300; n++) {
    for (const w of worlds) w.step(DT);
    if (n % 50 === 0) {
      const one = state(worlds[0]);
      for (const w of worlds.slice(1)) assert.equal(state(w), one, `step ${n}: the worlds differ`);
    } else {
      const one = Array.from(worlds[0].transforms);
      for (const w of worlds.slice(1)) assert.deepEqual(Array.from(w.transforms), one, `step ${n}: the transforms differ`);
    }
  }
  // Proof that it ran: the bits can't tell (see Jobs.parallel).
  for (const w of worlds.slice(1)) assert.ok(w._jobs.parallel > 1000, `only ${w._jobs.parallel} dispatches went to the workers`);
  assert.equal(worlds[0]._jobs, null, 'workers: 0 made a job system');
  for (const w of worlds) w.destroy();
});

await withWorkers('so does a landing pile of 1,000 boxes, at the thresholds as they ship', async () => {
  const one = pile(0), four = pile(4);
  // In step until the workers have started (the first step with a colour of
  // four starts them) and are ready, then 120 more.
  while (four._jobs.control === null) {
    one.step(DT);
    four.step(DT);
  }
  assert.ok(await four._jobs.ready(), 'the workers never reported ready');
  for (let n = 0; n < 120; n++) {
    one.step(DT);
    four.step(DT);
  }
  assert.equal(state(four), state(one));
  assert.ok(four._jobs.parallel > 1000, `only ${four._jobs.parallel} dispatches went to the workers`);
  one.destroy();
  four.destroy();
});

await withWorkers('workers sleep through the joints, which run on one thread', async () => {
  // Left spinning through a long joint pass, they slowed the stepping thread
  // (core/jobs.js, LULL_MS): a step's window opened, and a lull begun on a
  // stretch that last took 1 ms, twenty times the bound; no dispatch for 50 ms.
  const world = pile(2);
  assert.ok(await started(world));
  const jobs = world._jobs;
  assert.ok(jobs.begin(world, world._contacts, true), 'the window did not open');
  jobs.solo[0] = 1;
  jobs.lull(0);
  await new Promise((resolve) => setTimeout(resolve, 50));
  const asleep = Atomics.load(jobs.control, SLEEPING);
  jobs.end();
  assert.equal(asleep, jobs.workers.length, 'a worker was still spinning');
  // And a dispatch wakes them: the next steps still go to the workers, to the same bits.
  const one = pile(0);
  while (one.stepCount < world.stepCount) one.step(DT);
  const before = jobs.parallel;
  for (let n = 0; n < 30; n++) {
    one.step(DT);
    world.step(DT);
  }
  assert.ok(jobs.parallel > before, 'nothing went to the workers after they slept');
  assert.equal(state(world), state(one));
  one.destroy();
  world.destroy();
});

await withWorkers('a world too small to be worth them never starts any', async () => {
  // A stack: each box rests on the one below, so no colour holds four contacts.
  const world = new World({ workers: 2 });
  world.addBody({ type: 'static', size: [10, 1, 10], position: [0, -0.5, 0] });
  for (let i = 0; i < 6; i++) world.addBody({ position: [0, 0.5 + i, 0] });
  for (let n = 0; n < 120; n++) world.step(DT);
  assert.equal(world._jobs.control, null);
  assert.equal(world._jobs.workers.length, 0);
});

await withWorkers('destroy() ends the workers, and so does setting workers', async () => {
  const world = pile(2);
  assert.ok(await started(world));
  let ended = ends(world);
  // Two, or as many as the machine has threads to spare.
  assert.equal(ended.length, Math.min(2, cores() - 1));
  world.workers = 3;
  await Promise.all(ended);
  assert.equal(world.workers, 3);
  assert.ok(await started(world), 'the new ones never reported ready');
  ended = ends(world);
  world.destroy();
  await Promise.all(ended);
  assert.equal(world._jobs, null);
});

await withWorkers('a world dropped without destroy() ends its workers when it is collected', async () => {
  let world = pile(2);
  assert.ok(await started(world));
  const ended = ends(world);
  world = null;
  for (let i = 0; i < 20; i++) {
    await new Promise((resolve) => setTimeout(resolve, 10));
    globalThis.gc();
  }
  const timeout = new Promise((resolve) => setTimeout(() => resolve('still running'), 2000));
  assert.notEqual(await Promise.race([Promise.all(ended), timeout]), 'still running');
});

await withWorkers('a process whose world has workers running exits by itself', async () => {
  // Never destroyed: the workers are let go of when the program ends (rule 39).
  const code = `
    const { World } = await import(${JSON.stringify(BEARING)});
    const world = new World({ workers: 2 });
    world._jobs.least = [1, 1, 1, 1];
    world.addBody({ type: 'static', size: [20, 1, 20], position: [0, -0.5, 0] });
    for (let i = 0; i < 60; i++) world.addBody({ position: [(i % 6) * 1.1, 0.5 + Math.floor(i / 6), 0] });
    for (let n = 0; n < 30; n++) world.step(1 / 60);
    if (!(await world._jobs.ready())) process.exit(2);
    for (let n = 0; n < 30; n++) world.step(1 / 60);
    if (world._jobs.parallel === 0) process.exit(3);
  `;
  const run = spawnSync(process.execPath, ['--input-type=module', '-e', code], { timeout: 20000 });
  assert.equal(run.error, undefined, 'it did not exit within 20 s');
  assert.equal(run.status, 0, `exit ${run.status}: ${run.stderr}`);
});

await test('without shared memory, as on a page that is not cross-origin isolated: one thread, the same bits', async () => {
  const code = `
    delete globalThis.SharedArrayBuffer;
    const { World } = await import(${JSON.stringify(BEARING)});
    const { mixed } = await import(${JSON.stringify(new URL('./scenes.js', import.meta.url).href)});
    const { world } = mixed();
    if (world._jobs !== null) process.exit(2);
    for (let n = 0; n < 120; n++) world.step(1 / 60);
    process.stdout.write(JSON.stringify(world.save()));
  `;
  const run = spawnSync(process.execPath, ['--input-type=module', '-e', code], { timeout: 60000, maxBuffer: 1 << 28 });
  assert.equal(run.status, 0, `exit ${run.status}: ${run.stderr}`);
  // Against a world with workers, where there can be any.
  const { world } = mixed({ workers: 2 });
  if (world._jobs !== null) world._jobs.least = [1, 1, 1, 1];
  for (let n = 0; n < 120; n++) world.step(DT);
  assert.equal(String(run.stdout), state(world));
  world.destroy();
});
