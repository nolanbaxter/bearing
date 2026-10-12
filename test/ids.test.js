// Ids and slots: reuse, generations, retirement, and shrinking without
// letting an old id come back to life.
// Run: node test/ids.test.js

import assert from 'node:assert/strict';
import { World } from '../src/bearing.js';
import { Slots, SLOT_SPAN, MAX_GENERATION, MAX_ID } from '../src/core/slots.js';
import { suite, test } from './test.js';

suite('ids');

test('an id is its generation times 2^24 plus its slot, and stays an exact number', () => {
  assert.equal(SLOT_SPAN, 2 ** 24);
  assert.equal(MAX_ID, Number.MAX_SAFE_INTEGER);
  assert.equal(MAX_ID & 0xffffff, SLOT_SPAN - 1);
  assert.equal((MAX_ID - (MAX_ID & 0xffffff)) / SLOT_SPAN, MAX_GENERATION);
  const world = new World();
  const a = world.addBody({});
  assert.equal(a.id, 1 * SLOT_SPAN + 0);
  assert.equal(a.slot, 0);
});

test('a freed slot is reused under the next generation', () => {
  const world = new World();
  const a = world.addBody({});
  world.addBody({});
  world.remove(a);
  const b = world.addBody({});
  assert.equal(b.slot, a.slot);
  assert.equal(b.id, a.id + SLOT_SPAN);
});

test('a slot whose generation runs out is retired and never handed out again', () => {
  const slots = new Slots();
  slots.resize(4);
  const s = slots.take();
  slots.take();
  slots.generations[s] = MAX_GENERATION;
  const last = slots.id(s);
  assert.equal(last, MAX_ID - (SLOT_SPAN - 1) + s);
  slots.release(s);
  assert.equal(slots.slotOf(last), -1);
  for (let i = 0; i < 2; i++) assert.notEqual(slots.take(), s);
  assert.equal(slots.count, 3);
  assert.equal(slots.top, 4);
});

test('entries for slots above top are stale, and are skipped rather than handed out twice', () => {
  const world = new World();
  const [a, b, c] = [world.addBody({}), world.addBody({}), world.addBody({})];
  world.remove(b);
  world.remove(c);   // top falls past both, leaving two stale entries
  const d = world.addBody({}), e = world.addBody({});
  assert.deepEqual([a.slot, d.slot, e.slot].sort(), [0, 1, 2]);
  assert.ok(a.alive && d.alive && e.alive);
  assert.ok(!b.alive && !c.alive);
});

test('after storage shrinks and regrows, no old id matches a new body', () => {
  const world = new World();
  const bodies = [];
  for (let i = 0; i < 100; i++) bodies.push(world.addBody({}));
  // Churn slot 80 so its generation climbs, keeping a handle from each life.
  const old = [];
  for (let i = 0; i < 5; i++) {
    old.push(bodies[80]);
    world.remove(bodies[80]);
    bodies[80] = world.addBody({});
  }
  old.push(bodies[80]);
  for (const b of bodies.slice(1)) world.remove(b);
  assert.equal(world.transforms.length, 1 * 7, 'shrunk to 1 slot');
  // Regrow past slot 80 and churn it again. Without the floor, slot 80 would
  // start over at generation 1 and reach the old handles' generations.
  for (let i = 0; i < 99; i++) bodies[i + 1] = world.addBody({});
  for (let i = 0; i < 10; i++) {
    for (const h of old) assert.equal(h.alive, false);
    world.remove(bodies[80]);
    bodies[80] = world.addBody({});
  }
});

test('random churn keeps every live id and slot distinct, and every dead id dead', () => {
  const world = new World();
  const live = [], dead = [];
  let seed = 7;
  const random = () => (seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648;
  for (let round = 0; round < 20000; round++) {
    if (live.length === 0 || random() < 0.55) live.push(world.addBody({}));
    else {
      const [body] = live.splice(Math.floor(random() * live.length), 1);
      world.remove(body);
      dead.push(body);
    }
  }
  assert.equal(world.bodyCount, live.length);
  assert.equal(new Set(live.map((b) => b.id)).size, live.length);
  assert.equal(new Set(live.map((b) => b.slot)).size, live.length);
  for (const b of live) assert.ok(b.alive);
  for (const b of dead) assert.ok(!b.alive);
});
