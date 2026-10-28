// V8's fast properties: every object the step reads its columns through
// keeps them, made new, stepped, saved and loaded. An object given more than
// a dozen fields by computed name (this[name] = ...) becomes a dictionary,
// and every read of one of its fields a hash lookup: World and Contacts were,
// until 2026-10-28, and naming their fields first made the 1,000-box pile 19
// to 25% faster. A field added outside a constructor's named list would bring
// it back without failing any other test.
// Run: node --allow-natives-syntax test/fast.test.js

import assert from 'node:assert/strict';
import { World } from '../src/bearing.js';
import { suite, test } from './test.js';
import { mixed } from './scenes.js';

suite('fast properties');

const DT = 1 / 60;

/** The objects the step reads its columns through, by name; a world without joints has no joint store yet (off means off). */
function held(world) {
  return {
    World: world, Contacts: world._contacts, Pairs: world._pairs, Joints: world._joints,
    Forest: world._joints?.forest, Broadphase: world._broadphase, Slots: world._slots,
  };
}

function check(world, when) {
  for (const [name, o] of Object.entries(held(world))) {
    if (o == null) continue;
    assert.ok(%HasFastProperties(o), `${when}: ${name} has dictionary properties`);
  }
}

test('a world and what it holds keep fast properties, new, stepped, and loaded', () => {
  check(new World(), 'a new world');
  const { world } = mixed({ workers: 0 });
  check(world, 'built');
  for (let s = 0; s < 120; s++) world.step(DT);
  check(world, 'stepped');
  const loaded = World.load(JSON.parse(JSON.stringify(world.save())));
  check(loaded, 'loaded');
  for (let s = 0; s < 60; s++) loaded.step(DT);
  check(loaded, 'loaded and stepped');
  world.destroy();
  loaded.destroy();
});
