// Every error, word for word, both ways round.
// Run: node test/errors.test.js
//
// Each case below makes one call throw and names the API entry that documents
// it. The test checks three things: the message is exactly the one expected;
// that message matches a pattern in its entry's "Throws:" line (so nothing
// throws that the reference doesn't list); and every pattern in every entry
// is matched by some case (so nothing is listed that is never tested). In a
// pattern, `…` and `<…>` stand for a value.

import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { World } from '../src/bearing.js';
import { suite, test, thrown } from './test.js';

suite('errors');

const API = readFileSync(new URL('../docs/API.md', import.meta.url), 'utf8');

/** anchor -> the message patterns its Throws line lists, as regexes. */
const documented = new Map();
for (const m of API.matchAll(/<a id="([^"]+)"><\/a>\n###[^]*?(?=\n<a id=|$)/g)) {
  const line = m[0].split('\n').find((l) => l.startsWith('Throws:'));
  if (!line) continue;
  const patterns = [...line.matchAll(/`([^`]+)`/g)].map(([, text]) => {
    const parts = text.split(/…|<[^>]+>/).map((p) => p.replace(/[.*+?^${}()|[\]\\]/g, '\\$&'));
    return { text, regex: new RegExp(`^${parts.join('.+')}$`) };
  });
  documented.set(m[1], patterns);
}

const fresh = () => {
  const world = new World();
  return { world, body: world.addBody({}) };
};
const dead = () => {
  const { world, body } = fresh();
  world.remove(body);
  return { world, body };
};
const gone = () => {
  const { world, body } = fresh();
  world.destroy();
  return { world, body };
};

const OPTIONS = 'type, shape, size, radius, height, density, mass, position, rotation, velocity, angularVelocity';

/** [entry, the call, the exact message]. */
const cases = [
  ['world', () => new World(5), 'World: options must be an object, got 5'],
  ['world', () => new World({ gravty: [0, 0, 0] }), "World: unknown option 'gravty' (did you mean 'gravity'?)"],
  ['world', () => new World({ wind: 1 }), "World: unknown option 'wind' (the options are gravity)"],
  ['world', () => new World({ gravity: [0, -9.81] }), 'World: gravity must be 3 finite numbers, got 0,-9.81'],

  ['world-gravity', () => { const { world } = gone(); return world.gravity; }, 'gravity: this world was destroyed'],
  ['world-gravity', () => { fresh().world.gravity = [0, NaN, 0]; }, 'world.gravity must be 3 finite numbers, got 0,NaN,0'],
  ['world-bodycount', () => gone().world.bodyCount, 'bodyCount: this world was destroyed'],
  ['world-bodycount', () => { fresh().world.bodyCount = 3; }, 'world.bodyCount is read only'],
  ['world-transforms', () => gone().world.transforms, 'transforms: this world was destroyed'],
  ['world-transforms', () => { fresh().world.transforms = null; }, 'world.transforms is read only'],
  ['world-previous', () => gone().world.previous, 'previous: this world was destroyed'],
  ['world-previous', () => { fresh().world.previous = null; }, 'world.previous is read only'],

  ...['addBody', 'setBody'].flatMap((call) => {
    const run = (o) => () => {
      const { world, body } = fresh();
      return call === 'addBody' ? world.addBody(o) : world.setBody(body, o);
    };
    const name = call === 'addBody' ? 'options' : 'changes';
    const e = `world-${call.toLowerCase()}`;
    return [
      [e, () => { const { world, body } = gone(); return call === 'addBody' ? world.addBody({}) : world.setBody(body, {}); }, `${call}: this world was destroyed`],
      [e, run('box'), `${call}: ${name} must be an object, got 'box'`],
      [e, run({ postion: [0, 0, 0] }), `${call}: unknown option 'postion' (did you mean 'position'?)`],
      [e, run({ Mass: 5 }), `${call}: unknown option 'Mass' (did you mean 'mass'?)`],
      [e, run({ banana: 1 }), `${call}: unknown option 'banana' (the options are ${OPTIONS})`],
      [e, run({ type: 'fixed' }), `${call}: type must be 'dynamic', 'static' or 'kinematic', got 'fixed'`],
      [e, run({ shape: 'cube' }), `${call}: shape must be 'box', 'sphere' or 'capsule', got 'cube'`],
      [e, run({ radius: 0.5 }), `${call}: a box has no radius`],
      [e, run({ shape: 'sphere', height: 2 }), `${call}: a sphere has no height`],
      [e, run({ size: [1, 0, 1] }), `${call}: size must be 3 positive numbers, got 1,0,1`],
      [e, run({ shape: 'sphere', radius: -2 }), `${call}: radius must be a positive number, got -2`],
      [e, run({ shape: 'capsule', height: '2' }), `${call}: height must be a positive number, got '2'`],
      [e, run({ shape: 'capsule', height: 0.8 }), `${call}: height must be at least twice radius (1), got 0.8`],
      [e, run({ mass: 1, density: 1 }), `${call}: give mass or density, not both`],
      [e, run({ mass: 0 }), `${call}: mass must be a positive number, got 0`],
      [e, run({ density: Infinity }), `${call}: density must be a positive number, got Infinity`],
      [e, run({ position: { x: 1, y: 2, z: 3 } }), `${call}: position must be 3 finite numbers, got an object`],
      [e, run({ rotation: [0, 0, 0, 0] }), `${call}: rotation must be 4 finite numbers, not all zero, got 0,0,0,0`],
      [e, run({ type: 'static', velocity: [0, 0, 0] }), `${call}: a static body has no velocity`],
      [e, run({ velocity: [1, 2] }), `${call}: velocity must be 3 finite numbers, got 1,2`],
      [e, run({ angularVelocity: null }), `${call}: angularVelocity must be 3 finite numbers, got null`],
    ];
  }),
  ['world-addbody', () => { const world = new World(); world._slots.limit = 2; world.addBody(); world.addBody(); world.addBody(); },
    'addBody: a world holds at most 16777216 bodies (the slot field of an id is 24 bits)'],
  ['world-setbody', () => fresh().world.setBody(5, {}), 'setBody: body must be a Body, got 5'],
  ['world-setbody', () => new World().setBody(fresh().body, {}), 'setBody: that body belongs to another world'],
  ['world-setbody', () => { const { world, body } = dead(); world.setBody(body, {}); }, 'setBody: this body was removed'],

  ['world-bodyof', () => { const { world, body } = gone(); world.bodyOf(body); }, 'bodyOf: this world was destroyed'],
  ['world-bodyof', () => fresh().world.bodyOf({ id: 0 }), 'bodyOf: body must be a Body, got an object'],
  ['world-bodyof', () => new World().bodyOf(fresh().body), 'bodyOf: that body belongs to another world'],
  ['world-bodyof', () => { const { world, body } = dead(); world.bodyOf(body); }, 'bodyOf: this body was removed'],

  ['world-remove', () => { const { world, body } = gone(); world.remove(body); }, 'remove: this world was destroyed'],
  ['world-remove', () => fresh().world.remove(undefined), 'remove: body must be a Body, got undefined'],
  ['world-remove', () => new World().remove(fresh().body), 'remove: that body belongs to another world'],

  ['world-body', () => gone().world.body(0), 'body: this world was destroyed'],
  ['world-body', () => fresh().world.body(1.5), 'body: id must be a whole number from 0 to 9007199254740991, got 1.5'],

  ['world-step', () => gone().world.step(1 / 60), 'step: this world was destroyed'],
  ['world-step', () => fresh().world.step(0), 'step: dt must be a positive number of seconds, got 0'],

  ...['id', 'slot', 'world', 'alive'].map((p) => [`body-${p}`, () => { fresh().body[p] = 1; }, `body.${p} is read only`]),
  ...[['mass', 'setBody(body, { mass })'], ['position', 'setPosition(x, y, z)'], ['rotation', 'setRotation(q)'],
    ['velocity', 'setVelocity(x, y, z)'], ['angularVelocity', 'setAngularVelocity(x, y, z)']].flatMap(([p, hint]) => [
    [`body-${p.toLowerCase()}`, () => gone().body[p], `${p}: this world was destroyed`],
    [`body-${p.toLowerCase()}`, () => dead().body[p], `${p}: this body was removed`],
    [`body-${p.toLowerCase()}`, () => { fresh().body[p] = 1; }, `body.${p} is read only; use ${hint}`],
  ]),

  ...[['setPosition', [0, 0, 0]], ['setRotation', [[0, 0, 0, 1]]], ['setAxisAngle', [[0, 1, 0], 1]],
    ['setEuler', [0, 0]], ['setVelocity', [0, 0, 0]], ['setAngularVelocity', [0, 0, 0]]].flatMap(([m, args]) => [
    [`body-${m.toLowerCase()}`, () => gone().body[m](...args), `${m}: this world was destroyed`],
    [`body-${m.toLowerCase()}`, () => dead().body[m](...args), `${m}: this body was removed`],
  ]),
  ['body-setposition', () => fresh().body.setPosition(1, NaN, 0), 'setPosition: x, y and z must be finite numbers, got 1,NaN,0'],
  ['body-setrotation', () => fresh().body.setRotation([0, 0, 1]), 'setRotation: q must be 4 finite numbers, not all zero, got 0,0,1'],
  ['body-setaxisangle', () => fresh().body.setAxisAngle([0, 0, 0], 1), 'setAxisAngle: axis must be 3 finite numbers, not all zero, got 0,0,0'],
  ['body-setaxisangle', () => fresh().body.setAxisAngle([0, 1, 0], '90'), "setAxisAngle: radians must be a finite number, got '90'"],
  ['body-seteuler', () => fresh().body.setEuler(0, undefined), 'setEuler: yaw, pitch and roll must be finite numbers, got 0,undefined,0'],
  ['body-setvelocity', () => fresh().body.setVelocity(1, 2), 'setVelocity: x, y and z must be finite numbers, got 1,2,undefined'],
  ['body-setvelocity', () => new World().addBody({ type: 'static' }).setVelocity(1, 0, 0), 'setVelocity: a static body has no velocity'],
  ['body-setangularvelocity', () => fresh().body.setAngularVelocity(0, Infinity, 0), 'setAngularVelocity: x, y and z must be finite numbers, got 0,Infinity,0'],
  ['body-setangularvelocity', () => new World().addBody({ type: 'static' }).setAngularVelocity(0, 1, 0), 'setAngularVelocity: a static body has no angularVelocity'],
  ['body-destroy', () => gone().body.destroy(), 'destroy: this world was destroyed'],
];

test('every case throws exactly its message', () => {
  for (const [entry, call, message] of cases) {
    assert.equal(thrown(call), message, `in ${entry}`);
  }
});

test('every message thrown is listed in its entry', () => {
  for (const [entry, , message] of cases) {
    const patterns = documented.get(entry);
    assert.ok(patterns, `${entry} has no Throws line, but throws '${message}'`);
    assert.ok(patterns.some((p) => p.regex.test(message)), `${entry} doesn't list '${message}'`);
  }
});

test('every message listed is thrown by some case', () => {
  for (const [entry, patterns] of documented) {
    for (const p of patterns) {
      const hit = cases.some(([e, , message]) => e === entry && p.regex.test(message));
      assert.ok(hit, `${entry} lists '${p.text}', and no case throws it`);
    }
  }
});

test('a call that throws changes nothing', () => {
  const world = new World();
  const body = world.addBody({ position: [1, 2, 3], velocity: [4, 5, 6] });
  const before = JSON.stringify(world.bodyOf(body));
  // Valid options first, the bad one last: none of them may land.
  assert.throws(() => world.setBody(body, { position: [9, 9, 9], mass: 5, velocity: [1, 2] }));
  assert.throws(() => world.setBody(body, { shape: 'sphere', radius: 1, height: 3 }));
  assert.throws(() => body.setPosition(7, 7, NaN));
  assert.equal(JSON.stringify(world.bodyOf(body)), before);
  assert.equal(world.bodyCount, 1);
  assert.throws(() => world.addBody({ position: [0, 0, 0], radius: 1 }));
  assert.equal(world.bodyCount, 1);
});
