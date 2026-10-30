// Saving and loading worlds, and debug lines: a loaded world steps exactly
// as the saved one, through JSON too; debug lines draw what is there.
// Run: node test/save.test.js

import assert from 'node:assert/strict';
import { World, Body } from '../src/bearing.js';
import { suite, test } from './test.js';
import { mixed } from './scenes.js';

suite('save');

const DT = 1 / 60;
const corners = (w, h, d) => {
  const out = [];
  for (const z of [-d / 2, d / 2]) for (const y of [-h / 2, h / 2]) for (const x of [-w / 2, w / 2]) out.push(x, y, z);
  return out;
};

const bits = (a, b) => a.length === b.length && Array.from(a).every((x, i) => Object.is(x, b[i]));

test('a world saved mid-run and loaded steps exactly as it would have, directly and through JSON', () => {
  const { world } = mixed();
  for (let n = 0; n < 150; n++) world.step(DT);
  world.body(world._slots.id(5)).applyForce([0, 300, 0], [3, 3, 3]);
  const data = world.save();
  const direct = World.load(data), viaJson = World.load(JSON.parse(JSON.stringify(data)));
  for (let n = 0; n < 600; n++) {
    world.step(DT); direct.step(DT); viaJson.step(DT);
    assert.ok(bits(world.transforms, direct.transforms), `step ${n}: the loaded world went its own way`);
    assert.ok(bits(world.transforms, viaJson.transforms), `step ${n}: the world through JSON went its own way`);
    assert.deepEqual(direct.contacts().map((e) => [e.type, e.a.id, e.b.id]), world.contacts().map((e) => [e.type, e.a.id, e.b.id]));
  }
  // Everything inside, not only where things are: the next save is the same, to the character.
  assert.equal(JSON.stringify(viaJson.save()), JSON.stringify(world.save()));
  assert.ok(world.sleeping && [...Array(12).keys()].some((i) => world.body(world._slots.id(3 + i)).asleep), 'the pile never slept');
});

test('save(load(save(w))) is save(w), and a loaded world keeps every id and every reading', () => {
  const { world, walker } = mixed();
  for (let n = 0; n < 90; n++) world.step(DT);
  const data = world.save(), loaded = World.load(data);
  assert.deepEqual(loaded.save(), data);
  assert.equal(loaded.bodyCount, world.bodyCount);
  assert.equal(loaded.jointCount, world.jointCount);
  assert.equal(loaded.stepCount, world.stepCount);
  assert.deepEqual(loaded.gravity, world.gravity);
  for (let s = 0; s < world._slots.top; s++) {
    if (world._slots.state[s] !== 1) continue;
    const id = world._slots.id(s), a = world.body(id), b = loaded.body(id);
    assert.ok(b instanceof Body && b.world === loaded);
    assert.deepEqual(loaded.bodyOf(b), world.bodyOf(a));
    assert.deepEqual(b.velocity, a.velocity);
    assert.equal(b.asleep, a.asleep);
  }
  assert.deepEqual(loaded.groundOf(loaded.body(walker.id)), (() => { const g = world.groundOf(walker); return g && { ...g, body: loaded.body(g.body.id) }; })());
  assert.deepEqual(loaded.raycast([0, 10, 0], [0, -1, 0]), (() => { const h = world.raycast([0, 10, 0], [0, -1, 0]); return { ...h, body: loaded.body(h.body.id) }; })());
});

test('a loaded world is its own: changing either leaves the other be, and it can be destroyed', () => {
  const { world } = mixed();
  for (let n = 0; n < 30; n++) world.step(DT);
  const loaded = World.load(world.save());
  const before = Array.from(world.transforms);
  loaded.addBody({ position: [0, 9, 0] });
  loaded.setBody(loaded.body(world._slots.id(3)), { position: [9, 9, 9] });
  for (let n = 0; n < 30; n++) loaded.step(DT);
  assert.deepEqual(Array.from(world.transforms), before);
  loaded.destroy();
  assert.throws(() => loaded.step(DT), /step: this world was destroyed/);
  world.step(DT);
});

test('the save holds every field of the world: a new one must change FORMAT and this list', () => {
  // A field added to World is saved without a word, but data saved before it
  // would load without it: save.js's FORMAT must change with the list.
  const fields = Object.keys(new World().save().objects[0].f).sort();
  assert.deepEqual(fields, [
    '_angular', '_angularDamping', '_awake', '_awakeCount', '_awakeList', '_axes', '_box', '_broadphase', '_calm',
    '_catchSteps', '_catches', '_caught', '_ceiling', '_characterCount', '_characterSteps', '_com', '_comGiven',
    '_contacts', '_damping', '_density', '_destroyed', '_dims', '_drives', '_epoch', '_events', '_fast', '_force',
    '_forced', '_forcedSlots', '_frame', '_framed', '_framedCount', '_friction', '_frozen', '_frozenCount',
    '_geometry', '_gravity', '_gravityScale', '_ground', '_groundAt', '_impacts', '_inAwake', '_inertia',
    '_inertiaGiven', '_inner', '_invI', '_invInertia', '_invMass', '_island', '_islandNext', '_islandSteps',
    '_jointHead', '_jointSteps', '_jointTally', '_joints', '_layers', '_least', '_lock', '_logAt', '_logCount',
    '_logFloor', '_logLast', '_logSlots', '_mask', '_mass', '_massGiven', '_maxSlope', '_motion', '_movedAt',
    '_movedOut', '_movedStatics', '_node', '_offset', '_pairEpoch', '_pairs', '_parent', '_position', '_previous',
    '_reach', '_rest', '_restitution', '_rotation', '_seen', '_seenMark', '_sensed', '_sensedCount',
    '_sensor', '_sensorCount', '_sensorPasses', '_shape', '_size', '_sleeping', '_slept', '_sleptCount', '_slopeCos',
    '_slots', '_solverSteps', '_start', '_startQ', '_staticMoved', '_stepCount', '_stepDt', '_stepGiven', '_stepHeight',
    '_still', '_stillReady', '_strays', '_straysAt', '_straysEpoch', '_substeps', '_torque', '_transforms', '_tumbled',
    '_type', '_velocity', '_wakePending', '_walk',
  ]);
});

test('load refuses what save() didn\'t write, data of another format, and damaged data', () => {
  assert.throws(() => World.load(null), /^Error: load: data must be what world.save\(\) returned, got null$/);
  assert.throws(() => World.load({ bodies: [] }), /^Error: load: data must be what world.save\(\) returned, got an object$/);
  const data = new World().save();
  assert.throws(() => World.load({ ...data, format: 0 }), /^Error: load: data is format 0; this Bearing reads format 8$/);
  const damaged = JSON.parse(JSON.stringify(data));
  damaged.objects[0].f._slots = { r: 999 };
  assert.throws(() => World.load(damaged), /^Error: load: data is damaged: a link to object 999, of \d+$/);
});

test('debug lines: every shape outlined, contacts, joints and boxes, nine numbers to a segment', () => {
  const world = new World({ gravity: [0, 0, 0] });
  const counts = (o) => world.debugLines(o).length / 9;
  world.addBody({ position: [0, 0, 0] });
  assert.equal(counts({ contacts: false, joints: false }), 12, 'a box');
  world.addBody({ shape: 'sphere', position: [5, 0, 0] });
  assert.equal(counts({ contacts: false, joints: false }), 12 + 3 * 16, 'and a ball: three rings');
  world.addBody({ shape: 'capsule', position: [10, 0, 0] });
  assert.equal(counts({ contacts: false, joints: false }), 12 + 48 + 2 * 16 + 4 + 4 * 8, 'and a capsule');
  world.addBody({ shape: 'hull', vertices: [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1], position: [15, 0, 0] });
  assert.equal(counts({ contacts: false, joints: false }), 12 + 48 + 68 + 6, 'and a tetrahedron: its six edges');
  world.addBody({ shape: 'compound', parts: [{}, { shape: 'sphere', position: [2, 0, 0] }], position: [20, 0, 0] });
  assert.equal(counts({ contacts: false, joints: false }), 12 + 48 + 68 + 6 + 12 + 48, 'and a compound: its parts');
  world.addBody({ shape: 'mesh', vertices: [0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1], indices: [0, 2, 1, 0, 3, 2], position: [0, -5, 0] });
  assert.equal(counts({ contacts: false, joints: false }), 194 + 6, 'and a mesh of two triangles');
  assert.equal(counts({ shapes: false, contacts: false, joints: false, boxes: true }), 12 * 6, 'a box round each of the 6 bodies');
  const a = world.addBody({ position: [0, 5, 0] });
  world.addJoint({ a, anchor: [0, 6, 0] });
  assert.equal(counts({ shapes: false, contacts: false }), 2, 'a joint to the world: centre to anchor, anchor to its point');
  // Contacts: one line out of each point of the last step's.
  const touching = new World();
  touching.addBody({ type: 'static', size: [10, 1, 10], position: [0, -0.5, 0] });
  touching.addBody({ position: [0, 0.5, 0] });
  touching.step(DT);
  assert.equal(touching.debugLines({ shapes: false, joints: false }).length / 9, 4, 'a box flat on the floor: four points');
  const lines = touching.debugLines();
  assert.ok(lines instanceof Float32Array && lines.length % 9 === 0);
  for (let k = 6; k < lines.length; k += 9) assert.ok(lines[k] >= 0 && lines[k] <= 1 && lines[k + 1] <= 1 && lines[k + 2] <= 1, 'colours are 0 to 1');
});

test('debug lines cost nothing until asked for, and reuse their array', () => {
  const world = new World();
  world.addBody({});
  for (let n = 0; n < 10; n++) world.step(DT);
  assert.equal(world._lines.length, 0);
  const first = world.debugLines();
  assert.equal(world.debugLines().buffer, first.buffer, 'a second call, the same array');
});

test('the same world saved twice gives the same data, to the character', () => {
  const run = () => {
    const { world } = mixed();
    for (let n = 0; n < 120; n++) world.step(DT);
    return JSON.stringify(world.save());
  };
  assert.equal(run(), run());
});
