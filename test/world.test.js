// Worlds and bodies: making, changing, reading, removing and stepping them.
// Run: node test/world.test.js

import assert from 'node:assert/strict';
import { World, Body } from '../src/bearing.js';
import { suite, test } from './test.js';

suite('world');

const close = (a, b, tolerance, what) => {
  for (let i = 0; i < a.length; i++) {
    assert.ok(Math.abs(a[i] - b[i]) <= tolerance, `${what}[${i}]: ${a[i]} vs ${b[i]}`);
  }
};

test('addBody({}) makes a 1 m box of water at rest at the origin', () => {
  const world = new World();
  const body = world.addBody({});
  assert.ok(body instanceof Body);
  assert.equal(world.bodyCount, 1);
  assert.equal(body.mass, 1000);
  assert.deepEqual(world.bodyOf(body), {
    type: 'dynamic', shape: 'box', size: [1, 1, 1], density: 1000,
    position: [0, 0, 0], rotation: [0, 0, 0, 1], velocity: [0, 0, 0], angularVelocity: [0, 0, 0],
  });
  assert.deepEqual(world.addBody().position, [0, 0, 0]);
});

test('mass is density times volume, for every shape', () => {
  const world = new World();
  const r = 0.3, h = 1.8;
  assert.equal(world.addBody({ size: [2, 3, 4], density: 10 }).mass, 240);
  close([world.addBody({ shape: 'sphere', radius: r }).mass], [1000 * (4 / 3) * Math.PI * r ** 3], 1e-9, 'sphere');
  const capsule = 1000 * (Math.PI * r * r * (h - 2 * r) + (4 / 3) * Math.PI * r ** 3);
  close([world.addBody({ shape: 'capsule', radius: r, height: h }).mass], [capsule], 1e-9, 'capsule');
  assert.equal(world.addBody({ shape: 'capsule', mass: 80 }).mass, 80);
});

test('a given mass survives a change of shape; a density mass follows it', () => {
  const world = new World();
  const fixed = world.addBody({ mass: 5 });
  world.setBody(fixed, { size: [2, 2, 2] });
  assert.equal(fixed.mass, 5);
  const water = world.addBody({});
  world.setBody(water, { size: [2, 1, 1] });
  assert.equal(water.mass, 2000);
  world.setBody(fixed, { density: 1 });
  assert.equal(fixed.mass, 8);
  assert.equal(world.bodyOf(fixed).density, 1);
  assert.equal(world.bodyOf(fixed).mass, undefined);
});

test('bodyOf round-trips through addBody, for every shape and type', () => {
  const world = new World();
  const bodies = [
    world.addBody({ type: 'static', size: [20, 1, 20], position: [0, -0.5, 0] }),
    world.addBody({ shape: 'sphere', radius: 0.25, density: 2500, velocity: [1, 2, 3] }),
    world.addBody({ type: 'kinematic', shape: 'capsule', radius: 0.3, height: 1.8, mass: 80, angularVelocity: [0, 1, 0] }),
  ];
  for (const body of bodies) {
    const twin = world.addBody(world.bodyOf(body));
    assert.deepEqual(world.bodyOf(twin), world.bodyOf(body));
    assert.notEqual(twin.id, body.id);
  }
});

test('what is read is a copy: changing it changes nothing', () => {
  const world = new World();
  const body = world.addBody({ position: [1, 2, 3] });
  const info = world.bodyOf(body);
  info.position[0] = 99;
  assert.deepEqual(body.position, [1, 2, 3]);
  assert.ok(Object.isFrozen(body.position));
  assert.ok(Object.isFrozen(world.gravity));
});

test('a caller\'s array is copied in, so changing it later does nothing', () => {
  const world = new World();
  const at = [1, 2, 3];
  const body = world.addBody({ position: at });
  at[0] = 99;
  assert.deepEqual(body.position, [1, 2, 3]);
});

test('typed arrays are taken wherever arrays are', () => {
  const world = new World({ gravity: new Float32Array([0, -1, 0]) });
  const body = world.addBody({ position: new Float64Array([1, 2, 3]), rotation: new Float32Array([0, 0, 0, 1]) });
  assert.deepEqual(body.position, [1, 2, 3]);
  assert.deepEqual(world.gravity, [0, -1, 0]);
});

test('setters chain, and rotations are normalized as they come in', () => {
  const world = new World();
  const body = world.addBody({});
  assert.equal(body.setPosition(0, 5, 0).setVelocity(2, 0, 0).setRotation([0, 0, 0, 2]), body);
  assert.deepEqual(body.position, [0, 5, 0]);
  assert.deepEqual(body.velocity, [2, 0, 0]);
  assert.deepEqual(body.rotation, [0, 0, 0, 1]);
  body.setRotation([0, 1e200, 0, 1e200]);
  close(body.rotation, [0, Math.SQRT1_2, 0, Math.SQRT1_2], 1e-15, 'huge quaternion');
});

test('setAxisAngle and setEuler agree with each other and with the formulas', () => {
  const world = new World();
  const a = world.addBody({}).setAxisAngle([0, 2, 0], Math.PI / 2);
  close(a.rotation, [0, Math.SQRT1_2, 0, Math.SQRT1_2], 1e-15, 'axis-angle');
  const b = world.addBody({}).setEuler(Math.PI / 2, 0);
  close(b.rotation, a.rotation, 1e-15, 'yaw');
  const c = world.addBody({}).setEuler(0.3, -0.7, 1.1);
  // YXZ: yaw about Y, then pitch about X, then roll about Z.
  const mul = (p, q) => [
    p[3] * q[0] + p[0] * q[3] + p[1] * q[2] - p[2] * q[1],
    p[3] * q[1] + p[1] * q[3] + p[2] * q[0] - p[0] * q[2],
    p[3] * q[2] + p[2] * q[3] + p[0] * q[1] - p[1] * q[0],
    p[3] * q[3] - p[0] * q[0] - p[1] * q[1] - p[2] * q[2],
  ];
  const about = (axis, t) => [...axis.map((k) => k * Math.sin(t / 2)), Math.cos(t / 2)];
  const expected = mul(mul(about([0, 1, 0], 0.3), about([1, 0, 0], -0.7)), about([0, 0, 1], 1.1));
  close(c.rotation, expected, 1e-15, 'euler');
});

test('free fall follows semi-implicit Euler exactly', () => {
  const world = new World();
  const body = world.addBody({ position: [0, 100, 0] });
  const dt = 1 / 60, n = 120;
  for (let i = 0; i < n; i++) world.step(dt);
  // v_n = g n dt; x_n = x_0 + g dt^2 n (n + 1) / 2
  close(body.velocity, [0, -9.81 * n * dt, 0], 1e-12, 'velocity');
  close(body.position, [0, 100 - 9.81 * dt * dt * n * (n + 1) / 2, 0], 1e-11, 'position');
});

test('gravity can change at any time, and the next step uses it', () => {
  const world = new World();
  const body = world.addBody({});
  world.gravity = [0, 0, 0];
  world.step(1);
  assert.deepEqual(body.velocity, [0, 0, 0]);
  world.gravity = [1, 0, 0];
  world.step(1);
  assert.deepEqual(body.velocity, [1, 0, 0]);
});

test('static stays put, kinematic moves at its velocity and ignores gravity', () => {
  const world = new World();
  const ground = world.addBody({ type: 'static', position: [0, -1, 0] });
  const lift = world.addBody({ type: 'kinematic', velocity: [0, 2, 0] });
  for (let i = 0; i < 30; i++) world.step(1 / 60);
  assert.deepEqual(ground.position, [0, -1, 0]);
  close(lift.position, [0, 1, 0], 1e-14, 'lift');
  assert.deepEqual(lift.velocity, [0, 2, 0]);
});

test('a body made static stops; made dynamic again it falls from rest', () => {
  const world = new World();
  const body = world.addBody({ velocity: [3, 0, 0], angularVelocity: [0, 1, 0] });
  world.setBody(body, { type: 'static' });
  assert.deepEqual(body.velocity, [0, 0, 0]);
  assert.deepEqual(body.angularVelocity, [0, 0, 0]);
  assert.equal(world.bodyOf(body).velocity, undefined);
  world.setBody(body, { type: 'dynamic' });
  world.step(1);
  assert.deepEqual(body.velocity, [0, -9.81, 0]);
});

test('angular velocity turns a body about the world axis it names', () => {
  const world = new World({ gravity: [0, 0, 0] });
  const body = world.addBody({ angularVelocity: [0, Math.PI, 0] });
  for (let i = 0; i < 600; i++) world.step(1 / 600);   // half a turn
  const [x, y, z, w] = body.rotation;
  // Half a turn about Y is [0, 1, 0, 0]; the small-step integration lands close.
  close([x, Math.abs(y), z, w], [0, 1, 0, 0], 2e-3, 'half turn');
  close([Math.hypot(x, y, z, w)], [1], 1e-15, 'unit length');
});

test('transforms carry position and rotation; previous holds the step before', () => {
  const world = new World();
  const a = world.addBody({ position: [1, 2, 3] });
  const b = world.addBody({ position: [4, 5, 6], rotation: [0, 0, 1, 0] });
  const t = world.transforms;
  assert.deepEqual([...t.subarray(b.slot * 7, b.slot * 7 + 7)], [4, 5, 6, 0, 0, 1, 0]);
  assert.deepEqual([...world.previous.subarray(a.slot * 7, a.slot * 7 + 7)], [1, 2, 3, 0, 0, 0, 1]);
  world.step(0.1);
  const before = world.transforms[a.slot * 7 + 1];
  world.step(0.1);
  assert.equal(world.previous[a.slot * 7 + 1], before);
  assert.equal(world.transforms[a.slot * 7 + 1], Math.fround(a.position[1]));
});

test('moving a body by hand moves previous too: a teleport, not a slide', () => {
  const world = new World();
  const body = world.addBody({});
  world.step(1 / 60);
  body.setPosition(10, 0, 0);
  assert.equal(world.previous[body.slot * 7], 10);
  world.setBody(body, { rotation: [0, 1, 0, 0] });
  assert.equal(world.previous[body.slot * 7 + 4], 1);
  // Changing velocity alone is not a teleport.
  world.step(1 / 60);
  const prev = world.previous[body.slot * 7 + 1];
  body.setVelocity(0, 1, 0);
  assert.equal(world.previous[body.slot * 7 + 1], prev);
});

test('a removed body\'s handle stops working; its slot goes to a new id', () => {
  const world = new World();
  const a = world.addBody({});
  const keep = world.body(a.id);
  world.remove(a);
  assert.equal(a.alive, false);
  assert.equal(keep.alive, false);
  world.remove(a);   // removing twice does nothing
  a.destroy();
  const b = world.addBody({});
  assert.equal(b.slot, a.slot);
  assert.notEqual(b.id, a.id);
  assert.equal(a.alive, false);
  assert.equal(world.body(b.id).id, b.id);
  assert.equal(world.bodyCount, 1);
});

test('handles hold only a world and an id', () => {
  const world = new World();
  const body = world.addBody({});
  const again = world.body(body.id);
  assert.notEqual(again, body);
  assert.equal(again.id, body.id);
  again.setPosition(1, 1, 1);
  assert.deepEqual(body.position, [1, 1, 1]);
  assert.deepEqual(Object.keys(body), []);
});

test('two worlds never touch', () => {
  const one = new World(), two = new World({ gravity: [0, 0, 0] });
  const a = one.addBody({}), b = two.addBody({});
  assert.equal(a.id, b.id);
  one.step(1);
  assert.deepEqual(b.velocity, [0, 0, 0]);
  assert.throws(() => two.remove(a), { message: 'remove: that body belongs to another world' });
});

test('storage grows as bodies are added and shrinks as they go', () => {
  const world = new World();
  assert.equal(world.transforms.length, 0);
  const bodies = [];
  for (let i = 0; i < 1000; i++) bodies.push(world.addBody({}));
  assert.equal(world.transforms.length, 1024 * 7);
  for (const b of bodies.slice(10)) world.remove(b);
  assert.ok(world.transforms.length <= 40 * 7, `shrank to ${world.transforms.length / 7} slots`);
  for (const b of bodies.slice(0, 10)) assert.ok(b.alive);
  for (const b of bodies.slice(0, 10)) world.remove(b);
  assert.equal(world.transforms.length, 0);
});

test('a destroyed world lets go of everything and refuses to be used', () => {
  const world = new World();
  const body = world.addBody({});
  world.destroy();
  world.destroy();
  assert.equal(body.alive, false);
  assert.throws(() => world.addBody({}), { message: 'addBody: this world was destroyed' });
  assert.throws(() => body.position, { message: 'position: this world was destroyed' });
});

test('a step does what it did before, bit for bit', () => {
  const run = () => {
    const world = new World();
    const bodies = [];
    for (let i = 0; i < 50; i++) {
      bodies.push(world.addBody({
        shape: ['box', 'sphere', 'capsule'][i % 3],
        position: [i * 0.37, i * 1.1, -i * 0.13],
        velocity: [Math.sqrt(i), 1 / (i + 1), -i / 7],
        angularVelocity: [i / 3, -i / 5, i / 11],
      }).setAxisAngle([1, i, -i / 2], i * 0.7));
    }
    for (let s = 0; s < 240; s++) world.step(1 / 60);
    return [...world.transforms];
  };
  const a = run(), b = run();
  assert.ok(a.every((x, i) => Object.is(x, b[i])));
});
