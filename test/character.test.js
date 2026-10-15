// Character bodies and rotation locks: scripted walks over a test level, each
// ending where a hand-worked answer says.
// Run: node test/character.test.js

import assert from 'node:assert/strict';
import { World } from '../src/bearing.js';
import { suite, test } from './test.js';

suite('character');

const DT = 1 / 60, R = 0.3, H = 1.8;
// A capsule character's tolerance: a pixel, a 200th of its thinnest size.
const PX = (2 * R) / 200;
const near = (got, want, eps, what) => assert.ok(Math.abs(got - want) <= eps, `${what}: ${got}, expected ${want}`);
const floor = (world) => world.addBody({ type: 'static', size: [60, 1, 60], position: [0, -0.5, 0] });
const person = (world, at, more = {}) => world.addBody({ type: 'character', shape: 'capsule', radius: R, height: H, position: at, ...more });
/** Walks at (x, z) m/s for `seconds`, keeping its own vertical velocity, as a game would. */
const walk = (world, c, x, z, seconds, each) => {
  for (let i = 0; i < Math.round(seconds / DT); i++) {
    c.setVelocity(x, c.velocity[1], z);
    world.step(DT);
    if (each) each(i);
  }
};
/** A slope of `angle` rising along +x from x = 0, as a mesh of two triangles. */
const slope = (world, angle, length = 20) => {
  const top = length * Math.tan(angle);
  return world.addBody({ shape: 'mesh', vertices: [0, 0, -5, length, top, -5, length, top, 5, 0, 0, 5], indices: [0, 1, 2, 0, 2, 3] });
};

test('it falls, lands, and stands: on the ground it stops falling, and groundOf says on what', () => {
  const world = new World();
  const ground = floor(world);
  const c = person(world, [1, 3, 2]);
  assert.equal(world.groundOf(c), null);
  for (let i = 0; i < 90; i++) world.step(DT);
  near(c.position[1], H / 2, PX, 'stands at');
  assert.deepEqual(c.velocity, [0, 0, 0]);
  const g = world.groundOf(c);
  assert.equal(g.body.id, ground.id);
  assert.ok(Math.abs(g.normal[1] - 1) < 1e-12 && Math.abs(g.point[1]) < PX, `${g.normal} at ${g.point}`);
  // Walking 1 s at 2 m/s on the flat goes exactly 2 m.
  const x = c.position[0];
  walk(world, c, 2, 0, 1);
  near(c.position[0] - x, 2, 1e-9, 'walked');
  // A body that isn't a character has no ground to ask about.
  assert.equal(world.groundOf(ground), null);
});

test('into a wall at 30°, it slides along the wall at v cos 30°', () => {
  const world = new World();
  floor(world);
  const t = Math.PI / 6;
  // A wall whose face runs at 30° to +x, crossing the path ahead.
  world.addBody({ type: 'static', size: [20, 3, 0.5], position: [3, 1.5, 0], rotation: [0, Math.sin(-t / 2), 0, Math.cos(-t / 2)] });
  const c = person(world, [-3, H / 2, 0]);
  walk(world, c, 2, 0, 4);
  // Moving along the wall's face, at the part of 2 m/s along it.
  const before = c.position;
  walk(world, c, 2, 0, 0.5);
  const dx = c.position[0] - before[0], dz = c.position[2] - before[2];
  near(Math.hypot(dx, dz) / 0.5, 2 * Math.cos(t), 1e-9, 'speed along the wall');
  near(Math.atan2(dz, dx), t, 1e-9, 'heading along the wall');
  near(c.velocity[0] * Math.sin(t) - c.velocity[2] * Math.cos(t), 0, 1e-9, 'no velocity left into the wall');
});

test('up stairs of 15 cm risers to the top, and stopped by a 30 cm one', () => {
  const world = new World();
  floor(world);
  for (let k = 0; k < 5; k++) world.addBody({ type: 'static', size: [0.4, 0.15 * (k + 1), 3], position: [1.2 + k * 0.4, 0.075 * (k + 1), 0] });
  const top = world.addBody({ type: 'static', size: [4, 1.05, 3], position: [5, 0.525, 0] });
  const c = person(world, [0, H / 2, 0]);
  walk(world, c, 1.5, 0, 3);
  // A 1.8 m character's step height is a ninth of it, 0.2 m: on the top
  // tread, 0.75 m up, against the 30 cm riser at x = 3.
  near(c.position[1] - H / 2, 0.75, PX, 'feet on the top tread');
  near(c.position[0] + R, 3, PX, 'front against the tall riser');
  assert.notEqual(world.groundOf(c).body.id, top.id);
  // And back down, never leaving the ground.
  let airborne = 0;
  walk(world, c, -1.5, 0, 3, () => { if (!world.groundOf(c)) airborne++; });
  assert.equal(airborne, 0);
  near(c.position[1], H / 2, PX, 'back on the floor');
});

test('a step height scales with the character: a 20 cm one climbs 2 cm, not 3 cm', () => {
  for (const [rise, climbs] of [[0.02, true], [0.03, false]]) {
    const world = new World();
    floor(world);
    world.addBody({ type: 'static', size: [2, rise, 2], position: [1.2, rise / 2, 0] });
    const toy = world.addBody({ type: 'character', shape: 'capsule', radius: 0.05, height: 0.2, position: [0, 0.1, 0] });
    walk(world, toy, 0.5, 0, 2);
    assert.equal(toy.position[1] - 0.1 > rise / 2, climbs, `a ${rise} m step: feet at ${toy.position[1] - 0.1}`);
  }
});

test('up a 30° slope, rising d tan 30° for every d it runs; stopped at the foot of a 60° one', () => {
  const world = new World();
  floor(world);
  slope(world, Math.PI / 6);
  const c = person(world, [-1, H / 2, 0]);
  let airborne = 0;
  walk(world, c, 1.5, 0, 3, () => { if (!world.groundOf(c)) airborne++; });
  assert.equal(airborne, 0);
  near(c.position[0], -1 + 4.5, 1e-9, 'runs at the speed asked');
  // The capsule's lowest sphere rests on the plane: its centre r above it,
  // so its lowest point is x tan θ + r (1 / cos θ - 1) up.
  const x = c.position[0], t = Math.tan(Math.PI / 6);
  near(c.position[1] - H / 2, x * t + R * (1 / Math.cos(Math.PI / 6) - 1), PX, 'feet on the slope');

  const steep = new World();
  floor(steep);
  slope(steep, Math.PI / 3);
  const d = person(steep, [-1, H / 2, 0]);
  walk(steep, d, 1.5, 0, 3);
  // Its lowest sphere touching the 60° plane from the flat: centre at x where
  // the distance to the plane, -x sin 60° + r cos 60°, is r.
  near(d.position[0], (R * Math.cos(Math.PI / 3) - R) / Math.sin(Math.PI / 3), PX, 'stopped at the foot');
  near(d.position[1], H / 2, PX, 'and not up it');
});

test('dropped onto a slope too steep to stand on, it slides down it', () => {
  const world = new World();
  floor(world);
  slope(world, Math.PI / 3);
  const c = person(world, [3, 7, 0]);
  let slid = false, landedOnSlope = false;
  for (let i = 0; i < 120; i++) {
    const x = c.position[0];
    world.step(DT);
    if (c.position[1] - H / 2 > 0.5 && !world.groundOf(c) && c.position[0] < x) slid = true;
    if (c.position[1] - H / 2 > 0.5 && world.groundOf(c)) landedOnSlope = true;
  }
  assert.ok(slid && !landedOnSlope, 'never stood on the steep slope');
  near(c.position[1], H / 2, PX, 'down on the floor');
});

test('down a 20° slope at 4 m/s, and a 44° one at 15 m/s, it stays on the ground every step', () => {
  const world = new World();
  floor(world);
  const t = (20 * Math.PI) / 180, top = 8 * Math.tan(t);
  world.addBody({ shape: 'mesh', vertices: [0, top, -5, 8, 0, -5, 8, 0, 5, 0, top, 5], indices: [0, 1, 2, 0, 2, 3] });
  world.addBody({ type: 'static', size: [4, top, 10], position: [-2, top / 2, 0] });
  const c = person(world, [-1, top + H / 2, 0]);
  for (let i = 0; i < 5; i++) world.step(DT);
  let airborne = 0;
  walk(world, c, 4, 0, 2.5, () => { if (!world.groundOf(c)) airborne++; });
  assert.equal(airborne, 0);
  near(c.position[1], H / 2, PX, 'at the bottom');
  // Faster down a steeper one than a step a frame: 15 m/s down 44° drops 24 cm
  // a step, more than the 20 cm it could snap back down. It follows the slope
  // itself, so it doesn't fly off it.
  const steep = new World();
  floor(steep);
  const u = (44 * Math.PI) / 180, high = 20 * Math.tan(u);
  steep.addBody({ shape: 'mesh', vertices: [0, high, -5, 20, 0, -5, 20, 0, 5, 0, high, 5], indices: [0, 1, 2, 0, 2, 3] });
  steep.addBody({ type: 'static', size: [4, high, 10], position: [-2, high / 2, 0] });
  const d = person(steep, [-1, high + H / 2, 0]);
  for (let i = 0; i < 5; i++) steep.step(DT);
  airborne = 0;
  walk(steep, d, 15, 0, 1.2, () => { if (!steep.groundOf(d) && d.position[0] < 19) airborne++; });
  assert.equal(airborne, 0);
});

test('a moving platform carries it: 2 m in 1 s at 2 m/s, and 3 m walking along at 1 m/s', () => {
  for (const [own, expected] of [[0, 2], [1, 3]]) {
    const world = new World();
    floor(world);
    const platform = world.addBody({ type: 'kinematic', size: [6, 0.4, 6], position: [0, 2, 0], velocity: [2, 0, 0] });
    const c = person(world, [0, 2.2 + H / 2, 0]);
    world.step(DT);
    const x = c.position[0];
    walk(world, c, own, 0, 1);
    near(c.position[0] - x, expected, 1e-9, `walking ${own} m/s on it`);
    assert.equal(world.groundOf(c).body.id, platform.id);
    // Its velocity reads its own walk, not the platform's.
    near(c.velocity[0], own, 1e-12, 'own velocity');
  }
  // And a platform going down takes it down with it.
  const world = new World();
  floor(world);
  const lift = world.addBody({ type: 'kinematic', size: [3, 0.4, 3], position: [0, 3, 0], velocity: [0, -1, 0] });
  const c = person(world, [0, 3.2 + H / 2, 0]);
  world.step(DT);
  let airborne = 0;
  walk(world, c, 0, 0, 1, () => { if (!world.groundOf(c)) airborne++; });
  assert.equal(airborne, 0);
  near(c.position[1] - lift.position[1], 0.2 + H / 2, PX, 'riding it down');
});

test('a jump stops at a ceiling with no upward speed left, then lands', () => {
  const world = new World();
  const ground = floor(world);
  world.addBody({ type: 'static', size: [10, 0.5, 10], position: [0, 3.25, 0] });
  const c = person(world, [0, H / 2, 0]);
  world.step(DT);
  c.setVelocity(0, 6, 0);
  let head = 0, landed = false;
  for (let i = 0; i < 90; i++) {
    world.step(DT);
    head = Math.max(head, c.position[1] + H / 2);
    if (head > 3 - PX) assert.ok(c.velocity[1] <= 0, `still rising at the ceiling: ${c.velocity[1]}`);
    if (world.groundOf(c)) landed = true;
  }
  near(head, 3, PX, 'head at the ceiling');
  assert.ok(landed && world.groundOf(c).body.id === ground.id);
});

test('it is blocked by a crate rather than pushing it, holds one dropped on its head, and a zone sees it walk in', () => {
  const world = new World();
  floor(world);
  const crate = world.addBody({ size: [1, 1, 1], position: [3, 0.5, 0] });
  const zone = world.addBody({ type: 'static', size: [2, 2, 2], position: [-3, 1, 0], sensor: true });
  const c = person(world, [0, H / 2, 0]);
  walk(world, c, 1.5, 0, 2);
  near(crate.position[0], 3, 1e-9, 'the crate stayed put');
  near(c.position[0] + R, 2.5, PX, 'stopped at the crate');
  const box = world.addBody({ size: [0.5, 0.5, 0.5], position: [c.position[0], 3, 0] });
  for (let i = 0; i < 60; i++) world.step(DT);
  near(box.position[1], H + 0.25, 0.01, 'the box rests on its head');
  let entered = false;
  walk(world, c, -3, 0, 1.5, () => { for (const e of world.contacts(zone)) if (e.type === 'begin' && e.b.id === c.id) entered = true; });
  assert.ok(entered, 'the zone saw it');
});

test('a body locked upright doesn\'t tip when shoved, and still turns about y', () => {
  const tilt = (lock) => {
    const world = new World();
    floor(world);
    const b = world.addBody({ shape: 'capsule', radius: R, height: H, mass: 80, position: [0, H / 2, 0], lockRotation: lock });
    for (let i = 0; i < 30; i++) world.step(DT);
    b.applyImpulse([40, 0, 0], [0, 1.7, 0]);
    b.applyAngularImpulse([0, 20, 0]);
    for (let i = 0; i < 120; i++) world.step(DT);
    const q = b.rotation;
    return { up: 1 - 2 * (q[0] * q[0] + q[2] * q[2]), w: b.angularVelocity };
  };
  assert.ok(tilt([false, false, false]).up < 0.5, 'a free one falls over');
  const locked = tilt([true, false, true]);
  near(locked.up, 1, 1e-12, 'locked, it stays upright');
  assert.equal(locked.w[0], 0);
  assert.equal(locked.w[2], 0);
  assert.ok(locked.w[1] > 1, `and turns about y: ${locked.w[1]}`);
});

test('in 2.5D, a stack knocked over tumbles in its plane, every z where it started, to the bit', () => {
  const world = new World();
  world.addBody({ type: 'static', size: [40, 1, 4], position: [0, -0.5, 0] });
  const flat = [true, true, false];
  const bodies = [];
  for (let k = 0; k < 6; k++) bodies.push(world.addBody({ position: [0, 0.5 + k, 0], lockRotation: flat }));
  bodies.push(world.addBody({ shape: 'sphere', radius: 0.4, density: 8000, position: [-6, 4.2, 0], velocity: [15, 0, 0], lockRotation: flat }));
  for (let i = 0; i < 240; i++) world.step(DT);
  assert.ok(bodies.slice(0, 6).some((b) => Math.abs(b.position[0]) > 1), 'the stack was knocked over');
  for (const b of bodies) {
    assert.equal(b.position[2], 0);
    assert.equal(b.rotation[0], 0);
    assert.equal(b.rotation[1], 0);
  }
});

test('the same walk gives the same path, to the bit', () => {
  const run = () => {
    const world = new World();
    floor(world);
    slope(world, Math.PI / 6);
    for (let k = 0; k < 3; k++) world.addBody({ type: 'static', size: [0.4, 0.15 * (k + 1), 3], position: [-2 - k * 0.4, 0.075 * (k + 1), 2] });
    const c = person(world, [-1, H / 2, 1]);
    const path = [];
    walk(world, c, 1, 0.3, 2, () => path.push(...c.position));
    walk(world, c, -1.2, 0.2, 2, () => path.push(...c.position));
    return path;
  };
  assert.deepEqual(run(), run());
});

test('off means off: a world with no characters never runs their pass', () => {
  const world = new World();
  floor(world);
  world.addBody({ position: [0, 3, 0] });
  for (let i = 0; i < 60; i++) world.step(DT);
  assert.equal(world._characterSteps, 0);
  const c = person(world, [3, H / 2, 0]);
  world.step(DT);
  assert.equal(world._characterSteps, 1);
  world.remove(c);
  world.step(DT);
  assert.equal(world._characterSteps, 1);
  assert.equal(world._characterCount, 0);
});
