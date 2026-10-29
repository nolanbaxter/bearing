// Sleeping: bodies at rest leave the step, whatever disturbs them brings them
// back, and the moved list says exactly what an engine has to draw again.
// Run: node test/sleep.test.js

import assert from 'node:assert/strict';
import { World } from '../src/bearing.js';
import { suite, test } from './test.js';

suite('sleep');

const ground = (world, extra = {}) =>
  world.addBody({ type: 'static', size: [100, 1, 100], position: [0, -0.5, 0], ...extra });
const steps = (world, seconds) => { for (let i = 0; i < Math.round(seconds * 60); i++) world.step(1 / 60); };
const stack = (world, x, n = 5, z = 0) => {
  const boxes = [];
  for (let k = 0; k < n; k++) boxes.push(world.addBody({ position: [x, 0.5 + k, z] }));
  return boxes;
};
const asleep = (bodies) => bodies.every((b) => b.asleep);
const awake = (bodies) => bodies.every((b) => !b.asleep);

test('a box dropped on the floor comes to rest and goes to sleep, stopped exactly', () => {
  const world = new World();
  const floor = ground(world);
  const box = world.addBody({ position: [0, 3, 0] });
  steps(world, 3);
  assert.ok(box.asleep);
  assert.deepEqual(box.velocity, [0, 0, 0]);
  assert.deepEqual(box.angularVelocity, [0, 0, 0]);
  assert.ok(Math.abs(box.position[1] - 0.5) < 0.005, `rests at ${box.position[1]}`);
  assert.equal(floor.asleep, false, 'a static body is never asleep');
  // Asleep, it doesn't move at all, step after step.
  const at = box.position;
  steps(world, 1);
  assert.deepEqual(box.position, at);
});

test('a body buzzing in place, going nowhere, still goes to sleep: what counts is how far, not how fast', () => {
  const world = new World();
  ground(world, { friction: 0 });
  const box = world.addBody({ position: [0, 0.5, 0], friction: 0 });
  // Shaken back and forth at 5 cm/s, ten times the speed that counts as
  // still for a 1 m box, but never more than a millimetre from where it is.
  let slept = false;
  for (let i = 0; i < 180 && !slept; i++) {
    box.setVelocity(i % 2 ? 0.05 : -0.05, 0, 0);
    world.step(1 / 60);
    slept = box.asleep;
  }
  assert.ok(slept, 'never slept');
});

test('a heap of turned boxes goes to sleep: friction pulls back the slip it let through', () => {
  // 288 boxes dropped turned, eight layers of six by six. Before milestone 16,
  // friction only stopped speeds, so a heap's contacts crept a few millimetres
  // a second in the solver's leftover error, and creepers out of step kept the
  // island awake: 235 of these were still awake after 40 s. Now friction pulls
  // back slip (contact.js, store) and the rest pose is taken again each second.
  const world = new World();
  ground(world);
  const boxes = [];
  for (let i = 0; i < 288; i++) {
    const x = i % 6, z = Math.floor(i / 6) % 6, y = Math.floor(i / 36);
    const a = [Math.sin(i * 1.3), Math.cos(i * 0.7), Math.sin(i * 2.1) + 0.1], l = Math.hypot(...a), t = ((i * 0.618034) % 1) * Math.PI;
    boxes.push(world.addBody({
      position: [(x - 3) * 1.8, 1 + y * 1.8, (z - 3) * 1.8],
      rotation: [...a.map((v) => v / l * Math.sin(t / 2)), Math.cos(t / 2)],
    }));
  }
  steps(world, 20);
  assert.ok(asleep(boxes), `${boxes.filter((b) => !b.asleep).length} still awake after 20 s`);
});

test('a steady creep slower than a pixel a second sleeps; a faster one never does', () => {
  // On ice nothing slows a box, so it keeps the speed it is given. A 1 m box's
  // pixel is 5 mm: 4 mm/s strays 4 mm in any second, 6 mm/s strays 6. Two
  // boxes side by side, one island, the second dropped 45 cm so it comes to
  // rest 0.3 s after the first: measured from where each first came to rest,
  // as before milestone 16, each was still from 1 s to 1.25 s of every 1.25 s,
  // never both at once, and they never slept.
  const run = (speed) => {
    const world = new World();
    ground(world, { friction: 0 });
    const box = world.addBody({ position: [0, 0.5, 0], friction: 0, velocity: [speed, 0, 0] });
    world.addBody({ position: [0, 0.95, 1], friction: 0, velocity: [speed, 0, 0] });
    steps(world, 10);
    return box;
  };
  assert.ok(run(0.004).asleep, '4 mm/s: awake');
  assert.ok(!run(0.006).asleep, '6 mm/s: asleep');
});

test('a resting stack sleeps as one island, and a ball into it wakes it and not a stack beside it', () => {
  const world = new World();
  ground(world);
  const left = stack(world, -3), right = stack(world, 3);
  steps(world, 2);
  assert.ok(asleep(left) && asleep(right));
  world.addBody({ shape: 'sphere', radius: 0.3, position: [-6, 2.5, 0], velocity: [8, 0, 0], density: 5000 });
  steps(world, 0.5);
  assert.ok(awake(left), 'the stack it hit is awake');
  assert.ok(asleep(right), 'the other stack slept on');
});

test('a stack woken by a touch is held up in the step it wakes, not dropped for one', () => {
  const world = new World();
  ground(world);
  const boxes = stack(world, 0, 3);
  steps(world, 2);
  assert.ok(asleep(boxes));
  // A ball rolling into the bottom box, slowly, so it barely pushes.
  world.addBody({ shape: 'sphere', radius: 0.25, position: [-0.75, 0.25, 0], velocity: [0.05, 0, 0] });
  world.step(1 / 60);
  assert.ok(awake(boxes));
  // Had the floor's and the boxes' own contacts waited a step, gravity alone
  // would have had each box falling at g dt, 0.16 m/s, by now.
  for (const b of boxes) assert.ok(Math.abs(b.velocity[1]) < 0.01, `falling at ${b.velocity[1]} m/s`);
});

test('everything that can disturb a sleeping body wakes its island', () => {
  const disturbances = {
    setPosition: (b) => b.setPosition(0, 0.6, 0),
    setRotation: (b) => b.setRotation([0, 0, 0.01, 1]),
    setAxisAngle: (b) => b.setAxisAngle([0, 1, 0], 0.1),
    setEuler: (b) => b.setEuler(0.1, 0, 0),
    setVelocity: (b) => b.setVelocity(0, 1, 0),
    setAngularVelocity: (b) => b.setAngularVelocity(0, 1, 0),
    applyForce: (b) => b.applyForce([0, 100, 0]),
    applyTorque: (b) => b.applyTorque([0, 100, 0]),
    applyImpulse: (b) => b.applyImpulse([0, 10, 0]),
    applyAngularImpulse: (b) => b.applyAngularImpulse([0, 10, 0]),
    setBody: (b, w) => w.setBody(b, { friction: 0.1 }),
  };
  for (const [name, disturb] of Object.entries(disturbances)) {
    const world = new World();
    ground(world);
    const boxes = stack(world, 0, 3);
    steps(world, 2);
    assert.ok(asleep(boxes), name);
    disturb(boxes[0], world);
    // The whole island, not just the body touched: the bottom box was
    // disturbed, and the ones on it wake with it.
    assert.ok(awake(boxes), `${name} woke the island`);
  }
});

test('a sleeping stack falls when what holds it goes: the floor removed, moved, or a box taken out', () => {
  {
    const world = new World();
    const floor = ground(world);
    const boxes = stack(world, 0, 3);
    steps(world, 2);
    world.remove(floor);
    steps(world, 0.5);
    assert.ok(boxes[0].position[1] < -0.5, `fell to ${boxes[0].position[1]}`);
  }
  {
    const world = new World();
    const floor = ground(world);
    const boxes = stack(world, 0, 3);
    steps(world, 2);
    floor.setPosition(0, -3, 0);
    steps(world, 2);
    // The floor is 1 m thick, so its top is at -2.5 and the bottom box rests at -2.
    assert.ok(Math.abs(boxes[0].position[1] - (-2)) < 0.01, `fell to the floor at ${boxes[0].position[1]}`);
  }
  {
    const world = new World();
    ground(world);
    const boxes = stack(world, 0, 3);
    steps(world, 2);
    world.remove(boxes[0]);
    steps(world, 2);
    assert.ok(Math.abs(boxes[1].position[1] - 0.5) < 0.01, `the box above fell to ${boxes[1].position[1]}`);
  }
});

test('changing gravity, or switching sleeping off, wakes everything', () => {
  const world = new World();
  ground(world);
  const boxes = stack(world, 0, 3);
  steps(world, 2);
  world.gravity = [0, -9.81, 1];
  assert.ok(awake(boxes));
  steps(world, 0.5);
  world.gravity = [0, -9.81, 0];
  steps(world, 3);
  assert.ok(asleep(boxes));
  world.sleeping = false;
  assert.ok(awake(boxes));
  steps(world, 3);
  assert.ok(awake(boxes), 'and nothing sleeps while it is off');
});

test('a kinematic platform that starts moving carries what slept on it', () => {
  const world = new World();
  const platform = world.addBody({ type: 'kinematic', size: [4, 0.5, 4], position: [0, 0.25, 0] });
  const box = world.addBody({ position: [0, 1, 0] });
  steps(world, 2);
  assert.ok(box.asleep && platform.asleep);
  platform.setVelocity(1, 0, 0);
  steps(world, 1);
  // Friction brings it up to the platform's speed, so it lags by v²/2μg.
  const carried = 1 - 1 / (2 * 0.5 * 9.81);
  assert.ok(Math.abs(box.position[0] - carried) < 0.005, `carried to ${box.position[0]}, not ${carried}`);
});

test('sleeping changes nothing that can be seen: a bounce, a slide, a slow slope and a slow tip end where they would awake', () => {
  const deg = Math.PI / 180, tilt = (a) => [0, 0, Math.sin(a / 2), Math.cos(a / 2)];
  const scenes = {
    bounce: (w) => { ground(w); w.addBody({ shape: 'sphere', radius: 0.25, restitution: 0.5, position: [0, 2, 0] }); },
    slide: (w) => { ground(w); w.addBody({ position: [0, 0.5, 0], velocity: [3, 0, 0] }); },
    // A quarter of a degree past friction's hold: creeps, then slides.
    slope: (w) => {
      const a = Math.atan(0.5) + 0.25 * deg;
      w.addBody({ type: 'static', size: [40, 0.5, 2], rotation: tilt(a) });
      w.addBody({ position: [-Math.sin(a) * 0.75, Math.cos(a) * 0.75, 0], rotation: tilt(a) });
    },
    // A cube on its edge, a degree past balance: tips over, slowly at first.
    // (Within 0.41° its edge's patch holds it up, sleeping or not.)
    tip: (w) => {
      ground(w);
      const a = 46 * deg, r = Math.SQRT1_2;
      w.addBody({ position: [-r * Math.sin(a - 45 * deg), r * Math.cos(a - 45 * deg), 0], rotation: tilt(a) });
    },
  };
  for (const [name, build] of Object.entries(scenes)) {
    const on = new World(), off = new World({ sleeping: false });
    build(on); build(off);
    steps(on, 6); steps(off, 6);
    const body = (w) => w.body(w._slots.id(1));
    const a = body(on).position, b = body(off).position;
    const apart = Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
    assert.ok(apart < on._size[1] / 200, `${name}: ${apart} m apart`);
    if (name !== 'slope') assert.ok(body(on).asleep, `${name}: asleep by the end`);
  }
});

test('a sleeping world steps in almost no time: nothing is narrowed, solved or filed', () => {
  const world = new World();
  ground(world);
  for (let i = 0; i < 200; i++) stack(world, (i % 20) * 2 - 20, 10, Math.floor(i / 20) * 2 - 10);
  const time = (n) => {
    const t = [];
    for (let i = 0; i < n; i++) { const a = performance.now(); world.step(1 / 60); t.push(performance.now() - a); }
    return t.sort((x, y) => x - y)[t.length >> 1];
  };
  const awakeStep = time(5);
  steps(world, 2);
  assert.equal(world._awakeCount, 0, 'all asleep');
  const calls = { query: 0, move: 0 };
  for (const tree of world._broadphase.trees) {
    const query = tree.query, move = tree.move;
    tree.query = (...args) => { calls.query++; return query.apply(tree, args); };
    tree.move = (...args) => { calls.move++; return move.apply(tree, args); };
  }
  const solved = world._solverSteps;
  const asleepStep = time(20);
  assert.deepEqual(calls, { query: 0, move: 0 });
  assert.equal(world._solverSteps, solved, 'the solver never ran');
  assert.equal(world._pairs.active, 0, 'no pair is narrowed');
  assert.ok(asleepStep < awakeStep / 100, `asleep ${asleepStep} ms a step, awake ${awakeStep} ms`);
});

test('off means off: with sleeping off, no island is ever looked for', () => {
  const world = new World({ sleeping: false });
  ground(world);
  stack(world, 0, 5);
  steps(world, 3);
  assert.equal(world._islandSteps, 0);
  assert.equal(world._awakeCount, 5);
});

test('the moved list is exact over frames of any number of steps, through sleep, wakes, teleports, adds and removes', () => {
  let seed = 9;
  const rand = () => ((seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0) / 4294967296);
  const world = new World();
  ground(world);
  const bodies = [];
  for (let i = 0; i < 40; i++) bodies.push(world.addBody({ position: [(i % 8) * 1.5 - 6, 0.5 + Math.floor(i / 8) * 1.2, 0] }));
  // What an engine holds: each body's last copied transform and previous,
  // updated only for the slots moved() lists.
  const mirror = new Float32Array(2 * 7 * 1024).fill(NaN);
  const copy = (s) => {
    for (let k = 0; k < 7; k++) {
      mirror[14 * s + k] = world.transforms[7 * s + k];
      mirror[14 * s + 7 + k] = world.previous[7 * s + k];
    }
  };
  let drawn = 0, frames = 0, listed = 0, none = 0;
  // A disturbance every 100 frames or so, of each kind in turn, so most of
  // the world spends most of the time asleep, as in a game.
  for (let frame = 0; frame < 2400; frame++) {
    const r = rand();
    if (r < 0.0025) bodies.push(world.addBody({ position: [rand() * 10 - 5, 6, 0] }));
    else if (r < 0.005 && bodies.length > 10) world.remove(bodies.splice(Math.floor(rand() * bodies.length), 1)[0]);
    else if (r < 0.0075) bodies[Math.floor(rand() * bodies.length)].setPosition(rand() * 10 - 5, 4, 0);
    else if (r < 0.01) bodies[Math.floor(rand() * bodies.length)].applyImpulse([0, 2000, 0]);
    const n = Math.floor(rand() * 4);   // 0 to 3 steps this frame
    for (let i = 0; i < n; i++) world.step(1 / 60);
    // Now and then a since older than the log keeps, which must still miss nothing.
    const since = rand() < 0.05 ? Math.max(0, drawn - 50) : drawn;
    const moved = world.moved(since);
    for (const s of moved) copy(s);
    listed += moved.length;
    if (moved.length === 0) none++;
    drawn = world.stepCount;
    for (const b of bodies) {
      const s = b.slot;
      for (let k = 0; k < 7; k++) {
        assert.equal(mirror[14 * s + k], world.transforms[7 * s + k], `frame ${frame}, slot ${s}`);
        assert.equal(mirror[14 * s + 7 + k], world.previous[7 * s + k], `frame ${frame}, slot ${s} previous`);
      }
    }
    frames++;
  }
  // And it isn't exact by listing everything: well under half on average,
  // and nothing at all while the whole world sleeps.
  assert.ok(listed < frames * bodies.length / 2, `listed ${listed} over ${frames} frames`);
  assert.ok(none > frames / 10, `only ${none} frames listed nothing`);
});

test('a world that has gone to sleep lists nothing as moved', () => {
  const world = new World();
  ground(world);
  stack(world, 0, 5);
  steps(world, 3);
  assert.equal(world._awakeCount, 0);
  assert.equal(world.moved(world.stepCount).length, 0);
});

test('the same scene twice gives the same bits through sleeping and waking', () => {
  const run = () => {
    const world = new World();
    ground(world);
    const boxes = stack(world, 0, 6);
    stack(world, 3, 6);
    steps(world, 2);
    boxes[2].applyImpulse([3000, 0, 0]);
    steps(world, 3);
    return world;
  };
  const a = run(), b = run();
  assert.deepEqual(a.transforms, b.transforms);
  assert.equal(a._awakeCount, b._awakeCount);
});

test('part of an island freezes while the rest moves, and thaws whole when touched or switched off', () => {
  const world = new World();
  ground(world);
  const boxes = stack(world, 0, 6);
  // A ball spun in place on top keeps the island awake for good.
  const ball = world.addBody({ shape: 'sphere', radius: 0.25, position: [0, 6.25, 0] });
  const spin = (seconds) => { for (let i = 0; i < seconds * 60; i++) { ball.setAngularVelocity(0, 5, 0); world.step(1 / 60); } };
  spin(3);
  // The four boxes two or more contacts from the ball stop exactly; the two under it don't.
  assert.deepEqual(boxes.map((b) => world._frozen[b.slot] !== 0), [true, true, true, true, false, false]);
  assert.ok(awake(boxes));
  assert.deepEqual(boxes[0].velocity, [0, 0, 0]);
  const at = boxes[0].position;
  spin(1);
  assert.deepEqual(boxes[0].position, at);
  // Set by hand, one body thaws the whole piece in the next step.
  boxes[0].setVelocity(0, 0, 0);
  world.step(1 / 60);
  assert.equal(world._frozenCount, 0);
  spin(3);
  assert.equal(world._frozenCount, 4);
  // Off means off: switching sleeping off thaws everything, and nothing freezes after.
  world.sleeping = false;
  assert.equal(world._frozenCount, 0);
  spin(3);
  assert.equal(world._frozenCount, 0);
});
