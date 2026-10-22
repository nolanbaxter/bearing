// Continuous collision: fast bodies stopped by thin ones, at the moment of
// impact, and nothing touched that the shapes would have missed.
// Run: node test/ccd.test.js

import assert from 'node:assert/strict';
import { World } from '../src/bearing.js';
import { suite, test } from './test.js';

suite('ccd');

const DT = 1 / 60;
const speedOf = (b) => Math.hypot(...b.velocity);

test('a 1 cm ball at 500 m/s stops on a 1 cm wall, head on and at angles', () => {
  for (const angle of [0, 30, 60]) {
    const c = Math.cos(angle * Math.PI / 180), s = Math.sin(angle * Math.PI / 180);
    for (let k = 0; k < 20; k++) {
      const world = new World({ gravity: [0, 0, 0] });
      world.addBody({ type: 'static', size: [0.01, 4, 4], position: [0, 0, 0] });
      // Started a different fraction of a step's travel (8.3 m) from the wall each time.
      const from = 3 + k * 0.4137;
      const ball = world.addBody({ shape: 'sphere', radius: 0.005, position: [-from * c, -from * s, 0], velocity: [500 * c, 500 * s, 0] });
      for (let n = 0; n < 10; n++) {
        world.step(DT);
        assert.ok(ball.position[0] < 0, `${angle}°, start ${k}: through, at x ${ball.position[0]}`);
      }
    }
  }
});

test('a 1 cm ball at 500 m/s stops on a mesh wall and on a heightfield', () => {
  for (let k = 0; k < 20; k++) {
    const world = new World({ gravity: [0, 0, 0] });
    world.addBody({ shape: 'mesh', vertices: [0, -2, -2, 0, 2, -2, 0, 2, 2, 0, -2, 2], indices: [0, 1, 2, 0, 2, 3] });
    const ball = world.addBody({ shape: 'sphere', radius: 0.005, position: [-3 - k * 0.4137, 0.1, 0.2], velocity: [500, 0, 0] });
    for (let n = 0; n < 10; n++) {
      world.step(DT);
      assert.ok(ball.position[0] < 0, `mesh, start ${k}: through, at x ${ball.position[0]}`);
    }
  }
  for (let k = 0; k < 20; k++) {
    const world = new World({ gravity: [0, 0, 0] });
    world.addBody({ shape: 'heightfield', heights: new Array(25).fill(0).map((_, i) => 0.05 * (i % 5)), columns: 5, spacing: 1 });
    const ball = world.addBody({ shape: 'sphere', radius: 0.005, position: [0.3, 3 + k * 0.4137, -0.6], velocity: [0, -500, 0] });
    for (let n = 0; n < 10; n++) {
      world.step(DT);
      // Struck aslant, it slides down the slope, 0.05 m to a metre along x, and off the edge at x = -2.
      const [x, y] = ball.position;
      if (x >= -2) assert.ok(y > 0.1 + 0.05 * x, `heightfield, start ${k}: through, at y ${y}, x ${x}`);
    }
  }
});

test('a 1 cm ball at 500 m/s stays on its side of a thin dynamic box it strikes', () => {
  for (let k = 0; k < 20; k++) {
    const world = new World({ gravity: [0, 0, 0] });
    const plate = world.addBody({ size: [0.01, 1, 1], position: [0, 0, 0] });
    const ball = world.addBody({ shape: 'sphere', radius: 0.005, position: [-3 - k * 0.4137, 0.1, 0], velocity: [500, 0, 0] });
    for (let n = 0; n < 10; n++) {
      world.step(DT);
      // Its centre in the plate's own frame stays on the side it came from.
      const [x, y, z, w] = plate.rotation, d = ball.position.map((p, i) => p - plate.position[i]);
      const along = (1 - 2 * (y * y + z * z)) * d[0] + 2 * (x * y + w * z) * d[1] + 2 * (x * z - w * y) * d[2];
      assert.ok(along < 0, `start ${k}, step ${n}: through, ${along} along the plate's x`);
    }
  }
});

test('two 1 cm balls meeting head on at 500 m/s each bounce back, never through each other', () => {
  for (const off of [0, 0.002, 0.006]) {
    const world = new World({ gravity: [0, 0, 0] });
    const a = world.addBody({ shape: 'sphere', radius: 0.005, position: [-4.1, 0, 0], velocity: [500, 0, 0], restitution: 1 });
    const b = world.addBody({ shape: 'sphere', radius: 0.005, position: [4.1, off, 0], velocity: [-500, 0, 0], restitution: 1 });
    for (let n = 0; n < 6; n++) {
      world.step(DT);
      const gap = Math.hypot(...a.position.map((p, i) => p - b.position[i])) - 0.01;
      assert.ok(gap > -0.0001, `off ${off}, step ${n}: overlapping by ${-gap}`);
    }
    assert.ok(a.velocity[0] < 0 && b.velocity[0] > 0, `off ${off}: a ${a.velocity}, b ${b.velocity}`);
  }
});

test('no ghost hits: a ball passing a box\'s edge with room to spare keeps its velocity exactly', () => {
  for (const speed of [1, 10, 100, 300]) {
    for (const clear of [0.01, 0.05, 0.2]) {
      for (let k = 0; k < 10; k++) {
        const world = new World({ gravity: [0, 0, 0] });
        world.addBody({ type: 'static', size: [1, 1, 1], position: [0, 0, 0] });
        const ball = world.addBody({ shape: 'sphere', radius: 0.1, position: [-3 - k * 0.0371, 0.6 + clear, 0], velocity: [speed, 0, 0] });
        for (let n = 0; n < Math.ceil(6 * 60 / speed) + 2; n++) world.step(DT);
        assert.deepEqual(Array.from(ball.velocity), [speed, 0, 0], `${speed} m/s, ${clear} m clear, start ${k}`);
      }
    }
  }
});

test('a 20 cm ball through a 30 cm gap comes out at its speed', () => {
  for (const speed of [10, 50, 200]) {
    const world = new World({ gravity: [0, 0, 0] });
    world.addBody({ type: 'static', size: [1, 1, 1], position: [0, 0.65, 0] });
    world.addBody({ type: 'static', size: [1, 1, 1], position: [0, -0.65, 0] });
    const ball = world.addBody({ shape: 'sphere', radius: 0.1, position: [-4, 0, 0], velocity: [speed, 0, 0] });
    for (let n = 0; n < 120; n++) world.step(DT);
    assert.deepEqual(Array.from(ball.velocity), [speed, 0, 0], `${speed} m/s`);
    assert.ok(ball.position[0] > 1, `${speed} m/s: at x ${ball.position[0]}`);
  }
});

test('a 10 cm rod at 200 m/s, spinning at up to 80 rad/s, never passes a 2 cm wall, wherever in a step it reaches it', () => {
  // 50 starts spread over a whole step's travel (3.3 m). Spread over 0.65 m
  // only, as this test once was, it missed 5 starts in 100 that went through.
  for (let k = 0; k < 50; k++) {
    const world = new World({ gravity: [0, 0, 0] });
    world.addBody({ type: 'static', size: [0.02, 4, 4], position: [0, 0, 0] });
    const rod = world.addBody({ size: [0.1, 0.02, 0.02], position: [-3 - k * 0.0731, 0, 0], velocity: [200, 0, 0], angularVelocity: [0, 0, 30 + k] });
    for (let n = 0; n < 30; n++) world.step(DT);
    assert.ok(rod.position[0] < 0, `start ${k}, spin ${30 + k} rad/s: through, at x ${rod.position[0]}`);
  }
});

/** A fixed run of numbers in [0, 1), the same every time. */
const numbers = (seed) => () => (seed = (seed * 16807) % 2147483647) / 2147483647;

/**
 * `make`'s body thrown at `speed` along x, from any of a step's starts, at
 * a wall `at` builds: whether it ends past the wall's middle, x = 0.
 */
function thrown(at, size, speed, spin, rnd) {
  const world = new World({ gravity: [0, 0, 0] });
  at(world);
  const body = world.addBody({
    size, position: [-3 - rnd() * speed / 60, 0, 0], rotation: [rnd() - 0.5, rnd() - 0.5, rnd() - 0.5, 1],
    velocity: [speed, 0, 0], angularVelocity: [spin * rnd(), spin * rnd(), spin * rnd()],
  });
  for (let n = 0; n < 40; n++) world.step(DT);
  return { world, body, through: body.position[0] > 0 };
}

test('bodies turned any way, spinning or not, never pass a thin wall: rods, small cubes and planks', () => {
  // Before the core sweep (World._catch), turned rods went through a 1 cm wall 57 times in 100.
  const wall = (thick) => (world) => world.addBody({ type: 'static', size: [thick, 6, 6] });
  for (const [what, size, speed, spin, thick] of [
    ['a rod', [0.1, 0.02, 0.02], 200, 0, 0.01], ['a spinning rod', [0.1, 0.02, 0.02], 200, 100, 0.01],
    ['a 1 cm cube', [0.01, 0.01, 0.01], 500, 0, 0.01], ['a plank', [1, 0.1, 0.1], 50, 0, 0.05],
  ]) {
    const rnd = numbers(5);
    for (let i = 0; i < 40; i++) assert.ok(!thrown(wall(thick), size, speed, spin, rnd).through, `${what}, throw ${i}: through`);
  }
});

test('turned rods never pass a mesh wall or a heightfield either', () => {
  const rnd = numbers(11);
  const mesh = (world) => world.addBody({ shape: 'mesh', vertices: [0, -3, -3, 0, 3, -3, 0, 3, 3, 0, -3, 3], indices: [0, 1, 2, 0, 2, 3] });
  for (let i = 0; i < 30; i++) assert.ok(!thrown(mesh, [0.1, 0.02, 0.02], 200, 0, rnd).through, `mesh, throw ${i}: through`);
  for (let i = 0; i < 30; i++) {
    const world = new World({ gravity: [0, 0, 0] });
    world.addBody({ shape: 'heightfield', heights: new Array(25).fill(0), columns: 5, spacing: 2 });
    const rod = world.addBody({ size: [0.1, 0.02, 0.02], position: [0.3, 3 + rnd() * 200 / 60, -0.4], rotation: [rnd() - 0.5, rnd() - 0.5, rnd() - 0.5, 1], velocity: [0, -200, 0] });
    for (let n = 0; n < 40; n++) world.step(DT);
    assert.ok(rod.position[1] > 0, `heightfield, throw ${i}: through`);
  }
});

test('a turned rod at 200 m/s never passes a 1 cm steel plate floating free', () => {
  // Its sweep looks at slow bodies where they end the step; before, 47 in 100 went through.
  const rnd = numbers(7);
  for (let i = 0; i < 40; i++) {
    let plate;
    const { body } = thrown((world) => { plate = world.addBody({ size: [0.01, 6, 6], density: 7800 }); }, [0.1, 0.02, 0.02], 200, 0, rnd);
    assert.ok(body.position[0] < plate.position[0], `throw ${i}: through`);
  }
});

test('a body caught at a wall comes back off it or rests against it, and its energy only falls', () => {
  const rnd = numbers(5);
  let back = 0, caught = 0;
  for (let i = 0; i < 40; i++) {
    const { world, body } = thrown((w) => w.addBody({ type: 'static', size: [0.01, 6, 6] }), [0.1, 0.02, 0.02], 200, 0, rnd);
    for (let n = 0; n < 80; n++) world.step(DT);
    caught += world._catches;
    assert.ok(body.position[0] < 0, `throw ${i}: through`);
    // Never driven in: its middle stays clear of the wall's face.
    assert.ok(body.position[0] < -0.005, `throw ${i}: its centre is ${body.position[0]} into the wall`);
    if (body.velocity[0] < -0.1) back++;
    const v = body.velocity, w = body.angularVelocity, I = body.inertia;
    assert.ok(0.5 * body.mass * (v[0] ** 2 + v[1] ** 2 + v[2] ** 2) < 0.5 * body.mass * 200 * 200, `throw ${i}: gained energy`);
    assert.ok(Number.isFinite(w[0] + w[1] + w[2] + I[0]));
  }
  assert.ok(caught > 0, 'nothing was caught');
  assert.ok(back > 20, `only ${back} of 40 came back`);
});

test('spin is capped at half a turn a step, but for a sphere', () => {
  const world = new World({ gravity: [0, 0, 0] });
  const box = world.addBody({ angularVelocity: [0, 1000, 0] });
  const ball = world.addBody({ shape: 'sphere', position: [5, 0, 0], angularVelocity: [0, 1000, 0] });
  world.step(DT);
  assert.ok(Math.abs(Math.hypot(...box.angularVelocity) - Math.PI / DT) < 1e-9, `${box.angularVelocity}`);
  assert.equal(ball.angularVelocity[1], 1000);
});

test('a rod spun at 400 rad/s and dropped on another gains no energy', () => {
  const world = new World({ sleeping: false });
  world.addBody({ type: 'static', size: [10, 1, 10], position: [0, -0.5, 0] });
  const lying = world.addBody({ size: [0.1, 0.02, 0.02], position: [0, 0.01, 0] });
  const rod = world.addBody({ size: [0.1, 0.02, 0.02], position: [0.02, 0.06, 0.01], rotation: [0, 0, 0.2, 0.98], angularVelocity: [400, 0, 0] });
  // Kinetic, both ways, and height: what the step can only ever take away.
  const energy = () => [lying, rod].reduce((e, b) => {
    const v = b.velocity, w = b.angularVelocity, I = b.inertia, [x, y, z, s] = b.rotation;
    // ω in the body's own frame, where its inertia is diagonal.
    const r = [
      (1 - 2 * (y * y + z * z)) * w[0] + 2 * (x * y + s * z) * w[1] + 2 * (x * z - s * y) * w[2],
      2 * (x * y - s * z) * w[0] + (1 - 2 * (x * x + z * z)) * w[1] + 2 * (y * z + s * x) * w[2],
      2 * (x * z + s * y) * w[0] + 2 * (y * z - s * x) * w[1] + (1 - 2 * (x * x + y * y)) * w[2],
    ];
    return e + 0.5 * b.mass * (v[0] ** 2 + v[1] ** 2 + v[2] ** 2) + 0.5 * (I[0] * r[0] ** 2 + I[1] * r[1] ** 2 + I[2] * r[2] ** 2) + b.mass * 9.81 * b.position[1];
  }, 0);
  world.step(DT);   // the cap's own cut, from 400 rad/s to π/dt, is in the first step
  const start = energy();
  for (let n = 0; n < 600; n++) {
    world.step(DT);
    assert.ok(energy() <= start * (1 + 1e-9), `step ${n}: ${energy()} J, from ${start}`);
  }
});

test('a chain of 30 thin boxes whipping as it falls stays together', () => {
  const link = 0.1, world = new World({ sleeping: false }), links = [];
  for (let i = 0; i < 30; i++) links.push(world.addBody({ size: [link, link / 5, link / 5], position: [link * (i + 0.5), 0, 0] }));
  world.addJoint({ a: links[0], anchor: [0, 0, 0] });
  for (let i = 1; i < 30; i++) world.addJoint({ a: links[i - 1], b: links[i], anchor: [link * i, 0, 0] });
  for (let n = 0; n < 20 * 60; n++) {
    world.step(DT);
    for (const b of links) assert.ok(b.position.every(Number.isFinite), `step ${n}`);
  }
  // Every link's end within a millimetre of the next one's start.
  const end = (b, x) => {
    const [qx, qy, qz, qw] = b.rotation, p = b.position;
    return [p[0] + x * (1 - 2 * (qy * qy + qz * qz)), p[1] + x * 2 * (qx * qy + qw * qz), p[2] + x * 2 * (qx * qz - qw * qy)];
  };
  for (let i = 1; i < 30; i++) {
    const gap = Math.hypot(...end(links[i - 1], link / 2).map((v, k) => v - end(links[i], -link / 2)[k]));
    assert.ok(gap < 0.001, `joint ${i}: ${gap} m apart`);
  }
});

test('off means off: a world where nothing moves half its size in a step sweeps nothing after the solve', () => {
  const world = new World();
  world.addBody({ type: 'static', size: [20, 1, 20], position: [0, -0.5, 0] });
  for (let i = 0; i < 10; i++) world.addBody({ position: [i * 1.1, 0.5 + i, 0] });
  for (let n = 0; n < 120; n++) world.step(DT);
  assert.equal(world._catchSteps, 0);
  // And the count counts: a fast rod at a wall is swept.
  const fast = new World({ gravity: [0, 0, 0] });
  fast.addBody({ type: 'static', size: [0.01, 6, 6] });
  fast.addBody({ size: [0.1, 0.02, 0.02], position: [-3, 0, 0], velocity: [200, 0, 0] });
  for (let n = 0; n < 3; n++) fast.step(DT);
  assert.ok(fast._catchSteps > 0);
});

test('off means off: nothing moving fast against anything asks for a time of impact', () => {
  const world = new World();
  world.addBody({ type: 'static', size: [20, 1, 20], position: [0, -0.5, 0] });
  for (let i = 0; i < 10; i++) world.addBody({ position: [i * 1.1, 0.5, 0] });
  for (let i = 0; i < 10; i++) world.addBody({ shape: 'sphere', position: [i * 1.1, 1.5, 0] });
  world.addBody({ shape: 'sphere', radius: 0.1, position: [0, 5, 5], velocity: [100, 0, 0] });   // fast, with nothing near
  for (let n = 0; n < 60; n++) world.step(DT);
  assert.equal(world._impacts, 0);
  // And the count counts: one fast ball at a wall asks.
  const fast = new World({ gravity: [0, 0, 0] });
  fast.addBody({ type: 'static', size: [0.01, 4, 4] });
  fast.addBody({ shape: 'sphere', radius: 0.005, position: [-3, 0, 0], velocity: [500, 0, 0] });
  for (let n = 0; n < 2; n++) fast.step(DT);
  assert.ok(fast._impacts > 0);
});

test('the catch looks for bodies out of their tree boxes where the solve left them, character or not', () => {
  // A character's casts list such bodies as the step begins, under the same
  // step and epoch the catch would look them up by; before the speed audit
  // (milestone 16) the catch took that list, from before anything moved,
  // whenever a character was in the world. Checked where the catch starts.
  for (const character of [false, true]) {
    const world = new World();
    world.addBody({ type: 'static', size: [20, 1, 20], position: [0, -0.5, 0] });
    if (character) world.addBody({ type: 'character', shape: 'capsule', radius: 0.3, height: 1.6, position: [-8, 0.8, -8] });
    world.addBody({ size: [0.1, 0.02, 0.02], position: [0, 2, 0], velocity: [200, 0, 0] });
    let stale = 0, caught = 0;
    const own = world._catch;
    world._catch = function (next) {
      if (this._straysAt === this._stepCount) stale++;
      caught++;
      return own.call(this, next);
    };
    for (let n = 0; n < 5; n++) world.step(1 / 60);
    assert.equal(caught, 5);
    assert.equal(stale, 0, `with${character ? '' : 'out'} a character, ${stale} of 5 catches read a stale list`);
  }
});

test('fast bodies give the same result twice, to the bit', () => {
  const run = () => {
    const world = new World();
    world.addBody({ type: 'static', size: [10, 1, 10], position: [0, -0.5, 0] });
    world.addBody({ type: 'static', size: [0.02, 3, 3], position: [2, 1.5, 0] });
    const bodies = [];
    for (let i = 0; i < 8; i++) {
      bodies.push(world.addBody({ size: [0.1, 0.02, 0.02], position: [-2, 0.5 + i * 0.3, i * 0.1 - 0.4], velocity: [150 + i * 10, 2, 0], angularVelocity: [i, 20 + i * 7, 3] }));
      bodies.push(world.addBody({ shape: 'sphere', radius: 0.01, position: [-3, 0.6 + i * 0.3, 0.3 - i * 0.07], velocity: [300, -5, 1] }));
    }
    for (let n = 0; n < 120; n++) world.step(DT);
    return Array.from(world.transforms);
  };
  assert.deepEqual(run(), run());
});
