// How bodies turn and how they are pushed: inertia, tumbling, forces and
// impulses.
// Run: node test/motion.test.js

import assert from 'node:assert/strict';
import { World } from '../src/bearing.js';
import { rotate } from '../src/core/math.js';
import { suite, test } from './test.js';

suite('motion');

const near = (a, b, relative, what) => {
  for (let i = 0; i < a.length; i++) {
    assert.ok(Math.abs(a[i] - b[i]) <= relative * Math.abs(b[i]), `${what}[${i}]: ${a[i]} vs ${b[i]}`);
  }
};

/** World-space angular momentum R I Rᵀ ω, and kinetic energy ½ ωᵀ I ω, of a body. */
function momentum(body) {
  const I = body.inertia, q = body.rotation, w = body.angularVelocity;
  const b = rotate(q, 0, w[0], w[1], w[2], true, [0, 0, 0]);
  const L = rotate(q, 0, b[0] * I[0], b[1] * I[1], b[2] * I[2], false, [0, 0, 0]);
  return { L, E: 0.5 * (I[0] * b[0] * b[0] + I[1] * b[1] * b[1] + I[2] * b[2] * b[2]) };
}
/** Where the body's own axis k (0 x, 1 y, 2 z) points in the world. */
function axis(body, k) {
  return rotate(body.rotation, 0, k === 0 ? 1 : 0, k === 1 ? 1 : 0, k === 2 ? 1 : 0, false, [0, 0, 0]);
}

// ---- inertia ----

test('a box and a sphere have their textbook moments', () => {
  const world = new World();
  const box = world.addBody({ size: [1, 2, 3], mass: 12 });
  assert.deepEqual(box.inertia, [12 * (4 + 9) / 12, 12 * (1 + 9) / 12, 12 * (1 + 4) / 12]);
  const ball = world.addBody({ shape: 'sphere', radius: 2, mass: 5 });
  assert.deepEqual(ball.inertia, [0.4 * 5 * 4, 0.4 * 5 * 4, 0.4 * 5 * 4]);
});

test('a capsule\'s moments match slicing it into discs and adding them up', () => {
  // Independent of the closed form in src/shapes.js: a disc of radius ρ and
  // mass dm at height y gives dm ρ²/2 about y and dm (ρ²/4 + y²) about x.
  // Each piece's integrand is a polynomial in y, so Simpson's rule on enough
  // intervals is exact to rounding.
  for (const [r, H] of [[0.5, 2], [0.3, 1.8], [1, 2], [0.1, 5]]) {
    const L = H - 2 * r, mass = 7;
    const volume = Math.PI * r * r * L + (4 / 3) * Math.PI * r ** 3;
    const density = mass / volume;
    const rho2 = (y) => (Math.abs(y) <= L / 2 ? r * r : r * r - (Math.abs(y) - L / 2) ** 2);
    const simpson = (f, a, b, n = 2000) => {
      const h = (b - a) / n;
      let s = f(a) + f(b);
      for (let i = 1; i < n; i++) s += f(a + i * h) * (i % 2 ? 4 : 2);
      return s * h / 3;
    };
    const pieces = (f) => simpson(f, -H / 2, -L / 2) + simpson(f, -L / 2, L / 2) + simpson(f, L / 2, H / 2);
    const iy = pieces((y) => density * Math.PI * rho2(y) * rho2(y) / 2);
    const ix = pieces((y) => density * Math.PI * rho2(y) * (rho2(y) / 4 + y * y));
    const world = new World();
    const capsule = world.addBody({ shape: 'capsule', radius: r, height: H, mass });
    near(capsule.inertia, [ix, iy, ix], 1e-12, `capsule r ${r} h ${H}`);
  }
});

test('inertia follows the shape and mass, unless given; null follows the shape again', () => {
  const world = new World();
  const body = world.addBody({ size: [2, 2, 2] });
  const solid = body.inertia;
  world.setBody(body, { mass: 1 });
  assert.deepEqual(body.inertia, [(4 + 4) / 12, (4 + 4) / 12, (4 + 4) / 12]);
  world.setBody(body, { inertia: [1, 2, 3] });
  world.setBody(body, { size: [1, 1, 1], mass: 50 });
  assert.deepEqual(body.inertia, [1, 2, 3]);
  assert.deepEqual(world.bodyOf(body).inertia, [1, 2, 3]);
  world.setBody(body, { inertia: null, size: [2, 2, 2], density: 1000 });
  assert.deepEqual(body.inertia, solid);
  assert.equal(world.bodyOf(body).inertia, undefined);
  const twin = world.addBody({ ...world.bodyOf(world.addBody({ inertia: [4, 5, 6] })) });
  assert.deepEqual(twin.inertia, [4, 5, 6]);
});

// ---- forces and impulses ----

test('a force acts for one step, exactly F/m, and is then cleared', () => {
  const world = new World({ gravity: [0, 0, 0] });
  const body = world.addBody({ mass: 3 });
  const dt = 1 / 60;
  assert.equal(body.applyForce([7, -2, 0.5]).applyForce([1, 0, 0]), body);
  world.step(dt);
  assert.deepEqual(body.velocity, [8 * dt / 3, -2 * dt / 3, 0.5 * dt / 3]);
  const after = body.velocity;
  world.step(dt);
  assert.deepEqual(body.velocity, after);
});

test('a force off the centre turns the body by r × F, and a torque turns it directly', () => {
  const world = new World({ gravity: [0, 0, 0] });
  const dt = 0.1;
  const a = world.addBody({ size: [1, 2, 3], position: [1, 1, 1] });
  a.applyForce([0, 0, 10], [1.5, 1, 1]);   // r = (0.5, 0, 0): torque (0, -5, 0)
  const b = world.addBody({ size: [1, 2, 3], position: [10, 0, 0] });
  b.applyTorque([0, -5, 0]);
  world.step(dt);
  const inv = 1 / a.inertia[1];
  assert.deepEqual(a.angularVelocity, [0, -5 * dt * inv, 0]);
  assert.deepEqual(b.angularVelocity, a.angularVelocity);
});

test('an impulse changes velocity at once: Δv = J/m, Δω = I⁻¹ (r × J)', () => {
  const world = new World();
  const body = world.addBody({ size: [1, 2, 3], mass: 4, position: [0, 5, 0] });
  body.applyImpulse([0, 0, 2], [0, 6, 0]);   // r = (0, 1, 0): r × J = (2, 0, 0)
  assert.deepEqual(body.velocity, [0, 0, 2 / 4]);
  assert.deepEqual(body.angularVelocity, [2 * (1 / body.inertia[0]), 0, 0]);
  body.applyAngularImpulse([0, 3, 0]);
  assert.deepEqual(body.angularVelocity, [2 * (1 / body.inertia[0]), 3 * (1 / body.inertia[1]), 0]);
});

test('turned bodies take forces in world space', () => {
  const world = new World({ gravity: [0, 0, 0] });
  const body = world.addBody({ size: [1, 2, 3] }).setAxisAngle([0, 0, 1], Math.PI / 2);
  // A torque about world x is about the body's own y once it is turned a
  // quarter about z, so it meets the body's y moment.
  body.applyAngularImpulse([1, 0, 0]);
  near(body.angularVelocity, [1 / body.inertia[1], 0, 0], 1e-15, 'turned');
});

test('static and kinematic bodies ignore forces and impulses', () => {
  const world = new World();
  for (const type of ['static', 'kinematic']) {
    const body = world.addBody({ type });
    body.applyForce([100, 0, 0], [0, 1, 0]).applyTorque([0, 100, 0])
      .applyImpulse([100, 0, 0], [0, 1, 0]).applyAngularImpulse([0, 100, 0]);
    world.step(1 / 60);
    assert.deepEqual(body.velocity, [0, 0, 0]);
    assert.deepEqual(body.angularVelocity, [0, 0, 0]);
  }
});

test('a removed body\'s waiting force never reaches the body that gets its slot', () => {
  const world = new World({ gravity: [0, 0, 0] });
  const a = world.addBody({});
  a.applyForce([1000, 0, 0]);
  world.remove(a);
  const b = world.addBody({});
  assert.equal(b.slot, a.slot);
  world.step(1);
  assert.deepEqual(b.velocity, [0, 0, 0]);
});

test('off means off: with no force applied, the force pass never runs', () => {
  const world = new World();
  for (let i = 0; i < 100; i++) world.addBody({ position: [2 * i, 0, 0] });
  let runs = 0;
  const pass = world._applyForces;
  world._applyForces = function (dt) { runs++; return pass.call(this, dt); };
  for (let i = 0; i < 10; i++) world.step(1 / 60);
  assert.equal(runs, 0);
  world.body(world.addBody({ position: [-5, 0, 0] }).id).applyForce([1, 0, 0]);
  world.step(1 / 60);
  world.step(1 / 60);
  assert.equal(runs, 1);
});

// ---- tumbling ----

test('a spinning sphere or cube keeps its angular velocity to the bit', () => {
  const world = new World({ gravity: [0, 0, 0] });
  const cube = world.addBody({ angularVelocity: [0.3, 2, -1] });
  const ball = world.addBody({ shape: 'sphere', position: [5, 0, 0], angularVelocity: [0.3, 2, -1] });
  for (let i = 0; i < 600; i++) world.step(1 / 60);
  assert.deepEqual(cube.angularVelocity, [0.3, 2, -1]);
  assert.deepEqual(ball.angularVelocity, [0.3, 2, -1]);
});

test('a lopsided body tumbling freely keeps its angular momentum, and never gains energy', () => {
  // 120 s at 60 Hz, a 1 x 2 x 3 m box spun about its middle axis, where it
  // flips over and over. Momentum is carried exactly, so it holds to rounding:
  // a few ulps a step over 7,200 steps is about 1e-12, and 1e-11 leaves room.
  // Energy may only fall; how far is what the method costs, measured when it
  // was chosen (see tumble() in src/world.js) and held here so it can't grow.
  const cases = [[5, 0.99999], [20, 0.9998], [50, 0.993]];
  for (const [spin, kept] of cases) {
    const world = new World({ gravity: [0, 0, 0] });
    const body = world.addBody({ size: [1, 2, 3], angularVelocity: [spin / 100, spin, spin / 100] });
    const start = momentum(body);
    let lowest = 1;
    for (let i = 0; i < 60 * 120; i++) {
      world.step(1 / 60);
      const now = momentum(body);
      const drift = Math.hypot(now.L[0] - start.L[0], now.L[1] - start.L[1], now.L[2] - start.L[2]);
      assert.ok(drift <= 1e-11 * Math.hypot(...start.L), `${spin} rad/s, step ${i}: momentum drifted ${drift}`);
      assert.ok(now.E <= start.E * (1 + 1e-12), `${spin} rad/s, step ${i}: energy grew`);
      lowest = Math.min(lowest, now.E / start.E);
    }
    assert.ok(lowest >= kept, `${spin} rad/s kept ${lowest} of its energy`);
  }
});

test('spun mostly about its longest or shortest axis, a lopsided body keeps its momentum and never gains energy either', () => {
  // Away from the middle axis the turn's error doesn't flip from step to step
  // but drifts: before it was checked, these gained 2e-4 to 4e-4 in 120 s.
  for (const [inertia, w] of [[[0.00768, 0.00205, 0.00928], [3, 1, 20]], [[5, 8.33, 13.33], [3, 1, 20]], [[5, 8.33, 13.33], [20, 1, 3]]]) {
    const world = new World({ gravity: [0, 0, 0] });
    const body = world.addBody({ mass: 1, inertia, angularVelocity: w });
    const start = momentum(body);
    for (let i = 0; i < 60 * 120; i++) {
      world.step(1 / 60);
      const now = momentum(body);
      const drift = Math.hypot(now.L[0] - start.L[0], now.L[1] - start.L[1], now.L[2] - start.L[2]);
      assert.ok(drift <= 1e-11 * Math.hypot(...start.L), `${inertia}, step ${i}: momentum drifted ${drift}`);
      assert.ok(now.E <= start.E * (1 + 1e-12), `${inertia}, step ${i}: energy grew ${now.E / start.E - 1}`);
    }
  }
});

test('spun about its middle axis a box flips; about its longest or shortest it stays', () => {
  // The intermediate axis theorem. Moments of a 1 x 2 x 3 box: largest about
  // x, middle about y, smallest about z.
  for (const [k, expectFlip] of [[1, true], [0, false], [2, false]]) {
    const world = new World({ gravity: [0, 0, 0] });
    const w = [0.05, 0.05, 0.05];
    w[k] = 5;
    const body = world.addBody({ size: [1, 2, 3], angularVelocity: w });
    let lowest = 1;
    for (let i = 0; i < 60 * 20; i++) {
      world.step(1 / 60);
      lowest = Math.min(lowest, axis(body, k)[k]);
    }
    if (expectFlip) assert.ok(lowest < -0.99, `axis ${k} should flip, lowest ${lowest}`);
    else assert.ok(lowest > 0.99, `axis ${k} should stay, lowest ${lowest}`);
  }
});

test('forces, impulses and tumbling give the same result twice, to the bit', () => {
  const run = () => {
    const world = new World();
    const bodies = [];
    for (let i = 0; i < 30; i++) {
      bodies.push(world.addBody({
        shape: ['box', 'sphere', 'capsule'][i % 3],
        size: i % 3 === 0 ? [1 + i / 10, 1, 2] : undefined,
        position: [i, 0, 0], angularVelocity: [i / 3, 1, -i / 7],
      }));
    }
    for (let s = 0; s < 300; s++) {
      for (const [i, b] of bodies.entries()) {
        b.applyForce([i, 100, -i], [i + 0.3, 0.2, 0.1]);
        if (s % 10 === 0) b.applyImpulse([0, 1, 0], [i, 0.5, 0]).applyAngularImpulse([0.1, 0, 0]);
      }
      world.step(1 / 60);
    }
    return bodies.flatMap((b) => [...b.position, ...b.rotation, ...b.velocity, ...b.angularVelocity]);
  };
  const a = run(), b = run();
  assert.ok(a.every((x, i) => Object.is(x, b[i])));
});
