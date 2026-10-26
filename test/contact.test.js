// Bodies that touch: resting, stacking, bouncing, sliding and being carried.
// Every scene runs on the defaults unless it says otherwise; that is rule 35.
// Run: node test/contact.test.js

import assert from 'node:assert/strict';
import { World } from '../src/bearing.js';
import { suite, test } from './test.js';

suite('contact');

const ground = (world, extra = {}) =>
  world.addBody({ type: 'static', size: [100, 1, 100], position: [0, -0.5, 0], ...extra });
const steps = (world, seconds) => { for (let i = 0; i < Math.round(seconds * 60); i++) world.step(1 / 60); };
const speed = (body) => Math.hypot(...body.velocity);
const sideways = (body, from) => Math.hypot(body.position[0] - from[0], body.position[2] - from[2]);

test('a box dropped flat on the floor comes to rest where it lands, without sliding', () => {
  const world = new World();
  ground(world);
  const box = world.addBody({ position: [0, 3, 0] });
  steps(world, 4);
  assert.ok(speed(box) < 1e-9, `still moving at ${speed(box)}`);
  // Sinks no more than its tolerance (5 mm for 1 m), and never slides: the
  // four corners of a face are solved together (one sweep each), so the first
  // corner the solver visits can't tip it.
  assert.ok(box.position[1] <= 0.5 && box.position[1] > 0.5 - 0.005, `rests at ${box.position[1]}`);
  assert.ok(sideways(box, [0, 0, 0]) < 1e-6, `slid ${sideways(box, [0, 0, 0])} m`);
});

test('a stack of 10 boxes stands for 10 seconds', () => {
  // Measured with the default 8 substeps: no box strays 0.06 mm, the top sinks
  // 6.9 mm in all (about 0.7 mm a contact), and all is still by the end. (The
  // top strayed 0.9 to 1.1 mm while rounding changed the contact points' ids
  // from step to step: see collide.js clipPolygon.)
  const world = new World();
  ground(world);
  const boxes = [];
  for (let k = 0; k < 10; k++) boxes.push(world.addBody({ position: [0, 0.5 + k, 0] }));
  steps(world, 10);
  for (const [k, b] of boxes.entries()) assert.ok(sideways(b, [0, 0, 0]) < 0.001, `box ${k} strayed`);
  assert.ok(9.5 - boxes[9].position[1] < 0.01, `the top sank ${9.5 - boxes[9].position[1]}`);
  for (const b of boxes) assert.ok(speed(b) < 1e-3);
});

test('a pyramid of 210 boxes stands for 10 seconds', () => {
  // 20 on the bottom row. Measured: no box strays 0.07 mm (1.5 mm while
  // rounding changed contact ids, which the default substeps were chosen to
  // keep under half a box's tolerance, 2.5 mm); the top sinks 7.4 mm over 20
  // rows; nothing still moves 1 mm/s.
  const world = new World();
  ground(world);
  const boxes = [], start = [];
  for (let row = 0; row < 20; row++) {
    for (let k = 0; k < 20 - row; k++) {
      const at = [k - (19 - row) / 2, 0.5 + row, 0];
      boxes.push(world.addBody({ position: at }));
      start.push(at);
    }
  }
  steps(world, 10);
  for (const [k, b] of boxes.entries()) assert.ok(sideways(b, start[k]) < 0.0025, `box ${k} strayed ${sideways(b, start[k])}`);
  assert.ok(19.5 - boxes.at(-1).position[1] < 0.01, 'the top sank');
  for (const b of boxes) assert.ok(speed(b) < 1e-3);
});

test('a ball given a bounce bounces lower each time, then rests', () => {
  // Each bounce rises restitution² of the last: 0.49 of it for 0.7. Measured
  // to within 3% for the first three; the last few small hops, near the
  // cut-off, lose a little more. Then it settles, which is rule 35's point.
  const world = new World();
  ground(world);
  const ball = world.addBody({ shape: 'sphere', position: [0, 2, 0], restitution: 0.7 });
  const peaks = [];
  let last = ball.position[1], rising = false;
  for (let i = 0; i < 20 * 60; i++) {
    world.step(1 / 60);
    const y = ball.position[1];
    if (y < last && rising) peaks.push(last);
    rising = y > last;
    last = y;
  }
  const rises = [1.5, ...peaks.map((p) => p - 0.5)];
  assert.ok(rises.length >= 5, `only ${rises.length - 1} bounces`);
  for (let k = 1; k <= 3; k++) {
    const ratio = rises[k] / rises[k - 1];
    assert.ok(Math.abs(ratio / 0.49 - 1) < 0.03, `bounce ${k} kept ${ratio} of its height`);
  }
  for (let k = 1; k < rises.length; k++) assert.ok(rises[k] < rises[k - 1], 'each bounce lower');
  assert.equal(speed(ball), 0);
});

test('balls meeting with restitution 1 keep their momentum exactly and their energy', () => {
  for (const density of [1000, 3000]) {
    const world = new World({ gravity: [0, 0, 0] });
    const a = world.addBody({ shape: 'sphere', position: [-2, 0, 0], velocity: [3, 0, 0], restitution: 1, density });
    const b = world.addBody({ shape: 'sphere', position: [2, 0, 0], velocity: [-3, 0, 0], restitution: 1 });
    const momentum = () => a.mass * a.velocity[0] + b.mass * b.velocity[0];
    const energy = () => 0.5 * a.mass * a.velocity[0] ** 2 + 0.5 * b.mass * b.velocity[0] ** 2;
    const p0 = momentum(), e0 = energy();
    steps(world, 2);
    // Every impulse is equal and opposite, so momentum holds to rounding.
    assert.ok(Math.abs(momentum() - p0) <= 1e-12 * a.mass * 3, `momentum moved ${momentum() - p0}`);
    assert.ok(Math.abs(energy() / e0 - 1) < 1e-9, `energy ${energy() / e0}`);
    // And the speeds are the textbook ones: (m1 - m2) / (m1 + m2) and so on.
    const r = a.mass / b.mass, va = (3 * (r - 1) - 6) / (r + 1), vb = (6 * r + 3 * (r - 1)) / (r + 1);
    assert.ok(Math.abs(a.velocity[0] - va) < 1e-9 && Math.abs(b.velocity[0] - vb) < 1e-9, `${a.velocity[0]} ${b.velocity[0]}`);
  }
});

test('a box on a slope holds while tan θ < friction, and slides at g (sin θ - μ cos θ) when steeper', () => {
  for (const degrees of [20, 35]) {
    const theta = degrees * Math.PI / 180;
    const tilt = [0, 0, Math.sin(theta / 2), Math.cos(theta / 2)];
    const world = new World();
    world.addBody({ type: 'static', size: [40, 1, 10], rotation: tilt });
    const box = world.addBody({ position: [-Math.sin(theta), Math.cos(theta), 0], rotation: tilt });
    steps(world, 1);
    const along = (v) => v[0] * Math.cos(theta) + v[1] * Math.sin(theta);
    const v1 = along(box.velocity), p1 = box.position;
    steps(world, 1);
    const accel = along(box.velocity) - v1;
    if (Math.tan(theta) < 0.5) {
      assert.ok(Math.hypot(box.position[0] - p1[0], box.position[1] - p1[1]) < 1e-6, `slid at ${degrees}°`);
    } else {
      const expected = -9.81 * (Math.sin(theta) - 0.5 * Math.cos(theta));
      assert.ok(Math.abs(accel / expected - 1) < 1e-3, `${degrees}°: ${accel} vs ${expected}`);
    }
  }
});

test('a kinematic platform carries a box, and a kinematic lift raises one', () => {
  const world = new World();
  ground(world);
  const platform = world.addBody({ type: 'kinematic', size: [4, 0.5, 4], position: [0, 2, 0], velocity: [1, 0, 0] });
  const box = world.addBody({ position: [0, 2.75, 0] });
  steps(world, 3);
  // The platform starts at 1 m/s at once, so the box slips until friction,
  // μ g = 4.9 m/s², brings it up to speed: it falls behind by v² / 2μg, 10.2 cm,
  // and from then on rides along, still on top, at the platform's speed.
  const lag = 1 / (2 * 0.5 * 9.81);
  assert.ok(Math.abs(platform.position[0] - box.position[0] - lag) < 0.005, `lagged ${platform.position[0] - box.position[0]}`);
  assert.ok(Math.abs(box.velocity[0] - 1) < 1e-3);
  assert.ok(Math.abs(box.position[1] - 2.75) < 0.005);
  assert.equal(platform.velocity[0], 1);   // nothing pushes back on a kinematic body

  const lift = world.addBody({ type: 'kinematic', size: [2, 0.5, 2], position: [10, 0.25, 0], velocity: [0, 1, 0] });
  const crate = world.addBody({ position: [10, 1, 0] });
  steps(world, 2);
  assert.ok(Math.abs(crate.position[1] - (lift.position[1] + 0.75)) < 0.005, 'riding the lift');
});

test('a 1 cm marble and a 50 m building both settle, with nothing retuned', () => {
  const world = new World();
  ground(world);
  const marble = world.addBody({ shape: 'sphere', radius: 0.005, position: [0, 0.015, 0] });
  const building = world.addBody({ size: [20, 50, 20], position: [40, 25.5, 0] });
  steps(world, 5);
  assert.equal(speed(marble), 0);
  assert.ok(speed(building) < 1e-9);
  // A soft contact sags by g / ω² under its own weight whatever its size:
  // 0.07 mm at the defaults. That is 0.7% of the marble, and nothing of the
  // building (its tolerance is 10 cm). Both rest on the ground, not in it.
  assert.ok(0.005 - marble.position[1] < 0.0001, `marble sank ${0.005 - marble.position[1]}`);
  assert.ok(25 - building.position[1] < 0.001, `building sank ${25 - building.position[1]}`);
});

test('static and kinematic bodies pass through each other; nothing pushes them', () => {
  const world = new World();
  const wall = world.addBody({ type: 'static', size: [1, 4, 4] });
  const ram = world.addBody({ type: 'kinematic', position: [-3, 0, 0], velocity: [2, 0, 0] });
  steps(world, 3);
  assert.ok(Math.abs(ram.position[0] - 3) < 1e-9);
  assert.deepEqual(wall.position, [0, 0, 0]);
});

test('off means off: where nothing touches, the contact solver never runs', () => {
  const world = new World();
  for (let i = 0; i < 50; i++) world.addBody({ position: [3 * i, 10, 0] });
  steps(world, 1);
  assert.equal(world._solverSteps, 0);
  ground(world);
  steps(world, 2);
  assert.ok(world._solverSteps > 0);
});

test('no two manifolds of one colour share a moving body, and a slab carrying 49 boxes overflows and still holds them', () => {
  // The slab touches the floor and 49 boxes: 50 manifolds on one dynamic
  // body, more than the 32 colours (graph.js, MASKS), so some are solved in
  // the overflow, and every box still rests on it.
  const world = new World();
  ground(world);
  const slab = world.addBody({ size: [16, 0.5, 16], position: [0, 0.25, 0] });
  const boxes = [];
  for (let i = 0; i < 49; i++) boxes.push(world.addBody({ position: [(i % 7 - 3) * 2, 1, (Math.floor(i / 7) - 3) * 2] }));
  let overflowed = 0;
  for (let n = 0; n < 120; n++) {
    world.step(1 / 60);
    const c = world._contacts, type = world._type;
    for (let k = 0; k < 32; k++) {
      const seen = new Set();
      for (let m = c.starts[k]; m < c.starts[k + 1]; m++) {
        for (const s of [c.a[m], c.b[m]]) {
          if (type[s] !== 0) continue;
          assert.ok(!seen.has(s), `step ${n}: colour ${k} has body ${s} twice`);
          seen.add(s);
        }
      }
    }
    overflowed = Math.max(overflowed, c.starts[33] - c.starts[32]);
  }
  assert.ok(overflowed >= 50 - 32, `${overflowed}`);
  assert.ok(Math.abs(slab.position[1] - 0.25) < 0.005, `${slab.position[1]}`);
  for (const b of boxes) assert.ok(Math.abs(b.position[1] - 1) < 0.01, `${b.position[1]}`);
  steps(world, 2);
  assert.ok(slab.asleep && boxes.every((b) => b.asleep));
});

test('taking a box out of a stack lets the ones above fall', () => {
  const world = new World();
  ground(world);
  const boxes = [];
  for (let k = 0; k < 4; k++) boxes.push(world.addBody({ position: [0, 0.5 + k, 0] }));
  steps(world, 1);
  world.remove(boxes[1]);
  steps(world, 2);
  // The two above it fall a metre onto the bottom one and rest there.
  assert.ok(Math.abs(boxes[2].position[1] - 1.5) < 0.005, `${boxes[2].position[1]}`);
  assert.ok(Math.abs(boxes[3].position[1] - 2.5) < 0.01);
});

test('gravity scale and damping', () => {
  const world = new World();
  const float = world.addBody({ gravityScale: 0, position: [0, 5, 0] });
  const rise = world.addBody({ gravityScale: -1, position: [10, 5, 0] });
  const drag = world.addBody({ gravityScale: 0, damping: 2, velocity: [4, 0, 0], position: [20, 5, 0] });
  steps(world, 1);
  assert.deepEqual(float.position, [0, 5, 0]);
  assert.ok(rise.velocity[1] > 9.8);
  // Damped implicitly each substep: v (1 / (1 + c h))^N, here 60 steps of 8.
  const h = 1 / 480;
  assert.ok(Math.abs(drag.velocity[0] - 4 * (1 / (1 + 2 * h)) ** 480) < 1e-12);
});

test('a small pile gives the same result twice, to the bit', () => {
  const run = () => {
    const world = new World();
    ground(world);
    const boxes = [];
    for (let k = 0; k < 20; k++) {
      boxes.push(world.addBody({
        shape: ['box', 'sphere', 'capsule'][k % 3], position: [(k % 4) * 0.6, 1 + k * 0.7, (k % 3) * 0.4],
        rotation: [0, Math.sin(k), 0, Math.cos(k)],
      }));
    }
    steps(world, 3);
    return boxes.flatMap((b) => [...b.position, ...b.rotation]);
  };
  const a = run(), b = run();
  assert.ok(a.every((x, i) => Object.is(x, b[i])));
});

// ---- rolling and spinning: each contact a patch a tolerance across ----

const G = 9.81;
// For a ball or a capsule of radius r the patch is 2r / 200, so the rolling
// resistance coefficient, patch over radius, is 0.01 at any size.
const ROLL = 0.01;
const near = (got, want, part, what) =>
  assert.ok(Math.abs(got - want) <= Math.abs(want) * part, `${what}: ${got}, should be ${want}`);

test('a ball rolled across the floor slows and stops where rolling resistance says, then sleeps', () => {
  for (const r of [0.01, 0.4, 5]) {
    const world = new World();
    ground(world, { size: [400, 1, 400] });
    const ball = world.addBody({ shape: 'sphere', radius: r, position: [0, r, 0], velocity: [1, 0, 0], angularVelocity: [0, 0, -1 / r] });
    // Rolling without slipping, a solid ball slows at ROLL g / (1 + 2/5).
    const a = ROLL * G / 1.4;
    steps(world, 20);
    near(ball.position[0], 1 / (2 * a), 0.02, `radius ${r} m rolled`);
    assert.ok(ball.asleep, `radius ${r} m: still awake`);
  }
});

test('a ball rolls as far on a floor 1 cm thick as on one 1 m thick: the patch is the ball, not the floor', () => {
  const rolled = (thickness) => {
    const world = new World();
    world.addBody({ type: 'static', size: [400, thickness, 400], position: [0, -thickness / 2, 0] });
    const ball = world.addBody({ shape: 'sphere', radius: 0.4, position: [0, 0.4, 0], velocity: [1, 0, 0], angularVelocity: [0, 0, -2.5] });
    steps(world, 20);
    return ball.position[0];
  };
  near(rolled(0.01), rolled(1), 0.001, 'on the thin floor');
});

test('a ball spinning in place on its one point slows to a stop', () => {
  const world = new World();
  ground(world);
  const r = 0.4, ball = world.addBody({ shape: 'sphere', radius: r, position: [0, r, 0], angularVelocity: [0, 0.8, 0] });
  // Friction over the patch, 2/3 mu N times its radius, against I = 2/5 m r².
  const alpha = (2 / 3) * 0.5 * G * (2 * r / 200) / (0.4 * r * r);
  let stopped = null;
  for (let i = 1; i <= 600 && stopped === null; i++) {
    world.step(1 / 60);
    if (ball.angularVelocity[1] <= 0) stopped = i / 60;
  }
  near(stopped, 0.8 / alpha, 0.02, 'stopped at');
});

test('a capsule rolled on its side slows and stops as a ball does, with its own inertia', () => {
  const world = new World();
  ground(world, { size: [400, 1, 400] });
  const r = 0.25, turn = [0, 0, Math.SQRT1_2, Math.SQRT1_2];   // lying along x
  const capsule = world.addBody({ shape: 'capsule', radius: r, height: 1, position: [0, r, 0], rotation: turn,
    velocity: [0, 0, 1], angularVelocity: [1 / r, 0, 0] });
  const I = capsule.inertia[1], m = capsule.mass;   // about its own axis, its y
  const a = ROLL * G / (1 + I / (m * r * r));
  steps(world, 25);
  near(capsule.position[2], 1 / (2 * a), 0.02, 'rolled');
});

test('a ball rests on a slope gentler than its rolling resistance, and rolls down a steeper one at g (sin θ − 0.01 cos θ) / 1.4', () => {
  const deg = Math.PI / 180;
  const slope = (angle) => {
    const world = new World({ sleeping: false });
    const t = [0, 0, Math.sin(angle / 2), Math.cos(angle / 2)];
    world.addBody({ type: 'static', size: [60, 1, 4], position: [0, -0.5, 0], rotation: t });
    const r = 0.4, ball = world.addBody({ shape: 'sphere', radius: r, position: [-Math.sin(angle) * r, Math.cos(angle) * r, 0] });
    return { world, ball, along: (p) => p[0] * Math.cos(angle) + p[1] * Math.sin(angle) };
  };
  {
    const { world, ball } = slope(0.4 * deg);
    const at = ball.position;
    steps(world, 5);
    assert.ok(Math.hypot(ball.position[0] - at[0], ball.position[1] - at[1]) < 0.4 / 100, 'rolled on a 0.4° slope');
  }
  {
    const angle = 3 * deg, { world, ball, along } = slope(angle);
    const from = along(ball.position);
    steps(world, 3);
    const a = G * (Math.sin(angle) - ROLL * Math.cos(angle)) / 1.4;
    near(from - along(ball.position), a * 9 / 2, 0.02, 'rolled down a 3° slope');
  }
});

test('friction 0 leaves a spin in place alone, while rolling resistance still stops a roll', () => {
  const world = new World();
  ground(world, { friction: 0 });
  const r = 0.4;
  const top = world.addBody({ shape: 'sphere', radius: r, friction: 0, position: [0, r, 0], angularVelocity: [0, 2, 0] });
  const roller = world.addBody({ shape: 'sphere', radius: r, friction: 0, position: [5, r, 0], angularVelocity: [0, 0, -2] });
  steps(world, 10);
  assert.equal(top.angularVelocity[1], 2, 'the spin in place changed');
  assert.ok(Math.abs(roller.angularVelocity[2]) < 1e-6, `still turning at ${roller.angularVelocity[2]} rad/s`);
});

test('rolling and spinning only ever take energy away', () => {
  const world = new World({ sleeping: false });
  ground(world);
  const bodies = [
    world.addBody({ shape: 'sphere', radius: 0.4, position: [0, 0.4, 0], velocity: [2, 0, 1], angularVelocity: [1, 3, -5] }),
    world.addBody({ shape: 'capsule', radius: 0.25, height: 1, position: [4, 0.25, 0], rotation: [0, 0, Math.SQRT1_2, Math.SQRT1_2], velocity: [0, 0, 2], angularVelocity: [8, 1, 0] }),
  ];
  const energy = () => bodies.reduce((e, b) => {
    const v = b.velocity, w = b.angularVelocity, I = b.inertia, q = b.rotation;
    // ω in the body's own frame, where the inertia is diagonal.
    const [x, y, z, s] = q, wx = w[0], wy = w[1], wz = w[2];
    const tx = 2 * (y * wz - z * wy), ty = 2 * (z * wx - x * wz), tz = 2 * (x * wy - y * wx);
    const lx = wx - s * tx + (y * tz - z * ty), ly = wy - s * ty + (z * tx - x * tz), lz = wz - s * tz + (x * ty - y * tx);
    return e + 0.5 * b.mass * (v[0] ** 2 + v[1] ** 2 + v[2] ** 2) + 0.5 * (I[0] * lx * lx + I[1] * ly * ly + I[2] * lz * lz)
      + b.mass * G * b.position[1];
  }, 0);
  let last = energy();
  for (let i = 0; i < 600; i++) {
    world.step(1 / 60);
    const now = energy();
    assert.ok(now <= last + 1e-9 * Math.abs(last), `step ${i}: energy rose from ${last} to ${now}`);
    last = now;
  }
});

test('a cube on its edge stands within its tolerance of balance, and tips over past it', () => {
  // The edge's patch holds its weight leaning up to its tolerance, 5 mm for a
  // 1 m cube: 0.41° past balance, its centre 0.707 m above the edge.
  const deg = Math.PI / 180;
  for (const [past, stands] of [[0.3, true], [0.6, false]]) {
    const world = new World({ sleeping: false });
    ground(world);
    const a = (45 + past) * deg, r = Math.SQRT1_2;
    const cube = world.addBody({ position: [-r * Math.sin(a - 45 * deg), r * Math.cos(a - 45 * deg), 0], rotation: [0, 0, Math.sin(a / 2), Math.cos(a / 2)] });
    steps(world, 10);
    assert.equal(cube.position[1] > 0.6, stands, `${past}° past balance`);
  }
});

test('boxes and hulls stacked exactly in line, or turned 45° on each other, keep their contact points from step to step', () => {
  // Each point's id carries its impulse into the next step (warm starting).
  // Faces exactly on faces put every corner on a side, and rounding put it a
  // hair inside or out, so ids changed and impulses were lost: 59 in 600 steps
  // of this stack, 4,328 turned 45°, where rounding also picked which four of
  // the eight points to keep. A lost id here is the pair's memory after one
  // step holding an id it doesn't after the next, while it stays touching.
  const cube = []; for (const z of [-0.5, 0.5]) for (const y of [-0.5, 0.5]) for (const x of [-0.5, 0.5]) cube.push(x, y, z);
  for (const shape of [{}, { shape: 'hull', vertices: cube }]) {
    for (const turn of [0, Math.PI / 4]) {
      const world = new World({ sleeping: false });
      ground(world);
      for (let i = 0; i < 6; i++) world.addBody({ ...shape, position: [0, 0.5 + i, 0], rotation: [0, Math.sin(i % 2 ? turn / 2 : 0), 0, Math.cos(i % 2 ? turn / 2 : 0)] });
      const pairs = world._pairs, before = new Map();
      let lost = 0;
      for (let n = 0; n < 120; n++) {
        world.step(1 / 60);
        for (let p = 0; p < pairs.count; p++) {
          if (!pairs.touching[p]) continue;
          const key = `${pairs.idA[p]}:${pairs.idB[p]}`, ids = [...pairs.pid.subarray(4 * p, 4 * p + pairs.points[p])];
          for (const id of before.get(key) ?? []) if (!ids.includes(id)) lost++;
          before.set(key, ids);
        }
      }
      assert.equal(lost, 0, `${shape.shape ?? 'box'} turned ${turn}`);
    }
  }
});

test('a column of boxes leaning in a heap, each turned on the one below, comes to rest', () => {
  // Four boxes from the 1,000-box pile's seed 8 at 30 s, as they lay, held by
  // the four they leaned on, made static. The third sits tilted on the second,
  // turned against it, so their faces clip to eight points, and the four kept
  // were chosen by spread alone: two it pivoted on and two 11 to 16 mm off,
  // leaving out the two at its low end, 3 to 6 mm in. That end sank unheld
  // while the points kept pushed their own end out, then the four swung over
  // to it and pushed it out in turn: the column rocked at up to 15 cm/s for
  // good (still at 120 s), and kept the whole pile awake. With the pressed
  // points chosen first (collide.js reducePoints), it is still in 1.3 s.
  const world = new World();
  const statics = [
    [[2.0336, 0.4998, -5.6188], [0.5388, 0.5379, 0.4583, 0.4586]],
    [[3.4584, 4.5079, -3.9419], [-0.5822, -0.5669, 0.438, 0.3845]],
    [[1.8092, 5.2644, -5.7059], [0.1156, 0.2938, -0.1799, 0.9316]],
    [[3.1621, 5.4568, -5.1128], [0.5991, -0.0277, -0.7961, -0.0806]],
  ];
  const column = [
    [[2.1687, 1.5251, -5.5853], [-0.2987, 0.6439, 0.6173, 0.3393]],
    [[2.3826, 2.517, -5.3522], [-0.4844, 0.5133, 0.4759, 0.5248]],
    [[2.5064, 3.5176, -5.4385], [0.2275, 0.6753, -0.6841, 0.1558]],
    [[2.7642, 4.5116, -5.1921], [0.8373, -0.0538, -0.5401, -0.066]],
  ];
  for (const [position, rotation] of statics) world.addBody({ type: 'static', position, rotation });
  const boxes = column.map(([position, rotation]) => world.addBody({ position, rotation }));
  steps(world, 3);
  for (const [k, b] of boxes.entries()) assert.ok(b.asleep, `box ${k} awake at ${speed(b)} m/s`);
});
