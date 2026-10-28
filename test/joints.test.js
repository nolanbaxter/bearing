// Joints: what each kind holds, measured against hand-worked answers, and the
// API around them.
// Run: node test/joints.test.js

import assert from 'node:assert/strict';
import { World, Joint } from '../src/bearing.js';
import { suite, test } from './test.js';

suite('joints');

const DT = 1 / 60;
const near = (got, want, eps, what) => assert.ok(Math.abs(got - want) <= eps, `${what}: ${got}, expected ${want}`);
/** v turned by the unit quaternion q. */
const turned = (q, v) => {
  const [x, y, z, w] = q;
  const tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
  return [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)];
};
/** Where a point fixed in body b, `local` from its centre, is now. */
const at = (b, local) => {
  const r = turned(b.rotation, local), p = b.position;
  return [p[0] + r[0], p[1] + r[1], p[2] + r[2]];
};
const apart = (p, q) => Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]);
/** The angle a body has turned about y, for a body turned about y only. */
const yaw = (q) => 2 * Math.atan2(q[1], q[3]);

/** A chain of n boxes, `link` m long and a fifth of that thick, hanging from the origin. */
function chain(world, n, link) {
  const links = [];
  for (let i = 0; i < n; i++) links.push(world.addBody({ size: [link / 5, link, link / 5], position: [0, -link * (i + 0.5), 0] }));
  world.addJoint({ a: links[0], anchor: [0, 0, 0] });
  for (let i = 1; i < n; i++) world.addJoint({ a: links[i - 1], b: links[i], anchor: [0, -link * i, 0] });
  return links;
}

test('a pendulum keeps the period the formula gives, and all its swing', () => {
  // A 5 cm ball on a ball joint 1 m above it, let go at 5°. A physical
  // pendulum: T = 2π √(I / m g L), I = m L² + 2/5 m r², with the
  // finite-swing series 1 + θ²/16 + 11 θ⁴/3072.
  const world = new World({ sleeping: false });
  const L = 1, r = 0.05, th = 5 * Math.PI / 180, g = 9.81;
  const bob = world.addBody({ shape: 'sphere', radius: r, position: [L * Math.sin(th), -L * Math.cos(th), 0] });
  world.addJoint({ a: bob, anchor: [0, 0, 0] });
  const T = 2 * Math.PI * Math.sqrt((L * L + 0.4 * r * r) / (g * L)) * (1 + th * th / 16 + 11 * th ** 4 / 3072);
  const crossings = [];
  let prev = bob.position[0], swing = 0;
  for (let s = 1; s <= 20 * 60; s++) {
    world.step(DT);
    const x = bob.position[0];
    if (prev > 0 && x <= 0) crossings.push((s - 1 + prev / (prev - x)) * DT);
    prev = x;
    if (s > 19 * 60) swing = Math.max(swing, Math.atan2(Math.abs(x), -bob.position[1]));
  }
  const period = (crossings[crossings.length - 1] - crossings[0]) / (crossings.length - 1);
  // Measured: within 0.001% of the formula, and 99.9% of the swing left at 20 s.
  assert.ok(Math.abs(period / T - 1) < 0.0001, `period ${period} s, the formula ${T} s`);
  assert.ok(swing / th > 0.998, `${(swing / th * 100).toFixed(2)}% of the swing left`);
});

test('a 100-link chain hangs as long as it was built, at once, and sleeps a second later', () => {
  // The roadmap allows 1%; measured, under a micrometre, from the first step.
  for (const link of [0.1, 0.01]) {
    const world = new World();
    const links = chain(world, 100, link);
    let worst = 0, slept = -1;
    for (let s = 1; s <= 180; s++) {
      world.step(DT);
      worst = Math.max(worst, Math.abs(-links[99].position[1] + link / 2 - 100 * link));
      if (slept < 0 && links.every((b) => b.asleep)) slept = s;
    }
    assert.ok(worst < 100 * link * 1e-4, `${link} m links: the chain's end strayed ${worst} m`);
    assert.ok(slept > 0 && slept <= 61, `${link} m links: asleep at step ${slept}`);
  }
});

test('a weight 100 times a link on a chain of 10 hangs without stretching it', () => {
  const world = new World();
  const links = chain(world, 10, 0.1);
  const weight = world.addBody({ shape: 'sphere', radius: 0.1, mass: 100 * links[0].mass, position: [0, -1.1, 0] });
  world.addJoint({ a: links[9], b: weight, anchor: [0, -1, 0] });
  for (let s = 0; s < 120; s++) world.step(DT);
  near(weight.position[1], -1.1, 1e-5, 'the weight hangs at');
  assert.ok(weight.asleep, 'and sleeps');
});

test('a bridge of planks fixed at both ends sags and holds every joint', () => {
  // 20 planks hinged edge to edge, hanging in a V between two points 5% closer
  // than the bridge is long; it swings into a curve.
  const world = new World(), n = 20, w = 0.25, planks = [];
  const span = n * w * 0.95, tilt = Math.acos(span / (n * w));
  for (let i = 0; i < n; i++) {
    const left = i < n / 2, along = left ? (i + 0.5) * w : (n - i - 0.5) * w;
    const angle = left ? -tilt : tilt;
    planks.push(world.addBody({
      size: [w, 0.04, 1], rotation: [0, 0, Math.sin(angle / 2), Math.cos(angle / 2)],
      position: [left ? along * Math.cos(tilt) : span - along * Math.cos(tilt), -along * Math.sin(tilt), 0],
    }));
  }
  const end = (i, s) => at(planks[i], [s * w / 2, 0, 0]);
  const ends = [end(0, -1), end(n - 1, 1)];
  world.addJoint({ type: 'hinge', a: planks[0], anchor: ends[0], axis: [0, 0, 1] });
  for (let i = 1; i < n; i++) world.addJoint({ type: 'hinge', a: planks[i - 1], b: planks[i], anchor: end(i - 1, 1), axis: [0, 0, 1] });
  world.addJoint({ type: 'hinge', a: planks[n - 1], anchor: ends[1], axis: [0, 0, 1] });
  let worst = 0;
  for (let s = 0; s < 300; s++) {
    world.step(DT);
    for (let i = 1; i < n; i++) worst = Math.max(worst, apart(end(i - 1, 1), end(i, -1)));
    worst = Math.max(worst, apart(end(0, -1), ends[0]), apart(end(n - 1, 1), ends[1]));
  }
  // Measured: 0.017 mm. A plank's tolerance is a 200th of its 4 cm thickness, 0.2 mm.
  assert.ok(worst < 0.0002, `a joint opened ${worst} m`);
});

test('a door slammed against its stops stays within them to the tolerance', () => {
  // A 20 kg door 1 m wide, hinged at its edge, 0 to 90°, swung both ways at
  // up to 50 m/s at its edge. Its tolerance: a 200th of its 5 cm, 0.25 mm.
  for (const edge of [2, 10, 50]) {
    const world = new World({ gravity: [0, 0, 0] });
    const frame = world.addBody({ type: 'static', size: [0.1, 2, 0.1], position: [-0.05, 1, 0] });
    const door = world.addBody({ size: [1, 2, 0.05], position: [0.5, 1, 0], mass: 20 });
    world.addJoint({ type: 'hinge', a: door, b: frame, anchor: [0, 1, 0], axis: [0, 1, 0], min: 0, max: Math.PI / 2 });
    let past = 0;
    for (const spin of [edge, -edge]) {
      door.setAngularVelocity(0, spin, 0);
      for (let s = 0; s < 120; s++) {
        world.step(DT);
        const a = yaw(door.rotation);
        past = Math.max(past, a - Math.PI / 2, -a);
      }
    }
    assert.ok(past * 1 < 0.00025, `at ${edge} m/s, ${past} m past a stop at the edge`);
  }
});

test('a slider keeps to its line and its limits; a rope is never longer than its length', () => {
  const world = new World({ gravity: [0, 0, 0] });
  const rail = world.addBody({ type: 'static', size: [2, 0.1, 0.1] });
  const car = world.addBody({ size: [0.2, 0.2, 0.2], position: [0, 0.2, 0] });
  world.addJoint({ type: 'slider', a: car, b: rail, anchor: [0, 0.2, 0], axis: [1, 0, 0], min: -0.5, max: 0.5 });
  car.applyImpulse([50, 0, 3], [0, 0.3, 0]);
  let most = -Infinity, off = 0;
  for (let s = 0; s < 240; s++) {
    world.step(DT);
    most = Math.max(most, car.position[0]);
    off = Math.max(off, Math.hypot(car.position[1] - 0.2, car.position[2]), Math.hypot(...car.angularVelocity));
  }
  near(most, 0.5, 0.001, 'it slides to its stop at');
  assert.ok(off < 1e-9, `off its line by ${off}`);

  const ropes = new World();
  const ball = ropes.addBody({ shape: 'sphere', radius: 0.1, mass: 1, position: [2, 0, 0] });
  ropes.addJoint({ type: 'distance', a: ball, anchorB: [0, 0, 0], min: 0 });
  ball.setVelocity(-4, 4, 1);
  let shortest = Infinity, longest = 0;
  for (let s = 0; s < 600; s++) {
    ropes.step(DT);
    const l = Math.hypot(...ball.position);
    shortest = Math.min(shortest, l);
    longest = Math.max(longest, l);
  }
  // Thrown up and in, it goes slack; falling back, the rope catches it, and
  // swinging, it is held to 2 m, within the ball's tolerance, 1 mm.
  assert.ok(shortest < 1.9, `the rope never went slack: ${shortest}`);
  assert.ok(longest < 2.001, `the rope stretched to ${longest} m`);
});

test('a motor reaches its speed within 1%, and stalls at its force against a heavier load', () => {
  const world = new World({ gravity: [0, 0, 0] });
  const wheel = world.addBody({ shape: 'sphere', radius: 0.5, mass: 10 });   // inertia 1 kg·m²
  world.addJoint({ type: 'hinge', a: wheel, anchor: [0, 0, 0], axis: [0, 0, 1], motorSpeed: 2, motorForce: 50 });
  let reached = 0;
  for (let s = 1; s <= 60; s++) {
    world.step(DT);
    if (!reached && Math.abs(wheel.angularVelocity[2] - 2) < 0.02) reached = s;
  }
  // 50 N·m takes 1 kg·m² to 2 rad/s in 0.04 s: within 1% by the third step.
  assert.ok(reached > 0 && reached <= 3, `within 1% at step ${reached}`);
  near(wheel.angularVelocity[2], 2, 1e-9, 'and turns at');

  // A 10 kg weight on the edge of a drum 0.5 m round needs 49 N·m to hold;
  // the motor has 20, so it turns backwards, pushing as hard as it may.
  const lift = new World();
  const drum = lift.addBody({ shape: 'sphere', radius: 0.5, mass: 10 });
  const motor = lift.addJoint({ type: 'hinge', a: drum, anchor: [0, 0, 0], axis: [0, 0, 1], motorSpeed: 1, motorForce: 20 });
  const weight = lift.addBody({ shape: 'sphere', radius: 0.1, mass: 10, position: [0.5, -0.5, 0] });
  lift.addJoint({ a: drum, b: weight, anchor: [0.5, 0, 0] });
  for (let s = 0; s < 30; s++) lift.step(DT);
  assert.ok(drum.angularVelocity[2] < 0, `the drum turns at ${drum.angularVelocity[2]}`);
  const j = lift._joints.slots.slotOf(motor.id);
  near(lift._joints.impulse[11 * j + 6], 20 * DT / 8, 1e-12, 'the motor pushes, N·m·s a substep,');
});

test('a motor turned off pushes no more', () => {
  // Its impulse, kept for the warm start, went on being applied at every
  // substep with nothing left to solve it back: the wheel, turning at 0.38
  // rad/s when its motor went off, was at 0.76 a second later.
  const world = new World({ gravity: [0, 0, 0], sleeping: false });
  const wheel = world.addBody({ shape: 'sphere', radius: 0.5 });
  const j = world.addJoint({ type: 'hinge', a: wheel, anchor: [0, 0, 0], axis: [0, 0, 1], motorSpeed: 2, motorForce: 20 });
  for (let s = 0; s < 60; s++) world.step(DT);
  world.setJoint(j, { motorForce: 0 });
  const was = wheel.angularVelocity[2];
  assert.ok(was > 0.1, `turning at ${was}`);
  for (let s = 0; s < 60; s++) world.step(DT);
  assert.equal(wheel.angularVelocity[2], was);
});

test('a hinge\'s or slider\'s friction is its motor held at 0, to the bit', () => {
  // A 10 kg rod on a level hinge, 49 N·m of gravity about it, and a 1,000 kg
  // crate on a slider down a 45° slope, 6.9 kN along it, each with half and
  // one and a half times that: one slips, slowed, the other holds.
  const run = (k, given) => {
    const world = new World({ sleeping: false });
    const rod = world.addBody({ size: [1, 0.1, 0.1], position: [0.5, 2, 0] });
    world.addJoint({ type: 'hinge', a: rod, anchor: [0, 2, 0], axis: [0, 0, 1], ...given(49.05 * k) });
    const crate = world.addBody({ position: [4, 2, 0], velocity: [1, -1, 0] });
    world.addJoint({ type: 'slider', a: crate, axis: [1, -1, 0], ...given(6936.7 * k) });
    for (let s = 0; s < 120; s++) world.step(DT);
    return [...world.transforms, ...world._velocity, ...world._angular, ...world._joints.impulse];
  };
  for (const k of [0.5, 1.5]) {
    const friction = run(k, (f) => ({ friction: f }));
    assert.deepEqual(friction, run(k, (f) => ({ motorSpeed: 0, motorForce: f })), `${k} times the load`);
    assert.notDeepEqual(friction, run(k, () => ({})), `${k} times the load: friction does something`);
  }
});

test('a ball joint\'s friction holds a pendulum above it, slows one below, and is the same turned', () => {
  // A 5 cm ball held level 1 m out on a ball joint: gravity's torque about
  // the joint is m g L, 5.14 N·m.
  const pendulum = (k, turn = 0, kick = 0) => {
    const world = new World({ sleeping: false });
    const c = Math.cos(turn), s = Math.sin(turn);
    // Turned about y: x goes to (c, 0, -s), z to (s, 0, c).
    const bob = world.addBody({ shape: 'sphere', radius: 0.05, position: [c, 0, -s], velocity: [kick * s, 0, kick * c] });
    world.addJoint({ a: bob, anchor: [0, 0, 0], friction: k * bob.mass * 9.81 });
    return { world, bob, back: (p) => [c * p[0] - s * p[2], p[1], s * p[0] + c * p[2]] };
  };
  // Above: held where it was let go. It dips 0.04 mm in the first step, the
  // joint's point taking up the load, and comes back.
  const held = pendulum(1.2);
  let strayed = 0;
  for (let s = 0; s < 300; s++) {
    held.world.step(DT);
    strayed = Math.max(strayed, apart(held.bob.position, [1, 0, 0]));
  }
  assert.ok(strayed < 1e-4, `strayed ${strayed} m`);
  assert.ok(Math.hypot(...held.bob.velocity) < 1e-9, `moving at ${held.bob.velocity}`);
  // Below: it falls, slowed, and stops short of the bottom, where gravity's
  // torque, m g L sin θ, is no more than the friction. Measured: still by
  // 5 s, 25° out, where it is 0.43 of m g L.
  const slips = pendulum(0.8);
  for (let s = 0; s < 300; s++) slips.world.step(DT);
  const p = slips.bob.position, out = Math.atan2(p[0], -p[1]);
  assert.ok(p[1] < -0.5, `fell to ${p[1]}`);
  assert.ok(Math.hypot(...slips.bob.velocity) < 1e-6, `still moving at ${slips.bob.velocity}`);
  assert.ok(out > 0.1 && Math.sin(out) <= 0.8, `came to rest ${out} rad out`);
  // Swung sideways as it falls, so it turns about no one world axis, and
  // built turned about y: the same motion, turned. Measured 2e-9 m apart over
  // 4 s; with friction clamped axis by axis, as a box, 0.24 to 0.38 m.
  const ref = pendulum(0.5, 0, 1.5), path = [];
  for (let s = 0; s < 240; s++) {
    ref.world.step(DT);
    path.push(ref.bob.position);
  }
  for (const turn of [0.7, 2.1]) {
    const other = pendulum(0.5, turn, 1.5);
    let worst = 0;
    for (let s = 0; s < 240; s++) {
      other.world.step(DT);
      worst = Math.max(worst, apart(other.back(other.bob.position), path[s]));
    }
    assert.ok(worst < 1e-6, `turned ${turn} rad: ${worst} m apart`);
  }
});

test('ragdolls with friction in their joints stop swinging and sleep', () => {
  // bench/compare.html's 32 ragdolls, the upper 16 dropped on the lower. With
  // no friction, 286 of their 352 parts still swung at 12 s, each holding its
  // whole island awake; with 1 N·m in every joint, all slept by 5.55 s
  // (0.5 N·m: 341 by 12 s; 0.1: 319). 1 N·m is less than gravity's torque on
  // any limb held out level (2.1 N·m on a forearm, 19 on a thigh), so limbs
  // still fall as they would; it stops them swinging once down.
  const PARTS = [[[0.32, 0.2, 0.2], [0, 1, 0]], [[0.36, 0.45, 0.22], [0, 1.325, 0]], [[0.2, 0.24, 0.22], [0, 1.67, 0]],
    [[0.28, 0.1, 0.1], [-0.32, 1.5, 0]], [[0.26, 0.08, 0.08], [-0.59, 1.5, 0]], [[0.28, 0.1, 0.1], [0.32, 1.5, 0]],
    [[0.26, 0.08, 0.08], [0.59, 1.5, 0]], [[0.14, 0.44, 0.14], [-0.09, 0.68, 0]], [[0.12, 0.44, 0.12], [-0.09, 0.24, 0]],
    [[0.14, 0.44, 0.14], [0.09, 0.68, 0]], [[0.12, 0.44, 0.12], [0.09, 0.24, 0]]];
  const JOINTS = [['ball', 1, 0, [0, 1.1, 0]], ['ball', 2, 1, [0, 1.55, 0]], ['ball', 3, 1, [-0.18, 1.5, 0]],
    ['hinge', 4, 3, [-0.46, 1.5, 0], [0, 1, 0]], ['ball', 5, 1, [0.18, 1.5, 0]], ['hinge', 6, 5, [0.46, 1.5, 0], [0, 1, 0]],
    ['ball', 7, 0, [-0.09, 0.9, 0]], ['hinge', 8, 7, [-0.09, 0.46, 0], [1, 0, 0]], ['ball', 9, 0, [0.09, 0.9, 0]],
    ['hinge', 10, 9, [0.09, 0.46, 0], [1, 0, 0]]];
  const world = new World();
  world.addBody({ type: 'static', size: [200, 1, 200], position: [0, -0.5, 0] });
  const parts = [];
  for (let i = 0; i < 32; i++) {
    const layer = i >> 4, k = i & 15, at = [((k & 3) - 1.5) * 1.6 + layer * 0.5, 0.2 + layer * 2.4, ((k >> 2) - 1.5) + layer * 0.3];
    const plus = (p) => [at[0] + p[0], at[1] + p[1], at[2] + p[2]];
    const b = PARTS.map(([size, p]) => world.addBody({ size, position: plus(p) }));
    parts.push(...b);
    for (const [type, a, bb, p, axis] of JOINTS) world.addJoint({ type, a: b[a], b: b[bb], anchor: plus(p), friction: 1, ...(axis ? { axis } : {}) });
  }
  for (let step = 0; step < 8 * 60 && !parts.every((b) => b.asleep); step++) world.step(DT);
  assert.ok(parts.every((b) => b.asleep), `${parts.filter((b) => b.asleep).length} of 352 asleep at 8 s`);
});

test('a spring bounces at its frequency; a rod holds its length', () => {
  const world = new World({ gravity: [0, 0, 0] });
  const mass = world.addBody({ shape: 'sphere', radius: 0.05, mass: 1, position: [1.1, 0, 0] });
  world.addJoint({ type: 'distance', a: mass, anchorB: [0, 0, 0], length: 1, frequency: 1, dampingRatio: 0 });
  let prev = 0.1;
  const crossings = [];
  for (let s = 1; s <= 600; s++) {
    world.step(DT);
    const x = mass.position[0] - 1;
    if (prev > 0 && x <= 0) crossings.push((s - 1 + prev / (prev - x)) * DT);
    prev = x;
  }
  const period = (crossings[crossings.length - 1] - crossings[0]) / (crossings.length - 1);
  near(period, 1, 0.001, 'the period');   // measured 1.0001 s

  // With no spring and no limits, a distance joint is a rod: whirled round
  // at 6 rad/s, it stays 1 m to the balls' tolerance, a 200th of 20 cm.
  // (Measured 0.08 mm: a rod turning a substep's worth lengthens by its
  // turn squared, and is pulled back the next.)
  const rods = new World({ gravity: [0, 0, 0] });
  const a = rods.addBody({ shape: 'sphere', radius: 0.1, position: [-0.5, 0, 0] });
  const b = rods.addBody({ shape: 'sphere', radius: 0.1, position: [0.5, 0, 0] });
  rods.addJoint({ type: 'distance', a, b });
  a.setVelocity(0, -3, 0);
  b.setVelocity(0, 3, 1);
  for (let s = 0; s < 120; s++) {
    rods.step(DT);
    near(apart(a.position, b.position), 1, 0.001, 'the rod is');
  }
});

test('a floating leg, hip and knee, swings for a minute and never gains energy', () => {
  // A pelvis, a thigh on a ball joint and a shin on a hinge, spun, nothing
  // touching: what was measured to go wrong. A hinge's impulse kept from an
  // axis since turned pushed along the free way and spun the shin up to
  // blowing apart; tumbling, as a free body does, gained 10% in 50 s.
  const world = new World({ gravity: [0, 0, 0], sleeping: false });
  const part = (size, position) => world.addBody({ size, position, layers: 2, mask: 1 });
  const pelvis = part([0.32, 0.2, 0.2], [0, 1, 0]);
  const thigh = part([0.14, 0.44, 0.14], [-0.09, 0.68, 0]);
  const shin = part([0.12, 0.44, 0.12], [-0.09, 0.24, 0]);
  world.addJoint({ a: thigh, b: pelvis, anchor: [-0.09, 0.9, 0] });
  world.addJoint({ type: 'hinge', a: shin, b: thigh, anchor: [-0.09, 0.46, 0], axis: [1, 0, 0] });
  pelvis.setAngularVelocity(3, 1, 2);
  const energy = () => [pelvis, thigh, shin].reduce((e, b) => {
    const v = b.velocity, I = b.inertia, q = b.rotation, w = turned([-q[0], -q[1], -q[2], q[3]], b.angularVelocity);
    return e + 0.5 * b.mass * (v[0] ** 2 + v[1] ** 2 + v[2] ** 2) + 0.5 * (I[0] * w[0] ** 2 + I[1] * w[1] ** 2 + I[2] * w[2] ** 2);
  }, 0);
  for (let s = 0; s < 60; s++) world.step(DT);
  const first = energy();
  let most = 0;
  for (let s = 0; s < 3600; s++) {
    world.step(DT);
    most = Math.max(most, energy());
  }
  // Within a step energy wobbles by a few parts in a million, as an
  // integrator's does; over the minute it only falls. Measured: 93% kept;
  // 92.5% since the joints take each body's inertia where it has turned to
  // (turnedInverse()), at most 0.0015% over the first in between.
  assert.ok(most < first * 1.001, `energy rose ${((most / first - 1) * 100).toFixed(3)}%`);
  assert.ok(energy() > 0.9 * first, `${(energy() / first * 100).toFixed(1)}% of the energy left`);
});

test('a ball joint keeps its swing within its cone and its twist within its limits', () => {
  const world = new World({ gravity: [0, 0, 0] });
  const arm = world.addBody({ size: [0.1, 1, 0.1], position: [0, -0.5, 0] });
  world.addJoint({ type: 'ball', a: arm, anchor: [0, 0, 0], axis: [0, -1, 0], cone: 0.5, min: -0.3, max: 0.3 });
  arm.setAngularVelocity(3, 4, 2);
  let swing = 0, twist = 0;
  for (let s = 0; s < 240; s++) {
    world.step(DT);
    const down = turned(arm.rotation, [0, -1, 0]);
    swing = Math.max(swing, Math.acos(Math.min(1, -down[1])));
    // The twist: the arm's turn split into a swing of its axis and a turn
    // about it, which for a turn q from straight down is 2 atan2(q·y, w).
    const q = arm.rotation;
    twist = Math.max(twist, Math.abs(2 * Math.atan2(q[1], q[3])));
  }
  assert.ok(swing < 0.5 + 0.002, `swung ${swing} rad, the cone 0.5`);
  assert.ok(twist < 0.3 + 0.01, `twisted ${twist} rad, the limit 0.3`);
});

test('a fixed joint welds two bodies, and a joint to a body that can\'t move holds still', () => {
  const world = new World();
  world.addBody({ type: 'static', size: [20, 1, 20], position: [0, -0.5, 0] });
  const a = world.addBody({ position: [0, 3, 0], rotation: [0.2, 0.1, 0, 1] });
  const b = world.addBody({ shape: 'sphere', radius: 0.4, position: [0.9, 3.2, 0] });
  world.addJoint({ type: 'fixed', a, b });
  const local = turned([-a.rotation[0], -a.rotation[1], -a.rotation[2], a.rotation[3]],
    [b.position[0] - a.position[0], b.position[1] - a.position[1], b.position[2] - a.position[2]]);
  a.setAngularVelocity(0, 0, 4);
  // Tumbling, and landing: within the pair's tolerance, a 200th of the
  // ball's 0.8 m (measured 1.06 mm, the moment it hits the floor).
  let worst = 0;
  for (let s = 0; s < 180; s++) {
    world.step(DT);
    worst = Math.max(worst, apart(at(a, local), b.position));
  }
  assert.ok(worst < 0.004, `the weld opened ${worst} m`);
  // Joined to a static body, a kinematic one moves as it is told, and nothing breaks.
  const still = world.addBody({ type: 'static', position: [5, 1, 0] });
  const lift = world.addBody({ type: 'kinematic', position: [6, 1, 0], velocity: [0, 1, 0] });
  world.addJoint({ type: 'fixed', a: lift, b: still });
  world.step(DT);
  near(lift.position[1], 1 + DT, 1e-12, 'the lift rises regardless');
});

test('a body hanging from a moving kinematic one, or from a character, is carried', () => {
  const world = new World();
  const crane = world.addBody({ type: 'kinematic', position: [0, 5, 0] });
  const load = world.addBody({ shape: 'sphere', radius: 0.2, position: [0, 3, 0] });
  world.addJoint({ type: 'distance', a: load, b: crane, anchor: [0, 3, 0], anchorB: [0, 5, 0] });
  for (let s = 0; s < 120; s++) world.step(DT);
  assert.ok(load.asleep && crane.asleep, 'both asleep, hanging still');
  crane.setVelocity(1, 0, 0);
  for (let s = 0; s < 120; s++) world.step(DT);
  assert.ok(load.position[0] > 1, `the load followed to x = ${load.position[0]}`);
  near(apart(load.position, crane.position), 2, 1e-4, 'and hangs 2 m below,');

  const walker = world.addBody({ type: 'character', shape: 'capsule', radius: 0.3, height: 1.8, position: [10, 0.9, 0] });
  world.addBody({ type: 'static', size: [40, 1, 40], position: [0, -0.5, 0] });
  const lantern = world.addBody({ shape: 'sphere', radius: 0.05, position: [10.4, 0.8, 0] });
  world.addJoint({ a: lantern, b: walker, anchor: [10.4, 1.2, 0] });
  for (let s = 0; s < 120; s++) {
    walker.setVelocity(1, walker.velocity[1], 0);
    world.step(DT);
  }
  near(walker.position[0], 12, 1e-6, 'the character walked, unslowed, to');
  assert.ok(lantern.position[0] > 11.5, `the lantern came along to ${lantern.position[0]}`);
});

test('a chain on a turning kinematic plate is swept through a post, never flung, and stays a chain', () => {
  // Eight 40 cm links hung 5 m out from a plate turning about y, in the way
  // of a static post. Nothing stops the plate and nothing moves the post, so
  // the chain has to pass it. Struck against the post, a link can come back
  // at most as fast as it struck, and the chain whips it on as before: no
  // faster than twice the same chain with no post. And no joint opens wider
  // than a link is thick, 10 cm, or the chain has come apart. Before
  // (2026-10-27), replaying the tree's impulses at the warm start flung the
  // hinged chain at 3 rad/s to 2 x 10^6 m/s and opened its joints by
  // kilometres, and the catch held the ball-jointed one in the post, thrashing
  // at 4.7 times its speed with no post. Measured since: 1.00, 1.00 and 1.04
  // times, joints within 7.6, 3.2 and 30 mm; with each link's inertia taken
  // where it has turned to (turnedInverse()), 1.00 times all three, joints
  // within 7.6, 2.0 and 26 mm.
  const run = (type, spin, post) => {
    const world = new World();
    const plate = world.addBody({ type: 'kinematic', position: [0, 5, 0], size: [1, 0.2, 1], angularVelocity: [0, spin, 0] });
    if (post) world.addBody({ type: 'static', size: [0.3, 10, 0.3], position: [6, 5, 0] });
    const links = [];
    let last = plate;
    for (let i = 0; i < 8; i++) {
      const link = world.addBody({ size: [0.4, 0.1, 0.1], position: [-4 + 0.4 * (i + 0.5), 4, -3] });
      world.addJoint({ type, a: link, b: last, anchor: [-4 + 0.4 * i, 4, -3], axis: [0, 0, 1] });
      links.push(link);
      last = link;
    }
    let fastest = 0, gap = 0;
    for (let s = 0; s < 600; s++) {
      world.step(DT);
      for (const link of links) fastest = Math.max(fastest, Math.hypot(...link.velocity));
      for (let i = 1; i < 8; i++) gap = Math.max(gap, apart(at(links[i - 1], [0.2, 0, 0]), at(links[i], [-0.2, 0, 0])));
    }
    return { fastest, gap };
  };
  for (const [type, spin] of [['hinge', 3], ['ball', 1], ['hinge', 10]]) {
    const free = run(type, spin, false), hit = run(type, spin, true), what = `a ${type} chain at ${spin} rad/s`;
    assert.ok(hit.fastest <= 2 * free.fastest, `${what}: ${hit.fastest} m/s at the post, ${free.fastest} with none`);
    assert.ok(hit.gap < 0.1, `${what}: a joint opened ${hit.gap} m`);
  }
  // A free tree's bodies, held by their joints only to each other, are still
  // caught (World._catch): two 10 cm rods on a ball joint, thrown at 200 m/s
  // at a 1 cm wall, turned any way. With no jointed body caught, 18 in 40
  // went through; as it is, one, the 40th, which went through before too.
  let seed = 5, through = 0;
  const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
  for (let i = 0; i < 40; i++) {
    const world = new World({ gravity: [0, 0, 0] });
    world.addBody({ type: 'static', size: [0.01, 6, 6] });
    const q = [rnd() - 0.5, rnd() - 0.5, rnd() - 0.5, 1], x = -3 - rnd() * 200 / 60;
    const a = world.addBody({ size: [0.1, 0.02, 0.02], position: [x, 0, 0], rotation: q, velocity: [200, 0, 0] });
    const end = at(a, [0.05, 0, 0]), middle = at(a, [0.1, 0, 0]);
    const b = world.addBody({ size: [0.1, 0.02, 0.02], position: middle, rotation: a.rotation, velocity: [200, 0, 0] });
    world.addJoint({ a: b, b: a, anchor: end });
    for (let s = 0; s < 40; s++) world.step(DT);
    if (!(a.position[0] < 0 && b.position[0] < 0)) through++;
  }
  assert.ok(through <= 1, `${through} of 40 throws through the wall`);
});

test('a chain whipped round by a plate at 10 rad/s, touching nothing, keeps its joints shut', () => {
  // Eight 40 cm links hung 5 m out from a plate turning about y: the plate
  // yanks the first at 50 m/s and the chain cracks like a whip, its links
  // spinning at 100 to 250 rad/s, a few tenths of a radian a substep. A link
  // is 8.5 times easier to turn about its length than across it, and the
  // joints were solved with the inertia each link had as the step began
  // (contact.js, turnedInverse()): the ball-jointed chain opened its joints by
  // 4.9 m. Measured since: 26 mm, the hinged one 12 mm (17 mm before).
  for (const type of ['ball', 'hinge']) {
    const world = new World();
    let last = world.addBody({ type: 'kinematic', position: [0, 5, 0], size: [1, 0.2, 1], angularVelocity: [0, 10, 0] });
    const links = [last];
    for (let i = 0; i < 8; i++) {
      const link = world.addBody({ size: [0.4, 0.1, 0.1], position: [-4 + 0.4 * (i + 0.5), 4, -3] });
      world.addJoint({ type, a: link, b: last, anchor: [-4 + 0.4 * i, 4, -3], axis: [0, 0, 1] });
      links.push(link);
      last = link;
    }
    let gap = 0;
    for (let s = 0; s < 600; s++) {
      world.step(DT);
      gap = Math.max(gap, apart(at(links[0], [-4, -1, -3]), at(links[1], [-0.2, 0, 0])));
      for (let i = 2; i < 9; i++) gap = Math.max(gap, apart(at(links[i - 1], [0.2, 0, 0]), at(links[i], [-0.2, 0, 0])));
    }
    assert.ok(gap < 0.05, `a ${type} chain: a joint opened ${(gap * 1000).toFixed(1)} mm`);
  }
});

test('a joint and its bodies: removing either, the joint goes; handles, counts and alive', () => {
  const world = new World();
  const a = world.addBody({ position: [0, 5, 0] }), b = world.addBody({ position: [1, 5, 0] });
  const j = world.addJoint({ type: 'hinge', a, b });
  assert.ok(j instanceof Joint);
  assert.equal(world.jointCount, 1);
  assert.ok(j.alive);
  assert.equal(j.world, world);
  assert.equal(world.addJoint({ a: b }).alive, true);
  assert.equal(world.jointCount, 2);
  world.remove(b);   // both joints held b
  assert.equal(world.jointCount, 0);
  assert.equal(j.alive, false);
  world.remove(j);   // already gone: does nothing, as a body removed twice
  const c = world.addBody({ position: [2, 5, 0] });
  const k = world.addJoint({ a, b: c });
  // Its slot comes back under a new id: the old handle stays dead.
  assert.notEqual(k.id, j.id);
  assert.equal(j.alive, false);
  k.destroy();
  assert.equal(world.jointCount, 0);
  assert.equal(world.bodyCount, 2);
  world.destroy();
  assert.equal(k.alive, false);
});

test('joined bodies don\'t collide unless collide says they do, and can again once let go', () => {
  // Two boxes overlapping where a ball joint holds them, in no gravity.
  const world = new World({ gravity: [0, 0, 0] });
  const a = world.addBody({ position: [0, 0, 0] }), b = world.addBody({ position: [0.6, 0, 0] });
  const j = world.addJoint({ a, b, anchor: [0.3, 0, 0] });
  for (let s = 0; s < 10; s++) world.step(DT);
  assert.equal(world.touching(a).length, 0);
  near(b.position[0], 0.6, 1e-9, 'not pushed apart:');
  world.setJoint(j, { collide: true });
  world.step(DT);
  assert.equal(world.touching(a).length, 1);
  world.setJoint(j, { collide: false });
  world.step(DT);
  assert.equal(world.touching(a).length, 0);
  world.remove(j);
  for (let s = 0; s < 30; s++) world.step(DT);
  assert.ok(b.position[0] - a.position[0] > 0.99, 'let go, the overlap is pushed out');
});

test('a sleeping chain wakes as one when its joint changes, or anything touches it', () => {
  const world = new World();
  const links = chain(world, 10, 0.2);
  for (let s = 0; s < 90; s++) world.step(DT);
  assert.ok(links.every((b) => b.asleep), 'the chain sleeps');
  const steps = world._jointSteps;
  for (let s = 0; s < 10; s++) world.step(DT);
  assert.equal(world._jointSteps, steps, 'and asleep, solves nothing');
  // Any change to a joint wakes what it holds (rule 36), and a joint holds
  // its island: the whole chain wakes. Still hanging, it sleeps again.
  const top = world.addJoint({ type: 'distance', a: links[9], anchorB: [0, -2, 0], min: 0 });
  assert.ok(links.every((b) => !b.asleep), 'a new joint on its foot wakes all of it');
  world.remove(top);
  for (let s = 0; s < 90; s++) world.step(DT);
  assert.ok(links.every((b) => b.asleep), 'and it sleeps again');
  // Hit at its foot, all of it wakes, as a stack would.
  world.addBody({ shape: 'sphere', radius: 0.1, position: [0.15, -1.9, 0], velocity: [-2, 0, 0] });
  for (let s = 0; s < 10; s++) world.step(DT);
  assert.ok(links.every((b) => !b.asleep), 'hit, all of it wakes');
});

test('a ragdoll dropped on the floor comes to rest with its joints shut, and sleeps', () => {
  // bench/compare.html's ragdoll, 11 boxes and 10 joints, dropped from 0.2 m
  // and 1 m, turned 0 to 1.2 rad about x. With the joints solved before the
  // contacts in each pass, limbs lying on limbs rattled at 1 to 5 cm/s, step
  // to step: two of these never slept in 30 s, one took 20 s, and joints
  // opened up to 12 mm. Joints last, all slept within 10.9 s (most in 2 to
  // 4 s; the slowest waits on a forearm swinging on its hinge, which nothing
  // damps), and no joint opened more than 0.19 mm after the first 2 s. With
  // the joints taking each part's inertia where it has turned to
  // (turnedInverse()), all slept within 4.6 s and none opened more than 0.32 mm.
  const PARTS = [[[0.32, 0.2, 0.2], [0, 1, 0]], [[0.36, 0.45, 0.22], [0, 1.325, 0]], [[0.2, 0.24, 0.22], [0, 1.67, 0]],
    [[0.28, 0.1, 0.1], [-0.32, 1.5, 0]], [[0.26, 0.08, 0.08], [-0.59, 1.5, 0]], [[0.28, 0.1, 0.1], [0.32, 1.5, 0]],
    [[0.26, 0.08, 0.08], [0.59, 1.5, 0]], [[0.14, 0.44, 0.14], [-0.09, 0.68, 0]], [[0.12, 0.44, 0.12], [-0.09, 0.24, 0]],
    [[0.14, 0.44, 0.14], [0.09, 0.68, 0]], [[0.12, 0.44, 0.12], [0.09, 0.24, 0]]];
  const JOINTS = [['ball', 1, 0, [0, 1.1, 0]], ['ball', 2, 1, [0, 1.55, 0]], ['ball', 3, 1, [-0.18, 1.5, 0]],
    ['hinge', 4, 3, [-0.46, 1.5, 0], [0, 1, 0]], ['ball', 5, 1, [0.18, 1.5, 0]], ['hinge', 6, 5, [0.46, 1.5, 0], [0, 1, 0]],
    ['ball', 7, 0, [-0.09, 0.9, 0]], ['hinge', 8, 7, [-0.09, 0.46, 0], [1, 0, 0]], ['ball', 9, 0, [0.09, 0.9, 0]],
    ['hinge', 10, 9, [0.09, 0.46, 0], [1, 0, 0]]];
  const minus = (p, q) => [p[0] - q[0], p[1] - q[1], p[2] - q[2]];
  for (const height of [0.2, 1]) for (const angle of [0, 0.3, 0.6, 1.2]) {
    const q = [Math.sin(angle / 2), 0, 0, Math.cos(angle / 2)];
    const place = (p) => { const r = turned(q, p); return [r[0], r[1] + height, r[2]]; };
    const world = new World();
    world.addBody({ type: 'static', size: [20, 1, 20], position: [0, -0.5, 0] });
    const parts = PARTS.map(([size, p]) => world.addBody({ size, position: place(p), rotation: q }));
    const ends = JOINTS.map(([type, a, b, p, axis]) => {
      world.addJoint({ type, a: parts[a], b: parts[b], anchor: place(p), ...(axis ? { axis: turned(q, axis) } : {}) });
      return [parts[a], minus(p, PARTS[a][1]), parts[b], minus(p, PARTS[b][1])];
    });
    const what = `dropped from ${height} m turned ${angle} rad`;
    let gap = 0;
    for (let step = 0; step < 12 * 60 && !parts.every((b) => b.asleep); step++) {
      world.step(DT);
      if (step >= 120) for (const [a, la, b, lb] of ends) gap = Math.max(gap, apart(at(a, la), at(b, lb)));
    }
    assert.ok(parts.every((b) => b.asleep), `${what}: asleep within 12 s`);
    assert.ok(gap < 0.0005, `${what}: a joint opened ${(gap * 1000).toFixed(2)} mm`);
  }
});

test('off means off: no joints, no joint work', () => {
  const world = new World();
  world.addBody({ type: 'static', size: [20, 1, 20], position: [0, -0.5, 0] });
  for (let i = 0; i < 10; i++) world.addBody({ position: [i * 1.1, 0.5, 0] });
  for (let s = 0; s < 60; s++) world.step(DT);
  assert.equal(world._jointSteps, 0);
  assert.equal(world._joints.forest.capacity, 0);
});

test('jointOf gives back what addJoint took, and setJoint changes it', () => {
  const world = new World();
  const a = world.addBody({ position: [0, 2, 0] }), b = world.addBody({ position: [0, 3, 0] });
  const j = world.addJoint({ type: 'hinge', a, b, anchor: [0, 2.5, 0], axis: [1, 0, 0], min: -1, max: 1, motorForce: 5 });
  const o = world.jointOf(j);
  assert.equal(o.type, 'hinge');
  assert.equal(o.a.id, a.id);
  assert.equal(o.b.id, b.id);
  assert.deepEqual(o.anchor, [0, 2.5, 0]);
  assert.ok(apart(o.axis, [1, 0, 0]) < 1e-15, `${o.axis}`);
  assert.equal(o.min, -1);
  assert.equal(o.max, 1);
  assert.equal(o.motorSpeed, 0);
  assert.equal(o.motorForce, 5);
  assert.equal(o.friction, 0);
  assert.equal(o.collide, false);
  assert.equal('cone' in o || 'length' in o || 'anchorB' in o, false);
  // A copy, from what jointOf gave.
  const twin = world.addJoint(world.jointOf(j));
  assert.deepEqual(world.jointOf(twin), o);
  // A new type starts from its own defaults; the anchors and collide carry over.
  world.setJoint(j, { type: 'distance', collide: true });
  const d = world.jointOf(j);
  assert.equal(d.type, 'distance');
  assert.deepEqual(d.anchor, [0, 2.5, 0]);
  assert.deepEqual(d.anchorB, [0, 2.5, 0]);
  assert.equal(d.length, 0);
  assert.equal(d.collide, true);
  assert.equal('min' in d || 'max' in d || 'friction' in d, false);
  // Friction, for a ball joint, a hinge and a slider, and copied with the rest.
  const ball = world.addJoint({ a, b, friction: 3 });
  assert.equal(world.jointOf(ball).friction, 3);
  world.setJoint(ball, { friction: 0.5 });
  assert.equal(world.jointOf(world.addJoint(world.jointOf(ball))).friction, 0.5);
  // null puts a limit back to its default.
  world.setJoint(twin, { min: null });
  assert.equal('min' in world.jointOf(twin), false);
  // To the world, a distance joint's far end is a point, given as anchorB.
  const rope = world.addJoint({ type: 'distance', a, anchorB: [0, 6, 0], min: 0 });
  const r = world.jointOf(rope);
  assert.equal(r.b, null);
  assert.deepEqual(r.anchorB, [0, 6, 0]);
  near(r.length, 4, 1e-15, 'as long as its ends were apart:');
});

test('the tree solve agrees with a dense solve of the same equations', () => {
  // After a step, the last pass's joints, rows and targets, solved densely:
  // K λ = r, K = J W Jᵀ. Two planks, the far one held to the world at a leaf
  // of the tree, where solving leaves first has a block its children can't
  // move (forest.js, GIVE): exact but for the joints' millionth of give.
  for (const held of ['root', 'leaf', 'both']) {
    const world = new World();
    const p0 = world.addBody({ size: [0.25, 0.04, 1], position: [0.125, -0.1, 0], rotation: [0, 0, 0.1, 0.995] });
    const p1 = world.addBody({ size: [0.25, 0.04, 1], position: [0.375, -0.12, 0] });
    // Held at both ends, a V, so the system isn't the taut string a straight
    // one is, whose answer is no one set of impulses.
    if (held === 'root') world.addJoint({ type: 'hinge', a: p0, anchor: [0, -0.1, 0], axis: [0, 0, 1] });
    if (held === 'both') world.addJoint({ a: p0, anchor: [0, 0, 0] });
    world.addJoint({ type: 'hinge', a: p0, b: p1, anchor: [0.25, -0.11, 0], axis: [0, 0, 1] });
    if (held !== 'root') world.addJoint({ a: p1, anchor: [0.5, held === 'both' ? 0 : -0.12, 0] });
    world.step(DT);
    const F = world._joints.forest, rows = [];
    for (let t = 0; t < F.count; t++) for (let r = 0; r < F.rows[t]; r++) rows.push([t, r]);
    const J6 = (t, r, k) => {
      const o = 54 * t + 9 * r, w = F.row;
      if (F.side[2 * t] === k) return [-w[o], -w[o + 1], -w[o + 2], -w[o + 3], -w[o + 4], -w[o + 5]];
      if (F.side[2 * t + 1] === k) return [w[o], w[o + 1], w[o + 2], w[o + 6], w[o + 7], w[o + 8]];
      return null;
    };
    const n = rows.length, K = Array.from({ length: n }, () => new Array(n).fill(0));
    for (let k = 0; k < F.bodies; k++) {
      const s = F.body[k], m = world._invMass[s], I = world._invI.subarray(6 * s, 6 * s + 6);
      const W = [[m, 0, 0, 0, 0, 0], [0, m, 0, 0, 0, 0], [0, 0, m, 0, 0, 0],
        [0, 0, 0, I[0], I[1], I[2]], [0, 0, 0, I[1], I[3], I[4]], [0, 0, 0, I[2], I[4], I[5]]];
      for (let i = 0; i < n; i++) {
        const ji = J6(rows[i][0], rows[i][1], k);
        if (!ji) continue;
        const wj = W.map((row) => row.reduce((sum, x, q) => sum + x * ji[q], 0));
        for (let j = 0; j < n; j++) {
          const jj = J6(rows[j][0], rows[j][1], k);
          if (jj) K[j][i] += jj.reduce((sum, x, q) => sum + x * wj[q], 0);
        }
      }
    }
    const A = K.map((row, i) => [...row, F.rhs[6 * rows[i][0] + rows[i][1]]]);
    for (let c = 0; c < n; c++) {
      let b = c;
      for (let i = c + 1; i < n; i++) if (Math.abs(A[i][c]) > Math.abs(A[b][c])) b = i;
      [A[c], A[b]] = [A[b], A[c]];
      for (let i = 0; i < n; i++) {
        if (i === c) continue;
        const f = A[i][c] / A[c][c];
        for (let q = c; q <= n; q++) A[i][q] -= f * A[c][q];
      }
    }
    let scale = 0, worst = 0;
    for (let i = 0; i < n; i++) scale = Math.max(scale, Math.abs(A[i][n] / A[i][i]));
    for (let i = 0; i < n; i++) {
      const want = A[i][n] / A[i][i], got = F.vector[6 * rows[i][0] + rows[i][1]];
      worst = Math.max(worst, Math.abs(got - want) / scale);
    }
    assert.ok(worst < 1e-4, `held at the ${held}: off by ${worst} of the largest impulse`);
  }
});

test('the same joints give the same bits', () => {
  const run = () => {
    const world = new World();
    world.addBody({ type: 'static', size: [20, 1, 20], position: [0, -0.5, 0] });
    const links = chain(world, 8, 0.25);
    links[7].setVelocity(3, 0, 1);
    const box = world.addBody({ position: [1, 3, 0.2] });
    world.addJoint({ type: 'hinge', a: box, b: links[3], anchor: [0.5, -0.8, 0], axis: [0, 0, 1], min: -0.5, max: 0.5, motorSpeed: 1, motorForce: 5 });
    for (let s = 0; s < 240; s++) world.step(DT);
    return Array.from(world.transforms);
  };
  assert.deepEqual(run(), run());
});
