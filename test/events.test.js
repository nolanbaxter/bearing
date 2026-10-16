// Contact events and sensors: what began and ended touching, held to a
// touching set worked out without the code that writes the events.
// Run: node test/events.test.js

import assert from 'node:assert/strict';
import { World } from '../src/bearing.js';
import { collide, Shape, Manifold, SPHERE, CAPSULE, BOX } from '../src/collide.js';
import { suite, test } from './test.js';

suite('events');

const DT = 1 / 60;
const key = (x, y) => (x < y ? `${x}:${y}` : `${y}:${x}`);

/** A body's shape where it is now, from its options alone. */
function shapeOf(world, body) {
  const d = world.bodyOf(body), s = new Shape();
  if (d.shape === 'box') { s.kind = BOX; s.half.set(d.size.map((x) => x / 2)); }
  else if (d.shape === 'sphere') { s.kind = SPHERE; s.half[0] = d.radius; }
  else { s.kind = CAPSULE; s.half[0] = d.radius; s.half[1] = d.height / 2 - d.radius; }
  const p = body.position;
  return { shape: s.place(p[0], p[1], p[2], body.rotation), size: d.shape === 'box' ? Math.min(...d.size) : 2 * d.radius, desc: d };
}

/**
 * Which pairs touch after a step, worked out here: two bodies touch if,
 * where they are now, they are within the contact tolerance (a pixel, a
 * 200th of the smaller one's thinnest size) by the narrowphase run afresh,
 * or the solver pushed them apart in the step, as its contacts say (a ball
 * that hits and leaves within one step is never within it at the step's
 * end). Pairs that never collide are left out: two bodies neither of which is
 * dynamic, unless one is a sensor; two sensors; two static ones. Also the
 * pairs too near the line to call: within a millionth of a size of the
 * tolerance, where the step's own estimate of where it left them decides.
 */
function reference(world, bodies) {
  const touching = new Set(), close = new Set(), m = new Manifold();
  const placed = bodies.filter((b) => b.alive).map((b) => ({ body: b, ...shapeOf(world, b) }));
  // The step measures a pair's gap along the normal its contact had when the
  // step began, so a pair that slid past itself by d in the step is measured
  // off by up to about d² over its size when the contact is curved: two round
  // bodies, or a round one on a box's edge or corner. For those, the line is
  // that wide. (Against a flat face the normal holds as they slide, and
  // turning carries a contact's anchors with the bodies exactly.)
  const c = world._contacts, slots = world._slots, st = world._start, slip = new Map();
  for (let n = 0; n < c.count; n++) {
    const a = c.a[n], b = c.b[n], n3 = 3 * n, d = [0, 1, 2].map((k) => world._position[3 * b + k] - st[3 * b + k] - world._position[3 * a + k] + st[3 * a + k]);
    const along = d[0] * c.normal[n3] + d[1] * c.normal[n3 + 1] + d[2] * c.normal[n3 + 2];
    const isRound = (x) => world._shape[x] === 1 || world._shape[x] === 2;
    // On a box's face, the normal is along one of the box's axes.
    const onEdge = (x) => world._shape[x] === 0 && Math.max(...[0, 1, 2].map((k) => Math.abs(world._axes[9 * x + 3 * k] * c.normal[n3]
      + world._axes[9 * x + 3 * k + 1] * c.normal[n3 + 1] + world._axes[9 * x + 3 * k + 2] * c.normal[n3 + 2]))) < 1 - 1e-9;
    const round = (isRound(a) && (isRound(b) || onEdge(b))) || (isRound(b) && onEdge(a));
    if (round) slip.set(key(slots.id(a), slots.id(b)), Math.max(0, d[0] ** 2 + d[1] ** 2 + d[2] ** 2 - along ** 2));
  }
  for (let i = 0; i < placed.length; i++) {
    for (let j = i + 1; j < placed.length; j++) {
      const A = placed[i], B = placed[j], ta = A.desc.type, tb = B.desc.type, sa = A.desc.sensor, sb = B.desc.sensor;
      if (sa && sb) continue;
      if (!sa && !sb && ta !== 'dynamic' && tb !== 'dynamic') continue;
      if (ta === 'static' && tb === 'static') continue;
      const size = Math.min(A.size, B.size), tolerance = size / 200, k = key(A.body.id, B.body.id);
      const line = 1e-6 * size + (slip.get(k) ?? 0) / size;
      if (collide(A.shape, B.shape, tolerance + line, tolerance, m) === 0) continue;
      const least = Math.min(...m.separations.subarray(0, m.count));
      if (Math.abs(least - tolerance) < line) close.add(k);
      else if (least <= tolerance) touching.add(k);
    }
  }
  // What the solver pushed: its own contacts, read back, by their bodies'
  // slots (the pair list is reordered as islands fall asleep at a step's end).
  for (let n = 0; n < c.count; n++) {
    for (let k = 0; k < c.points[n]; k++) {
      if (c.maxNormalImpulse[n * 4 + k] > 0) touching.add(key(slots.id(c.a[n]), slots.id(c.b[n])));
    }
  }
  // And which touches the step foresaw: a contact that pushed, or read the
  // pair within its tolerance. One it didn't, a resting body knocked into
  // another in the middle of the step, or a turning body meeting with a
  // corner its contact didn't hold, is only seen at the next.
  const foreseen = new Set();
  for (let n = 0; n < c.count; n++) {
    let seen = c.gap[n] <= c.tolerance[n];
    for (let k = 0; k < c.points[n]; k++) seen ||= c.maxNormalImpulse[n * 4 + k] > 0;
    if (seen) foreseen.add(key(slots.id(c.a[n]), slots.id(c.b[n])));
  }
  return { touching, close, foreseen };
}

const isSensor = (world, bodies, id) => bodies.some((b) => String(b.id) === id && b.alive && world.bodyOf(b).sensor);

/** Plays the step's events onto `now`, checking each pair alternates begin, end, begin, from not touching. */
function play(events, now, step) {
  for (const e of events) {
    const k = key(e.a.id, e.b.id);
    if (e.type === 'begin') {
      assert.ok(!now.has(k), `step ${step}: a second begin for ${k}`);
      now.add(k);
    } else {
      assert.ok(now.has(k), `step ${step}: an end for ${k}, which wasn't touching`);
      now.delete(k);
    }
  }
}

/**
 * The scripted scene: a stack, a slide off a ledge, bouncing boxes and
 * balls, a capsule dropped, a kinematic platform through a sensor zone, a
 * paddle knocking balls away, a body replaced in the same spot (its slot
 * reused, touching the same neighbours) and one removed mid-scene. Bodies
 * knocked away within a step are the pairs the solver pushed that end it
 * more than a pixel apart.
 */
function scene() {
  const world = new World();
  const bodies = [];
  const add = (o) => { const b = world.addBody(o); bodies.push(b); return b; };
  add({ type: 'static', size: [30, 1, 30], position: [0, -0.5, 0] });
  add({ type: 'static', size: [4, 2, 4], position: [-8, 1, 0] });                         // a ledge
  const stack = [0, 1, 2].map((k) => add({ position: [3, 0.5 + k, 3] }));
  add({ position: [-9, 2.5, 0], velocity: [3, 0, 0], friction: 0.1 });                     // slides off the ledge
  add({ shape: 'sphere', radius: 0.3, position: [0, 4, -4], restitution: 0.8 });           // bounces
  add({ shape: 'sphere', radius: 0.5, position: [5, 3, -2] });
  add({ shape: 'capsule', radius: 0.25, height: 1.5, position: [-2, 2, 4], rotation: [0, 0, 0.3826834, 0.9238795] });
  add({ type: 'kinematic', size: [3, 0.4, 3], position: [-4, 0.2, -6], velocity: [2, 0, 0] });
  add({ type: 'static', size: [2, 2, 2], position: [1, 1, -6], sensor: true });           // a zone the platform passes
  const ball = add({ shape: 'sphere', radius: 0.4, position: [1, 4, -6] });               // falls into the zone
  for (let n = 0; n < 12; n++) {
    const at = [8 + (n % 4) * 1.2, 1 + Math.floor(n / 4) * 1.3, -8 + (n % 3) * 1.1];
    add(n % 2 ? { shape: 'sphere', radius: 0.35, position: at, restitution: 1 } : { size: [0.6, 0.6, 0.6], position: at, restitution: 1 });
  }
  for (let n = 0; n < 6; n++) add({ shape: 'sphere', radius: 0.2, position: [n * 0.45 - 2, 0.2, 9] });
  add({ type: 'kinematic', size: [0.5, 1, 4], position: [-6, 0.5, 9], velocity: [12, 0, 0] });   // the paddle
  return { world, bodies, stack, ball, add };
}

function run(script) {
  const { world, bodies, stack, ball, add } = scene();
  const now = new Set(), log = [];
  let close = 0, compared = 0, late = new Set(), lateCount = 0;
  for (let step = 0; step < 300; step++) {
    if (step === 120) {
      // The middle of the stack, replaced where it stood: the same slot, the same neighbours.
      const at = stack[1].position;
      world.remove(stack[1]);
      stack[1] = add({ position: at });
      assert.equal(stack[1].slot, bodies[3].slot);
    }
    if (step === 200) world.remove(ball);                        // removed mid-scene
    world.step(DT);
    const events = world.contacts();
    log.push(events.map((e) => [e.type, e.a.id, e.b.id]));
    play(events, now, step);
    if (script) script(world, bodies, now, step);
    const ref = reference(world, bodies);
    close += ref.close.size;
    // Last step's unforeseen touches: begun now, or no longer touching.
    for (const k of late) assert.ok(now.has(k) || !ref.touching.has(k), `step ${step}: ${k} still not begun`);
    late = new Set();
    for (const k of new Set([...now, ...ref.touching])) {
      if (ref.close.has(k)) continue;
      if (!now.has(k) && !ref.foreseen.has(k) && !k.split(':').some((id) => isSensor(world, bodies, id))) {
        late.add(k);
        lateCount++;
        continue;
      }
      compared++;
      assert.equal(now.has(k), ref.touching.has(k), `step ${step}: pair ${k} is ${now.has(k) ? '' : 'not '}touching by the events, and ${ref.touching.has(k) ? '' : 'not '}by the reference`);
    }
  }
  return { log, close, compared, lateCount, world, bodies, now };
}

test('over a scripted scene, the events are exactly the changes in an independently worked-out touching set', () => {
  const { close, compared, lateCount } = run();
  assert.ok(compared > 2000, `${compared} pair-steps compared`);
  // Unforeseen touches are the rare case, and a step late at most.
  assert.ok(lateCount < compared / 100, `${lateCount} unforeseen of ${compared}`);
  // Too near the line to call, so left out: a few, the curved contacts the
  // step measures along a fixed normal (11 of 4,138 when measured).
  assert.ok(close < compared / 100, `${close} too close to call, of ${compared}`);
});

test('world.touching agrees with the events, for every body, every step', () => {
  run((world, bodies, now, step) => {
    for (const b of bodies) {
      if (!b.alive) continue;
      const got = world.touching(b).map((o) => key(b.id, o.id)).sort();
      const want = [...now].filter((k) => k.split(':').includes(String(b.id))).sort();
      assert.deepEqual(got, want, `step ${step}: body ${b.id}`);
    }
  });
});

test('the same scene gives the same events, in the same order, every time', () => {
  assert.deepEqual(run().log, run().log);
});

test('removing a body ends its touches in the next step, its handle dead; one added touching begins', () => {
  const world = new World();
  const floor = world.addBody({ type: 'static', size: [10, 1, 10], position: [0, -0.5, 0] });
  const [low, mid, top] = [0, 1, 2].map((k) => world.addBody({ position: [0, 0.5 + k, 0] }));
  for (let i = 0; i < 30; i++) world.step(DT);
  assert.equal(world.touching(mid).length, 2);
  world.remove(mid);
  assert.deepEqual(world.contacts(), [], 'nothing until the step');
  world.step(DT);
  const ends = world.contacts(mid);
  assert.deepEqual(ends.map((e) => [e.type, e.a.id, e.b.id]).sort(), [['end', mid.id, low.id], ['end', mid.id, top.id]].sort());
  for (const e of ends) assert.equal(e.a.alive, false);
  // Set down already touching the floor: a begin, at once.
  const added = world.addBody({ position: [3, 0.5, 0] });
  world.step(DT);
  assert.deepEqual(world.contacts(added).map((e) => [e.type, e.a.id, e.b.id]), [['begin', added.id, floor.id]]);
});

test('a dropped ball begins touching at the speed it fell, √(2gh), and the point is where it landed', () => {
  for (const h of [1, 5, 20]) {
    const world = new World();
    world.addBody({ type: 'static', size: [10, 1, 10], position: [0, -0.5, 0] });
    const ball = world.addBody({ shape: 'sphere', radius: 0.25, position: [1, h + 0.25, 2] });
    let hit = null;
    for (let i = 0; i < 200 && !hit; i++) {
      world.step(DT);
      hit = world.contacts(ball).find((e) => e.type === 'begin');
    }
    const expected = Math.sqrt(2 * 9.81 * h);
    assert.ok(Math.abs(hit.speed - expected) < 0.01 * expected, `from ${h} m: ${hit.speed} m/s, expected ${expected}`);
    // Turned so the ball is a: the normal points from it, down into the floor.
    assert.ok(Math.abs(hit.normal[1] + 1) < 1e-12, `normal ${hit.normal}`);
    assert.ok(Math.hypot(hit.point[0] - 1, hit.point[1], hit.point[2] - 2) < 0.01, `point ${hit.point}`);
  }
});

test('a sensor sees a box in and out on the right steps, and changes nothing about its path', () => {
  const slide = (sensor) => {
    const world = new World();
    world.addBody({ type: 'static', size: [30, 1, 30], position: [0, -0.5, 0] });
    const box = world.addBody({ position: [-6, 0.5, 0], velocity: [6, 0, 0], friction: 0 });
    const zone = sensor ? world.addBody({ type: 'static', size: [2, 2, 2], position: [4, 1, 0], sensor: true }) : null;
    const path = [], seen = [];
    for (let i = 0; i < 180; i++) {
      world.step(DT);
      path.push([...box.position, ...box.rotation]);
      if (zone) for (const e of world.contacts(zone)) seen.push([i, e.type, e.b.id === box.id, e.speed]);
    }
    return { path, seen };
  };
  const plain = slide(false), sensed = slide(true);
  assert.deepEqual(sensed.path, plain.path, 'the sensor moved the box');
  // Frictionless at 6 m/s, the box's front reaches the zone (x = 3) after
  // 85 steps and its back leaves it (x = 5) after 116, worked out from the
  // speed alone; touching counts from within a pixel (5 mm).
  const front = (i) => -6 + 6 * (i + 1) * DT + 0.5, back = (i) => -6 + 6 * (i + 1) * DT - 0.5;
  let enter = 0;
  while (front(enter) < 3 - 0.005) enter++;
  let leave = enter;
  while (back(leave) <= 5 + 0.005) leave++;
  assert.deepEqual(sensed.seen.map((s) => s.slice(0, 3)), [[enter, 'begin', true], [leave, 'end', true]]);
  assert.ok(Math.abs(sensed.seen[0][3] - 6) < 1e-9, `met at ${sensed.seen[0][3]} m/s`);
});

test('a static zone sees a kinematic body; a kinematic sensor reports a sleeping pile without waking it', () => {
  const world = new World();
  world.addBody({ type: 'static', size: [30, 1, 30], position: [0, -0.5, 0] });
  const zone = world.addBody({ type: 'static', shape: 'sphere', radius: 1, position: [3, 1, 0], sensor: true });
  const lift = world.addBody({ type: 'kinematic', size: [1, 0.2, 1], position: [0, 1, 0], velocity: [1, 0, 0] });
  const pile = [0, 1, 2, 3].map((k) => world.addBody({ position: [-6 + k * 1.1, 0.5, 4] }));
  for (let i = 0; i < 150; i++) world.step(DT);
  assert.ok(pile.every((b) => b.asleep));
  assert.deepEqual(world.touching(zone).map((b) => b.id), [lift.id], 'the zone sees the lift');
  // A sweeping sensor: it sees each box of the pile, and wakes none.
  const sweep = world.addBody({ type: 'kinematic', size: [0.5, 2, 2], position: [-8, 1, 4], velocity: [4, 0, 0], sensor: true });
  const begun = new Set();
  for (let i = 0; i < 90; i++) {
    world.step(DT);
    for (const e of world.contacts(sweep)) if (e.type === 'begin') begun.add(e.b.id);
    assert.ok(pile.every((b) => b.asleep), `step ${i}: the sensor woke the pile`);
  }
  for (const b of pile) assert.ok(begun.has(b.id), `never saw box ${b.id}`);
  // Two sensors never see each other, and two static bodies never pair.
  const other = world.addBody({ type: 'kinematic', size: [1, 1, 1], position: [3, 1, 0], sensor: true });
  const rock = world.addBody({ type: 'static', size: [1, 1, 1], position: [3.5, 1, 0] });
  world.step(DT);
  assert.ok(!world.touching(zone).some((b) => b.id === other.id || b.id === rock.id));
});

test('sleeping keeps touches with no events, and waking without moving makes none', () => {
  const world = new World();
  const floor = world.addBody({ type: 'static', size: [10, 1, 10], position: [0, -0.5, 0] });
  const box = world.addBody({ position: [0, 0.5, 0] });
  let n = 0;
  while (!box.asleep && n++ < 300) world.step(DT);
  assert.ok(box.asleep);
  for (let i = 0; i < 120; i++) {
    world.step(DT);
    assert.deepEqual(world.contacts(), []);
  }
  assert.deepEqual(world.touching(box).map((b) => b.id), [floor.id]);
  world.setBody(box, { friction: 0.6 });
  assert.ok(!box.asleep);
  for (let i = 0; i < 30; i++) {
    world.step(DT);
    assert.deepEqual(world.contacts(), []);
  }
});

test('a box landing across many triangles of a mesh begins touching it once', () => {
  const world = new World();
  const vertices = [], indices = [];
  for (let j = 0; j <= 8; j++) for (let i = 0; i <= 8; i++) vertices.push(i * 0.5 - 2, 0, j * 0.5 - 2);
  for (let j = 0; j < 8; j++) for (let i = 0; i < 8; i++) { const a = j * 9 + i; indices.push(a, a + 9, a + 10, a, a + 10, a + 1); }
  const mesh = world.addBody({ shape: 'mesh', vertices, indices });
  const box = world.addBody({ size: [1.3, 1, 1.3], position: [0.1, 2, 0.2] });
  const all = [];
  for (let i = 0; i < 120; i++) { world.step(DT); all.push(...world.contacts()); }
  assert.deepEqual(all.map((e) => [e.type, e.a.id, e.b.id]), [['begin', mesh.id, box.id]]);
});

test('queries don\'t see sensors', () => {
  const world = new World();
  const zone = world.addBody({ type: 'static', size: [4, 4, 4], sensor: true });
  const wall = world.addBody({ type: 'static', size: [1, 4, 4], position: [5, 0, 0] });
  assert.equal(world.raycast([-10, 0, 0], [1, 0, 0]).body.id, wall.id);
  assert.equal(world.shapeCast({ shape: 'sphere', radius: 0.2 }, [-10, 0, 0], [1, 0, 0]).body.id, wall.id);
  assert.deepEqual(world.overlap({}, [0, 0, 0]), []);
  assert.equal(world.raycastAll([-10, 0, 0], [1, 0, 0]).length, 1);
  void zone;
});

test('off means off: no sensors, no sensor pass; nothing changing, no events', () => {
  const world = new World({ sleeping: false });
  world.addBody({ type: 'static', size: [10, 1, 10], position: [0, -0.5, 0] });
  for (let k = 0; k < 5; k++) world.addBody({ position: [0, 0.5 + k, 0] });
  for (let i = 0; i < 60; i++) world.step(DT);
  // Settled, and kept awake: every step looks at every contact, and changes nothing.
  for (let i = 0; i < 120; i++) {
    world.step(DT);
    assert.equal(world._events.count, 0);
  }
  assert.equal(world._sensorPasses, 0);
  const zone = world.addBody({ type: 'static', size: [1, 1, 1], position: [5, 5, 5], sensor: true });
  world.step(DT);
  // A sensor with nothing near it is in no pair, so there is still nothing to ask.
  assert.equal(world._sensorPasses, 0);
  world.setBody(zone, { position: [0, 2, 0] });
  world.step(DT);
  assert.equal(world._sensorPasses, 1);
  world.remove(zone);
  world.step(DT);
  assert.equal(world._sensorCount, 0);
});
