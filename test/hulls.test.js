// Convex hulls and compound bodies: quickhull, mass properties held to
// formulas and to integration, contacts held to boxes', and bodies that turn
// about a centre of mass off their origin.
// Run: node test/hulls.test.js

import assert from 'node:assert/strict';
import { World } from '../src/bearing.js';
import { build } from '../src/hull.js';
import { collide, Shape, Manifold, Placed, BOX, HULL } from '../src/collide.js';
import { suite, test } from './test.js';

suite('hulls');

const DT = 1 / 60;
const near = (got, want, eps, what) => assert.ok(Math.abs(got - want) <= eps, `${what}: ${got}, expected ${want}`);
const corners = (w, h, d) => {
  const out = [];
  for (const z of [-d / 2, d / 2]) for (const y of [-h / 2, h / 2]) for (const x of [-w / 2, w / 2]) out.push(x, y, z);
  return out;
};
/** A fixed run of numbers in [0, 1), the same every time. */
const numbers = (seed) => () => (seed = (seed * 16807) % 2147483647) / 2147483647;

test('quickhull keeps the corners: a cube with points inside and on it has 8 corners and 6 square faces', () => {
  const h = build([...corners(1, 1, 1), 0, 0, 0, 0.5, 0.5, 0, 0, 0.5, 0.2, 0.1, -0.5, 0.3, 0.5, 0.5, 0.5]);
  assert.equal(h.vertexCount, 8);
  assert.equal(h.faceCount, 6);
  for (let f = 0; f < 6; f++) assert.equal(h.start[f + 1] - h.start[f], 4);
});

test('quickhull of points on a sphere and in a cloud: every point inside, every face outward, V - E + F = 2', () => {
  const rnd = numbers(5), sphere = [], cloud = [];
  for (let i = 0; i < 300; i++) {
    const y = 1 - 2 * (i + 0.5) / 300, r = Math.sqrt(1 - y * y), t = i * 2.399963;
    sphere.push(r * Math.cos(t), y, r * Math.sin(t));
  }
  for (let i = 0; i < 2000; i++) cloud.push(rnd() * 2 - 1 + 50, rnd() * 3 - 1.5, rnd() - 0.5);
  for (const points of [sphere, cloud]) {
    const h = build(points), p = h.planes, v = h.vertices;
    assert.equal(h.vertexCount - h.edges.length / 2 + h.faceCount, 2);
    for (let f = 0; f < h.faceCount; f++) {
      for (let i = 0; i < points.length; i += 3) {
        assert.ok(p[4 * f] * points[i] + p[4 * f + 1] * points[i + 1] + p[4 * f + 2] * points[i + 2] - p[4 * f + 3] <= 0, `point ${i / 3} outside face ${f}`);
      }
      // Its corners run counter-clockwise seen from outside: the turn at each is along the normal.
      const a = h.start[f], n = h.start[f + 1] - a;
      for (let k = 0; k < n; k++) {
        const i = 3 * h.loops[a + k], j = 3 * h.loops[a + (k + 1) % n], l = 3 * h.loops[a + (k + 2) % n];
        const ux = v[j] - v[i], uy = v[j + 1] - v[i + 1], uz = v[j + 2] - v[i + 2];
        const wx = v[l] - v[j], wy = v[l + 1] - v[j + 1], wz = v[l + 2] - v[j + 2];
        const turn = (uy * wz - uz * wy) * p[4 * f] + (uz * wx - ux * wz) * p[4 * f + 1] + (ux * wy - uy * wx) * p[4 * f + 2];
        assert.ok(turn > 0, `face ${f} turns the wrong way at corner ${k}`);
      }
    }
  }
});

test('the same points give the same hull to the bit, and points in one plane make none', () => {
  const rnd = numbers(9), points = [];
  for (let i = 0; i < 500; i++) points.push(rnd(), rnd() * 2, rnd() * 0.5);
  const a = build(points), b = build(points);
  assert.deepEqual(Array.from(a.vertices), Array.from(b.vertices));
  assert.deepEqual(Array.from(a.planes), Array.from(b.planes));
  assert.equal(build([0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 0.5, 0.5, 0]), null);
  assert.equal(build([0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3]), null);
  assert.equal(build([0, 0, 0, 1, 0, 0, 0, 1, 0]), null);
});

test("a hull of a box\'s corners weighs and turns as the box, to rounding", () => {
  const world = new World();
  const box = world.addBody({ size: [1, 0.6, 0.8], position: [2, 0, 0] });
  const hull = world.addBody({ shape: 'hull', vertices: corners(1, 0.6, 0.8), position: [2, 0, 0] });
  near(hull.mass, box.mass, 1e-12 * box.mass, 'mass');
  for (let k = 0; k < 3; k++) {
    near(hull.inertia[k], box.inertia[k], 1e-12 * box.inertia[k], `inertia ${k}`);
    near(hull.centerOfMass[k], box.centerOfMass[k], 1e-14, `centre ${k}`);
  }
  assert.deepEqual(hull.position, [2, 0, 0]);
});

test("a tetrahedron has its formulas: volume 1/6, centre at the corners\' mean, principal moments ρ/60 and ρ/96 twice", () => {
  const world = new World();
  const t = world.addBody({ shape: 'hull', vertices: [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1] });
  near(t.mass, 1000 / 6, 1e-12, 'mass');
  for (let k = 0; k < 3; k++) near(t.centerOfMass[k], 0.25, 1e-15, `centre ${k}`);
  const moments = [...t.inertia].sort((a, b) => a - b);
  near(moments[0], 1000 / 96, 1e-12, 'least');
  near(moments[1], 1000 / 96, 1e-12, 'middle');
  near(moments[2], 1000 / 60, 1e-12, 'most');
});

/**
 * A hull's volume, centre and inertia tensor about its centre, for density
 * 1, by slicing it across z: each slice's cross-section, where the plane
 * cuts the hull's edges, is a convex polygon whose area and moments are
 * exact, and Simpson's rule sums the slices. Shares nothing with shapes.js.
 */
function sliced(hull, slices) {
  const v = hull.vertices, e = hull.edges;
  let lo = Infinity, hi = -Infinity;
  for (let i = 2; i < v.length; i += 3) { lo = Math.min(lo, v[i]); hi = Math.max(hi, v[i]); }
  const at = (z) => {
    const pts = [];
    for (let k = 0; k < e.length; k += 2) {
      const a = 3 * e[k], b = 3 * e[k + 1], za = v[a + 2], zb = v[b + 2];
      if ((za - z) * (zb - z) > 0 || za === zb) continue;
      const u = (z - za) / (zb - za);
      pts.push([v[a] + u * (v[b] - v[a]), v[a + 1] + u * (v[b + 1] - v[a + 1])]);
    }
    if (pts.length < 3) return [0, 0, 0, 0, 0, 0];
    const cx = pts.reduce((s, p) => s + p[0], 0) / pts.length, cy = pts.reduce((s, p) => s + p[1], 0) / pts.length;
    pts.sort((p, q) => Math.atan2(p[1] - cy, p[0] - cx) - Math.atan2(q[1] - cy, q[0] - cx));
    // Area, ∫x, ∫y, ∫x², ∫y², ∫xy over the polygon (Green's theorem).
    let A = 0, X = 0, Y = 0, XX = 0, YY = 0, XY = 0;
    for (let i = 0; i < pts.length; i++) {
      const [x0, y0] = pts[i], [x1, y1] = pts[(i + 1) % pts.length], c = x0 * y1 - x1 * y0;
      A += c / 2; X += (x0 + x1) * c / 6; Y += (y0 + y1) * c / 6;
      XX += (x0 * x0 + x0 * x1 + x1 * x1) * c / 12; YY += (y0 * y0 + y0 * y1 + y1 * y1) * c / 12;
      XY += (x0 * y1 + 2 * x0 * y0 + 2 * x1 * y1 + x1 * y0) * c / 24;
    }
    return [A, X, Y, XX, YY, XY];
  };
  // Simpson: ∫ f(z) [1, x, y, z, xx, yy, zz, xy, xz, yz] over the slices.
  const S = new Array(10).fill(0), h = (hi - lo) / slices;
  for (let i = 0; i <= slices; i++) {
    const z = lo + i * h, w = (i === 0 || i === slices ? 1 : i % 2 ? 4 : 2) * h / 3;
    const [A, X, Y, XX, YY, XY] = at(z);
    const f = [A, X, Y, z * A, XX, YY, z * z * A, XY, z * X, z * Y];
    for (let k = 0; k < 10; k++) S[k] += w * f[k];
  }
  const V = S[0], c = [S[1] / V, S[2] / V, S[3] / V];
  const Cxx = S[4] - V * c[0] * c[0], Cyy = S[5] - V * c[1] * c[1], Czz = S[6] - V * c[2] * c[2];
  const Cxy = S[7] - V * c[0] * c[1], Cxz = S[8] - V * c[0] * c[2], Cyz = S[9] - V * c[1] * c[2];
  return { V, c, I: [[Cyy + Czz, -Cxy, -Cxz], [-Cxy, Cxx + Czz, -Cyz], [-Cxz, -Cyz, Cxx + Cyy]] };
}

/** The eigenvalues of a symmetric 3x3 matrix, rising, in closed form (Smith, 1961). */
function eigenvalues(M) {
  const q = (M[0][0] + M[1][1] + M[2][2]) / 3;
  const p1 = M[0][1] ** 2 + M[0][2] ** 2 + M[1][2] ** 2;
  const p2 = (M[0][0] - q) ** 2 + (M[1][1] - q) ** 2 + (M[2][2] - q) ** 2 + 2 * p1, p = Math.sqrt(p2 / 6);
  const B = M.map((row, i) => row.map((x, j) => (x - (i === j ? q : 0)) / p));
  const det = B[0][0] * (B[1][1] * B[2][2] - B[1][2] * B[2][1]) - B[0][1] * (B[1][0] * B[2][2] - B[1][2] * B[2][0]) + B[0][2] * (B[1][0] * B[2][1] - B[1][1] * B[2][0]);
  const phi = Math.acos(Math.max(-1, Math.min(1, det / 2))) / 3;
  const e1 = q + 2 * p * Math.cos(phi), e3 = q + 2 * p * Math.cos(phi + 2 * Math.PI / 3);
  return [e3, 3 * q - e1 - e3, e1];
}

test("an irregular hull\'s mass, centre and principal moments match integration across slices to 0.1%", () => {
  const rnd = numbers(17), points = [];
  for (let i = 0; i < 40; i++) points.push(rnd() * 1.2 - 0.3, rnd() * 0.8 + 0.1, rnd() * 1.6 - 1);
  const world = new World();
  const body = world.addBody({ shape: 'hull', vertices: points, density: 1 });
  const want = sliced(build(points), 4000);
  near(body.mass, want.V, 1e-3 * want.V, 'mass');
  const size = Math.cbrt(want.V);
  for (let k = 0; k < 3; k++) near(body.centerOfMass[k], want.c[k], 1e-3 * size, `centre ${k}`);
  const got = [...body.inertia].sort((a, b) => a - b), moments = eigenvalues(want.I);
  for (let k = 0; k < 3; k++) near(got[k], moments[k], 1e-3 * moments[k], `moment ${k}`);
});

test('a compound of two boxes has their masses summed, the weighted centre, and the parallel-axis inertia', () => {
  const world = new World();
  const c = world.addBody({ shape: 'compound', parts: [{ size: [1, 1, 1] }, { size: [1, 1, 1], position: [2, 0, 0], density: 3000 }] });
  near(c.mass, 4000, 1e-9, 'mass');
  near(c.centerOfMass[0], 1.5, 1e-14, 'centre x');
  // About y: each box's own m/6, and m d² from the shared centre, 1.5 and 0.5 away.
  const iy = 1000 / 6 + 1000 * 1.5 * 1.5 + 3000 / 6 + 3000 * 0.5 * 0.5;
  const ix = 4000 / 6;
  const got = [...c.inertia].sort((a, b) => a - b);
  near(got[0], ix, 1e-9, 'about x');
  near(got[1], iy, 1e-9, 'about y');
  near(got[2], iy, 1e-9, 'about z');
  // A mass given scales every part to it.
  world.setBody(c, { mass: 400 });
  near(c.mass, 400, 1e-12, 'given mass');
  near(c.centerOfMass[0], 1.5, 1e-14, 'centre, scaled');
  near([...c.inertia].sort((a, b) => a - b)[1], iy / 10, 1e-9, 'inertia, scaled');
});

/** A stack of ten of `make`'s bodies on a floor, after 10 s: how far any strayed, and sank. */
function stack(make) {
  const world = new World();
  world.addBody({ type: 'static', size: [20, 1, 20], position: [0, -0.5, 0] });
  const bodies = [];
  for (let i = 0; i < 10; i++) bodies.push(world.addBody({ ...make(), position: [0, 0.5 + i, 0] }));
  for (let n = 0; n < 600; n++) world.step(DT);
  return {
    strayed: Math.max(...bodies.map((b) => Math.hypot(b.position[0], b.position[2]))),
    sank: Math.max(...bodies.map((b, i) => 0.5 + i - b.position[1])),
    asleep: bodies.every((b) => b.asleep),
  };
}

test('a stack of 10 cubes made as hulls stands as 10 boxes do', () => {
  const boxes = stack(() => ({})), hulls = stack(() => ({ shape: 'hull', vertices: corners(1, 1, 1) }));
  assert.ok(hulls.asleep && boxes.asleep);
  near(hulls.strayed, boxes.strayed, 1e-4, 'strayed');
  near(hulls.sank, boxes.sank, 1e-4, 'sank');
  assert.ok(hulls.strayed < 0.002 && hulls.sank < 0.01, `${hulls.strayed} strayed, ${hulls.sank} sank`);
});

test('a hull on a slope holds while tan θ < friction and slides at g (sin θ - μ cos θ) when steeper', () => {
  for (const degrees of [20, 35]) {
    const theta = degrees * Math.PI / 180, mu = 0.5;
    const q = [0, 0, Math.sin(theta / 2), Math.cos(theta / 2)];
    const world = new World();
    world.addBody({ type: 'static', size: [40, 1, 4], rotation: q, position: [0, -0.5, 0] });
    // Rest on the slope: 0.5 out along its normal.
    const n = [-Math.sin(theta), Math.cos(theta), 0];
    const body = world.addBody({ shape: 'hull', vertices: corners(1, 1, 1), rotation: q, position: n.map((x) => x * 0.5), friction: mu });
    for (let k = 0; k < 30; k++) world.step(DT);
    const v0 = Math.hypot(...body.velocity);
    for (let k = 0; k < 60; k++) world.step(DT);
    const a = (Math.hypot(...body.velocity) - v0) / 1;
    const want = Math.max(0, 9.81 * (Math.sin(theta) - mu * Math.cos(theta)));
    near(a, want, 0.02 * 9.81, `${degrees}°`);
  }
});

test('a cube made as a hull stands on its edge within its tolerance of balance, and tips past it, as a box does', () => {
  // contact.test.js's box: the edge's patch holds it up to 0.41° past balance.
  const deg = Math.PI / 180;
  for (const [past, stands] of [[0.3, true], [0.6, false]]) {
    const world = new World({ sleeping: false });
    world.addBody({ type: 'static', size: [10, 1, 10], position: [0, -0.5, 0] });
    const a = (45 + past) * deg, r = Math.SQRT1_2;
    const cube = world.addBody({ shape: 'hull', vertices: corners(1, 1, 1), position: [-r * Math.sin(a - 45 * deg), r * Math.cos(a - 45 * deg), 0], rotation: [0, 0, Math.sin(a / 2), Math.cos(a / 2)] });
    for (let n = 0; n < 600; n++) world.step(DT);
    assert.equal(cube.position[1] > 0.6, stands, `${past}° past balance`);
  }
});

test('a rock lying across another rock\'s edge comes to rest: the four points kept hold its weight up', () => {
  // Pairs of rocks from heaps on terrain (x8sleep's, seeds 5 and 12, at
  // 60 s), the lower made static. The upper touches the lower on five or more
  // points, its weight over one the four kept could leave out (collide.js
  // reducePoints, HELD UP): it sank there until that point came back, up to
  // a millimetre in, was pushed out, and tipped it back, 0.2 to 0.9 J a cycle,
  // for good. Each pose runs twice: as it comes, and with the pair's kept
  // manifold dropped every step (_pairEpoch, as when a body is changed), so
  // every step's contact is found afresh, as a moving pair's is; choosing
  // pressed points first held the second only with the kept manifold.
  // Measured: asleep in 1.0 to 2.5 s, each where it landed.
  const rnd = numbers(3), rock = [];
  for (let k = 0; k < 12; k++) rock.push(rnd() - 0.5, (rnd() - 0.5) * 0.8, rnd() - 0.5);
  const poses = [
    { lower: [-0.01, 0.22, 0.42, 0.88], position: [0.32, 0.45, -0.28], rotation: [-0.1, -0.03, -0.11, -0.99], rests: 0.29 },
    { lower: [0.96, -0.06, 0.28, 0.04], position: [0.21, 0.53, -0.27], rotation: [0.05, 0.41, -0.22, 0.89], rests: 0.44 },
  ];
  for (const [i, pose] of poses.entries()) {
    for (const afresh of [false, true]) {
      const world = new World();
      world.addBody({ type: 'static', shape: 'hull', vertices: rock, rotation: pose.lower });
      const top = world.addBody({ shape: 'hull', vertices: rock, position: pose.position, rotation: pose.rotation });
      for (let n = 0; n < 5 * 60 && !top.asleep; n++) {
        if (afresh) world._pairEpoch++;
        world.step(DT);
      }
      assert.ok(top.asleep, `pose ${i}${afresh ? ' found afresh' : ''}: still moving at ${Math.hypot(...top.velocity)} m/s`);
      // On the lower rock, where it landed a few millimetres below where it started.
      near(top.centerOfMass[1], pose.rests, 0.01, `pose ${i}: its centre's height`);
    }
  }
});

test('a rock lying face to face on another, its weight beside a line of pressed points, comes to rest', () => {
  // A pair from x8sleep's heightfield heap, seed 14, at 60 s, the lower made
  // static, the upper moving as it was. They meet face to face on six points:
  // three pressed nearly in a line, the middle one 19 mm off it, and the
  // weight line 3 mm off it on the middle one's side. The four kept left out
  // whichever of the middle and the outer pressed point rounding made the
  // shallower, so the rock tipped onto the middle one, then was pushed off it
  // when the four came back to it, for good (collide.js reducePoints, UNDER).
  // Sleeping is off: in a heap the rocking kept the island awake, but alone
  // it stays within its sleep tolerance. Measured: the old choice rocked it
  // at 4 to 6 mm/s through 20 s, kept or found afresh; with the four started
  // from the pressed point nearest the weight line it lies on four pressed
  // points and is below 1e-11 m/s from 3 s.
  const rnd = numbers(3), rock = [];
  for (let k = 0; k < 12; k++) rock.push(rnd() - 0.5, (rnd() - 0.5) * 0.8, rnd() - 0.5);
  for (const afresh of [false, true]) {
    const world = new World({ sleeping: false });
    world.addBody({ type: 'static', shape: 'hull', vertices: rock, rotation: [-0.0486, -0.3821, 0.1331, 0.9132] });
    const top = world.addBody({
      shape: 'hull', vertices: rock, position: [0.4745, 0.2492, -0.0558], rotation: [0.6743, 0.1476, 0.505, -0.5182],
      velocity: [-0.0035, -0.0002, 0.0109], angularVelocity: [0.0383, -0.0091, 0.0124],
    });
    let fastest = 0;
    for (let n = 0; n < 5 * 60; n++) {
      if (afresh) world._pairEpoch++;
      world.step(DT);
      if (n >= 4 * 60) fastest = Math.max(fastest, Math.hypot(...top.velocity));
    }
    assert.ok(fastest < 1e-6, `${afresh ? 'found afresh' : 'kept'}: still moving at up to ${fastest} m/s in its fifth second`);
  }
});

test('a hull point keeps its id whichever face is the reference', () => {
  // collide.test.js's forearm under a thigh, the thigh made as a hull, and
  // the forearm as a hull, then as a box (a box against a hull is a hull of
  // its corners). Tilted 1 mrad, the forearm's face is the reference; tilted
  // 3 mrad, the thigh's. The same four points come out and each keeps its id
  // (named() in collide.js), so its warm start carries over. Named by
  // reference and incident, all four changed: in ragdolls made of hulls a
  // thigh on its forearm, and a chest flat on the floor, swapped every few
  // steps, and the chest buzzed at 5 cm/s for 16 s.
  for (const box of [false, true]) {
    const a = new Shape();
    if (box) { a.kind = BOX; a.half.set([0.13, 0.04, 0.04]); } else { a.kind = HULL; a.hull = build(corners(0.26, 0.08, 0.08)); }
    a.place(0, 0, 0, [0, 0, 0, 1]);
    const at = (t) => {
      const b = new Shape(), m = new Manifold();
      b.kind = HULL; b.hull = build(corners(0.14, 0.14, 0.44));
      b.place(0.1, 0.108, 0.01, [Math.sin(t / 2), 0, 0, Math.cos(t / 2)]);
      collide(a, b, 0.01, 0.0004, m);
      return m;
    };
    const r = at(0.001), s = at(0.003), what = box ? 'box' : 'hull';
    assert.equal(r.normal[2], 0, `${what}: the forearm's face`);
    assert.ok(s.normal[2] > 0.002, `${what}: the thigh's face, normal ${s.normal}`);
    assert.equal(r.count, 4);
    assert.equal(s.count, 4);
    for (let k = 0; k < 4; k++) {
      const j = [...s.ids].indexOf(r.ids[k]);
      assert.ok(j >= 0 && j < 4, `${what}: point ${k}'s id ${r.ids[k]} is gone: ${[...s.ids]}`);
      for (let i = 0; i < 3; i++) near(s.points[3 * j + i], r.points[3 * k + i], 0.001, `${what}: point ${k}[${i}]`);
    }
  }
});

test('a pile of irregular hulls lands on a box, a ball, a capsule, a mesh and a heightfield, and sleeps', () => {
  const rnd = numbers(3), world = new World();
  world.addBody({ type: 'static', size: [10, 1, 10], position: [-5, -0.5, 0] });
  world.addBody({ shape: 'mesh', vertices: [0, 0, -5, 10, 0, -5, 10, 0, 5, 0, 0, 5], indices: [0, 2, 1, 0, 3, 2] });
  world.addBody({ shape: 'heightfield', heights: new Array(25).fill(0), columns: 5, spacing: 5, position: [0, 0, 15] });
  world.addBody({ type: 'static', shape: 'sphere', radius: 1, position: [-3, 0, 0] });
  world.addBody({ type: 'static', shape: 'capsule', radius: 0.5, height: 4, position: [4, 0.5, 0], rotation: [0, 0, Math.SQRT1_2, Math.SQRT1_2] });
  const bodies = [];
  for (let i = 0; i < 48; i++) {
    const pts = [];
    for (let k = 0; k < 12; k++) pts.push(rnd() - 0.5, (rnd() - 0.5) * 0.8, rnd() - 0.5);
    bodies.push(world.addBody({ shape: 'hull', vertices: pts, position: [(i % 8) * 1.2 - 5, 2 + Math.floor(i / 16) * 1.2, (Math.floor(i / 8) % 2) * 15 + rnd()], rotation: [rnd(), rnd(), rnd(), rnd()] }));
  }
  for (let n = 0; n < 20 * 60; n++) world.step(DT);
  assert.ok(bodies.every((b) => b.asleep), `${bodies.filter((b) => b.asleep).length} of 48 asleep`);
  assert.ok(bodies.every((b) => b.centerOfMass[1] > 0), 'one fell through');
});

test('a hammer thrown in zero gravity turns about its centre of mass, which flies straight, and keeps its momentum exactly', () => {
  const world = new World({ gravity: [0, 0, 0] });
  const hammer = world.addBody({
    shape: 'compound',
    parts: [{ shape: 'capsule', radius: 0.02, height: 0.3, density: 700 }, { size: [0.12, 0.04, 0.04], position: [0, 0.16, 0], density: 7800 }],
    velocity: [1, 0, 0], angularVelocity: [3, 1, 20],
  });
  const c0 = hammer.centerOfMass;
  const energy = () => {
    // Its angular momentum, from the world's own state: I ω in the principal frame.
    const w = hammer.angularVelocity, I = hammer.inertia, q = world._rotation.subarray(0, 4);
    const [x, y, z, s] = q;
    const R = [[1 - 2 * (y * y + z * z), 2 * (x * y - s * z), 2 * (x * z + s * y)], [2 * (x * y + s * z), 1 - 2 * (x * x + z * z), 2 * (y * z - s * x)], [2 * (x * z - s * y), 2 * (y * z + s * x), 1 - 2 * (x * x + y * y)]];
    const local = [0, 1, 2].map((k) => R[0][k] * w[0] + R[1][k] * w[1] + R[2][k] * w[2]);
    const L = [0, 1, 2].map((k) => I[k] * local[k]);
    return { L: [0, 1, 2].map((k) => R[k][0] * L[0] + R[k][1] * L[1] + R[k][2] * L[2]) };
  };
  const start = energy();
  for (let n = 1; n <= 240; n++) {
    world.step(DT);
    const c = hammer.centerOfMass, now = energy();
    near(Math.hypot(c[0] - (c0[0] + n * DT), c[1] - c0[1], c[2] - c0[2]), 0, 1e-12, `centre off its line at step ${n}`);
    for (let k = 0; k < 3; k++) near(now.L[k], start.L[k], 1e-9 * Math.hypot(...start.L), `angular momentum ${k} at step ${n}`);
  }
  // The origin, at the handle's middle, swings round the centre of mass as the hammer turns.
  const off = hammer.position.map((x, k) => x - hammer.centerOfMass[k]);
  near(Math.hypot(...off), Math.hypot(c0[0], c0[1], c0[2]), 1e-12, 'origin to centre');
});

test('an L-shaped compound stands while its centre of mass is over its foot, and tips over once past it', () => {
  // A 0.2 m square leg, 1 m tall, with an arm from its top: the arm's length moves the centre past the foot's edge at x = 0.1.
  const fall = (arm) => {
    const world = new World();
    world.addBody({ type: 'static', size: [10, 1, 10], position: [0, -0.5, 0] });
    const body = world.addBody({ shape: 'compound', parts: [{ size: [0.2, 1, 0.2] }, { size: [arm, 0.2, 0.2], position: [0.1 + arm / 2, 0.4, 0] }], position: [0, 0.5, 0] });
    for (let n = 0; n < 300; n++) world.step(DT);
    return { x: body.centerOfMass[0], fell: Math.abs(body.rotation[2]) > 0.2 };
  };
  // Centre of mass 5 mm inside the edge, then 5 mm past it: L (0.1 + L/2) = c (1 + L).
  assert.equal(fall(0.43092).fell, false);
  assert.equal(fall(0.46328).fell, true);
});

test('a lopsided compound tumbles as a box with the same inertia does', () => {
  // Three bars along x, y and z of different lengths, centred: its principal axes are its own.
  const world = new World({ gravity: [0, 0, 0] });
  const jack = world.addBody({ shape: 'compound', parts: [{ size: [3, 0.2, 0.2] }, { size: [0.2, 2, 0.2] }, { size: [0.2, 0.2, 1.2] }],
    angularVelocity: [0.04, 4, 0.04] });
  const box = world.addBody({ mass: jack.mass, inertia: [...jack.inertia], position: [10, 0, 0], angularVelocity: [0.04, 4, 0.04] });
  // y is its middle axis: spun about it, it flips over, and back, while its spin in the world stays put.
  let flipped = false;
  for (let n = 0; n < 1200; n++) {
    world.step(DT);
    // Its own y axis turned to point down: 1 - 2 (x² + z²) of its rotation.
    const [x, , z] = jack.rotation;
    if (1 - 2 * (x * x + z * z) < -0.9) flipped = true;
    for (let k = 0; k < 3; k++) near(jack.angularVelocity[k], box.angularVelocity[k], 1e-9, `ω ${k} at step ${n}`);
  }
  assert.ok(flipped, 'it never flipped');
});

test('a dumbbell of two balls rolls on them as a ball rolls, without hopping', () => {
  const world = new World();
  world.addBody({ type: 'static', size: [40, 1, 4], position: [0, -0.5, 0] });
  const bell = world.addBody({ shape: 'compound', parts: [{ shape: 'sphere', radius: 0.15, position: [0, 0, -0.3] }, { shape: 'sphere', radius: 0.15, position: [0, 0, 0.3] }],
    velocity: [2, 0, 0], angularVelocity: [0, 0, -2 / 0.15], position: [-10, 0.15, 0] });
  let high = 0;
  for (let n = 0; n < 300; n++) { world.step(DT); high = Math.max(high, Math.abs(bell.position[1] - 0.15)); }
  assert.ok(high < 1e-4, `hopped ${high} m`);
  // Its moment about the axis it rolls on is a ball's, 2/5 m r², so it slows as a ball does: 0.01 g / 1.4.
  near(bell.velocity[0], 2 - 0.01 * 9.81 / 1.4 * 5, 0.02, 'speed after 5 s');
});

test('position and transforms stay at the origin; centerOfMass moves the turn, and setBody keeps the body where it is', () => {
  const world = new World({ gravity: [0, 0, 0] });
  const rock = world.addBody({ shape: 'hull', vertices: [0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0, 2], position: [5, 1, 0], rotation: [0, 0.3826834, 0, 0.9238795] });
  for (let k = 0; k < 3; k++) near(world.transforms[rock.slot * 7 + k], rock.position[k], 1e-6, `transforms ${k}`);
  assert.deepEqual([...rock.position], [5, 1, 0]);
  near(rock.rotation[1], 0.3826834, 1e-7, 'rotation');
  // Its centre, at the corners' mean, turned 45° about y with it.
  near(rock.centerOfMass[1], 1.5, 1e-12, 'centre y');
  rock.setPosition(0, 0, 0);
  assert.deepEqual([...rock.position].map((x) => Math.abs(x) < 1e-15 ? 0 : x), [0, 0, 0]);
  rock.setRotation([0, 0, 0, 1]);
  for (let k = 0; k < 3; k++) near(rock.centerOfMass[k], 0.5, 1e-12, `centre after turning back ${k}`);
  // Reshaped: a new centre, the same origin.
  world.setBody(rock, { vertices: corners(1, 1, 1) });
  for (let k = 0; k < 3; k++) near(rock.centerOfMass[k], 0, 1e-12, `centre of a cube ${k}`);
  for (let k = 0; k < 3; k++) near(rock.position[k], 0, 1e-12, `origin ${k}`);
  // A box given a centre of mass: still drawn where it is, turning about the new point.
  const crate = world.addBody({ position: [0, 5, 0], centerOfMass: [0.5, 0, 0], angularVelocity: [0, 0, 1] });
  near(crate.centerOfMass[0], 0.5, 1e-15, 'given centre');
  assert.deepEqual([...crate.position], [0, 5, 0]);
  for (let n = 0; n < 60; n++) world.step(DT);
  near(crate.centerOfMass[0], 0.5, 1e-12, 'the centre stays put');
  near(Math.hypot(crate.position[0] - 0.5, crate.position[1] - 5), 0.5, 1e-12, 'the origin swings round it');
});

test('a tall box tipped 35° falls over, and rights itself with its centre of mass lowered', () => {
  const tip = (centerOfMass) => {
    const world = new World();
    world.addBody({ type: 'static', size: [10, 1, 10], position: [0, -0.5, 0] });
    const a = 35 * Math.PI / 180;
    // Resting on its bottom edge at x = -0.5, tipped about it.
    const body = world.addBody({ size: [1, 2, 1], centerOfMass, rotation: [0, 0, Math.sin(a / 2), Math.cos(a / 2)],
      position: [-0.5 + 0.5 * Math.cos(a) - Math.sin(a), 0.5 * Math.sin(a) + Math.cos(a) + 1e-4, 0] });
    for (let n = 0; n < 300; n++) world.step(DT);
    return 2 * Math.atan2(body.rotation[2], body.rotation[3]) * 180 / Math.PI;
  };
  assert.ok(Math.abs(Math.abs(tip(null)) - 90) < 2, `fell to ${tip(null)}°`);
  assert.ok(Math.abs(tip([0, -0.8, 0])) < 2, `righted to ${tip([0, -0.8, 0])}°`);
});

test("bodyOf gives a hull\'s corners and a compound\'s parts back, and addBody makes the same body from them", () => {
  const world = new World();
  const rnd = numbers(23), pts = [];
  for (let i = 0; i < 30; i++) pts.push(rnd(), rnd(), rnd());
  for (const options of [
    { shape: 'hull', vertices: pts, density: 500, position: [1, 2, 3] },
    { shape: 'compound', parts: [{ shape: 'sphere', radius: 0.3, density: 2000 }, { shape: 'hull', vertices: pts, position: [1, 0, 0], rotation: [0, 0, 0.7071068, 0.7071068] }], centerOfMass: [0.2, 0, 0] },
  ]) {
    const a = world.addBody(options), b = world.addBody(world.bodyOf(a));
    assert.deepEqual(world.bodyOf(b), world.bodyOf(a));
    assert.equal(b.mass, a.mass);
    assert.deepEqual([...b.inertia], [...a.inertia]);
    assert.deepEqual([...b.centerOfMass], [...a.centerOfMass]);
  }
  assert.equal(world.bodyOf(world.addBody({ shape: 'hull', vertices: [...corners(1, 1, 1), 0, 0, 0] })).vertices.length, 24);
});

test('rays, shape casts and overlaps find hulls and every part of a compound', () => {
  const world = new World();
  world.addBody({ type: 'static', shape: 'hull', vertices: corners(2, 2, 2), position: [0, 0, 0] });
  world.addBody({ type: 'static', shape: 'compound', parts: [{ size: [1, 1, 1] }, { shape: 'sphere', radius: 0.5, position: [0, 0, 6] }], position: [10, 0, 0] });
  // As a box of the same size would be: from the side, at 4 m, normal back along the ray.
  const hit = world.raycast([-5, 0.3, 0.2], [1, 0, 0]);
  near(hit.distance, 4, 1e-12, 'ray to the hull');
  assert.deepEqual(hit.normal, [-1, 0, 0]);
  near(world.raycast([0.2, 10, 0.1], [0, -1, 0]).distance, 9, 1e-12, 'from above');
  assert.equal(world.raycast([-5, 3, 0], [1, 0, 0]), null);
  // The compound's ball, past its box.
  near(world.raycast([10, 5, 6], [0, -1, 0]).distance, 4.5, 1e-12, 'ray to the ball part');
  const cast = world.shapeCast({ shape: 'sphere', radius: 0.25 }, [10, 0, 10], [0, 0, -1]);
  near(cast.distance, 3.25, 1e-3, 'cast to the ball part');
  assert.equal(world.overlap({ shape: 'box', size: [0.2, 0.2, 0.2] }, [10, 0, 6.6]).length, 1);
  assert.equal(world.overlap({ shape: 'box', size: [0.2, 0.2, 0.2] }, [10, 0, 3]).length, 0);
});

test('a compound sensor sees a ball pass through its far part', () => {
  const world = new World({ gravity: [0, 0, 0] });
  const zone = world.addBody({ type: 'static', sensor: true, shape: 'compound', parts: [{ size: [1, 1, 1] }, { size: [1, 1, 1], position: [5, 0, 0] }] });
  const ball = world.addBody({ shape: 'sphere', radius: 0.1, position: [5, 3, 0], velocity: [0, -2, 0] });
  let seen = false;
  for (let n = 0; n < 120; n++) {
    world.step(DT);
    if (world.touching(zone).some((b) => b.id === ball.id)) seen = true;
  }
  assert.ok(seen);
});

test('a 10 cm hull rod at 200 m/s stops on a 2 cm wall, wherever in a step it reaches it', () => {
  // 50 starts spread over a whole step's travel (3.3 m), so the wall comes at every point of a step.
  for (let k = 0; k < 50; k++) {
    const world = new World({ gravity: [0, 0, 0] });
    world.addBody({ type: 'static', size: [0.02, 4, 4], position: [0, 0, 0] });
    const rod = world.addBody({ shape: 'hull', vertices: corners(0.1, 0.02, 0.02), position: [-3 - k * 0.0731, 0, 0], velocity: [200, 0, 0] });
    for (let n = 0; n < 30; n++) world.step(DT);
    assert.ok(rod.centerOfMass[0] < 0, `start ${k}: through`);
  }
});

test('off means off: a world of boxes, balls and capsules has no frames to convert', () => {
  const world = new World();
  world.addBody({ type: 'static', size: [10, 1, 10], position: [0, -0.5, 0] });
  for (const shape of ['box', 'sphere', 'capsule']) world.addBody({ shape, position: [0, 2, 0] });
  for (let n = 0; n < 30; n++) world.step(DT);
  assert.equal(world._framedCount, 0);
  assert.ok(world._inner.every((g) => g === null));
  world.addBody({ shape: 'hull' });
  assert.equal(world._framedCount, 1);
});

test('overlapping hulls, and hulls and boxes, always touch, at least as deep as they overlap', () => {
  // Separating axes give the true depth: the least overlap over every face
  // normal and every cross product of two edges. A face kept on a tie with a
  // better axis used to clip to nothing, or to points above the other hull's
  // deepest corner, and the incident face, the one most against the normal
  // of all, wasn't always at that corner: rocks in a heap went whole steps
  // with no contact, 1 to 4 cm into each other, and never slept.
  const rnd = numbers(7), m = new Manifold(), P = new Placed(), Q = new Placed();
  const rock = () => {
    const pts = [];
    for (let k = 0; k < 12; k++) pts.push(rnd() - 0.5, (rnd() - 0.5) * 0.8, rnd() - 0.5);
    const h = build(pts), v = h.vertices, c = [0, 0, 0];
    for (let i = 0; i < v.length; i++) c[i % 3] += v[i] / (v.length / 3);
    return build(Array.from(v, (x, i) => x - c[i % 3]));
  };
  const turned = () => {
    const q = [rnd() - 0.5, rnd() - 0.5, rnd() - 0.5, rnd() - 0.5], l = Math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2);
    return q.map((x) => x / l);
  };
  const along = (x, y, z) => {
    let hi = -Infinity, lo = Infinity;
    for (let i = 0; i < P.hull.vertices.length; i += 3) hi = Math.max(hi, P.v[i] * x + P.v[i + 1] * y + P.v[i + 2] * z);
    for (let i = 0; i < Q.hull.vertices.length; i += 3) lo = Math.min(lo, Q.v[i] * x + Q.v[i + 1] * y + Q.v[i + 2] * z);
    return hi - lo;
  };
  const depth = () => {
    let least = Infinity;
    const axis = (x, y, z) => {
      const l = Math.sqrt(x * x + y * y + z * z);
      if (l > 1e-9) least = Math.min(least, along(x / l, y / l, z / l), along(-x / l, -y / l, -z / l));
    };
    for (const X of [P, Q]) for (let f = 0; f < X.hull.planes.length; f += 4) axis(X.planes[f], X.planes[f + 1], X.planes[f + 2]);
    const ea = P.hull.edges, eb = Q.hull.edges;
    for (let i = 0; i < ea.length; i += 2) {
      const u = 3 * ea[i], w = 3 * ea[i + 1], ux = P.v[w] - P.v[u], uy = P.v[w + 1] - P.v[u + 1], uz = P.v[w + 2] - P.v[u + 2];
      for (let j = 0; j < eb.length; j += 2) {
        const p = 3 * eb[j], q = 3 * eb[j + 1], tx = Q.v[q] - Q.v[p], ty = Q.v[q + 1] - Q.v[p + 1], tz = Q.v[q + 2] - Q.v[p + 2];
        axis(uy * tz - uz * ty, uz * tx - ux * tz, ux * ty - uy * tx);
      }
    }
    return least;
  };
  let tried = 0;
  for (let k = 0; k < 6000; k++) {
    const a = new Shape(), b = new Shape();
    a.kind = HULL; a.hull = rock();
    if (k % 3 === 0) {
      // Resting a corner into a floor, as on the ground under a heap.
      b.kind = BOX; b.half.set([2, 0.25, 2]); b.place(0, 0, 0, [0, 0, 0, 1]);
      const q = turned();
      a.place(0, 0, 0, q);
      Q.set(a);
      let low = Infinity;
      for (let i = 1; i < Q.hull.vertices.length; i += 3) low = Math.min(low, Q.v[i]);
      a.place(rnd() - 0.5, 0.25 - low - rnd() * 0.05, rnd() - 0.5, q);
    } else {
      if (k % 3 === 1) { b.kind = BOX; b.half.set([0.45, 0.3, 0.35]); } else { b.kind = HULL; b.hull = rock(); }
      const r = 0.5 + rnd() * 0.4, t = [rnd() - 0.5, rnd() - 0.5, rnd() - 0.5], l = Math.sqrt(t[0] ** 2 + t[1] ** 2 + t[2] ** 2);
      a.place(0, 0, 0, turned());
      b.place(t[0] / l * r, t[1] / l * r, t[2] / l * r, turned());
    }
    P.set(a); Q.set(b);
    const D = depth(), tolerance = 0.002;
    if (!(D > tolerance && D < 0.1)) continue;
    tried++;
    const n = collide(a, b, tolerance, tolerance, m);
    assert.ok(n > 0, `pose ${k}: ${D} m into each other and no contact`);
    const deepest = Math.min(...m.separations.subarray(0, n));
    assert.ok(deepest <= -D + tolerance, `pose ${k}: ${D} m into each other, deepest point ${deepest}`);
    assert.ok(along(m.normal[0], m.normal[1], m.normal[2]) <= D + tolerance + 1e-12, `pose ${k}: pushed apart the long way`);
  }
  assert.ok(tried > 2000, `${tried} poses overlapped`);
});

test('hulls and compounds give the same result twice, to the bit', () => {
  const run = () => {
    const rnd = numbers(41), world = new World();
    world.addBody({ type: 'static', size: [10, 1, 10], position: [0, -0.5, 0] });
    for (let i = 0; i < 12; i++) {
      const pts = [];
      for (let k = 0; k < 10; k++) pts.push(rnd() - 0.5, rnd() - 0.5, rnd() - 0.5);
      world.addBody(i % 2 ? { shape: 'hull', vertices: pts, position: [i * 0.3 - 2, 1 + i * 0.6, 0] }
        : { shape: 'compound', parts: [{ size: [0.4, 0.2, 0.2] }, { shape: 'sphere', radius: 0.15, position: [0.3, 0, 0] }], position: [i * 0.3 - 2, 1 + i * 0.6, 0.2] });
    }
    for (let n = 0; n < 240; n++) world.step(DT);
    return Array.from(world.transforms);
  };
  assert.deepEqual(run(), run());
});

test('a hull at rest keeps its contact from step to step, and boxes keep none', () => {
  const next = numbers(7), rock = [];
  for (let k = 0; k < 12; k++) rock.push(next() - 0.5, (next() - 0.5) * 0.8, next() - 0.5);
  const world = new World({ sleeping: false });
  world.addBody({ type: 'static', size: [20, 1, 20], position: [0, -0.5, 0] });
  world.addBody({ shape: 'hull', vertices: rock, position: [0, 1, 0] });
  world.addBody({ position: [3, 0.5, 0] });
  for (let n = 0; n < 180; n++) world.step(DT);
  const pairs = world._pairs;
  assert.equal(pairs.cached, 1, 'the rock on the floor keeps one manifold; the box on it none');
  // Found afresh, the pose it was found at would be written again each step
  // (the resting rock still moves by rounding); kept, it stays as it was.
  const pose = Array.from(pairs.cachePose.slice(0, 7));
  for (let n = 0; n < 30; n++) world.step(DT);
  assert.deepEqual(Array.from(pairs.cachePose.slice(0, 7)), pose);
});
