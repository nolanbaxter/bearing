// How should body state be stored? Measured, not assumed.
//
// Two questions, settled together because they interact:
//
//   precision  Float64Array or Float32Array. JavaScript computes in doubles
//              either way, so float32 costs a conversion on every read and a
//              rounding on every write, and buys half the memory traffic.
//   layout     columns (one array per field, as rule 75 has it), one
//              interleaved record per body, or Box2D v3's split: a hot record
//              holding only what the contact solver touches, and a cold one
//              holding the rest. JavaScript has no SIMD to reward columns, and
//              a contact solver visits bodies in contact order, not in a sweep.
//
// The step below is the shape of a real one, with made-up but stable numbers:
// a sweep that integrates velocities and turns each local inverse inertia into
// world space, a gather over contacts that computes their effective masses, 8
// solver iterations of a normal row and two friction rows per contact point,
// and a sweep that integrates positions and rotations. The bodies are a grid
// pile, each touching its +x, +y and +z neighbours at 4 points, added in loop
// order as a user would add them.
//
// Each variant is compiled from the same template with `new Function`, so
// each gets its own function and its own type feedback. Closures from one
// factory would share feedback, see six array types at one site, and measure
// the polymorphism instead of the layout.
//
// Variants run in alternating blocks, rotating which goes first, so drift
// (thermals, the OS, the GC) lands on all of them alike. The f64 variants must
// agree bit for bit, as must the f32 ones; a checksum proves the kernels do the
// same work in every layout.
//
// Run: node bench/storage.cjs     or open bench/storage.html as a file.
// .cjs and no imports so the page works from file://, where modules are refused.

'use strict';

// Hot fields: everything the contact solver reads or writes, 13 of them.
const HOT = ['vx', 'vy', 'vz', 'wx', 'wy', 'wz', 'im', 'i00', 'i01', 'i02', 'i11', 'i12', 'i22'];
// Cold fields: the transform, and the local inverse inertia's diagonal.
const COLD = ['px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw', 'lx', 'ly', 'lz'];

// Columns group fields the way a vec3 or quaternion column would.
// Named with a b prefix so they can't collide with the kernels' locals.
const GROUPS = {
  bv: ['vx', 'vy', 'vz'], bw: ['wx', 'wy', 'wz'], bim: ['im'],
  biw: ['i00', 'i01', 'i02', 'i11', 'i12', 'i22'],
  bp: ['px', 'py', 'pz'], bq: ['qx', 'qy', 'qz', 'qw'], bl: ['lx', 'ly', 'lz'],
};

const LAYOUTS = {
  columns: {
    arrays: (T, n) => Object.fromEntries(Object.entries(GROUPS).map(([g, f]) => [g, new T(n * f.length)])),
    prelude: (b) => `const ${b}1 = ${b}, ${b}3 = ${b} * 3, ${b}4 = ${b} * 4, ${b}6 = ${b} * 6;`,
    at(field, b) {
      for (const [g, f] of Object.entries(GROUPS)) {
        const k = f.indexOf(field);
        if (k >= 0) return `${g}[${b}${f.length} + ${k}]`;
      }
      throw new Error(field);
    },
  },
  // 23 fields padded to 24: three 64-byte cache lines a body in f64.
  interleaved: {
    arrays: (T, n) => ({ S: new T(n * 24) }),
    prelude: (b) => `const ${b}o = ${b} * 24;`,
    at: (field, b) => `S[${b}o + ${[...HOT, ...COLD].indexOf(field)}]`,
  },
  // Hot record padded to 16: two lines in f64, exactly one in f32.
  split: {
    arrays: (T, n) => ({ H: new T(n * 16), C: new T(n * 10) }),
    prelude: (b) => `const ${b}h = ${b} * 16, ${b}c = ${b} * 10;`,
    at: (field, b) => (HOT.includes(field)
      ? `H[${b}h + ${HOT.indexOf(field)}]`
      : `C[${b}c + ${COLD.indexOf(field)}]`),
  },
};

// Contact point record, 24 numbers: normal, the two arms, two tangents,
// three effective masses, three accumulated impulses, bias, friction.
const C = {
  nx: 0, ny: 1, nz: 2, rax: 3, ray: 4, raz: 5, rbx: 6, rby: 7, rbz: 8,
  t1x: 9, t1y: 10, t1z: 11, t2x: 12, t2y: 13, t2z: 14,
  mn: 15, m1: 16, m2: 17, jn: 18, j1: 19, j2: 20, bias: 21, mu: 22,
};
const CSTRIDE = 24;
const ITERATIONS = 8;   // a typical solver's default; the same for every variant

function compile(layoutName) {
  const L = LAYOUTS[layoutName];
  const A = (f) => L.at(f, 'a');
  const B = (f) => L.at(f, 'b');
  const I = (f) => L.at(f, 'i');
  const names = Object.keys(L.arrays(Float64Array, 0));
  const unpack = `const { ${names.join(', ')} } = st.bodies; const { cp, ci } = st;`;

  // Row solve: impulse along (dx,dy,dz) with arms ra, rb, effective mass m.
  // The world inverse inertia is symmetric, stored as its upper triangle.
  const iw = (X, ax, ay, az) => [
    `${X('i00')} * ${ax} + ${X('i01')} * ${ay} + ${X('i02')} * ${az}`,
    `${X('i01')} * ${ax} + ${X('i11')} * ${ay} + ${X('i12')} * ${az}`,
    `${X('i02')} * ${ax} + ${X('i12')} * ${ay} + ${X('i22')} * ${az}`,
  ];

  const row = (d, m, j, lo, hi) => `{
      const dx = cp[o + ${C[d + 'x']}], dy = cp[o + ${C[d + 'y']}], dz = cp[o + ${C[d + 'z']}];
      const rvx = ${B('vx')} + ${B('wy')} * rbz - ${B('wz')} * rby - ${A('vx')} - ${A('wy')} * raz + ${A('wz')} * ray;
      const rvy = ${B('vy')} + ${B('wz')} * rbx - ${B('wx')} * rbz - ${A('vy')} - ${A('wz')} * rax + ${A('wx')} * raz;
      const rvz = ${B('vz')} + ${B('wx')} * rby - ${B('wy')} * rbx - ${A('vz')} - ${A('wx')} * ray + ${A('wy')} * rax;
      const vn = rvx * dx + rvy * dy + rvz * dz;
      const old = cp[o + ${C[j]}];
      let next = old - cp[o + ${C[m]}] * (vn - ${d === 'n' ? 'cp[o + ' + C.bias + ']' : '0'});
      ${lo}
      ${hi}
      cp[o + ${C[j]}] = next;
      const lam = next - old;
      const px = lam * dx, py = lam * dy, pz = lam * dz;
      const ima = ${A('im')}, imb = ${B('im')};
      ${A('vx')} -= ima * px; ${A('vy')} -= ima * py; ${A('vz')} -= ima * pz;
      ${B('vx')} += imb * px; ${B('vy')} += imb * py; ${B('vz')} += imb * pz;
      const tax = ray * pz - raz * py, tay = raz * px - rax * pz, taz = rax * py - ray * px;
      const tbx = rby * pz - rbz * py, tby = rbz * px - rbx * pz, tbz = rbx * py - rby * px;
      ${A('wx')} -= ${iw(A, 'tax', 'tay', 'taz')[0]};
      ${A('wy')} -= ${iw(A, 'tax', 'tay', 'taz')[1]};
      ${A('wz')} -= ${iw(A, 'tax', 'tay', 'taz')[2]};
      ${B('wx')} += ${iw(B, 'tbx', 'tby', 'tbz')[0]};
      ${B('wy')} += ${iw(B, 'tbx', 'tby', 'tbz')[1]};
      ${B('wz')} += ${iw(B, 'tbx', 'tby', 'tbz')[2]};
    }`;

  // Effective mass of one row: im_a + im_b + (ra x d).Ia(ra x d) + (rb x d).Ib(rb x d).
  const mass = (d, m) => `{
      const dx = cp[o + ${C[d + 'x']}], dy = cp[o + ${C[d + 'y']}], dz = cp[o + ${C[d + 'z']}];
      const cax = ray * dz - raz * dy, cay = raz * dx - rax * dz, caz = rax * dy - ray * dx;
      const cbx = rby * dz - rbz * dy, cby = rbz * dx - rbx * dz, cbz = rbx * dy - rby * dx;
      const ka = cax * (${iw(A, 'cax', 'cay', 'caz')[0]}) + cay * (${iw(A, 'cax', 'cay', 'caz')[1]}) + caz * (${iw(A, 'cax', 'cay', 'caz')[2]});
      const kb = cbx * (${iw(B, 'cbx', 'cby', 'cbz')[0]}) + cby * (${iw(B, 'cbx', 'cby', 'cbz')[1]}) + cbz * (${iw(B, 'cbx', 'cby', 'cbz')[2]});
      const k = ${A('im')} + ${B('im')} + ka + kb;
      cp[o + ${C[m]}] = k > 0 ? 1 / k : 0;
    }`;

  const arms = `const rax = cp[o + ${C.rax}], ray = cp[o + ${C.ray}], raz = cp[o + ${C.raz}];
      const rbx = cp[o + ${C.rbx}], rby = cp[o + ${C.rby}], rbz = cp[o + ${C.rbz}];`;

  const src = `
    ${unpack}
    const n = st.n, points = st.points, dt = st.dt, gy = -9.81 * dt;

    // Sweep 1: gravity, and the world inverse inertia R diag(l) R^T.
    for (let i = 0; i < n; i++) {
      ${L.prelude('i')}
      if (${I('im')} === 0) continue;
      ${I('vy')} += gy;
      const x = ${I('qx')}, y = ${I('qy')}, z = ${I('qz')}, w = ${I('qw')};
      const r00 = 1 - 2 * (y * y + z * z), r01 = 2 * (x * y - z * w), r02 = 2 * (x * z + y * w);
      const r10 = 2 * (x * y + z * w), r11 = 1 - 2 * (x * x + z * z), r12 = 2 * (y * z - x * w);
      const r20 = 2 * (x * z - y * w), r21 = 2 * (y * z + x * w), r22 = 1 - 2 * (x * x + y * y);
      const l0 = ${I('lx')}, l1 = ${I('ly')}, l2 = ${I('lz')};
      ${I('i00')} = r00 * r00 * l0 + r01 * r01 * l1 + r02 * r02 * l2;
      ${I('i01')} = r00 * r10 * l0 + r01 * r11 * l1 + r02 * r12 * l2;
      ${I('i02')} = r00 * r20 * l0 + r01 * r21 * l1 + r02 * r22 * l2;
      ${I('i11')} = r10 * r10 * l0 + r11 * r11 * l1 + r12 * r12 * l2;
      ${I('i12')} = r10 * r20 * l0 + r11 * r21 * l1 + r12 * r22 * l2;
      ${I('i22')} = r20 * r20 * l0 + r21 * r21 * l1 + r22 * r22 * l2;
    }

    // Gather: effective masses, once a step.
    for (let c = 0; c < points; c++) {
      const a = ci[2 * c], b = ci[2 * c + 1], o = c * ${CSTRIDE};
      ${L.prelude('a')} ${L.prelude('b')}
      ${arms}
      ${mass('n', 'mn')} ${mass('t1', 'm1')} ${mass('t2', 'm2')}
    }

    // Solve: gather and scatter in contact order.
    for (let it = 0; it < ${ITERATIONS}; it++) {
      for (let c = 0; c < points; c++) {
        const a = ci[2 * c], b = ci[2 * c + 1], o = c * ${CSTRIDE};
        ${L.prelude('a')} ${L.prelude('b')}
        ${arms}
        const limit = cp[o + ${C.mu}] * cp[o + ${C.jn}];
        ${row('t1', 'm1', 'j1', 'if (next < -limit) next = -limit;', 'if (next > limit) next = limit;')}
        ${row('t2', 'm2', 'j2', 'if (next < -limit) next = -limit;', 'if (next > limit) next = limit;')}
        ${row('n', 'mn', 'jn', 'if (next < 0) next = 0;', '')}
      }
    }

    // Sweep 2: positions, and rotations by q += dt/2 (w,0) q, renormalized.
    for (let i = 0; i < n; i++) {
      ${L.prelude('i')}
      if (${I('im')} === 0) continue;
      ${I('px')} += ${I('vx')} * dt; ${I('py')} += ${I('vy')} * dt; ${I('pz')} += ${I('vz')} * dt;
      const wx = ${I('wx')} * dt * 0.5, wy = ${I('wy')} * dt * 0.5, wz = ${I('wz')} * dt * 0.5;
      const x = ${I('qx')}, y = ${I('qy')}, z = ${I('qz')}, w = ${I('qw')};
      let nx = x + wx * w + wy * z - wz * y;
      let ny = y - wx * z + wy * w + wz * x;
      let nz = z + wx * y - wy * x + wz * w;
      let nw = w - wx * x - wy * y - wz * z;
      const s = 1 / Math.sqrt(nx * nx + ny * ny + nz * nz + nw * nw);
      ${I('qx')} = nx * s; ${I('qy')} = ny * s; ${I('qz')} = nz * s; ${I('qw')} = nw * s;
    }`;
  return src;
}

/** A grid pile: side^3 bodies, the bottom layer static, each touching +x, +y, +z at 4 points. */
function scene(side) {
  const n = side * side * side;
  const id = (x, y, z) => x + side * (z + side * y);
  const pairs = [];
  for (let y = 0; y < side; y++) {
    for (let z = 0; z < side; z++) {
      for (let x = 0; x < side; x++) {
        if (x + 1 < side) pairs.push([id(x, y, z), id(x + 1, y, z), 0]);
        if (y + 1 < side) pairs.push([id(x, y, z), id(x, y + 1, z), 1]);
        if (z + 1 < side) pairs.push([id(x, y, z), id(x, y, z + 1), 2]);
      }
    }
  }
  const points = pairs.length * 4;
  const body = new Float64Array(n * 23);   // canonical values, field order HOT then COLD
  for (let y = 0; y < side; y++) {
    for (let z = 0; z < side; z++) {
      for (let x = 0; x < side; x++) {
        const o = id(x, y, z) * 23;
        const dynamic = y > 0;
        body[o + 1] = -0.01 * (y % 3);          // vy: some motion so impulses are non-zero
        body[o + 3] = 0.001 * ((x + z) % 5);    // wx
        body[o + 6] = dynamic ? 1 / 1000 : 0;   // im: a 1 m water box, or static
        body[o + 13] = x; body[o + 14] = y + 0.5; body[o + 15] = z;
        body[o + 19] = 1;                       // qw
        // Local inverse inertia of a 1 m, 1000 kg box: 6 / (m (h^2 + w^2)) = 0.006.
        if (dynamic) body[o + 20] = body[o + 21] = body[o + 22] = 0.006;
      }
    }
  }
  const ci = new Int32Array(points * 2);
  const cp = new Float64Array(points * CSTRIDE);
  const axes = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  let c = 0;
  for (const [a, b, axis] of pairs) {
    const nrm = axes[axis], t1 = axes[(axis + 1) % 3], t2 = axes[(axis + 2) % 3];
    for (let k = 0; k < 4; k++) {
      const s1 = k & 1 ? 0.5 : -0.5, s2 = k & 2 ? 0.5 : -0.5;
      ci[2 * c] = a; ci[2 * c + 1] = b;
      const o = c * CSTRIDE;
      for (let j = 0; j < 3; j++) {
        cp[o + C.nx + j] = nrm[j];
        cp[o + C.rax + j] = 0.5 * nrm[j] + s1 * t1[j] + s2 * t2[j];
        cp[o + C.rbx + j] = -0.5 * nrm[j] + s1 * t1[j] + s2 * t2[j];
        cp[o + C.t1x + j] = t1[j];
        cp[o + C.t2x + j] = t2[j];
      }
      cp[o + C.bias] = 0.001;
      cp[o + C.mu] = 0.5;
      c++;
    }
  }
  return { n, points, body, ci, cp };
}

/** The canonical scene, copied into one variant's arrays. Not timed. */
function load(v, sc) {
  const fields = [...HOT, ...COLD];
  const L = LAYOUTS[v.layout];
  const fns = Object.keys(v.st.bodies);
  // A tiny generated writer keeps this simple and layout-agnostic.
  const write = new Function('st', 'body', 'n', `
    const { ${fns.join(', ')} } = st.bodies;
    for (let i = 0; i < n; i++) {
      ${L.prelude('i')}
      ${fields.map((f, k) => `${L.at(f, 'i')} = body[i * 23 + ${k}];`).join('\n')}
    }`);
  write(v.st, sc.body, sc.n);
  v.st.cp.set(sc.cp);
}

/** Body state back in canonical field order, so layouts can be compared value for value. */
function read(v) {
  const fields = [...HOT, ...COLD];
  const L = LAYOUTS[v.layout];
  const fns = Object.keys(v.st.bodies);
  const out = new Float64Array(v.st.n * 23);
  new Function('st', 'out', 'n', `
    const { ${fns.join(', ')} } = st.bodies;
    for (let i = 0; i < n; i++) {
      ${L.prelude('i')}
      ${fields.map((f, k) => `out[i * 23 + ${k}] = ${L.at(f, 'i')};`).join('\n')}
    }`)(v.st, out, v.st.n);
  return out;
}

/**
 * A denormal costs x86 a microcode assist per operation, around a hundred
 * cycles, and a NaN would make the kernels' work meaningless. Either would
 * skew the comparison, so the state is checked for both.
 */
function health(values, T) {
  const tiny = T === Float32Array ? 1.1754943508222875e-38 : 2.2250738585072014e-308;
  let bad = 0, denormal = 0;
  for (const x of values) {
    if (!Number.isFinite(x)) bad++;
    else if (x !== 0 && Math.abs(x) < tiny) denormal++;
  }
  return { bad, denormal };
}

function median(xs) {
  const s = [...xs].sort((a, b) => a - b);
  return s.length % 2 ? s[s.length >> 1] : (s[s.length / 2 - 1] + s[s.length / 2]) / 2;
}

function run(log = console.log, { sides = [10, 20, 40], rounds = 15 } = {}) {
  const now = globalThis.performance ? () => performance.now() : () => Date.now();
  const variants = [];
  for (const layout of Object.keys(LAYOUTS)) {
    for (const [precision, T] of [['f64', Float64Array], ['f32', Float32Array]]) {
      variants.push({ name: `${layout} ${precision}`, layout, precision, T, step: new Function('st', compile(layout)) });
    }
  }
  log(`Storage A/B: ${variants.length} variants, ${rounds} alternating rounds each size, ${ITERATIONS} solver iterations.`);
  log(`Runtime: ${typeof navigator !== 'undefined' ? navigator.userAgent : `Node ${process.version} on ${process.platform} ${process.arch}`}`);
  const results = {};
  for (const side of sides) {
    const sc = scene(side);
    // Enough steps a block to be a few ms at the smallest size, so timer resolution doesn't matter.
    const steps = Math.max(3, Math.round(400000 / sc.points));
    for (const v of variants) {
      v.st = { n: sc.n, points: sc.points, dt: 1 / 60, ci: sc.ci, cp: new v.T(sc.points * CSTRIDE), bodies: LAYOUTS[v.layout].arrays(v.T, sc.n) };
      v.times = [];
    }
    // Warm-up: let every variant reach its optimized code before anything is timed.
    for (const v of variants) { load(v, sc); for (let s = 0; s < steps * 3; s++) v.step(v.st); }
    for (let r = 0; r < rounds; r++) {
      for (let k = 0; k < variants.length; k++) {
        const v = variants[(k + r) % variants.length];
        load(v, sc);
        const t0 = now();
        for (let s = 0; s < steps; s++) v.step(v.st);
        v.times.push((now() - t0) / steps);
      }
    }
    // Same work: the last block's end state agrees within each precision, bit for bit.
    const states = {};
    let bad = 0, denormal = 0;
    for (const v of variants) {
      load(v, sc);
      for (let s = 0; s < steps; s++) v.step(v.st);
      states[v.name] = read(v);
      const h = health(states[v.name], v.T);
      bad += h.bad; denormal += h.denormal;
    }
    const agree = (p) => {
      const [first, ...rest] = variants.filter((v) => v.precision === p).map((v) => states[v.name]);
      return rest.every((s) => s.every((x, i) => Object.is(x, first[i])));
    };
    const base = median(variants.find((v) => v.name === 'interleaved f64').times);
    log(`\n${sc.n} bodies, ${sc.points} contact points, ${steps} steps a block`
      + `  (f64 layouts agree: ${agree('f64')}, f32 layouts agree: ${agree('f32')},`
      + ` non-finite: ${bad}, denormal: ${denormal})`);
    log('  variant            ms/step   vs interleaved f64   spread (min-max)');
    const rows = variants.map((v) => ({ name: v.name, med: median(v.times), min: Math.min(...v.times), max: Math.max(...v.times) }))
      .sort((a, b) => a.med - b.med);
    for (const x of rows) {
      log(`  ${x.name.padEnd(18)} ${x.med.toFixed(3).padStart(7)}   ${((x.med / base - 1) * 100).toFixed(1).padStart(6)}%`
        + `              ${x.min.toFixed(3)}-${x.max.toFixed(3)}`);
    }
    results[sc.n] = rows;
  }
  return results;
}

if (typeof module !== 'undefined') module.exports = { run, compile };
else globalThis.storageBench = { run };
if (typeof require !== 'undefined' && require.main === module) run();
