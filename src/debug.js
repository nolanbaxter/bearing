// Debug lines: the world as line segments, for any engine to draw.
//
// world.debugLines() returns one Float32Array, nine numbers a segment: its
// two ends, x y z and x y z, then a colour, r g b from 0 to 1. Nothing here
// draws (Bearing draws nothing): an engine draws the segments in one loop,
// Winding's engine.debug.line() or three.js's LineSegments alike.
//
// What is drawn: every body's collision shape where the step works with it
// (a hull or compound about its centre of mass, as it collides), the last
// step's contacts, joints, and, asked for, the boxes the broadphase files
// bodies in. The array is the world's own, reused and grown as needed, so a
// frame's call allocates nothing once it has grown: the next call replaces it.
//
// Nothing here runs unless asked for, and nothing is kept but the array.

import { Shape, Parts, placePart, BOX as BOX_KIND, SPHERE as SPHERE_KIND, CAPSULE as CAPSULE_KIND, HULL as HULL_KIND } from './collide.js';
import { load } from './contact.js';
import { worldAnchor, ROTATION } from './joints.js';
import { sin, cos } from './core/math.js';

const DYNAMIC = 0, STATIC = 1, KINEMATIC = 2, CHARACTER = 3;   // World's type codes
const MESH = 5;                                                // World's shape code; a heightfield is 6
const P = 4;                                                   // points a manifold holds (contact.js)

// Colours, chosen to tell kinds apart at a glance on a dark or light
// background: awake bodies bright, asleep ones dim, what can't move grey.
const AWAKE = [0.35, 0.7, 1], ASLEEP = [0.25, 0.35, 0.5], STILL = [0.6, 0.6, 0.6];
const MOVED = [0.8, 0.5, 1], WALKER = [0.4, 1, 0.5], SENSOR = [1, 0.85, 0.3];
const CONTACT = [1, 0.3, 0.25], JOINT = [1, 0.6, 0.2], FILED = [0.5, 0.5, 0.35];

/**
 * Chords to a circle: 16 keep it within 2% of its radius (1 - cos(π/16)), a
 * circle at a glance, at 16 segments a ring.
 */
const CHORDS = 16;
const RING = new Float64Array(2 * (CHORDS + 1));
for (let k = 0; k <= CHORDS; k++) {
  RING[2 * k] = cos(2 * Math.PI * k / CHORDS);
  RING[2 * k + 1] = sin(2 * Math.PI * k / CHORDS);
}

let out = null, n = 0, colour = AWAKE;
const shape = new Shape(), tri = new Float64Array(9), a3 = new Float64Array(3), b3 = new Float64Array(3);

/** Room for k more segments in the world's array, grown by doubling. */
function room(world, k) {
  const need = 9 * (n + k);
  if (need <= world._lines.length) return;
  let c = Math.max(world._lines.length, 9 * 64);
  while (c < need) c *= 2;
  const grown = new Float32Array(c);
  grown.set(world._lines.subarray(0, 9 * n));
  world._lines = grown;
  out = grown;
}

function line(x0, y0, z0, x1, y1, z1) {
  const o = 9 * n++;
  out[o] = x0; out[o + 1] = y0; out[o + 2] = z0; out[o + 3] = x1; out[o + 4] = y1; out[o + 5] = z1;
  out[o + 6] = colour[0]; out[o + 7] = colour[1]; out[o + 8] = colour[2];
}

/** The segments, as asked: { shapes, contacts, joints, boxes }, each true or false. */
export function debugLines(world, o) {
  out = world._lines;
  n = 0;
  if (o.shapes) shapes(world);
  if (o.contacts) contacts(world);
  if (o.joints) joints(world);
  if (o.boxes) boxes(world);
  const result = world._lines.subarray(0, 9 * n);
  out = null;
  return result;
}

function shapes(world) {
  const slots = world._slots, type = world._type, p = world._position, q = world._rotation;
  for (let s = 0; s < slots.top; s++) {
    if (slots.state[s] !== 1) continue;
    colour = world._sensor[s] === 1 ? SENSOR : type[s] === STATIC ? STILL : type[s] === KINEMATIC ? MOVED
      : type[s] === CHARACTER ? WALKER : world._island[s] !== 0 ? ASLEEP : AWAKE;
    if (world._shape[s] >= MESH) { level(world, s); continue; }
    const inner = world._inner[s];
    if (inner instanceof Parts) {
      for (const part of inner.list) outline(world, placePart(part, p, 3 * s, q, 4 * s, shape));
      continue;
    }
    load(world, s, shape);
    outline(world, shape.place(p[3 * s], p[3 * s + 1], p[3 * s + 2], q, 4 * s));
  }
  colour = AWAKE;
}

/** A placed shape's outline. */
function outline(world, s) {
  const c = s.center, A = s.axes, h = s.half;
  // A point of the shape's own frame, in the world's, into a3.
  const at = (x, y, z, to) => {
    to[0] = c[0] + A[0] * x + A[3] * y + A[6] * z;
    to[1] = c[1] + A[1] * x + A[4] * y + A[7] * z;
    to[2] = c[2] + A[2] * x + A[5] * y + A[8] * z;
  };
  if (s.kind === BOX_KIND) {
    room(world, 12);
    for (let axis = 0; axis < 3; axis++) {
      for (let k = 0; k < 4; k++) {
        const e = [0, 0, 0], f = [0, 0, 0], u = (axis + 1) % 3, v = (axis + 2) % 3;
        e[axis] = -h[axis]; f[axis] = h[axis];
        e[u] = f[u] = k & 1 ? h[u] : -h[u];
        e[v] = f[v] = k & 2 ? h[v] : -h[v];
        at(e[0], e[1], e[2], a3); at(f[0], f[1], f[2], b3);
        line(a3[0], a3[1], a3[2], b3[0], b3[1], b3[2]);
      }
    }
  } else if (s.kind === HULL_KIND) {
    const v = s.hull.vertices, e = s.hull.edges;
    room(world, e.length / 2);
    for (let k = 0; k < e.length; k += 2) {
      at(v[3 * e[k]], v[3 * e[k] + 1], v[3 * e[k] + 2], a3);
      at(v[3 * e[k + 1]], v[3 * e[k + 1] + 1], v[3 * e[k + 1] + 2], b3);
      line(a3[0], a3[1], a3[2], b3[0], b3[1], b3[2]);
    }
  } else if (s.kind === SPHERE_KIND) {
    // A ring in each of its own three planes.
    const r = h[0];
    room(world, 3 * CHORDS);
    for (let plane = 0; plane < 3; plane++) ring(at, plane, 0, r);
  } else if (s.kind === CAPSULE_KIND) {
    // A ring round each end of its axis, and its four sides along y.
    const r = h[0], half = h[1];
    room(world, 2 * CHORDS + 4 + 4 * CHORDS);
    ring(at, 1, -half, r);
    ring(at, 1, half, r);
    for (let k = 0; k < 4; k++) {
      const x = k === 0 ? r : k === 1 ? -r : 0, z = k === 2 ? r : k === 3 ? -r : 0;
      at(x, -half, z, a3); at(x, half, z, b3);
      line(a3[0], a3[1], a3[2], b3[0], b3[1], b3[2]);
    }
    // Its two end caps, as half rings in the xy and zy planes.
    for (const end of [-1, 1]) {
      for (let side = 0; side < 2; side++) {
        for (let k = 0; k < CHORDS / 2; k++) {
          const c0 = RING[2 * k], s0 = RING[2 * k + 1], c1 = RING[2 * k + 2], s1 = RING[2 * k + 3];
          const y0 = end * (half + r * s0), y1 = end * (half + r * s1);
          if (side === 0) { at(r * c0, y0, 0, a3); at(r * c1, y1, 0, b3); } else { at(0, y0, r * c0, a3); at(0, y1, r * c1, b3); }
          line(a3[0], a3[1], a3[2], b3[0], b3[1], b3[2]);
        }
      }
    }
  }
}

/** A ring of radius r in the plane across its own axis `plane` (0 x, 1 y, 2 z), at `offset` along that axis. */
function ring(at, plane, offset, r) {
  for (let k = 0; k < CHORDS; k++) {
    const c0 = r * RING[2 * k], s0 = r * RING[2 * k + 1], c1 = r * RING[2 * k + 2], s1 = r * RING[2 * k + 3];
    if (plane === 0) { at(offset, c0, s0, a3); at(offset, c1, s1, b3); }
    else if (plane === 1) { at(c0, offset, s0, a3); at(c1, offset, s1, b3); }
    else { at(c0, s0, offset, a3); at(c1, s1, offset, b3); }
    line(a3[0], a3[1], a3[2], b3[0], b3[1], b3[2]);
  }
}

/** A mesh's or heightfield's triangles, each edge drawn by each triangle it borders. */
function level(world, s) {
  const g = world._geometry[s], p = world._position, q = world._rotation, s3 = 3 * s, s4 = 4 * s;
  room(world, 3 * g.count);
  const x = q[s4], y = q[s4 + 1], z = q[s4 + 2], w = q[s4 + 3];
  for (let t = 0; t < g.count; t++) {
    g.triangle(t, tri);
    // Into the world: turned by the body's rotation, then moved to it.
    for (let k = 0; k < 9; k += 3) {
      const vx = tri[k], vy = tri[k + 1], vz = tri[k + 2];
      const tx = 2 * (y * vz - z * vy), ty = 2 * (z * vx - x * vz), tz = 2 * (x * vy - y * vx);
      tri[k] = p[s3] + vx + w * tx + (y * tz - z * ty);
      tri[k + 1] = p[s3 + 1] + vy + w * ty + (z * tx - x * tz);
      tri[k + 2] = p[s3 + 2] + vz + w * tz + (x * ty - y * tx);
    }
    for (let e = 0; e < 3; e++) {
      const i = 3 * e, j = 3 * ((e + 1) % 3);
      line(tri[i], tri[i + 1], tri[i + 2], tri[j], tri[j + 1], tri[j + 2]);
    }
  }
}

/**
 * The last step's contacts, where it found them: each point, and a line out
 * of it along the normal, from a towards b, a quarter of the smaller body's
 * thinnest size long: seen at the body's scale, and never longer than it.
 */
function contacts(world) {
  const c = world._contacts, size = world._size;
  colour = CONTACT;
  room(world, 4 * c.count);
  for (let m = 0; m < c.count; m++) {
    const sa = size[c.a[m]], sb = size[c.b[m]];
    const length = 0.25 * (sa > 0 && sb > 0 ? Math.min(sa, sb) : Math.max(sa, sb));
    const nx = c.normal[3 * m] * length, ny = c.normal[3 * m + 1] * length, nz = c.normal[3 * m + 2] * length;
    for (let k = 0; k < c.points[m]; k++) {
      const i = 3 * (m * P + k);
      // Midway between its two ends, as a contact found at impact keeps them apart (contact.js).
      const x = c.point[i] + 0.5 * c.shift[i], y = c.point[i + 1] + 0.5 * c.shift[i + 1], z = c.point[i + 2] + 0.5 * c.shift[i + 2];
      line(x, y, z, x + nx, y + ny, z + nz);
    }
  }
  colour = AWAKE;
}

/**
 * Each joint: a line from each body's centre to its anchor on that body. A
 * rotation joint holds no point, so has no anchor to draw: a line between its
 * bodies' centres, none to the world.
 */
function joints(world) {
  const J = world._joints, slots = J.slots, p = world._position;
  colour = JOINT;
  room(world, 2 * slots.count);
  for (let j = 0; j < slots.top; j++) {
    if (slots.state[j] !== 1) continue;
    if (J.type[j] === ROTATION) {
      const a3 = 3 * J.a[j], b3 = 3 * J.b[j];
      if (b3 >= 0) line(p[a3], p[a3 + 1], p[a3 + 2], p[b3], p[b3 + 1], p[b3 + 2]);
      continue;
    }
    const a = J.a[j], b = J.b[j], A = worldAnchor(world, j, 0), B = worldAnchor(world, j, 1);
    line(p[3 * a], p[3 * a + 1], p[3 * a + 2], A[0], A[1], A[2]);
    if (b >= 0) line(p[3 * b], p[3 * b + 1], p[3 * b + 2], B[0], B[1], B[2]);
    else line(A[0], A[1], A[2], B[0], B[1], B[2]);
  }
  colour = AWAKE;
}

/** The boxes the broadphase files bodies in: what a body has to leave before it is filed again. */
function boxes(world) {
  const bp = world._broadphase, slots = world._slots;
  colour = FILED;
  for (let s = 0; s < slots.top; s++) {
    if (slots.state[s] !== 1 || bp.which[s] < 0) continue;
    const box = bp.trees[bp.which[s]].box, o = 6 * bp.leaf[s];
    room(world, 12);
    const lo = [box[o], box[o + 1], box[o + 2]], hi = [box[o + 3], box[o + 4], box[o + 5]];
    for (let axis = 0; axis < 3; axis++) {
      for (let k = 0; k < 4; k++) {
        const e = [0, 0, 0], f = [0, 0, 0], u = (axis + 1) % 3, v = (axis + 2) % 3;
        e[axis] = lo[axis]; f[axis] = hi[axis];
        e[u] = f[u] = k & 1 ? hi[u] : lo[u];
        e[v] = f[v] = k & 2 ? hi[v] : lo[v];
        line(e[0], e[1], e[2], f[0], f[1], f[2]);
      }
    }
  }
  colour = AWAKE;
}
