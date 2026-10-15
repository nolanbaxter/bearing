// Character bodies: moved at the velocity the game gives them, as kinematic
// bodies are, but colliding as a person walking does.
//
// A dynamic body makes a poor person: friction slides it down slopes, it
// tips over, it bounces off the edge of every stair and is launched by
// ramps. A kinematic one walks through walls. So a character moves by
// casting its shape through the world before each step (query.js's sweep),
// and going only as far as it can:
//
//   out      pushed out of anything it overlaps (a platform or another
//            character moved into it)
//   carried  moved with what it stands on, by that body's velocity at its feet
//   slid     its own motion, cast and slid along what it meets: walls take
//            away the part of the motion into them; walkable ground (no
//            steeper than maxSlope) turns it to follow the slope at the same
//            horizontal speed; a low obstacle is stepped up onto, if it is no
//            taller than stepHeight
//   snapped  back down onto the ground, by up to stepHeight, if it was on the
//            ground and isn't jumping: down slopes and stairs, it stays on them
//   grounded what is under it, for world.groundOf()
//
// It is then put where it found it can go, before the solve, and stands
// there through the step as an unmoving obstacle: what lands on it or runs
// into it is held, as by a wall, and what it walked up to is touched, not
// shoved. Moved through the step as a kinematic body instead, at the speed
// that got it there, it would arrive still moving, and the solver would push
// aside a crate it only reached: measured, 600 N·s into a 1,000 kg crate, at
// 1.5 m/s. The cost: it carries nothing on its head as it walks. After the
// step its velocity reads what it did of its own accord: walking into a wall
// drops the part into the wall, landing drops the fall. Gravity acts on it
// only in the air, and "up" is against gravity.
//
// Nothing here reads input, and the game's velocity is all it is told.

import { place, sweep, deepest } from './query.js';
import { PIXEL } from './contact.js';
import { Shape, SPHERE, CAPSULE } from './collide.js';

export const CHARACTER = 3;   // World's type code

/**
 * Casts a move takes before it gives up the rest. Each slide takes one
 * direction out of the motion, and in 3D three planes leave nothing to move
 * along, so a fourth cast either finishes the move or has nothing left.
 */
const SLIDES = 4;
/** Push-outs at the start of a move, for the same reason: three directions, and one more to find it clear. */
const PUSHES = 4;

// Scratch: a step's characters move one at a time, and nothing re-enters this.
const shape = new Shape();
const hit = new Float64Array(7), deep = new Float64Array(4);
const up = new Float64Array(3), pos = new Float64Array(3), rest = new Float64Array(3), vel = new Float64Array(3);
const planes = new Float64Array(9);
const face = new Float64Array(3), probe = new Shape(), fall = new Float64Array(7);
probe.kind = SPHERE;
let tolerance = 0, slopeCos = 0;

/** Every character in the world, before the step; see the top of this file. */
export function walk(world, dt) {
  const list = world._awakeList, n = world._awakeCount, type = world._type;
  const g = world._gravity, gl = Math.sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
  // Up is against gravity; with none, the world's own up, +y.
  if (gl > 0) { up[0] = -g[0] / gl; up[1] = -g[1] / gl; up[2] = -g[2] / gl; } else { up[0] = 0; up[1] = 1; up[2] = 0; }
  for (let i = 0; i < n; i++) if (type[list[i]] === CHARACTER) move(world, list[i], dt);
}

/** After the step: each character's velocity is what it did of its own accord, not the step's whole move. */
export function settle(world) {
  const list = world._awakeList, n = world._awakeCount, type = world._type, v = world._velocity, own = world._walk;
  for (let i = 0; i < n; i++) {
    const s3 = 3 * list[i];
    if (type[list[i]] === CHARACTER) { v[s3] = own[s3]; v[s3 + 1] = own[s3 + 1]; v[s3 + 2] = own[s3 + 2]; }
  }
}

function move(world, s, dt) {
  const s3 = 3 * s, p = world._position, v = world._velocity;
  tolerance = world._size[s] * PIXEL;
  slopeCos = world._slopeCos[s];
  const h = world._stepHeight[s];
  place(world, s, shape);
  pos[0] = p[s3]; pos[1] = p[s3 + 1]; pos[2] = p[s3 + 2];

  // Out of anything it overlaps; another character gives way half as much.
  for (let k = 0; k < PUSHES; k++) {
    at(pos);
    const o = deepest(world, shape, s, deep);
    if (o < 0) break;
    const share = world._type[o] === CHARACTER ? 0.5 : 1, d = deep[0] * share;
    pos[0] += deep[1] * d; pos[1] += deep[2] * d; pos[2] += deep[3] * d;
  }

  // What it stood on at the end of the last step, if it is still there.
  let ground = -1;
  const id = world._ground[s];
  if (id >= 0) ground = world._slots.slotOf(id);
  const grounded = ground >= 0;

  // Its own motion. Gravity only in the air; on the ground, the ground holds it.
  vel[0] = v[s3]; vel[1] = v[s3 + 1]; vel[2] = v[s3 + 2];
  if (!grounded) {
    const g = world._gravity, k = world._gravityScale[s] * dt;
    vel[0] += g[0] * k; vel[1] += g[1] * k; vel[2] += g[2] * k;
  }
  const jumping = dot(vel, up) > 0;
  rest[0] = vel[0] * dt; rest[1] = vel[1] * dt; rest[2] = vel[2] * dt;
  // Walking on the ground follows its slope, at the speed asked for across it.
  if (grounded && !jumping) follow(world._groundAt, 6 * s + 3);
  let onGround = grounded;
  let count = 0;
  for (let k = 0; k < SLIDES; k++) {
    const len = Math.sqrt(dot(rest, rest));
    if (!(len > 0)) break;
    const dx = rest[0] / len, dy = rest[1] / len, dz = rest[2] / len;
    at(pos);
    if (sweep(world, shape, s, -1, dx, dy, dz, len, hit) < 0) {
      pos[0] += rest[0]; pos[1] += rest[1]; pos[2] += rest[2];
      break;
    }
    const t = hit[0], left = 1 - t / len;
    pos[0] += dx * t; pos[1] += dy * t; pos[2] += dz * t;
    rest[0] *= left; rest[1] *= left; rest[2] *= left;
    const nx = hit[1], ny = hit[2], nz = hit[3], rise = nx * up[0] + ny * up[1] + nz * up[2];
    if (rise >= slopeCos) {
      // Walkable ground: walking follows it; landing on it stops the fall.
      onGround = true;
      if (!jumping) follow(hit, 1);
      else clip(rest, nx, ny, nz);
      const fall = dot(vel, up);
      if (fall < 0) { vel[0] -= up[0] * fall; vel[1] -= up[1] * fall; vel[2] -= up[2] * fall; }
    } else {
      // A step it can walk up onto, if it is on the ground and walking.
      if (onGround && !jumping && h > 0 && climb(world, s, h)) continue;
      // A wall, a ceiling, or ground too steep to walk up: none of it is climbed.
      stop(rest, nx, ny, nz);
      stop(vel, nx, ny, nz);
    }
    // Into a corner: along the crease the planes make, or not at all.
    for (let j = 0; j < count; j++) {
      const px = planes[3 * j], py = planes[3 * j + 1], pz = planes[3 * j + 2];
      if (rest[0] * px + rest[1] * py + rest[2] * pz >= 0) continue;
      let cx = py * nz - pz * ny, cy = pz * nx - px * nz, cz = px * ny - py * nx;
      const cl = Math.sqrt(cx * cx + cy * cy + cz * cz);
      if (!(cl > 0)) { rest[0] = rest[1] = rest[2] = 0; break; }
      cx /= cl; cy /= cl; cz /= cl;
      const along = rest[0] * cx + rest[1] * cy + rest[2] * cz;
      rest[0] = cx * along; rest[1] = cy * along; rest[2] = cz * along;
    }
    if (count < 3) { planes[3 * count] = nx; planes[3 * count + 1] = ny; planes[3 * count + 2] = nz; count++; }
  }

  // Back down onto the ground it walked off, by up to a step: down slopes and stairs.
  if (grounded && !jumping && under(world, s, 2 * tolerance) < 0) {
    at(pos);
    if (sweep(world, shape, s, -1, -up[0], -up[1], -up[2], h + 2 * tolerance, hit) >= 0 && stands(world, s, h)) {
      pos[0] -= up[0] * hit[0]; pos[1] -= up[1] * hit[0]; pos[2] -= up[2] * hit[0];
    }
  }

  // What it stands on now, for the next step and for groundOf().
  const found = under(world, s, 2 * tolerance);
  world._ground[s] = found >= 0 ? world._slots.id(found) : -1;
  const g6 = 6 * s;
  if (found >= 0) {
    for (let k = 0; k < 3; k++) { world._groundAt[g6 + k] = hit[4 + k]; world._groundAt[g6 + 3 + k] = face[k]; }
    // On the ground, it isn't falling.
    const fall = dot(vel, up);
    if (fall < 0) { vel[0] -= up[0] * fall; vel[1] -= up[1] * fall; vel[2] -= up[2] * fall; }
  }

  // Carried by what it stood on: that body's velocity where the character
  // stands, after its own motion. Its own motion was worked out against that
  // body where the step found it, as the ground was, so the two agree; then
  // both move on by the same. Cast like any move, but through that body
  // itself, so a platform going down takes it down, and a wall it is carried
  // into still stops it.
  if (ground >= 0) {
    const g3 = 3 * ground, gv = world._velocity, gw = world._angular, gp = world._position;
    const rx = pos[0] - gp[g3], ry = pos[1] - gp[g3 + 1], rz = pos[2] - gp[g3 + 2];
    const cx = (gv[g3] + gw[g3 + 1] * rz - gw[g3 + 2] * ry) * dt;
    const cy = (gv[g3 + 1] + gw[g3 + 2] * rx - gw[g3] * rz) * dt;
    const cz = (gv[g3 + 2] + gw[g3] * ry - gw[g3 + 1] * rx) * dt;
    const len = Math.sqrt(cx * cx + cy * cy + cz * cz);
    if (len > 0) {
      at(pos);
      const t = sweep(world, shape, s, ground, cx / len, cy / len, cz / len, len, hit) >= 0 ? hit[0] : len;
      pos[0] += cx / len * t; pos[1] += cy / len * t; pos[2] += cz / len * t;
    }
  }

  // Where it found it can go, now, and still through the step; its velocity afterwards is its own.
  p[s3] = pos[0]; p[s3 + 1] = pos[1]; p[s3 + 2] = pos[2];
  v[s3] = 0; v[s3 + 1] = 0; v[s3 + 2] = 0;
  const own = world._walk;
  own[s3] = vel[0]; own[s3 + 1] = vel[1]; own[s3 + 2] = vel[2];
}

/** The walkable ground within `reach` below the character, from where it is now, into hit; its slot or -1. */
function under(world, s, reach) {
  at(pos);
  const o = sweep(world, shape, s, -1, -up[0], -up[1], -up[2], reach, hit);
  return o >= 0 && stands(world, s, world._stepHeight[s]) ? o : -1;
}

/**
 * Whether the contact in hit is ground to stand on, with the ground's normal
 * into `face`: walkable itself, or the lip of walkable ground. A round shape
 * meets the edge of a step steeply (a capsule rests on a stair's lip at 51°,
 * say), though what it stands on there is the flat tread. So a contact too
 * steep to walk on is probed just past it, a pixel-sized ball dropped onto
 * the far side of the edge, and counts as ground if that finds walkable ground
 * at the contact's own height, and no higher above the character's lowest
 * point than a step: a ledge is under it, not a wall's top edge at its waist.
 * A steep slope probed so still finds itself. hit is a cast along -up.
 */
function stands(world, s, h) {
  const nx = hit[1], ny = hit[2], nz = hit[3], rise = nx * up[0] + ny * up[1] + nz * up[2];
  if (rise >= slopeCos) {
    face[0] = nx; face[1] = ny; face[2] = nz;
    return true;
  }
  if (!(rise > 0)) return false;   // a wall or a ceiling has no top to stand on
  const c = shape.center, base = c[0] * up[0] + c[1] * up[1] + c[2] * up[2] - hit[0] - down(shape);
  if (hit[4] * up[0] + hit[5] * up[1] + hit[6] * up[2] - base > h + tolerance) return false;
  // From the character across to what it rests on: the normal's level part, reversed.
  let ax = up[0] * rise - nx, ay = up[1] * rise - ny, az = up[2] * rise - nz;
  const al = Math.sqrt(ax * ax + ay * ay + az * az);
  if (!(al > 0)) return false;
  ax /= al; ay /= al; az /= al;
  const px = hit[4], py = hit[5], pz = hit[6];
  probe.half[0] = tolerance;
  probe.center[0] = px + 2 * tolerance * (ax + up[0]);
  probe.center[1] = py + 2 * tolerance * (ay + up[1]);
  probe.center[2] = pz + 2 * tolerance * (az + up[2]);
  if (sweep(world, probe, s, -1, -up[0], -up[1], -up[2], 4 * tolerance, fall) < 0) return false;
  if (fall[1] * up[0] + fall[2] * up[1] + fall[3] * up[2] < slopeCos) return false;
  if (Math.abs((fall[4] - px) * up[0] + (fall[5] - py) * up[1] + (fall[6] - pz) * up[2]) > tolerance) return false;
  face[0] = fall[1]; face[1] = fall[2]; face[2] = fall[3];
  return true;
}

/**
 * Up onto a step: up by the step height (or to a ceiling), forward by what is
 * left of the walk, and down onto walkable ground no further than it went up.
 * Moves pos and leaves what is left of the walk in rest if it works; changes
 * nothing if it doesn't, as when the way is still blocked, or the far side is
 * a drop, or too steep to stand on.
 */
function climb(world, s, h) {
  const rise = dot(rest, up);
  let fx = rest[0] - up[0] * rise, fy = rest[1] - up[1] * rise, fz = rest[2] - up[2] * rise;
  const len = Math.sqrt(fx * fx + fy * fy + fz * fz);
  if (!(len > tolerance)) return false;
  fx /= len; fy /= len; fz /= len;
  const x = pos[0], y = pos[1], z = pos[2];
  // Its lowest point now: a step is measured from there to what it stands on after.
  const base = x * up[0] + y * up[1] + z * up[2] - down(shape);
  at(pos);
  const u = sweep(world, shape, s, -1, up[0], up[1], up[2], h, hit) >= 0 ? hit[0] : h;
  if (!(u > tolerance)) return false;
  pos[0] = x + up[0] * u; pos[1] = y + up[1] * u; pos[2] = z + up[2] * u;
  at(pos);
  const f = sweep(world, shape, s, -1, fx, fy, fz, len, hit) >= 0 ? hit[0] : len;
  if (!(f > tolerance)) return back(x, y, z);
  pos[0] += fx * f; pos[1] += fy * f; pos[2] += fz * f;
  at(pos);
  if (sweep(world, shape, s, -1, -up[0], -up[1], -up[2], u + tolerance, hit) < 0 || !stands(world, s, h)
    || hit[4] * up[0] + hit[5] * up[1] + hit[6] * up[2] - base > h + tolerance) return back(x, y, z);
  pos[0] -= up[0] * hit[0]; pos[1] -= up[1] * hit[0]; pos[2] -= up[2] * hit[0];
  const left = len - f;
  rest[0] = fx * left; rest[1] = fy * left; rest[2] = fz * left;
  return true;
}

function back(x, y, z) {
  pos[0] = x; pos[1] = y; pos[2] = z;
  return false;
}

/**
 * The motion in rest, turned to run along the ground whose unit normal is at
 * n[o..o+2], at the same speed across it: its part along up is dropped, and
 * what rises or falls with the slope put back.
 */
function follow(n, o) {
  const rise = dot(rest, up);
  const hx = rest[0] - up[0] * rise, hy = rest[1] - up[1] * rise, hz = rest[2] - up[2] * rise;
  const nu = n[o] * up[0] + n[o + 1] * up[1] + n[o + 2] * up[2];
  const k = nu > 0 ? -(hx * n[o] + hy * n[o + 1] + hz * n[o + 2]) / nu : 0;
  rest[0] = hx + up[0] * k; rest[1] = hy + up[1] * k; rest[2] = hz + up[2] * k;
}

/**
 * Takes out of a (the motion left, or the velocity) the part into a surface
 * it can't walk on, normal n: a wall, a ceiling, too steep a slope. Slid
 * along such a slope, it rises no more than it was already rising, so it
 * can't be walked up; and what that leaves of a push across it into the
 * slope is stopped, as by a wall.
 */
function stop(a, nx, ny, nz) {
  const into = a[0] * nx + a[1] * ny + a[2] * nz;
  if (into >= 0) return;
  const was = Math.max(dot(a, up), 0);
  a[0] -= nx * into; a[1] -= ny * into; a[2] -= nz * into;
  const gain = dot(a, up) - was;
  if (!(gain > 0)) return;
  a[0] -= up[0] * gain; a[1] -= up[1] * gain; a[2] -= up[2] * gain;
  // The slope's own direction across the ground: no further into it.
  const nu = nx * up[0] + ny * up[1] + nz * up[2];
  let hx = nx - up[0] * nu, hy = ny - up[1] * nu, hz = nz - up[2] * nu;
  const hl = Math.sqrt(hx * hx + hy * hy + hz * hz);
  if (!(hl > 0)) return;
  hx /= hl; hy /= hl; hz /= hl;
  const across = a[0] * hx + a[1] * hy + a[2] * hz;
  if (across < 0) { a[0] -= hx * across; a[1] -= hy * across; a[2] -= hz * across; }
}

/** Takes out of a the part into a surface of normal n, if any. */
function clip(a, nx, ny, nz) {
  const into = a[0] * nx + a[1] * ny + a[2] * nz;
  if (into < 0) { a[0] -= nx * into; a[1] -= ny * into; a[2] -= nz * into; }
}

/** How far a placed shape reaches below its centre, along -up. */
function down(sh) {
  const a = sh.axes, r = sh.half;
  if (sh.kind === SPHERE) return r[0];
  if (sh.kind === CAPSULE) return r[1] * Math.abs(a[3] * up[0] + a[4] * up[1] + a[5] * up[2]) + r[0];
  return r[0] * Math.abs(a[0] * up[0] + a[1] * up[1] + a[2] * up[2]) + r[1] * Math.abs(a[3] * up[0] + a[4] * up[1] + a[5] * up[2])
    + r[2] * Math.abs(a[6] * up[0] + a[7] * up[1] + a[8] * up[2]);
}

const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

/** The character's shape, moved to (x, y, z). */
function at(p) {
  shape.center[0] = p[0]; shape.center[1] = p[1]; shape.center[2] = p[2];
}
