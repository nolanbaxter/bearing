// Joints: what holds two bodies to each other, or one to a point in the world.
//
// A free body can move in six ways: along three axes and about three. Each is
// a degree of freedom. A joint takes some away and leaves the rest: a hinge
// leaves one, turning about its axis; a slider one, moving along it; a ball
// joint three, every turn about its point; a fixed joint none. A distance
// joint takes away one, the gap between two points.
//
// ROWS. Each motion a joint forbids is a row: one number the solver drives to
// zero, such as how fast two anchors part along x. A row has a Jacobian, the
// direction it acts in on each body, linear and angular, and the solver pushes
// along it, as it pushes along a contact's normal. Its effective mass is how
// hard the two bodies are to move against each other that way, so one push
// settles that row exactly. A limit is a row that pushes only one way, and
// only at its stop, as a contact does; a motor is a row driven to a speed, with
// a cap on how hard it may push.
//
// TREES. The rows that must hold, in joints that join bodies into trees (a
// chain, a ragdoll, a bridge), are solved exactly, all together: see
// forest.js, and why. What remains is solved one joint at a time, as contacts
// are: the rows that may give (limits, motors, a distance joint's spring and
// slack), and joints that close a loop. There a ball joint's three rows (x, y
// and z of the gap between its anchors), which are coupled, since a push
// along x at an anchor off the centre turns the body and moves the anchor
// along y as well, are solved together as one 3x3 system; a hinge's two
// turning rows and a slider's two sliding rows as 2x2.
//
// THE SOFT STEP. Joints are solved in the same substeps as contacts, first,
// with the same warm start, solve and relax (contact.js): each substep starts
// from the impulses the last one ended with, the solve pass pulls the anchors
// together, and the relax pass takes back the speed that pulling left behind,
// so correcting a joint adds no energy. Anchors and axes are worked out afresh
// from the bodies' rotations at every solve pass, so a joint that turns
// quickly stays exact.
//
// STORAGE. Joints have slots and generational ids as bodies do
// (core/slots.js), and their data in columns. Each body keeps a list of its
// joints, through two of these columns, so removing a body can remove its
// joints, and the pair filter can see that two bodies are joined.

import { fail, show, finite, vec3, direction, oneOf, options, known, readOnly } from './core/check.js';
import { atan2, rotate } from './core/math.js';
import { Slots, resized, MAX_SLOTS } from './core/slots.js';
import { Body } from './body.js';
import { Forest, plant, factorForest, solveForest, inverseSPD, NODE } from './forest.js';

export const TYPES = ['fixed', 'ball', 'hinge', 'slider', 'distance'];
const FIXED = 0, BALL = 1, HINGE = 2, SLIDER = 3, DISTANCE = 4;
const DYNAMIC = 0;   // World's type code
const USED = 1;      // Slots' state for a slot in use

const OPTIONS = ['type', 'a', 'b', 'anchor', 'anchorB', 'axis', 'min', 'max', 'length', 'cone',
  'motorSpeed', 'motorForce', 'frequency', 'dampingRatio', 'collide'];
/** Options every joint has. */
const SHARED = ['type', 'a', 'b', 'anchor', 'anchorB', 'collide'];
/** And each type's own; giving one another type has throws. */
const OWN = [
  [],
  ['axis', 'min', 'max', 'cone'],
  ['axis', 'min', 'max', 'motorSpeed', 'motorForce'],
  ['axis', 'min', 'max', 'motorSpeed', 'motorForce'],
  ['length', 'min', 'max', 'frequency', 'dampingRatio'],
];

// What `given` records: whether min, max and anchorB were given, or follow
// their defaults (which for a distance joint follow its length and spring).
const MIN = 1, MAX = 2, ANCHOR_B = 4;

// Where each accumulated impulse is kept, in a joint's 11: the linear and
// angular impulses as world vectors, then one number per scalar row.
const LIN = 0, ANG = 3, MOTOR = 6, LOWER = 7, UPPER = 8, AXIAL = 9, CONE = 10, IMPULSES = 11;

/** Every per-joint column: name, array type, numbers per joint. */
const COLUMNS = [
  ['type', Uint8Array, 1],
  ['a', Int32Array, 1],           // body slots; b is -1 for a point in the world
  ['b', Int32Array, 1],
  ['localA', Float64Array, 3],    // the anchor in a's own frame
  ['localB', Float64Array, 3],    // in b's own frame, or the world point with no b
  ['frameA', Float64Array, 4],    // the joint's axes in a's own frame, as a rotation of x to its axis
  ['frameB', Float64Array, 4],
  ['min', Float64Array, 1],
  ['max', Float64Array, 1],
  ['given', Uint8Array, 1],
  ['length', Float64Array, 1],
  ['cone', Float64Array, 1],
  ['motorSpeed', Float64Array, 1],
  ['motorForce', Float64Array, 1],
  ['frequency', Float64Array, 1],
  ['dampingRatio', Float64Array, 1],
  ['collide', Uint8Array, 1],
  ['next', Int32Array, 2],        // the next joint in a's list and in b's, as an edge (2j + side) plus 1
  ['impulse', Float64Array, IMPULSES],
  ['list', Int32Array, 1],        // the joints solved this step, dense: see gather()
];

/**
 * How much of a tree joint's error the solve pass asks back each substep:
 * all of it. Measured on a 30-link chain let go sideways, whipping its end to
 * hundreds of rad/s: the widest any joint opened was 7.5 mm at 1, 8.5 mm at 0.5 and
 * 10.2 mm at 0.2, with hanging chains exact and no energy gained at any.
 */
const JOINT_BIAS = 1;

/**
 * How stiff the rows solved one joint at a time are (a stop being pushed
 * back, a joint closing a loop) while the solve pass pulls them, as a
 * fraction of the substep rate, and how damped: Box2D 3's joints, twice its
 * contacts' stiffness. Measured on a 100-link chain solved one joint at a
 * time: a third of the rate held it within 1.01%, at the edge of blowing up;
 * a half and stiffer, at any damping ratio from 5 to 50, blew up within 2 s.
 */
export const JOINT_STIFFNESS = 1 / 4;
export const JOINT_DAMPING_RATIO = 2;

export class Joints {
  constructor() {
    this.slots = new Slots();
    for (const [name, Type] of COLUMNS) this[name] = new Type(0);
    /** How many joints are in `list` this step. */
    this.active = 0;
    /** The trees among them, solved exactly (forest.js). */
    this.forest = new Forest();
  }

  get count() { return this.slots.count; }

  /** A slot for a new joint, growing the columns by doubling if it must. */
  take(caller) {
    let j = this.slots.take();
    if (j >= 0) return j;
    const slots = this.slots;
    if (slots.capacity >= slots.limit) {
      fail(caller, `a world holds at most ${MAX_SLOTS} joints (the slot field of an id is 24 bits)`);
    }
    this._resize(Math.min(Math.max(slots.capacity * 2, 1), slots.limit));
    j = this.slots.take();
    return j;
  }

  /** Frees slot j, shrinking by the world's quarter rule (see World._remove). */
  release(j) {
    this.impulse.fill(0, j * IMPULSES, j * IMPULSES + IMPULSES);
    this.slots.release(j);
    const top = this.slots.top;
    if (top <= this.slots.capacity / 4) {
      let capacity = top === 0 ? 0 : 1;
      while (capacity < top) capacity *= 2;
      this._resize(capacity);
    }
  }

  _resize(capacity) {
    this.slots.resize(capacity);
    for (const [name, , per] of COLUMNS) this[name] = resized(this[name], capacity * per);
    // The forest's storage follows, so joints removed give it back too.
    this.forest.fit(capacity);
  }
}

/**
 * A handle to one joint: a world and an id, and nothing else, as a Body is.
 * Compare `a.id === b.id`, not `a === b`.
 */
export class Joint {
  #world;
  #id;

  constructor(world, id) {
    this.#world = world;
    this.#id = id;
  }

  get world() { return this.#world; }
  set world(_) { readOnly('joint.world'); }

  /** A number, unique to this joint for the life of its world. */
  get id() { return this.#id; }
  set id(_) { readOnly('joint.id'); }

  /** false once the joint is removed, by itself or with either of its bodies, or its world destroyed. Never throws. */
  get alive() { return this.#world._jointAlive(this.#id); }
  set alive(_) { readOnly('joint.alive'); }

  /** Same as world.remove(joint). Does nothing if the joint is already gone. */
  destroy() {
    this.#world._remove('destroy', this);
  }
}

// ---- making, changing and reading ----

/** The joint's slot, from a handle given to `caller`. */
export function jointSlot(world, caller, joint) {
  world._live(caller);
  if (!(joint instanceof Joint)) fail(caller, `joint must be a Joint, got ${show(joint)}`);
  if (joint.world !== world) fail(caller, 'that joint belongs to another world');
  const j = world._joints.slots.slotOf(joint.id);
  if (j < 0) fail(caller, 'this joint was removed');
  return j;
}

/** A joined body's slot: `name` is 'a' or 'b'. */
function bodySlot(world, caller, name, body) {
  if (!(body instanceof Body) || body.world !== world) {
    fail(caller, `${name} must be a Body of this world${name === 'b' ? ', or null' : ''}, got ${show(body)}`);
  }
  const s = world._slots.slotOf(body.id);
  if (s < 0) fail(caller, `${name} was removed`);
  return s;
}

export function addJoint(world, given) {
  const caller = 'addJoint';
  world._live(caller);
  const o = options(caller, given);
  known(caller, o, OPTIONS);
  const has = (k) => o[k] !== undefined;
  const type = has('type') ? TYPES.indexOf(oneOf(caller, 'type', o.type, TYPES)) : BALL;
  own(caller, type, o);
  const a = bodySlot(world, caller, 'a', o.a);
  const b = o.b === undefined || o.b === null ? -1 : bodySlot(world, caller, 'b', o.b);
  if (a === b) fail(caller, 'a and b must be different bodies');
  const p = world._position, a3 = 3 * a, b3 = 3 * b;
  const centreA = [p[a3], p[a3 + 1], p[a3 + 2]];
  const centreB = b >= 0 ? [p[b3], p[b3 + 1], p[b3 + 2]] : centreA;
  // A distance joint joins the two centres unless told otherwise; any other
  // joint sits midway between them, or at a's centre held to the world.
  const anchor = has('anchor') ? vec3(caller, 'anchor', o.anchor)
    : type === DISTANCE ? centreA : [(centreA[0] + centreB[0]) * 0.5, (centreA[1] + centreB[1]) * 0.5, (centreA[2] + centreB[2]) * 0.5];
  const anchorB = has('anchorB') ? vec3(caller, 'anchorB', o.anchorB) : type === DISTANCE ? centreB : anchor;
  const axis = has('axis') ? direction(caller, 'axis', o.axis) : [0, 1, 0];
  const spec = settings(caller, type, o, DEFAULTS, between(anchor, anchorB));
  const collide = has('collide') ? flag(caller, 'collide', o.collide) : false;

  const J = world._joints;
  const j = J.take(caller);
  J.type[j] = type;
  J.a[j] = a;
  J.b[j] = b;
  write(J, j, spec);
  J.given[j] = spec.given | (has('anchorB') ? ANCHOR_B : 0);
  J.collide[j] = collide ? 1 : 0;
  place(world, j, anchor, anchorB, true, true);
  orient(world, j, axis);
  link(world, j, a, 0);
  if (b >= 0) link(world, j, b, 1);
  // Joined, they may no longer rest as they were (rule 36).
  world._wake(a);
  if (b >= 0) world._wake(b);
  return new Joint(world, J.slots.id(j));
}

export function setJoint(world, joint, changes) {
  const caller = 'setJoint';
  const j = jointSlot(world, caller, joint);
  const o = options(caller, changes, 'changes');
  known(caller, o, OPTIONS);
  const has = (k) => o[k] !== undefined;
  if (has('a') || has('b')) fail(caller, "a and b can't change; remove the joint and add another");
  const J = world._joints;
  const was = J.type[j];
  const type = has('type') ? TYPES.indexOf(oneOf(caller, 'type', o.type, TYPES)) : was;
  own(caller, type, o);
  const anchor = has('anchor') ? vec3(caller, 'anchor', o.anchor) : null;
  const anchorB = has('anchorB') ? vec3(caller, 'anchorB', o.anchorB) : null;
  const axis = has('axis') ? direction(caller, 'axis', o.axis) : null;
  // A new type starts from its own defaults, as a new shape does; the anchors,
  // axis and collide carry over. A new distance joint is as long as its two
  // ends are apart now.
  const base = type === was ? read(J, j) : DEFAULTS;
  const ends = ANCHORS;
  ends[0] = anchor ?? worldAnchor(world, j, 0);
  ends[1] = anchorB ?? (anchor && type !== DISTANCE && (J.given[j] & ANCHOR_B) === 0 ? anchor : worldAnchor(world, j, 1));
  const spec = settings(caller, type, o, base, between(ends[0], ends[1]));
  const collide = has('collide') ? flag(caller, 'collide', o.collide) : J.collide[j] === 1;

  // Everything checked: now written. What a change of shape or stop makes of
  // the impulses the joint remembers is no longer known, so they start over.
  const limits = spec.min !== base.min || spec.max !== base.max || spec.given !== base.given || spec.cone !== base.cone;
  if (type !== was || anchor || anchorB || axis) J.impulse.fill(0, j * IMPULSES, j * IMPULSES + IMPULSES);
  else if (limits) { const i = j * IMPULSES; J.impulse[i + LOWER] = 0; J.impulse[i + UPPER] = 0; J.impulse[i + CONE] = 0; }
  J.type[j] = type;
  write(J, j, spec);
  J.given[j] = spec.given | (J.given[j] & ANCHOR_B) | (anchorB ? ANCHOR_B : 0);
  if (anchor || anchorB) place(world, j, ends[0], ends[1], anchor !== null || anchorB === null, true);
  if (axis) orient(world, j, axis);
  const apart = J.collide[j] === 0 && collide;
  J.collide[j] = collide ? 1 : 0;
  // Let go of each other, the two may now collide: the broadphase is asked
  // about them again at the next step, as if one had moved.
  if (apart && J.b[j] >= 0) world._broadphase._mark(J.a[j]);
  world._wake(J.a[j]);
  if (J.b[j] >= 0) world._wake(J.b[j]);
}

/** A copy of joint slot j's options, as addJoint takes them, so addJoint(jointOf(j)) makes another. */
export function jointOf(world, j) {
  const J = world._joints, type = J.type[j], s = read(J, j);
  const a = J.a[j], b = J.b[j];
  const out = { type: TYPES[type], a: new Body(world, world._slots.id(a)), b: b >= 0 ? new Body(world, world._slots.id(b)) : null };
  out.anchor = worldAnchor(world, j, 0);
  if (type === DISTANCE || (J.given[j] & ANCHOR_B) !== 0) out.anchorB = worldAnchor(world, j, 1);
  if (type !== FIXED && type !== DISTANCE) {
    QA.set(world._rotation.subarray(4 * a, 4 * a + 4));
    multiply(QA, 0, J.frameA, 4 * j, QA, 0);
    out.axis = column(QA, 0);
  }
  if (type === DISTANCE) out.length = s.length;
  if (type !== FIXED) {
    if (s.given & MIN) out.min = s.min;
    if (s.given & MAX) out.max = s.max;
  }
  if (type === BALL) out.cone = s.cone;
  if (type === HINGE || type === SLIDER) { out.motorSpeed = s.motorSpeed; out.motorForce = s.motorForce; }
  if (type === DISTANCE) { out.frequency = s.frequency; out.dampingRatio = s.dampingRatio; }
  out.collide = J.collide[j] === 1;
  return out;
}

/** Takes joint slot j out: unlinked from its bodies, which wake, and its slot freed. */
export function dropJoint(world, j) {
  const J = world._joints, a = J.a[j], b = J.b[j];
  unlink(world, j, a, 0);
  if (b >= 0) unlink(world, j, b, 1);
  world._wake(a);
  if (b >= 0) {
    world._wake(b);
    if (J.collide[j] === 0) world._broadphase._mark(a);
  }
  J.release(j);
}

/** Every joint of body slot s, removed with it. */
export function dropJointsOf(world, s) {
  while (world._jointHead[s] !== 0) dropJoint(world, (world._jointHead[s] - 1) >> 1);
}

/** Whether a joint between body slots a and b keeps them from colliding: what the pair list asks. */
export function apart(world, a, b) {
  const J = world._joints, tally = world._jointTally;
  // The shorter of the two lists: a static body may hold many.
  const s = tally[a] <= tally[b] ? a : b, other = s === a ? b : a;
  for (let e = world._jointHead[s] - 1; e >= 0; e = J.next[e] - 1) {
    const j = e >> 1, o = (e & 1) === 0 ? J.b[j] : J.a[j];
    if (o === other && J.collide[j] === 0) return true;
  }
  return false;
}

const DEFAULTS = Object.freeze({
  min: -Infinity, max: Infinity, given: 0, length: 0,
  // A ball joint swings freely unless given a cone.
  cone: Math.PI,
  motorSpeed: 0, motorForce: 0,
  // No spring; given one, it settles without overshooting, as Bearing's
  // contacts do, unless told to bounce.
  frequency: 0, dampingRatio: 1,
});

/** A joint's type-specific settings, from its columns. */
function read(J, j) {
  return {
    min: J.min[j], max: J.max[j], given: J.given[j] & (MIN | MAX), length: J.length[j], cone: J.cone[j],
    motorSpeed: J.motorSpeed[j], motorForce: J.motorForce[j], frequency: J.frequency[j], dampingRatio: J.dampingRatio[j],
  };
}

function write(J, j, s) {
  J.min[j] = s.min; J.max[j] = s.max; J.length[j] = s.length; J.cone[j] = s.cone;
  J.motorSpeed[j] = s.motorSpeed; J.motorForce[j] = s.motorForce;
  J.frequency[j] = s.frequency; J.dampingRatio[j] = s.dampingRatio;
}

/** Throws for any option the type doesn't have. */
function own(caller, type, o) {
  for (const k of Object.keys(o)) {
    if (o[k] !== undefined && !SHARED.includes(k) && !OWN[type].includes(k)) fail(caller, `a ${TYPES[type]} joint has no ${k}`);
  }
}

/**
 * The type-specific settings, checked: `base` changed by `o`. `gap` is how
 * far apart the two anchors are, a distance joint's length unless given.
 */
function settings(caller, type, o, base, gap) {
  const has = (k) => o[k] !== undefined;
  const s = { ...base };
  if (type === DISTANCE && base === DEFAULTS) s.length = gap;
  if (has('length')) s.length = atLeastZero(caller, 'length', o.length);
  if (has('cone')) {
    const c = o.cone;
    if (typeof c !== 'number' || !(c >= 0 && c <= Math.PI)) fail(caller, `cone must be an angle from 0 to π, in radians, got ${show(c)}`);
    s.cone = c;
  }
  if (has('motorSpeed')) s.motorSpeed = finite(caller, 'motorSpeed', o.motorSpeed);
  if (has('motorForce')) s.motorForce = atLeastZero(caller, 'motorForce', o.motorForce);
  if (has('frequency')) s.frequency = atLeastZero(caller, 'frequency', o.frequency);
  if (has('dampingRatio')) s.dampingRatio = atLeastZero(caller, 'dampingRatio', o.dampingRatio);
  // null is a value here, as for a body's inertia: back to the default.
  if (o.min === null) s.given &= ~MIN;
  else if (has('min')) { s.min = bound(caller, type, 'min', o.min); s.given |= MIN; }
  if (o.max === null) s.given &= ~MAX;
  else if (has('max')) { s.max = bound(caller, type, 'max', o.max); s.given |= MAX; }
  if (type !== FIXED) {
    const lo = low(type, s.given, s.min, s.length, s.frequency), hi = high(type, s.given, s.max, s.length, s.frequency);
    if (lo > hi) fail(caller, `min must be at most max (${hi}), got ${lo}`);
  }
  return s;
}

/** A limit, checked for its type: angles for a hinge's or ball joint's turn, metres for a slider's or distance joint's. */
function bound(caller, type, name, v) {
  const none = name === 'min' ? -Infinity : Infinity;
  if (type === HINGE || type === BALL) {
    if (typeof v !== 'number' || !(v === none || (v >= -Math.PI && v <= Math.PI))) {
      fail(caller, `${name} must be from -π to π, in radians, or ${none}, got ${show(v)}`);
    }
  } else if (type === SLIDER) {
    if (typeof v !== 'number' || v !== v || v === -none) fail(caller, `${name} must be a number or ${none}, got ${show(v)}`);
  } else if (typeof v !== 'number' || !(v >= 0) || v === -none) {
    fail(caller, name === 'min' ? `min must be 0 or more, got ${show(v)}` : `max must be 0 or more, or Infinity, got ${show(v)}`);
  }
  return v;
}

/**
 * Where a joint's free coordinate may go, as the solver reads it. A distance
 * joint with no spring is a rod, its length both ways, unless given limits;
 * with one, it is free to stretch either way, its spring pulling it to length.
 */
function low(type, given, min, length, frequency) {
  if (given & MIN) return min;
  return type === DISTANCE ? (frequency > 0 ? 0 : length) : -Infinity;
}

function high(type, given, max, length, frequency) {
  if (given & MAX) return max;
  return type === DISTANCE && frequency === 0 ? length : Infinity;
}

/** How far apart two points are. */
function between(p, q) {
  const x = q[0] - p[0], y = q[1] - p[1], z = q[2] - p[2];
  return Math.sqrt(x * x + y * y + z * z);
}

function atLeastZero(caller, name, value) {
  if (typeof value !== 'number' || !(value >= 0) || value === Infinity) fail(caller, `${name} must be 0 or more, got ${show(value)}`);
  return value;
}

function flag(caller, name, value) {
  if (typeof value !== 'boolean') fail(caller, `${name} must be true or false, got ${show(value)}`);
  return value;
}

/** Sets joint j's anchors from world points, where its bodies are now: a's end, and b's. */
function place(world, j, anchor, anchorB, setA, setB) {
  const J = world._joints, p = world._position, q = world._rotation, a = J.a[j], b = J.b[j];
  if (setA) {
    rotate(q, 4 * a, anchor[0] - p[3 * a], anchor[1] - p[3 * a + 1], anchor[2] - p[3 * a + 2], true, V3);
    J.localA.set(V3, 3 * j);
  }
  if (!setB) return;
  if (b < 0) {
    J.localB.set(anchorB, 3 * j);
    return;
  }
  rotate(q, 4 * b, anchorB[0] - p[3 * b], anchorB[1] - p[3 * b + 1], anchorB[2] - p[3 * b + 2], true, V3);
  J.localB.set(V3, 3 * j);
}

/**
 * Sets joint j's axis, as a frame in each body's own: the shortest turn from
 * x to the axis, carried by each body as it is now. From here, the joint's
 * angle is 0, and a hinge's or ball joint's turn is measured from it.
 */
function orient(world, j, axis) {
  const J = world._joints, q = world._rotation, a = J.a[j], b = J.b[j];
  // The shortest turn from x to v is (x × v, 1 + x·v), normalized. Opposite
  // x it has no one axis; half a turn about y will do.
  let fx = 0, fy = -axis[2], fz = axis[1], fw = 1 + axis[0];
  if (fy === 0 && fz === 0 && fw <= 0) { fy = 1; fw = 0; }
  const l = 1 / Math.sqrt(fx * fx + fy * fy + fz * fz + fw * fw);
  F[0] = fx * l; F[1] = fy * l; F[2] = fz * l; F[3] = fw * l;
  conjugateTimes(q, 4 * a, F, 0, J.frameA, 4 * j);
  if (b >= 0) conjugateTimes(q, 4 * b, F, 0, J.frameB, 4 * j);
  else J.frameB.set(F, 4 * j);
}

/** Joint j's anchor on side 0 (a) or 1 (b), in the world, now: a fresh array. */
export function worldAnchor(world, j, side) {
  const J = world._joints, s = side === 0 ? J.a[j] : J.b[j], local = side === 0 ? J.localA : J.localB;
  if (s < 0) return [local[3 * j], local[3 * j + 1], local[3 * j + 2]];
  const p = world._position;
  rotate(world._rotation, 4 * s, local[3 * j], local[3 * j + 1], local[3 * j + 2], false, V3);
  return [p[3 * s] + V3[0], p[3 * s + 1] + V3[1], p[3 * s + 2] + V3[2]];
}

/** Puts joint j at the head of body slot s's list, as side 0 (a) or 1 (b). */
function link(world, j, s, side) {
  const J = world._joints, e = 2 * j + side;
  J.next[e] = world._jointHead[s];
  world._jointHead[s] = e + 1;
  world._jointTally[s]++;
}

function unlink(world, j, s, side) {
  const J = world._joints, e = 2 * j + side;
  if (world._jointHead[s] === e + 1) {
    world._jointHead[s] = J.next[e];
  } else {
    let f = world._jointHead[s] - 1;
    while (J.next[f] !== e + 1) f = J.next[f] - 1;
    J.next[f] = J.next[e];
  }
  J.next[e] = 0;
  world._jointTally[s]--;
}

// ---- solving ----

/**
 * Lists the joints this step solves: those with an awake body and a dynamic
 * one, since a joint between bodies that can't move has nothing to do. A
 * sleeping dynamic body held to an awake one is woken, as a touch would wake
 * it: joined dynamic bodies sleep as one island, so this is a body held to a
 * kinematic body that started moving, or one made dynamic since. Waking sets
 * the world's _wakePending, and the step comes round to this again, so the
 * list it leaves is complete.
 *
 * ponytail: a pass over every joint each step, asleep ones too; keep awake and
 * asleep runs, as the pair list does, if a scene of sleeping ragdolls measures it.
 */
export function gather(world) {
  const J = world._joints, state = J.slots.state, top = J.slots.top, list = J.list;
  const awake = world._awake, island = world._island, type = world._type;
  let n = 0;
  for (let j = 0; j < top; j++) {
    if (state[j] !== USED) continue;
    const a = J.a[j], b = J.b[j];
    const awakeA = awake[a] === 1, awakeB = b >= 0 && awake[b] === 1;
    if (!awakeA && !awakeB) continue;
    if (awakeA && b >= 0 && island[b] !== 0 && type[b] === DYNAMIC) world._wake(b);
    if (awakeB && island[a] !== 0 && type[a] === DYNAMIC) world._wake(a);
    if (type[a] !== DYNAMIC && (b < 0 || type[b] !== DYNAMIC)) continue;
    list[n++] = j;
  }
  J.active = n;
}

/** How soft the solve pass is this step: set by World from JOINT_STIFFNESS and the substep. */
export const jointSoft = { biasRate: 0, massScale: 1, impulseScale: 0 };

// The joint being solved: its two bodies' velocities (linear, then angular),
// inverse masses and world inverse inertias, loaded before and saved after,
// and its geometry. Module scratch: the solver is never re-entered.
const VA = new Float64Array(6), VB = new Float64Array(6);
const IA = new Float64Array(6), IB = new Float64Array(6);
let ima = 0, imb = 0;
const RA = new Float64Array(3), RB = new Float64Array(3), D = new Float64Array(3);
const QA = new Float64Array(4), QB = new Float64Array(4), REL = new Float64Array(4);
const A1 = new Float64Array(3), P1 = new Float64Array(3), P2 = new Float64Array(3), A2 = new Float64Array(3);
/** Up to six rows, 9 numbers each: n (linear, on b; -n on a), then the angular parts on a and on b. */
const ROWS = new Float64Array(54);
const C = new Float64Array(3), ACC = new Float64Array(3), LAMBDA = new Float64Array(3), RHS = new Float64Array(3);
const LEVER = new Float64Array(3);
const V3 = new Float64Array(3), F = new Float64Array(4);
const ANCHORS = [null, null];
const ZERO3 = new Float64Array(3), ZERO6 = new Float64Array(6), IDENTITY = new Float64Array([0, 0, 0, 1]);
/** A distance joint's own spring, worked out per joint. */
const spring = { biasRate: 0, massScale: 1, impulseScale: 0 };

/** Applies every solved joint's accumulated impulses, at the start of each substep. */
export function warmStartJoints(world) {
  const J = world._joints, list = J.list, n = J.active, acc = J.impulse;
  for (let i = 0; i < n; i++) {
    const j = list[i], type = J.type[j], o = j * IMPULSES;
    load(world, j);
    geometry(world, j);
    if (type === SLIDER) {
      frames(world, j);
      flatten(acc, o + LIN);
      reach();
      for (let k = 0; k < 3; k++) {
        lever(9 * k, k);
        apply(9 * k, acc[o + LIN + k]);
      }
      angles(acc, o + ANG);
      slide(0);
      apply(0, acc[o + MOTOR] + acc[o + LOWER] - acc[o + UPPER]);
    } else if (type === DISTANCE) {
      if (stretch(0) > 0) apply(0, acc[o + AXIAL] + acc[o + LOWER] - acc[o + UPPER]);
    } else {
      for (let k = 0; k < 3; k++) {
        point(9 * k, k);
        apply(9 * k, acc[o + LIN + k]);
      }
      if (type === HINGE) {
        frames(world, j);
        flatten(acc, o + ANG);
      }
      if (type === FIXED || type === HINGE) angles(acc, o + ANG);
      if (type === HINGE) {
        axle(0);
        apply(0, acc[o + MOTOR] + acc[o + LOWER] - acc[o + UPPER]);
      } else if (type === BALL && swings(J, j)) {
        frames(world, j);
        if (twistAxis(0)) apply(0, acc[o + LOWER] - acc[o + UPPER]);
        if (coneAxis(0) >= 0) apply(0, -acc[o + CONE]);
      }
    }
    save(world, j);
  }
}

/**
 * One pass over every solved joint. `bias` is true while solving, false
 * while relaxing. In each joint, the rows that may give (motors, limits,
 * springs) come first and the rows that must hold come last, so what holds
 * is solved against everything else's latest push: Box2D's order.
 */
export function solveJoints(world, h, bias) {
  const J = world._joints, list = J.list, n = J.active, acc = J.impulse, tree = J.forest.inTree;
  for (let i = 0; i < n; i++) {
    const j = list[i], o = j * IMPULSES, type = J.type[j], held = tree[j] === 1;
    // A fixed joint in a tree has nothing that gives: the forest holds it all.
    if (held && type === FIXED) continue;
    base = o;
    pins = type === FIXED || type === DISTANCE ? 0 : -1;
    pinType = type;
    load(world, j);
    geometry(world, j);
    switch (type) {
      case FIXED:
        frames(world, j);
        holdAngles(acc, o, bias);
        holdPoint(acc, o, bias);
        break;
      case BALL:
        if (swings(J, j)) {
          frames(world, j);
          twistLimits(J, j, acc, o, h, bias);
          if (J.cone[j] < Math.PI) coneLimit(J, j, acc, o, h, bias);
        }
        if (!held) holdPoint(acc, o, bias);
        break;
      case HINGE:
        frames(world, j);
        if (J.motorForce[j] > 0) {
          axle(0);
          motor(0, J.motorSpeed[j], J.motorForce[j] * h, acc, o + MOTOR);
        }
        hingeLimits(J, j, acc, o, h, bias);
        if (!held) {
          holdAxis(acc, o, bias);
          holdPoint(acc, o, bias);
        }
        break;
      case SLIDER:
        frames(world, j);
        reach();
        if (J.motorForce[j] > 0) {
          slide(0);
          motor(0, J.motorSpeed[j], J.motorForce[j] * h, acc, o + MOTOR);
        }
        sliderLimits(J, j, acc, o, h, bias);
        if (!held) {
          holdAngles(acc, o, bias);
          holdLine(acc, o, bias);
        }
        break;
      default:
        distance(J, j, acc, o, h, bias, held);
    }
    save(world, j);
  }
  if (J.forest.count > 0) holdForest(world, h, bias);
}

/** Whether joint slot j has rows that must hold, and so can be in a tree: all but a distance joint with room to stretch. */
function holds(J, j) {
  if (J.type[j] !== DISTANCE) return true;
  const f = J.frequency[j], length = J.length[j];
  return low(DISTANCE, J.given[j], J.min[j], length, f) === high(DISTANCE, J.given[j], J.max[j], length, f);
}

/** This step's trees, from the joints gather() listed: see forest.js. */
export function plantJoints(world) {
  const J = world._joints;
  plant(world, J.list, J.active, (j) => holds(J, j));
}

/**
 * Every tree joint's rows that must hold, solved exactly together
 * (forest.js). The solve pass builds the rows from where the bodies are,
 * factors them, and asks each for its whole error back this substep,
 * JOINT_BIAS of it; the relax pass, after the bodies have moved, solves the
 * same rows again for no speed along them.
 */
function holdForest(world, h, bias) {
  const J = world._joints, F = J.forest, T = F.count;
  if (bias) {
    build(world, JOINT_BIAS / h);
    factorForest(world);
  } else {
    const v = world._velocity, w = world._angular, row = F.row;
    for (let t = 0; t < T; t++) {
      const j = F.joint[t], a3 = 3 * J.a[j], b = J.b[j], b3 = 3 * b;
      for (let r = 0; r < F.rows[t]; r++) {
        const o = 54 * t + 9 * r;
        let s = -(row[o] * v[a3] + row[o + 1] * v[a3 + 1] + row[o + 2] * v[a3 + 2])
          - (row[o + 3] * w[a3] + row[o + 4] * w[a3 + 1] + row[o + 5] * w[a3 + 2]);
        if (b >= 0) {
          s += row[o] * v[b3] + row[o + 1] * v[b3 + 1] + row[o + 2] * v[b3 + 2]
            + row[o + 6] * w[b3] + row[o + 7] * w[b3 + 1] + row[o + 8] * w[b3 + 2];
        }
        F.rhs[6 * t + r] = -s;
      }
    }
  }
  settleForest(world);
}

/** Every forest joint's rows, from where its bodies are now, into F.row, with their targets. */
function build(world, rate) {
  const J = world._joints, F = J.forest, T = F.count;
  for (let t = 0; t < T; t++) {
    const j = F.joint[t], type = J.type[j];
    load(world, j);
    geometry(world, j);
    F.scale[t] = 0;
    let m = 0;
    if (type === FIXED || type === SLIDER) {
      frames(world, j);
      rotate(QA, 0, REL[0], REL[1], REL[2], false, C);
      for (let k = 0; k < 3; k++) {
        const r = 9 * k;
        ROWS.fill(0, r, r + 9);
        ROWS[r + 3 + k] = 1;
        ROWS[r + 6 + k] = 1;
        m = emit(F, t, m, r, 2 * C[k], rate);
      }
    }
    if (type === HINGE) {
      frames(world, j);
      const ex = A1[1] * A2[2] - A1[2] * A2[1], ey = A1[2] * A2[0] - A1[0] * A2[2], ez = A1[0] * A2[1] - A1[1] * A2[0];
      turning(0, P1);
      turning(9, P2);
      m = emit(F, t, m, 0, ex * P1[0] + ey * P1[1] + ez * P1[2], rate);
      m = emit(F, t, m, 9, ex * P2[0] + ey * P2[1] + ez * P2[2], rate);
    }
    if (type === FIXED || type === BALL || type === HINGE) {
      for (let k = 0; k < 3; k++) {
        point(9 * k, k);
        m = emit(F, t, m, 9 * k, D[k], rate);
      }
    } else if (type === SLIDER) {
      reach();
      along(0, P1, LEVER);
      along(9, P2, LEVER);
      m = emit(F, t, m, 0, D[0] * P1[0] + D[1] * P1[1] + D[2] * P1[2], rate);
      m = emit(F, t, m, 9, D[0] * P2[0] + D[1] * P2[1] + D[2] * P2[2], rate);
    } else if (type === DISTANCE) {
      const l = stretch(0);
      if (l === 0) ROWS.fill(0, 0, 9);
      const f = J.frequency[j], length = J.length[j];
      m = emit(F, t, m, 0, l - low(DISTANCE, J.given[j], J.min[j], length, f), rate);
    }
    F.rows[t] = m;
  }
}


/** Solves the factored forest and applies it: each body's change of velocity, and each joint's impulses, kept for the warm start. */
function settleForest(world) {
  const J = world._joints, F = J.forest, acc = J.impulse, T = F.count;
  solveForest(world);
  // Each body's change of velocity, then each joint's impulses, kept for the warm start.
  const v = world._velocity, w = world._angular, vec = F.vector;
  for (let k = 0; k < F.bodies; k++) {
    const s3 = 3 * F.body[k], o = NODE * (T + k);
    v[s3] += vec[o]; v[s3 + 1] += vec[o + 1]; v[s3 + 2] += vec[o + 2];
    w[s3] += vec[o + 3]; w[s3 + 1] += vec[o + 4]; w[s3 + 2] += vec[o + 5];
  }
  for (let t = 0; t < T; t++) {
    const j = F.joint[t], type = J.type[j], o = j * IMPULSES, row = F.row, r = 54 * t;
    const l = NODE * t;
    if (type === FIXED || type === SLIDER) {
      for (let k = 0; k < 3; k++) acc[o + ANG + k] += vec[l + k];
      if (type === SLIDER) {
        // The line's rows, along P1 and P2: their impulse as a world vector.
        for (let k = 0; k < 3; k++) acc[o + LIN + k] += vec[l + 3] * row[r + 27 + k] + vec[l + 4] * row[r + 36 + k];
      } else {
        for (let k = 0; k < 3; k++) acc[o + LIN + k] += vec[l + 3 + k];
      }
    } else if (type === HINGE) {
      for (let k = 0; k < 3; k++) acc[o + ANG + k] += vec[l] * row[r + 3 + k] + vec[l + 1] * row[r + 12 + k];
      for (let k = 0; k < 3; k++) acc[o + LIN + k] += vec[l + 2 + k];
    } else if (type === BALL) {
      for (let k = 0; k < 3; k++) acc[o + LIN + k] += vec[l + k];
    } else {
      acc[o + AXIAL] += vec[l];
    }
  }
}

/** Copies row ROWS[o..] into tree joint t's row m, asking for the speed change that brings its speed to -rate c; returns m + 1. */
function emit(F, t, m, o, c, rate) {
  F.row.set(ROWS.subarray(o, o + 9), 54 * t + 9 * m);
  F.rhs[6 * t + m] = -(speed(o) + rate * c);
  F.scale[t] += coupling(o, o);
  return m + 1;
}

/** Joint j's bodies into the scratch: a point in the world is a body that never moves. */
function load(world, j) {
  const J = world._joints, a = J.a[j], b = J.b[j], v = world._velocity, w = world._angular, invI = world._invI;
  const a3 = 3 * a, a6 = 6 * a;
  VA[0] = v[a3]; VA[1] = v[a3 + 1]; VA[2] = v[a3 + 2]; VA[3] = w[a3]; VA[4] = w[a3 + 1]; VA[5] = w[a3 + 2];
  for (let k = 0; k < 6; k++) IA[k] = invI[a6 + k];
  ima = world._invMass[a];
  if (b < 0) {
    VB.fill(0);
    IB.fill(0);
    imb = 0;
    return;
  }
  const b3 = 3 * b, b6 = 6 * b;
  VB[0] = v[b3]; VB[1] = v[b3 + 1]; VB[2] = v[b3 + 2]; VB[3] = w[b3]; VB[4] = w[b3 + 1]; VB[5] = w[b3 + 2];
  for (let k = 0; k < 6; k++) IB[k] = invI[b6 + k];
  imb = world._invMass[b];
}

function save(world, j) {
  const J = world._joints, a = J.a[j], b = J.b[j], v = world._velocity, w = world._angular;
  const a3 = 3 * a;
  v[a3] = VA[0]; v[a3 + 1] = VA[1]; v[a3 + 2] = VA[2]; w[a3] = VA[3]; w[a3 + 1] = VA[4]; w[a3 + 2] = VA[5];
  if (b < 0) return;
  const b3 = 3 * b;
  v[b3] = VB[0]; v[b3 + 1] = VB[1]; v[b3 + 2] = VB[2]; w[b3] = VB[3]; w[b3 + 1] = VB[4]; w[b3 + 2] = VB[5];
}

/** The anchors where the bodies' rotations put them now, from each centre (RA, RB), and the gap from a's to b's (D). */
function geometry(world, j) {
  const J = world._joints, a = J.a[j], b = J.b[j], p = world._position, q = world._rotation, j3 = 3 * j;
  rotate(q, 4 * a, J.localA[j3], J.localA[j3 + 1], J.localA[j3 + 2], false, RA);
  const a3 = 3 * a;
  if (b < 0) {
    // The world point: b's "centre" is the origin and its anchor the point,
    // which nothing turns, since b takes no impulse.
    RB[0] = J.localB[j3]; RB[1] = J.localB[j3 + 1]; RB[2] = J.localB[j3 + 2];
    D[0] = RB[0] - p[a3] - RA[0]; D[1] = RB[1] - p[a3 + 1] - RA[1]; D[2] = RB[2] - p[a3 + 2] - RA[2];
    return;
  }
  rotate(q, 4 * b, J.localB[j3], J.localB[j3 + 1], J.localB[j3 + 2], false, RB);
  const b3 = 3 * b;
  D[0] = p[b3] + RB[0] - p[a3] - RA[0];
  D[1] = p[b3 + 1] + RB[1] - p[a3 + 1] - RA[1];
  D[2] = p[b3 + 2] + RB[2] - p[a3 + 2] - RA[2];
}

/**
 * The joint's axes in the world now: a's (A1, with P1 and P2 at right
 * angles to it) and b's (A2), and b's frame as seen from a's (REL), the
 * shorter way round.
 */
function frames(world, j) {
  const J = world._joints, a = J.a[j], b = J.b[j], q = world._rotation;
  multiply(q, 4 * a, J.frameA, 4 * j, QA, 0);
  if (b >= 0) multiply(q, 4 * b, J.frameB, 4 * j, QB, 0);
  else QB.set(J.frameB.subarray(4 * j, 4 * j + 4));
  axes(QA, A1, P1, P2);
  const x = QB[0], y = QB[1], z = QB[2], w = QB[3];
  A2[0] = 1 - 2 * (y * y + z * z); A2[1] = 2 * (x * y + z * w); A2[2] = 2 * (x * z - y * w);
  conjugateTimes(QA, 0, QB, 0, REL, 0);
  if (REL[3] < 0) { REL[0] = -REL[0]; REL[1] = -REL[1]; REL[2] = -REL[2]; REL[3] = -REL[3]; }
}

/** Whether a ball joint has a limit, and so needs its axes. */
function swings(J, j) {
  return J.cone[j] < Math.PI || (J.given[j] & (MIN | MAX)) !== 0;
}

// ---- rows ----
//
// A row at ROWS[o..o+8] is n, the linear direction it pushes b along (and a
// against), then cA and cB, the angular parts on a and b: an impulse λ along
// it changes vA by -λ n / ma, ωA by -λ Ia⁻¹ cA, vB by λ n / mb, ωB by λ Ib⁻¹ cB.

/** Row o: the anchors' relative motion along world axis k (a ball joint's rows). */
function point(o, k) {
  ROWS.fill(0, o, o + 9);
  ROWS[o + k] = 1;
  cross(RA, k, ROWS, o + 3);
  cross(RB, k, ROWS, o + 6);
}

/** Row o: like point(), with a's lever reaching to b's anchor, for a slider, whose line a carries. */
function lever(o, k) {
  ROWS.fill(0, o, o + 9);
  ROWS[o + k] = 1;
  cross(LEVER, k, ROWS, o + 3);
  cross(RB, k, ROWS, o + 6);
}

/** For lever() and slide(): a's lever to b's anchor, RA + D, into LEVER. */
function reach() {
  LEVER[0] = RA[0] + D[0]; LEVER[1] = RA[1] + D[1]; LEVER[2] = RA[2] + D[2];
}

/** Row o: turning of b against a about unit axis u. */
function turning(o, u) {
  ROWS[o] = 0; ROWS[o + 1] = 0; ROWS[o + 2] = 0;
  ROWS[o + 3] = u[0]; ROWS[o + 4] = u[1]; ROWS[o + 5] = u[2];
  ROWS[o + 6] = u[0]; ROWS[o + 7] = u[1]; ROWS[o + 8] = u[2];
}

/**
 * A joint's free coordinate is a's relative to b: a hinge's angle is how far
 * a has turned about the axis from where b holds it, a slider's position how
 * far a's anchor is along the axis from b's, and a motor drives a that way.
 * a is the body, and b what it hangs from, the world if none. Inside, a's
 * frame carries the axis and the rows measure b against a, so these rows are
 * the turned-round ones: their speed is a's against b.
 */
function negate(o) {
  for (let k = 0; k < 9; k++) ROWS[o + k] = -ROWS[o + k];
}

/** Row o: a turning against b about the joint's axis (see negate()). */
function axle(o) {
  turning(o, A1);
  negate(o);
}

/** Row o: a's anchor moving along the joint's axis against b's, the lever in LEVER (see reach()). */
function slide(o) {
  along(o, A1, LEVER);
  negate(o);
}

/** Row o: b's anchor moving along unit u, from a's lever r. */
function along(o, u, r) {
  ROWS[o] = u[0]; ROWS[o + 1] = u[1]; ROWS[o + 2] = u[2];
  ROWS[o + 3] = r[1] * u[2] - r[2] * u[1];
  ROWS[o + 4] = r[2] * u[0] - r[0] * u[2];
  ROWS[o + 5] = r[0] * u[1] - r[1] * u[0];
  ROWS[o + 6] = RB[1] * u[2] - RB[2] * u[1];
  ROWS[o + 7] = RB[2] * u[0] - RB[0] * u[2];
  ROWS[o + 8] = RB[0] * u[1] - RB[1] * u[0];
}

/** A distance joint's row at o, along the gap; returns the gap's length, 0 if the anchors meet. */
function stretch(o) {
  const l = Math.sqrt(D[0] * D[0] + D[1] * D[1] + D[2] * D[2]);
  if (l === 0) return 0;
  V3[0] = D[0] / l; V3[1] = D[1] / l; V3[2] = D[2] / l;
  ROWS[o] = V3[0]; ROWS[o + 1] = V3[1]; ROWS[o + 2] = V3[2];
  ROWS[o + 3] = RA[1] * V3[2] - RA[2] * V3[1];
  ROWS[o + 4] = RA[2] * V3[0] - RA[0] * V3[2];
  ROWS[o + 5] = RA[0] * V3[1] - RA[1] * V3[0];
  ROWS[o + 6] = RB[1] * V3[2] - RB[2] * V3[1];
  ROWS[o + 7] = RB[2] * V3[0] - RB[0] * V3[2];
  ROWS[o + 8] = RB[0] * V3[1] - RB[1] * V3[0];
  return l;
}

/** A ball joint's twist row at o, about the axis halfway between a's and b's; false where that is lost. */
function twistAxis(o) {
  const x = A1[0] + A2[0], y = A1[1] + A2[1], z = A1[2] + A2[2], l2 = x * x + y * y + z * z;
  // Within about a thousandth of a radian of a half-turn swing (|a1 + a2|² =
  // 2 + 2 cos φ under 2^-20), a twist has no axis to be measured about.
  if (!(l2 > TWIST_LOST)) return false;
  const s = 1 / Math.sqrt(l2);
  V3[0] = x * s; V3[1] = y * s; V3[2] = z * s;
  turning(o, V3);
  negate(o);
  return true;
}
const TWIST_LOST = 9.5367431640625e-7;   // 2^-20

/**
 * A ball joint's cone row at o, about the axis b's has swung away from a's
 * on; returns the swing angle, or -1 when it has none to measure (exactly
 * aligned). Exactly opposed, any axis at right angles will do.
 */
function coneAxis(o) {
  const x = A1[1] * A2[2] - A1[2] * A2[1], y = A1[2] * A2[0] - A1[0] * A2[2], z = A1[0] * A2[1] - A1[1] * A2[0];
  const c = A1[0] * A2[0] + A1[1] * A2[1] + A1[2] * A2[2];
  const s = Math.sqrt(x * x + y * y + z * z);
  if (s === 0) {
    if (c > 0) return -1;
    turning(o, P1);
    return Math.PI;
  }
  V3[0] = x / s; V3[1] = y / s; V3[2] = z / s;
  turning(o, V3);
  return atan2(s, c);
}

/** b's relative velocity along row o. */
function speed(o) {
  return ROWS[o] * (VB[0] - VA[0]) + ROWS[o + 1] * (VB[1] - VA[1]) + ROWS[o + 2] * (VB[2] - VA[2])
    + ROWS[o + 6] * VB[3] + ROWS[o + 7] * VB[4] + ROWS[o + 8] * VB[5]
    - ROWS[o + 3] * VA[3] - ROWS[o + 4] * VA[4] - ROWS[o + 5] * VA[5];
}

/** Rows o and p's coupling: how fast a push along p moves things along o, per unit impulse. */
function coupling(o, p) {
  return (ima + imb) * (ROWS[o] * ROWS[p] + ROWS[o + 1] * ROWS[p + 1] + ROWS[o + 2] * ROWS[p + 2])
    + bilinear(IA, ROWS, o + 3, p + 3) + bilinear(IB, ROWS, o + 6, p + 6);
}

/** x·M y, for symmetric M (xx xy xz yy yz zz) and x, y at r[i..], r[k..]. */
function bilinear(M, r, i, k) {
  const x0 = r[i], x1 = r[i + 1], x2 = r[i + 2], y0 = r[k], y1 = r[k + 1], y2 = r[k + 2];
  return x0 * (M[0] * y0 + M[1] * y1 + M[2] * y2) + x1 * (M[1] * y0 + M[3] * y1 + M[4] * y2)
    + x2 * (M[2] * y0 + M[4] * y1 + M[5] * y2);
}

/** Impulse λ along row o. */
function apply(o, l) {
  if (l === 0) return;
  const nx = ROWS[o] * l, ny = ROWS[o + 1] * l, nz = ROWS[o + 2] * l;
  VA[0] -= ima * nx; VA[1] -= ima * ny; VA[2] -= ima * nz;
  VB[0] += imb * nx; VB[1] += imb * ny; VB[2] += imb * nz;
  const ax = ROWS[o + 3] * l, ay = ROWS[o + 4] * l, az = ROWS[o + 5] * l;
  VA[3] -= IA[0] * ax + IA[1] * ay + IA[2] * az;
  VA[4] -= IA[1] * ax + IA[3] * ay + IA[4] * az;
  VA[5] -= IA[2] * ax + IA[4] * ay + IA[5] * az;
  const bx = ROWS[o + 6] * l, by = ROWS[o + 7] * l, bz = ROWS[o + 8] * l;
  VB[3] += IB[0] * bx + IB[1] * by + IB[2] * bz;
  VB[4] += IB[1] * bx + IB[3] * by + IB[4] * bz;
  VB[5] += IB[2] * bx + IB[4] * by + IB[5] * bz;
}

/** One over row o's coupling with itself: its effective mass, or 0 if nothing can move along it. */
function mass(o) {
  const k = coupling(o, o);
  return k > 0 ? 1 / k : 0;
}

/**
 * Solves rows 0 to m-1 (2 or 3) together: λ = -massScale K⁻¹(speed + biasRate
 * C) - impulseScale ACC, into LAMBDA, and applies it. C holds each row's
 * error and ACC its accumulated impulse. Where K is singular, or near enough
 * that its inverse would multiply rounding by more than 2^20 (a body that
 * can't turn about some axes, held to one that can't either), the rows are
 * solved one at a time instead, which needs no inverse.
 */
function block(m, biasRate, massScale, impulseScale) {
  for (let r = 0; r < m; r++) RHS[r] = speed(9 * r) + biasRate * C[r];
  const k00 = coupling(0, 0), k01 = coupling(0, 9), k11 = coupling(9, 9);
  if (m === 2) {
    const det = k00 * k11 - k01 * k01;
    if (det > CONDITION * k00 * k11) {
      LAMBDA[0] = -massScale * (k11 * RHS[0] - k01 * RHS[1]) / det - impulseScale * ACC[0];
      LAMBDA[1] = -massScale * (k00 * RHS[1] - k01 * RHS[0]) / det - impulseScale * ACC[1];
      apply(0, LAMBDA[0]);
      apply(9, LAMBDA[1]);
      return;
    }
  } else {
    const k02 = coupling(0, 18), k12 = coupling(9, 18), k22 = coupling(18, 18);
    // The inverse by cofactors, of a symmetric 3x3.
    const c00 = k11 * k22 - k12 * k12, c01 = k02 * k12 - k01 * k22, c02 = k01 * k12 - k02 * k11;
    const det = k00 * c00 + k01 * c01 + k02 * c02;
    if (det > CONDITION * k00 * k11 * k22) {
      const c11 = k00 * k22 - k02 * k02, c12 = k01 * k02 - k00 * k12, c22 = k00 * k11 - k01 * k01;
      const x = RHS[0], y = RHS[1], z = RHS[2];
      LAMBDA[0] = -massScale * (c00 * x + c01 * y + c02 * z) / det - impulseScale * ACC[0];
      LAMBDA[1] = -massScale * (c01 * x + c11 * y + c12 * z) / det - impulseScale * ACC[1];
      LAMBDA[2] = -massScale * (c02 * x + c12 * y + c22 * z) / det - impulseScale * ACC[2];
      apply(0, LAMBDA[0]);
      apply(9, LAMBDA[1]);
      apply(18, LAMBDA[2]);
      return;
    }
  }
  for (let r = 0; r < m; r++) {
    const k = coupling(9 * r, 9 * r);
    LAMBDA[r] = k > 0 ? -massScale * (speed(9 * r) + biasRate * C[r]) / k - impulseScale * ACC[r] : 0;
    apply(9 * r, LAMBDA[r]);
  }
}
const CONDITION = 9.5367431640625e-7;   // 2^-20

/** The solve pass's softness, or none while relaxing. */
function soft(bias) {
  return bias ? jointSoft : RIGID;
}
const RIGID = { biasRate: 0, massScale: 1, impulseScale: 0 };

/** The anchors held together: a ball joint, and the point of a hinge or fixed joint. */
function holdPoint(acc, o, bias) {
  for (let k = 0; k < 3; k++) {
    point(9 * k, k);
    C[k] = D[k];
    ACC[k] = acc[o + LIN + k];
  }
  const s = soft(bias);
  block(3, s.biasRate, s.massScale, s.impulseScale);
  acc[o + LIN] += LAMBDA[0]; acc[o + LIN + 1] += LAMBDA[1]; acc[o + LIN + 2] += LAMBDA[2];
}

/**
 * b's frame held turned as a's: a fixed joint and a slider. The error is the
 * turn from a's frame to b's, as a world vector 2 sin(θ/2) along its axis:
 * θ itself for a small turn, which is all the solver leaves.
 */
function holdAngles(acc, o, bias) {
  rotate(QA, 0, REL[0], REL[1], REL[2], false, C);
  for (let k = 0; k < 3; k++) {
    C[k] *= 2;
    ACC[k] = acc[o + ANG + k];
    const r = 9 * k;
    ROWS.fill(0, r, r + 9);
    ROWS[r + 3 + k] = 1;
    ROWS[r + 6 + k] = 1;
  }
  const s = soft(bias);
  block(3, s.biasRate, s.massScale, s.impulseScale);
  acc[o + ANG] += LAMBDA[0]; acc[o + ANG + 1] += LAMBDA[1]; acc[o + ANG + 2] += LAMBDA[2];
}

/** A hinge's axes held in line: b's axis may not tip away from a's about either of a's other two. */
function holdAxis(acc, o, bias) {
  const ex = A1[1] * A2[2] - A1[2] * A2[1], ey = A1[2] * A2[0] - A1[0] * A2[2], ez = A1[0] * A2[1] - A1[1] * A2[0];
  turning(0, P1);
  turning(9, P2);
  C[0] = ex * P1[0] + ey * P1[1] + ez * P1[2];
  C[1] = ex * P2[0] + ey * P2[1] + ez * P2[2];
  ACC[0] = acc[o + ANG] * P1[0] + acc[o + ANG + 1] * P1[1] + acc[o + ANG + 2] * P1[2];
  ACC[1] = acc[o + ANG] * P2[0] + acc[o + ANG + 1] * P2[1] + acc[o + ANG + 2] * P2[2];
  const s = soft(bias);
  block(2, s.biasRate, s.massScale, s.impulseScale);
  for (let k = 0; k < 3; k++) acc[o + ANG + k] += LAMBDA[0] * P1[k] + LAMBDA[1] * P2[k];
}

/** A slider's anchor held on a's line: no motion across it, along P1 or P2. */
function holdLine(acc, o, bias) {
  along(0, P1, LEVER);
  along(9, P2, LEVER);
  C[0] = D[0] * P1[0] + D[1] * P1[1] + D[2] * P1[2];
  C[1] = D[0] * P2[0] + D[1] * P2[1] + D[2] * P2[2];
  ACC[0] = acc[o + LIN] * P1[0] + acc[o + LIN + 1] * P1[1] + acc[o + LIN + 2] * P1[2];
  ACC[1] = acc[o + LIN] * P2[0] + acc[o + LIN + 1] * P2[1] + acc[o + LIN + 2] * P2[2];
  const s = soft(bias);
  block(2, s.biasRate, s.massScale, s.impulseScale);
  for (let k = 0; k < 3; k++) acc[o + LIN + k] += LAMBDA[0] * P1[k] + LAMBDA[1] * P2[k];
}

/** A motor on row o: drives its speed to `target`, pushing no harder than `most` (its force times the substep). */
function motor(o, target, most, acc, k) {
  const l = -free(o) * (speed(o) - target), old = acc[k];
  let next = old + l;
  if (next > most) next = most; else if (next < -most) next = -most;
  acc[k] = next;
  pinned(o, next - old, acc);
}

/**
 * A stop on row o: c is how far the joint is from it, positive while short of
 * it; sign is 1 for a lower stop, -1 for an upper. Short of it, the row lets
 * the joint close exactly the gap this substep, no more, as a contact does
 * (contact.js); past it, the solve pass pushes back softly.
 */
function stop(o, c, sign, acc, k, h, bias) {
  // Short of the stop and not reaching it this substep, with nothing pushed
  // last pass: nothing to do.
  const closing = sign * speed(o);
  if (c > 0 && closing * h + c >= 0 && acc[k] === 0) return;
  let target, massScale = 1, impulseScale = 0;
  if (c > 0) {
    target = c / h;
  } else if (bias) {
    target = jointSoft.biasRate * c;
    massScale = jointSoft.massScale;
    impulseScale = jointSoft.impulseScale;
  } else {
    target = 0;
  }
  const l = -free(o) * massScale * (closing + target) - impulseScale * acc[k];
  const old = acc[k], next = old + l > 0 ? old + l : 0;
  acc[k] = next;
  pinned(o, sign * (next - old), acc);
}

// PINNED ROWS. A motor or a stop turns or slides a against b about the
// joint's free axis, but a's centre isn't on that axis: a door's is half its
// width from its hinge. Pushed alone, as a free body, the door would turn
// about its centre, inertia 1.7 kg·m² for a 20 kg 1 m door, and the hinge's
// pin, solved next, would hand back most of the turn; held at the hinge it
// turns about its edge, 6.7 kg·m². Measured, a door swung at 5 m/s ran 0.9 mm
// past its stop, and a slammed one 5 mm. So the row is solved together with
// its own joint's rows that must hold: its effective mass is the one it has
// while they hold, and its impulse comes with the pin's reaction, which keeps
// them holding. Within the one joint that is exact.

/**
 * The current joint's offset into Joints.impulse, its type, and how many of
 * its rows that must hold are pinned at ROWS rows 1 on: -1 until a row first
 * needs them, since most stops, most of the time, are nowhere near.
 */
let base = 0, pins = 0, pinType = 0;
/** What each pinned row is, for keeping its impulse: see pin(). */
const KIND = new Uint8Array(5);
const PT = 0, ANGLE = 3, AXIS1 = 6, AXIS2 = 7, LINE1 = 8, LINE2 = 9;
/** The pinned rows' coupling's inverse, and the rows' share of a free row's impulse. */
const PIN = new Float64Array(25), MU = new Float64Array(5), KF = new Float64Array(5);

/** The joint's rows that must hold, at ROWS rows 1 on, for free() and pinned(); none if they can't be inverted. */
function pin(type) {
  pins = 0;
  if (type === HINGE) {
    turning(9, P1); KIND[0] = AXIS1;
    turning(18, P2); KIND[1] = AXIS2;
    for (let k = 0; k < 3; k++) { point(27 + 9 * k, k); KIND[2 + k] = PT + k; }
    pins = 5;
  } else if (type === SLIDER) {
    for (let k = 0; k < 3; k++) {
      const r = 9 + 9 * k;
      ROWS.fill(0, r, r + 9);
      ROWS[r + 3 + k] = 1;
      ROWS[r + 6 + k] = 1;
      KIND[k] = ANGLE + k;
    }
    along(36, P1, LEVER); KIND[3] = LINE1;
    along(45, P2, LEVER); KIND[4] = LINE2;
    pins = 5;
  } else {
    for (let k = 0; k < 3; k++) { point(9 + 9 * k, k); KIND[k] = PT + k; }
    pins = 3;
  }
  for (let r = 0; r < pins; r++) {
    for (let c = r; c < pins; c++) {
      const v = coupling(9 + 9 * r, 9 + 9 * c);
      PIN[r * pins + c] = v;
      PIN[c * pins + r] = v;
    }
  }
  if (!inverseSPD(PIN, pins, PIN)) pins = 0;
}

/** Row o's effective mass with the pinned rows holding, and their share of its impulse into MU. */
function free(o) {
  if (pins < 0) pin(pinType);
  if (pins === 0) return mass(o);
  let k = coupling(o, o);
  for (let r = 0; r < pins; r++) KF[r] = coupling(o, 9 + 9 * r);
  for (let r = 0; r < pins; r++) {
    let sum = 0;
    for (let c = 0; c < pins; c++) sum += PIN[r * pins + c] * KF[c];
    MU[r] = -sum;
    k += KF[r] * MU[r];
  }
  return k > 0 ? 1 / k : 0;
}

/** Impulse λ along row o, with the pinned rows' share (MU, from free()), each kept in the joint's impulses. */
function pinned(o, l, acc) {
  apply(o, l);
  for (let r = 0; r < pins; r++) {
    const m = MU[r] * l, q = 9 + 9 * r;
    if (m === 0) continue;
    apply(q, m);
    const kind = KIND[r];
    if (kind < ANGLE) acc[base + LIN + kind] += m;
    else if (kind < AXIS1) acc[base + ANG + kind - ANGLE] += m;
    else if (kind < LINE1) for (let c = 0; c < 3; c++) acc[base + ANG + c] += m * ROWS[q + 3 + c];
    else for (let c = 0; c < 3; c++) acc[base + LIN + c] += m * ROWS[q + c];
  }
}

/** The angle a has turned about the joint's axis against b, from as built, between -π and π: twice the turn's half-angle. */
function twist() {
  return -2 * atan2(REL[0], REL[3]);
}

function hingeLimits(J, j, acc, o, h, bias) {
  const lo = low(HINGE, J.given[j], J.min[j], 0, 0), hi = high(HINGE, J.given[j], J.max[j], 0, 0);
  if (lo === -Infinity && hi === Infinity) return;
  const angle = twist();
  axle(0);
  if (lo > -Infinity) stop(0, angle - lo, 1, acc, o + LOWER, h, bias);
  if (hi < Infinity) stop(0, hi - angle, -1, acc, o + UPPER, h, bias);
}

function twistLimits(J, j, acc, o, h, bias) {
  const lo = low(BALL, J.given[j], J.min[j], 0, 0), hi = high(BALL, J.given[j], J.max[j], 0, 0);
  if (lo === -Infinity && hi === Infinity) return;
  if (!twistAxis(0)) return;
  const angle = twist();
  if (lo > -Infinity) stop(0, angle - lo, 1, acc, o + LOWER, h, bias);
  if (hi < Infinity) stop(0, hi - angle, -1, acc, o + UPPER, h, bias);
}

function coneLimit(J, j, acc, o, h, bias) {
  const swing = coneAxis(0);
  if (swing < 0) return;
  stop(0, J.cone[j] - swing, -1, acc, o + CONE, h, bias);
}

function sliderLimits(J, j, acc, o, h, bias) {
  const lo = low(SLIDER, J.given[j], J.min[j], 0, 0), hi = high(SLIDER, J.given[j], J.max[j], 0, 0);
  if (lo === -Infinity && hi === Infinity) return;
  const x = -(D[0] * A1[0] + D[1] * A1[1] + D[2] * A1[2]);
  slide(0);
  if (lo > -Infinity) stop(0, x - lo, 1, acc, o + LOWER, h, bias);
  if (hi < Infinity) stop(0, hi - x, -1, acc, o + UPPER, h, bias);
}

/**
 * A distance joint: a rod while its limits meet, held at that length as a
 * ball joint holds its point; otherwise free between them, pulled to its
 * length by its spring if it has one. The spring is a real force, not the
 * solver correcting itself, so it acts in the relax pass too; Box2D's.
 */
function distance(J, j, acc, o, h, bias, held) {
  const l = stretch(0);
  if (l === 0) return;
  const f = J.frequency[j], length = J.length[j];
  const lo = low(DISTANCE, J.given[j], J.min[j], length, f), hi = high(DISTANCE, J.given[j], J.max[j], length, f);
  if (lo === hi) {
    if (held) return;
    const s = soft(bias), k = o + AXIAL;
    const lambda = -mass(0) * s.massScale * (speed(0) + s.biasRate * (l - lo)) - s.impulseScale * acc[k];
    acc[k] += lambda;
    apply(0, lambda);
    return;
  }
  if (f > 0) {
    makeSpring(f, J.dampingRatio[j], h);
    const k = o + AXIAL;
    const lambda = -mass(0) * spring.massScale * (speed(0) + spring.biasRate * (l - length)) - spring.impulseScale * acc[k];
    acc[k] += lambda;
    apply(0, lambda);
  }
  if (lo > 0) stop(0, l - lo, 1, acc, o + LOWER, h, bias);
  if (hi < Infinity) stop(0, hi - l, -1, acc, o + UPPER, h, bias);
}

/** Box2D's soft constraint from a frequency and a damping ratio over h, as contact.js's makeSoft, into `spring`. */
function makeSpring(hertz, zeta, h) {
  const omega = 2 * Math.PI * hertz;
  const a1 = 2 * zeta + h * omega, a2 = h * omega * a1, a3 = 1 / (1 + a2);
  spring.biasRate = omega / a1;
  spring.massScale = a2 * a3;
  spring.impulseScale = a3;
}

/**
 * Takes out of the world vector at acc[i..] its part along the joint's axis
 * now (A1): a hinge's turning rows and a slider's sliding rows hold nothing
 * along it, and keep their impulse as a vector at right angles to it. When
 * the whole joint turns, so does its axis, and part of the old vector would
 * lie along the new one: a push along the very way the joint leaves free,
 * which nothing would ever take back. Kept, it spun a ragdoll's swinging
 * shin up from 25 to 90 rad/s in two steps.
 */
function flatten(acc, i) {
  const d = acc[i] * A1[0] + acc[i + 1] * A1[1] + acc[i + 2] * A1[2];
  acc[i] -= d * A1[0]; acc[i + 1] -= d * A1[1]; acc[i + 2] -= d * A1[2];
}

/** Applies an angular impulse kept as a world vector at acc[i..]: the turning rows' warm start. */
function angles(acc, i) {
  for (let k = 0; k < 3; k++) {
    const r = 9 * k;
    ROWS.fill(0, r, r + 9);
    ROWS[r + 3 + k] = 1;
    ROWS[r + 6 + k] = 1;
    apply(r, acc[i + k]);
  }
}

// ---- small maths ----

/** out[o..o+2] = r × e_k, for r a 3-array and e_k world axis k. */
function cross(r, k, out, o) {
  if (k === 0) { out[o] = 0; out[o + 1] = r[2]; out[o + 2] = -r[1]; } else if (k === 1) { out[o] = -r[2]; out[o + 1] = 0; out[o + 2] = r[0]; } else { out[o] = r[1]; out[o + 1] = -r[0]; out[o + 2] = 0; }
}

/** out[oo..] = a ⊗ b, quaternions at a[ao..] and b[bo..]. out may be a or b. */
function multiply(a, ao, b, bo, out, oo) {
  const ax = a[ao], ay = a[ao + 1], az = a[ao + 2], aw = a[ao + 3];
  const bx = b[bo], by = b[bo + 1], bz = b[bo + 2], bw = b[bo + 3];
  out[oo] = aw * bx + ax * bw + ay * bz - az * by;
  out[oo + 1] = aw * by + ay * bw + az * bx - ax * bz;
  out[oo + 2] = aw * bz + az * bw + ax * by - ay * bx;
  out[oo + 3] = aw * bw - ax * bx - ay * by - az * bz;
}

/** out[oo..] = conj(a) ⊗ b. */
function conjugateTimes(a, ao, b, bo, out, oo) {
  const ax = -a[ao], ay = -a[ao + 1], az = -a[ao + 2], aw = a[ao + 3];
  const bx = b[bo], by = b[bo + 1], bz = b[bo + 2], bw = b[bo + 3];
  out[oo] = aw * bx + ax * bw + ay * bz - az * by;
  out[oo + 1] = aw * by + ay * bw + az * bx - ax * bz;
  out[oo + 2] = aw * bz + az * bw + ax * by - ay * bx;
  out[oo + 3] = aw * bw - ax * bx - ay * by - az * bz;
}

/** The x, y and z axes of the rotation q, into x, y and z. */
function axes(q, x, y, z) {
  const qx = q[0], qy = q[1], qz = q[2], qw = q[3];
  x[0] = 1 - 2 * (qy * qy + qz * qz); x[1] = 2 * (qx * qy + qz * qw); x[2] = 2 * (qx * qz - qy * qw);
  y[0] = 2 * (qx * qy - qz * qw); y[1] = 1 - 2 * (qx * qx + qz * qz); y[2] = 2 * (qy * qz + qx * qw);
  z[0] = 2 * (qx * qz + qy * qw); z[1] = 2 * (qy * qz - qx * qw); z[2] = 1 - 2 * (qx * qx + qy * qy);
}

/** Column k of the rotation q, as a fresh array. */
function column(q, k) {
  axes(q, A1, P1, P2);
  const c = k === 0 ? A1 : k === 1 ? P1 : P2;
  return [c[0], c[1], c[2]];
}
