// Joints: what holds two bodies to each other, or one to a point in the world.
//
// A free body can move in six ways: along three axes and about three. Each is
// a degree of freedom. A joint takes some away and leaves the rest: a hinge
// leaves one, turning about its axis; a slider one, moving along it; a ball
// joint three, every turn about its point; a fixed joint none. A distance
// joint takes away one, the gap between two points; a rotation joint three,
// every turn, and leaves a's position entirely free.
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
// THE SOFT STEP. Joints are solved in the same substeps as contacts, after
// them, with the same warm start, solve and relax (contact.js): each substep
// starts from the impulses the last one ended with (but for some trees' rows:
// see warmStartJoints()), the solve pass pulls the anchors
// together, and the relax pass takes back the speed that pulling left behind,
// so correcting a joint adds no energy. Anchors and axes are worked out afresh
// from the bodies' rotations at every solve pass, so a joint that turns
// quickly stays exact.
//
// DRIVES. A motor given a target is a spring toward it, solved as a row
// like any other: its speed is asked to be the target's own speed less
// biasRate times how far the joint is from the target, softened by Box2D's
// soft step (makeSpring(), a frequency and a damping ratio), and clamped to
// the motor's force. Pushed from outside, as a torque worked out each frame
// from the error, a stiff spring overshoots: by the time the torque is
// applied the error it was worked out from has changed, and stiffer than
// about the step rate it gains energy each step and blows up. Inside the
// solver the push is worked out with the row's effective mass, against the
// velocity the push itself makes, so it is steady at any stiffness, as a
// contact is: a frequency of 0 is as stiff as the solver goes, as stiff as
// a stop pushed back (soften()), still no harder than the motor's force.
// A hinge's or slider's drive is its motor row, pinned to its
// joint's rows that must hold (PINNED ROWS, below); a ball or rotation
// joint's is three turning rows solved together (turnDrive()), clamped as
// one vector, as ball joint friction is.
//
// STORAGE. Joints have slots and generational ids as bodies do
// (core/slots.js), and their data in columns. Each body keeps a list of its
// joints, through two of these columns, so removing a body can remove its
// joints, and the pair filter can see that two bodies are joined.

import { fail, show, finite, vec3, direction, quat, oneOf, options, known, readOnly } from './core/check.js';
import { atan2, rotate } from './core/math.js';
import { Slots, resized, MAX_SLOTS } from './core/slots.js';
import { Body } from './body.js';
import { Forest, plant, factorForest, solveForest, inverseSPD, NODE } from './forest.js';
import { PIXEL } from './contact.js';

export const TYPES = ['fixed', 'ball', 'hinge', 'slider', 'distance', 'rotation'];
const FIXED = 0, BALL = 1, HINGE = 2, SLIDER = 3, DISTANCE = 4;
/** A rotation joint's code: debug.js draws it, holding no point, its own way. */
export const ROTATION = 5;
const DYNAMIC = 0;   // World's type code
const USED = 1;      // Slots' state for a slot in use

const OPTIONS = ['type', 'a', 'b', 'anchor', 'anchorB', 'axis', 'min', 'max', 'length', 'cone', 'target',
  'motorSpeed', 'motorForce', 'friction', 'frequency', 'dampingRatio', 'collide'];
/** Options every joint has. */
const SHARED = ['type', 'a', 'b', 'collide'];
/** And each type's own; giving one another type has throws. A rotation joint holds no point, so has no anchors. */
const DRIVE = ['target', 'motorForce', 'friction', 'frequency', 'dampingRatio'];
const OWN = [
  ['anchor', 'anchorB'],
  ['anchor', 'anchorB', 'axis', 'min', 'max', 'cone', ...DRIVE],
  ['anchor', 'anchorB', 'axis', 'min', 'max', 'motorSpeed', ...DRIVE],
  ['anchor', 'anchorB', 'axis', 'min', 'max', 'motorSpeed', ...DRIVE],
  ['anchor', 'anchorB', 'length', 'min', 'max', 'frequency', 'dampingRatio'],
  ['axis', 'cone', ...DRIVE],
];

// What `given` records: whether min, max, anchorB and a target were given, or
// follow their defaults (which for a distance joint follow its length and
// spring; with no target, a motor drives a speed).
const MIN = 1, MAX = 2, ANCHOR_B = 4, TARGET = 8;

// Where each accumulated impulse is kept, in a joint's 11: the linear and
// angular impulses as world vectors, then one number per scalar row. A ball
// joint holds nothing by turning, so its ANG is its friction's (ballFriction()),
// and a hinge's or slider's friction is its MOTOR row's.
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
  ['friction', Float64Array, 1],
  ['frequency', Float64Array, 1],
  ['dampingRatio', Float64Array, 1],
  // The target, if `given` has TARGET: a hinge's angle or a slider's position
  // in the first, a ball or rotation joint's turn of a against b, in b's own
  // axes (the world's with no b), as a unit quaternion [x, y, z, w].
  ['target', Float64Array, 4],
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
    // Every column named first, then made: an object given more than a dozen
    // fields by computed name becomes a dictionary (World's FAST PROPERTIES).
    this.type = null; this.a = null; this.b = null; this.localA = null; this.localB = null;
    this.frameA = null; this.frameB = null; this.min = null; this.max = null; this.given = null;
    this.length = null; this.cone = null; this.motorSpeed = null; this.motorForce = null;
    this.friction = null; this.frequency = null; this.dampingRatio = null; this.target = null;
    this.collide = null; this.next = null; this.impulse = null; this.list = null;
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
  world._pairEpoch++;   // its bodies may stop colliding (pairs.js holds)
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
  world._pairEpoch++;   // collide may change (pairs.js holds)
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
  const limits = spec.min !== base.min || spec.max !== base.max || (spec.given & (MIN | MAX)) !== (base.given & (MIN | MAX)) || spec.cone !== base.cone;
  if (type !== was || anchor || anchorB || axis) J.impulse.fill(0, j * IMPULSES, j * IMPULSES + IMPULSES);
  else if (limits) { const i = j * IMPULSES; J.impulse[i + LOWER] = 0; J.impulse[i + UPPER] = 0; J.impulse[i + CONE] = 0; }
  J.type[j] = type;
  write(J, j, spec);
  // A motor or friction switched off pushes no more. Its impulse, kept for
  // the warm start, would otherwise be applied at every substep with no row
  // left to solve it back: a wheel whose motor was turned off went on
  // speeding up. A fixed joint's ANG holds it together, so only a ball's goes;
  // a rotation joint's holds it again, with no cone row left to solve.
  const i = j * IMPULSES;
  if (spec.motorForce === 0 && spec.friction === 0) {
    J.impulse[i + MOTOR] = 0;
    if (type === BALL) { J.impulse[i + ANG] = 0; J.impulse[i + ANG + 1] = 0; J.impulse[i + ANG + 2] = 0; }
    if (type === ROTATION) J.impulse[i + CONE] = 0;
  }
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
  if (type !== ROTATION) out.anchor = worldAnchor(world, j, 0);
  if (type === DISTANCE || (type !== ROTATION && (J.given[j] & ANCHOR_B) !== 0)) out.anchorB = worldAnchor(world, j, 1);
  if (type !== FIXED && type !== DISTANCE) {
    QA.set(world._rotation.subarray(4 * a, 4 * a + 4));
    multiply(QA, 0, J.frameA, 4 * j, QA, 0);
    out.axis = column(QA, 0);
  }
  if (type === DISTANCE) out.length = s.length;
  if (type !== FIXED && type !== ROTATION) {
    if (s.given & MIN) out.min = s.min;
    if (s.given & MAX) out.max = s.max;
  }
  if (type === BALL || type === ROTATION) out.cone = s.cone;
  if (type !== FIXED && type !== DISTANCE) out.target = s.target;
  if (type === HINGE || type === SLIDER) out.motorSpeed = s.motorSpeed;
  if (type !== FIXED && type !== DISTANCE) { out.motorForce = s.motorForce; out.friction = s.friction; }
  if (type !== FIXED) { out.frequency = s.frequency; out.dampingRatio = s.dampingRatio; }
  out.collide = J.collide[j] === 1;
  return out;
}

/** Takes joint slot j out: unlinked from its bodies, which wake, and its slot freed. */
export function dropJoint(world, j) {
  world._pairEpoch++;   // its bodies may collide again (pairs.js holds)
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
  // No target: a motor drives a speed.
  target: null,
  // Turning freely, as Jolt's, Rapier's and Box2D's joints do unless given it.
  friction: 0,
  // No spring: a distance joint a rod, a drive as stiff as it can be. Given
  // one, it settles without overshooting, as Bearing's contacts do, unless
  // told to bounce.
  frequency: 0, dampingRatio: 1,
});

/** A joint's type-specific settings, from its columns. */
function read(J, j) {
  const t = 4 * j, turned = J.type[j] === BALL || J.type[j] === ROTATION;
  const target = (J.given[j] & TARGET) === 0 ? null
    : turned ? [J.target[t], J.target[t + 1], J.target[t + 2], J.target[t + 3]] : J.target[t];
  return {
    min: J.min[j], max: J.max[j], given: J.given[j] & (MIN | MAX | TARGET), length: J.length[j], cone: J.cone[j], target,
    motorSpeed: J.motorSpeed[j], motorForce: J.motorForce[j], friction: J.friction[j], frequency: J.frequency[j], dampingRatio: J.dampingRatio[j],
  };
}

function write(J, j, s) {
  J.min[j] = s.min; J.max[j] = s.max; J.length[j] = s.length; J.cone[j] = s.cone;
  J.motorSpeed[j] = s.motorSpeed; J.motorForce[j] = s.motorForce; J.friction[j] = s.friction;
  J.frequency[j] = s.frequency; J.dampingRatio[j] = s.dampingRatio;
  const t = s.target, o = 4 * j;
  if (t === null) J.target.fill(0, o, o + 4);
  else if (typeof t === 'number') { J.target[o] = t; J.target[o + 1] = 0; J.target[o + 2] = 0; J.target[o + 3] = 0; }
  else J.target.set(t, o);
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
  // null is a value, as for min and max: no target again.
  if (o.target === null) s.target = null;
  else if (has('target')) s.target = aim(caller, type, o.target);
  s.given = (s.given & ~TARGET) | (s.target !== null ? TARGET : 0);
  if (has('motorSpeed')) s.motorSpeed = finite(caller, 'motorSpeed', o.motorSpeed);
  if (has('motorForce')) s.motorForce = atLeastZero(caller, 'motorForce', o.motorForce);
  if (has('friction')) s.friction = atLeastZero(caller, 'friction', o.friction);
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

/** A target, checked for its type: an angle for a hinge, metres for a slider, a rotation for a ball or rotation joint, normalized. */
function aim(caller, type, v) {
  if (type === HINGE) {
    if (typeof v !== 'number' || !(v >= -Math.PI && v <= Math.PI)) fail(caller, `target must be from -π to π, in radians, got ${show(v)}`);
    return v;
  }
  if (type === SLIDER) return finite(caller, 'target', v);
  return quat(caller, 'target', v);
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

/**
 * The warm start, at the start of each substep, after the contacts': every
 * solved joint's accumulated impulses applied again, but for the rows of a
 * tree held down and touching something (forest.js, plant()), which start
 * from none.
 *
 * WHY NOT THOSE. The tree is solved exactly in every pass, so what its rows
 * start from changes nothing it ends with; a replay is for the contacts, so
 * they see a body already carrying what its joints carry. That holds while
 * the joints' impulses balance the contacts' replayed beside them, and a
 * chain held by a kinematic body and driven against something it can't push
 * aside (a post, a heavy box, a chain hung from the world) breaks it. The
 * contact sees the link as a light free body, though the joints tie it to one
 * that can't be stopped, so its push never stops the closing and grows substep
 * after substep, the joints' growing with it: thousands of N·s each, in
 * balance. Replayed, the contact's acts at levers taken when the step began
 * and the joints' at anchors that have moved since, and a narrowphase that
 * relabels the points drops the contact's altogether: on a thin link spinning
 * at a few hundred rad/s, two such impulses a few degrees apart turned it at
 * thousands of rad/s before the contacts were solved. The contacts can only
 * push, and what they pushed along the way the chain is free to swing
 * survived the tree solve, so the next replay was further out still.
 * Measured, eight links hung from a plate turning at 0.5 to 10 rad/s, among
 * chains hung from the world and a heap of boxes: flung past 1,000 m/s at
 * 1.5, 2, 3, 5 and 7 rad/s, and swept into a post at 3 rad/s, 640,000 m/s.
 *
 * Only there does it break: a free tree (a ragdoll) gives way, so its
 * contacts' pushes don't grow, and started from none, ragdolls on a floor
 * rattled, 11 of 352 parts asleep at 12 s against 66; and with no contact
 * there is nothing to fall out of balance with, while a stop, a motor or
 * friction, solved before the tree, needs the load replayed (a ball joint's
 * friction let a pendulum it should hold fall 22 cm). Solving the tree at the
 * warm start instead, so the contacts start from velocities its joints
 * accept, did as well everywhere (154 ragdoll parts asleep) but cost a build
 * of the rows and a solve a substep, 15% of a jointed scene.
 */
export function warmStartJoints(world) {
  const J = world._joints, list = J.list, n = J.active, acc = J.impulse, tree = J.forest.inTree;
  for (let i = 0; i < n; i++) {
    const j = list[i], type = J.type[j], o = j * IMPULSES;
    // A held-down, touching tree's rows start each substep from none; the forest's
    // solves add this substep's (settleForest), kept for save() and for a
    // joint that leaves the tree. The pinned rows' share of a stop's or a
    // motor's push, kept with them, goes too: the forest holds those rows
    // whatever pushes.
    if (tree[j] === 3) {
      acc[o + LIN] = 0; acc[o + LIN + 1] = 0; acc[o + LIN + 2] = 0;
      // A ball joint's ANG is its friction's, which may give, so is kept.
      if (type !== BALL) { acc[o + ANG] = 0; acc[o + ANG + 1] = 0; acc[o + ANG + 2] = 0; }
      if (type === DISTANCE) acc[o + AXIAL] = 0;
    }
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
    } else if (type === ROTATION) {
      // What holds it, or drives it, or its friction: all one angular impulse.
      angles(acc, o + ANG);
      if (gives(J, j) && J.cone[j] < Math.PI) {
        frames(world, j);
        if (coneAxis(0) >= 0) apply(0, -acc[o + CONE]);
      }
    } else {
      for (let k = 0; k < 3; k++) {
        point(9 * k, k);
        apply(9 * k, acc[o + LIN + k]);
      }
      if (type === HINGE) {
        frames(world, j);
        flatten(acc, o + ANG);
      }
      if (type === FIXED || type === HINGE || (type === BALL && gives(J, j))) angles(acc, o + ANG);
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
 * is solved against everything else's latest push: Box2D's order. `again`
 * is true after a step's first substep, when the trees may keep their factor
 * (holdForest()).
 */
export function solveJoints(world, h, bias, again = false) {
  const J = world._joints, list = J.list, n = J.active, acc = J.impulse, tree = J.forest.inTree;
  for (let i = 0; i < n; i++) {
    const j = list[i], o = j * IMPULSES, type = J.type[j], held = tree[j] !== 0;
    // A fixed joint in a tree has nothing that gives: the forest holds it all.
    // Nor has a rotation joint there, which is only in a tree while it holds.
    if (held && (type === FIXED || type === ROTATION)) continue;
    base = o;
    pins = type === FIXED || type === DISTANCE || type === ROTATION ? 0 : -1;
    pinType = type;
    load(world, j);
    geometry(world, j);
    switch (type) {
      case FIXED:
        frames(world, j);
        holdAngles(acc, o, bias);
        holdPoint(acc, o, bias);
        break;
      case BALL: {
        // Its motor with no target holds it from turning, as friction does: a
        // hinge's at motorSpeed 0 (driven()), and a ball joint has no speed.
        const aims = aimed(J, j), most = J.motorForce[j] > 0 ? J.motorForce[j] : J.friction[j];
        if (aims) {
          frames(world, j);
          turnDrive(world, J, j, most * h, h, bias, acc, o);
        } else if (most > 0) ballFriction(most * h, acc, o);
        if (swings(J, j)) {
          if (!aims) frames(world, j);
          twistLimits(J, j, acc, o, h, bias);
          if (J.cone[j] < Math.PI) coneLimit(J, j, acc, o, h, bias);
        }
        if (!held) holdPoint(acc, o, bias);
        break;
      }
      case HINGE:
        frames(world, j);
        if (J.motorForce[j] > 0 || J.friction[j] > 0) {
          axle(0);
          driven(world, j, h, acc, o, bias);
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
        if (J.motorForce[j] > 0 || J.friction[j] > 0) {
          slide(0);
          driven(world, j, h, acc, o, bias);
        }
        sliderLimits(J, j, acc, o, h, bias);
        if (!held) {
          holdAngles(acc, o, bias);
          holdLine(acc, o, bias);
        }
        break;
      case ROTATION:
        frames(world, j);
        if (gives(J, j)) {
          const most = J.motorForce[j] > 0 ? J.motorForce[j] : J.friction[j];
          if (aimed(J, j)) turnDrive(world, J, j, most * h, h, bias, acc, o);
          else ballFriction(most * h, acc, o);
          if (J.cone[j] < Math.PI) coneLimit(J, j, acc, o, h, bias);
        } else {
          holdAngles(acc, o, bias);
        }
        break;
      default:
        distance(J, j, acc, o, h, bias, held);
    }
    save(world, j);
  }
  if (J.forest.count > 0) holdForest(world, h, bias, again);
}

/**
 * Whether joint slot j has rows that must hold, and so can be in a tree: all
 * but a distance joint with room to stretch, and a rotation joint with a
 * motor or friction, all of whose rows may give.
 */
function holds(J, j) {
  if (J.type[j] === ROTATION) return !gives(J, j);
  if (J.type[j] !== DISTANCE) return true;
  const f = J.frequency[j], length = J.length[j];
  return low(DISTANCE, J.given[j], J.min[j], length, f) === high(DISTANCE, J.given[j], J.max[j], length, f);
}

/** This step's trees, from the joints gather() listed: see forest.js. */
export function plantJoints(world) {
  PLANTING = world._joints;
  plant(world, world._joints.list, world._joints.active, planted, pointed);
}
// plant()'s tests, made once: a closure made in each step would be the
// allocation in step() that rule 76 forbids. A step isn't re-entered.
let PLANTING = null;
const planted = (j) => holds(PLANTING, j);
const pointed = (j) => PLANTING.type[j] !== ROTATION;

/**
 * Every tree joint's rows that must hold, solved exactly together
 * (forest.js). The solve pass builds the rows from where the bodies are,
 * factors them, and asks each for its whole error back this substep,
 * JOINT_BIAS of it; the relax pass, after the bodies have moved, solves the
 * same rows again for no speed along them.
 *
 * ONCE A STEP. Factoring was 15% of a step of 32 ragdolls, and 5% more in
 * its inverses, so it is done at the first substep and kept for the rest, as
 * the contacts' effective masses are (contact.js, prepare()), but for a tree
 * a body of which has turned too far since (age()). A kept factor keeps both
 * the rows it was made from (F.jac) and the inertias (each body's block):
 * it is then the exact solve of the joints as they stood, and a factor of
 * one pose with rows of another solves nothing exactly (rows built afresh
 * with the step's first inertia opened a whipped chain by metres, see
 * turnedInertias() in contact.js). What each pass asks for is measured
 * along this pass's rows, from where the anchors are now, so the pass still
 * aims at the joints as they are, and the kept factor only answers it a
 * little off, by about the angle turned: what that leaves, the next pass
 * takes up. Asked along the kept rows instead, a pass held the joints as
 * they had stood and took the free swing across them for error: a floating
 * leg kept 60% of its energy over a minute, not 93%.
 */
function holdForest(world, h, bias, again) {
  const J = world._joints, F = J.forest, T = F.count;
  if (bias) {
    age(world, !again);
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

/**
 * How far a body may turn before its tree's factor is made again: θ with θ²
 * = PIXEL, 0.071 rad, as the cosine of its half, |q·q₀| ≥ cos(θ/2) =
 * 1 - θ²/8 (the next term, θ⁴/384, moves θ by under 10^-4 of itself). A kept
 * factor answers a pull as the bodies stood when it was made: a lever turned
 * θ since is off by θ of its length, and in the substep after, which turns it
 * by no more than θ again or the factor is made afresh, the anchor it misses
 * strays by about that error times that turn, |r| θ², within PIXEL of the
 * lever, as a contact is held to PIXEL of a body's size. Measured on the
 * chain whipped at 4 to 12 rad/s (test/joints.test.js), ball and hinged:
 * the widest gaps as before (medians 16.9 and 18.3 mm against 15.8 and 18.2);
 * at 0.1 rad a ball chain opened 234 mm at 11 rad/s, at 0.2 a hinged one
 * 820 mm at 10, and factored once a step whatever turned, hanging chains
 * blew apart.
 */
const KEPT = 1 - PIXEL / 8;

/**
 * Marks each tree in F.fresh to be factored at this solve pass, or to keep
 * its factor: `all` at a step's first substep, and after that a tree one of
 * whose bodies has turned past KEPT since its factor was made, whose
 * rotations F.since keeps, and every substep a tree held at two or more
 * points. Such a tree can have rows that repeat one another (a bridge fixed
 * at both ends, a door on two hinges), which the factor holds by the give
 * (forest.js, GIVE), a million times stiffer along their repeat than the
 * joints themselves: what this pass's rows ask along it, rows a factor made
 * at another pose doesn't see repeat, is answered a million times over. A
 * bridge of 20 planks opened 2.3 m. Held at one point or none, no tree's
 * rows repeat (each joint's rows hold its child, rows apart), and an error
 * of θ stays one of about θ. A rotation joint to the world counts, though it
 * holds no point: its rows hold the body's turn to the world's, and repeat
 * any other joint's that does too (a hinge to the world beside it).
 */
function age(world, all) {
  const F = world._joints.forest, T = F.count, order = F.order, parent = F.parent, side = F.side, fresh = F.fresh, since = F.since, q = world._rotation;
  // Each tree's nodes lie together in the order, its root, a body, first.
  let end = F.nodes;
  for (let i = end - 1; i >= 0; i--) {
    if (parent[order[i]] !== -1) continue;
    let again = all, held = 0;
    for (let k = i; k < end; k++) {
      const x = order[k];
      if (x < T) {
        if (side[2 * x] < 0 || side[2 * x + 1] < 0) held++;
      } else if (!again) {
        const o = 4 * (x - T), s4 = 4 * F.body[x - T];
        const dot = since[o] * q[s4] + since[o + 1] * q[s4 + 1] + since[o + 2] * q[s4 + 2] + since[o + 3] * q[s4 + 3];
        again = !(Math.abs(dot) >= KEPT);
      }
    }
    if (held > 1) again = true;
    for (let k = i; k < end; k++) {
      const x = order[k];
      fresh[x] = again ? 1 : 0;
      if (again && x >= T) {
        const o = 4 * (x - T), s4 = 4 * F.body[x - T];
        since[o] = q[s4]; since[o + 1] = q[s4 + 1]; since[o + 2] = q[s4 + 2]; since[o + 3] = q[s4 + 3];
      }
    }
    end = i;
  }
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
    if (type === FIXED || type === SLIDER || type === ROTATION) {
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
    // The directions the impulses were applied along are the factor's rows,
    // b's side of F.jac (forest.js, shape()), which a kept factor holds from
    // when it was made (age()); F.row is this pass's.
    const j = F.joint[t], type = J.type[j], o = j * IMPULSES, jac = F.jac, r = 72 * t + 36;
    const l = NODE * t;
    if (type === FIXED || type === SLIDER || type === ROTATION) {
      for (let k = 0; k < 3; k++) acc[o + ANG + k] += vec[l + k];
      if (type === ROTATION) continue;
      if (type === SLIDER) {
        // The line's rows, along P1 and P2: their impulse as a world vector.
        for (let k = 0; k < 3; k++) acc[o + LIN + k] += vec[l + 3] * jac[r + 18 + k] + vec[l + 4] * jac[r + 24 + k];
      } else {
        for (let k = 0; k < 3; k++) acc[o + LIN + k] += vec[l + 3 + k];
      }
    } else if (type === HINGE) {
      for (let k = 0; k < 3; k++) acc[o + ANG + k] += vec[l] * jac[r + 3 + k] + vec[l + 1] * jac[r + 9 + k];
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
  // Copied one by one, as is everything the step copies: subarray() makes a
  // view object a call, and this runs for every row of every pass.
  const row = F.row, r = 54 * t + 9 * m;
  for (let k = 0; k < 9; k++) row[r + k] = ROWS[o + k];
  F.rhs[6 * t + m] = -(speed(o) + rate * c);
  F.scale[t] += coupling(o, o);
  return m + 1;
}

/**
 * Joint j's bodies into the scratch: a point in the world is a body that never
 * moves. Their inverse inertias are worked out where they have turned to now,
 * as the anchors are (contact.js, turnedInertias()), and nothing but a
 * dynamic body turns under a push.
 */
function load(world, j) {
  const J = world._joints, a = J.a[j], b = J.b[j], v = world._velocity, w = world._angular;
  const invI = world._invIt, type = world._type;
  const a3 = 3 * a, a6 = 6 * a;
  VA[0] = v[a3]; VA[1] = v[a3 + 1]; VA[2] = v[a3 + 2]; VA[3] = w[a3]; VA[4] = w[a3 + 1]; VA[5] = w[a3 + 2];
  if (type[a] === DYNAMIC) for (let k = 0; k < 6; k++) IA[k] = invI[a6 + k];
  else IA.fill(0);
  ima = world._invMass[a];
  if (b < 0) {
    VB.fill(0);
    IB.fill(0);
    imb = 0;
    return;
  }
  const b3 = 3 * b, b6 = 6 * b;
  VB[0] = v[b3]; VB[1] = v[b3 + 1]; VB[2] = v[b3 + 2]; VB[3] = w[b3]; VB[4] = w[b3 + 1]; VB[5] = w[b3 + 2];
  if (type[b] === DYNAMIC) for (let k = 0; k < 6; k++) IB[k] = invI[b6 + k];
  else IB.fill(0);
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
  else for (let k = 0; k < 4; k++) QB[k] = J.frameB[4 * j + k];
  axes(QA, A1, P1, P2);
  const x = QB[0], y = QB[1], z = QB[2], w = QB[3];
  A2[0] = 1 - 2 * (y * y + z * z); A2[1] = 2 * (x * y + z * w); A2[2] = 2 * (x * z - y * w);
  conjugateTimes(QA, 0, QB, 0, REL, 0);
  if (REL[3] < 0) { REL[0] = -REL[0]; REL[1] = -REL[1]; REL[2] = -REL[2]; REL[3] = -REL[3]; }
}

/**
 * Whether a ball or rotation joint has rows that may give: a motor or
 * friction. A rotation joint without either holds a's turn against b fixed.
 */
function gives(J, j) {
  return J.motorForce[j] > 0 || J.friction[j] > 0;
}

/** Whether joint slot j drives toward a target: given one, and a motor to drive it (motorForce 0 is a motor off). */
function aimed(J, j) {
  return (J.given[j] & TARGET) !== 0 && J.motorForce[j] > 0;
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
 * A hinge's or slider's driven row, at row 0: its motor, or with the motor
 * off, its friction, which is a motor held to speed 0 that pushes no harder
 * than `friction`, as Jolt's mMaxFrictionTorque is. A motor already resists
 * up to its own force, so friction adds nothing to one. A motor given a
 * target drives toward it (drive()), the target moving at motorSpeed.
 */
function driven(world, j, h, acc, o, bias) {
  const J = world._joints, force = J.motorForce[j];
  if (force > 0 && (J.given[j] & TARGET) !== 0) {
    world._drives++;
    const t = 4 * j, rate = J.motorSpeed[j], hinge = J.type[j] === HINGE;
    // The target moves at motorSpeed, carried on a substep as the relax pass
    // begins: that pass measures the joint where the substep's move left it,
    // so the target has to be where it is at that moment too, and the next
    // solve pass, from the same place, finds it there. Left where it was set,
    // a target the bodies kept up with would be a substep's travel behind them
    // at every pass after the first, and the spring would hold them there.
    if (!bias && rate !== 0) J.target[t] = hinge ? round(J.target[t] + rate * h) : J.target[t] + rate * h;
    const error = hinge ? round(twist() - J.target[t]) : -(D[0] * A1[0] + D[1] * A1[1] + D[2] * A1[2]) - J.target[t];
    soften(J, j, h, bias);
    drive(0, error, rate, force * h, acc, o + MOTOR);
  } else if (force > 0) motor(0, J.motorSpeed[j], force * h, acc, o + MOTOR);
  else motor(0, 0, J.friction[j] * h, acc, o + MOTOR);
}

/**
 * An angle brought within -π to π, the short way round: a hinge's angle and
 * its target are each within that, so one turn either way is enough.
 */
function round(angle) {
  return angle > Math.PI ? angle - 2 * Math.PI : angle < -Math.PI ? angle + 2 * Math.PI : angle;
}

/**
 * The drive's softness this pass, into `spring`: with a frequency, the
 * spring it makes over the substep, which is a real force and so acts in the
 * relax pass too, as a distance joint's spring does. With none, as stiff as
 * any row solved one joint at a time can be: a stop pushed back's softness
 * (jointSoft, JOINT_STIFFNESS of the substep rate) while solving, and none
 * while relaxing, which takes back the speed that pulling left behind. Not
 * the tree's JOINT_BIAS, the whole error asked back each substep: a drive is
 * solved one joint at a time, not with the tree, and so stiff, the ragdoll
 * standing on every joint driven hopped about the floor at up to 1 m/s and
 * 5 rad/s and wandered 1.5 m in 10 s; at JOINT_STIFFNESS it stands within
 * 0.32 mm, and a level 1 m arm held by it sags 2.6e-5 rad.
 */
function soften(J, j, h, bias) {
  const f = J.frequency[j];
  if (f > 0) {
    makeSpring(f, J.dampingRatio[j], h);
  } else if (bias) {
    spring.biasRate = jointSoft.biasRate;
    spring.massScale = jointSoft.massScale;
    spring.impulseScale = jointSoft.impulseScale;
  } else {
    spring.biasRate = 0;
    spring.massScale = 1;
    spring.impulseScale = 0;
  }
}

/**
 * A drive on row o, softened by `spring` (soften()): asks the row's speed to
 * be the target's own, `rate`, less biasRate times `error`, how far past its
 * target the joint is along the row, and pushes no harder than `most`, the
 * motor's force times the substep, either way. Box2D's soft step with its
 * motor's clamp, pinned to the joint's rows that must hold as a motor is.
 */
function drive(o, error, rate, most, acc, k) {
  const old = acc[k];
  const l = -free(o) * spring.massScale * (speed(o) - rate + spring.biasRate * error) - spring.impulseScale * old;
  let next = old + l;
  if (next > most) next = most; else if (next < -most) next = -most;
  acc[k] = next;
  pinned(o, next - old, acc);
}

/**
 * A ball joint's friction: the three rows of a's turning against b, about the
 * world's axes, solved together with the joint's point held (PINNED ROWS,
 * below), driven to no relative turning, and clamped as one vector to a ball
 * of radius `most` (friction times the substep). The ball, not a box per
 * axis, so the joint resists the same way whichever way it turns, however
 * the scene is turned: a box would let a diagonal turn push √3 times as hard.
 * The impulse is kept in ANG as a world vector, as a fixed joint keeps its
 * turning rows', so the warm start applies it with angles().
 */
function ballFriction(most, acc, o) {
  // λ = -K⁻¹ v: what stops the turning this pass. Where K can't be inverted
  // (a body locked about some axes, held to one that can't turn), each row
  // is solved alone instead, a row that nothing can turn along taking none.
  // ponytail: rows alone are only Jacobi; solve the free 2x2 if partly
  // locked ball joints with friction ever need it exact.
  if (spinMass()) {
    for (let f = 0; f < 3; f++) SPUN[f] = -(K3[3 * f] * RHS[0] + K3[3 * f + 1] * RHS[1] + K3[3 * f + 2] * RHS[2]);
  } else {
    for (let f = 0; f < 3; f++) SPUN[f] = K3[4 * f] > 0 ? -RHS[f] / K3[4 * f] : 0;
  }
  spun(most, acc, o);
}

/**
 * A ball or rotation joint's drive: ballFriction()'s three rows of a turning
 * against b, asked to bring a to its target turn (turnError()), softened by
 * the drive's spring (soften()), and clamped as one vector to a ball of
 * radius `most`, the motor's force times the substep: the same pull whichever
 * way the joint is off, however the scene is turned. The impulse is kept in
 * ANG, as friction's is, which is off while a motor is on.
 */
function turnDrive(world, J, j, most, h, bias, acc, o) {
  world._drives++;
  turnError(world, J, j);
  soften(J, j, h, bias);
  const i = o + ANG, scale = spring.massScale, rate = spring.biasRate, keep = spring.impulseScale;
  const inverted = spinMass();
  for (let f = 0; f < 3; f++) RHS[f] += rate * ERR[f];
  if (inverted) {
    for (let f = 0; f < 3; f++) SPUN[f] = -scale * (K3[3 * f] * RHS[0] + K3[3 * f + 1] * RHS[1] + K3[3 * f + 2] * RHS[2]) - keep * acc[i + f];
  } else {
    for (let f = 0; f < 3; f++) SPUN[f] = K3[4 * f] > 0 ? -scale * RHS[f] / K3[4 * f] - keep * acc[i + f] : 0;
  }
  spun(most, acc, o);
}

/**
 * How far b is turned against a past what the target asks, as a world
 * vector, axis times angle, into ERR: the sense the turning rows' speeds
 * have (ωb - ωa), so a drive asks it back as a fixed joint asks back its
 * error. The target T is a's turn from where it was joined, in b's own axes
 * (the world's with no b), so in the world now it is qb T qb⁻¹; a's turn
 * since joined, carried as b has moved since, is X = QA QB⁻¹ (frames(): QA =
 * qa qa₀⁻¹ F and QB = qb qb₀⁻¹ F share the joint's axis frame F); the error
 * is E = X (qb T qb⁻¹)⁻¹. Its angle, 2 atan2(|v|, w), is Bearing's own
 * atan2, exact on every machine, and the short way round: q and -q are the
 * same turn, so w is taken 0 or more.
 */
function turnError(world, J, j) {
  const t = 4 * j, T = J.target, b = J.b[j];
  if (b >= 0) {
    rotate(world._rotation, 4 * b, T[t], T[t + 1], T[t + 2], false, ERR);
    AIM[0] = ERR[0]; AIM[1] = ERR[1]; AIM[2] = ERR[2];
  } else {
    AIM[0] = T[t]; AIM[1] = T[t + 1]; AIM[2] = T[t + 2];
  }
  AIM[3] = T[t + 3];
  timesConjugate(QA, 0, QB, 0, TURN, 0);
  timesConjugate(TURN, 0, AIM, 0, TURN, 0);
  let x = TURN[0], y = TURN[1], z = TURN[2], w = TURN[3];
  if (w < 0) { x = -x; y = -y; z = -z; w = -w; }
  const s = Math.sqrt(x * x + y * y + z * z);
  if (s === 0) {
    ERR[0] = 0; ERR[1] = 0; ERR[2] = 0;
    return;
  }
  const k = -2 * atan2(s, w) / s;
  ERR[0] = x * k; ERR[1] = y * k; ERR[2] = z * k;
}
const ERR = new Float64Array(3), AIM = new Float64Array(4), TURN = new Float64Array(4);

/**
 * The three turning rows of ballFriction() and turnDrive(), at SPIN, and
 * their coupling with the joint's pinned rows holding into K3, inverted if
 * this returns true, with the pinned rows' share of a push on each in SHARE,
 * and each row's speed in RHS.
 */
function spinMass() {
  if (pins < 0) pin(pinType);
  for (let f = 0; f < 3; f++) {
    const r = SPIN[f];
    ROWS.fill(0, r, r + 9);
    ROWS[r + 3 + f] = 1;
    ROWS[r + 6 + f] = 1;
  }
  // Their coupling with the point held, K - Kfp Kpp⁻¹ Kpf, as free() works
  // out one row's: the pinned rows' share of a push on each into SHARE.
  for (let f = 0; f < 3; f++) {
    for (let r = 0; r < pins; r++) KFP[3 * f + r] = coupling(SPIN[f], 9 + 9 * r);
    for (let r = 0; r < pins; r++) {
      let sum = 0;
      for (let c = 0; c < pins; c++) sum += PIN[r * pins + c] * KFP[3 * f + c];
      SHARE[3 * f + r] = -sum;
    }
  }
  for (let f = 0; f < 3; f++) {
    for (let g = f; g < 3; g++) {
      let k = coupling(SPIN[f], SPIN[g]);
      for (let r = 0; r < pins; r++) k += KFP[3 * f + r] * SHARE[3 * g + r];
      K3[3 * f + g] = k;
      K3[3 * g + f] = k;
    }
    RHS[f] = speed(SPIN[f]);
  }
  return inverseSPD(K3, 3, K3);
}

/** SPUN added to the turning rows' impulse at ANG, the sum clamped to a ball of radius `most`, and applied with the pinned rows' reaction. */
function spun(most, acc, o) {
  const i = o + ANG;
  let x = acc[i] + SPUN[0], y = acc[i + 1] + SPUN[1], z = acc[i + 2] + SPUN[2];
  const l2 = x * x + y * y + z * z;
  if (l2 > most * most) {
    const s = most / Math.sqrt(l2);
    x *= s; y *= s; z *= s;
  }
  SPUN[0] = x - acc[i]; SPUN[1] = y - acc[i + 1]; SPUN[2] = z - acc[i + 2];
  acc[i] = x; acc[i + 1] = y; acc[i + 2] = z;
  for (let f = 0; f < 3; f++) apply(SPIN[f], SPUN[f]);
  // The point's reaction, which keeps it held, kept with its own impulses.
  for (let r = 0; r < pins; r++) {
    const m = SHARE[r] * SPUN[0] + SHARE[3 + r] * SPUN[1] + SHARE[6 + r] * SPUN[2];
    if (m === 0) continue;
    apply(9 + 9 * r, m);
    acc[o + LIN + r] += m;
  }
}
/** ballFriction()'s rows in ROWS, clear of the point's pinned rows at 1 to 3. */
const SPIN = [0, 36, 45];
const K3 = new Float64Array(9), KFP = new Float64Array(9), SHARE = new Float64Array(9), SPUN = new Float64Array(3);

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

/** out[oo..] = a ⊗ conj(b). out may be a or b. */
function timesConjugate(a, ao, b, bo, out, oo) {
  const ax = a[ao], ay = a[ao + 1], az = a[ao + 2], aw = a[ao + 3];
  const bx = -b[bo], by = -b[bo + 1], bz = -b[bo + 2], bw = b[bo + 3];
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
