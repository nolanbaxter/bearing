// World: every body's data, in columns, and the step that moves them.
//
// A body is a slot (see core/slots.js), and its data lives at that slot in a
// column per field: positions in one Float64Array, rotations in another, and
// so on. There is no object per body. A Body is a handle, a world and an id,
// and holds nothing else, so the step's loops touch typed arrays only.
//
// FLOAT64 INSIDE, FLOAT32 OUT. JavaScript computes in doubles whatever an
// array holds, so a Float32Array column costs a conversion on every read and
// a rounding on every write. Measured with bench/storage.cjs on the i7-1260P
// laptop: float32 state made a solver-shaped step 34-37% slower at 1,000,
// 8,000 and 64,000 bodies, and halving the memory traffic never won it back.
// Doubles also stay exact far from the origin, where a float32 step at 10 km
// is about a millimetre. What renderers read, `transforms`, is a Float32Array,
// since that is what they upload.
//
// COLUMNS, not one record per body: the same benchmark put columns, an
// interleaved record and Box2D v3's hot/cold split within 2% of each other,
// so the layout is the one Winding uses and the rules ask for.
//
// Storage grows by doubling as bodies are added and shrinks when the slots in
// use fall to a quarter of it. Nothing is sized in advance.

import {
  fail, show, finite, positive, positive3, vec3, xyz, quat, oneOf, options, known, readOnly,
} from './core/check.js';
import { rotate } from './core/math.js';
import { Slots, resized, MAX_ID, MAX_SLOTS } from './core/slots.js';
import { volume, inertia as shapeInertia } from './shapes.js';
import {
  Contacts, PIXEL, narrow, store, prepare, makeSoft, warmStart, solve, restitution,
} from './contact.js';
import { Broadphase } from './broadphase.js';
import { Pairs } from './pairs.js';
import { Body } from './body.js';

const TYPES = ['dynamic', 'static', 'kinematic'];
const DYNAMIC = 0, STATIC = 1, KINEMATIC = 2;

const SHAPES = ['box', 'sphere', 'capsule'];
const BOX = 0, SPHERE = 1, CAPSULE = 2;
/** Which sizes each shape has; giving one another shape has throws. */
const SHAPE_SIZES = { box: ['size'], sphere: ['radius'], capsule: ['radius', 'height'] };

const WORLD_OPTIONS = ['gravity', 'substeps', 'sleeping'];

// THE SOLVER'S SHAPE, all measured with bench/solver.js (i7-1260P, Node 22),
// on a 1 m box dropped flat, a 10-box stack and a 210-box pyramid, 10 s at
// 60 Hz each; the full tables are in ROADMAP.md's milestone 4.
//
// Substeps, each one pass and one relaxing pass (Box2D 3's soft step), beat
// passes within a single step (sequential impulses) outright: one substep of
// four or eight passes let the 10-box stack fall, where every substepped
// shape held it. SUBSTEPS is the fewest that kept the pyramid within half a
// box's contact tolerance (2.5 mm): 1.47 mm at 8, against 3.02 at 7 and 9.49
// at 4, with the top sinking 8 mm over 20 rows.
const SUBSTEPS = 8;
// How stiff a contact's spring is, as a fraction of the substep rate. A
// quarter is the most the integrator can follow (ωh = π/2, inside semi-implicit
// Euler's ωh < 2), but with one pass a substep the solver can't keep up and the
// pyramid rang at 0.13 m/s after 10 s; an eighth let it settle to 0.0001 m/s.
// An eighth is also Box2D 3's own effective default: 30 Hz over 4 substeps at
// 60 Hz.
const STIFFNESS = 1 / 8;
// How damped that spring is. Ratios 1, 3 and 10 measured within a millimetre of
// each other; 10 settled the pyramid stillest, and it is Box2D 3's.
const DAMPING_RATIO = 10;

// SLEEPING. A body is still while no point of it has strayed more than its
// own tolerance, about a pixel, from where it was when it last came to rest:
// freezing it there can't be seen. An island sleeps once every body in it has
// been still for SLEEP_TIME. Moving steadily, that is any speed under its
// tolerance over SLEEP_TIME, 5 mm/s for a 1 m box and 0.05 mm/s for a 1 cm marble.
//
// It is how far the body has gone that counts, not how fast it is going. A
// heap of 1,000 boxes measured with a speed rule had 909 still awake after
// 40 s: a few boxes buzzed at 1.3 cm/s, twice the speed that counts as still,
// while going nowhere (one strayed 0.18 mm in 30 s), and kept the whole heap
// awake. Going by distance, a buzz in place is still, and a real creep isn't.
//
// SLEEP_TIME is the shortest that froze nothing about to move, measured with
// sleeping on and off over 8 s at 60 Hz: boxes on slopes 0.25° to 2° past
// friction's hold, which creep before they slide, and cubes 0.5° to 3° past
// balance on an edge, which tip slowly at first. (Closer than 0.41°, a 1 m
// cube stands: its edge's patch holds it, see contact.js.) At 0.25 s the box
// half a degree past friction's hold went to sleep where it stood; at 0.5 s,
// Box2D's and Jolt's, the cube 0.5° past did; from 1 s, none did. Longer
// costs: a 210-box pyramid was all asleep at 0.65 s with 0.5, at 1.13 s with
// 1, and at 2.15 s with 2. The limit that comes with any such time: a body starting from rest that speeds up more
// slowly than 2 tolerance / SLEEP_TIME², 0.01 m/s² for a 1 m box, strays less
// than a pixel in it and can fall asleep before it moves, such as a box on a
// slope within 0.05° of its friction angle.
const SLEEP_TIME = 1;
const BODY_OPTIONS = [
  'type', 'shape', 'size', 'radius', 'height', 'density', 'mass', 'inertia',
  'friction', 'restitution', 'damping', 'angularDamping', 'gravityScale', 'layers', 'mask',
  'position', 'rotation', 'velocity', 'angularVelocity',
];

// Defaults from real things (rule 32): Earth's gravity, water's density, and
// a metre for size. A capsule's 2 m of height over a 0.5 m radius is Godot's
// default capsule, roughly a person. Friction 0.5 is the middle of everyday
// dry materials, about 0.3 to 0.8 (wood on wood 0.3-0.5, rubber on concrete
// about 0.7): there is no one real value for an unnamed material, and Nolan
// chose the middle knowingly. Restitution 0, so piles and stacks don't bounce
// as they land; a ball that should bounce is given its bounce.
const GRAVITY = [0, -9.81, 0];
const DEFAULTS = Object.freeze({
  type: 'dynamic', shape: 'box', size: [1, 1, 1], radius: 0.5, height: 2,
  density: 1000, mass: 0, massGiven: false, inertia: [0, 0, 0], inertiaGiven: false,
  friction: 0.5, restitution: 0, damping: 0, angularDamping: 0, gravityScale: 1,
  // In the first layer, and colliding with all 32: everything collides with
  // everything until told otherwise (rule 110).
  layers: 1, mask: 0xffffffff,
  position: [0, 0, 0], rotation: [0, 0, 0, 1], velocity: [0, 0, 0], angularVelocity: [0, 0, 0],
});

// Module scratch for the step and the apply calls. Neither is re-entered, and
// a fresh array per body per step is exactly the allocation rule 76 forbids.
const v3 = new Float64Array(3);

/** Every per-body column: name, array type, numbers per body. */
const COLUMNS = [
  ['_position', Float64Array, 3],
  ['_rotation', Float64Array, 4],
  ['_velocity', Float64Array, 3],
  ['_angular', Float64Array, 3],
  ['_dims', Float64Array, 3],        // box: size; sphere: radius; capsule: radius, height
  ['_density', Float64Array, 1],
  ['_mass', Float64Array, 1],
  ['_inertia', Float64Array, 3],     // principal moments about the body's own axes, kg·m²
  ['_invInertia', Float64Array, 3],
  ['_force', Float64Array, 3],       // applied since the last step, cleared by it
  ['_torque', Float64Array, 3],
  ['_friction', Float64Array, 1],
  ['_restitution', Float64Array, 1],
  ['_damping', Float64Array, 1],
  ['_angularDamping', Float64Array, 1],
  ['_gravityScale', Float64Array, 1],
  ['_layers', Uint32Array, 1],       // which of 32 layers the body is in, as bits
  ['_mask', Uint32Array, 1],         // which layers it collides with
  ['_invMass', Float64Array, 1],     // 1 / mass for a dynamic body, 0 for anything else
  ['_size', Float64Array, 1],        // the shape's thinnest extent, m: what its tolerances scale with
  ['_reach', Float64Array, 1],       // the farthest any point of the shape is from its centre, m
  // Worked out by the step, for the step:
  ['_axes', Float64Array, 9],        // the body's own axes in world space
  ['_box', Float64Array, 6],         // its bounding box, grown by how far it can move: min, max
  ['_motion', Float64Array, 1],      // how far any point of it can move this step, m
  ['_invI', Float64Array, 6],        // world-space inverse inertia, xx xy xz yy yz zz
  ['_start', Float64Array, 3],       // its position when the step began
  ['_type', Uint8Array, 1],
  ['_shape', Uint8Array, 1],
  ['_massGiven', Uint8Array, 1],
  ['_inertiaGiven', Uint8Array, 1],
  ['_forced', Uint8Array, 1],        // 1 while the slot is in _forcedSlots
  ['_transforms', Float32Array, 7],
  ['_previous', Float32Array, 7],
  // Sleeping (see SLEEP_TIME and _sleep):
  ['_awake', Uint8Array, 1],         // 1 while awake; never for a static body
  ['_awakeList', Int32Array, 1],     // the awake bodies, dense: what every per-body loop walks
  ['_inAwake', Int32Array, 1],       // a body's place in _awakeList, plus 1; 0 when not in it
  ['_still', Float64Array, 1],       // how long it has been still, s
  ['_rest', Float64Array, 7],        // where it was when it last came to rest: position, rotation
  ['_island', Int32Array, 1],        // asleep: its island's first body, plus 1; 0 when awake
  ['_islandNext', Int32Array, 1],    // asleep: the next body of its island, plus 1; 0 at the end
  ['_parent', Int32Array, 1],        // union-find, while islands are found
  ['_least', Float64Array, 1],       // at an island's root, while islands are found: its least still time
  ['_slept', Int32Array, 1],         // the bodies that fell asleep in the last step, dense
  ['_staticMoved', Uint8Array, 1],   // 1 for a static body moved or changed since the last step
  // The moved list (see moved()):
  ['_movedAt', Float64Array, 1],     // the step it last moved in; stepCount + 1 if set by hand since
  ['_seen', Uint32Array, 1],         // for moved(), so a body is listed once
  ['_movedOut', Int32Array, 1],      // what moved() returns a view of
];

export class World {
  constructor(options_) {
    const o = options('World', options_);
    known('World', o, WORLD_OPTIONS);
    this._gravity = o.gravity === undefined ? [...GRAVITY] : vec3('World', 'gravity', o.gravity);
    this._slots = new Slots();
    for (const [name, Type] of COLUMNS) this[name] = new Type(0);
    /**
     * Slots with a force or torque waiting for the next step. A list, so a step
     * with nothing applied checks nothing (rule 28), however many bodies there
     * are. An entry whose `_forced` flag is 0 is stale (its body was removed,
     * or it is a repeat) and is skipped.
     */
    this._forcedSlots = [];
    /** Bodies filed in trees, for finding which might touch (broadphase.js). */
    this._broadphase = new Broadphase();
    /** Pairs whose fat boxes overlap, kept from step to step with their impulses (pairs.js). */
    this._pairs = new Pairs(this);
    /** This step's manifolds, the solver's working set (contact.js). */
    this._contacts = new Contacts();
    this._substeps = o.substeps === undefined ? SUBSTEPS : wholeFromOne('World', 'substeps', o.substeps);
    /** Steps that ran the contact solver: for the off-means-off test, which reads it. */
    this._solverSteps = 0;
    this._sleeping = o.sleeping === undefined ? true : flag('World', 'sleeping', o.sleeping);
    this._stepCount = 0;
    this._reset();
    /** Steps that looked for islands: for the off-means-off test, which reads it. */
    this._islandSteps = 0;
    this._destroyed = false;
  }

  /** The state that isn't a column: fresh at creation, and again when destroyed. */
  _reset() {
    this._awakeCount = 0;
    this._sleptCount = 0;
    /** Something woke, or a body was removed or a static one moved: _syncPairs has work to do. */
    this._wakePending = false;
    /** Static bodies with _staticMoved set, to clear once the step has seen them. */
    this._movedStatics = [];
    /** dt while a step is running, so a body woken mid-step gets its bounds for it; 0 between steps. */
    this._stepDt = 0;
    // The moved log: a body's slot goes here whenever its stamp changes other
    // than by moving in a step while awake, and whenever it leaves the awake
    // list, with a stamp at least its own. moved() reads the awake list and
    // the end of this, instead of every body. See moved() and _log().
    this._logSlots = new Int32Array(0);
    this._logAt = new Float64Array(0);
    this._logCount = 0;
    this._logLast = 0;
    /** Entries stamped this or earlier were dropped: moved() asked about older steps reads every body. */
    this._logFloor = 0;
    this._seenMark = 0;
  }

  // ---- settings and state ----

  /**
   * How many smaller steps each step is solved in. More hold tall stacks
   * stiffer, at a cost; the default is measured, not a guess (see SUBSTEPS).
   */
  get substeps() {
    this._live('substeps');
    return this._substeps;
  }

  set substeps(value) {
    this._live('substeps');
    this._substeps = wholeFromOne(null, 'world.substeps', value);
  }

  /** m/s². A copy; assign a new value to change it. */
  get gravity() {
    this._live('gravity');
    return Object.freeze([...this._gravity]);
  }

  set gravity(value) {
    this._live('gravity');
    this._gravity = vec3(null, 'world.gravity', value);
    // What held a body at rest may not now: every sleeping body wakes (rule 36).
    this._wakeAll();
  }

  /**
   * Whether bodies at rest go to sleep: left out of steps until something
   * disturbs them. Switching it off wakes every sleeping body.
   */
  get sleeping() {
    this._live('sleeping');
    return this._sleeping;
  }

  set sleeping(value) {
    this._live('sleeping');
    this._sleeping = flag(null, 'world.sleeping', value);
    if (!value) {
      this._wakeAll();
    } else {
      // Still times stopped counting while it was off; they start again from now.
      for (let i = 0; i < this._awakeCount; i++) this._settle(this._awakeList[i]);
    }
  }

  get stepCount() {
    this._live('stepCount');
    return this._stepCount;
  }

  set stepCount(_) { readOnly('world.stepCount'); }

  get bodyCount() {
    this._live('bodyCount');
    return this._slots.count;
  }

  set bodyCount(_) { readOnly('world.bodyCount'); }

  get transforms() {
    this._live('transforms');
    return this._transforms;
  }

  set transforms(_) { readOnly('world.transforms'); }

  get previous() {
    this._live('previous');
    return this._previous;
  }

  set previous(_) { readOnly('world.previous'); }

  // ---- bodies ----

  addBody(options_) {
    this._live('addBody');
    const o = options('addBody', options_);
    const spec = resolve('addBody', DEFAULTS, o);
    let s = this._slots.take();
    if (s < 0) {
      this._grow('addBody');
      s = this._slots.take();
    }
    this._write(s, spec, true);
    this._boundsOf(s, 0, 0);
    this._broadphase.add(this, s);
    if (this._type[s] !== STATIC) this._awaken(s);
    this._stamp(s);
    return new Body(this, this._slots.id(s));
  }

  setBody(body, changes) {
    const s = this._slotOfBody('setBody', body);
    const o = options('setBody', changes, 'changes');
    const spec = resolve('setBody', this._spec(s), o);
    // Any change can change what holds it, so its island wakes first (rule 36).
    this._wake(s);
    // Moving or turning a body by hand is a teleport: `previous` moves with it,
    // so interpolation doesn't draw it sliding across from where it was.
    const placed = o.position !== undefined || o.rotation !== undefined;
    this._write(s, spec, placed);
    if (this._type[s] === STATIC) {
      this._unawaken(s);
      // Whatever slept against it may no longer be held, or may be buried.
      this._markStatic(s);
      this._start.set(this._position.subarray(s * 3, s * 3 + 3), s * 3);
    } else {
      this._awaken(s);
    }
    if (placed) this._stamp(s);
    // Anything here can change what it touches: where it is, its shape, its
    // type, its filtering. Filed afresh, with no refresh call (rule 36).
    this._boundsOf(s, 0, 0);
    this._broadphase.refile(this, s);
  }

  bodyOf(body) {
    const s = this._slotOfBody('bodyOf', body);
    const spec = this._spec(s);
    const out = { type: spec.type, shape: spec.shape };
    for (const k of SHAPE_SIZES[spec.shape]) out[k] = k === 'size' ? [...spec.size] : spec[k];
    // Whichever the body was given, so addBody(world.bodyOf(b)) makes a copy.
    if (spec.massGiven) out.mass = spec.mass;
    else out.density = spec.density;
    if (spec.inertiaGiven) out.inertia = [...spec.inertia];
    for (const k of MATERIAL) out[k] = spec[k];
    out.position = [...spec.position];
    out.rotation = [...spec.rotation];
    if (spec.type !== 'static') {
      out.velocity = [...spec.velocity];
      out.angularVelocity = [...spec.angularVelocity];
    }
    return out;
  }

  remove(body) {
    this._remove('remove', body);
  }

  /**
   * The slots of the bodies that moved in any step after step `since`, and
   * always of those that moved in the last one, since they are still between
   * `previous` and `transforms`; with bodies added, or moved by hand, since
   * the last step. Pass the `stepCount` you last drew at, and only these need
   * drawing again. A view the next call or step overwrites.
   */
  moved(since) {
    this._live('moved');
    if (!Number.isInteger(since) || since < 0) {
      fail('moved', `since must be a whole number, 0 or more, got ${show(since)}`);
    }
    const after = Math.min(since, this._stepCount - 1);
    const at = this._movedAt, state = this._slots.state, top = this._slots.top, out = this._movedOut;
    let k = 0;
    if (after < this._logFloor) {
      // Asked about steps the log no longer covers: every body, by its stamp.
      // As exact, just not as quick; Winding's renderer falls back the same way.
      for (let s = 0; s < top; s++) if (state[s] === 1 && at[s] > after) out[k++] = s;
      return out.subarray(0, k);
    }
    let mark = ++this._seenMark;
    if (mark > 0xffffffff) {
      this._seen.fill(0);
      mark = this._seenMark = 1;
    }
    const seen = this._seen, list = this._awakeList;
    for (let i = 0; i < this._awakeCount; i++) {
      const s = list[i];
      if (at[s] > after) { seen[s] = mark; out[k++] = s; }
    }
    // Every other body with a newer stamp left the awake list, or was stamped
    // by hand, since then, and so has an entry at least that new.
    const slots = this._logSlots, stamps = this._logAt;
    for (let i = this._logCount - 1; i >= 0 && stamps[i] > after; i--) {
      const s = slots[i];
      if (s < top && state[s] === 1 && at[s] > after && seen[s] !== mark) { seen[s] = mark; out[k++] = s; }
    }
    return out.subarray(0, k);
  }

  body(id) {
    this._live('body');
    if (!Number.isInteger(id) || id < 0 || id > MAX_ID) {
      fail('body', `id must be a whole number from 0 to ${MAX_ID}, got ${show(id)}`);
    }
    return new Body(this, id);
  }

  // ---- stepping ----

  /**
   * Advances every body by `dt` seconds, exactly: no accumulator and no hidden
   * substeps, because the caller's loop owns the clock (rule 68).
   *
   * Semi-implicit Euler: velocity first, then position from the new velocity.
   * It is symplectic, so an orbit or a pendulum keeps its energy on average
   * instead of spiralling out as explicit Euler's does, at the same cost.
   *
   * In order: forces and torques applied since the last step act for this
   * one and are cleared; the pairs that touch, or will within the step, are
   * found; then the step runs as `substeps` smaller steps, each one gravity
   * and damping, the contact solver's passes, positions and rotations moving
   * (a lopsided spinning body carrying its angular momentum, see tumble()),
   * and the solver's relaxing passes. Bounces come last.
   */
  step(dt) {
    this._live('step');
    if (typeof dt !== 'number' || !(dt > 0) || dt === Infinity) {
      fail('step', `dt must be a positive number of seconds, got ${show(dt)}`);
    }
    const step = ++this._stepCount;
    const state = this._slots.state, list = this._awakeList, at = this._movedAt;
    const tr = this._transforms, prev = this._previous;

    if (this._forcedSlots.length > 0) this._applyForces(dt);
    // What a setter woke since the last step, or a removal or a static body
    // moved by hand disturbed, brings its pairs back among the awake ones.
    if (this._wakePending) this._syncPairs();
    if (this._movedStatics.length > 0) this._clearStatics();

    // Only awake bodies move in a step; static and sleeping ones' `previous`
    // already equals their transform. A body that fell asleep at the end of
    // the last step moved in it, so it gets this once more, and counts as
    // moved once more.
    for (let i = 0; i < this._awakeCount; i++) {
      const t7 = list[i] * 7;
      for (let k = 0; k < 7; k++) prev[t7 + k] = tr[t7 + k];
    }
    for (let i = 0; i < this._sleptCount; i++) {
      const s = this._slept[i];
      if (state[s] !== 1) continue;
      const t7 = s * 7;
      for (let k = 0; k < 7; k++) prev[t7 + k] = tr[t7 + k];
      at[s] = step;
      this._log(s);
    }
    this._sleptCount = 0;

    // Awake bodies' bounds, then the broadphase: re-file what left its fat
    // box, offer the pairs that newly overlap, and narrow every awake pair
    // down to its contact points. Static bodies' bounds were worked out when
    // they were placed, and sleeping ones' when they fell asleep.
    this._stepDt = dt;
    const next = this._contacts, pairs = this._pairs, broadphase = this._broadphase;
    this._bounds(dt);
    broadphase.update(this, dt);
    broadphase.candidates(this, pairs);
    // A touch that wakes an island brings its pairs into the awake run, past
    // where the pass stopped, and they are narrowed in turn until nothing
    // more wakes: a woken stack is held in the step it wakes, rather than
    // dropping a step's worth of gravity before its contacts come back.
    next.count = 0;
    let from = 0;
    do {
      if (this._wakePending) this._syncPairs();
      narrow(this, pairs, broadphase, next, from);
      from = pairs.active;
    } while (this._wakePending);
    next.trim();
    pairs.trim();
    const touching = next.count;

    const substeps = this._substeps, h = dt / substeps;
    if (touching > 0) {
      this._solverSteps++;
      // Static and sleeping bodies don't move, so theirs stays where it was set.
      const p = this._position, start = this._start;
      for (let i = 0; i < this._awakeCount; i++) {
        const s3 = list[i] * 3;
        start[s3] = p[s3]; start[s3 + 1] = p[s3 + 1]; start[s3 + 2] = p[s3 + 2];
      }
      prepare(this, next);
      makeSoft(STIFFNESS / h, DAMPING_RATIO, h);
    }
    for (let sub = 0; sub < substeps; sub++) {
      this._accelerate(h);
      if (touching > 0) {
        warmStart(this, next);
        solve(this, next, h, true);
      }
      this._move(h);
      if (touching > 0) {
        solve(this, next, h, false);
      }
    }
    if (touching > 0) {
      const g = this._gravity;
      restitution(this, next, Math.sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]));
      store(this._pairs, next);
    }

    const p = this._position, q = this._rotation;
    for (let i = 0; i < this._awakeCount; i++) {
      const s = list[i], t7 = s * 7, s3 = s * 3, s4 = s * 4;
      tr[t7] = p[s3]; tr[t7 + 1] = p[s3 + 1]; tr[t7 + 2] = p[s3 + 2];
      tr[t7 + 3] = q[s4]; tr[t7 + 4] = q[s4 + 1]; tr[t7 + 5] = q[s4 + 2]; tr[t7 + 6] = q[s4 + 3];
      at[s] = step;
    }
    this._stepDt = 0;
    if (this._sleeping) this._sleep(dt);
  }

  /**
   * Counts how long each awake body has been still, and puts to sleep every
   * island whose bodies all have been for SLEEP_TIME, at the end of a step.
   *
   * Islands are found afresh, and only in a step where some body has been
   * still long enough, so a scene still settling does no island work at all.
   * They are found by union-find: every awake body starts as its own island,
   * and each contact between two dynamic bodies joins theirs. Static and
   * kinematic bodies join nothing, or everything on one floor would be one
   * island. ponytail: rebuilt from the contacts each time, which costs a pass
   * over the awake bodies and the contacts; Box2D 3 keeps islands from step
   * to step instead, worth it only if this ever measures as a real share of
   * a step.
   */
  _sleep(dt) {
    const list = this._awakeList, p = this._position, q = this._rotation, rest = this._rest, still = this._still;
    const size = this._size, reach = this._reach;
    let ready = false;
    for (let i = 0; i < this._awakeCount; i++) {
      const s = list[i], s3 = s * 3, s4 = s * 4, r7 = s * 7;
      const dx = p[s3] - rest[r7], dy = p[s3 + 1] - rest[r7 + 1], dz = p[s3 + 2] - rest[r7 + 2];
      // How far its farthest point has swung: the turn from rest is
      // q conj(rest), whose vector part is sin(θ/2) long, and a point at
      // distance r turned by θ moves 2 r sin(θ/2), a chord, with no trigonometry.
      const qx = q[s4], qy = q[s4 + 1], qz = q[s4 + 2], qw = q[s4 + 3];
      const ax = rest[r7 + 3], ay = rest[r7 + 4], az = rest[r7 + 5], aw = rest[r7 + 6];
      const tx = aw * qx - qw * ax - (qy * az - qz * ay);
      const ty = aw * qy - qw * ay - (qz * ax - qx * az);
      const tz = aw * qz - qw * az - (qx * ay - qy * ax);
      const strayed = Math.sqrt(dx * dx + dy * dy + dz * dz) + 2 * reach[s] * Math.sqrt(tx * tx + ty * ty + tz * tz);
      if (strayed < size[s] * PIXEL) {
        still[s] += dt;
        if (still[s] >= SLEEP_TIME) ready = true;
      } else {
        this._settle(s);
      }
    }
    if (!ready) return;
    this._islandSteps++;

    const parent = this._parent, type = this._type, c = this._contacts, n = this._awakeCount;
    for (let i = 0; i < n; i++) parent[list[i]] = list[i];
    for (let m = 0; m < c.count; m++) {
      const a = c.a[m], b = c.b[m];
      if (type[a] !== DYNAMIC || type[b] !== DYNAMIC) continue;
      // The lower slot becomes the root, so islands depend only on the inputs.
      const ra = root(parent, a), rb = root(parent, b);
      if (ra < rb) parent[rb] = ra;
      else if (rb < ra) parent[ra] = rb;
    }
    const least = this._least;
    for (let i = 0; i < n; i++) least[list[i]] = Infinity;
    for (let i = 0; i < n; i++) {
      const s = list[i], r = root(parent, s);
      if (still[s] < least[r]) least[r] = still[s];
    }
    // Backwards, since a body falling asleep leaves the list and the last one
    // takes its place, and that one has already been seen.
    let slept = false;
    for (let i = n - 1; i >= 0; i--) {
      const s = list[i], r = root(parent, s);
      if (least[r] < SLEEP_TIME) continue;
      this._fallAsleep(s, r);
      slept = true;
    }
    if (!slept) return;
    // Pairs with no awake body left go to the asleep run, where no step looks.
    const pairs = this._pairs, awake = this._awake;
    for (let p = 0; p < pairs.active;) {
      if (awake[pairs.a[p]] === 0 && awake[pairs.b[p]] === 0) pairs.sleep(p);
      else p++;
    }
  }

  /** Slot s falls asleep, into the island whose first body is r. */
  _fallAsleep(s, r) {
    const s3 = s * 3;
    // Stopped exactly, so it can't creep while asleep or wake with a drift.
    this._velocity.fill(0, s3, s3 + 3);
    this._angular.fill(0, s3, s3 + 3);
    this._unawaken(s);
    this._island[s] = r + 1;
    if (s !== r) {
      this._islandNext[s] = this._islandNext[r];
      this._islandNext[r] = s + 1;
    }
    this._boundsOf(s, 0, 0);
    this._broadphase.settle(this, s);
    this._slept[this._sleptCount++] = s;
  }

  /** Puts slot s on the awake list, if it isn't already. */
  _awaken(s) {
    if (this._inAwake[s] !== 0) return;
    this._awakeList[this._awakeCount] = s;
    this._inAwake[s] = ++this._awakeCount;
    this._awake[s] = 1;
    this._settle(s);
  }

  /** Slot s starts counting still time again, from where it is now. */
  _settle(s) {
    this._still[s] = 0;
    this._rest.set(this._position.subarray(s * 3, s * 3 + 3), s * 7);
    this._rest.set(this._rotation.subarray(s * 4, s * 4 + 4), s * 7 + 3);
  }

  /** Takes slot s off the awake list, if it is on it; the last body takes its place. */
  _unawaken(s) {
    const i = this._inAwake[s] - 1;
    if (i < 0) return;
    const last = this._awakeList[--this._awakeCount];
    this._awakeList[i] = last;
    this._inAwake[last] = i + 1;
    this._inAwake[s] = 0;
    this._awake[s] = 0;
    // It may have moved in steps a later moved() asks about.
    this._log(s);
  }

  /**
   * Wakes slot s's island, if s is asleep: every way of disturbing a sleeping
   * body comes here. A static or awake body is left as it is.
   */
  _wake(s) {
    if (this._island[s] === 0) return;
    const dt = this._stepDt, g = this._gravity;
    const gl = dt > 0 ? Math.sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]) : 0;
    for (let b = this._island[s] - 1; b >= 0;) {
      const next = this._islandNext[b] - 1;
      this._island[b] = 0;
      this._islandNext[b] = 0;
      this._awaken(b);
      // Woken by a touch in the middle of a step: its bounds for the rest of it.
      if (dt > 0) this._boundsOf(b, dt, gl);
      b = next;
    }
    this._wakePending = true;
  }

  _wakeAll() {
    for (let s = 0; s < this._slots.top; s++) if (this._island[s] !== 0) this._wake(s);
  }

  /** A static body moved or changed by hand: what slept against it is woken at the next step. */
  _markStatic(s) {
    if (this._staticMoved[s] === 1) return;
    this._staticMoved[s] = 1;
    this._movedStatics.push(s);
    this._wakePending = true;
  }

  _clearStatics() {
    for (const s of this._movedStatics) this._staticMoved[s] = 0;
    this._movedStatics.length = 0;
  }

  /**
   * Brings into the awake run every asleep pair that now has an awake body,
   * and wakes what a removed body, or a static one moved by hand, leaves
   * unsupported or buried. Waking can wake more, so it goes round until
   * nothing does. Its cost is a pass over the asleep pairs, and it runs only
   * after something woke or changed.
   */
  _syncPairs() {
    const pairs = this._pairs, awake = this._awake, moved = this._staticMoved, slots = this._slots;
    while (this._wakePending) {
      this._wakePending = false;
      for (let p = pairs.active; p < pairs.count; p++) {
        const a = pairs.a[p], b = pairs.b[p];
        const goneA = slots.slotOf(pairs.idA[p]) !== a, goneB = slots.slotOf(pairs.idB[p]) !== b;
        if ((goneA || moved[a] === 1) && !goneB) this._wake(b);
        if ((goneB || moved[b] === 1) && !goneA) this._wake(a);
        // A pair with a body gone joins too, so the next narrowing drops it.
        if (goneA || goneB || awake[a] === 1 || awake[b] === 1) pairs.wake(p);
      }
    }
  }

  /** Marks slot s as moved by hand, or added, since the last step. */
  _stamp(s) {
    this._movedAt[s] = this._stepCount + 1;
    this._log(s);
  }

  /**
   * Notes slot s in the moved log. Entries go in stamp order, each at least
   * the body's own stamp, so moved() can read back from the end and stop.
   *
   * The log is cut whenever it holds more entries than there are bodies:
   * past that, reading every body's stamp is as quick as reading the log.
   * The older half goes, and moved() asked about a step that old reads every
   * body instead. Amortized, each entry is copied at most once.
   */
  _log(s) {
    let n = this._logCount;
    if (n === this._logSlots.length) {
      const length = Math.max(16, n * 2);
      this._logSlots = resized(this._logSlots, length);
      this._logAt = resized(this._logAt, length);
    }
    const stamp = Math.max(this._movedAt[s], this._logLast);
    this._logSlots[n] = s;
    this._logAt[n] = stamp;
    this._logLast = stamp;
    this._logCount = ++n;
    if (n <= this._slots.count) return;
    const floor = this._logAt[(n - 1) >> 1];
    let from = (n - 1) >> 1;
    while (from < n && this._logAt[from] <= floor) from++;
    this._logSlots.copyWithin(0, from, n);
    this._logAt.copyWithin(0, from, n);
    this._logCount = n - from;
    this._logFloor = floor;
    // The world's quarter rule, so a burst of bodies leaves no log behind.
    if (this._logSlots.length > 16 && this._logCount <= this._logSlots.length / 4) {
      let length = 16;
      while (length < this._logCount) length *= 2;
      this._logSlots = resized(this._logSlots, length);
      this._logAt = resized(this._logAt, length);
    }
  }

  /** Gravity, scaled per body, and damping, over h, for every awake dynamic body. */
  _accelerate(h) {
    const list = this._awakeList, n = this._awakeCount, type = this._type;
    const v = this._velocity, w = this._angular, scale = this._gravityScale;
    const gx = this._gravity[0] * h, gy = this._gravity[1] * h, gz = this._gravity[2] * h;
    const damping = this._damping, angularDamping = this._angularDamping;
    for (let i = 0; i < n; i++) {
      const s = list[i];
      if (type[s] !== DYNAMIC) continue;
      const s3 = s * 3, g = scale[s];
      v[s3] += gx * g; v[s3 + 1] += gy * g; v[s3 + 2] += gz * g;
      // dv/dt = -c v, taken implicitly: v / (1 + c h). Exact operations only
      // (Math.exp is not), never overshoots past zero however large c h is,
      // and Box2D's form.
      if (damping[s] !== 0) {
        const k = 1 / (1 + h * damping[s]);
        v[s3] *= k; v[s3 + 1] *= k; v[s3 + 2] *= k;
      }
      if (angularDamping[s] !== 0) {
        const k = 1 / (1 + h * angularDamping[s]);
        w[s3] *= k; w[s3 + 1] *= k; w[s3 + 2] *= k;
      }
    }
  }

  /** Positions and rotations on by h, for every awake body. */
  _move(h) {
    const list = this._awakeList, n = this._awakeCount, type = this._type;
    const p = this._position, q = this._rotation, v = this._velocity, w = this._angular, I = this._inertia;
    const half = h * 0.5;
    for (let i = 0; i < n; i++) {
      const s = list[i], s3 = s * 3, s4 = s * 4;
      p[s3] += v[s3] * h; p[s3 + 1] += v[s3 + 1] * h; p[s3 + 2] += v[s3 + 2] * h;
      const wx = w[s3], wy = w[s3 + 1], wz = w[s3 + 2];
      const turning = wx !== 0 || wy !== 0 || wz !== 0;
      // A lopsided dynamic body tumbles; see tumble(). A sphere or a cube has
      // the same moment about every axis, and then its angular velocity is
      // its angular momentum over one number, constant, so the plain update
      // below is already exact. A kinematic body turns as it is told.
      if (turning && type[s] === DYNAMIC && (I[s3] !== I[s3 + 1] || I[s3 + 1] !== I[s3 + 2])) {
        tumble(q, s4, I, this._invInertia, s3, w, h);
      } else if (turning) {
        // q += h/2 (ω, 0) q, with ω in world space, then back to unit length.
        // Not turning skips this and leaves the rotation exactly as it was,
        // rather than renormalizing it every substep for nothing.
        advance(q, s4, wx, wy, wz, half, q, s4);
      }
    }
  }

  /**
   * Each body's axes, how far it can move this step, and its bounding box
   * grown by that and its tolerance: what the pair search reads.
   *
   * How far it can move is its speed, plus what gravity adds over the step,
   * plus its spin times its reach, all over dt. Two bodies further apart than
   * the sum of theirs can't meet this step, so no contact is made for them;
   * two nearer get speculative points out to it (see collide.js).
   */
  _bounds(dt) {
    const list = this._awakeList, n = this._awakeCount;
    const g = this._gravity, gl = Math.sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
    for (let i = 0; i < n; i++) this._boundsOf(list[i], dt, gl);
  }

  /** _bounds for one body: every step for an awake one, and whenever a body is placed, changed or falls asleep. */
  _boundsOf(s, dt, gl) {
    const type = this._type;
    const p = this._position, q = this._rotation, v = this._velocity, w = this._angular;
    const ax = this._axes, box = this._box, motion = this._motion, d = this._dims;
    {
      const s3 = s * 3, s4 = s * 4, a9 = s * 9, b6 = s * 6;
      const x = q[s4], y = q[s4 + 1], z = q[s4 + 2], qw = q[s4 + 3];
      ax[a9] = 1 - 2 * (y * y + z * z); ax[a9 + 1] = 2 * (x * y + z * qw); ax[a9 + 2] = 2 * (x * z - y * qw);
      ax[a9 + 3] = 2 * (x * y - z * qw); ax[a9 + 4] = 1 - 2 * (x * x + z * z); ax[a9 + 5] = 2 * (y * z + x * qw);
      ax[a9 + 6] = 2 * (x * z + y * qw); ax[a9 + 7] = 2 * (y * z - x * qw); ax[a9 + 8] = 1 - 2 * (x * x + y * y);

      let m = 0;
      if (type[s] !== STATIC) {
        const speed = Math.sqrt(v[s3] * v[s3] + v[s3 + 1] * v[s3 + 1] + v[s3 + 2] * v[s3 + 2])
          + (type[s] === DYNAMIC ? gl * Math.abs(this._gravityScale[s]) * dt : 0)
          + Math.sqrt(w[s3] * w[s3] + w[s3 + 1] * w[s3 + 1] + w[s3 + 2] * w[s3 + 2]) * this._reach[s];
        m = speed * dt;
      }
      motion[s] = m;

      // Half the box's extent along each world axis.
      let ex, ey, ez;
      const shape = this._shape[s];
      if (shape === BOX) {
        const hx = d[s3] * 0.5, hy = d[s3 + 1] * 0.5, hz = d[s3 + 2] * 0.5;
        ex = Math.abs(ax[a9]) * hx + Math.abs(ax[a9 + 3]) * hy + Math.abs(ax[a9 + 6]) * hz;
        ey = Math.abs(ax[a9 + 1]) * hx + Math.abs(ax[a9 + 4]) * hy + Math.abs(ax[a9 + 7]) * hz;
        ez = Math.abs(ax[a9 + 2]) * hx + Math.abs(ax[a9 + 5]) * hy + Math.abs(ax[a9 + 8]) * hz;
      } else if (shape === SPHERE) {
        ex = ey = ez = d[s3];
      } else {
        const r = d[s3], hl = (d[s3 + 1] - 2 * r) * 0.5;
        ex = Math.abs(ax[a9 + 3]) * hl + r; ey = Math.abs(ax[a9 + 4]) * hl + r; ez = Math.abs(ax[a9 + 5]) * hl + r;
      }
      const grow = m + this._size[s] * PIXEL;
      box[b6] = p[s3] - ex - grow; box[b6 + 1] = p[s3 + 1] - ey - grow; box[b6 + 2] = p[s3 + 2] - ez - grow;
      box[b6 + 3] = p[s3] + ex + grow; box[b6 + 4] = p[s3 + 1] + ey + grow; box[b6 + 5] = p[s3 + 2] + ez + grow;
    }
  }

  /** Forces and torques waiting since the last step act for `dt`, and are cleared. */
  _applyForces(dt) {
    const list = this._forcedSlots;
    const forced = this._forced, type = this._type;
    const f = this._force, t = this._torque, v = this._velocity, w = this._angular;
    for (let i = 0; i < list.length; i++) {
      const s = list[i];
      if (forced[s] === 0) continue;   // removed since, or a repeat
      forced[s] = 0;
      const s3 = s * 3;
      // Made static or kinematic since the force was applied: it no longer
      // moves by forces, and the force goes with the step like any other.
      if (type[s] === DYNAMIC) {
        const m = this._mass[s];
        v[s3] += f[s3] * dt / m; v[s3 + 1] += f[s3 + 1] * dt / m; v[s3 + 2] += f[s3 + 2] * dt / m;
        this._turn(s, t[s3] * dt, t[s3 + 1] * dt, t[s3 + 2] * dt);
      }
      f[s3] = f[s3 + 1] = f[s3 + 2] = 0;
      t[s3] = t[s3 + 1] = t[s3 + 2] = 0;
    }
    list.length = 0;
  }

  /**
   * Adds the angular impulse (x, y, z), N·m·s in world space, to slot `s`'s
   * angular velocity: Δω = I⁻¹ L, where the world-space inverse inertia is
   * R diag(1/I) Rᵀ. Done as rotate into the body's frame, divide by its
   * moments, rotate back, so no 3x3 matrix is built.
   */
  _turn(s, x, y, z) {
    const s3 = s * 3, s4 = s * 4;
    const inv = this._invInertia;
    rotate(this._rotation, s4, x, y, z, true, v3);
    rotate(this._rotation, s4, v3[0] * inv[s3], v3[1] * inv[s3 + 1], v3[2] * inv[s3 + 2], false, v3);
    this._angular[s3] += v3[0]; this._angular[s3 + 1] += v3[1]; this._angular[s3 + 2] += v3[2];
  }

  /** Lets go of every body and all storage. Using the world afterwards throws; calling this again does nothing. */
  destroy() {
    if (this._destroyed) return;
    this._destroyed = true;
    this._slots = new Slots();
    for (const [name, Type] of COLUMNS) this[name] = new Type(0);
    this._forcedSlots = [];
    this._broadphase = new Broadphase();
    this._pairs = new Pairs(this);
    this._contacts = new Contacts();
    this._reset();
  }

  // ---- what Body calls (public, but not for users: see test/docs.test.js) ----

  _live(caller) {
    if (this._destroyed) fail(caller, 'this world was destroyed');
  }

  /** The slot of a live id, after checking the world; throws naming `caller` otherwise. */
  _slotOf(caller, id) {
    this._live(caller);
    const s = this._slots.slotOf(id);
    if (s < 0) fail(caller, 'this body was removed');
    return s;
  }

  _alive(id) {
    return !this._destroyed && this._slots.slotOf(id) >= 0;
  }

  _vector(caller, id, column, size) {
    const s = this._slotOf(caller, id);
    const out = new Array(size);
    for (let k = 0; k < size; k++) out[k] = column[s * size + k];
    return Object.freeze(out);
  }

  _massOf(caller, id) {
    return this._mass[this._slotOf(caller, id)];
  }

  _asleepOf(caller, id) {
    return this._island[this._slotOf(caller, id)] !== 0;
  }

  _setPosition(caller, id, x, y, z) {
    const s = this._slotOf(caller, id);
    const value = xyz(caller, x, y, z);
    this._wake(s);
    this._position.set(value, s * 3);
    this._moved(s);
  }

  _setRotation(caller, id, q) {
    const s = this._slotOf(caller, id);
    this._wake(s);
    this._rotation.set(q, s * 4);
    this._moved(s);
  }

  /** After a teleport: drawn there, stamped, filed afresh, and what a static body held woken. */
  _moved(s) {
    this._place(s);
    this._stamp(s);
    if (this._type[s] === STATIC) this._markStatic(s);
    this._boundsOf(s, 0, 0);
    this._broadphase.refile(this, s);
  }

  _setVelocity(caller, id, name, x, y, z) {
    const s = this._slotOf(caller, id);
    const value = xyz(caller, x, y, z);
    if (this._type[s] === STATIC) fail(caller, `a static body has no ${name}`);
    this._wake(s);
    (name === 'velocity' ? this._velocity : this._angular).set(value, s * 3);
  }

  // Forces and impulses. Each checks its arguments whatever the body's type,
  // then does nothing to a static or kinematic body: those act as infinitely
  // heavy, so no push moves them, and that is the physics rather than a
  // mistake to report. An explosion can push everything nearby without first
  // asking what each thing is.

  _applyForce(caller, id, force, point) {
    const s = this._slotOf(caller, id);
    const f = vec3(caller, 'force', force);
    const at = point === undefined ? null : vec3(caller, 'point', point);
    if (this._type[s] !== DYNAMIC) return;
    this._wake(s);
    const s3 = s * 3;
    this._force[s3] += f[0]; this._force[s3 + 1] += f[1]; this._force[s3 + 2] += f[2];
    if (at) {
      const rx = at[0] - this._position[s3], ry = at[1] - this._position[s3 + 1], rz = at[2] - this._position[s3 + 2];
      this._torque[s3] += ry * f[2] - rz * f[1];
      this._torque[s3 + 1] += rz * f[0] - rx * f[2];
      this._torque[s3 + 2] += rx * f[1] - ry * f[0];
    }
    this._queueForce(s);
  }

  _applyTorque(caller, id, torque) {
    const s = this._slotOf(caller, id);
    const t = vec3(caller, 'torque', torque);
    if (this._type[s] !== DYNAMIC) return;
    this._wake(s);
    const s3 = s * 3;
    this._torque[s3] += t[0]; this._torque[s3 + 1] += t[1]; this._torque[s3 + 2] += t[2];
    this._queueForce(s);
  }

  _applyImpulse(caller, id, impulse, point) {
    const s = this._slotOf(caller, id);
    const j = vec3(caller, 'impulse', impulse);
    const at = point === undefined ? null : vec3(caller, 'point', point);
    if (this._type[s] !== DYNAMIC) return;
    this._wake(s);
    const s3 = s * 3, m = this._mass[s];
    this._velocity[s3] += j[0] / m; this._velocity[s3 + 1] += j[1] / m; this._velocity[s3 + 2] += j[2] / m;
    if (at) {
      const rx = at[0] - this._position[s3], ry = at[1] - this._position[s3 + 1], rz = at[2] - this._position[s3 + 2];
      this._turn(s, ry * j[2] - rz * j[1], rz * j[0] - rx * j[2], rx * j[1] - ry * j[0]);
    }
  }

  _applyAngularImpulse(caller, id, impulse) {
    const s = this._slotOf(caller, id);
    const l = vec3(caller, 'impulse', impulse);
    if (this._type[s] !== DYNAMIC) return;
    this._wake(s);
    this._turn(s, l[0], l[1], l[2]);
  }

  _queueForce(s) {
    if (this._forced[s] === 0) {
      this._forced[s] = 1;
      this._forcedSlots.push(s);
    }
  }

  _remove(caller, body) {
    this._live(caller);
    if (!(body instanceof Body)) fail(caller, `body must be a Body, got ${show(body)}`);
    if (body.world !== this) fail(caller, 'that body belongs to another world');
    const s = this._slots.slotOf(body.id);
    if (s < 0) return;   // already gone: removing twice is harmless, as in Winding
    // A force waiting on this slot must not land on whichever body gets it next.
    this._forced[s] = 0;
    this._force.fill(0, s * 3, s * 3 + 3);
    this._torque.fill(0, s * 3, s * 3 + 3);
    // What it was holding up, or resting on, wakes: its own island now, and
    // anything asleep against it at the next step, which finds it gone.
    this._wake(s);
    this._unawaken(s);
    this._wakePending = true;
    // Out of its tree now; its pairs go at the next step, which finds the body gone.
    this._broadphase.remove(s);
    this._slots.release(s);
    // Shrink when the slots in use fall to a quarter of the storage, to the
    // power of two that growth would have chosen for them, so a world that
    // bursts and settles comes back to exactly the storage it started with.
    // It can't thrash: a shrink needs the slots in use to have fallen by at
    // least a quarter of the storage since the last resize, so every copy of
    // the columns is paid for by at least that many adds or removes. That is
    // the bound rule 81 asks for, with no clock and no tuned delay.
    const top = this._slots.top;
    if (top <= this._slots.capacity / 4) {
      let capacity = top === 0 ? 0 : 1;
      while (capacity < top) capacity *= 2;
      this._resize(capacity);
    }
    this._broadphase.trim();
  }

  // ---- inside ----

  _slotOfBody(caller, body) {
    this._live(caller);
    if (!(body instanceof Body)) fail(caller, `body must be a Body, got ${show(body)}`);
    if (body.world !== this) fail(caller, 'that body belongs to another world');
    return this._slotOf(caller, body.id);
  }

  _grow(caller) {
    const slots = this._slots;
    // slots.limit is MAX_SLOTS except in a test, which lowers it to reach this
    // without 16 million bodies; the message names the real limit either way.
    if (slots.capacity >= slots.limit) {
      fail(caller, `a world holds at most ${MAX_SLOTS} bodies (the slot field of an id is 24 bits)`);
    }
    this._resize(Math.min(Math.max(slots.capacity * 2, 1), slots.limit));
  }

  _resize(capacity) {
    this._slots.resize(capacity);
    for (const [name, , per] of COLUMNS) this[name] = resized(this[name], capacity * per);
    this._broadphase.resize(capacity);
  }

  /** A slot's full description, every size included, in option form. */
  _spec(s) {
    const shape = SHAPES[this._shape[s]];
    const d = this._dims;
    const s3 = s * 3, s4 = s * 4;
    return {
      type: TYPES[this._type[s]],
      shape,
      size: shape === 'box' ? [d[s3], d[s3 + 1], d[s3 + 2]] : [...DEFAULTS.size],
      radius: shape === 'box' ? DEFAULTS.radius : d[s3],
      height: shape === 'capsule' ? d[s3 + 1] : DEFAULTS.height,
      density: this._density[s],
      mass: this._mass[s],
      massGiven: this._massGiven[s] === 1,
      inertia: [this._inertia[s3], this._inertia[s3 + 1], this._inertia[s3 + 2]],
      inertiaGiven: this._inertiaGiven[s] === 1,
      friction: this._friction[s],
      restitution: this._restitution[s],
      damping: this._damping[s],
      angularDamping: this._angularDamping[s],
      gravityScale: this._gravityScale[s],
      layers: this._layers[s],
      mask: this._mask[s],
      position: [this._position[s3], this._position[s3 + 1], this._position[s3 + 2]],
      rotation: [this._rotation[s4], this._rotation[s4 + 1], this._rotation[s4 + 2], this._rotation[s4 + 3]],
      velocity: [this._velocity[s3], this._velocity[s3 + 1], this._velocity[s3 + 2]],
      angularVelocity: [this._angular[s3], this._angular[s3 + 1], this._angular[s3 + 2]],
    };
  }

  /** Writes a checked spec into the columns. `placed`: the transform was set by hand. */
  _write(s, spec, placed) {
    const s3 = s * 3;
    this._type[s] = TYPES.indexOf(spec.type);
    const shape = SHAPES.indexOf(spec.shape);
    this._shape[s] = shape;
    const d = this._dims;
    if (shape === BOX) d.set(spec.size, s3);
    else { d[s3] = spec.radius; d[s3 + 1] = shape === CAPSULE ? spec.height : 0; d[s3 + 2] = 0; }

    // Mass follows the shape unless it was given (rule 36: no
    // updateMassProperties()). Given a mass, the density is what makes it.
    const v = volume(spec);
    this._massGiven[s] = spec.massGiven ? 1 : 0;
    if (spec.massGiven) {
      this._mass[s] = spec.mass;
      this._density[s] = spec.mass / v;
    } else {
      this._density[s] = spec.density;
      this._mass[s] = spec.density * v;
    }
    // Inertia follows the shape and mass in the same way, unless it was given.
    this._inertiaGiven[s] = spec.inertiaGiven ? 1 : 0;
    if (spec.inertiaGiven) this._inertia.set(spec.inertia, s3);
    else this._inertia.set(shapeInertia(spec, this._mass[s], v3), s3);
    for (let k = 0; k < 3; k++) this._invInertia[s3 + k] = 1 / this._inertia[s3 + k];
    // What contacts use: how much a push moves it (nothing, unless it is
    // dynamic), its thinnest size, which its tolerances scale with, and how
    // far its surface reaches from its centre, which bounds how far a spin
    // can carry any point of it.
    this._invMass[s] = spec.type === 'dynamic' ? 1 / this._mass[s] : 0;
    // The solver works out awake bodies' world inverse inertia each step; a
    // static one's must stay zero, even if it was dynamic before.
    if (spec.type === 'static') this._invI.fill(0, s * 6, s * 6 + 6);
    if (shape === BOX) {
      this._size[s] = Math.min(spec.size[0], spec.size[1], spec.size[2]);
      this._reach[s] = 0.5 * Math.sqrt(spec.size[0] * spec.size[0] + spec.size[1] * spec.size[1] + spec.size[2] * spec.size[2]);
    } else {
      this._size[s] = 2 * spec.radius;
      this._reach[s] = shape === CAPSULE ? spec.height * 0.5 : spec.radius;
    }
    this._friction[s] = spec.friction;
    this._restitution[s] = spec.restitution;
    this._damping[s] = spec.damping;
    this._angularDamping[s] = spec.angularDamping;
    this._gravityScale[s] = spec.gravityScale;
    this._layers[s] = spec.layers;
    this._mask[s] = spec.mask;
    this._position.set(spec.position, s3);
    this._rotation.set(spec.rotation, s * 4);
    this._velocity.set(spec.velocity, s3);
    this._angular.set(spec.angularVelocity, s3);
    if (placed) this._place(s);
  }

  /** Copies a slot's position and rotation out to `transforms`, and to `previous`, for a teleport. */
  _place(s) {
    const t7 = s * 7, s3 = s * 3, s4 = s * 4;
    const tr = this._transforms;
    tr[t7] = this._position[s3]; tr[t7 + 1] = this._position[s3 + 1]; tr[t7 + 2] = this._position[s3 + 2];
    tr[t7 + 3] = this._rotation[s4]; tr[t7 + 4] = this._rotation[s4 + 1];
    tr[t7 + 5] = this._rotation[s4 + 2]; tr[t7 + 6] = this._rotation[s4 + 3];
    this._previous.set(tr.subarray(t7, t7 + 7), t7);
    // Where the solver measures this step's movement from, for a body that
    // won't be moving in it (static, or asleep) until something wakes it.
    this._start.set(this._position.subarray(s3, s3 + 3), s3);
  }
}

/** The root of s's island, halving the path on the way so the next look is quicker. */
function root(parent, s) {
  while (parent[s] !== s) {
    parent[s] = parent[parent[s]];
    s = parent[s];
  }
  return s;
}

/** true or false, and nothing else: a 0 or a 'false' is more likely a mistake than a choice. */
function flag(caller, name, value) {
  if (typeof value !== 'boolean') fail(caller, `${name} must be true or false, got ${show(value)}`);
  return value;
}

/**
 * The checked result of applying `given` to `base`, for `caller`. Checks every
 * option before anything is written (rule 22), so a call that throws leaves
 * the body exactly as it was.
 */
function resolve(caller, base, given) {
  known(caller, given, BODY_OPTIONS);
  // An option set to undefined counts as left out, so a spread of an object
  // with holes takes the defaults. A misspelt one still throws above.
  const has = (k) => given[k] !== undefined;

  const type = has('type') ? oneOf(caller, 'type', given.type, TYPES) : base.type;
  const shape = has('shape') ? oneOf(caller, 'shape', given.shape, SHAPES) : base.shape;
  for (const k of ['size', 'radius', 'height']) {
    if (has(k) && !SHAPE_SIZES[shape].includes(k)) fail(caller, `a ${shape} has no ${k}`);
  }
  // A new shape starts from its own defaults; sizes of the old one don't carry over.
  const fresh = shape !== base.shape;
  const size = has('size') ? positive3(caller, 'size', given.size) : fresh ? [...DEFAULTS.size] : base.size;
  const radius = has('radius') ? positive(caller, 'radius', given.radius) : fresh ? DEFAULTS.radius : base.radius;
  const height = has('height') ? positive(caller, 'height', given.height) : fresh ? DEFAULTS.height : base.height;
  if (shape === 'capsule' && height < 2 * radius) {
    fail(caller, `height must be at least twice radius (${2 * radius}), got ${height}`);
  }

  if (has('mass') && has('density')) fail(caller, 'give mass or density, not both');
  const mass = has('mass') ? positive(caller, 'mass', given.mass) : base.mass;
  const density = has('density') ? positive(caller, 'density', given.density) : base.density;
  const massGiven = has('mass') ? true : has('density') ? false : base.massGiven;

  // null is a value here, not a hole: it asks for the inertia to follow the
  // shape again, after one was given.
  let inertia = base.inertia, inertiaGiven = base.inertiaGiven;
  if (given.inertia === null) inertiaGiven = false;
  else if (has('inertia')) {
    const i = given.inertia;
    if (!(i !== null && typeof i === 'object' && i.length === 3
        && [0, 1, 2].every((k) => typeof i[k] === 'number' && i[k] > 0 && i[k] < Infinity))) {
      fail(caller, `inertia must be 3 positive numbers, or null to follow the shape, got ${show(i)}`);
    }
    inertia = [i[0], i[1], i[2]];
    inertiaGiven = true;
  }

  const friction = has('friction') ? atLeastZero(caller, 'friction', given.friction) : base.friction;
  let restitution = base.restitution;
  if (has('restitution')) {
    const r = given.restitution;
    // Past 1 a bounce would come back faster than it hit: energy from nowhere.
    if (typeof r !== 'number' || !(r >= 0 && r <= 1)) {
      fail(caller, `restitution must be from 0 to 1, got ${show(r)}`);
    }
    restitution = r;
  }
  const damping = has('damping') ? atLeastZero(caller, 'damping', given.damping) : base.damping;
  const angularDamping = has('angularDamping')
    ? atLeastZero(caller, 'angularDamping', given.angularDamping) : base.angularDamping;
  const gravityScale = has('gravityScale') ? finite(caller, 'gravityScale', given.gravityScale) : base.gravityScale;
  const layers = has('layers') ? bits(caller, 'layers', given.layers) : base.layers;
  const mask = has('mask') ? bits(caller, 'mask', given.mask) : base.mask;

  const position = has('position') ? vec3(caller, 'position', given.position) : base.position;
  const rotation = has('rotation') ? quat(caller, 'rotation', given.rotation) : base.rotation;
  let velocity, angularVelocity;
  if (type === 'static') {
    for (const k of ['velocity', 'angularVelocity']) if (has(k)) fail(caller, `a static body has no ${k}`);
    // A body made static stops: a static body is one that doesn't move.
    velocity = [0, 0, 0];
    angularVelocity = [0, 0, 0];
  } else {
    velocity = has('velocity') ? vec3(caller, 'velocity', given.velocity) : base.velocity;
    angularVelocity = has('angularVelocity')
      ? vec3(caller, 'angularVelocity', given.angularVelocity) : base.angularVelocity;
  }
  return {
    type, shape, size, radius, height, density, mass, massGiven, inertia, inertiaGiven,
    friction, restitution, damping, angularDamping, gravityScale, layers, mask,
    position, rotation, velocity, angularVelocity,
  };
}

/** The material options, in the order bodyOf lists them. */
const MATERIAL = ['friction', 'restitution', 'damping', 'angularDamping', 'gravityScale', 'layers', 'mask'];

/**
 * 32 bits, given as any whole number that fits: unsigned (0 to 2^32 - 1) or
 * signed (-2^31 to -1). JavaScript's bit operators give signed results, so
 * `~PLAYER` and `1 << 31` are negative; refusing them would make the usual way
 * of writing a mask throw. Nothing is clamped: the same 32 bits are kept,
 * read back unsigned.
 */
function bits(caller, name, value) {
  if (!Number.isInteger(value) || value < -0x80000000 || value > 0xffffffff) {
    fail(caller, `${name} must be a whole number that fits in 32 bits, got ${show(value)}`);
  }
  return value >>> 0;
}

/** A whole number, 1 or more. */
function wholeFromOne(caller, name, value) {
  if (!Number.isInteger(value) || value < 1) fail(caller, `${name} must be a whole number, 1 or more, got ${show(value)}`);
  return value;
}

function atLeastZero(caller, name, value) {
  if (typeof value !== 'number' || !(value >= 0) || value === Infinity) {
    fail(caller, `${name} must be 0 or more, got ${show(value)}`);
  }
  return value;
}

/**
 * Turns a lopsided body for one step: why a book spun about its middle axis
 * flips over, and why a tumbling body's angular velocity keeps changing when
 * nothing touches it.
 *
 * With no torque, what stays constant is the angular momentum L = I ω in
 * world space, not ω; the world-space inertia changes as the body turns, so
 * ω has to change with it. Engines usually add this as a gyroscopic torque
 * on ω. Taken explicitly, as Jolt and PhysX do when it is switched on, a fast
 * lopsided spin blows up, so both leave it off by default. Taken implicitly,
 * as Bullet does, it is stable but bleeds energy: a book flipping at 5 rad/s
 * kept 74% of its energy after 20 s at 60 Hz. An implicit midpoint version of
 * the same keeps the energy but let the momentum wander 8% at 20 rad/s, and
 * blew up at 50.
 *
 * So instead L is the thing carried across the step, exactly: work out L from
 * ω, turn the body by the angular velocity L gives at the half step, and
 * read the new ω back from L at the end. Angular momentum is then conserved
 * to rounding by construction. Energy can only drift down, and can never run
 * away, since with |L| fixed it is at most |L|² / 2 I_min. Measured over
 * 120 s of 60 Hz steps, each of the 8 substeps: under 0.001% lost flipping at
 * 5 rad/s, 0.02% at 20 rad/s, 0.6% at 50 rad/s. (Taken whole, with no
 * substeps, the same cost 0.04%, 6% and 11%: what a step's size costs.) It is
 * always on, costs six quaternion rotations a lopsided spinning body a
 * substep, and has nothing to tune. test/motion.test.js holds it to these.
 *
 * Reads the rotation at q[s4..] and ω at w[s3..]; writes both.
 */
function tumble(q, s4, I, inv, s3, w, dt) {
  // L = R I Rᵀ ω.
  rotate(q, s4, w[s3], w[s3 + 1], w[s3 + 2], true, v3);
  rotate(q, s4, v3[0] * I[s3], v3[1] * I[s3 + 1], v3[2] * I[s3 + 2], false, v3);
  const lx = v3[0], ly = v3[1], lz = v3[2];
  // Half a step on with the starting ω: q + dt/4 (ω, 0) q.
  advance(q, s4, w[s3], w[s3 + 1], w[s3 + 2], dt * 0.25, q4, 0);
  // The angular velocity L gives there, R I⁻¹ Rᵀ L, turns the whole step.
  angular(q4, 0, inv, s3, lx, ly, lz);
  advance(q, s4, v3[0], v3[1], v3[2], dt * 0.5, q, s4);
  // And the angular velocity L gives at the end is the body's new ω.
  angular(q, s4, inv, s3, lx, ly, lz);
  w[s3] = v3[0]; w[s3 + 1] = v3[1]; w[s3 + 2] = v3[2];
}

const q4 = new Float64Array(4);

/** out[oo..] = normalize(q + h (ω, 0) q), for q at q[o..]. out may be q. */
function advance(q, o, wx, wy, wz, h, out, oo) {
  const x = q[o], y = q[o + 1], z = q[o + 2], w = q[o + 3];
  const nx = x + h * (wx * w + wy * z - wz * y);
  const ny = y + h * (wy * w + wz * x - wx * z);
  const nz = z + h * (wz * w + wx * y - wy * x);
  const nw = w - h * (wx * x + wy * y + wz * z);
  const l = 1 / Math.sqrt(nx * nx + ny * ny + nz * nz + nw * nw);
  out[oo] = nx * l; out[oo + 1] = ny * l; out[oo + 2] = nz * l; out[oo + 3] = nw * l;
}

/** v3 = R I⁻¹ Rᵀ L, for R the rotation at q[o..]: the angular velocity that momentum gives. */
function angular(q, o, inv, s3, lx, ly, lz) {
  rotate(q, o, lx, ly, lz, true, v3);
  rotate(q, o, v3[0] * inv[s3], v3[1] * inv[s3 + 1], v3[2] * inv[s3 + 2], false, v3);
}
