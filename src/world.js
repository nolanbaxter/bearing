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
  Contacts, PIXEL, find, prepare, makeSoft, warmStart, solve, restitution,
} from './contact.js';
import { Body } from './body.js';

const TYPES = ['dynamic', 'static', 'kinematic'];
const DYNAMIC = 0, STATIC = 1, KINEMATIC = 2;

const SHAPES = ['box', 'sphere', 'capsule'];
const BOX = 0, SPHERE = 1, CAPSULE = 2;
/** Which sizes each shape has; giving one another shape has throws. */
const SHAPE_SIZES = { box: ['size'], sphere: ['radius'], capsule: ['radius', 'height'] };

const WORLD_OPTIONS = ['gravity', 'substeps'];

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
const BODY_OPTIONS = [
  'type', 'shape', 'size', 'radius', 'height', 'density', 'mass', 'inertia',
  'friction', 'restitution', 'damping', 'angularDamping', 'gravityScale',
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
    /** This step's contacts, and last step's, swapped each step so last step's impulses carry over. */
    this._contacts = new Contacts();
    this._spare = new Contacts();
    this._substeps = o.substeps === undefined ? SUBSTEPS : wholeFromOne('World', 'substeps', o.substeps);
    /** Steps that ran the contact solver: for the off-means-off test, which reads it. */
    this._solverSteps = 0;
    this._destroyed = false;
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
  }

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
    return new Body(this, this._slots.id(s));
  }

  setBody(body, changes) {
    const s = this._slotOfBody('setBody', body);
    const o = options('setBody', changes, 'changes');
    const spec = resolve('setBody', this._spec(s), o);
    // Moving or turning a body by hand is a teleport: `previous` moves with it,
    // so interpolation doesn't draw it sliding across from where it was.
    this._write(s, spec, o.position !== undefined || o.rotation !== undefined);
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
    const n = this._slots.top, state = this._slots.state, type = this._type;
    const tr = this._transforms, prev = this._previous;

    if (this._forcedSlots.length > 0) this._applyForces(dt);

    // Static bodies never move in a step, and their `previous` already equals
    // their transform, so they cost one compare here and none below.
    for (let s = 0; s < n; s++) {
      if (state[s] !== 1 || type[s] === STATIC) continue;
      const t7 = s * 7;
      for (let k = 0; k < 7; k++) prev[t7 + k] = tr[t7 + k];
    }

    // A lone body has nothing to touch: no bounds, no pair search.
    const last = this._contacts, next = this._spare;
    next.count = 0;
    if (n > 1) {
      this._bounds(dt);
      find(this, dt, last, next);
    }
    next.trim();
    const touching = next.count;
    this._contacts = next;
    this._spare = last;

    const substeps = this._substeps, h = dt / substeps;
    if (touching > 0) {
      this._solverSteps++;
      this._start.set(this._position.subarray(0, n * 3));
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
    }

    const p = this._position, q = this._rotation;
    for (let s = 0; s < n; s++) {
      if (state[s] !== 1 || type[s] === STATIC) continue;
      const t7 = s * 7, s3 = s * 3, s4 = s * 4;
      tr[t7] = p[s3]; tr[t7 + 1] = p[s3 + 1]; tr[t7 + 2] = p[s3 + 2];
      tr[t7 + 3] = q[s4]; tr[t7 + 4] = q[s4 + 1]; tr[t7 + 5] = q[s4 + 2]; tr[t7 + 6] = q[s4 + 3];
    }
  }

  /** Gravity, scaled per body, and damping, over h, for every dynamic body. */
  _accelerate(h) {
    const n = this._slots.top, state = this._slots.state, type = this._type;
    const v = this._velocity, w = this._angular, scale = this._gravityScale;
    const gx = this._gravity[0] * h, gy = this._gravity[1] * h, gz = this._gravity[2] * h;
    const damping = this._damping, angularDamping = this._angularDamping;
    for (let s = 0; s < n; s++) {
      if (state[s] !== 1 || type[s] !== DYNAMIC) continue;
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

  /** Positions and rotations on by h, for every body that moves. */
  _move(h) {
    const n = this._slots.top, state = this._slots.state, type = this._type;
    const p = this._position, q = this._rotation, v = this._velocity, w = this._angular, I = this._inertia;
    const half = h * 0.5;
    for (let s = 0; s < n; s++) {
      if (state[s] !== 1 || type[s] === STATIC) continue;
      const s3 = s * 3, s4 = s * 4;
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
    const n = this._slots.top, state = this._slots.state, type = this._type;
    const p = this._position, q = this._rotation, v = this._velocity, w = this._angular;
    const ax = this._axes, box = this._box, motion = this._motion, d = this._dims;
    const g = this._gravity, gl = Math.sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
    for (let s = 0; s < n; s++) {
      if (state[s] !== 1) continue;
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
    this._contacts = new Contacts();
    this._spare = new Contacts();
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

  _setPosition(caller, id, x, y, z) {
    const s = this._slotOf(caller, id);
    this._position.set(xyz(caller, x, y, z), s * 3);
    this._place(s);
  }

  _setRotation(caller, id, q) {
    const s = this._slotOf(caller, id);
    this._rotation.set(q, s * 4);
    this._place(s);
  }

  _setVelocity(caller, id, name, x, y, z) {
    const s = this._slotOf(caller, id);
    const value = xyz(caller, x, y, z);
    if (this._type[s] === STATIC) fail(caller, `a static body has no ${name}`);
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
    const s3 = s * 3;
    this._torque[s3] += t[0]; this._torque[s3 + 1] += t[1]; this._torque[s3 + 2] += t[2];
    this._queueForce(s);
  }

  _applyImpulse(caller, id, impulse, point) {
    const s = this._slotOf(caller, id);
    const j = vec3(caller, 'impulse', impulse);
    const at = point === undefined ? null : vec3(caller, 'point', point);
    if (this._type[s] !== DYNAMIC) return;
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
  }
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
    friction, restitution, damping, angularDamping, gravityScale,
    position, rotation, velocity, angularVelocity,
  };
}

/** The material options, in the order bodyOf lists them. */
const MATERIAL = ['friction', 'restitution', 'damping', 'angularDamping', 'gravityScale'];

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
