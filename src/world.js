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
  fail, show, finite, positive, positive3, vec3, xyz, quat, direction, oneOf, options, known, readOnly,
} from './core/check.js';
import { rotate } from './core/math.js';
import { Slots, resized, MAX_ID, MAX_SLOTS } from './core/slots.js';
import { volume, inertia as shapeInertia, hullMass, shifted, turnedTensor, matrixOf, principal } from './shapes.js';
import { build } from './hull.js';
import {
  Contacts, PIXEL, narrow, store, inertias, turnedInertias, prepare, makeSoft, warmStart, solve, restitution, touched, gapOf,
  WORK, crowded,
} from './contact.js';
import { Jobs, cores, sharedMemory } from './core/jobs.js';
import {
  Joints, Joint, addJoint, setJoint, jointOf, jointSlot, dropJoint, dropJointsOf, gather, warmStartJoints, solveJoints,
  jointSoft, JOINT_STIFFNESS, JOINT_DAMPING_RATIO, plantJoints,
} from './joints.js';
import { Broadphase } from './broadphase.js';
import { anchored } from './forest.js';
import { colour } from './graph.js';
import { Pairs } from './pairs.js';
import { Body } from './body.js';
import {
  Shape, Manifold, Part, Parts, SPHERE as SPHERE_KIND, CAPSULE as CAPSULE_KIND, BOX as BOX_KIND, HULL as HULL_KIND,
} from './collide.js';
import { ray, shapeCast as cast, overlap as overlapping, sense, sweep } from './query.js';
import { Events, END } from './events.js';
import { walk, settle, CHARACTER } from './character.js';
import { lockedInverse } from './contact.js';
import { cos, quatMultiply, axesOf } from './core/math.js';
import { Mesh, Heightfield } from './mesh.js';
import { save, load } from './save.js';
import { debugLines } from './debug.js';

const TYPES = ['dynamic', 'static', 'kinematic', 'character'];
const DYNAMIC = 0, STATIC = 1, KINEMATIC = 2;   // and CHARACTER, 3, from character.js
/** Options only a character has. */
const WALKING = ['maxSlope', 'stepHeight'];

const SHAPES = ['box', 'sphere', 'capsule', 'hull', 'compound', 'mesh', 'heightfield'];
const BOX = 0, SPHERE = 1, CAPSULE = 2, HULL = 3, COMPOUND = 4, MESH = 5;   // a heightfield is 6: shape >= MESH is level geometry
/**
 * What a query's shape can be: a box, ball or capsule. A hull is built from
 * its points by quickhull, too dear to do afresh for each query.
 */
const QUERY_SHAPES = ['box', 'sphere', 'capsule'];
/** Which sizes each shape has; giving one another shape has throws. */
const SHAPE_SIZES = {
  box: ['size'], sphere: ['radius'], capsule: ['radius', 'height'], hull: ['vertices'], compound: ['parts'],
  mesh: ['vertices', 'indices'], heightfield: ['heights', 'columns', 'spacing'],
};
const SIZES = ['size', 'radius', 'height', 'vertices', 'indices', 'heights', 'columns', 'spacing', 'parts'];
/** What a compound's part can be, and what it takes. */
const PART_SHAPES = ['box', 'sphere', 'capsule', 'hull'];
const PART_OPTIONS = ['shape', 'size', 'radius', 'height', 'vertices', 'position', 'rotation', 'density'];
/** A 1 m cube's corners: a hull with no vertices given starts as the box a body starts as. */
const CUBE_CORNERS = [-0.5, -0.5, -0.5, 0.5, -0.5, -0.5, -0.5, 0.5, -0.5, 0.5, 0.5, -0.5,
  -0.5, -0.5, 0.5, 0.5, -0.5, 0.5, -0.5, 0.5, 0.5, 0.5, 0.5, 0.5];

const WORLD_OPTIONS = ['gravity', 'substeps', 'sleeping', 'workers'];
const DEBUG_OPTIONS = ['shapes', 'contacts', 'joints', 'boxes'];
const RAY_OPTIONS = ['maxDistance', 'mask', 'ignore'];
const CAST_OPTIONS = ['maxDistance', 'mask', 'ignore', 'rotation'];
const OVERLAP_OPTIONS = ['mask', 'ignore', 'rotation'];
const SHAPE_OPTIONS = ['shape', 'size', 'radius', 'height'];

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
//
// Measured again in milestone 16, after the solver's own speed-ups, against
// more scenes (the 1,000-box benchmark pile and three 288-box heaps too).
// Fewer substeps cost what can be seen: a contact's spring is tuned to the
// substep rate (STIFFNESS), so the 10-box stack sank 6.9 mm at 8, 9.0 at 7,
// 12 at 6 and 28 at 4; at 7 the pyramid strayed 4.6 mm with sleeping off
// (2.3 at 8) and the pile was still awake at 60 s. 7 would have saved an
// eighth of the solver's work. Fewer sweeps a manifold at 8 (before the block
// solve in contact.js's solve()) saved 9 to 16%, and cost the same way.
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

// SLEEPING. A body is still while no point of it strays more than its own
// tolerance, about a pixel, within any SLEEP_TIME: where it was is taken again
// each SLEEP_TIME it stays within it. An island sleeps once every body in it
// has been still for SLEEP_TIME. Moving steadily, that is any speed under its
// tolerance over SLEEP_TIME, 5 mm/s for a 1 m box and 0.05 mm/s for a 1 cm
// marble, however long it keeps it up.
//
// Measured from where it first came to rest instead, as this was before
// milestone 16, a box creeping at 2 mm/s strayed its 5 mm in 2.5 s, started
// again, and was still only 60% of the time; and an island sleeps only when
// all its bodies are still at once. The benchmark's 1,000-box pile, one
// island, had a few dozen such boxes among 900 and never slept. The creep is
// the solver's (friction pulls back what it can; see store() in contact.js),
// and freezing it is the point of sleeping.
//
// It is how far the body has gone that counts, not how fast it is going. A
// heap of 1,000 boxes measured with a speed rule had 909 still awake after
// 40 s: a few boxes buzzed at 1.3 cm/s, twice the speed that counts as still,
// while going nowhere (one strayed 0.18 mm in 30 s), and kept the whole heap
// awake. Going by distance, a buzz in place is still, and a creep faster than
// a pixel a second isn't.
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

// SPIN: the most a body other than a sphere turns in a step, π, half a turn.
// Its contacts are found once a step, where the step starts or at the moment
// of impact (contact.js, impact.js), and a body turning faster than that
// meets the world on sides those contacts never saw: measured, a 10 cm rod
// spun at 400 rad/s and dropped on another ended at 9,600 rad/s, and a chain
// of thin boxes whipping its end blew apart. Box2D 3 and Jolt cap spin at a
// quarter turn a step (47 rad/s at 60 Hz); half a turn (188 rad/s at 60 Hz)
// held every case measured, and scales with the step. A sphere's contacts
// don't depend on how it spins, so it spins as fast as it is made to, unless
// its centre of mass is off its centre and its centre swings round.
const SPIN = Math.PI;

// WORKERS (core/jobs.js). 'auto' is half the threads the machine reports,
// less the one stepping the world, and at least one where there are two:
// where a core runs two threads, the second shares its arithmetic, so one
// worker a core. Measured on the i7-1260P (4 P-cores with two threads each,
// 8 E-cores, 16 threads), steps of the 1,000-box pile and the pyramid were
// 2.5 and 2.5 times as fast with 3 workers, 2.8 with 4, 3.3 and 3.2 with 7
// (the rule's), 3.4 and 3.2 with 9, 3.3 and 3.0 with 11, and 2.5 and 2.0
// with 15, every thread spinning against the system's own; the 8,000-box
// pile, the rock heap and the terrain 2.4, 2.0 and 2.0 times with 3, 2.7,
// 2.2 and 2.1 with 4, and 3.3, 2.4 and 2.2 with 7. A worker on the stepping
// thread's own core made the pile 1.26 times as fast, and one on another
// core 2.1. A number given is never more than the threads less one, since
// more only wait on each other.
const auto = (threads) => (threads < 2 ? 0 : Math.max(1, Math.floor(threads / 2) - 1));
/** The worker module: Bearing's own code, in a thread of its own. */
const WORKER = new URL('./worker.js', import.meta.url);

const BODY_OPTIONS = [
  'type', 'shape', 'size', 'radius', 'height', 'vertices', 'indices', 'heights', 'columns', 'spacing', 'parts',
  'density', 'mass', 'inertia', 'centerOfMass',
  'friction', 'restitution', 'damping', 'angularDamping', 'gravityScale', 'layers', 'mask', 'sensor',
  'lockRotation', 'maxSlope', 'stepHeight',
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
  // A mesh starts empty; a heightfield as a flat 1 m square, two samples
  // by two, a metre apart, as a box starts as a 1 m cube.
  vertices: [], indices: [], heights: [0, 0, 0, 0], columns: 2, spacing: 1, geometry: null,
  density: 1000, mass: 0, massGiven: false, inertia: [0, 0, 0], inertiaGiven: false, centerOfMass: null,
  friction: 0.5, restitution: 0, damping: 0, angularDamping: 0, gravityScale: 1,
  // In the first layer, and colliding with all 32: everything collides with
  // everything until told otherwise (rule 110).
  layers: 1, mask: 0xffffffff, sensor: false, lockRotation: [false, false, false],
  // A character walks up ground as steep as friction 1 can hold, 45°: about
  // where walking turns into climbing, and Unity's, Godot's, Rapier's and
  // PhysX's default. Its step height follows its size (see _write).
  maxSlope: Math.PI / 4, stepHeight: 0, stepGiven: false,
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
  ['_inertia', Float64Array, 3],     // principal moments, kg·m², about the axes the step turns the body by
  // A hull's or compound's centre of mass is off its origin, and its principal
  // axes off its own: the step works from those (see _pose). Both are zero
  // turns and offsets for a box, ball or capsule, unless given a centre of mass.
  ['_offset', Float64Array, 3],      // the centre of mass from the origin, along the principal axes
  ['_frame', Float64Array, 4],       // the principal axes as a turn from the body's own: q = rotation ⊗ frame
  ['_framed', Uint8Array, 1],        // 1 if either is not zero
  ['_com', Float64Array, 3],         // the centre of mass in the body's own frame, as given or worked out
  ['_comGiven', Uint8Array, 1],
  ['_invInertia', Float64Array, 3],
  ['_tumbled', Float64Array, 3],     // ω as tumble() left it: unchanged since, it is still tumbling freely
  ['_ceiling', Float64Array, 1],     // twice the most energy it may tumble with, since it was last spun
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
  ['_invIt', Float64Array, 6],       // the same where a jointed body has turned to in the substep: what its joints read (contact.js turnedInertias())
  ['_start', Float64Array, 3],       // its position when the step began
  ['_startQ', Float64Array, 4],      // and its rotation, for a fast body caught after the solve (_catch)
  ['_fast', Uint8Array, 1],          // during _catch: 1 for a fast body, which is never another's target; 2 for one not swept
  ['_caught', Uint8Array, 1],        // during _catch: 1 for a body put back
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
  ['_sensor', Uint8Array, 1],        // 1 for a sensor: it reports what it overlaps, and pushes nothing
  ['_lock', Uint8Array, 1],          // bit k: it never turns about world axis k
  ['_colours', Int32Array, 1],       // while a step colours its manifolds: the colours this body's hold, as bits (graph.js)
  // Characters (character.js):
  ['_maxSlope', Float64Array, 1],
  ['_slopeCos', Float64Array, 1],    // cos(maxSlope): ground whose normal is this near up is walkable
  ['_stepHeight', Float64Array, 1],
  ['_stepGiven', Uint8Array, 1],
  ['_ground', Float64Array, 1],      // the id of what it stands on, or -1
  ['_groundAt', Float64Array, 6],    // where it stands on it, and the ground's normal there
  ['_walk', Float64Array, 3],        // its own velocity, through a step that moves it by more
  // Joints (joints.js): each body's list of them, and how long it is.
  ['_jointHead', Int32Array, 1],     // its first joint, as an edge (2j + side) plus 1; 0 for none
  ['_jointTally', Int32Array, 1],
  ['_node', Int32Array, 1],          // while a step finds its trees of joints: the body's node plus 1 (forest.js)
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
     * By slot, the shape as given: a mesh's or heightfield's triangles
     * (mesh.js), a hull ({ hull }, in the body's own frame) or a compound's
     * parts ({ parts }); null for a box, ball or capsule.
     */
    this._geometry = [];
    /**
     * By slot, the shape as the step sees it, in the frame of its centre of
     * mass and principal axes: a hull's Hull, or Parts for a compound or a
     * box, ball or capsule given a centre of mass; else null.
     */
    this._inner = [];
    /** By slot, a hull body's placement, kept for the step (collide.js Placed.fresh); scratch, not saved. */
    this._placed = [];
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
    /** Every joint, in columns of its own (joints.js). */
    this._joints = new Joints();
    /** Steps that ran the joint solver: for the off-means-off test, which reads it. */
    this._jointSteps = 0;
    /** Times of impact worked out (contact.js): for the off-means-off test, which reads it. */
    this._impacts = 0;
    /** Steps that had a fast body to sweep, and bodies put back (_catch): for the tests, which read them. */
    this._catchSteps = 0;
    this._catches = 0;
    this._substeps = o.substeps === undefined ? SUBSTEPS : wholeFromOne('World', 'substeps', o.substeps);
    /** Steps that ran the contact solver: for the off-means-off test, which reads it. */
    this._solverSteps = 0;
    this._sleeping = o.sleeping === undefined ? true : flag('World', 'sleeping', o.sleeping);
    /** The workers setting as given, and the job system it makes, or null for none (see WORKERS). */
    this._workers = 'auto';
    this._jobs = null;
    this._setWorkers('World', 'workers', o.workers === undefined ? 'auto' : o.workers);
    this._stepCount = 0;
    this._reset();
    /** Steps that looked for islands: for the off-means-off test, which reads it. */
    this._islandSteps = 0;
    this._destroyed = false;
    /** debugLines()'s array, reused from call to call (debug.js); empty until asked for. */
    this._lines = new Float32Array(0);
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
    // Contact events (events.js): the last step's, written only when a pair
    // starts or stops touching. And the pairs with a sensor in them that the
    // step found, to ask about once it is over (_senseAll).
    this._events = new Events();
    this._sensed = new Int32Array(0);
    this._sensedCount = 0;
    this._sensorCount = 0;
    this._characterCount = 0;
    /** Bodies whose centre of mass or principal axes are off their origin's: transforms convert only those. */
    this._framedCount = 0;
    /** Steps that asked about sensor pairs: for the off-means-off test, which reads it. */
    this._sensorPasses = 0;
    /** Steps that moved characters: for the same. */
    this._characterSteps = 0;
    // Queries (query.js): bodies that ended the last step outside their fat
    // boxes, listed by the first query after the step, and anything that
    // adds, removes or moves a body by hand since.
    this._strays = [];
    this._straysAt = -1;
    this._straysEpoch = -1;
    this._epoch = 0;
    // Moved on by anything that can change whether two bodies are the same
    // ones or may collide: removing a body, setBody, a joint added, changed or
    // dropped. The pair list checks a pair again only when it has (pairs.js holds).
    this._pairEpoch = 0;
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

  /**
   * How many threads besides this one may share a step's work: 'auto', or
   * at most this many. Changing it ends the workers it had.
   */
  get workers() {
    this._live('workers');
    return this._workers;
  }

  set workers(value) {
    this._live('workers');
    this._setWorkers(null, 'world.workers', value);
  }

  /**
   * Checks and sets the workers setting, and makes the job system for it:
   * none for 0, where shared memory is missing, or on a machine of one
   * thread. Its workers start later, when a step first has work for them.
   */
  _setWorkers(caller, name, value) {
    if (value !== 'auto' && (!Number.isInteger(value) || value < 0)) {
      fail(caller, `${name} must be 'auto' or a whole number, 0 or more, got ${show(value)}`);
    }
    if (this._jobs !== null && this._jobs !== undefined) this._jobs.destroy();
    this._workers = value;
    const threads = cores(), count = Math.min(value === 'auto' ? auto(threads) : value, threads - 1);
    this._jobs = sharedMemory && count > 0 ? new Jobs(WORK, count, WORKER) : null;
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

  get jointCount() {
    this._live('jointCount');
    return this._joints.count;
  }

  set jointCount(_) { readOnly('world.jointCount'); }

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
    this._epoch++;
    this._pairEpoch++;
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
      const p = this._position, st = this._start, s3 = s * 3;
      st[s3] = p[s3]; st[s3 + 1] = p[s3 + 1]; st[s3 + 2] = p[s3 + 2];
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
    const g = spec.geometry;
    if (spec.shape === 'mesh') {
      out.vertices = Array.from(g.vertices);
      out.indices = Array.from(g.indices);
    } else if (spec.shape === 'heightfield') {
      out.heights = Array.from(g.heights);
      out.columns = g.columns;
      out.spacing = g.spacing;
    } else {
      if (spec.shape === 'hull') out.vertices = Array.from(g.hull.vertices);
      else if (spec.shape === 'compound') out.parts = g.parts.map(partOf);
      else for (const k of SHAPE_SIZES[spec.shape]) out[k] = k === 'size' ? [...spec.size] : spec[k];
      // Whichever the body was given, so addBody(world.bodyOf(b)) makes a copy.
      if (spec.massGiven) out.mass = spec.mass;
      else out.density = spec.density;
      if (spec.inertiaGiven) out.inertia = [...spec.inertia];
      if (spec.centerOfMass !== null) out.centerOfMass = [...spec.centerOfMass];
    }
    for (const k of MATERIAL) out[k] = spec[k];
    out.lockRotation = [...spec.lockRotation];
    if (spec.type === 'character') {
      out.maxSlope = spec.maxSlope;
      if (spec.stepGiven) out.stepHeight = spec.stepHeight;
    }
    out.position = [...spec.position];
    out.rotation = [...spec.rotation];
    if (spec.type !== 'static') {
      out.velocity = [...spec.velocity];
      out.angularVelocity = [...spec.angularVelocity];
    }
    return out;
  }

  /** Removes a body, with its joints, or a joint. */
  remove(handle) {
    this._remove('remove', handle);
  }

  // ---- saving and drawing ----

  /** The whole world as plain, JSON-safe data, for World.load() (save.js). */
  save() {
    this._live('save');
    return save(this, World);
  }

  /** The world save() wrote `data` from, ready to step exactly as it would have. */
  static load(data) {
    return load(data, World);
  }

  /**
   * The world as line segments, nine numbers each: two ends and a colour
   * (debug.js). The world's own array, replaced by the next call.
   */
  debugLines(options_) {
    this._live('debugLines');
    const o = options('debugLines', options_);
    known('debugLines', o, DEBUG_OPTIONS);
    const pick = (k, d) => (o[k] === undefined ? d : flag('debugLines', k, o[k]));
    return debugLines(this, { shapes: pick('shapes', true), contacts: pick('contacts', true), joints: pick('joints', true), boxes: pick('boxes', false) });
  }

  // ---- joints (joints.js) ----

  addJoint(options_) {
    return addJoint(this, options_);
  }

  setJoint(joint, changes) {
    setJoint(this, joint, changes);
  }

  jointOf(joint) {
    return jointOf(this, jointSlot(this, 'jointOf', joint));
  }

  /**
   * The last step's contact events: which pairs began touching and which
   * stopped, as fresh plain objects. Given a body, only its own, each turned
   * so that `a` is that body. A body removed since still has its events, its
   * handle reading as dead, so a removed body is taken here.
   */
  contacts(body) {
    this._live('contacts');
    let id = -1;
    if (body !== undefined) {
      if (!(body instanceof Body)) fail('contacts', `body must be a Body, got ${show(body)}`);
      if (body.world !== this) fail('contacts', 'that body belongs to another world');
      id = body.id;
    }
    const e = this._events, out = [];
    for (let i = 0; i < e.count; i++) {
      let a = e.idA[i], b = e.idB[i];
      const turned = id >= 0 && b === id;
      if (id >= 0 && a !== id && !turned) continue;
      if (turned) { b = a; a = id; }
      if (e.type[i] === END) {
        out.push({ type: 'end', a: new Body(this, a), b: new Body(this, b) });
        continue;
      }
      // Turned round, the normal points the other way; 0 - x keeps a zero from becoming -0.
      const i3 = 3 * i, n = e.normal, k = (x) => (turned ? 0 - x : x);
      out.push({
        type: 'begin', a: new Body(this, a), b: new Body(this, b),
        point: [e.point[i3], e.point[i3 + 1], e.point[i3 + 2]],
        normal: [k(n[i3]), k(n[i3 + 1]), k(n[i3 + 2])],
        speed: e.speed[i],
      });
    }
    return out;
  }

  /**
   * What a character stands on, as of the last step: { body, point, normal },
   * fresh copies, or null in the air. null for any other body, which no step
   * looks for ground under.
   */
  groundOf(body) {
    const s = this._slotOfBody('groundOf', body);
    if (this._type[s] !== CHARACTER || this._ground[s] < 0) return null;
    const a = this._groundAt, g6 = 6 * s;
    return {
      body: new Body(this, this._ground[s]),
      point: [a[g6], a[g6 + 1], a[g6 + 2]],
      normal: [a[g6 + 3], a[g6 + 4], a[g6 + 5]],
    };
  }

  /**
   * Every body touching this one, as of the last step, or overlapping it
   * where either is a sensor. Found through the broadphase tree, from the
   * body's own box: the bodies near it, not every pair in the world.
   */
  touching(body) {
    const s = this._slotOfBody('touching', body);
    const bp = this._broadphase, pairs = this._pairs, slots = this._slots, t = bp.which[s], out = [];
    if (t < 0) return out;
    const box = bp.trees[t].box, o = 6 * bp.leaf[s];
    const visit = (other) => {
      if (other === s) return;
      const a = s < other ? s : other, b = s < other ? other : s, at = pairs._find(a, b);
      if (at < 0) return;
      const p = pairs.index[at];
      if (pairs.touching[p] === 1 && slots.slotOf(pairs.idA[p]) === a && slots.slotOf(pairs.idB[p]) === b) {
        out.push(new Body(this, slots.id(other)));
      }
    };
    for (const tree of bp.trees) tree.query(box[o], box[o + 1], box[o + 2], box[o + 3], box[o + 4], box[o + 5], visit);
    return out;
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

  // ---- queries (query.js) ----

  /** The nearest body the ray from `origin` along `direction` meets, or null. */
  raycast(origin, direction_, options_) {
    const p = this._origin('raycast', origin), d = direction('raycast', 'direction', direction_);
    const o = this._queryOptions('raycast', options_, RAY_OPTIONS);
    let found = null;
    // Only ever nearer hits are reported after the first, so the last is the nearest.
    ray(this, p[0], p[1], p[2], d[0], d[1], d[2], o.maxDistance, o.mask, o.ignore, (s, t, n) => {
      found = this._hit(s, t, n, p[0] + t * d[0], p[1] + t * d[1], p[2] + t * d[2]);
    }, true);
    return found;
  }

  /** Every body the ray meets, nearest first. */
  raycastAll(origin, direction_, options_) {
    const p = this._origin('raycastAll', origin), d = direction('raycastAll', 'direction', direction_);
    const o = this._queryOptions('raycastAll', options_, RAY_OPTIONS);
    const hits = [];
    ray(this, p[0], p[1], p[2], d[0], d[1], d[2], o.maxDistance, o.mask, o.ignore, (s, t, n) => {
      hits.push(this._hit(s, t, n, p[0] + t * d[0], p[1] + t * d[1], p[2] + t * d[2]));
    }, false);
    // Ties go to the earlier slot, so the order depends only on the inputs.
    return hits.sort((a, b) => a.distance - b.distance || a.body.slot - b.body.slot);
  }

  /** The first body `shape`, moving from `origin` along `direction`, would touch, or null. */
  shapeCast(shape, origin, direction_, options_) {
    const s = this._queryShape('shapeCast', shape);
    const p = this._origin('shapeCast', origin), d = direction('shapeCast', 'direction', direction_);
    const o = this._queryOptions('shapeCast', options_, CAST_OPTIONS);
    s.place(p[0], p[1], p[2], o.rotation);
    let found = null;
    cast(this, s, d[0], d[1], d[2], o.maxDistance, o.mask, o.ignore, (slot, t, n, at) => {
      found = this._hit(slot, t, n, at[0], at[1], at[2]);
    });
    return found;
  }

  /** The bodies `shape`, placed at `position`, overlaps or touches. */
  overlap(shape, position, options_) {
    const s = this._queryShape('overlap', shape);
    const p = vec3('overlap', 'position', position);
    const o = this._queryOptions('overlap', options_, OVERLAP_OPTIONS);
    s.place(p[0], p[1], p[2], o.rotation);
    const bodies = [];
    overlapping(this, s, o.mask, o.ignore, (slot) => bodies.push(new Body(this, this._slots.id(slot))));
    return bodies;
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
    // Events are the last step's: this one's start afresh.
    this._events.count = 0;
    this._sensedCount = 0;
    const tr = this._transforms, prev = this._previous;

    if (this._forcedSlots.length > 0) this._applyForces(dt);
    // What a setter woke since the last step, or a removal or a static body
    // moved by hand disturbed, brings its pairs back among the awake ones.
    if (this._wakePending) this._syncPairs();
    if (this._movedStatics.length > 0) this._clearStatics();
    // Characters find where they can go before anything moves (character.js).
    if (this._characterCount > 0) {
      this._characterSteps++;
      walk(this, dt);
    }

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
    // Joints the same way: one holding a sleeping body to an awake one wakes
    // it, and the joints to solve are those with an awake body (joints.js).
    next.count = 0;
    next.onParts = 0;
    next.wasPlain = next.plain;
    next.plain = 0;
    let from = 0;
    const joints = this._joints, jointed = joints.count > 0;
    do {
      if (this._wakePending) this._syncPairs();
      narrow(this, pairs, broadphase, next, from);
      from = pairs.active;
      if (jointed) gather(this);
    } while (this._wakePending);
    next.trim();
    pairs.trim();
    const touching = next.count, held = jointed ? joints.active : 0;
    // Coloured, and the rows laid out colour by colour, before anything else
    // is worked out for them (graph.js).
    if (touching > 0) colour(this, next);
    if (held > 0) plantJoints(this);

    const substeps = this._substeps, h = dt / substeps;
    if (touching > 0 || held > 0) inertias(this);
    {
      // Where each awake body starts: what the solver measures its movement
      // from, and where _catch sweeps it from. Static and sleeping bodies
      // don't move, so theirs stays where it was set.
      const p = this._position, q = this._rotation, start = this._start, startQ = this._startQ;
      for (let i = 0; i < this._awakeCount; i++) {
        const s3 = list[i] * 3, s4 = list[i] * 4;
        start[s3] = p[s3]; start[s3 + 1] = p[s3 + 1]; start[s3 + 2] = p[s3 + 2];
        startQ[s4] = q[s4]; startQ[s4 + 1] = q[s4 + 1]; startQ[s4 + 2] = q[s4 + 2]; startQ[s4 + 3] = q[s4 + 3];
      }
    }
    // The workers (core/jobs.js) take their share of preparing the contacts
    // and of each colour's solve, from here to the last substep, if the
    // step's widest colour is worth them. Whatever happens, they go back to
    // sleep after: left spinning, they would hold cores for nothing. Moving
    // the bodies (_accelerate, _move) stays on this thread: measured, 1.1% of
    // the landing pile's step and 0.6% of the pyramid's, so sharing it could
    // save no more than that, for two more dispatches a substep.
    const jobs = this._jobs, parallel = jobs !== null && jobs.begin(this, next, touching > 0 && crowded(next, jobs.least));
    try {
      if (touching > 0) {
        this._solverSteps++;
        prepare(this, next);
        makeSoft(STIFFNESS / h, DAMPING_RATIO, h);
      }
      if (held > 0) {
        this._jointSteps++;
        makeSoft(JOINT_STIFFNESS / h, JOINT_DAMPING_RATIO, h, jointSoft);
      }
      // Contacts before joints, in each pass: what is solved last has the last
      // word on the velocities, and it has to be the joints. A contact sees
      // each body alone, a 0.5 kg forearm as 0.5 kg, though the joints tie it
      // to a whole ragdoll; the tree solve (forest.js) sees them all, and
      // hands a contact's push on to everything the joints tie it to. Solved
      // the other way, as Box2D 3 orders its soft joints, a contact's push
      // opened the joints, the next substep's solve pulled them shut in one go
      // (JOINT_BIAS) and drove the limb back into the contact: limbs at rest
      // rattled at 1 to 5 cm/s step to step, crept, and 32 ragdolls never
      // slept, their joints opening up to 13 cm (s17 in the settling audit).
      // This way none rattle, the joints hold within half a millimetre, and a
      // limb pressed into the floor by what lies on it sinks no deeper.
      // The joints run on this thread alone, so the workers are told (lull),
      // each time just after the contact work that went before: left spinning
      // through a long joint pass, they slowed it (core/jobs.js). The relax
      // pass's stretch runs on into the next substep's warm start.
      const lull = parallel && held > 0;
      if (held > 0) turnedInertias(this);
      for (let sub = 0; sub < substeps; sub++) {
        this._accelerate(h);
        if (touching > 0) warmStart(this, next);
        if (lull) jobs.lull(0);
        if (held > 0) warmStartJoints(this);
        if (touching > 0) solve(this, next, h, true, sub > 0);
        if (lull) jobs.lull(1);
        if (held > 0) solveJoints(this, h, true);
        this._move(h);
        if (held > 0) turnedInertias(this);
        if (touching > 0) solve(this, next, h, false);
        if (lull) jobs.lull(2);
        if (held > 0) solveJoints(this, h, false);
      }
    } finally {
      if (parallel) jobs.end();
    }
    if (touching > 0) {
      const g = this._gravity;
      restitution(this, next, Math.sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]));
      store(this, next);
    }
    // The catch's sweeps look for bodies out of their fat boxes where the
    // solve left them: a character's casts listed them as the step began, and
    // with the same step and epoch that list would be taken for this one.
    this._straysAt = -1;
    this._catch(next);
    if (touching > 0) touched(this, pairs, next);

    const p = this._position, q = this._rotation, framed = this._framedCount > 0 ? this._framed : null;
    for (let i = 0; i < this._awakeCount; i++) {
      const s = list[i], t7 = s * 7, s3 = s * 3, s4 = s * 4;
      if (framed !== null && framed[s] === 1) {
        this._pose(s, POSE);
        for (let k = 0; k < 7; k++) tr[t7 + k] = POSE[k];
      } else {
        tr[t7] = p[s3]; tr[t7 + 1] = p[s3 + 1]; tr[t7 + 2] = p[s3 + 2];
        tr[t7 + 3] = q[s4]; tr[t7 + 4] = q[s4 + 1]; tr[t7 + 5] = q[s4 + 2]; tr[t7 + 6] = q[s4 + 3];
      }
      at[s] = step;
    }
    this._stepDt = 0;
    if (this._characterCount > 0) settle(this);
    if (this._sensedCount > 0) this._senseAll();
    // Bodies out of their fat boxes are listed afresh for the next query: a
    // character's casts listed them as the step began.
    this._straysAt = -1;
    // Both give storage back after a burst, by the world's quarter rule.
    this._events.trim();
    if (this._sensed.length > 16 && this._sensedCount <= this._sensed.length / 4) {
      let c = 16;
      while (c < this._sensedCount) c *= 2;
      this._sensed = resized(this._sensed, c);
    }
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
    const size = this._size, reach = this._reach, type = this._type, v = this._velocity, w = this._angular;
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
      // A kinematic body moves only as it is told, so it is still exactly
      // while told nothing: measured by distance, a slow platform, a 10 m one
      // at 1 cm/s, would count as still, fall asleep and stop.
      // A character never sleeps, so whatever it stands on can always carry it.
      // ponytail: a few casts a step for each one standing still; let them
      // sleep, woken by their ground moving, if a scene of thousands measures it.
      if (type[s] === CHARACTER ? false : type[s] === KINEMATIC ? v[s3] === 0 && v[s3 + 1] === 0 && v[s3 + 2] === 0
        && w[s3] === 0 && w[s3 + 1] === 0 && w[s3 + 2] === 0 : strayed < size[s] * PIXEL) {
        const before = still[s];
        still[s] += dt;
        if (still[s] >= SLEEP_TIME) ready = true;
        // Each whole SLEEP_TIME still, where it rests is taken again.
        if (Math.floor(still[s] / SLEEP_TIME) !== Math.floor(before / SLEEP_TIME)) {
          rest[r7] = p[s3]; rest[r7 + 1] = p[s3 + 1]; rest[r7 + 2] = p[s3 + 2];
          rest[r7 + 3] = q[s4]; rest[r7 + 4] = q[s4 + 1]; rest[r7 + 5] = q[s4 + 2]; rest[r7 + 6] = q[s4 + 3];
        }
      } else {
        this._settle(s);
      }
    }
    if (!ready) return;
    this._islandSteps++;

    const parent = this._parent, c = this._contacts, n = this._awakeCount;
    for (let i = 0; i < n; i++) parent[list[i]] = list[i];
    for (let m = 0; m < c.count; m++) join(parent, type, c.a[m], c.b[m]);
    // Joined bodies sleep and wake together too: the joints solved this step
    // are every one with an awake body.
    const J = this._joints;
    for (let i = 0; i < J.active; i++) {
      const j = J.list[i];
      if (J.b[j] >= 0) join(parent, type, J.a[j], J.b[j]);
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
    // Copied one by one: subarray() makes a view object for each, and this
    // runs for nearly every body every step while a pile lands.
    const p = this._position, q = this._rotation, rest = this._rest, s3 = s * 3, s4 = s * 4, r7 = s * 7;
    rest[r7] = p[s3]; rest[r7 + 1] = p[s3 + 1]; rest[r7 + 2] = p[s3 + 2];
    rest[r7 + 3] = q[s4]; rest[r7 + 4] = q[s4 + 1]; rest[r7 + 5] = q[s4 + 2]; rest[r7 + 6] = q[s4 + 3];
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

  _origin(caller, value) {
    this._live(caller);
    return vec3(caller, 'origin', value);
  }

  /** A query's options, checked, with their defaults. */
  _queryOptions(caller, given, names) {
    const o = options(caller, given);
    known(caller, o, names);
    let maxDistance = Infinity;
    if (o.maxDistance !== undefined) {
      const m = o.maxDistance;
      if (typeof m !== 'number' || !(m >= 0)) fail(caller, `maxDistance must be 0 or more, got ${show(m)}`);
      maxDistance = m;
    }
    const mask = o.mask === undefined ? 0xffffffff : bits(caller, 'mask', o.mask);
    // A removed body leaves nothing to leave out; one from another world is a mistake.
    let ignore = -1;
    if (o.ignore !== undefined && o.ignore !== null) {
      if (!(o.ignore instanceof Body) || o.ignore.world !== this) fail(caller, `ignore must be a Body of this world, got ${show(o.ignore)}`);
      ignore = this._slots.slotOf(o.ignore.id);
    }
    const rotation = o.rotation === undefined ? [0, 0, 0, 1] : quat(caller, 'rotation', o.rotation);
    return { maxDistance, mask, ignore, rotation };
  }

  /**
   * A query's shape, checked as addBody checks a body's, with the same names
   * and defaults, into one Shape reused by every query.
   */
  _queryShape(caller, given) {
    this._live(caller);
    const o = options(caller, given, 'shape');
    known(caller, o, SHAPE_OPTIONS);
    const has = (k) => o[k] !== undefined;
    const shape = has('shape') ? oneOf(caller, 'shape', o.shape, QUERY_SHAPES) : DEFAULTS.shape;
    for (const k of ['size', 'radius', 'height']) {
      if (has(k) && !SHAPE_SIZES[shape].includes(k)) fail(caller, `a ${shape} has no ${k}`);
    }
    const s = QUERY;
    if (shape === 'box') {
      const size = has('size') ? positive3(caller, 'size', o.size) : DEFAULTS.size;
      s.kind = BOX_KIND;
      s.half[0] = size[0] * 0.5; s.half[1] = size[1] * 0.5; s.half[2] = size[2] * 0.5;
      return s;
    }
    const radius = has('radius') ? positive(caller, 'radius', o.radius) : DEFAULTS.radius;
    if (shape === 'sphere') {
      s.kind = SPHERE_KIND;
      s.half[0] = radius;
      return s;
    }
    const height = has('height') ? positive(caller, 'height', o.height) : DEFAULTS.height;
    if (height < 2 * radius) fail(caller, `height must be at least twice radius (${2 * radius}), got ${height}`);
    s.kind = CAPSULE_KIND;
    s.half[0] = radius;
    s.half[1] = (height - 2 * radius) * 0.5;
    return s;
  }

  /** A query's result: plain data, fresh copies, and a handle that can be kept. */
  _hit(s, distance, n, x, y, z) {
    return { body: new Body(this, this._slots.id(s)), point: [x, y, z], normal: [n[0], n[1], n[2]], distance };
  }

  /** Marks slot s as moved by hand, or added, since the last step. */
  _stamp(s) {
    this._epoch++;
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
    const damping = this._damping, angularDamping = this._angularDamping, shape = this._shape;
    const most = SPIN / (h * this._substeps);
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
      // At most half a turn a step, but for a sphere: see SPIN. Capped as
      // each substep starts, as Box2D 3 does, so a contact within it can
      // still turn a body as fast as it must: a rod struck at one end turns
      // about it, and capped after the solve it went on into the wall. Time
      // of impact (impact.js) predicts a body's path with the same cap.
      if (shape[s] !== SPHERE || this._framed[s] === 1) {
        const w2 = w[s3] * w[s3] + w[s3 + 1] * w[s3 + 1] + w[s3 + 2] * w[s3 + 2];
        if (w2 > most * most) {
          const k = most / Math.sqrt(w2);
          w[s3] *= k; w[s3 + 1] *= k; w[s3 + 2] *= k;
        }
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
      // A body locked about any axis turns only as its locks allow, held there
      // as by a constraint, so it doesn't tumble as a free one does. Nor does
      // a body a joint holds: tumbling carries its angular momentum across the
      // substep unchanged, as if nothing held it, and measured on a floating
      // leg (a box, a hinged thigh and shin), that added 10% to its energy in
      // 50 s; turned plainly, it only ever lost energy, 6.6% in 60 s.
      if (turning && type[s] === DYNAMIC && this._lock[s] === 0 && this._jointHead[s] === 0
        && (I[s3] !== I[s3 + 1] || I[s3 + 1] !== I[s3 + 2])) {
        tumble(q, s4, I, this._invInertia, s3, w, h, this._tumbled, this._ceiling, s);
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
      axesOf(q, s4, ax, a9);

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
      // Where the box's centre is from the body's: off it only for level
      // geometry, hulls and parts.
      let ox = 0, oy = 0, oz = 0;
      const inner = this._inner[s];
      if (shape === BOX || shape >= MESH || inner !== null) {
        let hx = d[s3] * 0.5, hy = d[s3 + 1] * 0.5, hz = d[s3 + 2] * 0.5;
        if (shape >= MESH || inner !== null) {
          // The box round its triangles, corners or parts in its own frame, turned into the world's.
          const g = inner !== null ? inner : this._geometry[s], lo = g.lo, hi = g.hi;
          const cx = (lo[0] + hi[0]) * 0.5, cy = (lo[1] + hi[1]) * 0.5, cz = (lo[2] + hi[2]) * 0.5;
          hx = (hi[0] - lo[0]) * 0.5; hy = (hi[1] - lo[1]) * 0.5; hz = (hi[2] - lo[2]) * 0.5;
          ox = ax[a9] * cx + ax[a9 + 3] * cy + ax[a9 + 6] * cz;
          oy = ax[a9 + 1] * cx + ax[a9 + 4] * cy + ax[a9 + 7] * cz;
          oz = ax[a9 + 2] * cx + ax[a9 + 5] * cy + ax[a9 + 8] * cz;
        }
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
      const cx = p[s3] + ox, cy = p[s3 + 1] + oy, cz = p[s3 + 2] + oz;
      box[b6] = cx - ex - grow; box[b6 + 1] = cy - ey - grow; box[b6 + 2] = cz - ez - grow;
      box[b6 + 3] = cx + ex + grow; box[b6 + 4] = cy + ey + grow; box[b6 + 5] = cz + ez + grow;
    }
  }

  /**
   * Fast bodies the solve put through something, put back where they would
   * have met it (Box2D 3's continuous step).
   *
   * WHY. A step's contacts are found once, where the step starts or at the
   * moment of impact (contact.js, impact.js), and the solver holds them by
   * the velocity of each point. A body struck hard off its centre is spun up
   * within a substep: a 10 cm rod meeting a wall corner-first at 200 m/s
   * needs some 3,000 rad/s to stop that corner, turns 100° in the substep
   * that follows, and sweeps the rest of itself through the wall where no
   * contact reaches. Measured, rods turned any way went through a 1 cm wall
   * 57 times in 100; re-finding their contacts at each substep left 31, and
   * a lower spin cap 53.
   *
   * So after the solve, each fast body (its centre moved more than half its
   * thinnest size, Box2D's measure: less, and contacts found a step ahead
   * already see whatever it can reach) sweeps its core, a ball at its centre
   * a quarter of its thinnest size across, from where the step started it to
   * where the step ended it. Against everything it collides with, as it is
   * at the end of the step, but other fast bodies. If the core meets
   * something, the body goes back to where it did, turned in proportion, and
   * its velocity into the surface is returned as a contact's restitution would
   * return it; its contacts meet properly in the next step. It loses the rest
   * of this step's motion, which is the alternative to going through. Its
   * velocity kept, a rod caught at a mesh, which has no thickness, drove on
   * in at 21 m/s, and its centre crossed the plane in the next step. A core
   * already at a surface and heading in stays where it started. The core, not the whole shape: swept whole, a spinning rod
   * bouncing off a floor was caught mid-bounce, put back up its path with its
   * velocity, and gained 27% of its energy; a core meets anything only when
   * the body's middle would. Measured: rods, cubes and planks turned any way,
   * spinning or not, 0 through in 100 each, and a rod into a 1 cm steel plate
   * floating free 0 in 100 (47 before).
   *
   * Not a body whose joints hold it, through its tree, to something that
   * doesn't move as the solve moves it: a kinematic or static body, or a
   * point in the world. Put back alone, it leaves its joints open by as far
   * as it went, and the next solve pass shuts them, all of it at once
   * (JOINT_BIAS), from the side that can't come back: the joints drag it on
   * into what it met, at whatever speed shuts the gap in a substep, to be
   * caught again. Measured, eight links hung from a plate turning at 1 rad/s,
   * swept into a post: caught there step after step, links stayed inside it
   * for 737 link-steps and thrashed at 99.5 m/s, 20 times the plate's 5 m/s
   * at the anchor; let go, they pass through it within 17, at 20.3 m/s,
   * their speed with no post. A kinematic body goes where it is told, and
   * what it holds goes with it. A free tree's bodies are still caught: what
   * pulls one back is the rest of the tree, which gives, and two rods on a
   * ball joint thrown at 200 m/s at a 1 cm wall went through it 18 times in
   * 40 when no jointed body was caught, and once, as before, as it is.
   *
   * Reads the bodies' end poses and _start/_startQ; changes only the caught.
   */
  _catch(next) {
    const list = this._awakeList, n = this._awakeCount, type = this._type, size = this._size;
    const p = this._position, q = this._rotation, st = this._start, sq = this._startQ, fast = this._fast;
    let count = 0;
    for (let i = 0; i < n; i++) {
      const s = list[i], s3 = 3 * s;
      if (type[s] !== DYNAMIC) continue;
      const dx = p[s3] - st[s3], dy = p[s3 + 1] - st[s3 + 1], dz = p[s3 + 2] - st[s3 + 2], half = 0.5 * size[s];
      if (dx * dx + dy * dy + dz * dz > half * half) { fast[s] = 1; count++; }
    }
    if (count === 0) return;
    this._catchSteps++;
    // Those held to what doesn't move (see above) are marked 2: still fast,
    // and so no target, but not swept. Only this step's forest is read.
    const J = this._joints;
    if (J.count > 0 && J.active > 0 && J.forest.count > 0) anchored(this, fast);
    let caught = 0;
    for (let i = 0; i < n; i++) {
      const s = list[i], s3 = 3 * s, s4 = 4 * s;
      if (fast[s] !== 1) continue;
      const dx = p[s3] - st[s3], dy = p[s3 + 1] - st[s3 + 1], dz = p[s3 + 2] - st[s3 + 2];
      const length = Math.sqrt(dx * dx + dy * dy + dz * dz);
      CORE.kind = SPHERE_KIND;
      CORE.half[0] = 0.125 * size[s];
      CORE.place(st[s3], st[s3 + 1], st[s3 + 2], IDENTITY, 0);
      const hit = sweep(this, CORE, s, -1, dx / length, dy / length, dz / length, length, SWEPT, fast, count);
      if (hit < 0) continue;
      SWEPT_SLOT[0] = hit;
      // Back to where its core met: along its path, and turned that far
      // between its rotations (normalized lerp, on the shorter way round).
      const f = SWEPT[0] / length;
      p[s3] = st[s3] + dx * f; p[s3 + 1] = st[s3 + 1] + dy * f; p[s3 + 2] = st[s3 + 2] + dz * f;
      const sign = sq[s4] * q[s4] + sq[s4 + 1] * q[s4 + 1] + sq[s4 + 2] * q[s4 + 2] + sq[s4 + 3] * q[s4 + 3] < 0 ? -1 : 1;
      let l = 0;
      for (let k = 0; k < 4; k++) {
        const v = (1 - f) * sq[s4 + k] + f * sign * q[s4 + k];
        TURNED[k] = v;
        l += v * v;
      }
      l = 1 / Math.sqrt(l);
      for (let k = 0; k < 4; k++) q[s4 + k] = TURNED[k] * l;
      // It met the surface: its velocity into it goes, as a contact's
      // restitution would return it, mixed as contacts mix it (the larger).
      const nx = SWEPT[1], ny = SWEPT[2], nz = SWEPT[3], v = this._velocity;
      const into = v[s3] * nx + v[s3 + 1] * ny + v[s3 + 2] * nz;
      if (into < 0) {
        const k = (1 + Math.max(this._restitution[s], this._restitution[SWEPT_SLOT[0]])) * into;
        v[s3] -= k * nx; v[s3 + 1] -= k * ny; v[s3 + 2] -= k * nz;
      }
      this._caught[s] = 1;
      this._catches++;
      caught++;
    }
    for (let i = 0; i < n; i++) fast[list[i]] = 0;
    if (caught === 0) return;
    // Its contacts' gaps were read where the solve left it: measured again
    // where it is now, for whether each pair still touches (contact.js).
    const c = this._caught;
    for (let m = 0; m < next.count; m++) if (c[next.a[m]] === 1 || c[next.b[m]] === 1) next.gap[m] = gapOf(this, next, m);
    for (let i = 0; i < n; i++) c[list[i]] = 0;
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
    if (this._lock[s] !== 0) {
      // About its free axes only: the inverse of its inertia there (contact.js).
      lockedInverse(TURN.place(0, 0, 0, this._rotation, s4).axes, 0, this._inertia, s3, this._lock[s], M6, 0);
      this._angular[s3] += M6[0] * x + M6[1] * y + M6[2] * z;
      this._angular[s3 + 1] += M6[1] * x + M6[3] * y + M6[4] * z;
      this._angular[s3 + 2] += M6[2] * x + M6[4] * y + M6[5] * z;
      return;
    }
    const inv = this._invInertia;
    rotate(this._rotation, s4, x, y, z, true, v3);
    rotate(this._rotation, s4, v3[0] * inv[s3], v3[1] * inv[s3 + 1], v3[2] * inv[s3 + 2], false, v3);
    this._angular[s3] += v3[0]; this._angular[s3 + 1] += v3[1]; this._angular[s3 + 2] += v3[2];
  }

  /** Lets go of every body and all storage, and ends its workers. Using the world afterwards throws; calling this again does nothing. */
  destroy() {
    if (this._destroyed) return;
    this._destroyed = true;
    if (this._jobs !== null) this._jobs.destroy();
    this._jobs = null;
    this._slots = new Slots();
    for (const [name, Type] of COLUMNS) this[name] = new Type(0);
    this._geometry = [];
    this._inner = [];
    this._placed = [];
    this._lines = new Float32Array(0);
    this._forcedSlots = [];
    this._broadphase = new Broadphase();
    this._pairs = new Pairs(this);
    this._contacts = new Contacts();
    this._joints = new Joints();
    this._reset();
  }

  /** Pair p has a sensor in it: the narrowphase leaves it to be asked about once the step is over. */
  _sense(p) {
    if (this._sensedCount === this._sensed.length) this._sensed = resized(this._sensed, Math.max(16, 2 * this._sensed.length));
    this._sensed[this._sensedCount++] = p;
  }

  /**
   * Every pair with a sensor in it, where the step left its bodies: touching
   * if within the tolerance of it, the same pixel as for any contact, and an
   * event if that changed. A begin carries the point and normal found, and
   * how fast b closes on a along it there. A sensor's pairs are asked only
   * here, after the step, so a body that passes all the way through a sensor
   * within one step isn't seen (continuous collision, milestone 13, is what
   * would see it).
   */
  _senseAll() {
    this._sensorPasses++;
    const pairs = this._pairs, events = this._events, size = this._size;
    const v = this._velocity, w = this._angular, pos = this._position;
    for (let i = 0; i < this._sensedCount; i++) {
      const p = this._sensed[i], a = pairs.a[p], b = pairs.b[p];
      // A mesh has no thickness of its own: then the other's size.
      const sa = size[a], sb = size[b], tolerance = (sa > 0 && sb > 0 ? Math.min(sa, sb) : sa + sb) * PIXEL;
      const now = sense(this, a, b, tolerance, SENSED) > 0 ? 1 : 0;
      if (now === pairs.touching[p]) continue;
      pairs.touching[p] = now;
      if (now === 0) {
        events.end(pairs.idA[p], pairs.idB[p]);
        continue;
      }
      const pt = SENSED.points, n = SENSED.normal, a3 = 3 * a, b3 = 3 * b;
      const rax = pt[0] - pos[a3], ray = pt[1] - pos[a3 + 1], raz = pt[2] - pos[a3 + 2];
      const rbx = pt[0] - pos[b3], rby = pt[1] - pos[b3 + 1], rbz = pt[2] - pos[b3 + 2];
      const dvx = v[b3] + w[b3 + 1] * rbz - w[b3 + 2] * rby - v[a3] - w[a3 + 1] * raz + w[a3 + 2] * ray;
      const dvy = v[b3 + 1] + w[b3 + 2] * rbx - w[b3] * rbz - v[a3 + 1] - w[a3 + 2] * rax + w[a3] * raz;
      const dvz = v[b3 + 2] + w[b3] * rby - w[b3 + 1] * rbx - v[a3 + 2] - w[a3] * ray + w[a3 + 1] * rax;
      const closing = -(dvx * n[0] + dvy * n[1] + dvz * n[2]);
      events.begin(pairs.idA[p], pairs.idB[p], pt[0], pt[1], pt[2], n[0], n[1], n[2], closing > 0 ? closing : 0);
    }
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

  _jointAlive(id) {
    return !this._destroyed && this._joints.slots.slotOf(id) >= 0;
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
    this._pose(s, POSE);
    this._place7(s, value, POSE_Q);
    this._moved(s);
  }

  _setRotation(caller, id, q) {
    const s = this._slotOf(caller, id);
    this._wake(s);
    this._pose(s, POSE);
    this._place7(s, POSE_P, q);
    this._moved(s);
  }

  /** body.position or body.rotation: the origin's, as given, a frozen copy. */
  _poseOf(caller, id, rotation) {
    const s = this._slotOf(caller, id);
    this._pose(s, POSE);
    return Object.freeze(rotation ? [POSE[3], POSE[4], POSE[5], POSE[6]] : [POSE[0], POSE[1], POSE[2]]);
  }

  /**
   * Slot s's origin and rotation as given, into out[0..6]: the step works
   * from its centre of mass p and principal axes q, and the origin is
   * P = p - q·offset, turned Q = q ⊗ frame⁻¹.
   */
  _pose(s, out) {
    const p = this._position, q = this._rotation, s3 = 3 * s, s4 = 4 * s;
    if (this._framed[s] === 0) {
      out[0] = p[s3]; out[1] = p[s3 + 1]; out[2] = p[s3 + 2];
      out[3] = q[s4]; out[4] = q[s4 + 1]; out[5] = q[s4 + 2]; out[6] = q[s4 + 3];
      return out;
    }
    const o = this._offset, f = this._frame;
    rotate(q, s4, o[s3], o[s3 + 1], o[s3 + 2], false, v3);
    out[0] = p[s3] - v3[0]; out[1] = p[s3 + 1] - v3[1]; out[2] = p[s3 + 2] - v3[2];
    INVERSE[0] = -f[s4]; INVERSE[1] = -f[s4 + 1]; INVERSE[2] = -f[s4 + 2]; INVERSE[3] = f[s4 + 3];
    quatMultiply(q, s4, INVERSE, 0, out, 3);
    return out;
  }

  /** Slot s placed with its origin at P and turned Q, as given: the inverse of _pose. */
  _place7(s, P, Q) {
    const p = this._position, q = this._rotation, s3 = 3 * s, s4 = 4 * s;
    if (this._framed[s] === 0) {
      p[s3] = P[0]; p[s3 + 1] = P[1]; p[s3 + 2] = P[2];
      q[s4] = Q[0]; q[s4 + 1] = Q[1]; q[s4 + 2] = Q[2]; q[s4 + 3] = Q[3];
      return;
    }
    quatMultiply(Q, 0, this._frame, s4, q, s4);
    const o = this._offset;
    rotate(q, s4, o[s3], o[s3 + 1], o[s3 + 2], false, v3);
    p[s3] = P[0] + v3[0]; p[s3 + 1] = P[1] + v3[1]; p[s3 + 2] = P[2] + v3[2];
  }

  /** body.centerOfMass: where the step works from, in the world, a frozen copy. */
  _centerOf(caller, id) {
    return this._vector(caller, id, this._position, 3);
  }

  /** After a teleport: drawn there, stamped, filed afresh, and what a static body held woken. */
  _moved(s) {
    this._place(s);
    this._ground[s] = -1;   // a character moved by hand stands on nothing it stood on
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
    if (name !== 'velocity') this._unlocked(s);
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
    if (body instanceof Joint) {
      if (body.world !== this) fail(caller, 'that joint belongs to another world');
      const j = this._joints.slots.slotOf(body.id);
      if (j >= 0) dropJoint(this, j);
      return;
    }
    if (!(body instanceof Body)) fail(caller, `handle must be a Body or a Joint, got ${show(body)}`);
    if (body.world !== this) fail(caller, 'that body belongs to another world');
    const s = this._slots.slotOf(body.id);
    if (s < 0) return;   // already gone: removing twice is harmless, as in Winding
    // Its joints go with it: a joint to nothing has no meaning.
    if (this._jointHead[s] !== 0) dropJointsOf(this, s);
    // A force waiting on this slot must not land on whichever body gets it next.
    this._forced[s] = 0;
    this._force.fill(0, s * 3, s * 3 + 3);
    this._torque.fill(0, s * 3, s * 3 + 3);
    this._geometry[s] = null;
    this._inner[s] = null;
    // Only where one was made: writing past the end would stretch the list to every slot.
    if (s < this._placed.length) this._placed[s] = null;
    if (this._framed[s] === 1) { this._framed[s] = 0; this._framedCount--; }
    if (this._sensor[s] === 1) { this._sensor[s] = 0; this._sensorCount--; }
    // The next body in the slot starts its count afresh, whatever its type.
    if (this._type[s] === CHARACTER) { this._type[s] = DYNAMIC; this._characterCount--; }
    this._epoch++;
    this._pairEpoch++;
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
    for (const g of [this._geometry, this._inner]) {
      const had = g.length;
      g.length = capacity;
      for (let s = had; s < capacity; s++) g[s] = null;
    }
    this._broadphase.resize(capacity);
  }

  /** A slot's full description, every size included, in option form. */
  _spec(s) {
    const shape = SHAPES[this._shape[s]];
    const d = this._dims;
    const s3 = s * 3, pose = this._pose(s, POSE);
    return {
      type: TYPES[this._type[s]],
      shape,
      geometry: this._geometry[s],
      size: shape === 'box' ? [d[s3], d[s3 + 1], d[s3 + 2]] : [...DEFAULTS.size],
      radius: shape === 'box' ? DEFAULTS.radius : d[s3],
      height: shape === 'capsule' ? d[s3 + 1] : DEFAULTS.height,
      density: this._density[s],
      mass: this._mass[s],
      massGiven: this._massGiven[s] === 1,
      inertia: [this._inertia[s3], this._inertia[s3 + 1], this._inertia[s3 + 2]],
      inertiaGiven: this._inertiaGiven[s] === 1,
      centerOfMass: this._comGiven[s] === 1 ? [this._com[s3], this._com[s3 + 1], this._com[s3 + 2]] : null,
      friction: this._friction[s],
      restitution: this._restitution[s],
      damping: this._damping[s],
      angularDamping: this._angularDamping[s],
      gravityScale: this._gravityScale[s],
      layers: this._layers[s],
      mask: this._mask[s],
      sensor: this._sensor[s] === 1,
      lockRotation: [(this._lock[s] & 1) !== 0, (this._lock[s] & 2) !== 0, (this._lock[s] & 4) !== 0],
      maxSlope: this._maxSlope[s],
      stepHeight: this._stepHeight[s],
      stepGiven: this._stepGiven[s] === 1,
      position: [pose[0], pose[1], pose[2]],
      rotation: [pose[3], pose[4], pose[5], pose[6]],
      velocity: [this._velocity[s3], this._velocity[s3 + 1], this._velocity[s3 + 2]],
      angularVelocity: [this._angular[s3], this._angular[s3 + 1], this._angular[s3 + 2]],
    };
  }

  /** Writes a checked spec into the columns. `placed`: the transform was set by hand. */
  _write(s, spec, placed) {
    const s3 = s * 3;
    this._characterCount += (spec.type === 'character' ? 1 : 0) - (this._type[s] === CHARACTER ? 1 : 0);
    this._type[s] = TYPES.indexOf(spec.type);
    const shape = SHAPES.indexOf(spec.shape);
    this._shape[s] = shape;
    this._geometry[s] = spec.geometry;
    const d = this._dims;
    if (shape === BOX) d.set(spec.size, s3);
    else if (shape === HULL || shape === COMPOUND || shape >= MESH) d.fill(0, s3, s3 + 3);
    else { d[s3] = spec.radius; d[s3 + 1] = shape === CAPSULE ? spec.height : 0; d[s3 + 2] = 0; }

    // Mass follows the shape unless it was given (rule 36: no
    // updateMassProperties()). Given a mass, the density is what makes it.
    this._massGiven[s] = spec.massGiven ? 1 : 0;
    this._inertiaGiven[s] = spec.inertiaGiven ? 1 : 0;
    const solid = shape === HULL || shape === COMPOUND ? solidMass(spec) : null;
    let center = ZERO, turn = IDENTITY;
    if (solid !== null) {
      this._mass[s] = solid.mass;
      this._density[s] = solid.density;
      center = solid.center;
      // Its inertia about its principal axes, unless given, about its own.
      if (spec.inertiaGiven) this._inertia.set(spec.inertia, s3);
      else {
        const axes = principal(solid.tensor);
        this._inertia.set(axes.moments, s3);
        turn = axes.rotation;
      }
    } else {
      const v = volume(spec);
      if (spec.massGiven) {
        this._mass[s] = spec.mass;
        this._density[s] = spec.mass / v;
      } else {
        this._density[s] = spec.density;
        this._mass[s] = spec.density * v;
      }
      // Inertia follows the shape and mass in the same way, unless it was given.
      if (spec.inertiaGiven) this._inertia.set(spec.inertia, s3);
      else this._inertia.set(shapeInertia(spec, this._mass[s], v3), s3);
    }
    // A centre of mass given moves where it is; the inertia stays the shape's,
    // about its own centre of mass, unless that is given too.
    this._comGiven[s] = spec.centerOfMass !== null ? 1 : 0;
    if (spec.centerOfMass !== null) center = spec.centerOfMass;
    this._com.set(center, s3);
    // The frame the step works in: centred on the centre of mass, turned to
    // the principal axes. The offset is the centre of mass along those axes.
    const R = matrixOf(turn), framed = center[0] !== 0 || center[1] !== 0 || center[2] !== 0 || turn !== IDENTITY;
    this._frame.set(turn, s * 4);
    for (let k = 0; k < 3; k++) this._offset[s3 + k] = R[k] * center[0] + R[3 + k] * center[1] + R[6 + k] * center[2];
    this._framedCount += (framed ? 1 : 0) - this._framed[s];
    this._framed[s] = framed ? 1 : 0;
    this._inner[s] = shape === HULL ? spec.geometry.hull.moved(center, transposed(R))
      : shape === COMPOUND ? new Parts(spec.geometry.parts.map((part) => inner(part, center, turn)))
        : shape <= CAPSULE && framed ? new Parts([inner({ ...spec, position: ZERO, rotation: IDENTITY }, center, turn)]) : null;
    // Level geometry has none, and is never dynamic: nothing turns it.
    for (let k = 0; k < 3; k++) this._invInertia[s3 + k] = this._inertia[s3 + k] > 0 ? 1 / this._inertia[s3 + k] : 0;
    // What contacts use: how much a push moves it (nothing, unless it is
    // dynamic), its thinnest size, which its tolerances scale with, and how
    // far its surface reaches from its centre, which bounds how far a spin
    // can carry any point of it.
    this._invMass[s] = spec.type === 'dynamic' ? 1 / this._mass[s] : 0;
    // The solver works out awake bodies' world inverse inertia each step; a
    // static one's must stay zero, even if it was dynamic before.
    if (spec.type === 'static') this._invI.fill(0, s * 6, s * 6 + 6);
    const g = this._inner[s];
    if (shape === HULL) {
      this._size[s] = g.size;
      this._reach[s] = g.reach;
    } else if (shape === COMPOUND) {
      // The thinnest of its parts, which is the thinnest thing of it that can touch.
      let size = Infinity, reach = 0;
      for (const part of g.list) {
        const o = part.position;
        size = Math.min(size, part.size);
        reach = Math.max(reach, Math.sqrt(o[0] * o[0] + o[1] * o[1] + o[2] * o[2]) + part.reach);
      }
      this._size[s] = size;
      this._reach[s] = reach;
    } else if (shape === BOX) {
      this._size[s] = Math.min(spec.size[0], spec.size[1], spec.size[2]);
      this._reach[s] = 0.5 * Math.sqrt(spec.size[0] * spec.size[0] + spec.size[1] * spec.size[1] + spec.size[2] * spec.size[2]);
    } else if (shape >= MESH) {
      // A surface, with no thickness: its contacts take the other body's tolerance (contact.js).
      this._size[s] = 0;
      this._reach[s] = spec.geometry.reach;
    } else {
      this._size[s] = 2 * spec.radius;
      this._reach[s] = shape === CAPSULE ? spec.height * 0.5 : spec.radius;
    }
    // A box, ball or capsule given a centre of mass reaches that much farther from it.
    if (shape <= CAPSULE && framed) this._reach[s] += Math.sqrt(center[0] * center[0] + center[1] * center[1] + center[2] * center[2]);
    this._friction[s] = spec.friction;
    this._restitution[s] = spec.restitution;
    this._damping[s] = spec.damping;
    this._angularDamping[s] = spec.angularDamping;
    this._gravityScale[s] = spec.gravityScale;
    this._layers[s] = spec.layers;
    this._mask[s] = spec.mask;
    const sensor = spec.sensor ? 1 : 0;
    this._sensorCount += sensor - this._sensor[s];
    this._sensor[s] = sensor;
    this._place7(s, spec.position, spec.rotation);
    this._velocity.set(spec.velocity, s3);
    this._angular.set(spec.angularVelocity, s3);
    // Never equal to any spin, so its first tumble takes its energy as it is.
    this._tumbled.fill(NaN, s3, s3 + 3);
    this._lock[s] = (spec.lockRotation[0] ? 1 : 0) | (spec.lockRotation[1] ? 2 : 0) | (spec.lockRotation[2] ? 4 : 0);
    this._unlocked(s);
    this._maxSlope[s] = spec.maxSlope;
    this._slopeCos[s] = cos(spec.maxSlope);
    this._stepGiven[s] = spec.stepGiven ? 1 : 0;
    // Unless given, a ninth of its height: building codes cap a stair's rise
    // near 0.2 m, a ninth of a 1.8 m person, and so it scales with its size.
    this._stepHeight[s] = spec.stepGiven ? spec.stepHeight : tallness(spec) / 9;
    if (placed) {
      this._place(s);
      this._ground[s] = -1;
    }
  }

  /** Slot s's spin about any axis it is locked about, which stays zero: setting it is kept to the free axes. */
  _unlocked(s) {
    const lock = this._lock[s], s3 = 3 * s;
    if (lock === 0) return;
    for (let k = 0; k < 3; k++) if (lock >> k & 1) this._angular[s3 + k] = 0;
  }

  /** Copies a slot's position and rotation out to `transforms`, and to `previous`, for a teleport. */
  _place(s) {
    const t7 = s * 7, s3 = s * 3;
    const tr = this._transforms;
    this._pose(s, POSE);
    // Copied one by one: subarray() would make a view for each, and a
    // platform moved by hand calls this every step. Measured: setting 1,000
    // kinematic bodies' position and rotation 9.5 to 10.8% faster (milestone 16).
    const previous = this._previous;
    for (let k = 0; k < 7; k++) { tr[t7 + k] = POSE[k]; previous[t7 + k] = tr[t7 + k]; }
    // Where the solver measures this step's movement from, for a body that
    // won't be moving in it (static, or asleep) until something wakes it.
    const p = this._position, st = this._start;
    st[s3] = p[s3]; st[s3 + 1] = p[s3 + 1]; st[s3 + 2] = p[s3 + 2];
  }
}

/**
 * Joins a's island and b's, if both are dynamic. Static and kinematic bodies
 * join nothing, or everything on one floor would be one island. The lower
 * slot becomes the root, so islands depend only on the inputs.
 */
function join(parent, type, a, b) {
  if (type[a] !== DYNAMIC || type[b] !== DYNAMIC) return;
  const ra = root(parent, a), rb = root(parent, b);
  if (ra < rb) parent[rb] = ra;
  else if (rb < ra) parent[ra] = rb;
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

  const shape = has('shape') ? oneOf(caller, 'shape', given.shape, SHAPES) : base.shape;
  const level = shape === 'mesh' || shape === 'heightfield';
  // addBody passes DEFAULTS itself, so a mesh or heightfield being made with
  // no type is static: what level geometry almost always is.
  const type = has('type') ? oneOf(caller, 'type', given.type, TYPES) : level && base === DEFAULTS ? 'static' : base.type;
  for (const k of SIZES) {
    if (has(k) && !SHAPE_SIZES[shape].includes(k)) fail(caller, `a ${shape} has no ${k}`);
  }
  if (level) {
    // Its triangles enclose nothing, so it has no mass to fall with or turn,
    // nor anything to walk with.
    if (type === 'dynamic' || type === 'character') fail(caller, `type must be 'static' or 'kinematic' for a ${shape}, got '${type}'`);
    for (const k of ['mass', 'density', 'inertia', 'centerOfMass']) if (has(k)) fail(caller, `a ${shape} has no ${k}`);
  }
  // A character finds its way by casting its own shape, and stands on its
  // lowest point: a box, ball or capsule's, found in closed form.
  if ((shape === 'hull' || shape === 'compound') && type === 'character') {
    fail(caller, `type must be 'dynamic', 'static' or 'kinematic' for a ${shape}, got 'character'`);
  }
  // A new shape starts from its own defaults; sizes of the old one don't carry over.
  const fresh = shape !== base.shape;
  let geometry = fresh ? null : base.geometry;
  if (shape === 'mesh' && (fresh || has('vertices') || has('indices'))) {
    const vertices = has('vertices') ? vertexList(caller, given.vertices) : fresh ? DEFAULTS.vertices : geometry.vertices;
    const indices = has('indices') ? given.indices : fresh ? DEFAULTS.indices : geometry.indices;
    triangleList(caller, indices, vertices.length / 3);
    geometry = new Mesh(vertices, indices);
  } else if (shape === 'heightfield' && (fresh || has('heights') || has('columns') || has('spacing'))) {
    const columns = has('columns') ? wholeFrom(caller, 'columns', given.columns, 2) : fresh ? DEFAULTS.columns : geometry.columns;
    const spacing = has('spacing') ? positive(caller, 'spacing', given.spacing) : fresh ? DEFAULTS.spacing : geometry.spacing;
    const heights = has('heights') ? given.heights : fresh ? DEFAULTS.heights : geometry.heights;
    grid(caller, heights, columns);
    geometry = new Heightfield(heights, columns, spacing);
  } else if (shape === 'hull' && (fresh || has('vertices'))) {
    geometry = { hull: hullOf(caller, 'vertices', has('vertices') ? vertexList(caller, given.vertices) : CUBE_CORNERS) };
  } else if (shape === 'compound' && (fresh || has('parts'))) {
    geometry = { parts: has('parts') ? partList(caller, given.parts) : partList(caller, [{}]) };
  }
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
  // null again: the centre of mass follows the shape.
  let centerOfMass = base.centerOfMass;
  if (given.centerOfMass === null) centerOfMass = null;
  else if (has('centerOfMass')) centerOfMass = vec3(caller, 'centerOfMass', given.centerOfMass);
  // A character is never turned by anything, and casts its shape from its centre.
  if (type === 'character' && centerOfMass !== null) fail(caller, 'a character body has no centerOfMass');

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
  const sensor = has('sensor') ? flag(caller, 'sensor', given.sensor) : base.sensor;
  let lockRotation = base.lockRotation;
  if (has('lockRotation')) {
    const l = given.lockRotation;
    if (!(l !== null && typeof l === 'object' && l.length === 3 && [0, 1, 2].every((k) => typeof l[k] === 'boolean'))) {
      fail(caller, `lockRotation must be 3 true or false values, got ${show(l)}`);
    }
    lockRotation = [l[0], l[1], l[2]];
  }
  if (type !== 'character') for (const k of WALKING) if (has(k)) fail(caller, `a ${type} body has no ${k}`);
  let maxSlope = base.maxSlope;
  if (has('maxSlope')) {
    const a = given.maxSlope;
    if (typeof a !== 'number' || !(a >= 0 && a <= Math.PI / 2)) fail(caller, `maxSlope must be an angle from 0 to π/2, in radians, got ${show(a)}`);
    maxSlope = a;
  }
  // null is a value here, as for inertia: the step height follows the size again.
  let stepHeight = base.stepHeight, stepGiven = base.stepGiven;
  if (given.stepHeight === null) stepGiven = false;
  else if (has('stepHeight')) {
    const h = given.stepHeight;
    if (typeof h !== 'number' || !(h >= 0) || h === Infinity) fail(caller, `stepHeight must be 0 or more, or null to follow the size, got ${show(h)}`);
    stepHeight = h;
    stepGiven = true;
  }

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
    type, shape, size, radius, height, geometry, density, mass, massGiven, inertia, inertiaGiven, centerOfMass,
    friction, restitution, damping, angularDamping, gravityScale, layers, mask, sensor,
    lockRotation, maxSlope, stepHeight, stepGiven, position, rotation, velocity, angularVelocity,
  };
}

/** The hull of a list of points, or a throw naming `name` if they don't span a solid. */
function hullOf(caller, name, vertices) {
  if (lastHull !== null && sameNumbers(lastVertices, vertices)) return lastHull;
  const hull = build(vertices);
  const n = vertices.length / 3;
  if (hull === null) fail(caller, `${name} must span a solid (4 points not in one plane), got ${n < 4 ? `${n} points` : `${n} points in one plane`}`);
  lastVertices = Float64Array.from(vertices);
  lastHull = hull;
  return hull;
}

// The last hull built, and a copy of the numbers it was built from: a game
// adds the same rock or crate many times, and the same numbers build the same
// hull, which nothing changes once built, so it is shared. Kept by value, so
// a caller changing its array afterwards changes nothing. Measured: see
// ROADMAP.md, milestone 16.
let lastVertices = null, lastHull = null;

/** Whether a and b hold the same numbers, -0 and 0 told apart. */
function sameNumbers(a, b) {
  if (a.length !== b.length) return false;
  for (let i = 0; i < a.length; i++) if (!Object.is(a[i], b[i])) return false;
  return true;
}

/**
 * A compound's parts, checked: a list of one or more, each a plain object
 * of PART_OPTIONS, named by where it is in the list (`parts[1].radius`).
 */
function partList(caller, value) {
  if (!Array.isArray(value) || value.length === 0) fail(caller, `parts must be a list of one or more parts, got ${show(value)}`);
  return value.map((given, i) => {
    const name = `parts[${i}]`;
    const o = options(caller, given, name);
    known(`${caller}: ${name}`, o, PART_OPTIONS);
    const has = (k) => o[k] !== undefined;
    const shape = has('shape') ? oneOf(caller, `${name}.shape`, o.shape, PART_SHAPES) : 'box';
    for (const k of ['size', 'radius', 'height', 'vertices']) {
      if (has(k) && !SHAPE_SIZES[shape].includes(k)) fail(caller, `${name}: a ${shape} has no ${k}`);
    }
    const part = {
      shape,
      size: has('size') ? positive3(caller, `${name}.size`, o.size) : [...DEFAULTS.size],
      radius: has('radius') ? positive(caller, `${name}.radius`, o.radius) : DEFAULTS.radius,
      height: has('height') ? positive(caller, `${name}.height`, o.height) : DEFAULTS.height,
      hull: shape === 'hull' ? hullOf(caller, `${name}.vertices`, has('vertices') ? vertexList(caller, o.vertices, `${name}.vertices`) : CUBE_CORNERS) : null,
      position: has('position') ? vec3(caller, `${name}.position`, o.position) : [0, 0, 0],
      rotation: has('rotation') ? quat(caller, `${name}.rotation`, o.rotation) : [0, 0, 0, 1],
      density: has('density') ? positive(caller, `${name}.density`, o.density) : null,
    };
    if (shape === 'capsule' && part.height < 2 * part.radius) {
      fail(caller, `${name}.height must be at least twice radius (${2 * part.radius}), got ${part.height}`);
    }
    return part;
  });
}

/** A part as bodyOf gives it back: the options it was made from. */
function partOf(part) {
  const out = { shape: part.shape };
  if (part.shape === 'hull') out.vertices = Array.from(part.hull.vertices);
  else for (const k of SHAPE_SIZES[part.shape]) out[k] = k === 'size' ? [...part.size] : part[k];
  out.position = [...part.position];
  out.rotation = [...part.rotation];
  if (part.density !== null) out.density = part.density;
  return out;
}

/**
 * A hull's or compound's mass, centre of mass (in its own frame) and inertia
 * tensor about it (shapes.js). A compound's parts are summed: their masses,
 * their centres weighted by mass, and their tensors, each turned to the
 * body's frame and moved to the shared centre by the parallel-axis theorem.
 * Given a mass, every part is scaled to it, and the density is the body's.
 */
function solidMass(spec) {
  if (spec.shape === 'hull') {
    const h = hullMass(spec.geometry.hull);
    const mass = spec.massGiven ? spec.mass : spec.density * h.volume, density = mass / h.volume;
    return { mass, density, center: h.center, tensor: h.tensor.map((x) => x * density) };
  }
  const parts = spec.geometry.parts.map((part) => {
    const R = matrixOf(part.rotation);
    let volume_, center = [0, 0, 0], tensor;
    if (part.shape === 'hull') {
      const h = hullMass(part.hull);
      volume_ = h.volume;
      center = [0, 1, 2].map((k) => R[3 * k] * h.center[0] + R[3 * k + 1] * h.center[1] + R[3 * k + 2] * h.center[2]);
      tensor = turnedTensor(h.tensor, R);
    } else {
      volume_ = volume(part);
      const I = shapeInertia(part, volume_, [0, 0, 0]);
      tensor = turnedTensor([I[0], 0, 0, I[1], 0, I[2]], R);
    }
    const rho = part.density !== null ? part.density : spec.density, m = rho * volume_;
    return { m, center: center.map((x, k) => x + part.position[k]), tensor: tensor.map((x) => x * rho) };
  });
  let mass = 0;
  const center = [0, 0, 0];
  for (const p of parts) {
    mass += p.m;
    for (let k = 0; k < 3; k++) center[k] += p.m * p.center[k];
  }
  for (let k = 0; k < 3; k++) center[k] /= mass;
  let tensor = [0, 0, 0, 0, 0, 0];
  for (const p of parts) {
    const t = shifted(p.tensor, p.m, [0, 1, 2].map((k) => p.center[k] - center[k]));
    tensor = tensor.map((x, k) => x + t[k]);
  }
  if (spec.massGiven) {
    const scale = spec.mass / mass;
    tensor = tensor.map((x) => x * scale);
    mass = spec.mass;
  }
  return { mass, density: spec.density, center, tensor };
}

/**
 * Part (a part's options, or a box, ball or capsule's own) in the frame the
 * step works in: its position from the centre of mass along the principal
 * axes, and its turn from them. `turn` is those axes as a quaternion.
 */
function inner(part, center, turn) {
  const R = matrixOf(turn), o = part.position, q = part.rotation;
  const x = o[0] - center[0], y = o[1] - center[1], z = o[2] - center[2];
  const position = [0, 1, 2].map((k) => R[k] * x + R[3 + k] * y + R[6 + k] * z);
  const rotation = [0, 0, 0, 0];
  quatMultiply([-turn[0], -turn[1], -turn[2], turn[3]], 0, q, 0, rotation, 0);
  const kind = part.shape === 'box' ? BOX_KIND : part.shape === 'sphere' ? SPHERE_KIND : part.shape === 'capsule' ? CAPSULE_KIND : HULL_KIND;
  const half = part.shape === 'box' ? part.size.map((x) => x * 0.5)
    : part.shape === 'capsule' ? [part.radius, (part.height - 2 * part.radius) * 0.5, 0] : [part.radius, 0, 0];
  return new Part(kind, half, part.shape === 'hull' ? part.hull : null, position, rotation);
}

/** A rotation matrix's transpose (9 numbers, row by row): its inverse. */
function transposed(R) {
  return [R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]];
}

const ZERO = Object.freeze([0, 0, 0]), IDENTITY = Object.freeze([0, 0, 0, 1]);
/** _pose's and _place7's scratch; neither is re-entered. */
const POSE = new Float64Array(7), INVERSE = new Float64Array(4);
/** POSE's position and rotation, as views made once: setPosition and setRotation are called every step for a moved platform. */
const POSE_P = POSE.subarray(0, 3), POSE_Q = POSE.subarray(3, 7);
/** _catch's: the core it sweeps, what the sweep found, and a rotation; a step isn't re-entered. */
const CORE = new Shape(), SWEPT = new Float64Array(7), TURNED = new Float64Array(4), SWEPT_SLOT = new Int32Array(1);

/** How tall a shape stands along its own y: a capsule's height, a box's, a sphere's diameter. */
function tallness(spec) {
  return spec.shape === 'box' ? spec.size[1] : spec.shape === 'capsule' ? spec.height : 2 * spec.radius;
}

/** _turn's scratch: the body's axes, and its locked inverse inertia; it isn't re-entered. */
const TURN = new Shape(), M6 = new Float64Array(6);

/** The material options, in the order bodyOf lists them. */
const MATERIAL = ['friction', 'restitution', 'damping', 'angularDamping', 'gravityScale', 'layers', 'mask', 'sensor'];

/** Where _senseAll's pair met; it isn't re-entered. */
const SENSED = new Manifold();

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
  return wholeFrom(caller, name, value, 1);
}

/** A whole number, `least` or more. */
function wholeFrom(caller, name, value, least) {
  if (!Number.isInteger(value) || value < least) fail(caller, `${name} must be a whole number, ${least} or more, got ${show(value)}`);
  return value;
}

/** An array or typed array, as a mesh's lists are given. */
const list = (value) => Array.isArray(value) || (ArrayBuffer.isView(value) && !(value instanceof DataView));

/**
 * Where a long list goes wrong, for its message: the whole of it would be
 * thousands of numbers, so the first bad entry and where it is.
 */
const entry = (value, i) => `${show(value[i])} at index ${i}`;

/** A mesh's vertices: finite numbers, three to a vertex. */
function vertexList(caller, value, name = 'vertices') {
  const rule = `${name} must be finite numbers, three to a vertex`;
  if (!list(value)) fail(caller, `${rule}, got ${show(value)}`);
  if (value.length % 3 !== 0) fail(caller, `${rule}, got ${value.length} numbers`);
  for (let i = 0; i < value.length; i++) {
    if (typeof value[i] !== 'number' || !Number.isFinite(value[i])) fail(caller, `${rule}, got ${entry(value, i)}`);
  }
  return value;
}

/** A mesh's indices: whole numbers below the vertex count, three to a triangle. */
function triangleList(caller, value, count) {
  const rule = `indices must be whole numbers below the vertex count (${count}), three to a triangle`;
  if (!list(value)) fail(caller, `${rule}, got ${show(value)}`);
  if (value.length % 3 !== 0) fail(caller, `${rule}, got ${value.length} numbers`);
  for (let i = 0; i < value.length; i++) {
    const k = value[i];
    if (!Number.isInteger(k) || k < 0 || k >= count) fail(caller, `${rule}, got ${entry(value, i)}`);
  }
}

/** A heightfield's heights: finite numbers, whole rows of `columns`, at least two rows. */
function grid(caller, value, columns) {
  const rule = `heights must be finite numbers, rows of columns (${columns}), 2 rows or more`;
  if (!list(value)) fail(caller, `${rule}, got ${show(value)}`);
  if (value.length % columns !== 0 || value.length < 2 * columns) fail(caller, `${rule}, got ${value.length} numbers`);
  for (let i = 0; i < value.length; i++) {
    if (typeof value[i] !== 'number' || !Number.isFinite(value[i])) fail(caller, `${rule}, got ${entry(value, i)}`);
  }
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
 * to rounding by construction, and the energy can never run away, since
 * with |L| fixed it is at most |L|² / 2 I_min. With L fixed the energy
 * depends only on how the body is turned, and the half-step turn is a hair
 * off: near the middle axis that only ever loses energy, but spun mostly
 * about the longest or shortest axis it drifted up, 2e-4 to 4e-4 in 120 s.
 * So a body tumbling freely that ends a substep with more energy than it had
 * when it was last spun (by anything but tumbling) is turned back onto that
 * energy (untwist()), and energy can only fall. Against each substep's own
 * start instead, it caught every rise of the turn's error that a fall had
 * just made room for, and ratcheted down: 14% lost in 120 s at 50 rad/s. Measured over
 * 120 s of 60 Hz steps, each of the 8 substeps: under 0.001% lost flipping at
 * 5 rad/s, 0.02% at 20 rad/s, 0.6% at 50 rad/s. (Taken whole, with no
 * substeps, the same cost 0.04%, 6% and 11%: what a step's size costs.) It is
 * always on, costs six quaternion rotations a lopsided spinning body a
 * substep, and has nothing to tune. test/motion.test.js holds it to these.
 *
 * Reads the rotation at q[s4..] and ω at w[s3..]; writes both.
 */
function tumble(q, s4, I, inv, s3, w, dt, left, ceiling, s) {
  // L = R I Rᵀ ω.
  rotate(q, s4, w[s3], w[s3 + 1], w[s3 + 2], true, v3);
  rotate(q, s4, v3[0] * I[s3], v3[1] * I[s3 + 1], v3[2] * I[s3 + 2], false, v3);
  const lx = v3[0], ly = v3[1], lz = v3[2];
  // Spun since its last tumble, by a contact, a force, damping or a setter:
  // twice the energy it has now, ω·L, is the most it may have from here.
  if (w[s3] !== left[s3] || w[s3 + 1] !== left[s3 + 1] || w[s3 + 2] !== left[s3 + 2]) {
    ceiling[s] = w[s3] * lx + w[s3 + 1] * ly + w[s3 + 2] * lz;
  }
  const most = ceiling[s];
  // Half a step on with the starting ω: q + dt/4 (ω, 0) q.
  advance(q, s4, w[s3], w[s3 + 1], w[s3 + 2], dt * 0.25, q4, 0);
  // The angular velocity L gives there, R I⁻¹ Rᵀ L, turns the whole step.
  angular(q4, 0, inv, s3, lx, ly, lz);
  advance(q, s4, v3[0], v3[1], v3[2], dt * 0.5, q, s4);
  // And the angular velocity L gives at the end is the body's new ω.
  angular(q, s4, inv, s3, lx, ly, lz);
  if (v3[0] * lx + v3[1] * ly + v3[2] * lz > most * (1 + ROUNDED)) {
    untwist(q, s4, inv, s3, lx, ly, lz, most);
    angular(q, s4, inv, s3, lx, ly, lz);
  }
  w[s3] = v3[0]; w[s3 + 1] = v3[1]; w[s3 + 2] = v3[2];
  left[s3] = v3[0]; left[s3 + 1] = v3[1]; left[s3 + 2] = v3[2];
}

/**
 * Turns the body at q[s4..] a little, so that L, unchanged, gives it twice
 * the energy `most` again: tumble()'s turn left it slightly off the curve
 * its momentum traces in the body (on the sphere of L's length, where the
 * energy is constant), and on the side where the energy is higher. In the
 * body's frame, L is moved along that sphere against the energy's gradient
 * (Newton's method, to where the energy is `most`), and the body is turned
 * by the small rotation that carries L's old place in it to the new one, so
 * L in the world is exactly what it was. The turn's error is about a part in
 * a million of the energy a substep, so one Newton step leaves it to about a
 * part in 10^12 and a second to rounding: two, always.
 */
function untwist(q, s4, inv, s3, lx, ly, lz, most) {
  rotate(q, s4, lx, ly, lz, true, B);
  const bx = B[0], by = B[1], bz = B[2], ll = bx * bx + by * by + bz * bz;
  let x = bx, y = by, z = bz;
  for (let round = 0; round < 2; round++) {
    // The energy's gradient, I⁻¹ b, less its part along b: the way along the sphere.
    const gx = x * inv[s3], gy = y * inv[s3 + 1], gz = z * inv[s3 + 2];
    const over = x * gx + y * gy + z * gz - most;
    if (!(over > 0)) break;
    const k = (gx * x + gy * y + gz * z) / ll, tx = gx - k * x, ty = gy - k * y, tz = gz - k * z;
    const tt = tx * tx + ty * ty + tz * tz;
    // Along a principal axis the energy doesn't change to first order: nothing to move.
    if (!(tt > 0)) break;
    // Twice the energy falls by 2 g_t·d per d moved: half `over` along t, over |t|².
    const a = 0.5 * over / tt;
    x -= a * tx; y -= a * ty; z -= a * tz;
    const back = Math.sqrt(ll / (x * x + y * y + z * z));
    x *= back; y *= back; z *= back;
  }
  // The rotation carrying the new place to the old, (n × o, |n||o| + n·o), on the body's side of q.
  const cx = y * bz - z * by, cy = z * bx - x * bz, cz = x * by - y * bx;
  const cw = ll + x * bx + y * by + z * bz;
  const l = Math.sqrt(cx * cx + cy * cy + cz * cz + cw * cw);
  if (!(l > 0)) return;
  T4[0] = cx / l; T4[1] = cy / l; T4[2] = cz / l; T4[3] = cw / l;
  quatMultiply(q, s4, T4, 0, Q4, 0);
  const n = 1 / Math.sqrt(Q4[0] * Q4[0] + Q4[1] * Q4[1] + Q4[2] * Q4[2] + Q4[3] * Q4[3]);
  q[s4] = Q4[0] * n; q[s4 + 1] = Q4[1] * n; q[s4 + 2] = Q4[2] * n; q[s4 + 3] = Q4[3] * n;
}

/**
 * A rise in energy smaller than this part of it is rounding, not the turn:
 * 2^-44, 256 times a double's rounding (2^-52), where ω·L's own is a few.
 * The turn's real drift is about 1e-9 of the energy a substep.
 */
const ROUNDED = 1 / 17592186044416;   // 2^-44

const q4 = new Float64Array(4), B = new Float64Array(3), T4 = new Float64Array(4), Q4 = new Float64Array(4);

/** The shape a query is asked about, filled by _queryShape; queries aren't re-entered. */
const QUERY = new Shape();

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
