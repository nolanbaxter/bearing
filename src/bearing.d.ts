// Types for Bearing, written by hand beside the JavaScript rather than built
// from it. docs/API.md is the reference; these follow it, and test/types.ts
// holds them to its examples.

/** Three numbers: a plain array, or a typed array of length 3. */
export type Vec3 = readonly [number, number, number] | Float32Array | Float64Array;
/** A quaternion [x, y, z, w]: a plain array, or a typed array of length 4. */
export type Quat = readonly [number, number, number, number] | Float32Array | Float64Array;

/**
 * 'dynamic' is moved by gravity; 'static' never moves unless moved; 'kinematic' moves at its
 * velocity; 'character' moves at its velocity as far as the world lets it, walking.
 */
export type BodyType = 'dynamic' | 'static' | 'kinematic' | 'character';
export type Shape = 'box' | 'sphere' | 'capsule' | 'hull' | 'compound' | 'mesh' | 'heightfield';
/** A list of numbers: a plain array or any typed array. */
export type Numbers = ArrayLike<number>;

/** The sizes of shapes other than this one: refused, here and at run time. */
interface NoLevelSizes {
  vertices?: never;
  indices?: never;
  heights?: never;
  columns?: never;
  spacing?: never;
  parts?: never;
}

interface BodyCommon {
  /** Default 'dynamic'. */
  type?: BodyType;
  /** kg/m³, default 1000 (water). Give this or `mass`, not both. */
  density?: number;
  /** kg. Give this or `density`, not both. */
  mass?: number;
  /** Moments of inertia about the body's own axes, kg·m². Worked out from the shape and mass, about its principal axes, unless given; null goes back to that. */
  inertia?: Vec3 | null;
  /** Where its mass is centred, m, in its own frame. Worked out from the shape unless given; null goes back to that. Not for a character. */
  centerOfMass?: Vec3 | null;
  /** How much sideways push a contact resists per newton pressing it, default 0.5. 0 is ice. */
  friction?: number;
  /** Fraction of the speed it hits at that comes back as a bounce, 0 to 1, default 0. */
  restitution?: number;
  /** Velocity lost per second, 1/s, default 0. */
  damping?: number;
  /** Spin lost per second, 1/s, default 0. */
  angularDamping?: number;
  /** Multiplies the world's gravity for this body, default 1. */
  gravityScale?: number;
  /** Which of 32 layers the body is in, as bits, default 1. Negative 32-bit numbers (`~x`, `1 << 31`) are taken as the same bits. */
  layers?: number;
  /** Which layers it collides with, as bits, default 0xffffffff (all). */
  mask?: number;
  /** Reports what overlaps it and pushes nothing, default false. */
  sensor?: boolean;
  /** The world axes it never turns about, x, y and z, default [false, false, false]. */
  lockRotation?: readonly [boolean, boolean, boolean];
  /** A character's steepest walkable ground, radians from 0 to π/2, default π/4. Characters only. */
  maxSlope?: number;
  /** The tallest step a character walks up, m, default its height ÷ 9; null follows its size. Characters only. */
  stepHeight?: number | null;
  /** m, default [0, 0, 0]. */
  position?: Vec3;
  /** Quaternion [x, y, z, w], default [0, 0, 0, 1]. Normalized as it comes in. */
  rotation?: Quat;
  /** m/s, default [0, 0, 0]. Not for a static body. */
  velocity?: Vec3;
  /** rad/s about each world axis, default [0, 0, 0]. Not for a static body. */
  angularVelocity?: Vec3;
}

/** A box: the default shape. */
export interface BoxOptions extends BodyCommon, NoLevelSizes {
  shape?: 'box';
  /** Full width, height and depth in m, default [1, 1, 1]. */
  size?: Vec3;
  radius?: never;
  height?: never;
}

export interface SphereOptions extends BodyCommon, NoLevelSizes {
  shape: 'sphere';
  /** m, default 0.5. */
  radius?: number;
  size?: never;
  height?: never;
}

export interface CapsuleOptions extends BodyCommon, NoLevelSizes {
  shape: 'capsule';
  /** m, default 0.5. */
  radius?: number;
  /** Full height tip to tip along its own y, in m, default 2. At least twice radius. */
  height?: number;
  size?: never;
}

/** A convex hull of points: the smallest convex solid round them. Never a character. */
export interface HullOptions extends Omit<BodyCommon, 'type'> {
  shape: 'hull';
  /** Default 'dynamic'. */
  type?: 'dynamic' | 'static' | 'kinematic';
  /** Points, three numbers each, in m, in the body's own frame: at least 4, not all in one plane. Default a 1 m cube's corners. */
  vertices?: Numbers;
  size?: never;
  radius?: never;
  height?: never;
  indices?: never;
  heights?: never;
  columns?: never;
  spacing?: never;
  parts?: never;
}

interface PartCommon {
  /** m, from the body's origin, in its own frame. Default [0, 0, 0]. */
  position?: Vec3;
  /** Quaternion [x, y, z, w] against the body, default [0, 0, 0, 1]. */
  rotation?: Quat;
  /** kg/m³, default the body's. */
  density?: number;
}

/** One shape of a compound: a box, ball, capsule or hull, sized as a body of that shape is. */
export type PartOptions =
  | (PartCommon & { shape?: 'box'; size?: Vec3; radius?: never; height?: never; vertices?: never })
  | (PartCommon & { shape: 'sphere'; radius?: number; size?: never; height?: never; vertices?: never })
  | (PartCommon & { shape: 'capsule'; radius?: number; height?: number; size?: never; vertices?: never })
  | (PartCommon & { shape: 'hull'; vertices?: Numbers; size?: never; radius?: never; height?: never });

/** Several shapes as one rigid body. Never a character. */
export interface CompoundOptions extends Omit<BodyCommon, 'type'> {
  shape: 'compound';
  /** Default 'dynamic'. */
  type?: 'dynamic' | 'static' | 'kinematic';
  /** One or more. Default one 1 m box. */
  parts?: readonly PartOptions[];
  size?: never;
  radius?: never;
  height?: never;
  vertices?: never;
  indices?: never;
  heights?: never;
  columns?: never;
  spacing?: never;
}

/** Level geometry: static unless made kinematic, never dynamic, with no mass, density or inertia. */
interface LevelCommon extends Omit<BodyCommon, 'type' | 'density' | 'mass' | 'inertia' | 'centerOfMass' | 'maxSlope' | 'stepHeight'> {
  maxSlope?: never;
  stepHeight?: never;
  /** Default 'static'. */
  type?: 'static' | 'kinematic';
  density?: never;
  mass?: never;
  inertia?: never;
  centerOfMass?: never;
  size?: never;
  radius?: never;
  height?: never;
  parts?: never;
}

/** Triangles, for level geometry. See docs/API.md, "Meshes and heightfields". */
export interface MeshOptions extends LevelCommon {
  shape: 'mesh';
  /** Corners, three numbers each, in m, in the body's own frame. Default none. */
  vertices?: Numbers;
  /** Triangles, three corner numbers each, from 0. Default none. */
  indices?: Numbers;
  heights?: never;
  columns?: never;
  spacing?: never;
}

/** A grid of heights, centred on the body: columns along x, rows along z. */
export interface HeightfieldOptions extends LevelCommon {
  shape: 'heightfield';
  /** m, row by row, `columns` to a row, at least 2 rows. Default [0, 0, 0, 0]: a flat 1 m square. */
  heights?: Numbers;
  /** Heights to a row, a whole number from 2, default 2. */
  columns?: number;
  /** m between neighbouring heights, default 1. */
  spacing?: number;
  vertices?: never;
  indices?: never;
}

/** What world.addBody takes. A size option the shape doesn't have is refused, here and at run time. */
export type BodyOptions = BoxOptions | SphereOptions | CapsuleOptions | HullOptions | CompoundOptions | MeshOptions | HeightfieldOptions;

/** What world.setBody takes: any of addBody's options. Which sizes fit is checked against the body's shape when called. */
export interface BodyChanges extends BodyCommon {
  shape?: Shape;
  size?: Vec3;
  radius?: number;
  height?: number;
  vertices?: Numbers;
  indices?: Numbers;
  heights?: Numbers;
  columns?: number;
  spacing?: number;
  parts?: readonly PartOptions[];
}

interface BodyInfoCommon {
  type: BodyType;
  /** Present if the body was given a mass. */
  mass?: number;
  /** Present if the body was not given a mass. */
  density?: number;
  /** Present if the body was given an inertia. */
  inertia?: [number, number, number];
  /** Present if the body was given a centre of mass. */
  centerOfMass?: [number, number, number];
  friction: number;
  restitution: number;
  damping: number;
  angularDamping: number;
  gravityScale: number;
  /** Unsigned, 0 to 4294967295. */
  layers: number;
  /** Unsigned, 0 to 4294967295. */
  mask: number;
  sensor: boolean;
  lockRotation: [boolean, boolean, boolean];
  /** A character's. */
  maxSlope?: number;
  /** A character's, if it was given one. */
  stepHeight?: number;
  position: [number, number, number];
  rotation: [number, number, number, number];
  /** Absent for a static body. */
  velocity?: [number, number, number];
  /** Absent for a static body. */
  angularVelocity?: [number, number, number];
}

/** What world.bodyOf returns: a copy, in the form addBody takes. */
export type BodyInfo =
  | (BodyInfoCommon & { shape: 'box'; size: [number, number, number] })
  | (BodyInfoCommon & { shape: 'sphere'; radius: number })
  | (BodyInfoCommon & { shape: 'capsule'; radius: number; height: number })
  | (SolidInfoCommon & { shape: 'hull'; vertices: number[] })
  | (SolidInfoCommon & { shape: 'compound'; parts: PartInfo[] })
  | (LevelInfoCommon & { shape: 'mesh'; vertices: number[]; indices: number[] })
  | (LevelInfoCommon & { shape: 'heightfield'; heights: number[]; columns: number; spacing: number });

/** A mesh's or heightfield's: never dynamic, and no mass, density or inertia. */
type LevelInfoCommon = Omit<BodyInfoCommon, 'type' | 'mass' | 'density' | 'inertia' | 'centerOfMass' | 'maxSlope' | 'stepHeight'> & { type: 'static' | 'kinematic' };

/** A hull's or compound's: never a character. */
type SolidInfoCommon = Omit<BodyInfoCommon, 'type' | 'maxSlope' | 'stepHeight'> & { type: 'dynamic' | 'static' | 'kinematic' };

/** A compound's part as bodyOf gives it: the options it was made from. A hull's vertices are its corners. */
export type PartInfo = {
  position: [number, number, number];
  rotation: [number, number, number, number];
  /** Present if the part was given a density. */
  density?: number;
} & (
  | { shape: 'box'; size: [number, number, number] }
  | { shape: 'sphere'; radius: number }
  | { shape: 'capsule'; radius: number; height: number }
  | { shape: 'hull'; vertices: number[] }
);

/** What world.save() returns: plain data, its layout Bearing's own, named by `format`. */
export interface SavedWorld {
  format: number;
  objects: unknown[];
}

/** What world.debugLines draws. */
export interface DebugLineOptions {
  /** Every body's collision shape, default true. */
  shapes?: boolean;
  /** The last step's contact points and normals, default true. */
  contacts?: boolean;
  /** Each joint's anchors, default true. */
  joints?: boolean;
  /** The broadphase's boxes, default false. */
  boxes?: boolean;
}

/** What world.groundOf finds under a character. */
export interface Ground {
  body: Body;
  point: [number, number, number];
  /** Unit, up out of the ground: a tread's, even on its edge. */
  normal: [number, number, number];
}

/** Two bodies began touching in the last step: where, which way (from a to b), and how fast they met, m/s. */
export interface BeginEvent {
  type: 'begin';
  a: Body;
  b: Body;
  point: [number, number, number];
  normal: [number, number, number];
  speed: number;
}

/** Two bodies stopped touching in the last step. A removed body's handle reads as dead. */
export interface EndEvent {
  type: 'end';
  a: Body;
  b: Body;
}

export type ContactEvent = BeginEvent | EndEvent;

/** What raycast, raycastAll and shapeCast find. */
export interface Hit {
  /** The body met. A handle: safe to keep. */
  body: Body;
  /** Where on its surface, m. */
  point: [number, number, number];
  /** The surface's outward direction there, unit length, facing back at the ray or shape. */
  normal: [number, number, number];
  /** How far along, m. */
  distance: number;
}

export interface RayOptions {
  /** Ignore hits this far or further, m. Default Infinity. */
  maxDistance?: number;
  /** Which of the 32 layers to hit, as bits. Default 0xffffffff. */
  mask?: number;
  /** A body to leave out. */
  ignore?: Body | null;
}

export interface CastOptions extends RayOptions {
  /** The shape's rotation, [x, y, z, w]. Default [0, 0, 0, 1]. */
  rotation?: Quat;
}

export interface OverlapOptions {
  mask?: number;
  ignore?: Body | null;
  rotation?: Quat;
}

/** A shape to cast or test, with addBody's names and defaults: {} is a 1 m box. */
export type QueryShape =
  | { shape?: 'box'; size?: Vec3 }
  | { shape: 'sphere'; radius?: number }
  | { shape: 'capsule'; radius?: number; height?: number };

export interface GlbOptions {
  /** The name of one node: only it and the nodes under it are read, still placed as in the scene. */
  node?: string;
}

/**
 * Every triangle of a .glb file's scene, each node placed as the scene places
 * it, ready for addBody({ shape: 'mesh', ...readGlb(bytes) }). `bytes` is the
 * file itself, as fetch's arrayBuffer() or Node's readFile gives it.
 */
export function readGlb(bytes: ArrayBuffer | ArrayBufferView, options?: GlbOptions): { vertices: Float64Array; indices: Uint32Array };

export type JointType = 'fixed' | 'ball' | 'hinge' | 'slider' | 'distance' | 'rotation';

/** What every joint takes. */
interface JointBase {
  /** The body. Its angle, position and motor are measured against b. */
  a: Body;
  /** What a is held to; null, or left out, for a point in the world that never moves. */
  b?: Body | null;
  /** Whether a and b still collide with each other. Default false. */
  collide?: boolean;
}

/** What every joint that holds a point takes: all but a rotation joint. */
interface JointCommon extends JointBase {
  /** World space, m: where the joint is, where the bodies are now. Default midway between their centres (a distance joint's, a's centre). */
  anchor?: Vec3;
  /** Where it is on b, or in the world with no b, if not the same point: a distance joint's other end. */
  anchorB?: Vec3;
}

/** A motor's drive to a target, as a spring: hinges, sliders, ball and rotation joints. */
interface Drive {
  /** The most the motor may push: N·m, or N for a slider. 0, the default, is the motor off: a target with no motorForce does nothing. */
  motorForce?: number;
  /** Hz: how stiff the drive to target is. Default 0, as stiff as the solver goes. */
  frequency?: number;
  /** 1, the default, settles without overshooting; 0 has no damping of its own. */
  dampingRatio?: number;
  /** The most the joint resists a turning (sliding) against b while the motor is off: N·m, or N. Default 0, none. */
  friction?: number;
}

/** A ball or rotation joint's: a's turn against b, about b's own axes (the world's with no b), [x, y, z, w], any length; [0, 0, 0, 1] is as joined. null, the default, is none. */
interface TurnDrive extends Drive {
  target?: Quat | null;
}

interface Turning {
  /** The joint's axis, world space; needn't be unit length. Default [0, 1, 0]. */
  axis?: Vec3;
  /** The least a's angle or position may be; null goes back to none. */
  min?: number | null;
  /** The most; null goes back to none. */
  max?: number | null;
}

interface Driven extends Turning, Drive {
  /** rad/s about axis, or m/s along it, that the motor drives a at against b; with a target, how fast the target moves. */
  motorSpeed?: number;
  /** A hinge's angle, radians, -π to π, or a slider's position, m, measured as min and max are, that the motor drives a to. null, the default, is none: the motor drives a speed. */
  target?: number | null;
}

/** Welded: nothing free. */
export interface FixedJointOptions extends JointCommon { type: 'fixed' }
/** Any turn about its anchor; min and max limit its twist about axis, cone its swing. */
export interface BallJointOptions extends JointCommon, Turning, TurnDrive {
  type?: 'ball';
  /** How far axis may swing, radians, 0 to π. Default π, none. */
  cone?: number;
}
/** Turns about axis; min and max limit its angle, radians, -π to π. */
export interface HingeJointOptions extends JointCommon, Driven { type: 'hinge' }
/** Slides along axis; min and max limit its position, m. */
export interface SliderJointOptions extends JointCommon, Driven { type: 'slider' }
/** Keeps its anchors length apart, or between min and max; a spring with a frequency. */
export interface DistanceJointOptions extends JointCommon {
  type: 'distance';
  /** m. Default: how far apart the anchors are. */
  length?: number;
  /** m; default length, or 0 with a spring. null goes back to that. */
  min?: number | null;
  /** m; default length, or none with a spring. */
  max?: number | null;
  /** Hz: makes it a spring pulling to length. Default 0, none. */
  frequency?: number;
  /** 1, the default, settles without overshooting; 0 bounces for ever. */
  dampingRatio?: number;
}

/** Holds how a is turned against b (or the world), leaving where it is free; with a motor and a target, drives it there. */
export interface RotationJointOptions extends JointBase, TurnDrive {
  type: 'rotation';
  /** For cone: world space; needn't be unit length. Default [0, 1, 0]. */
  axis?: Vec3;
  /** How far axis may swing from where it was, radians, 0 to π, while the motor or friction lets it turn. Default π, none. */
  cone?: number;
}

export type JointOptions = FixedJointOptions | BallJointOptions | HingeJointOptions | SliderJointOptions | DistanceJointOptions | RotationJointOptions;

/** What world.setJoint takes: any of addJoint's options but a and b. Which fit is checked against the joint's type when called. */
export interface JointChanges {
  type?: JointType;
  anchor?: Vec3;
  anchorB?: Vec3;
  axis?: Vec3;
  min?: number | null;
  max?: number | null;
  length?: number;
  cone?: number;
  /** A number for a hinge or slider, a rotation for a ball or rotation joint; null for none. */
  target?: number | Quat | null;
  motorSpeed?: number;
  motorForce?: number;
  friction?: number;
  frequency?: number;
  dampingRatio?: number;
  collide?: boolean;
}

interface JointInfoBase {
  a: Body;
  b: Body | null;
  collide: boolean;
}

interface JointInfoCommon extends JointInfoBase {
  /** Where a's end is now, world space. */
  anchor: [number, number, number];
}

/** A drive's settings, as jointOf gives them. */
interface DriveInfo {
  motorForce: number;
  friction: number;
  frequency: number;
  dampingRatio: number;
}

/** What world.jointOf returns: a copy, in the form addJoint takes. min and max only if given. */
export type JointInfo =
  | (JointInfoCommon & { type: 'fixed'; anchorB?: [number, number, number] })
  | (JointInfoCommon & DriveInfo & { type: 'ball'; anchorB?: [number, number, number]; axis: [number, number, number]; min?: number; max?: number; cone: number; target: [number, number, number, number] | null })
  | (JointInfoCommon & DriveInfo & { type: 'hinge' | 'slider'; anchorB?: [number, number, number]; axis: [number, number, number]; min?: number; max?: number; target: number | null; motorSpeed: number })
  | (JointInfoCommon & { type: 'distance'; anchorB: [number, number, number]; length: number; min?: number; max?: number; frequency: number; dampingRatio: number })
  | (JointInfoBase & DriveInfo & { type: 'rotation'; axis: [number, number, number]; cone: number; target: [number, number, number, number] | null });

export interface WorldOptions {
  /** m/s², default [0, -9.81, 0]. */
  gravity?: Vec3;
  /** Smaller steps each step is solved in, a whole number from 1, default 8. */
  substeps?: number;
  /** Whether bodies at rest go to sleep, default true. */
  sleeping?: boolean;
  /** Threads besides yours that may share each step's work: 'auto' (default), 0 for none, or at most this many. */
  workers?: 'auto' | number;
}

export class World {
  constructor(options?: WorldOptions);

  /** m/s². Reading gives a frozen copy; assign to change it at any time. */
  get gravity(): readonly [number, number, number];
  set gravity(value: Vec3);
  /** Smaller steps each step is solved in, a whole number from 1. Assign to change it at any time. */
  substeps: number;
  /** Whether bodies at rest go to sleep. Switching it off wakes every sleeping body. */
  sleeping: boolean;
  /** Threads besides yours that may share each step's work: 'auto', 0 for none, or at most this many. Same results either way. */
  workers: 'auto' | number;
  /** How many bodies the world holds. */
  readonly bodyCount: number;
  /** How many joints the world holds. */
  readonly jointCount: number;
  /** How many times the world has been stepped: what moved() takes. */
  readonly stepCount: number;
  /** 7 numbers per slot: position x, y, z, then quaternion x, y, z, w. Replaced when storage resizes: don't keep it. */
  readonly transforms: Float32Array;
  /** As transforms, before the last step: blend from this to transforms to interpolate. */
  readonly previous: Float32Array;

  /**
   * The slots of bodies that moved in any step after `since` (the stepCount you
   * last drew at), always those that moved in the last step, and any added or
   * moved by hand since. Overwritten by the next call or step: don't keep it.
   */
  moved(since: number): Int32Array;
  addBody(options?: BodyOptions): Body;
  setBody(body: Body, changes: BodyChanges): void;
  bodyOf(body: Body): BodyInfo;
  /** A body, with its joints, or a joint. Does nothing if it is already gone. */
  remove(handle: Body | Joint): void;
  /** Holds a to b, or to a point in the world, as its type allows. */
  addJoint(options: JointOptions): Joint;
  setJoint(joint: Joint, changes: JointChanges): void;
  jointOf(joint: Joint): JointInfo;
  /** The nearest body a ray meets. direction is normalized, so distance is in metres. */
  raycast(origin: Vec3, direction: Vec3, options?: RayOptions): Hit | null;
  /** Every body a ray meets, nearest first. */
  raycastAll(origin: Vec3, direction: Vec3, options?: RayOptions): Hit[];
  /** The first body a shape, slid from origin along direction, would touch. */
  shapeCast(shape: QueryShape, origin: Vec3, direction: Vec3, options?: CastOptions): Hit | null;
  /** Every body a shape placed at position overlaps or touches. */
  overlap(shape: QueryShape, position: Vec3, options?: OverlapOptions): Body[];
  /** The last step's begin and end events; given a body, only its own, with it as `a`. A removed body is taken. */
  contacts(body?: Body): ContactEvent[];
  /** Every body touching this one as of the last step, or overlapping it where either is a sensor. */
  touching(body: Body): Body[];
  /** What a character stands on, as of the last step; null in the air, or for any other body. */
  groundOf(body: Body): Ground | null;
  /** A handle for an id. Doesn't check the body is alive. */
  body(id: number): Body;
  /** Moves the world on by dt seconds, exactly. */
  step(dt: number): void;
  /** The whole world as plain, JSON-safe data, for World.load. */
  save(): SavedWorld;
  /** A world from what save() returned, directly or through JSON, stepping exactly as the saved one would. */
  static load(data: SavedWorld): World;
  /** Line segments, nine numbers each: two ends, then r g b. The world's own array: the next call replaces it. */
  debugLines(options?: DebugLineOptions): Float32Array;
  destroy(): void;
}

/** A handle: a world and an id. Compare a.id === b.id, not a === b. */
export class Body {
  private constructor();

  readonly world: World;
  /** Unique to this body for the life of its world; safe to store and send. */
  readonly id: number;
  /** Where its numbers are in world.transforms: at slot * 7. Reused after the body is removed. */
  readonly slot: number;
  /** false once removed, or once its world is destroyed. */
  readonly alive: boolean;
  /** true while at rest and left out of steps until disturbed. Never for a static body. */
  readonly asleep: boolean;
  /** kg. */
  readonly mass: number;
  /** kg·m², the principal moments: about its own x, y and z axes for a box, ball or capsule. A frozen copy. */
  readonly inertia: readonly [number, number, number];
  /** m, a frozen copy: its origin, where it was placed, whatever its centre of mass. */
  readonly position: readonly [number, number, number];
  /** m, in the world, a frozen copy: where its mass is centred, which it turns about. */
  readonly centerOfMass: readonly [number, number, number];
  /** Quaternion [x, y, z, w], a frozen copy. */
  readonly rotation: readonly [number, number, number, number];
  /** m/s, a frozen copy. */
  readonly velocity: readonly [number, number, number];
  /** rad/s about each world axis, a frozen copy. */
  readonly angularVelocity: readonly [number, number, number];

  setPosition(x: number, y: number, z: number): this;
  setRotation(q: Quat): this;
  setAxisAngle(axis: Vec3, radians: number): this;
  /** Yaw about Y, then pitch about X, then roll about Z, in radians. */
  setEuler(yaw: number, pitch: number, roll?: number): this;
  setVelocity(x: number, y: number, z: number): this;
  setAngularVelocity(x: number, y: number, z: number): this;
  /** N, world space, for the next step only. At `point` (world space, m) it turns the body too. */
  applyForce(force: Vec3, point?: Vec3): this;
  /** N·m, world space, for the next step only. */
  applyTorque(torque: Vec3): this;
  /** N·s, world space, at once. At `point` (world space, m) it turns the body too. */
  applyImpulse(impulse: Vec3, point?: Vec3): this;
  /** N·m·s, world space, at once. */
  applyAngularImpulse(impulse: Vec3): this;
  /** Same as world.remove(body). */
  destroy(): void;
}

/** A handle to a joint: a world and an id. Compare a.id === b.id, not a === b. */
export class Joint {
  private constructor();

  readonly world: World;
  /** Unique to this joint for the life of its world. */
  readonly id: number;
  /** false once removed, with either of its bodies, or once its world is destroyed. */
  readonly alive: boolean;
  /** Same as world.remove(joint). */
  destroy(): void;
}
