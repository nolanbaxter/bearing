// Types for Bearing, written by hand beside the JavaScript rather than built
// from it. docs/API.md is the reference; these follow it, and test/types.ts
// holds them to its examples.

/** Three numbers: a plain array, or a typed array of length 3. */
export type Vec3 = readonly [number, number, number] | Float32Array | Float64Array;
/** A quaternion [x, y, z, w]: a plain array, or a typed array of length 4. */
export type Quat = readonly [number, number, number, number] | Float32Array | Float64Array;

/** 'dynamic' is moved by gravity; 'static' never moves unless moved; 'kinematic' moves at its velocity. */
export type BodyType = 'dynamic' | 'static' | 'kinematic';
export type Shape = 'box' | 'sphere' | 'capsule';

interface BodyCommon {
  /** Default 'dynamic'. */
  type?: BodyType;
  /** kg/m³, default 1000 (water). Give this or `mass`, not both. */
  density?: number;
  /** kg. Give this or `density`, not both. */
  mass?: number;
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
export interface BoxOptions extends BodyCommon {
  shape?: 'box';
  /** Full width, height and depth in m, default [1, 1, 1]. */
  size?: Vec3;
  radius?: never;
  height?: never;
}

export interface SphereOptions extends BodyCommon {
  shape: 'sphere';
  /** m, default 0.5. */
  radius?: number;
  size?: never;
  height?: never;
}

export interface CapsuleOptions extends BodyCommon {
  shape: 'capsule';
  /** m, default 0.5. */
  radius?: number;
  /** Full height tip to tip along its own y, in m, default 2. At least twice radius. */
  height?: number;
  size?: never;
}

/** What world.addBody takes. A size option the shape doesn't have is refused, here and at run time. */
export type BodyOptions = BoxOptions | SphereOptions | CapsuleOptions;

/** What world.setBody takes: any of addBody's options. Which sizes fit is checked against the body's shape when called. */
export interface BodyChanges extends BodyCommon {
  shape?: Shape;
  size?: Vec3;
  radius?: number;
  height?: number;
}

interface BodyInfoCommon {
  type: BodyType;
  /** Present if the body was given a mass. */
  mass?: number;
  /** Present if the body was not given a mass. */
  density?: number;
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
  | (BodyInfoCommon & { shape: 'capsule'; radius: number; height: number });

export interface WorldOptions {
  /** m/s², default [0, -9.81, 0]. */
  gravity?: Vec3;
}

export class World {
  constructor(options?: WorldOptions);

  /** m/s². Reading gives a frozen copy; assign to change it at any time. */
  get gravity(): readonly [number, number, number];
  set gravity(value: Vec3);
  /** How many bodies the world holds. */
  readonly bodyCount: number;
  /** 7 numbers per slot: position x, y, z, then quaternion x, y, z, w. Replaced when storage resizes: don't keep it. */
  readonly transforms: Float32Array;
  /** As transforms, before the last step: blend from this to transforms to interpolate. */
  readonly previous: Float32Array;

  addBody(options?: BodyOptions): Body;
  setBody(body: Body, changes: BodyChanges): void;
  bodyOf(body: Body): BodyInfo;
  /** Does nothing if the body is already gone. */
  remove(body: Body): void;
  /** A handle for an id. Doesn't check the body is alive. */
  body(id: number): Body;
  /** Moves the world on by dt seconds, exactly. */
  step(dt: number): void;
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
  /** kg. */
  readonly mass: number;
  /** m, a frozen copy. */
  readonly position: readonly [number, number, number];
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
  /** Same as world.remove(body). */
  destroy(): void;
}
