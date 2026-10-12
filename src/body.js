// Body: a handle to one body in a world.
//
// A Body holds a world and an id, and nothing else. The body's data lives in
// the world's columns, at the slot the id names, so creating a thousand
// handles moves no data, and two handles to one body are two different
// objects. Compare `a.id === b.id`, not `a === b`.
//
// SETTERS, NOT PROPERTIES, for the same reason as Winding's Node. The data is
// shared columns, so `body.position.x = 5` could only work through a live view
// or a Proxy, and neither would know to keep `previous` in step with a
// teleport. So `position` is a frozen copy, assigning to it throws, and
// `setPosition(x, y, z)` is how a body moves.

import { fail, finite, direction, quat, readOnly, show } from './core/check.js';
import { quatAxisAngle, quatEuler } from './core/math.js';
import { SLOT_SPAN } from './core/slots.js';

export class Body {
  #world;
  #id;

  constructor(world, id) {
    this.#world = world;
    this.#id = id;
  }

  get world() { return this.#world; }
  set world(_) { readOnly('body.world'); }

  /** A number, unique to this body for the life of its world. */
  get id() { return this.#id; }
  set id(_) { readOnly('body.id'); }

  /** Where this body's numbers are in `world.transforms`: at slot * 7. */
  get slot() { return this.#id % SLOT_SPAN; }
  set slot(_) { readOnly('body.slot'); }

  /** false once the body is removed, or its world destroyed. Never throws. */
  get alive() { return this.#world._alive(this.#id); }
  set alive(_) { readOnly('body.alive'); }

  /** kg. */
  get mass() { return this.#world._massOf('mass', this.#id); }
  set mass(_) { readOnly('body.mass', "setBody(body, { mass })"); }

  /** m, a frozen copy. */
  get position() { return this.#world._vector('position', this.#id, this.#world._position, 3); }
  set position(_) { readOnly('body.position', 'setPosition(x, y, z)'); }

  /** A quaternion [x, y, z, w], a frozen copy. */
  get rotation() { return this.#world._vector('rotation', this.#id, this.#world._rotation, 4); }
  set rotation(_) { readOnly('body.rotation', 'setRotation(q)'); }

  /** m/s, a frozen copy. */
  get velocity() { return this.#world._vector('velocity', this.#id, this.#world._velocity, 3); }
  set velocity(_) { readOnly('body.velocity', 'setVelocity(x, y, z)'); }

  /** rad/s about each world axis, a frozen copy. */
  get angularVelocity() { return this.#world._vector('angularVelocity', this.#id, this.#world._angular, 3); }
  set angularVelocity(_) { readOnly('body.angularVelocity', 'setAngularVelocity(x, y, z)'); }

  setPosition(x, y, z) {
    this.#world._setPosition('setPosition', this.#id, x, y, z);
    return this;
  }

  /** Takes a quaternion [x, y, z, w], normalized on the way in. */
  setRotation(q) {
    this.#world._slotOf('setRotation', this.#id);
    this.#world._setRotation('setRotation', this.#id, quat('setRotation', 'q', q));
    return this;
  }

  /** Turns the body `radians` about `axis`, which needn't be unit length. Replaces the whole rotation. */
  setAxisAngle(axis, radians) {
    this.#world._slotOf('setAxisAngle', this.#id);
    const a = direction('setAxisAngle', 'axis', axis);
    finite('setAxisAngle', 'radians', radians);
    this.#world._setRotation('setAxisAngle', this.#id, quatAxisAngle(a, radians));
    return this;
  }

  /** Radians: yaw about Y, then pitch about X, then roll about Z, as Winding's setEuler. */
  setEuler(yaw, pitch, roll = 0) {
    this.#world._slotOf('setEuler', this.#id);
    const angles = [yaw, pitch, roll];
    if (!angles.every((a) => typeof a === 'number' && Number.isFinite(a))) {
      fail('setEuler', `yaw, pitch and roll must be finite numbers, got ${show(angles)}`);
    }
    this.#world._setRotation('setEuler', this.#id, quatEuler(yaw, pitch, roll));
    return this;
  }

  setVelocity(x, y, z) {
    this.#world._setVelocity('setVelocity', this.#id, 'velocity', x, y, z);
    return this;
  }

  /** rad/s about each world axis. */
  setAngularVelocity(x, y, z) {
    this.#world._setVelocity('setAngularVelocity', this.#id, 'angularVelocity', x, y, z);
    return this;
  }

  /** Same as world.remove(body). Does nothing if the body is already gone. */
  destroy() {
    this.#world._remove('destroy', this);
  }
}
