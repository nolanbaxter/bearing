# Bearing API reference

Every public call, option and setting, with what it does, what it takes, what it returns and what
it throws. Look a call up in the [index](#index) below, or search this page for its name. The
[README](../README.md) says what Bearing is; this page is how to use it.

```js
import { World } from 'bearing-physics';
```

TypeScript types come with the package, so an editor shows these calls' options as you type. The
option shapes have names you can import, such as `BodyOptions`:
`import type { BodyOptions } from 'bearing-physics'`.

<a id="conventions"></a>
## Conventions

These hold everywhere, so one learned is one learned for good.

- **Every kind of thing works the same way.** `world.addX(options)` makes one and returns a handle.
  `world.setX(handle, changes)` changes any of the options `addX` took, leaving the rest.
  `world.xOf(handle)` returns a copy of its options. It goes with
  [`world.remove(handle)`](#world-remove) or `handle.destroy()`. The kinds so far: bodies.
- **Options are one plain object, and every field has a default.** Leaving one out means the
  default, so `addBody({})` makes a valid body: a 1 m box of water. A field set to `undefined`
  counts as left out.
- **A misspelt option throws**, with the nearest name it might have been:
  `addBody: unknown option 'postion' (did you mean 'position'?)`. A typo is never quietly ignored.
- **Handles hold nothing.** A [`Body`](#body) is a world and an id; the body's data lives in the
  world. Two handles can name the same body, so compare `a.id === b.id`, not `a === b`.
- **Bodies are moved by methods, not by writing properties.** `body.setPosition(x, y, z)`, never
  `body.position = …`, which throws. Setters return the body, so they chain:
  `ball.setPosition(0, 5, 0).setVelocity(2, 0, 0)`.
- **What you read is a copy.** [`world.bodyOf`](#world-bodyof), `body.position` and the like
  return fresh copies; changing them changes nothing.
- **Units are metres, kilograms and seconds**, as in glTF. Y is up, and the axes are
  right-handed. Angles are radians. Time is seconds, as the `dt` you step by.
- **Vectors are plain arrays**, `[x, y, z]`, or any array-like such as a `Float32Array`. A rotation
  is a quaternion `[x, y, z, w]`, as in glTF.
- **Errors are thrown, at the call that took the bad value**, naming the call, the option, what it
  must be and what it got: `addBody: radius must be a positive number, got -2`. Every message a
  call can throw is listed in its entry, word for word, with `…` or `<…>` where a value goes. A
  call that throws has changed nothing.
- **Nothing needs setting up or tearing down in order.** There's no `init()`. The world grows its
  storage as bodies are added and gives it back as they're removed: nothing is sized in advance.
- **The same inputs give the same results, bit for bit, on every machine.** See
  [Determinism](#determinism).

<a id="contents"></a>
## Contents

[World](#world-class) · [Body](#body) · [Contacts](#contacts) · [Layers and masks](#layers) · [Sleeping](#sleeping) · [Queries](#queries) · [Determinism](#determinism) · [Limits](#limits)

<a id="index"></a>
## Index

Every call, setting and entry, A to Z by its name.

**A** &nbsp; [`world.addBody()`](#world-addbody) · [`body.alive`](#body-alive) · [`body.angularVelocity`](#body-angularvelocity) · [`body.applyAngularImpulse()`](#body-applyangularimpulse) · [`body.applyForce()`](#body-applyforce) · [`body.applyImpulse()`](#body-applyimpulse) · [`body.applyTorque()`](#body-applytorque) · [`body.asleep`](#body-asleep)

**B** &nbsp; [`world.body()`](#world-body) · [`world.bodyCount`](#world-bodycount) · [`world.bodyOf()`](#world-bodyof)

**C** &nbsp; [Contacts](#contacts)

**D** &nbsp; [`body.destroy()`](#body-destroy) · [`world.destroy()`](#world-destroy) · [Determinism](#determinism)

**G** &nbsp; [`world.gravity`](#world-gravity)

**I** &nbsp; [`body.id`](#body-id) · [`body.inertia`](#body-inertia)

**L** &nbsp; [Layers and masks](#layers) · [Limits](#limits)

**M** &nbsp; [`body.mass`](#body-mass) · [`world.moved()`](#world-moved)

**O** &nbsp; [`world.overlap()`](#world-overlap)

**P** &nbsp; [`body.position`](#body-position) · [`world.previous`](#world-previous)

**Q** &nbsp; [Queries](#queries)

**R** &nbsp; [`world.raycast()`](#world-raycast) · [`world.raycastAll()`](#world-raycastall) · [`world.remove()`](#world-remove) · [`body.rotation`](#body-rotation)

**S** &nbsp; [`body.setAngularVelocity()`](#body-setangularvelocity) · [`body.setAxisAngle()`](#body-setaxisangle) · [`world.setBody()`](#world-setbody) · [`body.setEuler()`](#body-seteuler) · [`body.setPosition()`](#body-setposition) · [`body.setRotation()`](#body-setrotation) · [`body.setVelocity()`](#body-setvelocity) · [`world.shapeCast()`](#world-shapecast) · [`world.sleeping`](#world-sleeping) · [Sleeping](#sleeping) · [`body.slot`](#body-slot) · [`world.step()`](#world-step) · [`world.stepCount`](#world-stepcount) · [`world.substeps`](#world-substeps)

**T** &nbsp; [`world.transforms`](#world-transforms)

**V** &nbsp; [`body.velocity`](#body-velocity)

**W** &nbsp; [`new World()`](#world) · [`body.world`](#body-world)

<a id="world-class"></a>
## World

A world holds bodies and moves them when you step it. Worlds are independent: two in one page, or
one per worker, never touch each other.

<a id="world"></a>
### `new World(options)` → `World`

Makes an empty world, ready to step at once.

| Option | Default | Meaning |
|---|---|---|
| `gravity` | `[0, -9.81, 0]` | Acceleration of every dynamic body, in m/s². Earth's, pointing down Y. |
| `substeps` | `8` | How many smaller steps each step is solved in. See [`world.substeps`](#world-substeps). |
| `sleeping` | `true` | Whether bodies at rest go to sleep. See [`world.sleeping`](#world-sleeping). |

Returns: the world.

Throws: `World: options must be an object, got …`; `World: unknown option '…' (did you mean '…'?)`; `World: unknown option '…' (the options are gravity, substeps, sleeping)`; `World: gravity must be 3 finite numbers, got …`; `World: substeps must be a whole number, 1 or more, got …`; `World: sleeping must be true or false, got …`.

```js
const world = new World();
const moon = new World({ gravity: [0, -1.62, 0] });
```

Notes: there is nothing to size and nothing to set up. The world holds no storage until the first
body is added.

<a id="world-gravity"></a>
### `world.gravity` → `[x, y, z]`

The acceleration of every dynamic body, in m/s². Reading it gives a frozen copy; assign a new
array to change it, at any time, and the next step uses it.

Throws: `gravity: this world was destroyed`; `world.gravity must be 3 finite numbers, got …`.

```js
world.gravity = [0, 0, 0];         // drifting in space
world.gravity = [0, -9.81, 0];     // and back
```

Notes: static and kinematic bodies are not pulled by gravity. See [`addBody`](#world-addbody)'s
`type`. Changing it wakes every sleeping body, since what held one at rest may not now.

<a id="world-substeps"></a>
### `world.substeps` → `number`

How many smaller steps each [`step`](#world-step) is solved in. Contacts are solved and bodies
moved once per substep, so more substeps hold tall stacks and heavy piles stiffer, and cost about
that much more. Change it at any time.

Throws: `substeps: this world was destroyed`; `world.substeps must be a whole number, 1 or more, got …`.

```js
world.substeps = 16;   // a very tall tower, held stiffer
```

Notes: the default, 8, is measured rather than chosen: the fewest that kept a pyramid of 210
one-metre boxes within 2.5 mm of where it was built after 10 s at 60 Hz (at 7 it strayed 3 mm, at
4, 9 mm). It is not there to be tuned for a correct result; raise it for unusual scenes, such as a
heavy body resting on a much lighter one, a very tall stack, or a step much longer than 1/60 s.

<a id="world-sleeping"></a>
### `world.sleeping` → `boolean`

Whether bodies at rest go to sleep: left out of steps, at no cost, until something disturbs
them. On by default. Change it at any time; switching it off wakes every sleeping body. See
[Sleeping](#sleeping).

Throws: `sleeping: this world was destroyed`; `world.sleeping must be true or false, got …`.

```js
world.sleeping = false;   // everything stays awake, as when checking a scene by eye
```

Notes: with it off, nothing about sleeping is kept or worked out at all. Results with it on and
off differ slightly, because a body falling asleep has its last, too-small-to-see motion stopped;
each is deterministic.

<a id="world-bodycount"></a>
### `world.bodyCount` → `number`

How many bodies the world holds. Read only.

Throws: `bodyCount: this world was destroyed`; `world.bodyCount is read only`.

```js
console.log(world.bodyCount);
```

<a id="world-stepcount"></a>
### `world.stepCount` → `number`

How many times the world has been [stepped](#world-step) since it was made. What
[`moved`](#world-moved) takes. Read only.

Throws: `stepCount: this world was destroyed`; `world.stepCount is read only`.

```js
world.step(1 / 60);
console.log(world.stepCount);   // 1
```

<a id="world-transforms"></a>
### `world.transforms` → `Float32Array`

Every body's position and rotation, 7 numbers per slot: x, y, z in metres, then the quaternion
x, y, z, w. A body's numbers start at `body.slot * 7`. This is what a renderer reads, in one
loop, after each step. Read only.

Throws: `transforms: this world was destroyed`; `world.transforms is read only`.

```js
const t = world.transforms;
for (const body of bodies) {
  const o = body.slot * 7;
  const position = [t[o], t[o + 1], t[o + 2]];
  const rotation = [t[o + 3], t[o + 4], t[o + 5], t[o + 6]];
  // hand position and rotation to whatever draws this body
}
```

Notes: the array is replaced when the world's storage grows or shrinks, which happens only in
[`addBody`](#world-addbody) and [`remove`](#world-remove), so read `world.transforms` again after
those, or simply every frame; do not keep it. Slots of removed bodies, and slots no body has used,
hold leftover numbers: read only the slots of bodies you have. It is a `Float32Array` because that
is what renderers upload. The simulation itself runs in doubles, so these numbers are rounded
copies, and reading them never changes the simulation.

<a id="world-previous"></a>
### `world.previous` → `Float32Array`

Every body's position and rotation as they were before the last [`step`](#world-step), laid out
as [`world.transforms`](#world-transforms) is. With a fixed step, a frame usually falls between two
steps; blending from `previous` to `transforms` by how far it falls draws smooth motion at any
frame rate. Read only.

Throws: `previous: this world was destroyed`; `world.previous is read only`.

```js
// alpha: how far this frame sits between the last step and the next, from 0 to 1.
function blendedPosition(body, alpha) {
  const t = world.transforms, p = world.previous, o = body.slot * 7;
  return [0, 1, 2].map((k) => p[o + k] + (t[o + k] - p[o + k]) * alpha);
}
```

Notes: replaced along with `transforms`; do not keep it. A body moved by hand, with
[`setPosition`](#body-setposition), [`setRotation`](#body-setrotation) or `setBody`'s `position`
or `rotation`, has its `previous` set too, so a teleport is drawn where it lands rather than
sliding there. Blend rotations the short way: if the two quaternions' dot product is negative,
negate one first.

<a id="world-moved"></a>
### `world.moved(since)` → `Int32Array`

The slots of every body an engine has to draw again, if it last drew at step `since`: those that
moved in any step after it, and always those that moved in the last step, which are still between
[`previous`](#world-previous) and [`transforms`](#world-transforms); with bodies added, or moved by
hand, since the last step. A body asleep, or static, and untouched, is never in it, so a world of
10,000 sleeping bodies lists none.

Returns: an `Int32Array` of slots, each once, in no particular order.

Throws: `moved: this world was destroyed`; `moved: since must be a whole number, 0 or more, got …`.

```js
let drawn = 0;   // the stepCount these bodies were last drawn at
function frame(alpha) {
  const t = world.transforms, p = world.previous;
  for (const slot of world.moved(drawn)) {
    const o = slot * 7;
    // draw the body in this slot between p and t, at alpha
  }
  drawn = world.stepCount;
}
```

Notes: exact however many steps run between two frames, none or several. The array is overwritten
by the next call or step; do not keep it. Removed bodies are not listed, since you removed them.
Asked about a step long past, it still lists every body that moved since, only less quickly: it
keeps a short log of what stopped moving, as long as there are bodies, and past that looks at
every body.

<a id="world-addbody"></a>
### `world.addBody(options)` → `Body`

Adds a body and returns its [handle](#body). With no options it is a dynamic 1 m box of water,
1000 kg, at the origin.

| Option | Default | Meaning |
|---|---|---|
| `type` | `'dynamic'` | `'dynamic'`: moved by gravity, and later by contacts and forces. `'static'`: never moves unless you move it, as the ground. `'kinematic'`: moves at the velocity you give it, ignoring gravity, as a lift or a moving platform. |
| `shape` | `'box'` | `'box'`, `'sphere'` or `'capsule'`. |
| `size` | `[1, 1, 1]` | A box's full width, height and depth, along its own x, y and z, in metres. |
| `radius` | `0.5` | A sphere's or capsule's radius, in metres. |
| `height` | `2` | A capsule's full height, tip to tip, along its own y, in metres. At least twice `radius`. The defaults are Godot's capsule, about a person. |
| `density` | `1000` | kg/m³; water's. The mass is the density times the shape's volume. |
| `mass` | from `density` | kg. Give this instead of `density` for a body of a known weight. |
| `inertia` | from the shape and mass | The moments of inertia `[x, y, z]` about the body's own axes, in kg·m²: how hard it is to turn about each. Worked out for a solid shape; give it for one that isn't, such as a hollow ball. `null` goes back to working it out. |
| `friction` | `0.5` | How much sideways push a contact can resist for each newton pressing it together: 0 is ice. The middle of everyday dry materials, about 0.3 to 0.8. |
| `restitution` | `0` | How much of the speed it hits at comes back as a bounce, from 0 (none) to 1 (all of it). |
| `damping` | `0` | Velocity lost per second, as air drag would take it, in 1/s. 0 is none. |
| `angularDamping` | `0` | The same, for its spin, in 1/s. |
| `gravityScale` | `1` | Multiplies the world's gravity, for this body: 0 floats, -1 rises. |
| `layers` | `1` | Which of 32 layers the body is in, one bit each: the first layer only. See [Layers and masks](#layers). |
| `mask` | `0xffffffff` | Which layers it collides with, one bit each: all of them. |
| `position` | `[0, 0, 0]` | Where the body's centre is, in metres. |
| `rotation` | `[0, 0, 0, 1]` | A quaternion `[x, y, z, w]`, normalized as it comes in. |
| `velocity` | `[0, 0, 0]` | m/s. Not for a static body. |
| `angularVelocity` | `[0, 0, 0]` | rad/s about each world axis. Not for a static body. |

Returns: a [`Body`](#body).

Throws: `addBody: this world was destroyed`; `addBody: options must be an object, got …`; `addBody: unknown option '…' (did you mean '…'?)`; `addBody: unknown option '…' (the options are type, shape, size, radius, height, density, mass, inertia, friction, restitution, damping, angularDamping, gravityScale, layers, mask, position, rotation, velocity, angularVelocity)`; `addBody: type must be 'dynamic', 'static' or 'kinematic', got …`; `addBody: shape must be 'box', 'sphere' or 'capsule', got …`; `addBody: a <shape> has no <option>`; `addBody: size must be 3 positive numbers, got …`; `addBody: radius must be a positive number, got …`; `addBody: height must be a positive number, got …`; `addBody: height must be at least twice radius (…), got …`; `addBody: give mass or density, not both`; `addBody: mass must be a positive number, got …`; `addBody: density must be a positive number, got …`; `addBody: inertia must be 3 positive numbers, or null to follow the shape, got …`; `addBody: friction must be 0 or more, got …`; `addBody: restitution must be from 0 to 1, got …`; `addBody: damping must be 0 or more, got …`; `addBody: angularDamping must be 0 or more, got …`; `addBody: gravityScale must be a finite number, got …`; `addBody: layers must be a whole number that fits in 32 bits, got …`; `addBody: mask must be a whole number that fits in 32 bits, got …`; `addBody: position must be 3 finite numbers, got …`; `addBody: rotation must be 4 finite numbers, not all zero, got …`; `addBody: a static body has no <option>`; `addBody: velocity must be 3 finite numbers, got …`; `addBody: angularVelocity must be 3 finite numbers, got …`; `addBody: a world holds at most 16777216 bodies (the slot field of an id is 24 bits)`.

```js
const ground = world.addBody({ type: 'static', size: [20, 1, 20], position: [0, -0.5, 0] });
const marble = world.addBody({ shape: 'sphere', radius: 0.01, density: 2500, position: [0, 1, 0] });
const player = world.addBody({ shape: 'capsule', radius: 0.3, height: 1.8, mass: 80 });
```

Notes: a size option the shape doesn't have throws rather than being ignored, so
`addBody({ radius: 0.5 })`, which forgot `shape: 'sphere'`, says `a box has no radius` instead of
quietly making a 1 m box. A capsule whose height is exactly twice its radius is a sphere, and is
allowed. Bodies collide as soon as they are added; see [Contacts](#contacts).

<a id="world-setbody"></a>
### `world.setBody(body, changes)`

Changes any of the options [`addBody`](#world-addbody) took, with the same names, defaults and
checks, and leaves the rest as they are.

Throws: `setBody: this world was destroyed`; `setBody: body must be a Body, got …`; `setBody: that body belongs to another world`; `setBody: this body was removed`; `setBody: changes must be an object, got …`; `setBody: unknown option '…' (did you mean '…'?)`; `setBody: unknown option '…' (the options are type, shape, size, radius, height, density, mass, inertia, friction, restitution, damping, angularDamping, gravityScale, layers, mask, position, rotation, velocity, angularVelocity)`; `setBody: type must be 'dynamic', 'static' or 'kinematic', got …`; `setBody: shape must be 'box', 'sphere' or 'capsule', got …`; `setBody: a <shape> has no <option>`; `setBody: size must be 3 positive numbers, got …`; `setBody: radius must be a positive number, got …`; `setBody: height must be a positive number, got …`; `setBody: height must be at least twice radius (…), got …`; `setBody: give mass or density, not both`; `setBody: mass must be a positive number, got …`; `setBody: density must be a positive number, got …`; `setBody: inertia must be 3 positive numbers, or null to follow the shape, got …`; `setBody: friction must be 0 or more, got …`; `setBody: restitution must be from 0 to 1, got …`; `setBody: damping must be 0 or more, got …`; `setBody: angularDamping must be 0 or more, got …`; `setBody: gravityScale must be a finite number, got …`; `setBody: layers must be a whole number that fits in 32 bits, got …`; `setBody: mask must be a whole number that fits in 32 bits, got …`; `setBody: position must be 3 finite numbers, got …`; `setBody: rotation must be 4 finite numbers, not all zero, got …`; `setBody: a static body has no <option>`; `setBody: velocity must be 3 finite numbers, got …`; `setBody: angularVelocity must be 3 finite numbers, got …`.

```js
world.setBody(crate, { size: [2, 1, 1] });           // twice as long, and twice as heavy
world.setBody(crate, { mass: 5 });                   // 5 kg, whatever its size
world.setBody(crate, { shape: 'sphere', radius: 1 });
```

Notes: everything that follows from a change follows at once: a new shape or size gives a new
mass and inertia, unless the body was given a `mass` or an `inertia`, which it keeps. A new shape starts from its own default
sizes; the old shape's don't carry over, so a sphere of radius 1.5 made a capsule needs a `height`
of at least 3. A body made static stops: its velocities become zero. A capsule's radius and height
are checked together, so changing one must still fit the other. Setting `position` or `rotation`
moves `previous` too; see [`world.previous`](#world-previous).

<a id="world-bodyof"></a>
### `world.bodyOf(body)` → `object`

A copy of the body's options as they are now, in the form [`addBody`](#world-addbody) takes, so
`world.addBody(world.bodyOf(body))` makes a copy of it. Changing the result changes nothing; pass
it to [`setBody`](#world-setbody) for that.

Returns: `{ type, shape, …sizes, mass or density, inertia, friction, restitution, damping, angularDamping, gravityScale, layers, mask, position, rotation, velocity, angularVelocity }`.
`layers` and `mask` come back unsigned, from 0 to 4294967295, whichever way they were given.
The sizes are the shape's own: `size` for a box, `radius` for a sphere, `radius` and `height` for a
capsule. `mass` if the body was given one, `density` if not, and `inertia` only if it was given one. A
static body has no `velocity` or `angularVelocity`.

Throws: `bodyOf: this world was destroyed`; `bodyOf: body must be a Body, got …`; `bodyOf: that body belongs to another world`; `bodyOf: this body was removed`.

```js
const twin = world.addBody(world.bodyOf(crate));
console.log(world.bodyOf(crate).position);
```

<a id="world-remove"></a>
### `world.remove(body)`

Removes a body. Its handle stops working: [`body.alive`](#body-alive) becomes `false`, and its
methods throw. Removing a body that is already gone does nothing.

Throws: `remove: this world was destroyed`; `remove: body must be a Body, got …`; `remove: that body belongs to another world`.

```js
world.remove(crate);
```

Notes: the body's slot is given to a later body, under a new id, so an old handle can never reach
the new body. When the bodies left use a quarter of the world's storage or less, the storage
shrinks to half, which replaces [`world.transforms`](#world-transforms). Same as
[`body.destroy()`](#body-destroy).

<a id="world-body"></a>
### `world.body(id)` → `Body`

A handle for a body's id: for an id you stored, sent over a network, or saved. It does not check
that the body is alive; check [`body.alive`](#body-alive).

Throws: `body: this world was destroyed`; `body: id must be a whole number from 0 to 9007199254740991, got …`.

```js
const id = crate.id;
const again = world.body(id);
console.log(again.id === crate.id);   // true
```

<a id="world-raycast"></a>
### `world.raycast(origin, direction, options)` → `{ body, point, normal, distance } | null`

Finds the nearest body a ray meets, from `origin` along `direction`, both world-space arrays of 3.
`direction` needn't be unit length: it is normalized, so `distance` is always in metres. See
[Queries](#queries).

| Option | Default | Meaning |
|---|---|---|
| `maxDistance` | `Infinity` | Ignore hits this far or further, in metres. |
| `mask` | `0xffffffff` | Which of the 32 layers to hit, as bits: a body is hit if any of its [`layers`](#layers) is in it. |
| `ignore` | `null` | A body to leave out, such as the one the ray starts from. |

Returns: `{ body, point, normal, distance }` for the nearest hit, or `null`. `body` is the
[`Body`](#body) hit, `point` where the ray met its surface, `normal` the surface's outward
direction there (unit length, facing back at the ray), and `distance` how far along the ray, in
metres. The arrays are fresh copies, and `body` is a handle that can be kept.

Throws: `raycast: this world was destroyed`; `raycast: origin must be 3 finite numbers, got …`; `raycast: direction must be 3 finite numbers, not all zero, got …`; `raycast: options must be an object, got …`; `raycast: unknown option '…' (did you mean '…'?)`; `raycast: unknown option '…' (the options are maxDistance, mask, ignore)`; `raycast: maxDistance must be 0 or more, got …`; `raycast: mask must be a whole number that fits in 32 bits, got …`; `raycast: ignore must be a Body of this world, got …`.

```js
const below = world.raycast(player.position, [0, -1, 0], { maxDistance: 2, ignore: player });
if (below) console.log('ground', below.distance, 'm down');
```

Notes: a ray that starts inside a body meets it at distance 0, with `normal` back along the ray.
Every body can be hit: static, kinematic, dynamic and sleeping. `mask` is matched against the
bodies' `layers` only; what a body's own `mask` lets it collide with doesn't decide what a query
sees. Winding's `scene.raycast` doesn't normalize `direction`; this one does.

<a id="world-raycastall"></a>
### `world.raycastAll(origin, direction, options)` → `Array`

Finds every body a ray meets, nearest first. The same as [`raycast`](#world-raycast) in every
other way, with the same options.

| Option | Default | Meaning |
|---|---|---|
| `maxDistance` | `Infinity` | Ignore hits this far or further, in metres. |
| `mask` | `0xffffffff` | Which of the 32 layers to hit, as bits: a body is hit if any of its [`layers`](#layers) is in it. |
| `ignore` | `null` | A body to leave out, such as the one the ray starts from. |

Returns: an array of `{ body, point, normal, distance }`, nearest first; `[]` if none. Two bodies
met at the same distance come in the order they were added.

Throws: `raycastAll: this world was destroyed`; `raycastAll: origin must be 3 finite numbers, got …`; `raycastAll: direction must be 3 finite numbers, not all zero, got …`; `raycastAll: options must be an object, got …`; `raycastAll: unknown option '…' (did you mean '…'?)`; `raycastAll: unknown option '…' (the options are maxDistance, mask, ignore)`; `raycastAll: maxDistance must be 0 or more, got …`; `raycastAll: mask must be a whole number that fits in 32 bits, got …`; `raycastAll: ignore must be a Body of this world, got …`.

```js
for (const hit of world.raycastAll([0, 20, 0], [0, -1, 0])) console.log(hit.body.id, hit.distance);
```

Notes: each body is listed once, where the ray first meets it.

<a id="world-shapecast"></a>
### `world.shapeCast(shape, origin, direction, options)` → `{ body, point, normal, distance } | null`

Slides a shape from `origin` along `direction` and finds the first body it would touch. `shape`
is a plain object with [`addBody`](#world-addbody)'s names and defaults: `shape` (`'box'`,
`'sphere'` or `'capsule'`), and `size`, `radius` or `height`; `{}` is a 1 m box. The shape
doesn't need to be a body, and nothing moves. See [Queries](#queries).

| Option | Default | Meaning |
|---|---|---|
| `maxDistance` | `Infinity` | Ignore touches this far along or further, in metres. |
| `mask` | `0xffffffff` | Which of the 32 layers to hit, as bits. |
| `ignore` | `null` | A body to leave out, such as the one the shape stands for. |
| `rotation` | `[0, 0, 0, 1]` | The shape's rotation, a quaternion `[x, y, z, w]`. |

Returns: `{ body, point, normal, distance }` for the first touch, or `null`. `distance` is how far
the shape slid before touching, in metres; `point` is where it touches the body, on the body's
surface; `normal` is the body's surface direction there, facing back at the shape.

Throws: `shapeCast: this world was destroyed`; `shapeCast: shape must be an object, got …`; `shapeCast: unknown option '…' (did you mean '…'?)`; `shapeCast: unknown option '…' (the options are shape, size, radius, height)`; `shapeCast: shape must be 'box', 'sphere' or 'capsule', got …`; `shapeCast: a <shape> has no <option>`; `shapeCast: size must be 3 positive numbers, got …`; `shapeCast: radius must be a positive number, got …`; `shapeCast: height must be a positive number, got …`; `shapeCast: height must be at least twice radius (…), got …`; `shapeCast: origin must be 3 finite numbers, got …`; `shapeCast: direction must be 3 finite numbers, not all zero, got …`; `shapeCast: options must be an object, got …`; `shapeCast: unknown option '…' (the options are maxDistance, mask, ignore, rotation)`; `shapeCast: maxDistance must be 0 or more, got …`; `shapeCast: mask must be a whole number that fits in 32 bits, got …`; `shapeCast: ignore must be a Body of this world, got …`; `shapeCast: rotation must be 4 finite numbers, not all zero, got …`.

```js
// How far can the player step forward before walking into something?
const ahead = world.shapeCast({ shape: 'capsule', radius: 0.3, height: 1.8 }, player.position, [0, 0, -1],
  { maxDistance: 0.5, ignore: player });
```

Notes: it stops within the contact tolerance, about a pixel, short of touching, never past it, so a
small fast shape can't skip through a thin wall however far it slides. A shape that starts
overlapping a body touches it at distance 0, with `normal` the way out.

<a id="world-overlap"></a>
### `world.overlap(shape, position, options)` → `Array`

Finds every body a shape placed at `position` overlaps or touches. `shape` is as for
[`shapeCast`](#world-shapecast). See [Queries](#queries).

| Option | Default | Meaning |
|---|---|---|
| `mask` | `0xffffffff` | Which of the 32 layers to find, as bits. |
| `ignore` | `null` | A body to leave out. |
| `rotation` | `[0, 0, 0, 1]` | The shape's rotation, a quaternion `[x, y, z, w]`. |

Returns: an array of [`Body`](#body), in no particular order; `[]` if none.

Throws: `overlap: this world was destroyed`; `overlap: shape must be an object, got …`; `overlap: unknown option '…' (did you mean '…'?)`; `overlap: unknown option '…' (the options are shape, size, radius, height)`; `overlap: shape must be 'box', 'sphere' or 'capsule', got …`; `overlap: a <shape> has no <option>`; `overlap: size must be 3 positive numbers, got …`; `overlap: radius must be a positive number, got …`; `overlap: height must be a positive number, got …`; `overlap: height must be at least twice radius (…), got …`; `overlap: position must be 3 finite numbers, got …`; `overlap: options must be an object, got …`; `overlap: unknown option '…' (the options are mask, ignore, rotation)`; `overlap: mask must be a whole number that fits in 32 bits, got …`; `overlap: ignore must be a Body of this world, got …`; `overlap: rotation must be 4 finite numbers, not all zero, got …`.

```js
const blast = [0, 1, 0];   // where an explosion went off
for (const body of world.overlap({ shape: 'sphere', radius: 4 }, blast)) body.applyImpulse([0, 2000, 0]);
```

<a id="world-step"></a>
### `world.step(dt)`

Moves the world on by `dt` seconds, exactly: there is no hidden accumulator, so call it from
your loop's fixed update, once per step. A dynamic body is pulled by gravity, pushed by whatever
it touches, and moves and turns at its velocities, after any forces and torques applied since the
last step have acted; a kinematic body moves and turns as its velocities say; a static body stays
put.

Throws: `step: this world was destroyed`; `step: dt must be a positive number of seconds, got …`.

```js
function update(dt) {   // called at a fixed rate, such as 60 times a second
  world.step(dt);
}
```

Notes: a step is solved as [`substeps`](#world-substeps) smaller steps of `dt / substeps` each; it
still moves the world on by exactly `dt`. Each is semi-implicit Euler, velocity first and position
from the new velocity, which keeps orbits and pendulums from gaining energy the way plain Euler
does, at the same cost. Before moving anything, a step copies [`transforms`](#world-transforms)
into [`previous`](#world-previous). A world with no bodies costs almost nothing to step; a step
with no forces applied doesn't look for any, one where nothing touches runs no contact solver,
bodies that stay within their boxes from the step before aren't looked up again (see
[Contacts](#contacts)), and sleeping bodies cost nothing at all (see [Sleeping](#sleeping)).

A lopsided body turning freely tumbles as a real one does: a book spun about its middle axis
flips over, and spun about its longest or shortest it doesn't. What a step carries across exactly
is the body's angular momentum, so a tumbling body keeps it to rounding. Its energy can drift
down a little where the real motion keeps it all: measured over 120 s at 60 Hz, a 1 × 2 × 3 m box
flipping at 5 rad/s keeps all of it to within 0.001%, at 20 rad/s 99.98%, and at 50 rad/s 99.4%.
It never gains energy, so it never blows up. A sphere or a cube turns at a constant
angular velocity, exactly.

<a id="world-destroy"></a>
### `world.destroy()`

Lets go of every body and all of the world's storage. Afterwards every call on the world throws,
naming the call, and every body's [`alive`](#body-alive) is `false`. Calling it again does nothing.

```js
world.destroy();
```

Notes: a world starts nothing that keeps running, no timers and no listeners, so there is nothing
else to stop.

<a id="body"></a>
## Body

A Body is a handle to one body in a world. It holds nothing itself: `body.world` is its world and
`body.id` its id, and the data lives in the world. Two Body objects can name the same body, so
compare `a.id === b.id`, not `a === b`. You never make one with `new`;
[`world.addBody`](#world-addbody) and [`world.body`](#world-body) do. Once the body is removed,
every method and reading below throws `<call>: this body was removed`.

<a id="body-id"></a>
### `body.id` → `number`

The body's id: a whole number, unique to this body for the life of its world, safe to store, send
and compare. Read only.

Throws: `body.id is read only`.

```js
const players = new Map([[ball.id, 'Ana']]);
```

Notes: an id is the body's slot plus 2^24 times how many times that slot has been used, which
fits a JavaScript number exactly (below 2^53). See [Limits](#limits).

<a id="body-slot"></a>
### `body.slot` → `number`

Where the body's numbers are in [`world.transforms`](#world-transforms) and
[`world.previous`](#world-previous): at `slot * 7`. Read only.

Throws: `body.slot is read only`.

```js
const o = ball.slot * 7;
const height = world.transforms[o + 1];
```

Notes: a removed body's slot goes to a later body, so key your own objects by slot only while the
body is alive. To name a body for good, use its [`id`](#body-id).

<a id="body-world"></a>
### `body.world` → `World`

The world the body is in. Read only.

Throws: `body.world is read only`.

```js
ball.world.remove(ball);
```

<a id="body-alive"></a>
### `body.alive` → `boolean`

`false` once the body has been removed or its world destroyed. Never throws, so it is the one
thing safe to ask of any handle. Read only.

Throws: `body.alive is read only`.

```js
if (ball.alive) ball.setVelocity(0, 5, 0);
```

<a id="body-asleep"></a>
### `body.asleep` → `boolean`

`true` while the body is asleep: at rest, and left out of steps until something disturbs it. A
static body never is. Read only: a body is woken by whatever disturbs it, never by hand. See
[Sleeping](#sleeping).

Throws: `asleep: this world was destroyed`; `asleep: this body was removed`; `body.asleep is read only`.

```js
if (crate.asleep) console.log('the crate has settled');
```

<a id="body-mass"></a>
### `body.mass` → `number`

The body's mass in kg: the one it was given, or its density times its shape's volume. Read only;
change it with [`setBody`](#world-setbody).

Throws: `mass: this world was destroyed`; `mass: this body was removed`; `body.mass is read only; use setBody(body, { mass })`.

```js
console.log(ball.mass);   // 523.6 for a 0.5 m ball of water
```

Notes: a static or kinematic body acts as if it were infinitely heavy. Its mass is kept, and used
if it is made dynamic.

<a id="body-inertia"></a>
### `body.inertia` → `[x, y, z]`

The body's moments of inertia about its own x, y and z axes, in kg·m², as a frozen copy: how hard
it is to turn about each. Worked out from the shape and mass, or the `inertia` it was given. Read
only; change it with [`setBody`](#world-setbody).

Throws: `inertia: this world was destroyed`; `inertia: this body was removed`; `body.inertia is read only; use setBody(body, { inertia })`.

```js
const [ix, iy, iz] = crate.inertia;   // 166.7 each, for a 1 m box of water
```

Notes: a box w × h × d of mass m has m (h² + d²) / 12 about x, and the same pattern about y and z;
a sphere has 2/5 m r² about every axis; a capsule, a cylinder with two hemispheres, has the sum of
its parts about its centre.

<a id="body-position"></a>
### `body.position` → `[x, y, z]`

Where the body's centre is, in metres, as a frozen copy. Read only; move it with
[`setPosition`](#body-setposition).

Throws: `position: this world was destroyed`; `position: this body was removed`; `body.position is read only; use setPosition(x, y, z)`.

```js
if (ball.position[1] < -100) ball.setPosition(0, 5, 0);
```

Notes: frozen, so `ball.position[1] = 5` throws in a module rather than changing a copy for
nothing. For many bodies at once, read [`world.transforms`](#world-transforms).

<a id="body-rotation"></a>
### `body.rotation` → `[x, y, z, w]`

The body's rotation as a unit quaternion, a frozen copy. Read only; turn it with
[`setRotation`](#body-setrotation), [`setAxisAngle`](#body-setaxisangle) or
[`setEuler`](#body-seteuler).

Throws: `rotation: this world was destroyed`; `rotation: this body was removed`; `body.rotation is read only; use setRotation(q)`.

```js
const [x, y, z, w] = ball.rotation;
```

<a id="body-velocity"></a>
### `body.velocity` → `[x, y, z]`

The body's velocity in m/s, a frozen copy. Read only; change it with
[`setVelocity`](#body-setvelocity). Always zero for a static body.

Throws: `velocity: this world was destroyed`; `velocity: this body was removed`; `body.velocity is read only; use setVelocity(x, y, z)`.

```js
const [vx, vy, vz] = ball.velocity;
const speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
```

<a id="body-angularvelocity"></a>
### `body.angularVelocity` → `[x, y, z]`

How fast the body turns about each world axis, in rad/s, a frozen copy. Read only; change it
with [`setAngularVelocity`](#body-setangularvelocity). Always zero for a static body.

Throws: `angularVelocity: this world was destroyed`; `angularVelocity: this body was removed`; `body.angularVelocity is read only; use setAngularVelocity(x, y, z)`.

```js
const spin = ball.angularVelocity[1];   // about Y
```

<a id="body-setposition"></a>
### `body.setPosition(x, y, z)` → `Body`

Moves the body's centre to (x, y, z), in metres, at once.

Throws: `setPosition: this world was destroyed`; `setPosition: this body was removed`; `setPosition: x, y and z must be finite numbers, got …`.

```js
ball.setPosition(0, 5, 0).setVelocity(2, 0, 0);
```

Notes: a teleport: [`previous`](#world-previous) moves too, so the body is drawn where it lands
rather than sliding there. Its velocity is unchanged. All three numbers are needed; there is no 2D
shorthand in a 3D world.

<a id="body-setrotation"></a>
### `body.setRotation(q)` → `Body`

Sets the body's rotation to the quaternion `q`, `[x, y, z, w]`, normalized as it comes in.
Replaces the whole rotation.

Throws: `setRotation: this world was destroyed`; `setRotation: this body was removed`; `setRotation: q must be 4 finite numbers, not all zero, got …`.

```js
ball.setRotation([0, 0.7071, 0, 0.7071]);   // a quarter turn about Y
```

Notes: a teleport, as [`setPosition`](#body-setposition) is.

<a id="body-setaxisangle"></a>
### `body.setAxisAngle(axis, radians)` → `Body`

Sets the body's rotation to `radians` about `axis`. The axis needn't be unit length; only its
direction counts. Replaces the whole rotation.

Throws: `setAxisAngle: this world was destroyed`; `setAxisAngle: this body was removed`; `setAxisAngle: axis must be 3 finite numbers, not all zero, got …`; `setAxisAngle: radians must be a finite number, got …`.

```js
crate.setAxisAngle([0, 1, 0], Math.PI / 4);   // an eighth of a turn about Y
```

Notes: a teleport, as [`setPosition`](#body-setposition) is. Worked out with Bearing's own sine
and cosine, so the same angle gives the same rotation, to the bit, on every machine; see
[Determinism](#determinism).

<a id="body-seteuler"></a>
### `body.setEuler(yaw, pitch, roll = 0)` → `Body`

Sets the body's rotation from three angles in radians: yaw about Y, then pitch about X, then roll
about Z. The same order as Winding's `setEuler`. Replaces the whole rotation.

Throws: `setEuler: this world was destroyed`; `setEuler: this body was removed`; `setEuler: yaw, pitch and roll must be finite numbers, got …`.

```js
player.setEuler(Math.PI / 2, 0);   // face along -X
```

Notes: a teleport, as [`setPosition`](#body-setposition) is. Turned into a quaternion at once;
the angles aren't kept, so reading back gives [`rotation`](#body-rotation).

<a id="body-setvelocity"></a>
### `body.setVelocity(x, y, z)` → `Body`

Sets the body's velocity, in m/s.

Throws: `setVelocity: this world was destroyed`; `setVelocity: this body was removed`; `setVelocity: x, y and z must be finite numbers, got …`; `setVelocity: a static body has no velocity`.

```js
ball.setVelocity(0, 5, 0);   // a hop
```

Notes: a static body doesn't move; to move a body by velocity without gravity, make it
`'kinematic'` with [`setBody`](#world-setbody).

<a id="body-setangularvelocity"></a>
### `body.setAngularVelocity(x, y, z)` → `Body`

Sets how fast the body turns about each world axis, in rad/s.

Throws: `setAngularVelocity: this world was destroyed`; `setAngularVelocity: this body was removed`; `setAngularVelocity: x, y and z must be finite numbers, got …`; `setAngularVelocity: a static body has no angularVelocity`.

```js
crate.setAngularVelocity(0, Math.PI, 0);   // half a turn a second about Y
```

<a id="body-applyforce"></a>
### `body.applyForce(force, point)` → `Body`

Pushes the body with `force`, in newtons, for the next [`step`](#world-step), after which it is
cleared. Applied at `point`, in world space and metres, it turns the body too; left out, it acts
through the body's centre. Several forces before one step add up.

Throws: `applyForce: this world was destroyed`; `applyForce: this body was removed`; `applyForce: force must be 3 finite numbers, got …`; `applyForce: point must be 3 finite numbers, got …`.

```js
function update(dt) {
  crate.applyForce([0, 15000, 0]);   // a steady lift: apply it every step
  world.step(dt);
}
```

Notes: a force lasts one step, so a steady push is applied every step, before each
[`step`](#world-step); nothing builds up between steps out of sight. Static and kinematic bodies act
as if infinitely heavy, so forces don't move them; the arguments are still checked. Forces are in
world space, whichever way the body is turned.

<a id="body-applytorque"></a>
### `body.applyTorque(torque)` → `Body`

Twists the body with `torque`, in newton-metres about each world axis, for the next
[`step`](#world-step), after which it is cleared.

Throws: `applyTorque: this world was destroyed`; `applyTorque: this body was removed`; `applyTorque: torque must be 3 finite numbers, got …`.

```js
crate.applyTorque([0, 200, 0]);   // spin it up about Y, this step
```

Notes: as [`applyForce`](#body-applyforce): one step, adding up, nothing for a static or kinematic
body.

<a id="body-applyimpulse"></a>
### `body.applyImpulse(impulse, point)` → `Body`

Changes the body's velocity at once, by `impulse` in newton-seconds divided by its mass: a kick, a
hit, a jump. Applied at `point`, in world space and metres, it changes the angular velocity too;
left out, it acts through the body's centre.

Throws: `applyImpulse: this world was destroyed`; `applyImpulse: this body was removed`; `applyImpulse: impulse must be 3 finite numbers, got …`; `applyImpulse: point must be 3 finite numbers, got …`.

```js
crate.applyImpulse([0, 0, -500], [2.4, 1, 0]);   // kicked near one edge: it slides and swings
```

Notes: unlike a force, it doesn't wait for a step: [`velocity`](#body-velocity) reads the change
straight away. Nothing for a static or kinematic body.

<a id="body-applyangularimpulse"></a>
### `body.applyAngularImpulse(impulse)` → `Body`

Changes the body's angular velocity at once, by `impulse` in newton-metre-seconds about each
world axis, through the body's inertia.

Throws: `applyAngularImpulse: this world was destroyed`; `applyAngularImpulse: this body was removed`; `applyAngularImpulse: impulse must be 3 finite numbers, got …`.

```js
ball.applyAngularImpulse([0, 50, 0]);   // set it spinning about Y
```

Notes: nothing for a static or kinematic body.

<a id="body-destroy"></a>
### `body.destroy()`

Removes the body from its world. Same as [`world.remove(body)`](#world-remove); does nothing if
the body is already gone.

Throws: `destroy: this world was destroyed`.

```js
crate.destroy();
```

<a id="contacts"></a>
## Contacts

Bodies collide as soon as they are added: there is nothing to switch on, and nothing to tune.
Every step finds which bodies touch, or will within the step, and pushes them so they don't sink
into each other, slide no more than friction allows, and bounce as much as their restitution says.

- **Who pushes whom.** Dynamic bodies push each other. A kinematic body pushes dynamic ones and is
  never pushed back, so a moving platform carries what stands on it. Static and kinematic bodies
  don't collide with each other, since neither would move.
- **Materials mix.** Two bodies' frictions combine as their geometric mean, √(a · b), so ice is
  slippery against anything; their restitutions as the larger, so a bouncy ball bounces off
  anything.
- **Friction is a cone.** A contact resists sliding in every direction alike, up to its friction
  times how hard it is pressed, so a box pushed diagonally slows exactly as one pushed straight.
- **Tolerances scale with what touches.** A contact's tolerance is a two-hundredth of the smaller
  body's thinnest size: about a pixel, for a body you can see whole on screen. That is 5 mm for a
  1 m box, 0.05 mm for a 1 cm marble and 10 cm for a 20 m-wide building. It sets how fast an overlap
  is pushed out, how close two answers count as the same, and when a bounce is too small to keep,
  so a marble and a building both settle with nothing changed between them.
- **Resting contacts sag a little.** A contact is a very stiff spring, and a spring holds a weight
  by giving under it: g / ω², which at the default [`substeps`](#world-substeps) and 60 Hz is
  0.07 mm, whatever the size of the body. That is within the tolerance of anything over about 1.4 cm
  across, and 0.7% of a 1 cm marble. Doubling the substeps quarters it. A stack sags by about that
  much at each contact, more at the bottom, which carries more: a 10-box stack of 1 m boxes settles
  7 mm in all.
- **Contacts start before touching.** A pair close enough to meet within the step gets its contact
  early, so a falling box is stopped at the floor rather than sinking in and being pushed back out,
  and a fast body doesn't pass through a thin one between two steps.
- **Only what can touch is looked at.** Each body sits in a tree of nested boxes, inside a box a
  little bigger than itself and stretched a few steps ahead along its velocity. Only a body that
  leaves its box is looked up again, and only bodies whose boxes overlap are tested in detail, so a
  pile at rest costs almost nothing to keep, a body flying steadily is looked up every few steps
  rather than every step, and a world of thousands of bodies spread apart costs about what each one
  moving does.
- **Contacts are patches, not points.** A ball touches the floor at one point and a capsule along
  one line, and a point can't stop a body turning about itself: rolling or spinning in place, it
  would go on for ever. A real contact is a small flattened patch, and so is every contact here:
  as wide as its tolerance, from the size of the body rolling on it. Its push, leaning to the
  patch's edge as the body rolls over it, resists rolling; friction across it resists spinning.
  For any ball or capsule, that is a rolling resistance of 0.01, about rubber on concrete or a
  billiard ball on cloth: a ball rolled at 1 m/s rolls 7.1 m and stops, at any size. A ball spinning
  in place at 0.8 rad/s with a 0.4 m radius stops in 3.9 s. Contacts of three or four points, a
  box flat on something, already resist both through their spread, and get neither.
- **Bounces stop.** A contact bounces only when it meets faster than a bounce that would rise no
  higher than its tolerance, √(2 g tolerance): below that, a body settles instead of bouncing for
  ever on rounding.

Notes: a contact's patch also holds things a point contact wouldn't. A ball rests on a slope
gentler than its rolling resistance, 0.57° (tan θ = 0.01), and rolls down a steeper one. A box
standing on an edge stays up while its centre is within its tolerance of being over the edge: a
1 m cube 0.41° past balance stands, and 0.45° past falls. Friction 0 leaves no friction across a
patch, so a ball spinning in place on ice spins on, though its rolling still slows.

<a id="layers"></a>
## Layers and masks

Every body is in some of 32 layers, given as the bits of [`layers`](#world-addbody), and collides
only with the layers in its [`mask`](#world-addbody). Two bodies collide when each one's layers are
in the other's mask, so either one can refuse the other. By default every body is in the first
layer and its mask has all 32, so everything collides with everything.

```js
const PLAYER = 1 << 1, DEBRIS = 1 << 2;
world.addBody({ shape: 'capsule', layers: PLAYER });
world.addBody({ size: [0.2, 0.2, 0.2], layers: DEBRIS, mask: ~PLAYER });   // passes through players
```

Notes: either form of a 32-bit number is taken, so `~PLAYER` and `1 << 31`, which JavaScript
makes negative, work as written; [`bodyOf`](#world-bodyof) gives them back unsigned. A change
with [`setBody`](#world-setbody) takes effect at the next step: contacts between bodies that may no
longer collide end there, and bodies that now may collide start to.

<a id="sleeping"></a>
## Sleeping

A body that has stopped is put to sleep: left out of every step, at no cost, until something
disturbs it. A pile at rest then costs nothing, which is most of what makes a large world fast:
in a game, most things are at rest most of the time. There is nothing to call, and nothing to
tune; [`world.sleeping`](#world-sleeping) switches it off.

- **Islands sleep together.** Bodies touching each other form an island, and an island sleeps
  only when every body in it has stopped, and wakes all at once. If the bottom box of a stack
  slept while the top one still moved, the top one would push on a body that couldn't answer.
  Static and kinematic bodies join no island, or everything on one floor would be one island.
- **Stopped means gone nowhere you could see.** A body counts as stopped while no point of it
  strays more than its contact tolerance, about a pixel, from where it came to rest. Moving
  steadily, that is under 5 mm/s for a 1 m box and 0.05 mm/s for a 1 cm marble; buzzing in place,
  as a body deep in a pile can, it is stopped however fast it buzzes. An island sleeps once all
  its bodies have been stopped for a second, and its velocities are then set to exactly zero. A
  second is measured, the shortest that put nothing to sleep that was about to move: a box
  creeping on a slope a quarter of a degree past where friction holds it, and a cube a tenth of a
  degree past balance on its edge, tipping slowly at first.
- **Anything that disturbs a body wakes its island.** A touch from an awake body; any setter,
  force or impulse; [`setBody`](#world-setbody); removing a body it rests on or that rests on it;
  moving or changing a static body it touches; changing [`world.gravity`](#world-gravity). A
  stack woken by a touch is held up in that same step.
- **What it costs.** A sleeping body is not moved, bounded, filed or solved, and its contacts are
  not looked at. A world whose bodies are all asleep steps in a few microseconds, whatever its
  size.

Notes: a body starting from rest that would only speed up very slowly, slower than twice its
tolerance over a second squared (0.01 m/s² for a 1 m box), can fall asleep before it moves: a
box on a slope within a twentieth of a degree of where friction holds it stays put. Nudge it, or
switch sleeping off. A heap sleeps only when all of it has stopped, so one slowly settling corner
keeps the whole heap awake until it, too, comes to rest.

<a id="queries"></a>
## Queries

[`raycast`](#world-raycast), [`raycastAll`](#world-raycastall), [`shapeCast`](#world-shapecast) and
[`overlap`](#world-overlap) ask the world questions without changing it.

- **They see the world as it is now.** Every body is tested where it is at the moment of asking,
  after the last step and any teleport since, not where the last step began.
- **They look only near the answer.** Each walks the tree of nested boxes the bodies are filed in
  for finding contacts, opening only boxes the ray or shape reaches; a ray stops looking past
  the nearest thing it has already met. So a ray across a world of thousands of bodies tests a
  handful exactly.
- **Exact for every shape.** A ray meets a sphere, a capsule or a box where the equations say. A
  shape cast measures the gap to each body it might touch with GJK (Gilbert, Johnson and
  Keerthi's closest-point method), slides the shape forward by as much as is surely safe, and
  measures again, which can never step past a touch.
- **Nothing kept, nothing spent.** A query costs nothing until called, and asking never changes
  what the world does next: the same steps give the same results to the bit, asked or not.

Notes: results are fresh objects each call; a query isn't meant for the inside of a step.

<a id="determinism"></a>
## Determinism

The same calls, with the same arguments, in the same order, give the same results to the bit: the
same `transforms`, the same velocities, the same everything, on any CPU, in any browser, in Node,
Deno or Bun, in a worker or not. That is what replays, tests that compare whole worlds, and
lockstep multiplayer need.

It holds because JavaScript defines `+`, `-`, `*`, `/` and `Math.sqrt` exactly: it forbids fused
multiply-add and extended precision, so every engine rounds them the same way. The functions the
language leaves to each engine, such as `Math.sin`, `Math.cos` and `Math.atan2`, are never used
inside Bearing; where it needs an angle, as in [`setAxisAngle`](#body-setaxisangle), it uses its own
sine and cosine, built from the exact operations. A test fails if any of the others appears.

What it does not cover:

- **Different versions of Bearing.** A fix can change results. Pin the version on every machine
  that has to agree.
- **Your own inputs.** If your code works out a position with `Math.sin` on two machines, the two
  may differ in the last bit before Bearing sees them. Send the numbers, not the formula, or
  compute them the same way everywhere.

<a id="limits"></a>
## Limits

One limit is hard, and it is derived rather than chosen:

- **16,777,216 bodies in one world (2^24).** A body's id is its slot plus 2^24 times its
  generation, and the whole id has to stay below 2^53 to be an exact JavaScript number. Of those
  53 bits, 24 hold the slot and 29 hold the generation. Adding a body past the limit throws
  `addBody: a world holds at most 16777216 bodies (the slot field of an id is 24 bits)`.

A slot can be reused 536,870,911 times (2^29 − 1) before it is retired and never handed out
again, so no old id can ever match a new body. Reused once a step at 60 Hz, that is 280 years, so
in practice a world never retires one.
