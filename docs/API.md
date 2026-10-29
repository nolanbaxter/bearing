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
  [`world.remove(handle)`](#world-remove) or `handle.destroy()`. The kinds so far: bodies and
  joints.
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

[World](#world-class) · [Body](#body) · [Joint](#joint) · [Hulls and compounds](#hulls) · [Meshes and heightfields](#meshes) · [Character bodies](#characters) · [Joints](#joints) · [Contacts](#contacts) · [Contact events and sensors](#events) · [Layers and masks](#layers) · [Sleeping](#sleeping) · [Queries](#queries) · [Determinism](#determinism) · [Limits](#limits)

<a id="index"></a>
## Index

Every call, setting and entry, A to Z by its name.

**A** &nbsp; [`world.addBody()`](#world-addbody) · [`world.addJoint()`](#world-addjoint) · [`body.alive`](#body-alive) · [`joint.alive`](#joint-alive) · [`body.angularVelocity`](#body-angularvelocity) · [`body.applyAngularImpulse()`](#body-applyangularimpulse) · [`body.applyForce()`](#body-applyforce) · [`body.applyImpulse()`](#body-applyimpulse) · [`body.applyTorque()`](#body-applytorque) · [`body.asleep`](#body-asleep)

**B** &nbsp; [`world.body()`](#world-body) · [`world.bodyCount`](#world-bodycount) · [`world.bodyOf()`](#world-bodyof)

**C** &nbsp; [`body.centerOfMass`](#body-centerofmass) · [Character bodies](#characters) · [Contact events and sensors](#events) · [Contacts](#contacts) · [`world.contacts()`](#world-contacts)

**D** &nbsp; [`world.debugLines()`](#world-debuglines) · [`body.destroy()`](#body-destroy) · [`joint.destroy()`](#joint-destroy) · [`world.destroy()`](#world-destroy) · [Determinism](#determinism)

**G** &nbsp; [`world.gravity`](#world-gravity) · [`world.groundOf()`](#world-groundof)

**H** &nbsp; [Hulls and compounds](#hulls)

**I** &nbsp; [`body.id`](#body-id) · [`joint.id`](#joint-id) · [`body.inertia`](#body-inertia)

**J** &nbsp; [Joint](#joint) · [`world.jointCount`](#world-jointcount) · [`world.jointOf()`](#world-jointof) · [Joints](#joints)

**L** &nbsp; [Layers and masks](#layers) · [Limits](#limits) · [`World.load()`](#world-load)

**M** &nbsp; [`body.mass`](#body-mass) · [Meshes and heightfields](#meshes) · [`world.moved()`](#world-moved)

**O** &nbsp; [`world.overlap()`](#world-overlap)

**P** &nbsp; [`body.position`](#body-position) · [`world.previous`](#world-previous)

**Q** &nbsp; [Queries](#queries)

**R** &nbsp; [`world.raycast()`](#world-raycast) · [`world.raycastAll()`](#world-raycastall) · [`readGlb()`](#readglb) · [`world.remove()`](#world-remove) · [`body.rotation`](#body-rotation)

**S** &nbsp; [`world.save()`](#world-save) · [`body.setAngularVelocity()`](#body-setangularvelocity) · [`body.setAxisAngle()`](#body-setaxisangle) · [`world.setBody()`](#world-setbody) · [`body.setEuler()`](#body-seteuler) · [`world.setJoint()`](#world-setjoint) · [`body.setPosition()`](#body-setposition) · [`body.setRotation()`](#body-setrotation) · [`body.setVelocity()`](#body-setvelocity) · [`world.shapeCast()`](#world-shapecast) · [`world.sleeping`](#world-sleeping) · [Sleeping](#sleeping) · [`body.slot`](#body-slot) · [`world.step()`](#world-step) · [`world.stepCount`](#world-stepcount) · [`world.substeps`](#world-substeps)

**T** &nbsp; [`world.touching()`](#world-touching) · [`world.transforms`](#world-transforms)

**V** &nbsp; [`body.velocity`](#body-velocity)

**W** &nbsp; [`new World()`](#world) · [`world.workers`](#world-workers) · [`body.world`](#body-world) · [`joint.world`](#joint-world)

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
| `workers` | `'auto'` | How many threads besides yours may share each step's work. See [`world.workers`](#world-workers). |

Returns: the world.

Throws: `World: options must be an object, got …`; `World: unknown option '…' (did you mean '…'?)`; `World: unknown option '…' (the options are gravity, substeps, sleeping, workers)`; `World: gravity must be 3 finite numbers, got …`; `World: substeps must be a whole number, 1 or more, got …`; `World: sleeping must be true or false, got …`; `World: workers must be 'auto' or a whole number, 0 or more, got …`.

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

Notes: static and kinematic bodies are not pulled by gravity, and a character only in the air. See [`addBody`](#world-addbody)'s
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

<a id="world-workers"></a>
### `world.workers` → `'auto' | number`

How many workers, threads besides the one stepping the world, may share each
[`step`](#world-step)'s work: `'auto'`, the default, for Bearing's choice; `0` for none; or a
whole number for at most that many. Change it at any time; changing it ends the workers the world
had.

Throws: `workers: this world was destroyed`; `world.workers must be 'auto' or a whole number, 0 or more, got …`.

```js
world.workers = 0;   // one thread: nothing started, nothing shared
```

Notes: results are the same to the bit with any number of workers or none (see
[Determinism](#determinism)). The workers share three things with the stepping thread, which
takes its share too: finding the contacts between boxes, balls and capsules; preparing every
contact; and solving them, in groups that share no body that moves, one group after another as on
one thread, where only the order within a group, which changes no result, differs. Hulls, meshes,
heightfields and compounds are found, and joints, sleep and the rest run, on the stepping thread.

- **They start by themselves**, the first step with enough work for them (a group of four
  contacts, or 128 pairs of bodies near enough to touch), and steps run on one thread until they
  are running, a few tens of milliseconds later. A world that never has that much never starts
  any. There is nothing to wait for and nothing to call.
- **`'auto'` is half the machine's threads, less one**, and at least one where there are two
  (`navigator.hardwareConcurrency` in a browser, `os.availableParallelism()` in Node): one worker
  a core where each core runs two threads, since the second shares the core's arithmetic. On an
  i7-1260P laptop, 16 threads, that is 7, which stepped a 1,000-box pile 3.3 times as fast as one
  thread, within 2% of the best of any count; 11 and 15 were slower. A number is never more than
  the machine's threads less one. Fewer workers spin on fewer cores while a step runs: set a
  number to trade speed for power.
- **Shared memory is needed.** In a browser that means a cross-origin isolated page, one served
  with `Cross-Origin-Opener-Policy: same-origin` and `Cross-Origin-Embedder-Policy: require-corp`
  (`crossOriginIsolated` is then `true`). Without it, or in Node before 20.16, the world runs on
  one thread, with the same results.
- **Between steps they sleep**, using nothing. While a step runs they wait for the next group by
  spinning, so a step uses up to that many more cores while it lasts; but not through its joints,
  which run on one thread: through a joint pass that last took over 50 µs they sleep, and the
  next group wakes them.
- **[`destroy()`](#world-destroy) ends them**, and so does the world being garbage collected. They
  never keep a Node process from exiting.
- **They aren't saved.** A world from [`World.load`](#world-load) has `'auto'`, as a new one does.

<a id="world-bodycount"></a>
### `world.bodyCount` → `number`

How many bodies the world holds. Read only.

Throws: `bodyCount: this world was destroyed`; `world.bodyCount is read only`.

```js
console.log(world.bodyCount);
```

<a id="world-jointcount"></a>
### `world.jointCount` → `number`

How many joints the world holds. Read only.

Throws: `jointCount: this world was destroyed`; `world.jointCount is read only`.

```js
console.log(world.jointCount);
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
| `type` | `'dynamic'`; `'static'` for a mesh or heightfield | `'dynamic'`: moved by gravity, and later by contacts and forces. `'static'`: never moves unless you move it, as the ground. `'kinematic'`: moves at the velocity you give it, ignoring gravity, as a lift or a moving platform. `'character'`: moves at the velocity you give it, but collides as a person walking does; see [Character bodies](#characters). A mesh or heightfield can't be dynamic or a character; a hull or compound can't be a character. |
| `shape` | `'box'` | `'box'`, `'sphere'`, `'capsule'`, `'hull'`, `'compound'`, `'mesh'` or `'heightfield'`. A hull is the convex solid round some points, and a compound several shapes as one body: see [Hulls and compounds](#hulls). The last two are level geometry: see [Meshes and heightfields](#meshes). |
| `size` | `[1, 1, 1]` | A box's full width, height and depth, along its own x, y and z, in metres. |
| `radius` | `0.5` | A sphere's or capsule's radius, in metres. |
| `height` | `2` | A capsule's full height, tip to tip, along its own y, in metres. At least twice `radius`. The defaults are Godot's capsule, about a person. |
| `vertices` | a hull's: a 1 m cube's corners; a mesh's: `[]` | A hull's points, or a mesh's corners, three numbers each, in metres, in its own frame. Any array or typed array. A hull's must span a solid: 4 or more, not all in one plane. |
| `indices` | `[]` | A mesh's triangles, three corner numbers each, counting from 0. |
| `heights` | `[0, 0, 0, 0]` | A heightfield's heights, in metres, row by row: `columns` along x to a row, rows along z. The default is a flat 1 m square. |
| `columns` | `2` | How many heights to a heightfield's row: 2 or more. |
| `spacing` | `1` | Metres between a heightfield's neighbouring heights, along x and z. |
| `parts` | one 1 m box | A compound's shapes: a list of one or more, each `{ shape, size, radius, height, vertices, position, rotation, density }`. `shape` is `'box'` (the default), `'sphere'`, `'capsule'` or `'hull'`, sized by the same options and defaults as a body of that shape; `position` and `rotation` place it from the body's origin and against its rotation; `density` is the body's unless given. See [Hulls and compounds](#hulls). |
| `density` | `1000` | kg/m³; water's. The mass is the density times the shape's volume. Not for a mesh or heightfield, which encloses none. |
| `mass` | from `density` | kg. Give this instead of `density` for a body of a known weight. Not for a mesh or heightfield. |
| `inertia` | from the shape and mass | The moments of inertia `[x, y, z]` about the body's own axes, in kg·m²: how hard it is to turn about each. Worked out for a solid shape, a hull's or compound's about its principal axes (see [Hulls and compounds](#hulls)); give it for one that isn't solid, such as a hollow ball. `null` goes back to working it out. Not for a mesh or heightfield. |
| `centerOfMass` | from the shape | Where its mass is centred, `[x, y, z]` in metres, in its own frame: a box's, ball's or capsule's centre, and worked out for a hull or compound. Give it to move it, as a car body's is lowered so it doesn't roll over on a turn; the inertia stays the shape's, about its own centre of mass, unless `inertia` is given too. `null` goes back to the shape's. Not for a mesh, heightfield or character. |
| `friction` | `0.5` | How much sideways push a contact can resist for each newton pressing it together: 0 is ice. The middle of everyday dry materials, about 0.3 to 0.8. |
| `restitution` | `0` | How much of the speed it hits at comes back as a bounce, from 0 (none) to 1 (all of it). |
| `damping` | `0` | Velocity lost per second, as air drag would take it, in 1/s. 0 is none. |
| `angularDamping` | `0` | The same, for its spin, in 1/s. |
| `gravityScale` | `1` | Multiplies the world's gravity, for this body: 0 floats, -1 rises. |
| `layers` | `1` | Which of 32 layers the body is in, one bit each: the first layer only. See [Layers and masks](#layers). |
| `mask` | `0xffffffff` | Which layers it collides with, one bit each: all of them. |
| `sensor` | `false` | A sensor reports what overlaps it and pushes nothing. See [Contact events and sensors](#events). |
| `lockRotation` | `[false, false, false]` | The world axes it never turns about, x, y and z: `[true, false, true]` keeps it upright. See [Character bodies](#characters). |
| `maxSlope` | `π / 4` (45°) | A character's steepest walkable ground, in radians, from 0 to π/2: about where walking turns into climbing, the slope friction 1 can hold. Characters only. |
| `stepHeight` | its height ÷ 9 | The tallest step a character walks up, in metres: a capsule's height, a box's, or a sphere's diameter, over 9. Building codes cap a stair's rise near 0.2 m, a ninth of a 1.8 m person, so a 20 cm toy climbs 2.2 cm. `null` follows the size again. Characters only. |
| `position` | `[0, 0, 0]` | Where the body's origin is, in metres: a box's, ball's or capsule's centre, and the point a hull's points and a compound's parts are given from. |
| `rotation` | `[0, 0, 0, 1]` | A quaternion `[x, y, z, w]`, normalized as it comes in. |
| `velocity` | `[0, 0, 0]` | m/s. Not for a static body. |
| `angularVelocity` | `[0, 0, 0]` | rad/s about each world axis. Not for a static body. A body other than a sphere spins at most half a turn a step, which the step holds it to; see [Contacts](#contacts). |

Returns: a [`Body`](#body).

Throws: `addBody: this world was destroyed`; `addBody: options must be an object, got …`; `addBody: unknown option '…' (did you mean '…'?)`; `addBody: unknown option '…' (the options are type, shape, size, radius, height, vertices, indices, heights, columns, spacing, parts, density, mass, inertia, centerOfMass, friction, restitution, damping, angularDamping, gravityScale, layers, mask, sensor, lockRotation, maxSlope, stepHeight, position, rotation, velocity, angularVelocity)`; `addBody: type must be 'dynamic', 'static', 'kinematic' or 'character', got …`; `addBody: shape must be 'box', 'sphere', 'capsule', 'hull', 'compound', 'mesh' or 'heightfield', got …`; `addBody: a <shape> has no <option>`; `addBody: type must be 'static' or 'kinematic' for a <shape>, got …`; `addBody: size must be 3 positive numbers, got …`; `addBody: radius must be a positive number, got …`; `addBody: height must be a positive number, got …`; `addBody: height must be at least twice radius (…), got …`; `addBody: vertices must be finite numbers, three to a vertex, got …`; `addBody: indices must be whole numbers below the vertex count (…), three to a triangle, got …`; `addBody: columns must be a whole number, 2 or more, got …`; `addBody: spacing must be a positive number, got …`; `addBody: heights must be finite numbers, rows of columns (…), 2 rows or more, got …`; `addBody: vertices must span a solid (4 points not in one plane), got …`; `addBody: type must be 'dynamic', 'static' or 'kinematic' for a <shape>, got …`; `addBody: parts must be a list of one or more parts, got …`; `addBody: parts[…] must be an object, got …`; `addBody: parts[…]: unknown option '…' (did you mean '…'?)`; `addBody: parts[…]: unknown option '…' (the options are shape, size, radius, height, vertices, position, rotation, density)`; `addBody: parts[…].shape must be 'box', 'sphere', 'capsule' or 'hull', got …`; `addBody: parts[…]: a <shape> has no <option>`; `addBody: parts[…].size must be 3 positive numbers, got …`; `addBody: parts[…].radius must be a positive number, got …`; `addBody: parts[…].height must be a positive number, got …`; `addBody: parts[…].height must be at least twice radius (…), got …`; `addBody: parts[…].vertices must be finite numbers, three to a vertex, got …`; `addBody: parts[…].vertices must span a solid (4 points not in one plane), got …`; `addBody: parts[…].position must be 3 finite numbers, got …`; `addBody: parts[…].rotation must be 4 finite numbers, not all zero, got …`; `addBody: parts[…].density must be a positive number, got …`; `addBody: give mass or density, not both`; `addBody: mass must be a positive number, got …`; `addBody: density must be a positive number, got …`; `addBody: inertia must be 3 positive numbers, or null to follow the shape, got …`; `addBody: centerOfMass must be 3 finite numbers, got …`; `addBody: a character body has no centerOfMass`; `addBody: friction must be 0 or more, got …`; `addBody: restitution must be from 0 to 1, got …`; `addBody: damping must be 0 or more, got …`; `addBody: angularDamping must be 0 or more, got …`; `addBody: gravityScale must be a finite number, got …`; `addBody: layers must be a whole number that fits in 32 bits, got …`; `addBody: mask must be a whole number that fits in 32 bits, got …`; `addBody: sensor must be true or false, got …`; `addBody: lockRotation must be 3 true or false values, got …`; `addBody: maxSlope must be an angle from 0 to π/2, in radians, got …`; `addBody: stepHeight must be 0 or more, or null to follow the size, got …`; `addBody: position must be 3 finite numbers, got …`; `addBody: rotation must be 4 finite numbers, not all zero, got …`; `addBody: a <type> body has no <option>`; `addBody: velocity must be 3 finite numbers, got …`; `addBody: angularVelocity must be 3 finite numbers, got …`; `addBody: a world holds at most 16777216 bodies (the slot field of an id is 24 bits)`.

```js
const ground = world.addBody({ type: 'static', size: [20, 1, 20], position: [0, -0.5, 0] });
const marble = world.addBody({ shape: 'sphere', radius: 0.01, density: 2500, position: [0, 1, 0] });
const player = world.addBody({ shape: 'capsule', radius: 0.3, height: 1.8, mass: 80 });
```

Notes: a size option the shape doesn't have throws rather than being ignored, so
`addBody({ radius: 0.5 })`, which forgot `shape: 'sphere'`, says `a box has no radius` instead of
quietly making a 1 m box. A capsule whose height is exactly twice its radius is a sphere, and is
allowed. A heightfield pushes only up, along its own +y, out of the ground; a mesh pushes from
either side ([Meshes and heightfields](#meshes)). Bodies collide as soon as they are added; see [Contacts](#contacts). A long list that
throws names the first bad entry and where it is (`got NaN at index 2`), or how many numbers it
has (`got 10 numbers`), rather than every number in it.

<a id="world-setbody"></a>
### `world.setBody(body, changes)`

Changes any of the options [`addBody`](#world-addbody) took, with the same names, defaults and
checks, and leaves the rest as they are.

Throws: `setBody: this world was destroyed`; `setBody: body must be a Body, got …`; `setBody: that body belongs to another world`; `setBody: this body was removed`; `setBody: changes must be an object, got …`; `setBody: unknown option '…' (did you mean '…'?)`; `setBody: unknown option '…' (the options are type, shape, size, radius, height, vertices, indices, heights, columns, spacing, parts, density, mass, inertia, centerOfMass, friction, restitution, damping, angularDamping, gravityScale, layers, mask, sensor, lockRotation, maxSlope, stepHeight, position, rotation, velocity, angularVelocity)`; `setBody: type must be 'dynamic', 'static', 'kinematic' or 'character', got …`; `setBody: shape must be 'box', 'sphere', 'capsule', 'hull', 'compound', 'mesh' or 'heightfield', got …`; `setBody: a <shape> has no <option>`; `setBody: type must be 'static' or 'kinematic' for a <shape>, got …`; `setBody: size must be 3 positive numbers, got …`; `setBody: radius must be a positive number, got …`; `setBody: height must be a positive number, got …`; `setBody: height must be at least twice radius (…), got …`; `setBody: vertices must be finite numbers, three to a vertex, got …`; `setBody: indices must be whole numbers below the vertex count (…), three to a triangle, got …`; `setBody: columns must be a whole number, 2 or more, got …`; `setBody: spacing must be a positive number, got …`; `setBody: heights must be finite numbers, rows of columns (…), 2 rows or more, got …`; `setBody: vertices must span a solid (4 points not in one plane), got …`; `setBody: type must be 'dynamic', 'static' or 'kinematic' for a <shape>, got …`; `setBody: parts must be a list of one or more parts, got …`; `setBody: parts[…] must be an object, got …`; `setBody: parts[…]: unknown option '…' (did you mean '…'?)`; `setBody: parts[…]: unknown option '…' (the options are shape, size, radius, height, vertices, position, rotation, density)`; `setBody: parts[…].shape must be 'box', 'sphere', 'capsule' or 'hull', got …`; `setBody: parts[…]: a <shape> has no <option>`; `setBody: parts[…].size must be 3 positive numbers, got …`; `setBody: parts[…].radius must be a positive number, got …`; `setBody: parts[…].height must be a positive number, got …`; `setBody: parts[…].height must be at least twice radius (…), got …`; `setBody: parts[…].vertices must be finite numbers, three to a vertex, got …`; `setBody: parts[…].vertices must span a solid (4 points not in one plane), got …`; `setBody: parts[…].position must be 3 finite numbers, got …`; `setBody: parts[…].rotation must be 4 finite numbers, not all zero, got …`; `setBody: parts[…].density must be a positive number, got …`; `setBody: give mass or density, not both`; `setBody: mass must be a positive number, got …`; `setBody: density must be a positive number, got …`; `setBody: inertia must be 3 positive numbers, or null to follow the shape, got …`; `setBody: centerOfMass must be 3 finite numbers, got …`; `setBody: a character body has no centerOfMass`; `setBody: friction must be 0 or more, got …`; `setBody: restitution must be from 0 to 1, got …`; `setBody: damping must be 0 or more, got …`; `setBody: angularDamping must be 0 or more, got …`; `setBody: gravityScale must be a finite number, got …`; `setBody: layers must be a whole number that fits in 32 bits, got …`; `setBody: mask must be a whole number that fits in 32 bits, got …`; `setBody: sensor must be true or false, got …`; `setBody: lockRotation must be 3 true or false values, got …`; `setBody: maxSlope must be an angle from 0 to π/2, in radians, got …`; `setBody: stepHeight must be 0 or more, or null to follow the size, got …`; `setBody: position must be 3 finite numbers, got …`; `setBody: rotation must be 4 finite numbers, not all zero, got …`; `setBody: a <type> body has no <option>`; `setBody: velocity must be 3 finite numbers, got …`; `setBody: angularVelocity must be 3 finite numbers, got …`.

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
moves `previous` too; see [`world.previous`](#world-previous). A mesh's or heightfield's triangles
are worked out again only when its `vertices`, `indices`, `heights`, `columns` or `spacing`
change; a change of `friction` or `position` leaves them be. A dynamic body made a mesh must be
made static or kinematic in the same call.

<a id="world-bodyof"></a>
### `world.bodyOf(body)` → `object`

A copy of the body's options as they are now, in the form [`addBody`](#world-addbody) takes, so
`world.addBody(world.bodyOf(body))` makes a copy of it. Changing the result changes nothing; pass
it to [`setBody`](#world-setbody) for that.

Returns: `{ type, shape, …sizes, mass or density, inertia, friction, restitution, damping, angularDamping, gravityScale, layers, mask, sensor, lockRotation, maxSlope, stepHeight, position, rotation, velocity, angularVelocity }`.
A character's `maxSlope` always, and its `stepHeight` only if it was given one.
`layers` and `mask` come back unsigned, from 0 to 4294967295, whichever way they were given.
The sizes are the shape's own: `size` for a box, `radius` for a sphere, `radius` and `height` for a
capsule, `vertices` and `indices` for a mesh, as plain arrays, and `heights`, `columns` and
`spacing` for a heightfield. A mesh's `indices` are the triangles it kept, without any that had no
area. `mass` if the body was given one, `density` if not, and `inertia` only if it was given one;
none of the three for a mesh or heightfield. A static body has no `velocity` or `angularVelocity`.

Throws: `bodyOf: this world was destroyed`; `bodyOf: body must be a Body, got …`; `bodyOf: that body belongs to another world`; `bodyOf: this body was removed`.

```js
const twin = world.addBody(world.bodyOf(crate));
console.log(world.bodyOf(crate).position);
```

<a id="world-remove"></a>
### `world.remove(handle)`

Removes a body, with every joint holding it, or a joint. Its handle stops working:
[`body.alive`](#body-alive) or [`joint.alive`](#joint-alive) becomes `false`, and its methods throw.
Removing what is already gone does nothing.

Throws: `remove: this world was destroyed`; `remove: handle must be a Body or a Joint, got …`; `remove: that body belongs to another world`; `remove: that joint belongs to another world`.

```js
world.remove(crate);
```

Notes: the body's slot is given to a later body, under a new id, so an old handle can never reach
the new body. When the bodies left use a quarter of the world's storage or less, the storage
shrinks to half, which replaces [`world.transforms`](#world-transforms). Same as
[`body.destroy()`](#body-destroy) and [`joint.destroy()`](#joint-destroy). What a removed joint
held wakes.

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

<a id="world-addjoint"></a>
### `world.addJoint(options)` → `Joint`

Holds body `a` to body `b`, or to a point in the world, so they move against each other only as
the joint's type allows, and returns its [handle](#joint). See [Joints](#joints).

| Option | Default | Meaning |
|---|---|---|
| `type` | `'ball'` | `'fixed'`: welded, nothing free. `'ball'`: turns any way about its anchor. `'hinge'`: turns about `axis`. `'slider'`: slides along `axis`, turning not at all. `'distance'`: keeps two points `length` apart, or between `min` and `max`. |
| `a` | none; it must be given | The body. Its angle, position and motor are measured against `b`. |
| `b` | `null` | What `a` is held to: another body, or `null` for a point in the world that never moves. |
| `anchor` | midway between their centres; with no `b`, `a`'s centre; a distance joint's, `a`'s centre | Where the joint is, in world space, in metres, where the bodies are now. Bearing works out where that is on each. |
| `anchorB` | `anchor`; a distance joint's, `b`'s centre | Where it is on `b`, or in the world with no `b`, if not the same point: a distance joint's other end. For any other joint, a point it pulls to `anchor`. |
| `axis` | `[0, 1, 0]` | A hinge's axis, a slider's direction, a ball joint's axis for `cone` and its twist. Needn't be unit length. Not for a fixed or distance joint. |
| `min` | none | The least a's free coordinate may be: a hinge's angle or a ball joint's twist about `axis`, in radians, from -π to π, or -Infinity for none; a slider's position along `axis`, in metres; a distance joint's length, in metres, 0 or more, its `length` without a spring and 0 with one. Measured from where the joint is made. `null` goes back to the default. Not for a fixed joint. |
| `max` | none | The most, the same way: up to π, or Infinity for none; a distance joint's, its `length` without a spring. |
| `length` | how far apart the anchors are | A distance joint's length, in metres, 0 or more: what it holds, or springs back to. |
| `cone` | `π` (none) | How far a ball joint's `axis` on `a` may swing from its `axis` on `b`, in radians, from 0 to π. |
| `motorSpeed` | `0` | The speed a hinge's or slider's motor drives `a` at against `b`: rad/s about `axis`, or m/s along it. |
| `motorForce` | `0` (no motor) | The most the motor may push, 0 or more: N·m for a hinge, N for a slider. |
| `friction` | `0` (none) | The most a ball joint, hinge or slider resists `a` turning or sliding against `b` with, 0 or more, as a stiff hinge does: N·m for a ball joint or hinge, N for a slider. A hinge's or slider's acts while its motor is off. Not for a fixed or distance joint. |
| `frequency` | `0` (no spring) | Makes a distance joint a spring, pulling it to `length`: how many times a second it would bounce, in Hz. |
| `dampingRatio` | `1` | How quickly the spring stops bouncing: 1 settles without overshooting, as Bearing's contacts do; 0 bounces for ever. |
| `collide` | `false` | Whether `a` and `b` still collide with each other. |

Returns: a [`Joint`](#joint).

Throws: `addJoint: this world was destroyed`; `addJoint: options must be an object, got …`; `addJoint: unknown option '…' (did you mean '…'?)`; `addJoint: unknown option '…' (the options are type, a, b, anchor, anchorB, axis, min, max, length, cone, motorSpeed, motorForce, friction, frequency, dampingRatio, collide)`; `addJoint: type must be 'fixed', 'ball', 'hinge', 'slider' or 'distance', got …`; `addJoint: a <type> joint has no <option>`; `addJoint: a must be a Body of this world, got …`; `addJoint: a was removed`; `addJoint: b must be a Body of this world, or null, got …`; `addJoint: b was removed`; `addJoint: a and b must be different bodies`; `addJoint: anchor must be 3 finite numbers, got …`; `addJoint: anchorB must be 3 finite numbers, got …`; `addJoint: axis must be 3 finite numbers, not all zero, got …`; `addJoint: length must be 0 or more, got …`; `addJoint: cone must be an angle from 0 to π, in radians, got …`; `addJoint: motorSpeed must be a finite number, got …`; `addJoint: motorForce must be 0 or more, got …`; `addJoint: friction must be 0 or more, got …`; `addJoint: frequency must be 0 or more, got …`; `addJoint: dampingRatio must be 0 or more, got …`; `addJoint: min must be from -π to π, in radians, or -Infinity, got …`; `addJoint: max must be from -π to π, in radians, or Infinity, got …`; `addJoint: min must be a number or -Infinity, got …`; `addJoint: max must be a number or Infinity, got …`; `addJoint: min must be 0 or more, got …`; `addJoint: max must be 0 or more, or Infinity, got …`; `addJoint: min must be at most max (…), got …`; `addJoint: collide must be true or false, got …`; `addJoint: a world holds at most 16777216 joints (the slot field of an id is 24 bits)`.

```js
const frame = world.addBody({ type: 'static', size: [0.1, 2, 0.1], position: [-0.05, 1, 0] });
const door = world.addBody({ size: [1, 2, 0.05], position: [0.5, 1, 0], mass: 20 });
world.addJoint({ type: 'hinge', a: door, b: frame, anchor: [0, 1, 0], axis: [0, 1, 0], min: 0, max: Math.PI / 2 });
const lamp = world.addBody({ shape: 'sphere', radius: 0.2, position: [0, 4, 0] });
world.addJoint({ a: lamp, anchor: [0, 5, 0] });                          // hung from a point, a ball joint
const rope = world.addJoint({ type: 'distance', a: ball, b: crate, min: 0 });   // a rope, as long as they are apart
```

Notes: a joint wakes the bodies it holds. An option for another type throws, as `radius` does on a
box: `a hinge joint has no cone`. A joint between two bodies neither of which is dynamic does
nothing until one is made dynamic.

<a id="world-setjoint"></a>
### `world.setJoint(joint, changes)`

Changes any of the options [`addJoint`](#world-addjoint) took but `a` and `b`, with the same names,
defaults and checks, and leaves the rest as they are.

Throws: `setJoint: this world was destroyed`; `setJoint: joint must be a Joint, got …`; `setJoint: that joint belongs to another world`; `setJoint: this joint was removed`; `setJoint: changes must be an object, got …`; `setJoint: unknown option '…' (did you mean '…'?)`; `setJoint: unknown option '…' (the options are type, a, b, anchor, anchorB, axis, min, max, length, cone, motorSpeed, motorForce, friction, frequency, dampingRatio, collide)`; `setJoint: a and b can't change; remove the joint and add another`; `setJoint: type must be 'fixed', 'ball', 'hinge', 'slider' or 'distance', got …`; `setJoint: a <type> joint has no <option>`; `setJoint: anchor must be 3 finite numbers, got …`; `setJoint: anchorB must be 3 finite numbers, got …`; `setJoint: axis must be 3 finite numbers, not all zero, got …`; `setJoint: length must be 0 or more, got …`; `setJoint: cone must be an angle from 0 to π, in radians, got …`; `setJoint: motorSpeed must be a finite number, got …`; `setJoint: motorForce must be 0 or more, got …`; `setJoint: friction must be 0 or more, got …`; `setJoint: frequency must be 0 or more, got …`; `setJoint: dampingRatio must be 0 or more, got …`; `setJoint: min must be from -π to π, in radians, or -Infinity, got …`; `setJoint: max must be from -π to π, in radians, or Infinity, got …`; `setJoint: min must be a number or -Infinity, got …`; `setJoint: max must be a number or Infinity, got …`; `setJoint: min must be 0 or more, got …`; `setJoint: max must be 0 or more, or Infinity, got …`; `setJoint: min must be at most max (…), got …`; `setJoint: collide must be true or false, got …`.

```js
const hinge = world.addJoint({ type: 'hinge', a: crate, anchor: [2, 1, 0], axis: [0, 0, 1] });
world.setJoint(hinge, { motorSpeed: 2, motorForce: 500 });   // turn it at 2 rad/s, up to 500 N·m
world.setJoint(hinge, { min: -0.5, max: 0.5 });
world.setJoint(hinge, { min: null });                        // no lower limit again
```

Notes: everything follows at once, and the joint's bodies wake. A new `anchor` or `anchorB` is a
world point where the bodies are now; a new `axis` also counts the joint's angle and position
from 0 again from here. A new type starts from its own defaults, as a new shape does: the anchors,
axis and `collide` carry over, and a new distance joint is as long as its ends are apart. A
joint made to let its bodies collide does so from the next step.

<a id="world-jointof"></a>
### `world.jointOf(joint)` → `object`

A copy of the joint's options as they are now, in the form [`addJoint`](#world-addjoint) takes,
so `world.addJoint(world.jointOf(joint))` makes another like it. Changing the result changes
nothing.

Returns: `{ type, a, b, anchor, anchorB, axis, min, max, length, cone, motorSpeed, motorForce, friction, frequency, dampingRatio, collide }`,
each only for the types that have it: `anchor` where `a`'s end is now, in world space, and
`anchorB` for a distance joint, or one given it, where `b`'s is. `min` and `max` only if they were
given. `a` and `b` are [`Body`](#body) handles, `b` `null` for a point in the world.

Throws: `jointOf: this world was destroyed`; `jointOf: joint must be a Joint, got …`; `jointOf: that joint belongs to another world`; `jointOf: this joint was removed`.

```js
const swing = world.addJoint({ type: 'hinge', a: crate, anchor: [2, 1, 0], max: 1 });
console.log(world.jointOf(swing).max);   // 1
```

Notes: a joint's angle and position count from where it was made, so a copy made where its bodies
are now counts from there.

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

<a id="world-contacts"></a>
### `world.contacts(body)` → `Array`

The last step's contact events: which pairs of bodies began touching in it, and which stopped. Read
it after each step, in the same place you step. Given a body, only that body's events, each turned
so that `a` is that body. See [Contact events and sensors](#events).

Returns: an array of fresh plain objects, in the order the step found them (the same order every
run). A begin is `{ type: 'begin', a, b, point, normal, speed }`: `a` and `b` are the two
[`Body`](#body) handles, `point` is where they met and `normal` the unit direction from `a`
towards `b` there, both world-space arrays of 3, and `speed` is how fast they closed along `normal`
when they met, in m/s, 0 or more: for the loudness of a sound, or damage. An end is
`{ type: 'end', a, b }`. `[]` before the first step, and after a step in which nothing changed.

Throws: `contacts: this world was destroyed`; `contacts: body must be a Body, got …`; `contacts: that body belongs to another world`.

```js
world.step(1 / 60);
for (const e of world.contacts()) {
  if (e.type === 'begin' && e.speed > 2) console.log('thud', e.point, e.speed);
}
for (const e of world.contacts(crate)) {
  if (e.type === 'end') console.log('crate let go of', e.b.id);
}
```

Notes: a body removed since is still taken, so its last events can be read: they come with the
step after its removal, and its handle in them reads `alive` as `false`; compare handles by `id`.
The handles in an event can be kept; the arrays are fresh each call.

<a id="world-groundof"></a>
### `world.groundOf(body)` → `{ body, point, normal } | null`

What a character stands on, as of the last step: whether it can jump, what it is standing on, and
how steep that is there. See [Character bodies](#characters).

Returns: `{ body, point, normal }`, or `null` while it is in the air. `body` is the [`Body`](#body)
under it, `point` where it stands on it, and `normal` that ground's unit direction up out of it
there, world-space arrays of 3, fresh copies. `null` for a body that isn't a character.

Throws: `groundOf: this world was destroyed`; `groundOf: body must be a Body, got …`; `groundOf: that body belongs to another world`; `groundOf: this body was removed`.

```js
const hero = world.addBody({ type: 'character', shape: 'capsule', radius: 0.3, height: 1.8, position: [0, 0.9, 0] });
world.step(1 / 60);
const ground = world.groundOf(hero);
if (ground) console.log('standing on', ground.body.id, 'a slope of', Math.acos(ground.normal[1]), 'rad');
```

Notes: standing on the edge of a step, a character's round bottom meets the lip at a slant, but
what it stands on is the tread, so `normal` is the tread's. A character moved by hand has no
ground until the next step.

<a id="world-touching"></a>
### `world.touching(body)` → `Array`

Every body touching this one, as of the last step, or overlapping it where either of the two is a
sensor. What `begin` and `end` events say, asked at any time: "is the player on the ground?",
"what is in this zone?". See [Contact events and sensors](#events).

Returns: an array of [`Body`](#body), in the same order every run; `[]` if none.

Throws: `touching: this world was destroyed`; `touching: body must be a Body, got …`; `touching: that body belongs to another world`; `touching: this body was removed`.

```js
const grounded = world.touching(player).length > 0;
const inside = world.touching(crate).map((b) => b.id);
```

Notes: it costs nothing until called, and then looks only at the bodies near this one, through
the same tree the contacts use, not at every pair in the world. A sleeping body's touches are
kept, and listed. A body moved by hand since the last step is listed as it was at the step.

<a id="world-step"></a>
### `world.step(dt)`

Moves the world on by `dt` seconds, exactly: there is no hidden accumulator, so call it from
your loop's fixed update, once per step. A dynamic body is pulled by gravity, pushed by whatever
it touches, and moves and turns at its velocities, after any forces and torques applied since the
last step have acted; a kinematic body moves and turns as its velocities say; a character goes as
far along its velocity as the world lets it (see [Character bodies](#characters)); a static body
stays put.

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
[Contacts](#contacts)), a world with no joints runs no joint solver, and sleeping bodies cost
nothing at all (see [Sleeping](#sleeping)).

A lopsided body turning freely tumbles as a real one does: a book spun about its middle axis
flips over, and spun about its longest or shortest it doesn't. What a step carries across exactly
is the body's angular momentum, so a tumbling body keeps it to rounding. Its energy can drift
down a little where the real motion keeps it all: measured over 120 s at 60 Hz, a 1 × 2 × 3 m box
flipping at 5 rad/s keeps all of it to within 0.001%, at 20 rad/s 99.98%, and at 50 rad/s 99.4%.
It never gains energy, so it never blows up. A sphere or a cube turns at a constant
angular velocity, exactly.

<a id="world-save"></a>
### `world.save()` → `object`

The whole world as plain data: everything a step reads, so a world loaded from it with
[`World.load`](#world-load) steps exactly as this one would have, to the bit, on any machine. A
replay, a test, a level loaded where it was left, a bug report that reproduces.

Returns: a fresh object of plain numbers, strings, lists and objects, ready for `JSON.stringify`
as it is.

Throws: `save: this world was destroyed`.

```js
const data = world.save();
const text = JSON.stringify(data);          // to a file, or a bug report
const again = World.load(JSON.parse(text)); // steps exactly as world would
```

Notes: everything, not only the bodies as [`bodyOf`](#world-bodyof) gives them: their state in
the step's own frame, the impulses contacts carry from one step to the next, sleep timers and
islands, the broadphase's trees and its pairs in order, joints, characters, and the last step's
events. With less, a loaded pile would start without the impulses holding it and twitch, and
drift from the saved one. So it is large: a heap of 1,000 boxes dropped turned is about 7 MB of
JSON once it has settled and 10 to 11 MB while it lands, and takes a tenth of a second or so to
save or to load (i7-1260P, Node 22). Numbers JSON can't hold, NaN, ±Infinity and -0, are written as strings,
so JSON carries every bit. The data's layout follows Bearing's own, and says which in its
`format`: data saved by another version of Bearing may not load (see
[`World.load`](#world-load)).

<a id="world-load"></a>
### `World.load(data)` → `World`

A world from what [`world.save()`](#world-save) returned, directly or through JSON: every body,
joint and setting, with the same ids, so [`world.body(id)`](#world-body) finds each one, and
stepping exactly as the saved world would have.

Returns: a new [`World`](#world), its own: changing it changes nothing in the world it was saved
from, or the data.

Throws: `load: data must be what world.save() returned, got …`; `load: data is format …; this Bearing reads format 7`; `load: data is damaged: …`.

```js
const checkpoint = world.save();
// ... later, back to where it was:
const world2 = World.load(checkpoint);
```

Notes: `format` is the layout of the data, which changes when Bearing's stored state does; data
of another format throws rather than load as something subtly different. Handles from the saved
world belong to it: take the loaded world's own from `world.body(id)` with the same ids.

<a id="world-debuglines"></a>
### `world.debugLines(options)` → `Float32Array`

The world as line segments, for any engine to draw in one loop: nine numbers a segment, its two
ends `x y z` and `x y z`, then its colour, `r g b` from 0 to 1. Bearing draws nothing; this is the
data.

| Option | Default | Meaning |
|---|---|---|
| `shapes` | `true` | Every body's collision shape: a box's 12 edges, a ball's 3 rings, a capsule's rings and sides, a hull's edges, a compound's parts, a mesh's or heightfield's triangles. Awake bodies blue, asleep dimmer, static grey, kinematic purple, characters green, sensors yellow. |
| `contacts` | `true` | The last step's contact points, each a red line out along its normal (from `a` towards `b`) a quarter of the smaller body's thinnest size long. |
| `joints` | `true` | Each joint, orange: a line from each body's centre to its anchor on that body. |
| `boxes` | `false` | The boxes the broadphase files each body in, which it is filed again on leaving. |

Returns: a `Float32Array`, a view of the world's own array, which the next call replaces: copy it
to keep it.

Throws: `debugLines: this world was destroyed`; `debugLines: options must be an object, got …`; `debugLines: unknown option '…' (did you mean '…'?)`; `debugLines: unknown option '…' (the options are shapes, contacts, joints, boxes)`; `debugLines: <option> must be true or false, got …`.

```js
const lines = world.debugLines({ contacts: false });
for (let i = 0; i < lines.length; i += 9) {
  const from = [lines[i], lines[i + 1], lines[i + 2]], to = [lines[i + 3], lines[i + 4], lines[i + 5]];
  const colour = [lines[i + 6], lines[i + 7], lines[i + 8]];
  // Your engine's own line: Winding's engine.debug.line(from, to, colour), say.
}
```

Notes: shapes are drawn where the step works with them: a hull or compound about its centre of
mass, as it collides. Contacts are where the last step found them, at its start. It costs nothing
until called, and a call allocates only when the world has more to draw than the array has held.
A big mesh or heightfield is many segments: three for each of its triangles.

<a id="world-destroy"></a>
### `world.destroy()`

Lets go of every body and all of the world's storage, and ends its [workers](#world-workers).
Afterwards every call on the world throws, naming the call, and every body's
[`alive`](#body-alive) is `false`. Calling it again does nothing.

```js
world.destroy();
```

Notes: a world starts nothing else that keeps running, no timers and no listeners, so there is
nothing else to stop. A world dropped without it ends its workers when it is garbage collected.

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
static body never is, and nor is a frozen one, stopped while the rest of its island moves. Read
only: a body is woken by whatever disturbs it, never by hand. See [Sleeping](#sleeping).

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

Notes: a static, kinematic or character body acts as if it were infinitely heavy. Its mass is
kept, and used if it is made dynamic.

<a id="body-inertia"></a>
### `body.inertia` → `[x, y, z]`

The body's principal moments of inertia, in kg·m², as a frozen copy: how hard it is to turn about
each of three axes at right angles. For a box, ball or capsule, or a body given `inertia`, those
are its own x, y and z axes; for a hull or compound, its principal axes (see
[Hulls and compounds](#hulls)). Worked out from the shape and mass, or the `inertia` it was given.
Read only; change it with [`setBody`](#world-setbody).

Throws: `inertia: this world was destroyed`; `inertia: this body was removed`; `body.inertia is read only; use setBody(body, { inertia })`.

```js
const [ix, iy, iz] = crate.inertia;   // 166.7 each, for a 1 m box of water
```

Notes: a box w × h × d of mass m has m (h² + d²) / 12 about x, and the same pattern about y and z;
a sphere has 2/5 m r² about every axis; a capsule, a cylinder with two hemispheres, has the sum of
its parts about its centre.

<a id="body-position"></a>
### `body.position` → `[x, y, z]`

Where the body's origin is, in metres, as a frozen copy: a box's, ball's or capsule's centre, and
the point a hull's points and a compound's parts are given from. Where its mass is centred is
[`centerOfMass`](#body-centerofmass). Read only; move it with [`setPosition`](#body-setposition).

Throws: `position: this world was destroyed`; `position: this body was removed`; `body.position is read only; use setPosition(x, y, z)`.

```js
if (ball.position[1] < -100) ball.setPosition(0, 5, 0);
```

Notes: frozen, so `ball.position[1] = 5` throws in a module rather than changing a copy for
nothing. For many bodies at once, read [`world.transforms`](#world-transforms).

<a id="body-centerofmass"></a>
### `body.centerOfMass` → `[x, y, z]`

Where the body's mass is centred, in the world, in metres, as a frozen copy: the point it turns
about, and the point its `velocity` is the velocity of. The same as `position` for a box, ball or
capsule unless it was given a `centerOfMass`. Read only; move it with
[`setBody`](#world-setbody).

Throws: `centerOfMass: this world was destroyed`; `centerOfMass: this body was removed`; `body.centerOfMass is read only; use setBody(body, { centerOfMass })`.

```js
const hammer = world.addBody({
  shape: 'compound',
  parts: [{ shape: 'capsule', radius: 0.02, height: 0.3 }, { size: [0.12, 0.04, 0.04], position: [0, 0.16, 0], density: 7800 }],
});
console.log(hammer.centerOfMass);   // [0, 0.129, 0]: most of its mass is in the head
```

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

Notes: about an axis the body is locked about ([`lockRotation`](#world-addbody)), its spin stays 0:
only the free axes are set.

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

Notes: as [`applyForce`](#body-applyforce): one step, adding up, nothing for a static, kinematic or
character body. A body's [`lockRotation`](#world-addbody) axes take nothing.

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
straight away. Nothing for a static, kinematic or character body.

<a id="body-applyangularimpulse"></a>
### `body.applyAngularImpulse(impulse)` → `Body`

Changes the body's angular velocity at once, by `impulse` in newton-metre-seconds about each
world axis, through the body's inertia.

Throws: `applyAngularImpulse: this world was destroyed`; `applyAngularImpulse: this body was removed`; `applyAngularImpulse: impulse must be 3 finite numbers, got …`.

```js
ball.applyAngularImpulse([0, 50, 0]);   // set it spinning about Y
```

Notes: nothing for a static, kinematic or character body, nor about an axis the body is locked
about ([`lockRotation`](#world-addbody)).

<a id="body-destroy"></a>
### `body.destroy()`

Removes the body from its world. Same as [`world.remove(body)`](#world-remove); does nothing if
the body is already gone.

Throws: `destroy: this world was destroyed`.

```js
crate.destroy();
```

<a id="joint"></a>
## Joint

A Joint is a handle to one joint in a world, as a [`Body`](#body) is to a body: `joint.world` is
its world and `joint.id` its id, and nothing else. Compare `a.id === b.id`, not `a === b`.
[`world.addJoint`](#world-addjoint) makes one. Change it with [`world.setJoint`](#world-setjoint),
read it with [`world.jointOf`](#world-jointof).

<a id="joint-id"></a>
### `joint.id` → `number`

The joint's id: a whole number, unique to this joint for the life of its world. Joints count their
own ids, apart from bodies'. Read only.

Throws: `joint.id is read only`.

```js
const hinge = world.addJoint({ type: 'hinge', a: crate });
const names = new Map([[hinge.id, 'lid']]);
```

<a id="joint-world"></a>
### `joint.world` → `World`

The world the joint is in. Read only.

Throws: `joint.world is read only`.

```js
const tie = world.addJoint({ a: ball, b: crate });
tie.world.remove(tie);
```

<a id="joint-alive"></a>
### `joint.alive` → `boolean`

`false` once the joint has been removed, by itself or with either of its bodies, or its world
destroyed. Never throws. Read only.

Throws: `joint.alive is read only`.

```js
const tie = world.addJoint({ a: ball, b: crate });
world.remove(crate);
console.log(tie.alive);   // false: a joint goes with its bodies
```

<a id="joint-destroy"></a>
### `joint.destroy()`

Removes the joint from its world, and wakes what it held. Same as
[`world.remove(joint)`](#world-remove); does nothing if the joint is already gone.

Throws: `destroy: this world was destroyed`.

```js
const tie = world.addJoint({ a: ball, b: crate });
tie.destroy();
```

<a id="hulls"></a>
## Hulls and compounds

Shapes that aren't a box, a ball or a capsule, for bodies that move: rocks, crates with bevelled
edges, wedges, car bodies, and tools, furniture and anything else made of several pieces.

```js
// A rock: any points; the hull keeps the corners of the convex solid round them.
const rock = world.addBody({ shape: 'hull', vertices: [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0.6, 0.6, 0.6], position: [0, 3, 0] });
// A hammer: a wooden handle and a steel head, as one rigid body.
const hammer = world.addBody({
  shape: 'compound',
  parts: [
    { shape: 'capsule', radius: 0.02, height: 0.3, density: 700 },
    { shape: 'box', size: [0.12, 0.04, 0.04], position: [0, 0.16, 0], density: 7800 },
  ],
  position: [2, 1, 0],
});
```

- **A hull is the skin round its points.** Give any points, three numbers each, in the body's own
  frame: a model's vertices, say, from [`readGlb`](#readglb). The hull is the smallest convex solid
  that holds them all, what a sheet of rubber stretched round them would make, found by quickhull
  (Barber, Dobkin and Huhdanpaa, 1996). Points inside it, or on its edges and faces, are dropped,
  and triangles lying in one plane are merged, so a cube's 8 corners make 6 square faces, and a
  cube made as a hull stacks exactly as a box does. [`bodyOf`](#world-bodyof) gives back the
  corners it kept.
- **A hull is convex.** An L, a bowl or a cup comes out as its hull, filled in. Make it a compound
  of convex parts instead.
- **Its points must span a solid**: four or more, not all in one plane, or it throws. A flat hull
  has no volume, so no mass.
- **Building one costs once.** It happens in `addBody`, and in `setBody` when `vertices` change,
  never in a step: 200 points take about 3 ms on an i7-1260P. A step's cost grows with a hull's
  corners and faces: a pile of 60 hulls of about 10 corners each cost 1.5 times the same pile of
  boxes.
- **A compound is several shapes as one body.** Its `parts` are boxes, spheres, capsules and hulls,
  each placed from the body's origin. They move as one, each collides as a shape of its own, and
  they never collide with each other. Each part's `density` is its own, or the body's; a `mass`
  given to the body scales them all to it. A part can't be a mesh, a heightfield or another
  compound.
- **The centre of mass is worked out, and the body turns about it.** A hull's from its volume, a
  compound's from its parts' masses. A thrown hammer turns about a point near its head, and its
  handle swings round it. `position` and [`world.transforms`](#world-transforms) stay at the
  body's origin, where its points and parts are measured from, so a renderer draws a model where
  its own file puts it; [`centerOfMass`](#body-centerofmass) says where its mass is.
- **Its inertia is about its principal axes.** A lopsided shape's inertia isn't three numbers
  about its own axes, but it is about three others at right angles, its principal axes, the ones
  it spins about steadily. Bearing finds them when the body is made (Jacobi's method, from square
  roots only, so the same on every machine), and [`inertia`](#body-inertia) gives the moments
  about them.
- **Velocity is the centre of mass's.** `velocity` and [`setVelocity`](#body-setvelocity) are how
  fast the centre of mass moves, as in Rapier and Jolt, and a force or impulse with no `point` acts
  there, so it never spins the body.
- **`centerOfMass` moves it,** on any body that isn't level geometry or a character: a car body's,
  lowered so it doesn't roll over on a turn. The inertia stays the shape's, about its own centre of
  mass, unless `inertia` is given too. A character's casts take its shape from its centre, so it
  can't have one.
- **Hulls collide by separating axes**, as boxes do: each face's direction, and the pairs of edges
  that can meet (Gregorius, 2013), then the face most against the other clipped to its sides for
  up to four points. A hull made of a box's corners gives the box's contacts, within the contact
  tolerance.
- **A compound's parts are tried in pairs.** Each part of one body against each part of the other,
  where their reaches overlap: fine for a hammer or a chair, and a body of dozens of parts against
  another costs the two counts multiplied.
- **Not for characters or queries.** A character can't be a hull or compound: its casts and its
  feet are worked out for a box, ball or capsule. A query's shape is a box, ball or capsule too:
  a hull is built from its points by quickhull, too dear to do for each query. Queries see hulls
  and compounds as bodies like any other.

<a id="meshes"></a>
## Meshes and heightfields

Level geometry: the ground, walls, ramps and terrain a game is played on, as triangles.

```js
// A ramp, as a mesh: four corners, two triangles.
const ramp = world.addBody({
  shape: 'mesh',
  vertices: [0, 0, 0, 4, 2, 0, 4, 2, 3, 0, 0, 3],
  indices: [0, 1, 2, 0, 2, 3],
});
// Hills, as a heightfield: 3 by 3 heights, 10 m apart, centred on its position.
const hills = world.addBody({ shape: 'heightfield', heights: [0, 1, 0, 1, 3, 1, 0, 1, 0], columns: 3, spacing: 10 });
```

- **A mesh is any triangles.** `vertices` are numbers, three to a corner, in metres, in the body's
  own frame; `indices` are three to a triangle, each the number of a corner. Models from a `.glb`
  file come ready to use from [`readGlb`](#readglb). Triangles with no area are left out: nothing
  could touch them.
- **A heightfield is a grid of heights**, cheaper to keep than the same ground as a mesh: one number
  a sample instead of a triangle's worth. `heights` runs row by row, `columns` to a row; columns go
  along the body's x, rows along its z, `spacing` metres apart, and the grid is centred on the
  body's position. Each square of four samples is cut corner to corner into two triangles, from
  the sample at its lowest x and z to the one at its highest.
- **Static, or kinematic, never dynamic.** Triangles enclose nothing, so they have no mass to fall
  with or turn. A mesh or heightfield is static unless given `type: 'kinematic'`, which makes a
  moving platform of any shape; `'dynamic'` throws. Its `mass` is 0. A falling object of any
  shape will come with compound shapes, in a later version.
- **A mesh's triangles have two sides.** A body is pushed out on whichever side it is, so a wall
  needs no thickness, and the order of a triangle's corners doesn't matter.
- **A heightfield has one side: it is ground.** Everything under its surface is in the ground, and
  is pushed up out of it, along the face of the triangle it is under, however deep it has gone.
  Up is the body's own +y, so a heightfield turned over (`rotation: [1, 0, 0, 0]`) is a ceiling,
  pushing down. A two-sided heightfield pushed a body pressed in past its centre down through the
  ground instead.
- **The seams between triangles can't be caught on.** Where two triangles meet flat, or fold
  inward, nothing could touch the edge between them without touching a face first, so a contact
  pointing off that edge is left out, and the face beside holds the body instead. A corner pressed
  into a valley, under its seam, where neither face is over it, is held by both faces. A box slides
  across a floor of 800 triangles on exactly the path it takes across one big box, and a ball
  rolls over a heightfield without bumping at its seams. Where triangles fold outward, the edge is
  a real corner, and a ball rolls over it as over any edge. Two triangles count as flat to within a
  part in about a million of their size, sixteen times what a model file rounds to.
- **Only the triangles under a body are looked at.** A mesh files its triangles in a tree, as the
  world files its bodies, and a heightfield finds the squares under a body from where it is; a box
  on a floor of 100,000 triangles tests about 17 a step. A 100,000-triangle mesh takes about
  0.2 s to add (i7-1260P, Node 22), once.
- **Contacts and queries work on them as on any body.** Rays, shape casts and overlap tests see
  their triangles; see [Queries](#queries). A mesh takes the contact tolerance of whatever touches
  it, since it has no thickness of its own.

<a id="readglb"></a>
### `readGlb(bytes, options)` → `{ vertices, indices }`

Reads every triangle of a `.glb` file's scene, each node placed where the scene puts it, ready for
`addBody({ shape: 'mesh', ... })`. A `.glb` is glTF 2.0 in one binary file, which every 3D package
exports; Bearing reads it itself, with no dependency.

| Option | Default | Meaning |
|---|---|---|
| `node` | every node in the scene | The name of one node: only it and the nodes under it are read, still placed as in the scene. The first node of that name, if several share it. |

Returns: `{ vertices, indices }`: a `Float64Array` of corner positions, three numbers each, in
metres, with every node's translation, rotation, scale or matrix applied; and a `Uint32Array` of
corner numbers, three to a triangle. Fresh arrays, yours to keep.

Throws: `readGlb: options must be an object, got …`; `readGlb: unknown option '…' (did you mean '…'?)`; `readGlb: unknown option '…' (the options are node)`; `readGlb: node must be a string, got …`; `readGlb: node must be the name of a node in the file, got …`; `readGlb: bytes must be an ArrayBuffer or a typed array, got …`; `readGlb: bytes must be a binary glTF 2.0 file (.glb), got …`; `readGlb: the file needs …, which Bearing doesn't read; export it without that`; `readGlb: the file is damaged: …`.

```js
// levelBytes: the file, from fetch('level.glb') then .arrayBuffer() in a page, or
// readFileSync('level.glb') in Node.
const level = world.addBody({ shape: 'mesh', ...readGlb(levelBytes) });
const stairs = world.addBody({ shape: 'mesh', ...readGlb(levelBytes, { node: 'stairs' }), position: [0, 0, 20] });
```

Notes: `bytes` is the file itself, not a URL: fetching or reading it is yours to do, as Bearing
does no input or output of its own. Triangles, triangle strips and triangle fans are read; points
and lines have no triangles and are left out, as are materials, textures and animation, which say
nothing about shape. Geometry compressed with Draco (`KHR_draco_mesh_compression`) or meshopt
(`EXT_meshopt_compression`), or quantized (`KHR_mesh_quantization`), would need a decoder, and
throws, naming the extension: export the model without it. glTF's units are metres and its y is
up, as Bearing's are, so a model lines up with what any engine draws from the same file.

<a id="contacts"></a>
## Contacts

Bodies collide as soon as they are added: there is nothing to switch on, and nothing to tune.
Every step finds which bodies touch, or will within the step, and pushes them so they don't sink
into each other, slide no more than friction allows, and bounce as much as their restitution says.

- **Who pushes whom.** Dynamic bodies push each other. A kinematic body pushes dynamic ones and is
  never pushed back, so a moving platform carries what stands on it. Static and kinematic bodies
  don't collide with each other, since neither would move. A character finds its own way, by
  casting through the world, and stands like a wall to what runs into it; see
  [Character bodies](#characters).
- **Materials mix.** Two bodies' frictions combine as their geometric mean, √(a · b), so ice is
  slippery against anything; their restitutions as the larger, so a bouncy ball bounces off
  anything.
- **Friction is a cone.** A contact resists sliding in every direction alike, up to its friction
  times how hard it is pressed, so a box pushed diagonally slows exactly as one pushed straight.
  It is worked out once for the whole contact, at the centre of its points, and turning about the
  normal is resisted up to friction times each point's push times its distance from that centre,
  as Box3D and Jolt do: a box resting flat holds and slides as it would with friction at each
  corner, for a third of the work.
- **Friction holds where it holds.** While a contact isn't sliding, friction also pulls back any
  slip, the way an overlap is pushed out, so a heap that has settled stays put instead of creeping
  a few millimetres a second and staying awake. A contact sliding anywhere, or no longer pressed,
  lets go of its slip and starts afresh. A rolling ball doesn't slip, so it rolls as before.
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
- **Fast bodies meet where they would meet.** A pair not yet touching that moves against itself by
  more than its tolerance in the step is first asked when, within the step, its shapes would touch,
  following each body's velocity, gravity and spin. If they wouldn't, it gets no contact, so a ball
  passing a box's corner with room to spare, or flying through a gap wider than itself, keeps its
  velocity exactly. If they would, its contact is made where both bodies will be at that moment,
  with the real normal and points of impact, and the solver lets the pair close to exactly there.
  Nothing needs flagging: a 1 cm ball at 500 m/s stops on a 1 cm wall, at any angle, and on a mesh,
  a heightfield or a thin dynamic box; a 10 cm rod at 200 m/s spinning at up to 80 rad/s stops on
  a 2 cm wall. Pairs at rest, or moving slowly against each other, never ask, and cost nothing
  more. Against a mesh or heightfield the contact is still made where the step starts, so a fast
  body skimming past a mesh's edge can still be touched by it.
- **Nothing fast goes through.** A body struck hard off its centre, a rod meeting a wall corner-first
  at 200 m/s, needs some 3,000 rad/s to stop the corner, and turns through the wall within a step
  where its contacts don't reach. So after the solve, each body whose centre moved more than half
  its thinnest size in the step sweeps its core, a ball at its centre a quarter of that size
  across, from where the step started it to where it ended, against everything it collides with as
  it is at the end of the step but other fast bodies (Box2D 3's continuous step, with dynamic
  bodies included). If the core meets something, the body goes back to where it did, turned that
  far, and its velocity into the surface is returned as restitution would return it: it loses the
  rest of that step's motion, rather than going through. Measured: rods, 1 cm cubes and planks
  turned any way, spinning or not, at 50 to 500 m/s, none through a 1 to 5 cm wall, a mesh, a
  heightfield or a thin steel plate floating free, where turned rods went through a 1 cm wall 57
  times in 100. A body whose centre moves less than that never sweeps, and costs nothing more.
- **Spin is capped at half a turn a step, but for a sphere.** A body's contacts are found once a
  step, and a body turning more than half a turn in one would meet the world on sides they never
  saw: a 10 cm rod spun at 400 rad/s and dropped on another came off it at 9,600 rad/s before the
  cap. So a body other than a sphere turns at most π radians a step, 188 rad/s at 60 Hz, and more
  with smaller steps; Box2D and Jolt cap at a quarter turn. A sphere's contacts don't depend on how
  it spins, so it spins as fast as it is made to.
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

Notes: one case the solver can't settle in a step is a very light body wedged between two much
heavier ones meeting it at an angle: a 1 cm marble 225,000 times lighter than what it is caught
between, pressed by two contacts 20° apart, can be squeezed into one of them before the solver
works it out. It is pushed back out over the next steps, at the tolerance a substep. Struck
square-on, or by bodies nearer its own weight, it isn't.

A contact's patch also holds things a point contact wouldn't. A ball rests on a slope
gentler than its rolling resistance, 0.57° (tan θ = 0.01), and rolls down a steeper one. A box
standing on an edge stays up while its centre is within its tolerance of being over the edge: a
1 m cube 0.41° past balance stands, and 0.45° past falls. Friction 0 leaves no friction across a
patch, so a ball spinning in place on ice spins on, though its rolling still slows.

<a id="characters"></a>
## Character bodies

A person, a creature, a vehicle driven like one: a body the game moves where it wants, that still
collides with the world as something walking does. Physics engines call this a character
controller; it reads no input. The game gives it a velocity, as it would a kinematic body, and
Bearing works out where it can actually go.

```js
const hero = world.addBody({ type: 'character', shape: 'capsule', radius: 0.3, height: 1.8, position: [0, 0.9, 0] });

// Each fixed step: the velocity the game wants, keeping the one it falls or rises at.
const ground = world.groundOf(hero);
const jump = false;   // the game's own decision
hero.setVelocity(2, ground && jump ? 5 : hero.velocity[1], 0);
world.step(1 / 60);
```

Neither of the other moving types makes a good person. A dynamic body slides down slopes under
friction, tips over, bounces off the edge of every stair and is launched by ramps. A kinematic one
walks through walls. A character:

- **Slides along walls.** It keeps the part of its motion along whatever it meets: walking into a
  wall at 30° to it, it goes on along the wall at cos 30° of its speed.
- **Walks up slopes up to `maxSlope`,** at the speed it is given across the ground, so a hill
  doesn't slow it. Ground steeper than that is a wall it can't walk up, and dropped onto it, it
  slides down.
- **Steps up onto anything up to `stepHeight`** that blocks its feet: stairs, kerbs, the edge of a
  platform. Taller is a wall.
- **Stays on the ground going down** slopes and stairs, rather than stepping off into the air.
- **Falls under gravity in the air, and lands.** On the ground the ground holds it, and
  [`groundOf`](#world-groundof) says what it stands on.
- **Rides what it stands on,** moving with a platform, a lift or a crate as they move, then walking
  on them; a rotating platform carries it round its centre without turning which way it faces.
- **Up is against gravity,** so its slopes and steps stay right if gravity changes, and with none,
  it is +y.

Its velocity, read after a step, is what it did of its own accord: walking into a wall drops the
part into the wall, landing drops the fall, and on the ground its vertical speed is 0. So a game
sets the speed it walks at, keeps `velocity[1]`, and sets it to jump.

- **What blocks it:** everything it would collide with, by its own layers and mask, dynamic bodies
  and other characters included, but not a body a joint holds to it (see [Joints](#joints)). It doesn't push a crate it walks up to, and is never pushed: what
  lands on it or runs into it is held, as by a wall. Pushing things is the game's to decide, with
  [`applyImpulse`](#body-applyimpulse). It carries nothing on its head as it walks. Sensors see it.
- **How it moves:** before each step, by casting its shape through the world, as
  [`shapeCast`](#world-shapecast) does, and stopping where that says; then it stands there through
  the step. A few casts a step for each character: built for tens or hundreds of them, not
  thousands. It doesn't sleep, so whatever it stands on can always carry it.
- **Its shape:** any of a box, a sphere or a capsule. A capsule is best: its round bottom rides up
  small edges, and its round sides slide along corners.

Notes: forces and impulses don't move a character, as they don't a kinematic body. Its rotation is
the game's: it turns only as its angular velocity or [`setRotation`](#body-setrotation) say.

**Rotation locks.** [`lockRotation`](#world-addbody), on any body, names the world axes it never
turns about: `[true, false, true]` keeps a dynamic capsule upright, still free to turn about y to
face a way, for a character moved by forces; `[true, true, false]` lets a 2.5D game's bodies turn
only in the screen's plane. A locked axis is one nothing turns it about, as if its inertia there
were endless: contacts, torques and impulses can't, and its spin about it stays 0. Bodies built in
one plane, locked to turn only in it, push each other only within it, so they stay in it, to the
bit, and there is no position lock to need.

<a id="joints"></a>
## Joints

A joint holds two bodies together, or one body to a point in the world, so that they can move
against each other only in some ways: a door turns on its hinges, a drawer slides in and out, a
chain's links swing on each other.

```js
const post = world.addBody({ type: 'static', size: [0.2, 3, 0.2], position: [0, 1.5, 0] });
const sign = world.addBody({ size: [1, 0.5, 0.05], position: [0.6, 2.5, 0] });
world.addJoint({ type: 'hinge', a: sign, b: post, anchor: [0.1, 2.75, 0], axis: [1, 0, 0] });
```

A free body can move in six ways, three along and three about the axes: its degrees of freedom.
A joint takes some away, and leaves the rest:

| `type` | What it leaves free | Its free coordinate, what `min` and `max` limit |
|---|---|---|
| `'fixed'` | Nothing: the two are welded. | None |
| `'ball'` | Any turn about the anchor. | Its twist about `axis`; and `cone`, how far `axis` may swing |
| `'hinge'` | Turning about `axis`. | Its angle about `axis` |
| `'slider'` | Moving along `axis`. | Its position along `axis` |
| `'distance'` | Everything but the gap between its two anchors. | The gap's length |

- **`a` moves against `b`.** `a` is the body, and `b` what it is held to, a point in the world if
  left out. A joint's angle, position, limits and motor are `a`'s against `b`: a hinge's angle is
  how far `a` has turned about the axis, positive the way your right hand's fingers curl with the
  thumb along it, and a slider's position how far `a`'s anchor has moved along it from `b`'s. Both
  count from 0 where the joint is made.
- **Where, as in the scene.** `anchor` and `axis` are world space, where the bodies are when the
  joint is made or changed; Bearing works out where that is on each body.
- **Chains, ragdolls and bridges are held exactly.** Joints that join bodies into trees (a chain,
  a ragdoll, a bridge of planks, fixed at one end or both) are solved all together, exactly, every
  pass (Baraff's method), rather than one at a time, so a weight on a long chain doesn't stretch
  it: a chain of 100 links hangs as long as it was built from its first step, at any size, and a
  weight 100 times a link's mass on a chain of 10 stretches it by less than a hundredth of a
  millimetre. A joint that closes a loop among moving bodies, as a ladder's second rail does, is
  solved one at a time, as contacts are, and can give a little under load.
- **Limits are stops.** `min` and `max` are hard stops, as a door frame is: met at speed, a joint
  stops at its stop, rather than passing it and being pulled back. A 1 m door swung shut at 50 m/s
  at its edge stopped within 0.002 mm of its stop. A ball joint's `cone` stops its axis swinging
  further from `b`'s than that angle.
- **Motors drive a speed.** A hinge's or slider's motor pushes `a` toward `motorSpeed` against `b`,
  as hard as `motorForce` allows and no harder, so a motor too weak for its load stalls, or is
  driven backwards, pushing as hard as it can. Turned off (`motorForce: 0`), it pushes no more.
- **Friction stops a joint swinging.** `friction` resists `a` turning (or sliding) against `b`,
  up to that torque (or force) and no more, as a stiff hinge does: a joint loaded less than that
  holds still, and one loaded more turns, slowed. A hinge's or slider's is its motor held at speed
  0, so `friction: F` acts exactly as `motorSpeed: 0, motorForce: F` would, and a motor that is on
  already resists up to its own force. A ball joint's resists turning every way alike, however the
  joint is turned. 32 ragdolls with 1 N·m on every joint all sleep within 6 s; with none, most of
  them still swing at 12 s. None by default, as in most engines.
- **A distance joint is a rod, a rope or a spring.** With no limits and no spring it is a rod, its
  `length` both ways. `min: 0` makes a rope, as long as `length` and no longer, slack shorter.
  `frequency` makes a spring that pulls it to `length`, free to stretch either way unless limited,
  bouncing at that frequency and settling as `dampingRatio` says.
- **Joined bodies don't collide with each other,** unless the joint's `collide` says they do: a
  ragdoll's limbs overlap at their joints, and would fight them. A character doesn't walk into
  what is joined to it.
- **A joint goes with its bodies.** Removing either body removes the joint, and its handle reads
  `alive: false`.
- **Joined bodies sleep and wake together.** A chain sleeps as one, and wakes as one when anything
  touches it. A body hanging from a kinematic body or a character is woken, and carried, when that
  moves.
- **What can't move is held fixed.** A joint holds a static, kinematic or character body as a
  point in the world is held: nothing it does moves them. A joint between two bodies neither of
  which is dynamic does nothing. A body's [`lockRotation`](#world-addbody) holds in a joint too.
- **No joints, no cost.** A world without joints runs no joint solver, and a sleeping chain's
  joints cost nothing.

Notes: a joint gives by a millionth of what it pushes, so that solving a tree leaves first never
divides by nothing where a body can't move along some row: far too little to see. A
joint turning very fast opens a little within a step and closes again: a chain's end whipped to
hundreds of radians a second opened 7.5 mm for a moment. A body hanging from a dangling joint with
nothing to rub on swings for as long as a pendulum would: to settle, give its joint some
`friction`, or the body some [`angularDamping`](#world-addbody).

<a id="events"></a>
## Contact events and sensors

Games need to know when things touch: a crate lands and a thud plays, a player walks into a coin,
something enters a goal. After each step, [`world.contacts()`](#world-contacts) lists the pairs
that began touching in it and the pairs that stopped, and [`world.touching(body)`](#world-touching)
says what touches a body now. Sensors are bodies that notice what overlaps them and push nothing.

```js
const coin = world.addBody({ type: 'static', shape: 'sphere', radius: 0.4, sensor: true, position: [3, 1, 0] });
world.step(1 / 60);
for (const e of world.contacts(coin)) {
  if (e.type === 'begin' && e.b.id === player.id) world.remove(coin);
}
```

- **Read after the step, never called back.** A function called from inside the solver at the
  moment two bodies touch couldn't safely remove one of them: it would pull the body out of arrays
  the solver is halfway through, so engines that call back forbid changing anything inside the
  call. Read after the step, the step is over, and anything can change.
- **What touching means.** A pair touched in a step if the solver pushed them apart in it, or the
  step left them within a contact's tolerance of touching (a pixel; see [Contacts](#contacts)).
  The push catches a body that hits another and is knocked away within the same step; the distance,
  a box resting against a wall with nothing pressing it.
- **Begin and end, once each.** A pair's `begin` comes in the step it starts touching, and its
  `end` in the step it stops, and never two begins without an end between. A body against a mesh
  or heightfield is one pair, however many of its triangles it touches.
- **No stay events.** A pair still touching writes nothing. A stay event a step for every touching
  pair would be 30,000 a step for a resting pile of 10,000 boxes, and sleeping bodies, whose
  contacts nothing looks at, would have to be walked to give them. Ask
  [`touching()`](#world-touching) instead, when you want to know.
- **Removed bodies end their touches.** Their `end` events come with the next step, the removed
  body's handle dead. A body added already touching something begins touching it at the next
  step, and a slot reused in the same place ends the old body's touches and begins the new one's.
- **Sleeping keeps touches.** Bodies falling asleep while touching write nothing, nor do they when
  woken, unless they then move apart.
- **A touch the step couldn't foresee comes a step late.** Contacts are found from how bodies are
  moving when the step starts. A resting body knocked into another in the middle of the step meets
  it unforeseen, as does a spinning box whose contact held one corner while another swung down, and
  that touch begins at the next step, 1/60 s later at 60 Hz. In a scene of bouncing and knocked
  bodies, 12 of 4,138 pair-steps were.
- **Where a contact curves, the line is a little soft.** The step measures a pair's gap along the
  normal its contact had when the step began. Two balls (or capsules) sliding past each other by d
  in a step, or a ball rolling over a box's edge, are measured off by up to about d² over their
  size: 0.3 mm for two 40 cm balls passing at 0.9 m/s. Against a flat face the normal holds, and
  there is no such error.
- **Events cost nothing when nothing changes.** The step writes an event only when a pair starts or
  stops touching, into storage it reuses, and builds objects only when you call `contacts()`.
  Measured on a settled pile of 1,000 boxes kept awake, and while it landed, steps cost the same as
  before events, within 2% either way, run alternately in one process.

**Sensors.** Any body is a sensor with `sensor: true`: any shape, any type. A static sensor is a
zone; a kinematic one follows something, as a trigger around a character does.

- **They push nothing, and nothing pushes them.** Bodies pass through a sensor exactly as if it
  weren't there: their paths are the same to the bit. A dynamic sensor falls through everything.
- **They see every body that isn't a sensor.** Dynamic and kinematic bodies always, so a zone sees
  a moving platform or a character pass; static ones too, unless the sensor is static as well,
  since then neither can come or go. Two sensors never see each other: rarely wanted, and it
  costs pairs (Box2D 3's rule). Layers and masks filter them as any body.
- **They wake nothing.** A sensor sweeping through a sleeping pile reports each body without waking
  it, and a sleeping body inside a zone stays touching it.
- **Queries don't see them.** A ray, a shape cast or an overlap test passes through a sensor: a
  bullet's ray shouldn't stop at an invisible trigger zone. What is in a zone is for events and
  [`touching()`](#world-touching).
- **They look where a step leaves bodies.** A sensor's pairs are checked once a step is over, so a
  body that passes all the way through a sensor within one step isn't seen; continuous collision,
  in a later version, will. A mesh or heightfield sensor sees what touches its triangles, not what
  is inside it, since triangles are a surface; two meshes never meet.

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

- **Islands sleep together.** Bodies touching each other, or joined, form an island, and an island sleeps
  only when every body in it has stopped, and wakes all at once. If the bottom box of a stack
  slept while the top one still moved, the top one would push on a body that couldn't answer.
  Static, kinematic and character bodies join no island, or everything on one floor would be
  one island. A character never sleeps, so whatever it stands on can always carry it; a
  kinematic body sleeps once it is told no speed at all.
- **Stopped means gone nowhere you could see.** A body counts as stopped while no point of it
  strays more than its contact tolerance, about a pixel, in any second. Moving steadily, that is
  under 5 mm/s for a 1 m box and 0.05 mm/s for a 1 cm marble, however long it keeps it up, so a
  heap whose boxes creep slower than that still sleeps; buzzing in place, as a body deep in a pile
  can, it is stopped however fast it buzzes. An island sleeps once all
  its bodies have been stopped for a second, and its velocities are then set to exactly zero. A
  second is measured, the shortest that put nothing to sleep that was about to move: a box
  creeping on a slope a quarter of a degree past where friction holds it, and a cube a tenth of a
  degree past balance on its edge, tipping slowly at first.
- **Anything that disturbs a body wakes its island.** A touch from an awake body; any setter,
  force or impulse; [`setBody`](#world-setbody); removing a body it rests on or that rests on it;
  moving or changing a static body it touches; changing [`world.gravity`](#world-gravity). A
  stack woken by a touch is held up in that same step.
- **Part of an island can stop while the rest moves.** A body that has been stopped for a second,
  as above, and whose neighbours and their neighbours have too, is frozen: its velocities are set
  to exactly zero, nothing moves it, and its contacts with other frozen bodies or static ones are
  not solved, while the rest of its island goes on. It is still awake: [`body.asleep`](#body-asleep)
  is `false`, its contacts and events are kept, and [`world.moved()`](#world-moved) leaves it out
  once it has stopped. The whole frozen piece starts moving again, in the same step, as soon as
  anything within two contacts of it stops being stopped, or anything is set on any body of it.
  Bodies with a joint and characters never freeze, and nothing does with sleeping off. On a
  settling pile of 1,000 boxes this solved 73% of the contact points it did before, and took 19%
  less time over 15 s.
- **What it costs.** A sleeping body is not moved, bounded, filed or solved, and its contacts are
  not looked at. A world whose bodies are all asleep steps in a few microseconds, whatever its
  size.

Notes: a body starting from rest that would only speed up very slowly, slower than twice its
tolerance over a second squared (0.01 m/s² for a 1 m box), can fall asleep before it moves: a
box on a slope within a twentieth of a degree of where friction holds it stays put. Nudge it, or
switch sleeping off. A heap sleeps only when all of it has stopped, so one corner still settling
faster than a pixel a second keeps the whole heap awake until it, too, comes to rest.

A frozen body two or more contacts from anything moving doesn't notice the load on it change, as
a sleeping one doesn't: it answers motion, not force. A body added, or set by hand, wakes what it
touches, so what a program does is always seen; a change that reaches a frozen body only as a
push through bodies that don't move, which a game rarely has, is not.

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
- **Sensors aren't seen.** A query passes through a sensor, so a bullet's ray doesn't stop at a
  trigger zone. What is in a zone is for [`touching()`](#world-touching).

Notes: results are fresh objects each call; a query isn't meant for the inside of a step.

<a id="determinism"></a>
## Determinism

The same calls, with the same arguments, in the same order, give the same results to the bit: the
same `transforms`, the same velocities, the same everything, on any CPU, in any browser, in Node,
Deno or Bun, in a worker or not, and with any number of [`workers`](#world-workers) or none. That
is what replays, tests that compare whole worlds, and lockstep multiplayer need.

It holds because JavaScript defines `+`, `-`, `*`, `/` and `Math.sqrt` exactly: it forbids fused
multiply-add and extended precision, so every engine rounds them the same way. The functions the
language leaves to each engine, such as `Math.sin`, `Math.cos` and `Math.atan2`, are never used
inside Bearing; where it needs an angle, as in [`setAxisAngle`](#body-setaxisangle) or a hinge's
angle, it uses its own sine, cosine and arctangent, built from the exact operations. A test fails
if any of the others appears.

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
  `addBody: a world holds at most 16777216 bodies (the slot field of an id is 24 bits)`. Joints
  have ids made the same way, so as many joints, apart from the bodies:
  `addJoint: a world holds at most 16777216 joints (the slot field of an id is 24 bits)`.

One rate is capped, and the cap is documented where it acts: a body other than a sphere spins at
most half a turn a step, π / dt rad/s (188 rad/s at 60 Hz). See [Contacts](#contacts).

A slot can be reused 536,870,911 times (2^29 − 1) before it is retired and never handed out
again, so no old id can ever match a new body. Reused once a step at 60 Hz, that is 280 years, so
in practice a world never retires one.
