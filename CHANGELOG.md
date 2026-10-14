# Changelog

All notable changes to Bearing are recorded here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow
[Semantic Versioning](https://semver.org/spec/v2.0.0.html), measured against the
[API reference](docs/API.md).

## [Unreleased]

### Added

- `new World({ gravity })`, with `world.gravity`, `world.bodyCount`, `world.step(dt)` and
  `world.destroy()`. A step is exactly `dt` seconds of semi-implicit Euler: gravity for dynamic
  bodies, velocity for dynamic and kinematic ones.
- Bodies: `world.addBody`, `world.setBody`, `world.bodyOf`, `world.remove` and `world.body(id)`.
  Box, sphere and capsule shapes, with mass from the shape's volume and a density (water's by
  default) unless a mass is given. Dynamic, static and kinematic types.
- `Body` handles: `id`, `slot`, `world`, `alive`, `mass`, read-only copies of `position`,
  `rotation`, `velocity` and `angularVelocity`, and the chaining setters `setPosition`,
  `setRotation`, `setAxisAngle`, `setEuler`, `setVelocity` and `setAngularVelocity`.
- `world.transforms` and `world.previous`: every body's position and rotation as one
  `Float32Array`, now and before the last step, for any renderer to copy and interpolate.
- Ids that can never reach the wrong body: a 24-bit slot and a 29-bit generation in one exact
  number. Storage that grows as bodies are added and shrinks as they go.
- Bit-identical results on every machine: nothing in Bearing calls an implementation-approximated
  function, and rotations from angles use Bearing's own sine and cosine.
- Every error names its call, its option, what it must be and what it got, and an unknown option
  suggests the nearest real one.
- Inertia from the shape and mass, about the body's own axes, or an `inertia` option to give your
  own; `body.inertia` reads it.
- Lopsided bodies tumble as real ones do: a box spun about its middle axis flips over. A step
  carries angular momentum across exactly, so it is kept to rounding, and energy can only fall,
  so a fast spin never blows up.
- Forces and impulses: `body.applyForce(force, point)` and `body.applyTorque(torque)` act for the
  next step; `body.applyImpulse(impulse, point)` and `body.applyAngularImpulse(impulse)` act at
  once. Static and kinematic bodies are not moved by them.
- Collision. Every pair of box, sphere and capsule gets exact contact points, found before the
  shapes touch, and a soft-step solver keeps them apart: bodies rest, stack, slide and bounce. A
  210-box pyramid stands within 1.5 mm of where it was built; a box on a slope holds until tan θ
  passes its friction and then slides at exactly g (sin θ − μ cos θ); balls with restitution 1 keep
  their momentum and energy. Kinematic bodies push dynamic ones and carry what stands on them.
- Body options `friction` (0.5), `restitution` (0), `damping`, `angularDamping` and `gravityScale`.
  Friction mixes as a geometric mean, restitution as the larger.
- `world.substeps` (8), the one solver setting, measured rather than chosen: the fewest that keep
  the pyramid within half a box's tolerance.
- Every contact tolerance comes from the size of the bodies touching (a two-hundredth of the
  smaller one's thinnest size, about a pixel), so a 1 cm marble and a 50 m building both settle
  with nothing retuned.
- Thousands of bodies: a broadphase finds which bodies might touch without testing every pair.
  Bodies sit in a tree of boxes, and only a body that moves out of its box is looked up again, so
  a pile at rest costs almost nothing to keep. No world bounds and nothing to size.
- Body options `layers` and `mask`: 32 collision layers as bits. Two bodies collide when each one's
  layers are in the other's mask; by default everything collides with everything.
- Sleeping. Touching bodies form islands, and an island whose bodies have all stayed within a pixel
  of where they came to rest for a second goes to sleep: left out of every step, at no cost, until
  something disturbs it. Anything that does wakes it, with nothing to call: a touch, any setter,
  force or impulse, `setBody`, removing or moving what it rests on, or changing `world.gravity`.
  10,000 boxes asleep step in a few microseconds. `world.sleeping` (on by default) switches it off,
  and `body.asleep` says whether a body is.
- Balls and capsules roll and spin to a stop. Every contact is a patch as wide as its tolerance,
  which resists rolling and spinning as a real one does: a rolling resistance of 0.01 for any ball
  or capsule, so a ball rolled at 1 m/s rolls 7.1 m and stops, at any size. A fast-rolling ball
  also stays on the floor: the solver now follows a ball's contact by its centre, not by a point
  on its surface that rolled up and away.
- Queries. `world.raycast(origin, direction, options)` finds the nearest body a ray meets, and
  `world.raycastAll` every one, nearest first, each as `{ body, point, normal, distance }`.
  `world.shapeCast(shape, origin, direction, options)` slides a sphere, box or capsule and finds
  the first body it would touch, stopping within a pixel of it and never past it.
  `world.overlap(shape, position, options)` lists the bodies a shape placed there overlaps. Each
  takes `mask` and `ignore`; rays and casts take `maxDistance`. Queries see the world as it is
  between steps, walk the broadphase's tree, and never change what the world does. 10,000 rays into
  1,000 boxes take 73 ms on the laptop.
- `world.moved(since)`: the slots of the bodies an engine has to draw again since it last drew at
  step `since`, exact however many steps ran in between, so a world of sleeping bodies copies
  nothing. `world.stepCount` counts the steps.
- Triangle meshes and heightfields, for level geometry: `addBody({ shape: 'mesh', vertices,
  indices })` and `addBody({ shape: 'heightfield', heights, columns, spacing })`, static unless
  made kinematic. Every shape collides with their triangles, from either side, and the seams
  between triangles can't be caught on: a box slides across a floor of 800 triangles on exactly
  the path it takes across one box, and a ball rolls over a wavy heightfield without leaving it.
  Only the triangles under a body are looked at, about 17 a step for a box on 100,000 of them.
  Rays, shape casts and overlap tests see them too.
- `readGlb(bytes, options)`: every triangle of a `.glb` file's scene, or of one named node, placed
  as the scene places it, ready for a mesh body. No dependency; a file compressed with Draco or
  meshopt throws, naming the extension to export without.
- A kinematic body moving however slowly stays awake. Measured by how far it had strayed, a 10 m
  platform at 1 cm/s counted as still, fell asleep and stopped; now one is still only when its
  velocities are zero.
- Contact events. `world.contacts(body)` lists the last step's events: a `begin` when two bodies
  start touching, with where, which way and how fast they met, and an `end` when they stop. Read
  after the step, never called back from inside it, so anything can be changed while reading. A
  pair touches when the solver pushed it in the step or the step left it within a pixel. Given a
  body, only its events, turned so it is `a`. Removed bodies' touches end at the next step.
- `world.touching(body)`: every body touching it now, through the broadphase tree, for "is the
  player on the ground?" without a stay event per touching pair per step.
- Sensors: `addBody({ sensor: true })`, any shape and type. A sensor reports what overlaps it and
  pushes nothing; bodies pass through it on exactly the path they'd take without it. Sensors see
  every body but other sensors, never two static bodies, wake nothing, and queries pass through
  them. Events and sensors cost nothing when nothing changes.
