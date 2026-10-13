# Rules for Bearing

**Bearing** is a physics package that works with any engine. These rules are for building it. Its API copies the
style of Winding (github.com/nolanbaxter/winding, npm `winding-engine`), Nolan's WebGPU engine, so
that someone who knows one already knows how the other works. Winding's API reference
(`docs/API.md` in that repo) is the model. When these rules don't settle a question, do what
Winding does, and when Winding doesn't cover it either, ask Nolan.

## Decisions since these rules were written

These were made with Nolan and override the rules below where they differ.

- **Work to `ROADMAP.md`** (2026-10-13). It holds the goals, what 1.0 has to be, and every
  milestone with what "done" means. Update its status in the same commit that finishes a
  milestone; change the plan there first, with Nolan's OK, and log why.
- **Nothing in `src/` draws** (2026-10-13, Nolan: "Nothing in the core or src/ can do any
  visuals"). No canvas, WebGL, WebGPU, DOM or debug renderer in the package; `debugLines()`
  returns data; engine examples in the docs are examples, not package code. Demos may draw:
  they live in `demo/` (drawn with Winding from a pinned CDN URL), are never published, never
  imported by `src/`, and never needed by a test. Tests decide; demos show. Benchmark pages
  print numbers.
- **Engine-neutral, not built for Winding** (2026-10-12). Bearing copies Winding's naming and
  conventions, but it is not built around Winding. Every engine — Three.js, Babylon.js,
  PlayCanvas, Pixi, a plain canvas, Winding — is an equal user. Section 8 describes one consumer,
  not the design centre; nothing Winding-specific shapes the core, and the docs' examples stay
  engine-neutral.
- **Determinism: bit-identical results on every machine** (2026-10-12), not only in the same
  build. JavaScript forbids fused multiply-add and extended precision, so `+ - * /` and
  `Math.sqrt`, `abs`, `min`, `max`, `floor`, `ceil`, `round`, `sign`, `trunc` and `fround` give
  the same bits on every CPU and engine. The functions the spec calls *implementation-approximated*
  do not: `Math.sin`, `cos`, `tan`, `asin`, `acos`, `atan`, `atan2`, `exp`, `expm1`, `log` and
  family, `pow`, `cbrt`, `hypot`, the hyperbolics, and the `**` operator. **Nothing in `src/` may
  call them** — not only the step: a setter like `setAxisAngle` writes state too, so it has to be
  exact as well. Where an angle is needed, use `sin`/`cos` from `src/core/math.js` (fdlibm's
  method, exact operations only, within 1 ulp). `test/math.test.js` enforces this. Iteration
  order must not depend on anything but the inputs (no `Math.random`, no sort with an
  inconsistent comparator).
- **Body state is Float64Array columns; only `transforms`/`previous` are Float32Array**
  (2026-10-12, measured by `bench/storage.cjs`). Float32 state was 34–37% slower at 1k, 8k and
  64k bodies on the i7-1260P (conversion on every read and write), and columns, an interleaved
  record and a hot/cold split were within 2%. This overrides the "Float32Array columns" in rules
  17 and 75. Re-measure if the solver's access pattern changes the picture.
- **Ids are 53-bit**: `generation * 2^24 + slot`, 29 bits of generation, so a slot never
  retires in practice and the soak test's slot count can return (Winding's 8-bit generation
  can't). See `src/core/slots.js`.
- **Storage shrinks by the 1/4 rule, not a timer**: when the slots in use fall to a quarter of
  capacity, shrink to the power of two growth would have picked. Amortized O(1), no thrash, and
  rule 81's "quiet period" isn't needed.
- **Tumbling carries angular momentum, not a gyroscopic torque** (2026-10-12, step 2, measured
  in a scratch comparison). Bullet's implicit gyroscopic step kept only 74% of a flipping box's
  energy after 20 s; an implicit-midpoint version kept energy but let momentum wander 8% at
  20 rad/s and blew up at 50. `tumble()` in `src/world.js` works out L, turns the body by the ω L
  gives at the half step, and reads ω back from L: momentum exact to rounding, energy only ever
  falls (bounded by |L|²/2I_min). `test/motion.test.js` holds the measured energy figures.
- **The solver is the soft step, 8 substeps** (2026-10-13, milestone 4, measured in
  `bench/solver.js`). Sequential impulses (one substep, many passes) couldn't hold a 10-box stack.
  Each substep: one solving pass, one relaxing pass; contact stiffness 1/8 of the substep rate,
  damping ratio 10. A manifold's normal points are swept once per point (four for a box face),
  which is what stops a flat-landing box tipping and sliding. Every contact tolerance is
  `PIXEL` (1/200) of the smaller body's thinnest size. `world.substeps` is the one public knob.
  Pairs come from the broadphase (below).
- **The broadphase is a dynamic AABB tree** (2026-10-13, milestone 5, measured in
  `bench/broadphase.js` on recorded motion against sweep and prune and sort and sweep). Static
  bodies in their own tree; fat boxes of `FAT` (1/4 of the thinnest size) around and `AHEAD` (16
  steps of velocity) ahead; a persistent pair list (`src/pairs.js`) holding warm-start impulses.
  Layers and masks are 32 bits, and negative numbers (`~MASK`, `1 << 31`) are accepted as the same
  bits (Nolan, 2026-10-13). Re-run the bench's default after any change to the broadphase.
- **Sleeping** (2026-10-13, milestone 6). An island sleeps when every body in it has stayed within
  its tolerance of where it came to rest for `SLEEP_TIME`, 1 s, measured (see its comment in
  `src/world.js`). Distance, not speed: a speed rule kept a buzzing heap awake for good. Islands
  are rebuilt by union-find over the contacts, only once something has been still long enough.
  Per-body loops walk the awake list; asleep pairs sit past `pairs.active`; `moved(since)` reads
  per-body stamps through the awake list and a log bounded by the body count. API approved:
  `world.sleeping`, `body.asleep`, `world.stepCount`, `world.moved(since)`.
- **Contacts are patches** (2026-10-13, approved). A contact of one or two points resists rolling
  (up to N times its patch radius) and, with one point, spinning (up to 2/3 mu N times it); the
  patch is PIXEL of the rolling body's size, so rolling resistance is 0.01 for every ball and
  capsule. No option. Within a step, a round shape's contact is followed by its core (centre or
  axis), not a surface point, which on a fast-rolling ball rolled away and made it hop.
- **Forces last one step; static and kinematic bodies ignore forces and impulses** (no throw:
  they act as infinitely heavy). `inertia: null` returns to the shape's inertia.
- **Tests:** `npm test` (plain scripts, helper in `test/test.js`). `test/errors.test.js` checks
  every message in `docs/API.md`'s Throws lines both ways; `test/docs.test.js` runs every example
  in the docs and README. Types are checked in CI only (`npx -p typescript@7`), as in Winding.
- **Developed in the open and released as it goes, as Winding was** (2026-10-13). Bearing can be
  public; 0.x versions are released along the way, not held for 1.0. Versions follow Winding's
  rules: before 1.0, a breaking or additive change is `0.x.0` and a rework of existing code (fix,
  speed-up, docs) is `0.x.y`; from 1.0, major breaks, minor adds, patch reworks. Releases are made
  Winding's way (`ROADMAP.md`, "Versions and releases"). Still: never release, tag, publish or
  change the repo's visibility without Nolan's go-ahead for that specific action.
- **CI runs only what a change reaches** (2026-10-13, Nolan: "I *definitely* want that"). Port
  Winding's `.github/affected.js` (the Node-suite part: imports and `new URL()` reads, followed
  down) and its test before the first release. This overrides rule 95's "once the suite is big
  enough". Winding's local checkout is `..\3D concept`; read it for how Winding did anything.
- **GitHub: Nolan's personal account only** (2026-10-12). The repo is `nolanbaxter/bearing`,
  public since 2026-10-13 (Nolan: "You can make it public whenever you are ready"). Commits are authored as
  `LmnTrtl <294360573+nolanbaxter@users.noreply.github.com>` (set in this repo's git config), and
  every `gh` call runs as the personal account (`GH_TOKEN=$(gh auth token --user NolanLT)`). The
  work account and work email never appear anywhere in this project.
- **`package.json` is `"private": true` until the first release**, so a stray `npm publish` from
  the repo is refused. The name claim in rule 9 is a separate placeholder package that Nolan
  publishes himself (it needs his 2FA), and it is also what npm needs to exist before trusted
  publishing can be configured.

**The name.** The project is **Bearing**; the npm package is **`bearing-physics`** (plain `bearing`
is taken). The repository is `nolanbaxter/bearing`, with no suffix, as Winding's is
`nolanbaxter/winding`. Write "Bearing" in prose, with a capital and no ".js", the way Winding is
written. Imports look like Winding's:

```js
import { World } from 'bearing-physics';
```

Why this name, so it doesn't have to be worked out again: it follows the same pattern as
Winding. Winding is an everyday word that is also a term from rendering (winding order). A bearing
is the plainest piece of mechanics, the part that carries a load and lets things turn and roll, and
the word also means a direction. Next to Winding it reads as one family: both end in -ing, and
both are parts of a machine (a clock's winding, a wheel's bearing).

Bearing must not depend on Winding. Winding is the API to copy, and the first engine
it will be used with. It is not something to import.

---

## 1. Ground rules

1. **No dependencies.** None at runtime, and no dev dependencies either if it can be helped.
   Tests run on Node's own `node:assert` and `node:test`, or on a plain script.
2. **No build step.** Plain ES modules, published as they are. The files people import are the
   files in the repo. No TypeScript compile, no bundler, no transpile. This also means no
   WebAssembly unless Nolan decides otherwise: a plain-JavaScript physics library that is
   actually fast is the gap this package fills (see section 15).
3. **Runs anywhere JavaScript runs:** browsers, Node 20+, workers, Deno and Bun. No DOM, no
   `window`, no `document`, and no rendering API anywhere in the core.
4. **Types are written by hand** in a `.d.ts` next to the JavaScript, not generated from it. They
   give TypeScript users and plain-JavaScript editors completion and hover help. A test checks
   that the API reference's examples type-check against them.
5. **Branches:** `develop` is where work lands. `main` holds released code only. Commit and push
   to `develop` as the work goes.
6. **Batch releases.** Land a planned set of changes on `develop`, then cut one version for all
   of them. Don't release each change on its own.
7. **Semantic versioning, measured against the API reference:**
   - patch: fixes to documented behaviour, performance, and docs;
   - minor: additions, and renames that keep the old name working;
   - major: anything that breaks code written against the reference.
8. **Public actions need Nolan's go-ahead.** That covers publishing, releases, posting anything,
   and making the repo public. Never handle npm one-time codes or 2FA. Releases publish from CI
   with npm trusted publishing (OIDC), with no tokens stored anywhere.
9. **Claim `bearing-physics` on npm early.** It was free on 12 October 2026. npm refused
   `winding` as too similar to `bindings` (which is why Winding's package is `winding-engine`),
   and that check only runs at the first publish. So, with Nolan's go-ahead, publish a placeholder
   0.0.0 before the repo goes public, to find out whether npm accepts the name while it can still
   change cheaply.

---

## 2. The shape of the API

Winding's API reference opens with one sentence, and it is the rule for this package too: "These
hold everywhere, so one learned is one learned for good."

### Creating things

10. **One way to make, change, read and remove every kind of thing.** In Winding, for every kind
    (lights, sprites, decals and so on):
    - `scene.addX(options)` makes one and returns a handle.
    - `scene.setX(handle, changes)` changes any of the options `addX` took and leaves the rest.
    - `scene.xOf(handle)` returns a **copy** of its current options, or `null` if the handle has
      none of that kind.
    - `scene.remove(handle)` or `handle.destroy()` removes it.

    Do the same here:
    - `world.addBody(options)`, `world.setBody(body, changes)`, `world.bodyOf(body)`;
    - `world.addJoint(...)`, `world.setJoint(...)`, `world.jointOf(...)`;
    - `world.remove(x)` and `x.destroy()` for all of them.

    Don't give one kind its own pattern, such as a constructor, a `create...()` call, or its own
    remove method.
11. **Options are one plain object with a default for every field.** Leaving something out means
    "the default", never "error". `addBody({})` makes a valid body.
12. **Every option of `addX` is also a valid change to `setX`,** with the same name, meaning and
    checks. If something can only be set at creation, say so in its docs and throw if `setX` is
    given it. Keep such options rare.
13. **Placement options are shared.** In Winding, `position` and `parent` are options of every
    `addX`. Here, `position`, `rotation` and the velocities should be accepted by every body,
    with the same meaning everywhere.
14. **No `init()`, and no required order.** Nothing needs setting up or tearing down in a
    particular order. Make a world, add bodies, step it. The world grows its storage as things are
    added: no capacity to choose in advance, and no limit hit later. A `capacity` hint is fine as
    an option, but never as a requirement.
15. **`new` for things that are ready at once; an async static factory only when creation has to
    wait.** Winding uses `await Winding.create(canvas)` because a GPU device takes time to get, and
    `new Camera()` for everything else. A pure-JavaScript world needs nothing to wait for, so it
    should be `new World(options)`. If something ever does need to wait (loading a mesh file, or
    starting workers), use `await World.create(...)` with the same options, and keep the sync form
    too if it can work.
16. **A static `supported()` check if anything can be missing.** Winding's
    `await Winding.supported()` returns `true` or `false` and never throws. Physics in plain
    JavaScript should never need one. If an optional fast path is added later (workers,
    `SharedArrayBuffer`), it should switch on by itself when available and fall back silently,
    with identical results either way. Winding's job system does exactly this.

### Handles

17. **Handles, not objects that own data.** A Winding `Node` stores nothing itself: it holds a
    scene and an entity id, and the data lives in typed arrays. A body handle here should work the
    same way: a small object with the world and an id, while positions and velocities live in
    shared `Float32Array` columns.
18. **Compare handles by id, not by identity.** Two handle objects can refer to the same body, so
    users compare `a.id === b.id`. Say so in the docs, as Winding does for `node.entity`.
19. **Make ids generational from day one:** a slot index plus a generation counter, so a removed
    body's slot can be reused without an old handle reaching the new body. Winding didn't do this
    early, and fixing it now would be a major version. `body.alive` is `false` once the body is
    removed, and using a dead handle throws with the call's name.
20. **Transforms are set with methods, never by writing properties.** Winding's rule, and its
    reason, apply exactly. `body.position.x = 5` can't work when the data lives in shared columns,
    because the write would never mark the body as changed. So use
    `body.setPosition(x, y, z)`, `body.setRotation(q)` and `body.setVelocity(x, y, z)`.
21. **Setters return the handle, so they chain:**
    `ball.setPosition(0, 5, 0).setVelocity(2, 0, 0)`.
22. **Setters check everything before writing anything.** A non-finite value throws
    `setPosition: non-finite in (…)` before any field is written, so a caught throw leaves the
    body where it was.

### Reading back

23. **Getters return copies or plain data, never live internals.** `world.bodyOf(body)` returns a
    fresh plain object. Changing it does nothing; the user passes it to `setBody` to change
    anything.
24. **Queries return a plain result or `null`.** Winding's
    `scene.raycast(origin, direction, { maxDistance })` returns `{ node, renderable, distance }`
    or `null` for a miss, and never throws on a miss. Here:
    `world.raycast(origin, direction, options)` returns `{ body, point, normal, distance }` or
    `null`. The same holds for shape casts and overlap tests.
25. **Say in the docs which values in a result can't be kept.** Winding documents that
    `renderable` is an index that changes when anything is removed: "so do not keep it". Say the
    same about anything here that changes after a step or after a removal.
26. **Read-only properties are documented as read only** and throw if assigned to, rather than
    quietly ignoring the write.

### Settings

27. **World settings are plain properties, writable at any time:** `world.gravity`,
    `world.iterations`, `world.sleeping`. Winding's renderer settings (`ao`, `taa`, `shadows`)
    all switch at runtime. Nolan asked whether runtime switching was slower, it was measured, it
    wasn't, and runtime won over "set at creation only".
28. **Off means off.** A feature that is switched off, or has nothing to work on, costs nothing:
    no pass runs, no memory is held, and no per-step check runs that could be avoided. Winding
    enforced this across every feature in 1.6.1. Examples here:
    - no joints means the joint solver doesn't run;
    - sleeping off means no sleep bookkeeping;
    - no continuous collision detection on any body means none of that code runs;
    - a world with no bodies steps in almost no time.

    Test for it.

---

## 3. Nothing arbitrary, and out of the way

This is the rule that the rest of Winding's design follows from. Its README ends the hello world
with: "That is the whole setup. There is no `init()` to forget, no render-order to get right, no
pass list to maintain, and no far plane to tune." Bearing should be able to say the
same about itself. The user says what they want simulated, and the package works out the rest.

**Nothing arbitrary**

29. **Every number in the code has a reason, written next to it.** A constant must be one of two
   kinds:
   - **derived**, from the data, a format's field width, the physics, or the step size;
   - **measured**, with the measurement named in the comment.

   "Seemed about right" is not a reason. If a number can't be justified, it is a setting the user
   shouldn't have to think about, and it should be derived instead.
30. **No limits the user has to plan around:** no maximum body count, no world bounds for the
   broadphase, no fixed pool sizes, no "reserve N contacts". Storage grows as needed, and capacity
   options are starting sizes, not budgets. Winding's README says it this way: "Nothing is sized
   in advance." Winding has exactly two hard limits left: 2^24 entities, the index field of a
   handle, and 4096 materials, the material field of a sort key. Both are "derived rather than
   chosen", and the README names them. Any hard limit here must meet the same test: derived from
   a format, documented with its reason, and hit with a clear error naming the limit, never a
   silent failure.
31. **Tolerances scale with what is being simulated,** not fixed constants tuned for one size of
   object. Where possible, derive these from body size, gravity and the step:
   - contact slop;
   - sleep thresholds;
   - when CCD kicks in;
   - the speed below which bounces stop.

   Then a 1 cm marble and a 50 m building both behave without the user retuning anything. Where
   one fixed tolerance can't be avoided, document it with the scale it assumes. This is also why
   the 2D units question in section 5 matters.
32. **Defaults come from real things, not convenient numbers:**
   - gravity is Earth's, 9.81 m/s², in 3D;
   - mass comes from the shape's volume times a density, with water's, 1000 kg/m³, as the default;
   - the inertia tensor is computed from the shape.

   The user can override each one, but never has to supply them for a correct result.
33. **No silent clamping or thresholds.** If a value is out of range, throw, as the errors section
   says. If behaviour changes past some point, document the point and why it exists. Nothing
   should change quietly at a number the user can't see.

**Out of the way**

34. **Ask the user only for what only they know:** which shapes, where, how heavy if not the
   default, how bouncy, how slippery, and what collides with what. Everything that can be worked
   out is worked out.
35. **No knobs the user must tune to get a correct result.** With the defaults, in normal units:
   - a box dropped on a floor comes to rest;
   - a stack of boxes stands;
   - a ball bounces and settles;
   - a fast object doesn't pass through a wall it should hit.

   Settings such as iterations and substeps exist for unusual needs, never to make the defaults
   work. If users keep needing to set something, the default is wrong, so fix the default.
36. **No bookkeeping calls.** A change takes effect when made. There is no
   `updateMassProperties()`, no `wakeUp()` needed after `setPosition`, no `refreshBroadphase()`,
   no `commit()`. Moving or pushing a sleeping body wakes it. Changing a shape updates its mass and
   inertia. Adding or removing a body between steps just works.
37. **No required order.** Bodies, shapes, joints and settings can be added and changed in any
   order. The only requirement is that something referred to must exist, such as the bodies a
   joint connects.
38. **No global state.** Two worlds in one page, or one per worker, don't affect each other. There
   is no singleton, no global configuration and no registration step for shapes or materials.
39. **The package doesn't own anything that belongs to the user:**
   - not the loop or the clock (rule 68: `step(dt)` takes the step it's given);
   - not the user's objects, the canvas or input;
   - no patched globals, no event listeners, no timers.

   Nothing it starts keeps running after `world.destroy()`.
40. **Quiet in normal use.** No console output from correct code. A warning only for a likely
   mistake that isn't an error. Winding warns when one clip has been joined onto the same node
   more than 64 times, because that is almost certainly a loop calling `play` every frame.
41. **The hello world is the test.** If a line in the README's minimal example is there to make it
   work, rather than to say what the user wants, that line is a bug in the package. Examples:
   - setting up a broadphase;
   - setting iterations;
   - computing a mass;
   - waking a body.

   Winding's hello world is 11 lines because every line is the user's own decision.

---

## 4. Naming

42. **Short, real names.** Winding's guide puts it as `resolution`, not
    `renderScaleFactorValue`, and not an invented word. Nolan prefers short names, but not made-up
    or forced ones. Established terms stay as they are, such as `ccd`, `restitution`, `friction`
    and `aabb`.
43. **One name per idea, everywhere.** If `radius` means a sphere's radius, it also means a
    capsule's radius. If `size` means box extents in one place, it can't mean something else
    elsewhere. Winding had an audit that found things like an emitter's `size: [a, b]` meaning
    "start and end" while every other `size` meant width and height. It became `size` and
    `sizeEnd`. Avoid that from the start.
44. **Pairs follow one pattern:** `x` and `xEnd`, `min` and `max`, `inner` and `outer`. Pick one
    pattern per kind of pair and keep it everywhere.
45. **Booleans read as a statement:** `sleeping`, `castShadow`, `ccd`, `sensor`. Not
    `isSleepingEnabled`, and not `enableSleep`.
46. **Methods are verbs and properties are nouns:** `step()`, `raycast()`, `remove()`;
    `gravity`, `bodyCount`.
47. **Renames keep working for one minor version, then throw with directions.** Winding's renamed
    options and properties throw, naming what they are called now. For example:
    `run: overlay is now hud, with the same { scene, camera }`. The API reference keeps a
    "Renamed" section listing every old name and its new one. A renamed method can simply be
    gone, so calling it says it isn't a function.

---

## 5. Units and conventions (the same as Winding's, so data moves straight across)

48. **3D units are metres, kilograms and seconds,** as in glTF and as in Winding.
49. **3D is Y-up and right-handed,** as in glTF and Winding.
50. **2D units match Winding's 2D view:** one unit is a CSS pixel, and y points down. Gravity in
    2D therefore points toward +y, `[0, 980]` or so. **Get Nolan's decision on this before
    writing the 2D module.** The other choice is metres with y up, then scaled for display. Box2D
    works in metres because its tolerances are tuned for metre-sized objects, and pixel units
    break those tolerances. Whichever is chosen, tune the tolerances for it and document it in
    one place.
51. **Angles are radians.** In 2D, a positive angle turns clockwise, matching Winding's
    `Camera2D` and CSS `rotate()`, which follow from y pointing down.
52. **Vectors are plain arrays** (`[x, y, z]`), or separate numbers in a setter (`setPosition(x,
    y, z)`). No vector classes in the public API. Accept any array-like, including a
    `Float32Array`.
53. **Rotations are quaternions as `[x, y, z, w]`,** as in glTF and Winding's `setRotation(q)`.
    Offer `setAxisAngle(axis, angle)` and `setEuler(yaw, pitch, roll)` as Winding does, and
    `setAngle(radians)` for 2D.
54. **Two numbers mean 2D,** if the 2D and 3D modules share any calls. In Winding, `[x, y]` or
    `(x, y)` works wherever a 3D call takes three: z is 0 for a position and 1 for a scale.
55. **Time is seconds,** as a `dt` argument. Never milliseconds, and never frames.

---

## 6. Errors

56. **Bad input throws, at the call that took it, naming the call, the option, what it should
    be, and what it got.** This is Winding's exact format:
    `addLight: radius must be positive, got -2`
    `addLight: color must be 3 finite numbers, 0 or more, got 1,NaN,0`
    The pattern is `` `${caller}: ${option} must be ${rule}, got ${value}` ``. A shared check
    function takes the caller's name, so `setBody` errors say `setBody`, not `addBody`. The audit
    found Winding's `setTile` throwing errors named `setTiles`; avoid that.
57. **Never draw, or here simulate, something quietly wrong.** NaN must not be allowed into the
    solver. Check at the boundary, so that an exploding simulation can always be traced to a bad
    input or a real solver problem.
58. **Reject unknown options. Winding gets this wrong today.** A misspelt `castShadows` instead
    of `castShadow` was silently ignored, and a whole test ran for weeks without testing shadows.
    In this package, an option name that isn't recognised throws:
    `addBody: unknown option 'resitution' (did you mean 'restitution'?)`. Suggesting the nearest
    known name is cheap and catches most typos.
59. **Every message a call can throw is listed in its API entry,** under "Throws:", word for
    word.
60. **Validation is not optional for speed.** Check options at `addX`/`setX`, which are rare
    calls. Inside `step()`, check nothing per body per step: the data was checked when it came in.

---

## 7. Keep rendering out of the core

61. **The core knows nothing about any engine, any renderer, the DOM or the GPU.** It takes
    shapes, masses and transforms, and gives back transforms, contacts and query results.
62. **The main output is one typed array per world, read in bulk.** For example, a
    `Float32Array` with 7 floats per body in 3D (position plus quaternion) or 3 in 2D (x, y,
    angle). Each engine copies it in one loop. This is faster than handing out objects per body
    and works the same for every engine, with nothing engine-specific to maintain.
63. **Say when a body moved.** Give a way to visit only bodies that changed this step, such as a
    list of their slots, so an engine with 10,000 sleeping bodies copies nothing. Winding's
    renderer works this way internally ("changed" lists, not "check everything").
64. **Adapters for each engine live outside the core,** as small separate modules or just as
    examples in the docs: Winding, Three.js, Babylon.js, PlayCanvas, Pixi, a plain 2D canvas.
    They must not be imported by the core, and core tests must not need them.
65. **Contacts are read after a step, not delivered by callbacks.**
    `world.contacts()` returns that step's begin/stay/end events as plain data. Callbacks fired
    from inside the solver are where reentrancy bugs come from: a user removing a body during a
    callback mid-solve. Reading after a step fits a fixed-step loop and copies across to a worker
    cleanly. (This is a suggestion; check it with Nolan.)
66. **Debug drawing goes out as data, not draw calls:** `world.debugLines()` returns line
    segments as a `Float32Array`. Winding's `engine.debug.line()` can draw that in one loop, and
    so can any other engine.

---

## 8. Fitting into Winding (the first engine it will be used with)

67. **The step goes in Winding's fixed update.** `engine.run(scene, camera, { update, frame })`
    calls `update(dt)` at a fixed 60 Hz (`engine.clock.fixedDt`), zero or more times per drawn
    frame. `world.step(dt)` belongs there. `frame(alpha)` gives how far the drawn frame sits
    between the last two steps, so the adapter can interpolate the rendered position from the
    previous and current transforms. **Keep the previous step's transforms available** (a second
    array, or a `world.previous` view) so interpolation needs no extra copy.
68. **`world.step(dt)` takes exactly the step it's given.** No hidden accumulator or substepping
    inside it by default, since Winding's loop already owns the fixed step. An optional
    `substeps` option is fine.
69. **Winding nodes take transforms with chained setters,** `node.setPosition(x, y, z)` and
    `node.setRotation(q)`. Setting thousands per frame is already fast: Winding's 10,000-cube
    benchmark does exactly that. If profiling ever shows the per-node copy matters, Winding can add
    a bulk transform write. Ask then, not before.
70. **Mesh colliders from loaded models need Winding to keep geometry.** Since 1.6.1, Winding
    drops a model's CPU copy after uploading it to the GPU (`asset.source = null`), which cut
    Sponza's heap from 9.4 MB to 0.4 MB. A triangle-mesh or convex-hull collider made from a
    glTF needs vertices and indices. When Bearing is ready for this, report it, and
    Winding will add an opt-in load option that keeps them. Until then, colliders come from
    shapes, or from Bearing reading the `.glb` itself, which it can do with no
    dependency since glTF is JSON plus a binary buffer.
71. **When the package is ready to be used with Winding,** report what it exports, a minimal
    example, and anything Winding would need to add. Winding's side is done in the Winding
    session.

---

## 9. Performance

72. **Measure with A/B tests in one page.** Old code and new, timed in the same page in
    alternating blocks, because separate runs drift by more than most changes. A change made for
    speed comes with that measurement. A change that measures no better is left out, however
    sensible it looked. Winding reverted render bundles for exactly this reason.
73. **Fast for everyone, not tuned to one machine.** Nolan's laptop is an Intel Iris Xe and his
    desktop a Ryzen 5 5500. Measure on both, and flag any win that depends on the CPU and who might
    lose from it.
74. **Same results for less work.** An optimisation must not change the simulation's output,
    except where a documented tolerance allows it. Test that it doesn't.
75. **Data-oriented storage.** Struct-of-arrays in typed arrays: positions, velocities, inverse
    masses and so on each in their own `Float32Array` column, indexed by slot. No object per body
    in the hot loop. This is Winding's design (`TransformStore` columns), and in JavaScript it is
    the difference between fast and slow.
76. **No allocation in `step()`.** Reuse scratch arrays. Shared module-level scratch is fine,
    since a step isn't re-entered; Winding's `node.js` does this with a comment saying why.
77. **Benchmark against the competition the same way Winding does.** Winding has a
    `bench/compare.html` page that runs the same scenes through four engines in one page, taking
    turns. Do the same here against Rapier, Jolt (JoltPhysics.js), cannon-es and, for 2D,
    Box2D/planck.js and matter.js. Suggested scenes:
    - a pile of 1,000 to 10,000 boxes settling;
    - a stack (the solver-quality test);
    - many raycasts per step;
    - a ragdoll or a chain of joints;
    - the same in 2D.

    Measure time per step, and whether the stack stays standing.
78. **Say plainly when something won't help.** Nolan wants the facts plus a recommendation, and
    pushback on ideas that won't improve things, rather than going along with them.

---

## 10. Memory

79. **Nothing grows forever.** Winding has a permanent leak test (a "soak"): it runs each
    scenario many times and fails if buffers, objects or listeners keep growing. Build the same
    early here:
    - add and remove 10,000 bodies, many times over;
    - check that the world's memory and slot count return to where they started;
    - check, with `WeakRef` and `--expose-gc`, that removed bodies and worlds are collected.

    Nolan's words: "keep all of those leaks at bay permanently… not a memory colander".
80. **Freed slots are reused,** using the generations from rule 19.
81. **Storage can shrink.** After a big burst followed by a long quiet period, arrays can shrink
    back. Winding shrinks a buffer after 120 quiet frames. Don't shrink on every removal, or it
    thrashes.
82. **`world.destroy()` releases everything,** and using a destroyed world throws with the call's
    name.

---

## 11. Documentation

83. **One API reference, `docs/API.md`, covers every public call, option and setting.** It opens
    with a conventions section (this file's sections 2 to 6, in user-facing words), then contents,
    then an A–Z index of every entry.
84. **Every entry has the same parts, in this order:**
    - an `<a id="world-addbody"></a>` anchor;
    - a heading in the form `` ### `world.addBody(options)` → `Body` ``;
    - one or two sentences on what it does;
    - an options table with three columns: `Option | Default | Meaning`;
    - `Returns:` …;
    - `Throws:` followed by every message, word for word;
    - a short example;
    - `Notes:` for edge cases.
85. **The docs change in the same commit as the behaviour.** A new call, a changed option or
    default, a new error message, or behaviour changed by a fix all update `docs/API.md` along
    with the types, the README and the CHANGELOG.
86. **A test keeps the reference complete.** Winding's `test/docs.test.js` reads every public
    method and getter of every exported class and fails if one has no entry or a link goes
    nowhere. Methods that are public but not meant for users go in a list with a reason each.
87. **Writing style:**
    - Plain English, complete sentences, no marketing.
    - Say what a thing does, what it takes, what it returns and what can go wrong.
    - Units go with every number.
    - Spell out the reason behind a rule.
    - Limitations are stated plainly. Winding's README says "WebGPU only" and explains why.
88. **The README** says:
    - what the package is;
    - a short hello world;
    - how it compares, with measured numbers and the method and machines named;
    - how to install from a CDN with a pinned version and an import map;
    - running the tests.
89. **A side-by-side page** like Winding's `docs/SIDE-BY-SIDE.md` is worth having: the same small
    program in this package, Rapier, cannon-es and so on, each run to check it works, with the
    versions it was written against.
90. **The CHANGELOG** follows Keep a Changelog: `## [Unreleased]` with Added, Changed and Fixed
    sections, and each release opens with a short plain-English paragraph on what it brings.

---

## 12. Tests

91. **A change comes with a test that fails without it.** A bug fix starts from a script that
    shows the bug.
92. **Plain Node test files** (`test/*.test.js`) with `node:assert/strict`, run by `npm test`. No
    framework.
93. **Test what physics must get right, not only that the code runs:**
    - Energy and momentum: elastic collisions conserve them within a tolerance.
    - Stacking: a stack of 10 boxes stays standing after 10 seconds of simulation.
    - Tunnelling: a fast small body doesn't pass through a thin wall with CCD on.
    - Determinism: same inputs give the same outputs, bit for bit, in the same build.
    - Sleeping: a resting pile goes to sleep, and a hit wakes it.
    - Off means off: a disabled feature's code doesn't run. Count calls or time it.
    - Errors: every documented throw is triggered once, and its message is checked word for word.
94. **Test assets live in `test/assets/`,** each listed in a README there with where it came from
    and its licence. Nothing there is published.
95. **CI runs only what a change reaches.** Winding's CI runs only the test suites whose imports
    reach a changed file, and a docs-only change finishes in seconds. Copy that once the suite is
    big enough to matter. Until then, run everything on Node 20, 22 and 24.

---

## 13. Code style

96. **Match the code around you.**
97. **Comments say why, in full sentences.** Explain the reason for a decision, the alternative
    that was rejected, and what would break. Don't narrate what the next line does. Winding's
    `node.js` header, explaining why there are setters instead of properties, is the model.
98. **Plain modules, named exports, one public entry file** (`src/bearing.js`, as Winding's is `src/winding.js`), plus
    lower-level modules reachable by path through `"exports": { "./*": ... }`. Dropping down a
    level is always possible: there is no wall, only a floor.
99. **Mark a deliberate simplification** that has a known limit with a comment naming the limit
    and what to do when it is reached, for example "O(n²) pair test; use the broadphase when
    bodies > 64".
100. **Commit messages:** `path/or/area: what changed`, then a body saying why. A release is a
    single commit titled `Release x.y.z: <theme>`.

---

## 14. Working with Nolan

101. **Ask before:** releases, publishing, anything public, downloads (state the file, where it
    comes from and its size), and breaking changes to the API.
102. **Give facts and a recommendation,** not a menu of options. Push back when something won't
    help, and say so plainly.
103. **Proposed API changes are shown before they are made.** He approves them in a batch, and
    they land in one change together with their docs.
104. **Consistency is how he learns an engine.** In his words, the same naming and conventions
    everywhere matter more than any single clever name.

---

## 15. Extra thoughts on the physics itself

These are suggestions, not rules. Weigh them, measure them, and ask Nolan where they matter.

105. **The opening:** Rapier and Jolt are WebAssembly builds; cannon-es is the only widely used
    plain-JavaScript 3D option, and it is slow and barely maintained. A plain-JavaScript library
    with no dependencies that is genuinely fast is the gap. The benchmark in rule 77 is how to
    prove it, and the README chart is how to show it.
106. **Separate 2D and 3D modules that share conventions,** as Rapier ships `rapier2d` and
    `rapier3d`. One generic API over both makes both worse. They can share code that works in
    either, such as the broadphase or island building, where it doesn't cost speed.
107. **Decide determinism early.** Same inputs giving the same outputs in the same build is cheap
    and very useful for replays and tests. Identical results on every machine is much harder:
    `Math.sin` and similar functions may differ between JavaScript engines, so it would need its
    own versions. Multiplayer with lockstep networking needs that. Decide which one is promised,
    and write it in the docs. *(Decided: every machine. See the decisions at the top.)*
108. **Order of features that users actually reach for:**
    1. rigid bodies with spheres, boxes and capsules;
    2. raycasts;
    3. static triangle meshes and heightfields for the level;
    4. sleeping;
    5. a character controller (the most-asked-for feature in game physics);
    6. joints;
    7. CCD;
    8. convex hulls;
    9. sensors and triggers;
    10. collision layers and masks.

    Ragdolls, vehicles and soft bodies come later.
109. **Kinematic bodies** are moved by the user and push dynamic ones. Moving platforms and
    animated characters need them, and they come up early.
110. **Collision layers and masks:** use a 32-bit membership mask and a filter mask per body, as
    most engines do. Make the default "everything collides with everything".
111. **Broadphase:** a sweep-and-prune or a dynamic AABB tree. Measure both on the benchmark scenes
    before choosing. An AABB tree also speeds up raycasts. Winding uses its own BVH for raycasts
    over meshes.
112. **Solver:** a sequential-impulse solver with warm starting is the standard, well understood,
    and stacks well with a few iterations. Substepping (the "small steps" approach used in recent
    research and in Box2D v3) is often more stable for the same cost. Benchmark it against the
    stack test.
113. **Workers later, not first.** Get one thread fast first. If workers come, results must be
     identical with and without them, and it must switch on by itself when the page allows it
     (`SharedArrayBuffer` needs cross-origin isolation), as Winding's job system does.
114. **Serialisation:** `world.save()` returning plain data and `World.load(data)` make replays,
     tests, networking and bug reports much easier. A bug report that includes a saved world can
     be reproduced exactly.
