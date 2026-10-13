# Bearing roadmap

What Bearing is for, what 1.0 has to be, and the milestones that get it there. Every session works
to this file. When a milestone finishes, its status changes here in the same commit. When the
plan changes, this file changes first, with Nolan's OK, and the reason goes in its log at the
bottom.

---

## The goals

Bearing is a physics package you call. It is not a renderer, an engine, or a demo. These are the
same goals Winding was built to, applied to physics, and 1.0 is the point where all of them are
met and measured.

1. **Professional grade.** The features a real game reaches for, each built the way production
   engines build them, and each held to physics tests, not only "it runs": stacks stand, bounces
   settle, momentum is conserved, nothing tunnels, and the same inputs give the same results.
2. **Fast, in plain JavaScript.** No WebAssembly, no build step, no dependencies. Measured against
   Rapier, Jolt and cannon-es in one page, on Nolan's laptop (Intel i7-1260P, Iris Xe) and
   desktop (Ryzen 5 5500).
3. **Out of the way.** The user says what to simulate and nothing else. No `init()`, no
   bookkeeping calls, no settings to tune for a correct result, and tolerances derived from the
   size, speed and step of what is simulated, never fixed for one scale.
4. **Winding's conventions.** One way to add, set, read and remove every kind of thing; errors that
   name the call, the option, the rule and the value; one API reference with every call in it;
   the same units, axes and naming.
5. **Any engine, and no rendering.** Bearing takes shapes and transforms and gives back
   transforms, contacts and query results as data. Nothing in it draws or touches the DOM, and
   it imports nothing from any engine. Three.js, Babylon.js, PlayCanvas, Pixi, a plain canvas and
   Winding are equal users.

### What Bearing never does

- **Draw anything in the package.** Nothing in `src/` touches a canvas, WebGL, WebGPU or the DOM,
  and there is no debug renderer: `world.debugLines()` returns numbers for an engine to draw.
  Demos are separate (see [Demos](#demos)): they live in `demo/`, draw with an engine, and are
  never published, never imported by `src/`, and never needed by a test.
- **Own the loop, the clock, input, or the user's objects.** `world.step(dt)` takes the step it's
  given.
- **Depend on anything**, at run time or for its tests, except Node's own modules. CI may fetch
  tools for a check (TypeScript for the types, Bun for a second JavaScript engine); the package
  never needs them.

---

## 1.0: what it has to be

### Features

| Area | In 1.0 |
|---|---|
| Bodies | Dynamic, static and kinematic. Mass and inertia from the shape, or given. Forces, torques and impulses. Damping and gravity scale. Rotation locks. |
| Shapes | Sphere, box, capsule, convex hull (built from points), compound (several shapes on one body), triangle mesh and heightfield (static). |
| Contacts | Friction, restitution, stable stacking, speculative contacts, kinematic bodies pushing dynamic ones, materials per body. |
| Broadphase | Grows with the world, no bounds to set. Collision layers and masks. |
| Sleeping | Islands; a resting pile sleeps, a touch wakes it. A list of the bodies that moved, so an engine copies only those. |
| Queries | Raycast (nearest, and all hits), shape cast, overlap test. |
| Events | Contact begin, stay and end, and sensors (triggers), read after a step as plain data. |
| Gameplay | A character controller: walk, slide along walls, climb steps and slopes, stand on moving platforms. |
| Joints | Fixed, ball, hinge, slider, and distance/spring, with limits and motors. |
| CCD | Fast small bodies don't pass through thin walls. |
| Data | `world.save()` and `World.load(data)`, exact. `world.debugLines()` as a `Float32Array`. Content read from a `.glb` with no dependency. |

### Correctness, held by tests, with the defaults alone

- A box dropped on a floor comes to rest, and goes to sleep.
- A stack of 10 boxes stands for 10 s. A pyramid with 20 boxes on its bottom row (210 in all)
  stands.
- A ball bounces and settles. An elastic collision keeps momentum and energy within a stated
  tolerance.
- A 1 cm marble and a 50 m building both behave, with nothing retuned between them.
- A fast small body doesn't pass through a thin wall.
- The same inputs give bit-identical results on V8 (Node) and JavaScriptCore (Bun), checked in CI
  against a stored hash of a whole scene.
- The leak test (soak) passes for every kind of thing that can be added and removed.
- Off means off: every feature that isn't used costs nothing, counted by a test.

### Speed

Goals, provisional until milestone 7 measures where Bearing actually stands. If the numbers show
a goal can't be met, it is reset with those numbers and Nolan's OK, never quietly.

- Faster than cannon-es on every benchmark scene.
- Within 2× of Rapier's time per step on every benchmark scene, on both machines.
- The stack-quality scenes stand in Bearing wherever they stand in Rapier and Jolt.

Benchmark scenes: a pile of 1,000 and of 10,000 boxes settling; a 10-box stack and the 210-box
pyramid; 10,000 raycasts a step into 1,000 bodies; a 100-link chain; a ragdoll (from milestone 12).

### Ease

- The README's hello world is under 10 lines, and every line is the user's own decision.
- Every scene in the correctness list runs with no setting changed from its default.
- Every call is in `docs/API.md`, with every error it throws, word for word, and an example that
  a test runs.

---

## Milestones

Each one is finished, to the bar below, before the next starts. A milestone with new public API
starts with a proposal Nolan approves; one without starts with a short design note.

**The bar for every milestone.** Tests that would fail without the work. `docs/API.md`, the
types, the README and the CHANGELOG updated in the same change. The errors test, the docs test and
the soak cover anything new. Determinism holds. Off means off, tested. Performance measured where
the milestone touches the step, with A/B blocks in one run. No new dependency, no rendering. CI
green on Node 20, 22 and 24 and on the types.

Status: ✅ done · 🔨 in progress · ⬜ not started

### Part one: bodies and collision

**1. Foundations ✅**
World and Body handles; add, set, read and remove; 53-bit generational ids; Float64 columns,
measured; storage that grows and shrinks; errors, docs, types, soak; bit-identical on every
machine.

**2. Rotation and forces ✅**
Inertia from every shape; tumbling that keeps angular momentum exactly; forces, torques and
impulses.

**3. Narrowphase ✅** *(approved and done 2026-10-13)*
Contacts between every pair of sphere, box and capsule: exact per-pair methods, separating axes and
clipping for boxes, up to 4 points, feature ids, speculative points out to a margin. Internal, no
public API (`src/collide.js`).
*Done when:* hand-worked cases for every pair; thousands of random pairs agree with a brute-force
reference built only from support functions; normal, symmetry, continuity and determinism
invariants hold; cost per pair recorded.
*Result:* `test/collide.test.js` — 33 hand-worked cases; 3,600 random pairs against a reference
that searches the whole sphere of directions, every kink circle and every crossing of two; points
never outside either shape. Found and fixed on the way: boxes apart and nearest at their corners
got no speculative contact (now the true closest points); crossed capsules reported end points
that weren't touching (now kept only within the tolerance of the normal); an edge axis could beat a
face on rounding alone inside a capsule–box overlap (now faces win ties, as between boxes).
Cost per pair on the i7-1260P, Node 22, from `bench/collide.js`: spheres 0.03–0.12 µs,
capsule–capsule 0.4 µs, capsule–box 1.4 µs, box–box 1.6 µs. Box–box is the one to watch: a
10,000-box pile has about 30,000 touching pairs, about 48 ms a step. Caching each pair's separating
axis from the step before is the known remedy; it is milestone 16's to measure.

**4. Contact solver and materials ⬜**
Bodies collide and rest. Sequential impulses with warm starting, and Box2D v3's soft step with
substeps, both built and run against the stack and pyramid scenes; the better one is kept and the
measurement recorded. Friction, restitution, contact tolerance and the speed below which bounces
stop, all derived from size, gravity and step. Kinematic bodies push. Body options `friction`,
`restitution`, `damping`, `gravityScale` *(proposal)*. Bodies are paired in an O(n²) loop, marked as
such, until milestone 5.
*Done when:* every scene in the correctness list that doesn't need sleeping or CCD passes with the
defaults: the box rests, the 10-stack and the pyramid stand, the ball settles, elastic collisions
conserve within tolerance, the marble and the building both behave.

**5. Broadphase and filtering ⬜**
A dynamic AABB tree and sweep-and-prune, both built and timed on 1,000 and 10,000 boxes; the
better one kept. Pairs found incrementally, with no world bounds. Collision layers and masks: a
32-bit membership and filter per body, everything colliding with everything by default
*(proposal)*.
*Done when:* 10,000 boxes pair up in a measured time on both machines; filtering is tested both
ways; the tree is ready for queries.

**6. Islands and sleeping ⬜**
Bodies touching are grouped into islands; an island at rest sleeps; anything that moves it, a
touch, a push, a setter, wakes it, with no `wakeUp()`. Sleep thresholds derived. The moved-bodies
list (rule 63), exact even when a frame runs several steps *(proposal)*. `world.sleeping` to switch
it off.
*Done when:* a resting pile sleeps, a hit wakes it, a sleeping world steps in almost no time, and
the moved list is exact over multi-step frames.

**7. Benchmark ⬜**
`bench/compare.html`: the benchmark scenes through Bearing, Rapier, Jolt and cannon-es, in one
page, taking turns, printing numbers only. Run on both machines. The speed goals above are
confirmed or reset with these numbers.
*Needs Nolan:* OK to load the three libraries (named with versions and sizes first); running it
on the desktop.
*Done when:* both machines' numbers are recorded in `bench/RESULTS.md`, and the goals are
confirmed or reset with Nolan.

### Part two: querying and level content

**8. Queries ⬜**
`world.raycast` (nearest hit), all hits along a ray, shape casts and overlap tests, through the
broadphase tree; results as plain data or `null` *(proposal)*. GJK arrives here for shape casts.
*Done when:* every query matches a brute-force answer on random scenes; 10,000 rays a step are
timed.

**9. Triangle meshes and heightfields ⬜**
Static level geometry: a triangle mesh with its own BVH, and a heightfield. Every primitive
against triangles, with internal edges handled so a sliding box doesn't catch on the seams between
triangles. Meshes read straight from a `.glb`, with no dependency *(proposal)*.
*Done when:* a box slides across a flat mesh of many triangles exactly as across one box; a
sphere rolls over a heightfield; a `.glb` loads.

**10. Contact events and sensors ⬜**
`world.contacts()`: begin, stay and end after each step, as plain data. Sensors: shapes that
report overlaps and push nothing.
*Needs Nolan:* the choice in rule 65, read after the step rather than callbacks. Recommended:
read after the step.
*Done when:* events match the pairs exactly over a scripted scene; removing a body mid-scene gives
its end events; no event fires twice.

### Part three: gameplay

**11. Character controller and rotation locks ⬜**
A kinematic capsule that walks, slides along walls, climbs steps and slopes up to a limit, stays
on the ground going down them, and rides moving platforms. Rotation locks, for upright dynamic
characters and 2.5D games *(proposal)*.
*Done when:* scripted walks over a test level end where hand-worked answers say.

**12. Joints ⬜**
`world.addJoint`, `setJoint`, `jointOf`, `remove`: fixed, ball, hinge, slider, and distance/spring,
with limits and motors *(proposal)*. No joints means the joint solver doesn't run.
*Done when:* a pendulum keeps its period; a 100-link chain hangs without stretching; limits hold;
motors reach their speed; a ragdoll is added to the benchmark.

**13. Continuous collision ⬜**
Time of impact for fast bodies, switched on where speed against size calls for it, from a derived
threshold rather than a setting the user must find *(proposal)*.
*Done when:* a 1 cm ball at 500 m/s hits a 1 cm wall, every time; CCD costs nothing when nothing
is fast.

**14. Convex hulls and compound shapes ⬜**
Hulls built from points (quickhull), reusing milestone 3's separating axes and clipping; compound
bodies of several shapes, with a centre of mass apart from the body's origin *(proposal)*.
*Done when:* hull inertia matches integration; a compound tumbles about its true centre of mass;
hull contacts pass the same reference tests as boxes.

### Part four: 1.0

**15. Save, load and debug lines ⬜**
`world.save()` to plain data and `World.load(data)` back, exact to the bit; `world.debugLines()`
as line segments in a `Float32Array` *(proposal)*.
*Done when:* a saved world, loaded and stepped, matches the original to the bit.

**16. Performance pass ⬜**
Profile every benchmark scene on both machines; every change made for speed comes with its A/B
measurement and a test that results didn't change.
*Done when:* the speed goals are met, or the gap is stated plainly with the reason.

**17. Documentation ⬜**
The README with the measured comparison, its method and machines; `docs/SIDE-BY-SIDE.md`, the same
small program in Bearing, Rapier and cannon-es, each run to check it works; and engine examples in
the docs, showing how Three.js, Babylon.js, PlayCanvas, Pixi and Winding copy `transforms` in.
They are examples, not code in the package.
*Done when:* every example in every doc runs in a test.

**18. 1.0 ⬜**
Every goal above met and measured; the determinism hash checked on Bun as well as Node in CI;
1.0.0 released (see [Versions and releases](#versions-and-releases)).
*Needs Nolan:* the go-ahead for the release.

### After 1.0

- **2D**, as a separate module with the same conventions (rule 106). *Needs Nolan first:* pixel
  units with y down, as Winding's 2D view has, or metres with y up (rule 50).
- **Workers**, switching on by themselves where the page allows, with identical results either way
  (rule 113).
- **Vehicles and ragdoll helpers**, built on joints.
- Not planned: soft bodies, cloth and fluids. They are different solvers, and would be a different
  package.

---

## Demos

Seeing it matters as much as testing it: a stack that passes its test can still look wrong.
From milestone 4, each milestone that changes what bodies do adds or updates a scene in `demo/`:
a page that steps a world and draws it with an engine.

- **Outside the package.** `demo/` is not in `package.json`'s `files`, so it is never published.
  Nothing in `src/` imports it, and no test needs it: demos show, tests decide.
- **Drawn by Winding**, loaded from jsDelivr at a pinned version. It is Nolan's engine and the
  first real user, so the demo doubles as the first check of how an engine reads `transforms`.
  This is the demo's choice, not the package's: Bearing still imports nothing from Winding.
- **What the scenes show** grows with the milestones: the stack and pyramid (4), a pile of
  thousands (5), sleeping bodies tinted (6), rays (8), a level mesh (9), a character walking it
  (11), a chain and a ragdoll (12), bullets against thin walls (13).
- **Needs Nolan:** WebGPU in the browser, since Winding draws with it; a `serve.js` like Winding's
  serves the page, because modules don't load over `file://`.

---

## Versions and releases

Bearing is developed in the open and released as it goes, as Winding was: public from its first
release, through a quick run of 0.x versions, to 1.0 once the goals above are met. Every release,
and making the repository public, still waits for Nolan's go-ahead (rule 8); this section is the
plan, not permission.

### Version numbers

`vMAJOR.MINOR.PATCH`, measured against `docs/API.md`, with Winding's rules:

| Change | Before 1.0 | From 1.0 |
|---|---|---|
| Breaks code written against the API reference | `0.x.0` | `x.0.0` (major) |
| Adds to the API: a call, an option, a feature | `0.x.0` | `1.x.0` (minor) |
| Reworks existing code without changing the API: a fix, a speed-up, a docs correction | `0.x.y` | `1.x.y` (patch) |

Winding did exactly this: 0.2.0 removed five exports and was a minor, while 0.1.1, a correctness
pass that changed no API, was a patch. A rename keeps the old name working, throwing with
directions, for one minor version (rule 47).

### Planned versions

Each milestone that adds API is a minor version. Fixes between them are patches.

| Version | Milestones | What it brings |
|---|---|---|
| 0.1.0 | 1, 2 | Bodies: add, set, read, remove; mass and inertia; forces; tumbling. Nothing collides yet. |
| 0.2.0 | 3, 4 | Things collide, rest and stack: contacts, the solver, materials. |
| 0.3.0 | 5 | Thousands of bodies: the broadphase, layers and masks. |
| 0.4.0 | 6 | Sleeping, and the moved-bodies list. |
| — | 7 | The benchmark changes no package code, so it releases nothing by itself. |
| 0.5.0 | 8 | Queries. |
| 0.6.0 | 9 | Triangle meshes, heightfields, `.glb`. |
| 0.7.0 | 10 | Contact events and sensors. |
| 0.8.0 | 11 | The character controller and rotation locks. |
| 0.9.0 | 12 | Joints. |
| 0.10.0 | 13 | Continuous collision. |
| 0.11.0 | 14 | Convex hulls and compound shapes. |
| 0.12.0 | 15 | Save, load and debug lines. |
| 0.12.x | 16, 17 | The performance pass and the docs: reworks, so patches. |
| 1.0.0 | 18 | Every goal met and measured. |

### How a release is made

Winding's way, copied before the first one:

1. On `develop`, one commit titled `Release x.y.z: <theme>` sets the version in `package.json` and
   moves `## [Unreleased]` in the CHANGELOG under `## [x.y.z] - <date>`, opening with a short
   plain-English paragraph on what it brings.
2. A pull request from `develop` into `main`; `main` takes nothing else.
3. Merging it runs `.github/workflows/release.yml`, which tags `vx.y.z`, writes the GitHub release
   from that CHANGELOG section, and publishes to npm with trusted publishing: no token stored
   anywhere. `.github/release-notes.js` refuses to release a version the CHANGELOG doesn't
   describe, so "bumped" and "written up" are one event.

### Before the first release

- `release.yml` and `release-notes.js`, adapted from Winding's.
- **CI runs only the tests a change reaches**, as Winding's does: `.github/affected.js` follows
  each suite's imports, and the files it reads through `new URL(…, import.meta.url)`, down to
  every file it depends on, and runs only the suites a changed file is in. The types check runs
  only when the declarations or `test/types.ts` change. A change to CI, `package.json` or the
  planner itself runs everything; a change to `ROADMAP.md` or the CHANGELOG runs nothing. It
  comes with its own test, as Winding's `test/affected.test.js`. Nolan wants this in the release
  flow without fail.
- `package.json` loses `"private": true`.
- The `bearing-physics` name exists on npm. npm can only trust a workflow for a package that already
  exists, so the very first publish is by hand, by Nolan, since it needs his 2FA: either a 0.0.0
  placeholder or 0.1.0 itself.
- What Winding has for a public repository: `CONTRIBUTING.md` (branches, tests, what a change
  includes, versions), `SECURITY.md`, `CODE_OF_CONDUCT.md`, and issue and pull request templates.

---

## Log

- **2026-10-13.** Roadmap written, at Nolan's request: fixed goals and a whole plan instead of one
  step at a time. Milestones 1 and 2 were already done. Nolan's line on drawing: nothing in the
  core or `src/` draws; demos may, in `demo/`, outside the package.
- **2026-10-13.** "No public release before 1.0" replaced with Winding's way, at Nolan's word:
  Bearing can be public, as Winding was the whole time, and is released as it goes, 0.x first.
  Versions follow Winding's rules, written out in [Version numbers](#version-numbers), with a
  planned version for each milestone.
