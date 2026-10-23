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
| Gameplay | Character bodies: walk, slide along walls, climb steps and slopes, stand on moving platforms. |
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

Each milestone names the **concepts** it is built from, as Winding's design document did:
Bearing is also how to learn what goes into a physics engine, and these are the ideas to read
up on, in the order the engine needs them.

### Part one: bodies and collision

**1. Foundations ✅**
*Concepts:* struct-of-arrays storage; generational handles; why floating point can be deterministic in JavaScript; amortized growth.
World and Body handles; add, set, read and remove; 53-bit generational ids; Float64 columns,
measured; storage that grows and shrinks; errors, docs, types, soak; bit-identical on every
machine.

**2. Rotation and forces ✅**
*Concepts:* the inertia tensor; angular momentum against angular velocity; the intermediate axis theorem; symplectic integration.
Inertia from every shape; tumbling that keeps angular momentum exactly; forces, torques and
impulses.

**3. Narrowphase ✅** *(approved and done 2026-10-13)*
*Concepts:* broadphase against narrowphase; contact manifolds; the separating axis theorem; polygon clipping; closest points between segments; speculative contacts; feature ids.
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

**4. Contact solver and materials ✅** *(approved and done 2026-10-13)*
*Concepts:* impulses; effective mass; Gauss-Seidel iteration (sequential impulses); warm starting; soft constraints as springs; substepping; the Coulomb friction cone; restitution.
Bodies collide and rest. Sequential impulses with warm starting, and Box2D v3's soft step with
substeps, both built and run against the stack and pyramid scenes; the better one is kept and the
measurement recorded. Friction, restitution, contact tolerance and the speed below which bounces
stop, all derived from size, gravity and step. Kinematic bodies push. Body options `friction`,
`restitution`, `damping`, `gravityScale` *(proposal)*. Bodies are paired in an O(n²) loop, marked as
such, until milestone 5.
*Done when:* every scene in the correctness list that doesn't need sleeping or CCD passes with the
defaults: the box rests, the 10-stack and the pyramid stand, the ball settles, elastic collisions
conserve within tolerance, the marble and the building both behave.
*Result:* `src/contact.js`, held by `test/contact.test.js`'s 13 scenes. The solver is the soft
step: substeps of one solving and one relaxing pass, with soft contacts. Sequential impulses
couldn't hold a 10-box stack at 4 or 8 passes, and every substepped shape could; the table is in
`bench/solver.js`. Measured, not chosen: 8 substeps (the fewest keeping the 210-box pyramid
within 2.5 mm, against 3 mm at 7), contact stiffness an eighth of the substep rate (a quarter rang),
damping ratio 10. Results on the defaults: the pyramid strays 1.5 mm and sinks 8 mm over 20 rows;
the 10-stack strays 0.5 mm; a box on a slope holds at 20° and slides at exactly
g (sin θ − μ cos θ) at 35°; elastic collisions keep momentum to rounding and energy to 1e-9;
bounces keep e² of their height to within 3%; the platform's lag is exactly v²/2μg.
Found and fixed on the way: a box dropped flat slid 7.6 mm, because one solver sweep let the first
corner it visited stop the fall alone and tip the box (now one sweep a point, over a face's points
together: 0.000 mm); bounces came back low, measured from the closing speed at the start of the
step (now at the first push); contact storage never shrank (now the quarter rule). Stated plainly
in the docs: a resting contact sags by g/ω², 0.07 mm at the defaults whatever the body's size,
which is 0.7% of a 1 cm marble.
*Cost to watch:* about 23 ms a step for the pyramid's 211 bodies, two thirds of it the solve loop
itself, which reads and writes each body's velocity per point and multiplies by its inverse
inertia on every push. Box2D keeps a pair's velocities in locals and precomputes the angular parts
once a step; milestone 16 measures doing the same.

**5. Broadphase and filtering ✅** *(approved and done 2026-10-13; the Ryzen run is still to do)*
*Concepts:* bounding volumes; dynamic AABB trees; sweep-and-prune; pair caches; collision filtering with bit masks.
A dynamic AABB tree and sweep-and-prune, both built and timed on 1,000 and 10,000 boxes; the
better one kept. Pairs found incrementally, with no world bounds. Collision layers and masks: a
32-bit membership and filter per body, everything colliding with everything by default
*(proposal)*.
*Done when:* 10,000 boxes pair up in a measured time on both machines; filtering is tested both
ways; the tree is ready for queries.
*Result:* `src/tree.js`, `src/broadphase.js` and `src/pairs.js`, held by
`test/broadphase.test.js`: on a scene of every shape and type, with bodies added, removed,
teleported and refiltered as it runs, the pairs a step touches are exactly those testing every pair
finds, and breaking the broadphase three ways each fails it. The choice was measured in
`bench/broadphase.js` on recorded Bearing motion, replayed into each candidate alone, so later
changes to the solver can't tilt it: a dynamic AABB tree (static bodies in a tree of their own)
against three-axis incremental sweep and prune and a one-axis sort and sweep, over a dense pile,
fast sparse fliers, standing towers and a half-static level. The tree won every scene at 10,000
bodies on the i7-1260P (pile 10.9 ms a step against 22.1 and 31.9; at rest 0.27 against 4.9 and
6.3) and answered box queries 1.4 to 63 times faster. What made it win was found by the bench: a
moving tree's cost is re-filing, so each fat box reaches 16 steps ahead along the body's velocity
(`AHEAD`) as well as a quarter of its size around it (`FAT`); without the reach ahead it only tied
sweep in the pile and was nearly 9 times slower in the level. Layers and masks: 32 bits each,
negative numbers taken as the same bits (Nolan's OK), tested both ways and changed mid-scene. The
10,000-body soak steps again, and tree nodes and pairs come back each round. Found and fixed on
the way: a body re-filed by `setBody` was listed twice.
*Cost to watch:* in a dense pile about one body in 25 is still re-filed each step (415 of
10,000), at a few microseconds each: a remove, an insert and a query; milestone 16 measures growing the leaf and
refitting instead. Bearing's whole step for 10,000 bodies in a pile is about 480 ms, nearly all of
it the solver: the broadphase is about 2% of it today.

**6. Islands and sleeping ✅** *(approved and done 2026-10-13)*
*Concepts:* islands as connected components (union-find); sleep thresholds; waking.
Bodies touching are grouped into islands; an island at rest sleeps; anything that moves it, a
touch, a push, a setter, wakes it, with no `wakeUp()`. Sleep thresholds derived. The moved-bodies
list (rule 63), exact even when a frame runs several steps *(proposal)*. `world.sleeping` to switch
it off.
*Done when:* a resting pile sleeps, a hit wakes it, a sleeping world steps in almost no time, and
the moved list is exact over multi-step frames.
*Result:* in `src/world.js` (the awake list, `_sleep`, `_wake`, `moved()`) and `src/pairs.js`
(awake and asleep runs of the pair list), held by `test/sleep.test.js`'s 14 tests, each of which
fails if its piece is broken. API, as approved: `world.sleeping`, `body.asleep`,
`world.stepCount`, `world.moved(since)`. Every per-body loop walks a list of awake bodies;
sleeping pairs sit past the awake ones, where no step looks; a body falling asleep is re-filed
snug. Islands come from union-find over the contacts, only in steps where some body has been
still long enough. Measured (`bench/sleep.js`, i7-1260P): 10,000 standing boxes step in 425 ms
awake and 3.6 µs asleep; finding islands is 0.3 to 0.4% of all step time. `moved()` reads the awake
list and a log of what stopped, kept no longer than there are bodies; tested exact over frames of
0 to 3 steps through sleeps, wakes, teleports, adds and removes.
Changed from the proposal, with the reason: *still* is how far a body has strayed from where it
came to rest, not how fast it is going. A speed rule kept a 1,000-box heap awake for good: a few
boxes buzzed at 1.3 cm/s, twice the speed that counted as still, while going nowhere (0.18 mm in
30 s). For steady motion the two rules agree. Under it, the time a body must stay still was
measured again: at 0.5 s a cube 0.1° past balance went to sleep instead of tipping; from 1 s,
nothing about to move did.
A heap sleeps only once all of it has stopped: a 384-box heap slept at 61 s, kept by one column
that slid 13 cm over a minute on its edges; with the patches below, which hold such edges, at 10.7 s.
*Closing piece, rolling and spinning* (approved 2026-10-13): nothing slowed a ball or capsule
rolling, or a ball spinning in place on its one point, so one doing either kept awake any heap it
touched (355 of 384 mixed shapes at 150 s). Now every contact is a patch as wide as its tolerance,
from the rolling body's size, and resists rolling (the push leaning to its edge, N times its
radius) and spinning (friction over it, 2/3 mu N times its radius), for contacts of one or two
points. Derived, nothing to set: a rolling resistance of 0.01 for every ball and capsule. Held by
eight tests in `test/contact.test.js` to within 2% of the worked-out figures, at radii from 1 cm
to 5 m. Found on the way: the solver followed a contact through a step by a point fixed on the
body, which on a fast-rolling ball rolled up and away, so the ball seemed to lift and was caught
again each step: a 1 cm marble at 1 m/s hopped a third of its radius and lost 9% of its speed in
5 s. Round shapes are now followed by their core (a ball's centre, a capsule's axis).
SLEEP_TIME re-measured with cubes that really tip: still 1 s.

**7. Benchmark ⬜**
*Concepts:* A/B measurement; variance between runs; comparing engines fairly.
`bench/compare.html`: the benchmark scenes through Bearing, Rapier, Jolt and cannon-es, in one
page, taking turns, printing numbers only. Run on both machines. The speed goals above are
confirmed or reset with these numbers.
*Needs Nolan:* OK to load the three libraries (named with versions and sizes first); running it
on the desktop.
*Done when:* both machines' numbers are recorded in `bench/RESULTS.md`, and the goals are
confirmed or reset with Nolan.
*So far (2026-10-14):* the page is built and the laptop's numbers are in `bench/RESULTS.md`:
stacks stand and hold tightest of the four, but Bearing is 14 to 23 times slower than Rapier
and 1.4 times slower than cannon-es on the 1,000-box pile. The 10,000-box pile is left for the
performance pass: run three times it took most of an hour. The goals are kept as they are, as
targets for milestone 16 (Nolan, 2026-10-14). Still to do: the desktop's run.

### Part two: querying and level content

**8. Queries ✅** *(approved and done 2026-10-14)*
*Concepts:* ray–shape intersection; GJK; shape casts by conservative advancement; tree traversal.
`world.raycast` (nearest hit), all hits along a ray, shape casts and overlap tests, through the
broadphase tree; results as plain data or `null` *(proposal)*. GJK arrives here for shape casts.
*Done when:* every query matches a brute-force answer on random scenes; 10,000 rays a step are
timed.
*Result:* `world.raycast`, `raycastAll`, `shapeCast` and `overlap` (`src/query.js`), with GJK
(`src/gjk.js`) and a ray walk of the tree (`src/tree.js`), held by `test/query.test.js` against
answers worked out without the code they check: rays against marching the ray into each shape by
its options, to 1e-9 m; the world's rays against every body tested alone, through mask, ignore and
maxDistance; GJK against sampling one core against exact distances to the other; shape casts
against marching the shape forward and testing the narrowphase; overlap against the narrowphase on
every body. Breaking the tree walk, the filters or the casts fails them. Direction is normalized,
so distances are metres; a ray starting inside a body meets it at 0. A body knocked out of its fat
box mid-step is still found before the next step, without the query touching the tree, so asking
never changes the simulation. Shape casts by conservative advancement: over 2,000 random casts a
hit took at most 6 advances. 10,000 rays into 1,000 boxes: Bearing 72.5 ms, Rapier 32.6, Jolt 342,
cannon-es 650 (`bench/RESULTS.md`).

**9. Triangle meshes and heightfields ✅** *(approved and done 2026-10-14)*
*Concepts:* triangle BVHs; heightfields; internal edges and ghost collisions; the glTF binary format.
Static level geometry: a triangle mesh with its own BVH, and a heightfield. Every primitive
against triangles, with internal edges handled so a sliding box doesn't catch on the seams between
triangles. Meshes read straight from a `.glb`, with no dependency *(proposal)*.
*Done when:* a box slides across a flat mesh of many triangles exactly as across one box; a
sphere rolls over a heightfield; a `.glb` loads.
*Result:* `addBody({ shape: 'mesh', vertices, indices })`, `addBody({ shape: 'heightfield', heights,
columns, spacing })` and `readGlb(bytes, { node })` (`src/mesh.js`, `src/triangle.js`, `src/glb.js`),
held by `test/mesh.test.js`. Static or kinematic, never dynamic; triangles have two sides. A body against a
mesh touches each triangle under it, and the points of every triangle that pushes the same way
become one manifold, reduced to four, with the points clipping makes on a seam left out, so a box on
a flat floor of 800 triangles gets its four corners, as on one box. Its path across that floor
matches the path across one box to 1.6e-14 m, straight, diagonal, turned, and with its corners
riding exactly along the lines between triangles. A capsule rolls down a heightfield slope as down
a tilted box to 3e-15 m, a ball to 7.6 µm (where a landing rounds to either side of zero gap); a
ball rolls over waves within 0.19 mm of the ground. Seams: an edge is a corner from a side when the
triangle across folds away from it, or there is none; a contact at a seam is left out, the triangle
the body is over holding it, except a ball over this triangle too (to the tolerance), which takes
its face. Two things found on the way: a ball's seam contact taken as the face of a triangle it
wasn't over held it 29 mm off a wave's crest (that plane, carried past the seam, runs above the
ground there); and a box's axes must be tried both ways against a triangle, which isn't symmetric
about its centre as a box is. Every shape against a lone triangle agrees with a brute-force
signed distance on 1,200 random pairs. Warm starting: each mesh pair remembers its manifolds and
matches them by direction. Rays on a mesh of a box's 12 triangles match the box to 1e-12 m, on a
heightfield match marching down to the ground to 1e-9 m; casts and overlaps match every triangle
tested alone. 100,352 triangles build in 0.21 s (i7-1260P, Node 22) and a box on them looks at
16.6 a step. A `.glb` built byte by byte reads back exactly, node transforms, matrices, strips and
fans included. Also fixed: a slow kinematic body fell asleep and stopped (a 10 m platform at
1 cm/s); a kinematic body is now still only when its velocities are zero.

**10. Contact events and sensors ✅** *(approved and done 2026-10-14)*
*Concepts:* contact lifecycles (begin and end); triggers; why callbacks from inside a solver cause re-entrancy bugs.
`world.contacts(body)`: the last step's begin and end events, as plain data, read after the step
(rule 65, Nolan's choice). `world.touching(body)`: what touches a body now, in place of `stay`
events. Sensors (`sensor: true`): bodies that report overlaps and push nothing; they see every body
but other sensors, never two static bodies, and queries don't see them.
*Done when:* events match an independently worked-out touching set over a scripted scene, with no
event twice and every begin closed by one end; removing a body mid-scene gives its end events; a
dropped ball's begin speed is √(2gh); a sensor sees a box in and out and changes nothing about its
path; sleeping keeps touches with no events; events and sensors cost nothing when nothing changes,
measured on the 1,000-box pile.
*Result:* `world.contacts(body)`, `world.touching(body)` and `sensor` (`src/events.js`, with the
touching decision in `src/contact.js`'s `touched()` and sensors asked in `World._senseAll`), held by
`test/events.test.js`. Over a scripted scene (a stack, a slide off a ledge, bouncing boxes and
balls, a paddle knocking balls away, a kinematic platform through a zone, a body replaced in its
own slot, one removed), every step's events matched a touching set worked out from the bodies'
final positions with the narrowphase run afresh, plus the solver's pushes: 3,209 pair-steps, none
too close to call, no event twice. The push matters: in a scene of bouncing boxes and balls the
solver pushed 1,510 contacts that the step left more than a pixel apart, and 1,142 in a tumbling
pile. Five touches in the scene weren't foreseen by the step (a resting ball knocked into another
mid-step) and began at the next one, which the docs say. Removing a body ends its touches at the
next step; a dropped ball begins at 9.892 m/s from 5 m against √(2gh) = 9.905; a sensor saw a box in
and out on the steps worked out from its speed alone, with the box's path bit-identical to the
world without it; a kinematic sensor swept a sleeping pile without waking it. Undoing any of six
pieces (the push, an end on removal, a reused slot's end, sensors waking nothing or pushing
nothing, queries passing them) fails a test. Measured on the 1,000-box pile against the last
commit, alternating in one process: +0.5% and +1.8% landing, +0.5% and +1.9% settled and kept
awake, bit-identical results.

### Part three: gameplay

**11. Character bodies and rotation locks ✅** *(approved and done 2026-10-15)*
*Concepts:* kinematic character movement; move-and-slide; ground detection; step heights and slope limits.
`type: 'character'`: a body moved at the velocity the game gives it, as a kinematic one, that
collides as a person walking does: slides along walls, walks up slopes to `maxSlope` (π/4) and
steps to `stepHeight` (its height ÷ 9), stays on the ground going down them, falls and lands under
gravity, and rides moving bodies. `world.groundOf(body)`: what it stands on, or `null`. Bearing
never sees input. `lockRotation: [x, y, z]` on any body: world axes it never turns about, for
upright dynamic characters and 2.5D games. No `lockPosition`: in-plane contacts keep bodies in
their plane.
*Done when:* scripted walks over a test level end where hand-worked answers say: along a wall at
v·cos 30°; up 15 cm stairs and not a 30 cm riser; up a 30° slope by d·tan 30° and not up 60°; down
slopes and stairs without leaving the ground; carried by a platform; stopped by a ceiling; landing
on the floor. An upright capsule doesn't tip; a 2.5D stack keeps every z to the bit.
*Result:* `type: 'character'`, `maxSlope`, `stepHeight`, `world.groundOf(body)` and `lockRotation`
(`src/character.js`, casts in `src/query.js`'s `sweep()` and `deepest()`, locks in `src/contact.js`'s
`lockedInverse()`), held by `test/character.test.js`, every answer worked by hand: walked 2 m in
1 s at 2 m/s to 1e-9; slid along a wall at 30° at 2 cos 30° m/s, heading along it, to 1e-9; up
15 cm stairs onto the top tread and stopped by the 30 cm riser, both to a pixel (3 mm), and back
down never leaving the ground; a 20 cm character climbs 2 cm and not 3 cm; up a 30° slope at the
speed asked with its feet at x tan 30° + r (1/cos 30° − 1), and stopped at the foot of 60°, to a
pixel; dropped onto 60°, slid down; down 20° at 4 m/s and 44° at 15 m/s without leaving the ground;
carried 2.0000 and 3.0000 m by a 2 m/s platform, and down with a lift; head stopped at a 3 m
ceiling; blocked by a crate it doesn't move; a capsule locked upright doesn't tip and still turns;
a knocked-over 2.5D stack keeps every z and its out-of-plane turn at exactly 0. Found on the way:
a cast in contact must stop only if the move goes deeper than it is by more than rounding (a
floor's normal 2e-14 off up stalled walking); a round bottom meets a stair's lip at a slant, so a
contact too steep to stand on is probed just past for the tread, within a step of the feet; its
own motion is worked out against its platform where the step finds it, then carried, or it rode a
descending lift 1.7 cm up; and moved by the step at constant speed it pushed a crate it only
reached (600 N·s into 1,000 kg), so it is placed before the solve and stands still through it.
Undoing any of six pieces fails a test. Worlds without characters or locks: same results to the
bit, and the same speed within noise (five alternating runs, −5% to +9%).

**12. Joints ✅** *(approved and done 2026-10-15)*
*Concepts:* constraints and their Jacobians; degrees of freedom; limits and motors; mass ratios.
`world.addJoint`, `setJoint`, `jointOf`, `jointCount`, `remove`, and `Joint` handles: fixed, ball,
hinge, slider, and distance (rod, rope or spring), with limits, a ball joint's cone, and motors.
Anchors and axes in world space; a joint's angle, position and motor are `a`'s against `b`, the
world if none. Joined bodies don't collide unless `collide` says so; removing a body removes its
joints. No joints means the joint solver doesn't run.
*Done when:* a pendulum keeps its period; a 100-link chain hangs without stretching; limits hold;
motors reach their speed; a ragdoll is added to the benchmark.
*Result:* `src/joints.js` (rows, limits, motors, springs, the API) and `src/forest.js` (trees solved
exactly), held by `test/joints.test.js`: a 1 m pendulum's period within 0.001% of the formula, with
99.9% of its swing at 20 s; a 100-link chain hangs at its built length from the first step, within
a micrometre, for 10 cm and 1 cm links, and sleeps 1.0 s later; a weight 100 times a link's mass on
a chain of 10 doesn't stretch it (1e-5 m); a bridge of 20 hinged planks fixed at both ends sags and
holds every joint to 0.017 mm; a door slammed at up to 50 m/s at its edge stops within 0.002 mm of
its stop; a slider keeps to its line (1e-9) and its limit; a rope goes slack and is held to its
length within 0.47 mm; a motor reaches 2 rad/s within 1% in three steps and stalls at its force; a
1 Hz spring bounces at 1.0001 s; a ball joint's cone and twist hold; the tree solve agrees with a
dense solve to its compliance. Found on the way, each measured: joints solved one at a time can't
hold a long chain (2% long at once and bobbing at 30 s), harder correction blew it up, and soft
joints, Box2D 3's answer, sag 1.8% for a 10 m chain and 18% for a 1 m one, so trees are solved
exactly (Baraff, 1996), with every joint giving a millionth so a body held to the world at a leaf
can't leave the elimination dividing by nothing; a stop solved as a free body's row pushed a door a
quarter as hard as its hinge needs, so stops and motors are solved against their own joint's pin
(0.9 mm past a stop at 5 m/s before, 0 after); a hinge's or slider's impulse kept as a world vector
picked up a part along its free axis as the joint turned, which spun a ragdoll's shin to blowing
apart, so that part is dropped; and tumbling, right for free bodies, added 10% to a jointed leg's
energy in 50 s, so a jointed body turns plainly and its energy only falls. Ragdolls in
`bench/compare.html` (see `bench/RESULTS.md`). Worlds without joints: the same results to the bit,
and the same speed within noise (−2.0%, +2.5%, +0.3% on the 1,000-box pile).

**13. Continuous collision ✅** *(approved and done 2026-10-15)*
*Concepts:* tunnelling; time of impact; conservative advancement; swept volumes.
Speculative contacts stay, which already stopped fast bodies; a pair not yet touching that moves
against itself by more than its tolerance in the step is first asked when it meets (conservative
advancement, with each body's velocity, gravity and spin). Not this step: no contact. At time t:
its contact is made where the bodies will be then, and the solver lets it close to exactly there.
No ghost hits, no flag, no options. Spin is capped at half a turn a step, spheres excepted.
*Done when:* a 1 cm ball at 500 m/s hits a 1 cm wall, every time, also at angles, on meshes and
heightfields, and against a dynamic box; no ghost hits past a box's edge at 1–300 m/s, and a ball
through a gap wider than it comes out at its speed; a 10 cm rod at 200 m/s spinning at up to
80 rad/s never passes a 2 cm wall; the spin cap gains no energy and milestone 12's thin-box chain
stays together; CCD costs nothing when nothing is fast.
*Result:* `src/impact.js` (time of impact) and the impact contacts in `src/contact.js`, held by
`test/ccd.test.js`: 0 of 60 1 cm balls at 500 m/s through a 1 cm wall at 0°, 30° and 60°, 0 of 20
through a mesh wall, a heightfield or a thin dynamic box, and two meeting head on bounce back; a
ball passing a box's edge 1 cm to 20 cm clear at 1 to 300 m/s keeps its velocity to the bit (before:
slowed or turned by up to 73% of its speed at 5 m/s with 1 cm to spare, and every time at
100 m/s with 20 cm); a ball through a 30 cm gap comes out
at its speed; a 10 cm rod at 200 m/s spinning at 30 to 80 rad/s, 0 of 25 through a 2 cm wall
(before: 28 of 50); a rod spun at 400 rad/s onto another only loses energy, and a 30-link chain of
thin boxes falls 20 s and holds every joint within a millimetre. Found on the way, each measured:
an impact contact must be carried back to the step's start as the solver anchors it (a ball's at
its centre), or a spinning ball's gap read 4 cm wide and two balls met with nothing between; GJK
called a flat simplex "inside", so boxes 0.22 mm apart read touching; capping spin after the solve
undid the spin a corner strike gives a rod, so the cap acts as each substep starts; and an impact
contact needs every point within reach of the rest of the step, not only the first, or a struck rod
swings its other end through. Left as limits, documented: against a mesh or heightfield the contact
is still made where the step starts, so ghost hits off a mesh's edges remain; and a body very much
lighter than two it is wedged between at an angle (225,000:1, contacts 20° apart) can be squeezed
into one for a step. Speed on the settled 1,000-box pile against the last commit: within noise
(+3.9%, +2.5%, −6.2%).

**14. Convex hulls and compound shapes ✅** *(approved and done 2026-10-16)*
*Concepts:* quickhull; mass properties of polyhedra; compound shapes; centres of mass apart from the origin.
`shape: 'hull'` (`vertices`) and `shape: 'compound'` (`parts`: boxes, spheres, capsules and hulls,
each placed and with its own density), `body.centerOfMass`, and a `centerOfMass` option for any
body. `position` and `transforms` stay at the origin; the step works from the centre of mass and
principal axes; `velocity` is the centre of mass's. Neither shape can be a character or a query
shape.
*Done when:* hull mass, centre and inertia exact for a box's corners and a tetrahedron, and within
0.1% of integration for an irregular hull; quickhull's hulls hold every point, face outward and
merge flat faces; hull contacts pass the box tests; a compound tumbles about its true centre of
mass; placement, queries and fast motion work; boxes, balls and capsules unchanged; a hull pile
in the benchmark.
*Result:* `src/hull.js` (quickhull), the mass properties in `src/shapes.js`, hull pairs in
`src/collide.js` and `src/triangle.js`, parts pairs in `src/contact.js`, held by
`test/hulls.test.js`'s 23 tests:
- **Quickhull and mass.** A cube's 8 corners with points inside and on it give 8 corners and 6
  square faces; 300 points on a sphere and 2,000 in a cloud are all inside their hulls, every
  face turns outward, and V − E + F = 2. A hull of a box's corners has the box's mass, centre and
  inertia to 1e-12; a tetrahedron its formulas; an irregular hull of 40 points matches slicing it
  into 4,000 exact cross-sections and summing them to 0.1%; two boxes as a compound sum by the
  parallel-axis theorem exactly.
- **Contacts.** A hull of a box's corners against the box code over 27,000 random poses: the same
  points every time, and within 3.6 mm (the tolerance is 5 mm) where they overlap less than
  2 cm. A stack of 10 such hulls strays and sinks exactly as the boxes; the slope test and the
  edge-balance test pass as the box's do; 48 irregular hulls land on a box, a ball, a capsule, a
  mesh and a heightfield and are all asleep in 20 s.
- **Compounds.** A thrown hammer's centre of mass flies straight to 1e-12 m with its angular
  momentum exact; an L stands with its centre of mass 5 mm inside its foot and tips 5 mm past
  it; a lopsided compound tumbles exactly as a box of its inertia; a dumbbell rolls on its two
  balls, slowing as a ball does, without hopping; a tall box with its centre of mass lowered
  rights itself from 35°, where it falls over without.
- **The rest.** Rays, casts, overlaps and sensors find every part; bodyOf round-trips; a hull
  rod at 200 m/s stops on a 2 cm wall from 50 starts; worlds without hulls give the same results
  to the bit, at +0.2% to +7.1% of the time (median +1.9%, paired per step, a noisy machine).
  The 1,000-hull pile in `bench/compare.html`: 45 to 52 ms a step, 5 to 6 times faster than
  cannon-es and 14 times Rapier's as it lands (`bench/RESULTS.md`).
- **Found on the way.** A box made a hull has four parallel edges along each axis, and the first
  of them won the capsule's and the triangle's edge tests (an edge counts now only if it reaches
  farthest along the axis); and a box given a centre of mass had its shape placed at its world
  position.

### Part four: 1.0

**15. Save, load and debug lines ✅** *(approved and done 2026-10-16)*
*Concepts:* serialisation; exact floating-point round trips; replays.
`world.save()` to plain, JSON-safe data and `World.load(data)` back, exact to the bit: a full
snapshot of everything a step reads, with ids kept, its layout named by `format`.
`world.debugLines(options)` as line segments in a `Float32Array`, nine numbers each (two ends and
a colour): `shapes`, `contacts`, `joints`, `boxes`.
*Done when:* a mixed world saved mid-run, loaded and stepped matches the original to the bit,
directly and through JSON; `save(load(save(w)))` equals `save(w)`; unknown formats and damaged data
throw; debug lines draw every shape kind, contacts and joints; the soak leaves nothing behind.
*Result:* `src/save.js` and `src/debug.js`, held by `test/save.test.js`: a world of a sleeping
pile, bouncing balls, a capsule, a hull, a compound, a jointed chain with a motor, a character, a
sensor, a kinematic platform, a mesh, a heightfield, a fast rod and a waiting force, saved after
150 steps, loads and steps 600 more identically to the bit with the same events, directly and
through JSON, and its next save is the same to the character; every id, `bodyOf`, ground and
raycast comes back. The world is saved by walking its fields, not by a list, so a field added
later is saved too; a test holds the world's field names to a list, so a change also changes
`format`. A world of 1,000 boxes is 3.5 MB of JSON, saved in 36 ms and loaded in 55 (i7-1260P).
Debug lines count right for every shape kind, contacts and joints, reuse one array, and cost
nothing until called; the soak saves, loads and draws 30 times and keeps nothing.

**16. Performance pass and workers ⬜**
*Concepts:* profiling; cache behaviour; writing JavaScript the JIT optimises well; graph colouring;
shared memory and a parallel-for.
Profile every benchmark scene on both machines; every change made for speed comes with its A/B
measurement and a test that results didn't change. Measured candidates, from milestone 4's cost
to watch and from Winding's own ledger: the solver keeping a pair's velocities in locals and
working out its angular terms once a step instead of at every push; a re-filed body's leaf grown
and its ancestors refitted instead of being removed and reinserted, with `FAT` and `AHEAD`
measured again after.
Then workers, as Winding's job system does them (`src/core/jobs.js` there): a parallel-for over
chunks claimed from one atomic cursor, the main thread working too, and the plain loop instead
wherever shared memory is missing, the workers aren't ready yet, or the work is under a measured
size. The same function runs either way, so results are bit-identical with and without workers,
and a test proves the parallel path actually ran. Bounds, the narrowphase and moving bodies split
as they are, since each body or pair is independent. The solver doesn't: each push changes the
velocities the next contact reads. It needs graph colouring first (Box2D 3): contacts grouped so
no two in a group share a moving body, each group solvable in any order, the groups solved in
turn. Colouring changes the solve order on one thread too, so it is measured against the stack and
pyramid before anything else, and lands before 1.0, so 1.0's results are the ones workers
reproduce. In a browser, shared memory needs cross-origin isolation; without it everything still
runs, on one thread. No `init()` and no async factory: until the workers report ready, steps run
on one thread *(proposal for any setting this needs)*.
*Done when:* the speed goals are met, or the gap is stated plainly with the reason. With workers,
a large world steps faster on both machines than on one thread, to the same bits, and a small one
costs no more than before.
*Approved (2026-10-16), in this order:* (1) settled piles go to sleep, cause first; (2) the solver:
bodies gathered into solver arrays, each point's turning terms worked out once a step, a pair's
velocities in locals; (3) the substep count measured again (it must hold the stack, the pyramid
and a heap's drift); (4) no allocation in the step; (5) the tree refitted instead of re-filed, `FAT`
and `AHEAD` measured again; (6) raycasts profiled, cheap fixes only; (7) graph colouring; (8)
workers. Decisions: results may change once at rounding level from reordered arithmetic (every
machine still identical, every physics test holding its figures); stopping a creep the solver
made is accepted within a measured threshold; a `workers` option on `new World()` (automatic by
default, `0` for off, a number to cap them); part A as 0.12.x patches, workers as 0.13.0; the
Rapier goal reset only after measuring, with Nolan's OK; the desktop numbers run by Nolan.
*So far:* (1) done. The benchmark pile's 883 awake bodies were creeping, not buzzing: settled
contacts held at a third to a half of friction's limit yet slid 0.5 to 1.5 mm/s, the solver's
leftover error leaning the same way every step. Friction now carries a contact's slip from step to
step and pulls it back (`store()` in `src/contact.js`), which held the pile's fastest creeper to
1 to 1.5 mm/s where it had reached 8.9. That alone wasn't enough: a body counted as still only
while within a pixel of where it first came to rest, so a box creeping at 2 mm/s was still only
60% of the time, and a few dozen out of step kept the one island of 900 awake. Now its rest pose
is taken again every second it stays still, so "still" is what the docs always said: under a pixel
in any second. Five starts of the pile all sleep, at 13 to 35 s, and cost nothing from then, where
they cost 45 ms a step for good; 18 smaller piles all sleep, the slowest at 17 s. Either fix alone
left a pile awake at 60 s. Measured on the way: 4 substeps let the settled pile drift ten times as
far as 8, so (3) must test a heap's drift too.
(2) done. Each contact point's turning terms (lever x direction, and that through the inverse
inertia, for the normal and both tangents) are worked out once a step in `prepare()` (`J` in
`src/contact.js`); the solve and the warm start hold a pair's velocities in locals for the whole
manifold and write them back once. Measured in one process, pinned to one P-core of the i7-1260P,
three runs: the turning terms 6 to 9% on the landing pile and 9 to 12% on the pyramid; the locals
a further 33% and 41%; the warm start's locals 4 to 5% more. Together: the pile 44 to 25 ms a
step (43% faster), the pyramid 25 to 12 (51%). Every stack, pyramid and drop figure in
`bench/solver.js` is unchanged at every substep count. Measured and left out: turning the normal
into each body's frame once a manifold for the separation, rather than each point's anchors out
(0.5 to 4%, within the 5% the same code reads apart in two module copies). Not done: gathering
the bodies into compact solver arrays. A contact point costs 33% more at 8,000 boxes than at
1,000, so memory does start to tell, but the gather means moving the whole substep (positions,
the separation, the moves) onto those arrays, which graph colouring (7) restructures anyway; it
is done there, once.
(3) done: 8 substeps and four sweeps a face stay. Measured with sleeping off for the stack and
pyramid (10 s), the time three 288-box heaps and the 1,000-box pile take to sleep, and time a
step pinned to one P-core:

| substeps x sweeps | drop slide | stack sank | pyramid stray / sank | heaps asleep | pile asleep | cost |
|---|---|---|---|---|---|---|
| 8 x 4 (kept) | 0 | 6.9 mm | 2.3 / 9.0 mm | 15.0, 14.7, 10.7 s | 18.2 s | 100% |
| 8 x 2 | 0.01 mm | 6.9 mm | 2.6 / 9.0 mm | 12.2, 12.4, 12.8 s | 13.6 s | 84-91% |
| 8 x 1 | 1.5 mm | 6.9 mm | 3.5 / 9.3 mm | 8.6, 12.7, 10.0 s | 15.9 s | |
| 7 x 4 | 0 | 9.0 mm | 4.6 / 12.4 mm | 12.0, 19.0, 16.3 s | awake at 60 s | |
| 6 x 4 | 0 | 12.3 mm | 5.0 / 16.9 mm | 9.1, 10.9, 15.3 s | 25.4 s | 80-84% |
| 4 x 4 | 0 | 27.6 mm | 19.7 / 37.4 mm | 14.0, 13.5, 13.1 s | 17.0 s | |

Fewer substeps soften every contact (its spring is tuned to the substep rate), which shows as
sinking; two sweeps let the pyramid stray 2.4 mm with sleeping on, against 1.0, past the 1.5 mm
`test/contact.test.js` holds it to, and slide the flat drop 0.01 mm. Neither is worth 10 to 15%.
(4) done. A step allocated about 2.3 MB on the landing pile, nearly all V8 boxing: a decimal
passed to or returned from a function V8 doesn't inline becomes a 16-byte heap number. Taking
the largest source out (`prepare()`'s helpers, a third of it) halved what a step allocated and
measured no faster, within 1% either way in three pinned runs: young heap numbers cost V8 almost
nothing to clear. So boxing stays, and the narrowphase isn't reshaped for it. Three copies did make
real objects, a `subarray()` view each, one per body not still every step and two per joint row
every pass; now copied one by one. `test/soak.test.js` steps a world of every feature (the save
test's, now `test/scenes.js`) and fails if a step calls anything that makes an array or a view.
(5) done, and nothing changed. On the landing 1,000-box pile the broadphase, its trees and the pair
list are 2% of a step: the 7.5 pairs listed for each contact cost six comparisons each before the
narrowphase drops them. Refitting a re-filed body in place, rather than re-inserting it, would
speed up the part of the tree's time that re-filing is, 3% of `bench/broadphase.js`'s samples at
10,000 bodies against the queries' 15%, and re-inserting is what keeps queries short; not done.
`FAT` and `AHEAD` measured again (the bench had stopped running: its stand-in world lacked the
sensor and joint columns milestones 10 and 12 added, now there): no other margin beat 1/4; reaching
32 steps ahead instead of 16 made whole steps 8 to 12% faster for 10,000 fliers or a level of
10,000, but 5% slower for a pile of 10,000 and 3 to 13% slower for all three at 1,000. Both stay.
(6) done. `bench/compare.html`'s 10,000 rays into the 1,000-box pile, run the same way in Node:
65 ms, 62% of it walking the tree, which opened child1 before child2 whichever the ray reached
first. Now both children are tested as their parent is opened and the nearer is opened next, with
where the ray enters each kept beside it, so a near hit comes early and everything entered beyond
it is skipped (`raycast()` in `src/tree.js`; shape casts, the catch's sweep and characters walk the
same way). And `ray()` makes its set for telling strays apart only when there are strays. 29 ms,
the same 10,000 hits to the same distances to the last bit; the walk is 29 of the 36 ms saved and
the set 2. Rapier took 32.6 ms in the browser on 2026-10-14; `bench/RESULTS.md` is redone at the
end of part A. Measured and left out: building `world.raycast`'s result once instead of for each
nearer hit (no difference).

**17. Documentation ⬜**
*Concepts:* writing reference documentation people can learn from.
The README with the measured comparison, its method and machines; `docs/SIDE-BY-SIDE.md`, the same
small program in Bearing, Rapier and cannon-es, each run to check it works; and engine examples in
the docs, showing how Three.js, Babylon.js, PlayCanvas, Pixi and Winding copy `transforms` in.
They are examples, not code in the package.
*Done when:* every example in every doc runs in a test.

**18. 1.0 ⬜**
*Concepts:* release engineering; trusted publishing; checking determinism across JavaScript engines.
Every goal above met and measured; the determinism hash checked on Bun as well as Node in CI;
1.0.0 released (see [Versions and releases](#versions-and-releases)).
*Needs Nolan:* the go-ahead for the release.

### After 1.0

- **2D**, as a separate module with the same conventions (rule 106). *Needs Nolan first:* pixel
  units with y down, as Winding's 2D view has, or metres with y up (rule 50).
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
| 0.8.0 | 11 | Character bodies and rotation locks. |
| 0.9.0 | 12 | Joints. |
| 0.10.0 | 13 | Continuous collision. |
| 0.11.0 | 14 | Convex hulls and compound shapes. |
| 0.12.0 | 15 | Save, load and debug lines. |
| 0.12.x | 16, 17 | The performance pass, workers and the docs: reworks, so patches. A setting for workers would be an addition, and 0.13.0. |
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
- **2026-10-13.** Workers moved from "After 1.0" into milestone 16, at Nolan's word, after a survey
  of what Winding does for large loads. Graph colouring comes first and before 1.0, so the solve
  order 1.0 ships with is the one workers reproduce. Also written into 16: solver locals and
  precomputed angular terms, and refitting the tree instead of re-filing.
- **2026-10-13.** Rolling and spinning friction added to milestone 6 as its closing piece, at
  Nolan's word: milestone 6 found that balls and capsules rolling or spinning kept heaps awake for
  good, and rule 35's "a ball bounces and settles" failed for any ball moving sideways.
- **2026-10-14.** Milestone 9 done. Per-corner seam flags (a corner a seam only if every edge at it
  is) were built and then taken out: once a ball's seam contact is only kept over its own triangle,
  every remaining seam rule just leaves a contact out, which a real fold's own triangles keep, and
  no test told the two apart. The test reference for a box against a triangle found a box's axes
  were tried only towards the triangle's centre; now both ways.
- **2026-10-14.** Milestone 10's `stay` events replaced by `world.touching(body)`, at Nolan's word:
  a stay event fires for every touching pair every step, about 30,000 a step for a resting pile of
  10,000 boxes, and sleeping pairs, which nothing looks at, would have to be walked to report them.
  What stay is for ("is the player on the ground?") is a question, asked when wanted. Box2D 3 made
  the same choice. Events are read after the step, not called back (rule 65).
- **2026-10-14.** Milestone 10 done. Touching was planned as "pushed in the step, or ended it within
  a pixel"; for a moment the push looked redundant (bounces are applied after the substeps, so a
  ball ends its impact step at the surface), and measuring settled it: bodies knocked away by
  others within a step are pushed yet end it apart, about one contact in 30 in a bouncing scene.
  So both stay, and the test scene has bouncing boxes and a paddle to hold the push to it.
- **2026-10-15.** Milestone 11 renamed from "Character controller" to "Character bodies", with
  Nolan: "controller" is the industry's word (Unity, PhysX, Rapier, Jolt), but it reads as input
  handling, and nothing here touches input. A character is a body type the game gives a velocity
  to, as it does a kinematic one.
- **2026-10-15.** Milestone 12 done, with three changes to its proposal. The proposal said solving a
  ball joint's rows as one 3x3 block would keep long chains tight; measured, it didn't, and the
  joints that form trees are now solved exactly together (`src/forest.js`), which a long chain
  needs. A distance joint's anchors default to the two centres, as the proposal's rope example
  ("as long as they are apart now") needed, where its table said midway. And a joint's free
  coordinate is `a`'s against `b`, so a motor on a body hung from the world turns it the way its
  speed says. Found and not fixed, for Nolan to decide: a thin box spinning faster than about
  200 rad/s gains energy from its contacts (a 0.1 m rod at 400 rad/s ends at 9,600), because a
  step's contact points and normal are found where the step starts; a chain of thin boxes let go
  sideways reaches that whipping, and blows up. Box2D 3 and Jolt cap spin at a quarter turn a step
  (47 rad/s at 60 Hz) for this; a cap of a half turn stopped every case measured. It is milestone
  13's (fast motion), or sooner.
- **2026-10-15.** Milestone 13 done, as approved: time of impact for pairs moving against
  themselves by more than a pixel, and a spin cap of half a turn a step, spheres excepted. The
  events test's "a step late" now covers a spinning box meeting with a corner its contact didn't
  hold, and its line between touching and not is as wide as a fixed normal's error for two round
  bodies sliding past each other (d² over their size); both were in the code before, and the new
  paths of fast bodies found them. Mesh and heightfield pairs keep their speculative contacts: a
  time of impact against each triangle is a bigger change, for when a game needs it.
- **2026-10-16.** Milestone 14 done, as approved with the `centerOfMass` option. Found and not
  fixed, for Nolan to decide:
  - Milestone 13's rod tests only flew rods lined up with their flight, from starts inside one
    step's travel. A 10 cm rod at 200 m/s turned any way goes through a 1 cm wall 52 times in 100
    (in milestone 13's code too); lined up and spinning, 8 in 50 when the starts cover a whole
    step. Struck on a corner, the rod is spun hard and pivots its far end through the wall within
    the step, where its contact, the face that met, doesn't reach.
  - Tumbling's energy can rise: a body spun mostly about its longest or shortest axis gains 2e-4
    to 4e-4 of its energy in 120 s. Momentum stays exact, which bounds it; "energy only falls" is
    true only near the middle axis, where `test/motion.test.js` spins its bodies.
- **2026-10-22.** The competition studied (Rapier, Jolt, Box2D v3 and Box3D, PhysX, Bullet,
  cannon-es; sources in the session's research): each solves a contact point's normal 5 to 12
  times a step where Bearing solved it 64 (8 substeps x 2 passes x 4 sweeps), which is most of
  the gap and of Bearing's tighter stacks, and spends 2 to 4 friction rows on a box face where
  Bearing spent 8. Three of their techniques built as experiments and measured against the code
  of the day, pinned to one P-core: friction once a manifold (box pile 8% faster, pyramid 11-12%,
  quality as good or better; landed, Nolan approved), a pair's manifold reused while its pose has
  barely changed (the hull pile 27% faster, boxes no faster; to land for hull pairs only), and a
  manifold's normal points solved together as one small LCP (pile 8-10%, pyramid 10-12%, but it
  exposed contact ids flickering on exactly aligned stacks; to measure again once that is fixed).
  Sleeping sooner measured and not taken: 0.5 s froze slow starters (a box 0.1 deg past friction's
  hold, cubes 0.45-0.5 deg past balance) and didn't make heaps sleep sooner, since a heap is still
  genuinely settling until about then; a `sleepTime` setting is not added (Nolan agreed).
  Certified sleeping (sleep once friction sticks and the body is supported) measured first and not
  built: the benchmark pile is still collapsing, with hundreds of contacts sliding, until about
  19 s and asleep at 20, so it could save about a second.
- **2026-10-22.** `bench/compare.html` run again on the laptop after part A and the audit, results in
  `bench/RESULTS.md`: Bearing now faster than cannon-es on every scene but the ragdolls (1.7 times
  slower there), the box pile 2.2 to 2.3 times faster than on 2026-10-14; against Rapier 1.6x on
  rays to 11x on the pyramid, so the 2x goal is not met. Run one scene per page load: the browser
  pane ran out of memory with all of them in one, and lost the 10,000-box pile for Jolt and
  cannon-es. Still to do: the desktop's run.
- **2026-10-22.** Speed audit, at Nolan's request, between milestone 16's parts A and B: every
  file read for speed by five reviewers, each finding timed against the code before it (one
  process, alternating blocks, pinned to one P-core), kept only if faster and only if bit-identical
  (one exception, the forest's reused products, at rounding level). The base math (`core/math.js`)
  was found as fast as it can be and stay bit-identical: fdlibm's own formulas, operation for
  operation, and under 1% of any scene. Kept, against the code before the audit: 1,000 hulls piling
  12.9% faster (a hull placed once a step and shared by its pairs; the edge test's dot products
  shared), 300 hulls on a heightfield 14.3% (also one pass over the hull's corners per axis, not
  three), 30 swinging chains 15.9% (the forest's loops tightened, a 6x6 inverse written out, the
  factor's products reused), 1,000 kinematic bodies moved by hand 9.8%, the landing pile 4.8% and the
  pyramid 4.2% (pairs checked again only after something could change them; the slip measure's turn
  once a manifold; box-box's face products shared), terrain 1.5%; adding 2,000 copies of one rock
  68 -> 16 ms (the last hull built is shared when the same numbers come again), finding a
  131,072-triangle mesh's seams 110 -> 31 ms (typed hash tables, not a Map of strings). Fixed on the
  way: the catch read a stray list from before the step's motion whenever a character existed;
  seam keys overlapped past 2^21 vertices; a closure per step in the joints; a removal stretched a
  list to every slot. Measured and left out: `rotate`'s sign as a branch, merging the step's walks,
  joint frames kept for the substep, a heightfield's edge flags kept, query internals' allocations.
  Not tried, for later if a scene needs them: SAT caching for hulls, the forest keeping its Cholesky
  factor rather than inverses, building a mesh's tree in bulk. Open, for Nolan: `events.js` hands
  storage back by the quarter rule each step, so event counts swinging past 64 reallocate inside
  the step.
- **2026-10-22.** Milestone 15's save size corrected: the 3.5 MB quoted for 1,000 boxes didn't
  hold for the benchmark's turned pile, which was 9.5 MB before milestone 16 and is now about
  7 MB settled and 10 to 11 MB while landing. The API reference now names the scene.
- **2026-10-16.** Milestone 15 done, as approved. The save is larger than proposed (3.5 MB of JSON
  for 1,000 boxes, not about 1): every number a step reads is in it, a body's axes, rest pose,
  bounds and both transforms among them, and each is needed for the loaded world to step to the
  bit.
- **2026-10-16.** Milestone 13's gap fixed, the option Nolan chose of three measured (a quarter-turn
  spin cap left 53 of 100 turned rods through a 1 cm wall; re-finding contacts each substep, 31):
  after the solve, each body whose centre moved more than half its thinnest size sweeps a core, a
  ball at its centre a quarter of that size across, along its path, against everything but other
  fast bodies as they end the step (Box2D 3's continuous step, with slow dynamic bodies as targets,
  which a thin plate floating free needs: 47 through in 100 without). Caught, a body goes back to
  where its core met, turned that far, its velocity into the surface returned as restitution
  would; kept, a rod caught at a mesh drove on in and crossed it the next step. The whole shape
  swept instead caught a spinning rod mid-bounce and gained it 27% of its energy. Now none of 100
  through for rods turned, spinning or lined up, 1 cm cubes at 500 m/s and 1 m planks, nor through
  a mesh, a heightfield or the plate; the settling pile is unchanged (+0.1%, to the bit), and 1,000
  marbles falling, every one fast every step, cost 5.8% more a step. Milestone 13's rod test now
  starts over a whole step's travel. The events test's line for curved contacts now includes a
  ball on a box's edge, which a ball rolling off the floor met.
- **2026-10-16.** Tumbling's energy rise fixed, at Nolan's word ("just fix that"): a body tumbling
  freely that would end a substep with more energy than it had when last spun by anything else is
  turned back onto it, keeping its momentum exact. The off-axis cases now hold their energy to
  1e-15 over 120 s; the middle-axis cases come out exactly as before. Checked against each
  substep's own start instead, it ratcheted energy away (14% in 120 s at 50 rad/s).
