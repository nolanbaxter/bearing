# Benchmark results

From `bench/compare.html`: the same scenes through Bearing, Rapier, Jolt and cannon-es in one
page, taking turns in blocks of 10 steps, medians of 3 runs. 1 m boxes of 1,000 kg, friction 0.5,
restitution 0, steps of 1/60 s. Each engine's own solver settings; cannon-es given its
sweep-and-prune broadphase and sleeping, as its docs advise for many bodies. Lower is better for
ms per step; for a stack, less strayed and sank is better, and "stands" means every box ended
within 10 cm of where it started.

To run it: `node serve.js`, open <http://localhost:8080/bench/compare.html>, press Run, and paste
the result here. `?scene=stack,pyramid,pile` runs those only.

## Laptop: Intel i7-1260P, Iris Xe

2026-10-14, headless Chrome 153. Versions: Bearing at commit `933f706`; Rapier 0.21.0, SIMD build;
Jolt 1.1.0, single-threaded; cannon-es 0.20.0.

### 10-box stack, 10 s

| Engine | ms per step | stands | strayed | sank |
|---|---|---|---|---|
| Bearing | 0.0419 | yes | 0.95 mm | 6.90 mm |
| Rapier | 0.0142 | yes | 9.54 mm | 23.9 mm |
| Jolt | 0.00356 | yes | 20.0 mm | 13.1 mm |
| cannon-es | 0.128 | **no** | 294.9 mm | 49.0 mm |

### 210-box pyramid, 10 s

| Engine | ms per step | stands | strayed | sank |
|---|---|---|---|---|
| Bearing | 2.21 | yes | 1.67 mm | 7.94 mm |
| Rapier | 0.111 | yes | 11.1 mm | 27.3 mm |
| Jolt | 0.115 | yes | 6.38 mm | 25.3 mm |
| cannon-es | 8.34 | **no** | 320.4 mm | 91.9 mm |

### 1,000-box pile, 10 s

| Engine | ms per step, 0–5 s | ms per step, 5–10 s | asleep at 10 s | fell through |
|---|---|---|---|---|
| Bearing | 40.2 | 45.0 | 92 of 1,000 | 0 |
| Rapier | 2.92 | 1.96 | 1,000 of 1,000 | 0 |
| Jolt | 7.16 | 6.99 | 107 of 1,000 | 0 |
| cannon-es | 28.0 | 32.4 | 451 of 1,000 | 0 |

### 10,000 rays into 1,000 boxes

From above into the 1,000-box pile as it starts, each the nearest hit within 100 m; each engine's
own nearest-hit query. 2026-10-14, with milestone 8's queries.

| Engine | ms for all 10,000 | rays that hit |
|---|---|---|
| Bearing | 72.5 | 10,000 |
| Rapier | 32.6 | 10,000 |
| Jolt | 342 | 10,000 |
| cannon-es | 650 | 10,000 |

### 32 ragdolls, 8 s

Added with milestone 12 and run at its commit, 2026-10-15, one run. 11 boxes each, ball
joints and hinges without limits (cannon-es's hinge has none), joined parts not colliding in any
engine; two layers of 16, the upper dropped onto the lower.

| Engine | ms per step, 0–4 s | ms per step, 4–8 s | widest joint gap at the end | asleep at the end | fell through |
|---|---|---|---|---|---|
| Bearing | 20.9 | 21.8 | 0.44 mm | 0 of 352 | 0 |
| Rapier | 1.74 | 1.48 | 7.08 mm | 11 of 352 | 0 |
| Jolt | 3.99 | 0.00208 | 0.10 mm | 352 of 352 | 0 |
| cannon-es | 9.84 | 8.76 | 5.25 mm | 149 of 352 | 0 |

Bearing holds its joints tighter than Rapier and cannon-es, and is the slowest: 12 to 15 times
Rapier, 2.1 to 2.5 times cannon-es. Its ragdolls don't sleep in 8 s: with no limits and nothing to
rub on, a dangling limb swings as a pendulum does; Jolt's sleeping is the quickest to call a slow
swing still.

### 1,000-hull pile, 10 s

Added with milestone 14 and run at its commit, 2026-10-16, one run. The box pile's places, each a
convex hull of the same 12 fixed points about 1 m across, each engine's own hull from them (Jolt's
with no rounding; cannon-es given Bearing's faces, as it can't build its own), water's density;
the four masses agreed.

| Engine | ms per step, 0–5 s | ms per step, 5–10 s | asleep at the end | fell through |
|---|---|---|---|---|
| Bearing | 44.9 | 52.4 | 31 of 1000 | 0 |
| Rapier | 3.19 | 0.229 | 1000 of 1000 | 0 |
| Jolt | 6.77 | 0.276 | 1000 of 1000 | 0 |
| cannon-es | 232 | 338 | 107 of 1000 | 0 |

Bearing is 5 to 6 times faster than cannon-es here, and 14 times Rapier's time as the pile lands.
Afterwards Rapier and Jolt have put the whole pile to sleep and Bearing hardly any of it, as in the
box pile: what Bearing pays for most of the second half is keeping a still pile awake.

### 10,000-box pile

Not run. Once it lands, Bearing and cannon-es each take a second or more a step, and three runs
of it took most of an hour without finishing. It waits for the performance pass (milestone 16),
when it is worth the time.

### Against the goals

- **Faster than cannon-es on every scene:** not yet. Faster on the pyramid (3.8×) and the stack,
  where cannon-es also lets both fall; 1.4× slower on the 1,000-box pile.
- **Within 2× of Rapier's time per step:** far from it. 3× on the stack, 20× on the pyramid, 14 to
  23× on the pile. Jolt is between: 6× faster than Bearing on the pile. Raycasts are nearest:
  2.2× Rapier's time, and 4.7 times faster than Jolt's.
- **Stacks stand wherever they stand in Rapier and Jolt:** met, and held tightest of the four: the
  stack strayed 0.95 mm in Bearing against 9.5 in Rapier and 20 in Jolt, and the pyramid sank
  7.9 mm against 25 to 27.

## Desktop: Ryzen 5 5500

Not run yet.
