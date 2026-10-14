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

### 10,000-box pile

Not run. Once it lands, Bearing and cannon-es each take a second or more a step, and three runs
of it took most of an hour without finishing. It waits for the performance pass (milestone 16),
when it is worth the time.

### Against the goals

- **Faster than cannon-es on every scene:** not yet. Faster on the pyramid (3.8×) and the stack,
  where cannon-es also lets both fall; 1.4× slower on the 1,000-box pile.
- **Within 2× of Rapier's time per step:** far from it. 3× on the stack, 20× on the pyramid, 14 to
  23× on the pile. Jolt is between: 6× faster than Bearing on the pile.
- **Stacks stand wherever they stand in Rapier and Jolt:** met, and held tightest of the four: the
  stack strayed 0.95 mm in Bearing against 9.5 in Rapier and 20 in Jolt, and the pyramid sank
  7.9 mm against 25 to 27.

## Desktop: Ryzen 5 5500

Not run yet.
