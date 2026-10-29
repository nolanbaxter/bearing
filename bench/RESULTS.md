# Benchmark results

From `bench/compare.html`: the same scenes through Bearing, Rapier, Jolt and cannon-es in one
page, taking turns in blocks of 10 steps, medians of 3 runs. 1 m boxes of 1,000 kg, friction 0.5,
restitution 0, steps of 1/60 s. Each engine's own solver settings; cannon-es given its
sweep-and-prune broadphase and sleeping, as its docs advise for many bodies. Lower is better for
ms per step; for a stack, less strayed and sank is better, and "stands" means every box ended
within 10 cm of where it started.

To run it: `node serve.js`, open <http://localhost:8080/bench/compare.html>, press Run, and paste
the result here. `?scene=stack,pyramid,pile` runs those only, `&auto=1` starts at once; in a
browser with less memory to spare, run a scene or two per page load.

## Laptop: Intel i7-1260P, Iris Xe

2026-10-29, on the charger, after contact recycling and per-body freezing: Bearing at commit
`25db6d7`, with workers ('auto') and on one thread (`workers: 0`); Rapier 0.21.0, SIMD build; Jolt
1.1.0, single-threaded; cannon-es 0.20.0. Run in the Claude desktop app's browser pane (Chrome
152), in view and cross-origin isolated, one page load per scene or three; the rays alone, as
sharing a page with the ragdolls slowed every engine's rays 1.5 to 5 times. The 10,000-box pile
ran without cannon-es, which lost the page twice. This replaces a run of the same morning at
`3346a58`, whose 10,000-box pile read twice as slow for every engine.

### 10-box stack, 10 s

| Engine | ms per step | stands | strayed | sank |
|---|---|---|---|---|
| Bearing | 0.0200 | yes | 0.00 mm | 6.57 mm |
| Bearing (1 thread) | 0.0153 | yes | 0.00 mm | 6.57 mm |
| Rapier | 0.0127 | yes | 9.54 mm | 23.9 mm |
| Jolt | 0.00328 | yes | 20.0 mm | 13.1 mm |
| cannon-es | 0.0966 | **no** | 294.9 mm | 49.0 mm |

### 210-box pyramid, 10 s

| Engine | ms per step | stands | strayed | sank |
|---|---|---|---|---|
| Bearing | 0.411 | yes | 0.08 mm | 7.34 mm |
| Bearing (1 thread) | 0.415 | yes | 0.08 mm | 7.34 mm |
| Rapier | 0.0767 | yes | 11.1 mm | 27.3 mm |
| Jolt | 0.0746 | yes | 6.38 mm | 25.3 mm |
| cannon-es | 6.78 | **no** | 320.4 mm | 91.9 mm |

### 1,000-box pile, 10 s

| Engine | ms per step, 0–5 s | ms per step, 5–10 s | asleep at 10 s | fell through |
|---|---|---|---|---|
| Bearing | 6.59 | 4.80 | 101 of 1,000 | 0 |
| Bearing (1 thread) | 12.6 | 8.85 | 101 of 1,000 | 0 |
| Rapier | 2.22 | 1.30 | 1,000 of 1,000 | 0 |
| Jolt | 5.45 | 4.62 | 107 of 1,000 | 0 |
| cannon-es | 21.0 | 22.6 | 451 of 1,000 | 0 |

Bearing's pile is all asleep at about 10.5 s, just past this scene's end (14 s before freezing).

### 10,000 rays into 1,000 boxes

| Engine | ms for all 10,000 | rays that hit |
|---|---|---|
| Bearing | 28.6 | 10,000 |
| Bearing (1 thread) | 28.7 | 10,000 |
| Rapier | 15.4 | 10,000 |
| Jolt | 238 | 10,000 |
| cannon-es | 389 | 10,000 |

### 32 ragdolls, 8 s

| Engine | ms per step, 0–4 s | ms per step, 4–8 s | widest joint gap at the end | asleep at the end | fell through |
|---|---|---|---|---|---|
| Bearing | 7.73 | 5.45 | 0.00 mm | 88 of 352 | 0 |
| Bearing (1 thread) | 7.65 | 5.95 | 0.00 mm | 88 of 352 | 0 |
| Rapier | 1.17 | 0.912 | 7.08 mm | 11 of 352 | 0 |
| Jolt | 2.93 | 0.00123 | 0.10 mm | 352 of 352 | 0 |
| cannon-es | 5.96 | 4.85 | 5.25 mm | 149 of 352 | 0 |

### 1,000-hull pile, 10 s

| Engine | ms per step, 0–5 s | ms per step, 5–10 s | asleep at the end | fell through |
|---|---|---|---|---|
| Bearing | 15.0 | 1.81 | 1,000 of 1,000 | 0 |
| Bearing (1 thread) | 19.7 | 2.16 | 1,000 of 1,000 | 0 |
| Rapier | 3.52 | 0.249 | 1,000 of 1,000 | 0 |
| Jolt | 8.03 | 0.305 | 1,000 of 1,000 | 0 |
| cannon-es | 269 | 381 | 107 of 1,000 | 0 |

### 10,000-box pile, 1 s

| Engine | ms per step, 0–1 s | asleep at the end | fell through |
|---|---|---|---|
| Bearing | 52.0 | 0 of 10,000 | 0 |
| Bearing (1 thread) | 84.8 | 0 of 10,000 | 0 |
| Rapier | 24.9 | 0 of 10,000 | 0 |
| Jolt | 41.5 | 0 of 10,000 | 0 |

### Since 2026-10-22

Bearing on one thread, then and now: the pile 17.5 / 20.6 to 12.6 / 8.85 ms a step, the pyramid
1.16 to 0.415, the ragdolls 9.35 / 8.87 to 7.65 / 5.95, the hull pile 32.8 / 39.5 to 19.7 / 2.16
with all 1,000 hulls asleep by 10 s (40 then), the rays 33.7 to 28.7 ms, the 10,000-box pile 121
to 84.8 ms. With workers the pile's last 5 s are 4.80 ms, Jolt's 4.62, and the 10,000-box pile
52.0 against Jolt's 41.5. Its stack and pyramid stray 0.00 and 0.08 mm (0.88 and 0.96 then). The
other engines' times moved by a few percent.

### Against the goals

- **Faster than cannon-es on every scene:** every scene but the ragdolls, where cannon-es is 1.1
  to 1.3 times faster (4.85 against 5.45 ms in the last 4 s) and holds its joints far looser
  (5.25 mm against 0.00).
- **Within 2× of Rapier's time per step:** met on the stack (1.2× on one thread) and the rays
  (1.9×), nearly on the 10,000-box pile (2.1× with workers); not on the rest: 3 to 4× on the
  box pile, 4 to 7× on the hulls and ragdolls, 5× on the pyramid. Settled scenes are where
  Bearing still loses most: Rapier and Jolt put the pyramid and ragdolls to sleep, Bearing keeps
  solving them (its sleep rule is stricter by design; see the findings of 2026-10-28).
- **Stacks stand wherever they stand in Rapier and Jolt:** met, and held tightest of the four.

## Earlier run

2026-10-22, after milestone 16's single-thread work and the speed audit: Bearing at commit
`65e5136`; Rapier 0.21.0, SIMD build; Jolt 1.1.0, single-threaded; cannon-es 0.20.0. Run in the
Claude desktop app's browser pane (Chrome 152), in view, one page load per scene or two: all of
them in one page ran out of the pane's memory after the box pile, and the 10,000-box pile only
completed for Bearing and Rapier there.

### 10-box stack, 10 s

| Engine | ms per step | stands | strayed | sank |
|---|---|---|---|---|
| Bearing | 0.0324 | yes | 0.88 mm | 6.90 mm |
| Rapier | 0.0144 | yes | 9.54 mm | 23.9 mm |
| Jolt | 0.00424 | yes | 20.0 mm | 13.1 mm |
| cannon-es | 0.167 | **no** | 294.9 mm | 49.0 mm |

### 210-box pyramid, 10 s

| Engine | ms per step | stands | strayed | sank |
|---|---|---|---|---|
| Bearing | 1.16 | yes | 0.96 mm | 8.19 mm |
| Rapier | 0.103 | yes | 11.1 mm | 27.3 mm |
| Jolt | 0.128 | yes | 6.38 mm | 25.3 mm |
| cannon-es | 6.47 | **no** | 320.4 mm | 91.9 mm |

### 1,000-box pile, 10 s

| Engine | ms per step, 0–5 s | ms per step, 5–10 s | asleep at 10 s | fell through |
|---|---|---|---|---|
| Bearing | 17.5 | 20.6 | 107 of 1,000 | 0 |
| Rapier | 2.36 | 1.65 | 1,000 of 1,000 | 0 |
| Jolt | 5.80 | 5.64 | 107 of 1,000 | 0 |
| cannon-es | 23.1 | 29.3 | 451 of 1,000 | 0 |

Bearing's pile is all asleep at about 25 s (milestone 16), past this scene's end.

### 10,000 rays into 1,000 boxes

From above into the 1,000-box pile as it starts, each the nearest hit within 100 m; each engine's
own nearest-hit query.

| Engine | ms for all 10,000 | rays that hit |
|---|---|---|
| Bearing | 33.7 | 10,000 |
| Rapier | 20.7 | 10,000 |
| Jolt | 260 | 10,000 |
| cannon-es | 503 | 10,000 |

### 32 ragdolls, 8 s

11 boxes each, ball joints and hinges without limits (cannon-es's hinge has none), joined parts
not colliding in any engine; two layers of 16, the upper dropped onto the lower.

| Engine | ms per step, 0–4 s | ms per step, 4–8 s | widest joint gap at the end | asleep at the end | fell through |
|---|---|---|---|---|---|
| Bearing | 9.35 | 8.87 | 0.36 mm | 0 of 352 | 0 |
| Rapier | 1.01 | 0.916 | 7.08 mm | 11 of 352 | 0 |
| Jolt | 2.14 | 0.00125 | 0.10 mm | 352 of 352 | 0 |
| cannon-es | 5.49 | 5.19 | 5.25 mm | 149 of 352 | 0 |

### 1,000-hull pile, 10 s

The box pile's places, each a convex hull of the same 12 fixed points about 1 m across, each
engine's own hull from them (Jolt's with no rounding; cannon-es given Bearing's faces, as it can't
build its own), water's density; the four masses agree.

| Engine | ms per step, 0–5 s | ms per step, 5–10 s | asleep at the end | fell through |
|---|---|---|---|---|
| Bearing | 32.8 | 39.5 | 40 of 1,000 | 0 |
| Rapier | 3.39 | 0.280 | 1,000 of 1,000 | 0 |
| Jolt | 7.25 | 0.327 | 1,000 of 1,000 | 0 |
| cannon-es | 258 | 392 | 107 of 1,000 | 0 |

### 10,000-box pile, 1 s

One run of its first second, as the boxes fall and the bottom layers meet the floor. Jolt and
cannon-es didn't complete it in the browser pane (the page was lost, alone or together).

| Engine | ms per step, 0–1 s | asleep at the end | fell through |
|---|---|---|---|
| Bearing | 121 | 0 of 10,000 | 0 |
| Rapier | 27.8 | 0 of 10,000 | 0 |

#### Since 2026-10-14

The same scenes, Bearing's ms per step then and now: the stack 0.0419 to 0.0324, the pyramid 2.21
to 1.16, the box pile 40.2 / 45.0 to 17.5 / 20.6, the rays 72.5 to 33.7 ms, the ragdolls 20.9 /
21.8 to 9.35 / 8.87, the hull pile 44.9 / 52.4 to 32.8 / 39.5. The other three engines' times moved
by a few percent with the browser (headless Chrome 153 then, the app's Chrome 152 now).

#### Against the goals, then

- **Faster than cannon-es on every scene:** every scene but the ragdolls, where cannon-es is 1.7
  times faster (5.5 against 9.4 ms) and holds its joints 15 times looser. The box pile went from
  1.4 times slower to 1.3 to 1.4 times faster; the hulls are 8 to 10 times faster.
- **Within 2× of Rapier's time per step:** not met. 1.6× on the rays, 2.2× on the stack, 4.4× on
  the 10,000-box pile's fall, 7 to 12× on the box pile, 9 to 10× on the ragdolls and the hulls as
  they land, 11× on the pyramid. Jolt is 3 to 4 times faster than Bearing on the pile.
- **Stacks stand wherever they stand in Rapier and Jolt:** met, and held tightest of the four: the
  stack strayed 0.88 mm in Bearing against 9.5 in Rapier and 20 in Jolt; the pyramid 0.96 mm
  against 6 to 11, and sank 8.2 mm against 25 to 27.

## Desktop: Ryzen 5 5500

Not run yet.
