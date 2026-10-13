// How many substeps keep things standing, for what they cost.
// Run: node bench/solver.js
//
// The default (World's SUBSTEPS) is the fewest that keep a 210-box pyramid
// within half a box's contact tolerance, 2.5 mm, after 10 s at 60 Hz. Run this
// to check it still is, after any change to the solver or the narrowphase.
//
// HOW THE SOLVER WAS CHOSEN (milestone 4, i7-1260P, Node 22). The solver runs
// substeps of one solving pass and one relaxing pass. Before that was settled,
// substeps and passes per substep were both dials, so sequential impulses (one
// substep, several passes, as Box2D 2) and the soft step (several substeps, one
// pass, as Box2D 3) were the same code at the same cost, differing only in shape:
//
//   substeps x passes   stack stray / sank       pyramid stray / sank
//   1 x 4               1663 / 263 mm  (fell)    267 / 131 mm
//   1 x 8                317 / 116 mm  (fell)    148 / 124 mm
//   2 x 2                 30 /  27 mm             17 /  34 mm
//   4 x 1                 20 / 6.9 mm             60 /  11 mm
//   4 x 2                2.2 / 6.9 mm            4.4 / 8.4 mm
//   8 x 1               0.25 / 1.7 mm             20 / 3.0 mm, still moving
//
// Substeps won outright. The contact's stiffness was then measured: a quarter
// of the substep rate rang with one pass (the pyramid still moving 0.13 m/s at
// 10 s), an eighth settled it. The table below is with an eighth, as shipped.
//
// Scenes, 60 Hz, defaults otherwise:
//   drop     a 1 m box falls 2.5 m flat onto the ground; how far it slides
//   stack    ten 1 m boxes, one on another, for 10 s
//   pyramid  20 boxes on the bottom row, 210 in all, for 10 s
// For the stack and pyramid: how far any box strayed sideways, how far the top
// sank, and the fastest any box was still moving at the end.

import { World } from '../src/bearing.js';

const SUBSTEPS = [4, 5, 6, 7, 8, 10, 12];

function world(substeps) {
  const w = new World({ substeps });
  w.addBody({ type: 'static', size: [100, 1, 100], position: [0, -0.5, 0] });
  return w;
}

function drop(substeps) {
  const w = world(substeps);
  const box = w.addBody({ position: [0, 3, 0] });
  for (let k = 0; k < 240; k++) w.step(1 / 60);
  return Math.hypot(box.position[0], box.position[2]) * 1000;
}

function settle(w, boxes, seconds) {
  const start = boxes.map((b) => b.position);
  let ms = 0;
  for (let k = 0; k < 60 * seconds; k++) {
    const t0 = performance.now();
    w.step(1 / 60);
    ms += performance.now() - t0;
  }
  let stray = 0, fastest = 0;
  boxes.forEach((b, k) => {
    const p = b.position;
    stray = Math.max(stray, Math.hypot(p[0] - start[k][0], p[2] - start[k][2]));
    fastest = Math.max(fastest, Math.hypot(...b.velocity));
  });
  const sank = start[boxes.length - 1][1] - boxes[boxes.length - 1].position[1];
  return { stray: stray * 1000, sank: sank * 1000, fastest, ms: ms / (60 * seconds) };
}

function stack(substeps) {
  const w = world(substeps);
  const boxes = [];
  for (let k = 0; k < 10; k++) boxes.push(w.addBody({ position: [0, 0.5 + k, 0] }));
  return settle(w, boxes, 10);
}

function pyramid(substeps) {
  const w = world(substeps);
  const boxes = [];
  for (let row = 0; row < 20; row++) {
    for (let k = 0; k < 20 - row; k++) boxes.push(w.addBody({ position: [k - (19 - row) / 2, 0.5 + row, 0] }));
  }
  return settle(w, boxes, 10);
}

const f = (x, d = 2) => x.toFixed(d).padStart(8);
console.log('Distances in mm; speed in m/s; time per step in ms.');
console.log('substeps |    drop | stack: stray    sank fastest | pyramid: stray    sank fastest ms/step');
for (const s of SUBSTEPS) {
  const d = drop(s), st = stack(s), py = pyramid(s);
  console.log(`${String(s).padStart(8)} |${f(d)} |${f(st.stray)}${f(st.sank)}${f(st.fastest, 4)} |${f(py.stray)}${f(py.sank)}${f(py.fastest, 4)}${f(py.ms)}`);
}
