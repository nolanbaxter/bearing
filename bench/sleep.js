// What sleeping saves, and what finding islands costs.
// Run: node bench/sleep.js   (about three minutes)
//
// Scenes, 60 Hz, defaults otherwise:
//   towers  1,000 stacks of ten 1 m boxes, 10,000 bodies, standing from the
//           start: a step while awake, and once all are asleep
//   boxes   384 boxes dropped in a heap: when it is all asleep, and what
//           counting still time and finding islands cost, as a share of all
//           the steps' time and at its slowest. (Its share of one step means
//           little: with six bodies awake a step is a few microseconds.)
//   mixed   the same with spheres and capsules
//
// RESULTS (i7-1260P, Node 22, 2026-10-13), the run that closed milestone 6:
//   towers  awake 425 ms a step; asleep 3.6 µs, about 1/118,000 of it
//   boxes   still time and islands 0.27% of all step time, 1 ms at the
//           slowest; all asleep at 61 s. It took that long because one
//           column in it really was still sliding, 13 cm over a minute with
//           sleeping off, and a heap sleeps only when all of it has stopped.
//   mixed   0.40% of all step time; 355 of 384 still awake at 150 s. Nothing
//           in Bearing yet slows a ball or capsule rolling on the floor, or a
//           ball spinning in place on one point, and one touching the heap
//           keeps all of it awake.

import os from 'node:os';
import { World } from '../src/bearing.js';

const median = (xs) => [...xs].sort((a, b) => a - b)[xs.length >> 1];
console.log(`Sleeping: ${os.cpus()[0].model.trim()}, Node ${process.versions.node}\n`);

{
  const world = new World();
  world.addBody({ type: 'static', size: [100, 1, 100], position: [0, -0.5, 0] });
  for (let i = 0; i < 1000; i++) {
    const x = (i % 32) * 2 - 32, z = Math.floor(i / 32) * 2 - 32;
    for (let k = 0; k < 10; k++) world.addBody({ position: [x, 0.5 + k, z] });
  }
  const awake = [];
  for (let i = 0; i < 10; i++) { const t = performance.now(); world.step(1 / 60); awake.push(performance.now() - t); }
  let n = 0;
  while (world._awakeCount > 0 && n++ < 600) world.step(1 / 60);
  const asleep = [];
  for (let i = 0; i < 200; i++) { const t = performance.now(); world.step(1 / 60); asleep.push(performance.now() - t); }
  const a = median(awake), s = median(asleep);
  console.log(`towers  ${world.bodyCount - 1} bodies: awake ${a.toFixed(0)} ms a step; asleep ${(s * 1000).toFixed(1)} µs, 1/${Math.round(a / s).toLocaleString('en')} of it`);
}

const SECONDS = 150;
for (const mixed of [false, true]) {
  const world = new World();
  world.addBody({ type: 'static', size: [200, 1, 200], position: [0, -0.5, 0] });
  let n = 0;
  for (let layer = 0; layer < 6; layer++) for (let x = 0; x < 8; x++) for (let z = 0; z < 8; z++) {
    const k = mixed ? n % 3 : 0, at = [x * 1.25 + Math.sin(++n * 12.9898) * 0.3, 1.5 + layer * 1.4, z * 1.25 + Math.cos(n * 78.233) * 0.3];
    if (k === 0) world.addBody({ size: [0.9, 0.6, 0.7], position: at });
    else if (k === 1) world.addBody({ shape: 'sphere', radius: 0.4, position: at });
    else world.addBody({ shape: 'capsule', radius: 0.25, height: 1, position: at });
  }
  const sleep = world._sleep;
  let spent = 0;
  world._sleep = function (dt) { const t = performance.now(); sleep.call(this, dt); spent = performance.now() - t; };
  let slowest = 0, sleeping = 0, stepping = 0, allAsleep = null, i = 0;
  while (++i <= SECONDS * 60 && allAsleep === null) {
    const t = performance.now();
    world.step(1 / 60);
    stepping += performance.now() - t;
    sleeping += spent;
    slowest = Math.max(slowest, spent);
    if (world._awakeCount === 0) allAsleep = i / 60;
  }
  const share = (sleeping / stepping * 100).toFixed(2), at = (slowest * 1000).toFixed(0);
  const end = allAsleep === null ? `${world._awakeCount} still awake at ${SECONDS} s` : `all asleep at ${allAsleep.toFixed(1)} s`;
  console.log(`${mixed ? 'mixed' : 'boxes'}   ${world.bodyCount - 1} bodies: still time and islands ${share}% of all step time, ${at} µs at the slowest; ${end}`);
}
