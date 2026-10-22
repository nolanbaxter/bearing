// Scenes more than one test file steps.

import { World } from '../src/bearing.js';

/**
 * A world with a bit of everything: a pile that falls asleep, bouncing
 * balls, capsules, a hull and a compound, a chain of joints, a character
 * walking, a sensor, a kinematic platform, a mesh and a heightfield, a fast
 * rod, and a force waiting.
 */
export function mixed(options = {}) {
  const world = new World(options);
  world.addBody({ type: 'static', size: [30, 1, 30], position: [0, -0.5, 0] });
  world.addBody({ shape: 'mesh', vertices: [20, 0, -5, 30, 2, -5, 30, 2, 5, 20, 0, 5], indices: [0, 1, 2, 0, 2, 3], position: [-4, 0.01, 0] });
  world.addBody({ shape: 'heightfield', heights: [0, 0.2, 0, 0.3, 0.5, 0.1, 0, 0.2, 0], columns: 3, spacing: 2, position: [-10, 0.01, 8] });
  for (let i = 0; i < 12; i++) world.addBody({ position: [3, 0.5 + i * 1.01, 3] });
  for (let i = 0; i < 6; i++) world.addBody({ shape: 'sphere', radius: 0.3, restitution: 0.7, position: [-3 + i, 3 + i, -2], velocity: [1, 0, 0.5] });
  world.addBody({ shape: 'capsule', radius: 0.25, height: 1.2, position: [0, 4, 0], angularVelocity: [2, 0, 1] });
  world.addBody({ shape: 'hull', vertices: [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0.6, 0.6, 0.6], position: [5, 2, -4] });
  world.addBody({ shape: 'compound', parts: [{ size: [0.6, 0.2, 0.2] }, { shape: 'sphere', radius: 0.2, position: [0.4, 0, 0] }], position: [-5, 3, 5], angularVelocity: [0, 3, 1] });
  let last = null;
  for (let i = 0; i < 6; i++) {
    const link = world.addBody({ size: [0.1, 0.4, 0.1], position: [8, 6 - 0.4 * (i + 0.5), 0] });
    world.addJoint(last ? { type: 'ball', a: link, b: last, anchor: [8, 6 - 0.4 * i, 0] } : { type: 'hinge', a: link, anchor: [8, 6, 0], axis: [0, 0, 1], motorSpeed: 1, motorForce: 5 });
    last = link;
  }
  const walker = world.addBody({ type: 'character', shape: 'capsule', radius: 0.3, height: 1.6, position: [-6, 0.8, -6], velocity: [1.5, 0, 0] });
  world.addBody({ type: 'static', sensor: true, size: [2, 2, 2], position: [-3, 1, -6] });
  world.addBody({ type: 'kinematic', size: [2, 0.2, 2], position: [6, 0.5, 6], velocity: [0, 0, -1] });
  world.addBody({ size: [0.1, 0.02, 0.02], position: [-12, 1, 12], velocity: [200, 0, 0], rotation: [0.2, 0.3, 0.1, 1] });
  world.addBody({ type: 'static', size: [0.01, 4, 4], position: [-8, 2, 12] });
  return { world, walker };
}
