// The types, held to the docs' own examples. Never run, only checked, in CI:
//   npx -y -p typescript@7 tsc --noEmit --strict --target es2022 --module nodenext --lib es2022 test/types.ts
// Each @ts-expect-error is a mistake the types must refuse; tsc fails if one is let through.

import { World, Body } from '../src/bearing.js';
import type { BodyOptions, BodyInfo, Vec3 } from '../src/bearing.js';

export function examples(): void {
  const world = new World();
  const moon = new World({ gravity: [0, -1.62, 0] });
  moon.gravity = new Float32Array([0, -1.62, 0]);
  world.gravity = [0, 0, 0];
  const g: readonly [number, number, number] = world.gravity;
  const count: number = world.bodyCount;

  const ground = world.addBody({ type: 'static', size: [20, 1, 20], position: [0, -0.5, 0], friction: 0.8 });
  world.addBody({ shape: 'sphere', restitution: 0.7, damping: 0.1, angularDamping: 0.2, gravityScale: 0.5 });
  world.substeps = 16;
  const substeps: number = new World({ substeps: 4 }).substeps;
  const bounce: number = world.bodyOf(ground).restitution;
  void [substeps, bounce];
  const marble = world.addBody({ shape: 'sphere', radius: 0.01, density: 2500, position: [0, 1, 0] });
  const player = world.addBody({ shape: 'capsule', radius: 0.3, height: 1.8, mass: 80 });
  const crate: Body = world.addBody();
  world.addBody({});

  world.setBody(crate, { size: [2, 1, 1] });
  world.setBody(crate, { mass: 5 });
  world.setBody(crate, { shape: 'sphere', radius: 1 });
  const info: BodyInfo = world.bodyOf(crate);
  const twin = world.addBody(world.bodyOf(crate));
  if (info.shape === 'box') info.size[0].toFixed();

  const t: Float32Array = world.transforms;
  const p: Float32Array = world.previous;
  const o = marble.slot * 7;
  const position: Vec3 = [t[o], t[o + 1], t[o + 2]];
  marble.setPosition(position[0], 5, 0).setVelocity(2, 0, 0).setRotation([0, 0, 0, 1]);
  player.setEuler(Math.PI / 2, 0).setAxisAngle([0, 1, 0], 1).setAngularVelocity(0, 1, 0);
  const [x, y, z, w] = marble.rotation;
  const mass: number = marble.mass;
  const [ix, iy, iz] = crate.inertia;
  world.setBody(crate, { inertia: [1, 2, 3] });
  world.setBody(crate, { inertia: null });
  world.addBody({ shape: 'sphere', inertia: [2, 2, 2] });
  crate.applyForce([0, 15000, 0]).applyForce([1, 0, 0], [0.5, 0, 0]).applyTorque([0, 200, 0]);
  crate.applyImpulse([0, 0, -500], [2.4, 1, 0]).applyImpulse(new Float32Array(3)).applyAngularImpulse([0, 50, 0]);
  void [ix, iy, iz];
  const alive: boolean = marble.alive;
  const again: Body = world.body(marble.id);
  const same: boolean = again.id === marble.id;
  const home: World = marble.world;

  world.step(1 / 60);
  world.remove(twin);
  ground.destroy();
  world.destroy();
  void [g, count, p, x, y, z, w, mass, alive, same, home];
}

export function mistakes(world: World, body: Body): void {
  // @ts-expect-error a box has no radius: shape: 'sphere' was forgotten
  world.addBody({ radius: 0.5 });
  // @ts-expect-error a sphere has no size
  world.addBody({ shape: 'sphere', size: [1, 1, 1] });
  // @ts-expect-error an unknown option
  world.addBody({ postion: [0, 0, 0] });
  // @ts-expect-error not a type
  world.addBody({ type: 'fixed' });
  // @ts-expect-error position is read only: use setPosition
  body.position = [0, 0, 0];
  // @ts-expect-error ids are read only
  body.id = 3;
  // @ts-expect-error bodies come from addBody, not new
  new Body();
  // @ts-expect-error three numbers, not two
  body.setPosition(1, 2);
  // @ts-expect-error a misspelt material
  world.addBody({ resitution: 0.5 });
  // @ts-expect-error substeps is a number
  world.substeps = '8';
  // @ts-expect-error a force is a vector, not three numbers
  body.applyForce(0, 100, 0);
  // @ts-expect-error inertia is read only: use setBody
  body.inertia = [1, 1, 1];
  // @ts-expect-error transforms are read only
  world.transforms = new Float32Array(0);
  const options: BodyOptions = { shape: 'capsule', height: 2 };
  void options;
}
