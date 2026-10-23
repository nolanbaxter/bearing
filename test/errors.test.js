// Every error, word for word, both ways round.
// Run: node test/errors.test.js
//
// Each case below makes one call throw and names the API entry that documents
// it. The test checks three things: the message is exactly the one expected;
// that message matches a pattern in its entry's "Throws:" line (so nothing
// throws that the reference doesn't list); and every pattern in every entry
// is matched by some case (so nothing is listed that is never tested). In a
// pattern, `…` and `<…>` stand for a value.

import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { World, readGlb } from '../src/bearing.js';
import { suite, test, thrown } from './test.js';

suite('errors');

const API = readFileSync(new URL('../docs/API.md', import.meta.url), 'utf8');

/** anchor -> the message patterns its Throws line lists, as regexes. */
const documented = new Map();
for (const m of API.matchAll(/<a id="([^"]+)"><\/a>\n###[^]*?(?=\n<a id=|$)/g)) {
  const line = m[0].split('\n').find((l) => l.startsWith('Throws:'));
  if (!line) continue;
  const patterns = [...line.matchAll(/`([^`]+)`/g)].map(([, text]) => {
    const parts = text.split(/…|<[^>]+>/).map((p) => p.replace(/[.*+?^${}()|[\]\\]/g, '\\$&'));
    return { text, regex: new RegExp(`^${parts.join('.+')}$`) };
  });
  documented.set(m[1], patterns);
}

const fresh = () => {
  const world = new World();
  return { world, body: world.addBody({}) };
};
const dead = () => {
  const { world, body } = fresh();
  world.remove(body);
  return { world, body };
};
const gone = () => {
  const { world, body } = fresh();
  world.destroy();
  return { world, body };
};

/** A world with a body and a ball joint holding it to a point. */
const joined = () => {
  const { world, body } = fresh();
  return { world, body, joint: world.addJoint({ a: body }) };
};

const JOINT_OPTIONS = 'type, a, b, anchor, anchorB, axis, min, max, length, cone, motorSpeed, motorForce, frequency, dampingRatio, collide';
const OPTIONS = 'type, shape, size, radius, height, vertices, indices, heights, columns, spacing, parts, density, mass, inertia, centerOfMass, friction, restitution, damping, angularDamping, gravityScale, layers, mask, sensor, lockRotation, maxSlope, stepHeight, position, rotation, velocity, angularVelocity';

/** The smallest .glb: one node, 'floor', showing one triangle; `json` adds to its JSON. */
function glb(json = {}) {
  const corners = new Float32Array([0, 0, 0, 1, 0, 0, 0, 0, 1]);
  let text = JSON.stringify({
    asset: { version: '2.0' }, nodes: [{ name: 'floor', mesh: 0 }],
    meshes: [{ primitives: [{ attributes: { POSITION: 0 } }] }],
    accessors: [{ bufferView: 0, componentType: 5126, count: 3, type: 'VEC3' }],
    bufferViews: [{ buffer: 0, byteLength: 36 }], buffers: [{ byteLength: 36 }], ...json,
  });
  while (text.length % 4) text += ' ';
  const j = new TextEncoder().encode(text), out = new Uint8Array(20 + j.length + 8 + 36), v = new DataView(out.buffer);
  v.setUint32(0, 0x46546c67, true); v.setUint32(4, 2, true); v.setUint32(8, out.length, true);
  v.setUint32(12, j.length, true); v.setUint32(16, 0x4e4f534a, true); out.set(j, 20);
  v.setUint32(20 + j.length, 36, true); v.setUint32(24 + j.length, 0x004e4942, true);
  out.set(new Uint8Array(corners.buffer), 28 + j.length);
  return out;
}

/** [entry, the call, the exact message]. */
const cases = [
  ['world', () => new World(5), 'World: options must be an object, got 5'],
  ['world', () => new World({ gravty: [0, 0, 0] }), "World: unknown option 'gravty' (did you mean 'gravity'?)"],
  ['world', () => new World({ wind: 1 }), "World: unknown option 'wind' (the options are gravity, substeps, sleeping)"],
  ['world', () => new World({ gravity: [0, -9.81] }), 'World: gravity must be 3 finite numbers, got 0,-9.81'],
  ['world', () => new World({ substeps: 0 }), 'World: substeps must be a whole number, 1 or more, got 0'],
  ['world', () => new World({ sleeping: 0 }), 'World: sleeping must be true or false, got 0'],
  ['world-sleeping', () => gone().world.sleeping, 'sleeping: this world was destroyed'],
  ['world-sleeping', () => { fresh().world.sleeping = 'no'; }, "world.sleeping must be true or false, got 'no'"],
  ['world-stepcount', () => gone().world.stepCount, 'stepCount: this world was destroyed'],
  ['world-stepcount', () => { fresh().world.stepCount = 0; }, 'world.stepCount is read only'],
  ['world-moved', () => gone().world.moved(0), 'moved: this world was destroyed'],
  ['world-moved', () => fresh().world.moved(-1), 'moved: since must be a whole number, 0 or more, got -1'],
  ['world-moved', () => fresh().world.moved(), 'moved: since must be a whole number, 0 or more, got undefined'],
  ['world-substeps', () => gone().world.substeps, 'substeps: this world was destroyed'],
  ['world-substeps', () => { fresh().world.substeps = 2.5; }, 'world.substeps must be a whole number, 1 or more, got 2.5'],

  ['world-gravity', () => { const { world } = gone(); return world.gravity; }, 'gravity: this world was destroyed'],
  ['world-gravity', () => { fresh().world.gravity = [0, NaN, 0]; }, 'world.gravity must be 3 finite numbers, got 0,NaN,0'],
  ['world-bodycount', () => gone().world.bodyCount, 'bodyCount: this world was destroyed'],
  ['world-bodycount', () => { fresh().world.bodyCount = 3; }, 'world.bodyCount is read only'],
  ['world-transforms', () => gone().world.transforms, 'transforms: this world was destroyed'],
  ['world-transforms', () => { fresh().world.transforms = null; }, 'world.transforms is read only'],
  ['world-previous', () => gone().world.previous, 'previous: this world was destroyed'],
  ['world-previous', () => { fresh().world.previous = null; }, 'world.previous is read only'],

  ...['addBody', 'setBody'].flatMap((call) => {
    const run = (o) => () => {
      const { world, body } = fresh();
      return call === 'addBody' ? world.addBody(o) : world.setBody(body, o);
    };
    const name = call === 'addBody' ? 'options' : 'changes';
    const e = `world-${call.toLowerCase()}`;
    return [
      [e, () => { const { world, body } = gone(); return call === 'addBody' ? world.addBody({}) : world.setBody(body, {}); }, `${call}: this world was destroyed`],
      [e, run('box'), `${call}: ${name} must be an object, got 'box'`],
      [e, run({ postion: [0, 0, 0] }), `${call}: unknown option 'postion' (did you mean 'position'?)`],
      [e, run({ Mass: 5 }), `${call}: unknown option 'Mass' (did you mean 'mass'?)`],
      [e, run({ banana: 1 }), `${call}: unknown option 'banana' (the options are ${OPTIONS})`],
      [e, run({ type: 'fixed' }), `${call}: type must be 'dynamic', 'static', 'kinematic' or 'character', got 'fixed'`],
      [e, run({ maxSlope: 0.5 }), `${call}: a dynamic body has no maxSlope`],
      [e, run({ type: 'character', maxSlope: 2 }), `${call}: maxSlope must be an angle from 0 to π/2, in radians, got 2`],
      [e, run({ type: 'character', stepHeight: -0.1 }), `${call}: stepHeight must be 0 or more, or null to follow the size, got -0.1`],
      [e, run({ lockRotation: true }), `${call}: lockRotation must be 3 true or false values, got true`],
      [e, run({ shape: 'heightfield', type: 'character' }), `${call}: type must be 'static' or 'kinematic' for a heightfield, got 'character'`],
      [e, run({ shape: 'cube' }), `${call}: shape must be 'box', 'sphere', 'capsule', 'hull', 'compound', 'mesh' or 'heightfield', got 'cube'`],
      [e, run({ shape: 'hull', vertices: [0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0] }), `${call}: vertices must span a solid (4 points not in one plane), got 4 points in one plane`],
      [e, run({ shape: 'hull', type: 'character' }), `${call}: type must be 'dynamic', 'static' or 'kinematic' for a hull, got 'character'`],
      [e, run({ shape: 'compound', parts: [] }), `${call}: parts must be a list of one or more parts, got an empty array`],
      [e, run({ shape: 'compound', parts: [5] }), `${call}: parts[0] must be an object, got 5`],
      [e, run({ shape: 'compound', parts: [{}, { radus: 1 }] }), `${call}: parts[1]: unknown option 'radus' (did you mean 'radius'?)`],
      [e, run({ shape: 'compound', parts: [{ banana: 1 }] }),
        `${call}: parts[0]: unknown option 'banana' (the options are shape, size, radius, height, vertices, position, rotation, density)`],
      [e, run({ shape: 'compound', parts: [{ shape: 'mesh' }] }), `${call}: parts[0].shape must be 'box', 'sphere', 'capsule' or 'hull', got 'mesh'`],
      [e, run({ shape: 'compound', parts: [{ radius: 1 }] }), `${call}: parts[0]: a box has no radius`],
      [e, run({ shape: 'compound', parts: [{ size: [1, 0, 1] }] }), `${call}: parts[0].size must be 3 positive numbers, got 1,0,1`],
      [e, run({ shape: 'compound', parts: [{ shape: 'sphere', radius: 0 }] }), `${call}: parts[0].radius must be a positive number, got 0`],
      [e, run({ shape: 'compound', parts: [{ shape: 'capsule', height: -1 }] }), `${call}: parts[0].height must be a positive number, got -1`],
      [e, run({ shape: 'compound', parts: [{ shape: 'capsule', height: 0.8 }] }), `${call}: parts[0].height must be at least twice radius (1), got 0.8`],
      [e, run({ shape: 'compound', parts: [{ shape: 'hull', vertices: [0, 0, NaN] }] }),
        `${call}: parts[0].vertices must be finite numbers, three to a vertex, got NaN at index 2`],
      [e, run({ shape: 'compound', parts: [{ shape: 'hull', vertices: [0, 0, 0, 1, 0, 0, 0, 1, 0] }] }),
        `${call}: parts[0].vertices must span a solid (4 points not in one plane), got 3 points`],
      [e, run({ shape: 'compound', parts: [{ position: [1, 2] }] }), `${call}: parts[0].position must be 3 finite numbers, got 1,2`],
      [e, run({ shape: 'compound', parts: [{ rotation: [0, 0, 0, 0] }] }), `${call}: parts[0].rotation must be 4 finite numbers, not all zero, got 0,0,0,0`],
      [e, run({ shape: 'compound', parts: [{ density: -1 }] }), `${call}: parts[0].density must be a positive number, got -1`],
      [e, run({ centerOfMass: 1 }), `${call}: centerOfMass must be 3 finite numbers, got 1`],
      [e, run({ type: 'character', shape: 'capsule', centerOfMass: [0, 0, 0] }), `${call}: a character body has no centerOfMass`],
      [e, run({ shape: 'mesh', type: 'dynamic' }), `${call}: type must be 'static' or 'kinematic' for a mesh, got 'dynamic'`],
      [e, run({ shape: 'heightfield', type: 'static', mass: 5 }), `${call}: a heightfield has no mass`],
      [e, run({ shape: 'mesh', type: 'static', vertices: [0, 0, NaN] }), `${call}: vertices must be finite numbers, three to a vertex, got NaN at index 2`],
      [e, run({ shape: 'mesh', type: 'static', vertices: [0, 0, 0], indices: [0, 0, 1] }),
        `${call}: indices must be whole numbers below the vertex count (1), three to a triangle, got 1 at index 2`],
      [e, run({ shape: 'heightfield', type: 'static', columns: 1 }), `${call}: columns must be a whole number, 2 or more, got 1`],
      [e, run({ shape: 'heightfield', type: 'static', spacing: 0 }), `${call}: spacing must be a positive number, got 0`],
      [e, run({ shape: 'heightfield', type: 'static', heights: [0, 0, 0] }),
        `${call}: heights must be finite numbers, rows of columns (2), 2 rows or more, got 3 numbers`],
      [e, run({ radius: 0.5 }), `${call}: a box has no radius`],
      [e, run({ shape: 'sphere', height: 2 }), `${call}: a sphere has no height`],
      [e, run({ size: [1, 0, 1] }), `${call}: size must be 3 positive numbers, got 1,0,1`],
      [e, run({ shape: 'sphere', radius: -2 }), `${call}: radius must be a positive number, got -2`],
      [e, run({ shape: 'capsule', height: '2' }), `${call}: height must be a positive number, got '2'`],
      [e, run({ shape: 'capsule', height: 0.8 }), `${call}: height must be at least twice radius (1), got 0.8`],
      [e, run({ mass: 1, density: 1 }), `${call}: give mass or density, not both`],
      [e, run({ mass: 0 }), `${call}: mass must be a positive number, got 0`],
      [e, run({ density: Infinity }), `${call}: density must be a positive number, got Infinity`],
      [e, run({ friction: -0.1 }), `${call}: friction must be 0 or more, got -0.1`],
      [e, run({ restitution: 1.2 }), `${call}: restitution must be from 0 to 1, got 1.2`],
      [e, run({ damping: NaN }), `${call}: damping must be 0 or more, got NaN`],
      [e, run({ angularDamping: Infinity }), `${call}: angularDamping must be 0 or more, got Infinity`],
      [e, run({ gravityScale: '1' }), `${call}: gravityScale must be a finite number, got '1'`],
      [e, run({ layers: 2 ** 32 }), `${call}: layers must be a whole number that fits in 32 bits, got 4294967296`],
      [e, run({ mask: 0.5 }), `${call}: mask must be a whole number that fits in 32 bits, got 0.5`],
      [e, run({ sensor: 1 }), `${call}: sensor must be true or false, got 1`],
      [e, run({ inertia: [1, 0, 1] }), `${call}: inertia must be 3 positive numbers, or null to follow the shape, got 1,0,1`],
      [e, run({ position: { x: 1, y: 2, z: 3 } }), `${call}: position must be 3 finite numbers, got an object`],
      [e, run({ rotation: [0, 0, 0, 0] }), `${call}: rotation must be 4 finite numbers, not all zero, got 0,0,0,0`],
      [e, run({ type: 'static', velocity: [0, 0, 0] }), `${call}: a static body has no velocity`],
      [e, run({ velocity: [1, 2] }), `${call}: velocity must be 3 finite numbers, got 1,2`],
      [e, run({ angularVelocity: null }), `${call}: angularVelocity must be 3 finite numbers, got null`],
    ];
  }),
  ['world-addbody', () => { const world = new World(); world._slots.limit = 2; world.addBody(); world.addBody(); world.addBody(); },
    'addBody: a world holds at most 16777216 bodies (the slot field of an id is 24 bits)'],
  ['world-setbody', () => fresh().world.setBody(5, {}), 'setBody: body must be a Body, got 5'],
  ['world-setbody', () => new World().setBody(fresh().body, {}), 'setBody: that body belongs to another world'],
  ['world-setbody', () => { const { world, body } = dead(); world.setBody(body, {}); }, 'setBody: this body was removed'],

  ['world-save', () => gone().world.save(), 'save: this world was destroyed'],
  ['world-load', () => World.load(5), 'load: data must be what world.save() returned, got 5'],
  ['world-load', () => World.load({ ...new World().save(), format: 4 }), 'load: data is format 4; this Bearing reads format 3'],
  ['world-load', () => World.load({ format: 3, objects: [{ c: 'Box', f: {} }] }), "load: data is damaged: an object of an unknown kind, 'Box'"],
  ['world-debuglines', () => gone().world.debugLines(), 'debugLines: this world was destroyed'],
  ['world-debuglines', () => fresh().world.debugLines(true), 'debugLines: options must be an object, got true'],
  ['world-debuglines', () => fresh().world.debugLines({ shape: true }), "debugLines: unknown option 'shape' (did you mean 'shapes'?)"],
  ['world-debuglines', () => fresh().world.debugLines({ colours: 1 }), "debugLines: unknown option 'colours' (the options are shapes, contacts, joints, boxes)"],
  ['world-debuglines', () => fresh().world.debugLines({ boxes: 1 }), 'debugLines: boxes must be true or false, got 1'],
  ['world-bodyof', () => { const { world, body } = gone(); world.bodyOf(body); }, 'bodyOf: this world was destroyed'],
  ['world-bodyof', () => fresh().world.bodyOf({ id: 0 }), 'bodyOf: body must be a Body, got an object'],
  ['world-bodyof', () => new World().bodyOf(fresh().body), 'bodyOf: that body belongs to another world'],
  ['world-bodyof', () => { const { world, body } = dead(); world.bodyOf(body); }, 'bodyOf: this body was removed'],

  ['world-remove', () => { const { world, body } = gone(); world.remove(body); }, 'remove: this world was destroyed'],
  ['world-remove', () => fresh().world.remove(undefined), 'remove: handle must be a Body or a Joint, got undefined'],
  ['world-remove', () => new World().remove(fresh().body), 'remove: that body belongs to another world'],
  ['world-remove', () => new World().remove(joined().joint), 'remove: that joint belongs to another world'],

  ['world-jointcount', () => gone().world.jointCount, 'jointCount: this world was destroyed'],
  ['world-jointcount', () => { fresh().world.jointCount = 2; }, 'world.jointCount is read only'],
  ...['addJoint', 'setJoint'].flatMap((call) => {
    const run = (o) => () => {
      const { world, body } = fresh();
      if (call === 'addJoint') return world.addJoint(o !== null && typeof o === 'object' ? { a: body, ...o } : o);
      return world.setJoint(world.addJoint({ a: body }), o);
    };
    const name = call === 'addJoint' ? 'options' : 'changes';
    const e = `world-${call.toLowerCase()}`;
    return [
      [e, () => { const { world, joint, body } = joined(); world.destroy(); return call === 'addJoint' ? world.addJoint({ a: body }) : world.setJoint(joint, {}); }, `${call}: this world was destroyed`],
      [e, run('hinge'), `${call}: ${name} must be an object, got 'hinge'`],
      [e, run({ anchr: [0, 0, 0] }), `${call}: unknown option 'anchr' (did you mean 'anchor'?)`],
      [e, run({ banana: 1 }), `${call}: unknown option 'banana' (the options are ${JOINT_OPTIONS})`],
      [e, run({ type: 'weld' }), `${call}: type must be 'fixed', 'ball', 'hinge', 'slider' or 'distance', got 'weld'`],
      [e, run({ type: 'fixed', axis: [1, 0, 0] }), `${call}: a fixed joint has no axis`],
      [e, run({ anchor: [0, 0] }), `${call}: anchor must be 3 finite numbers, got 0,0`],
      [e, run({ anchorB: 'x' }), `${call}: anchorB must be 3 finite numbers, got 'x'`],
      [e, run({ type: 'hinge', axis: [0, 0, 0] }), `${call}: axis must be 3 finite numbers, not all zero, got 0,0,0`],
      [e, run({ type: 'distance', length: -1 }), `${call}: length must be 0 or more, got -1`],
      [e, run({ cone: 4 }), `${call}: cone must be an angle from 0 to π, in radians, got 4`],
      [e, run({ type: 'hinge', motorSpeed: NaN }), `${call}: motorSpeed must be a finite number, got NaN`],
      [e, run({ type: 'slider', motorForce: -1 }), `${call}: motorForce must be 0 or more, got -1`],
      [e, run({ type: 'distance', frequency: Infinity }), `${call}: frequency must be 0 or more, got Infinity`],
      [e, run({ type: 'distance', dampingRatio: '1' }), `${call}: dampingRatio must be 0 or more, got '1'`],
      [e, run({ type: 'hinge', min: 4 }), `${call}: min must be from -π to π, in radians, or -Infinity, got 4`],
      [e, run({ max: -Infinity }), `${call}: max must be from -π to π, in radians, or Infinity, got -Infinity`],
      [e, run({ type: 'slider', min: NaN }), `${call}: min must be a number or -Infinity, got NaN`],
      [e, run({ type: 'slider', max: -Infinity }), `${call}: max must be a number or Infinity, got -Infinity`],
      [e, run({ type: 'distance', min: -1 }), `${call}: min must be 0 or more, got -1`],
      [e, run({ type: 'distance', max: -2 }), `${call}: max must be 0 or more, or Infinity, got -2`],
      [e, run({ type: 'slider', min: 2, max: 1 }), `${call}: min must be at most max (1), got 2`],
      [e, run({ collide: 'yes' }), `${call}: collide must be true or false, got 'yes'`],
    ];
  }),
  ['world-addjoint', () => fresh().world.addJoint({}), 'addJoint: a must be a Body of this world, got undefined'],
  ['world-addjoint', () => { const { world, body } = dead(); world.addJoint({ a: body }); }, 'addJoint: a was removed'],
  ['world-addjoint', () => { const { world, body } = fresh(); world.addJoint({ a: body, b: 5 }); }, 'addJoint: b must be a Body of this world, or null, got 5'],
  ['world-addjoint', () => { const { world, body } = fresh(); const b = world.addBody({}); world.remove(b); world.addJoint({ a: body, b }); }, 'addJoint: b was removed'],
  ['world-addjoint', () => { const { world, body } = fresh(); world.addJoint({ a: body, b: body }); }, 'addJoint: a and b must be different bodies'],
  ['world-addjoint', () => { const { world, body } = fresh(); world._joints.slots.limit = 2; world.addJoint({ a: body }); world.addJoint({ a: body }); world.addJoint({ a: body }); },
    'addJoint: a world holds at most 16777216 joints (the slot field of an id is 24 bits)'],
  ['world-setjoint', () => fresh().world.setJoint(5, {}), 'setJoint: joint must be a Joint, got 5'],
  ['world-setjoint', () => new World().setJoint(joined().joint, {}), 'setJoint: that joint belongs to another world'],
  ['world-setjoint', () => { const { world, joint } = joined(); world.remove(joint); world.setJoint(joint, {}); }, 'setJoint: this joint was removed'],
  ['world-setjoint', () => { const { world, joint, body } = joined(); world.setJoint(joint, { a: body }); }, "setJoint: a and b can't change; remove the joint and add another"],
  ['world-jointof', () => { const { world, joint } = joined(); world.destroy(); world.jointOf(joint); }, 'jointOf: this world was destroyed'],
  ['world-jointof', () => fresh().world.jointOf({ id: 0 }), 'jointOf: joint must be a Joint, got an object'],
  ['world-jointof', () => new World().jointOf(joined().joint), 'jointOf: that joint belongs to another world'],
  ['world-jointof', () => { const { world, joint } = joined(); joint.destroy(); world.jointOf(joint); }, 'jointOf: this joint was removed'],
  ...['id', 'world', 'alive'].map((p) => [`joint-${p}`, () => { joined().joint[p] = 1; }, `joint.${p} is read only`]),
  ['joint-destroy', () => { const { world, joint } = joined(); world.destroy(); joint.destroy(); }, 'destroy: this world was destroyed'],

  ['world-contacts', () => gone().world.contacts(), 'contacts: this world was destroyed'],
  ['world-contacts', () => fresh().world.contacts({ id: 1 }), 'contacts: body must be a Body, got an object'],
  ['world-contacts', () => new World().contacts(fresh().body), 'contacts: that body belongs to another world'],
  ['world-groundof', () => { const { world, body } = gone(); world.groundOf(body); }, 'groundOf: this world was destroyed'],
  ['world-groundof', () => fresh().world.groundOf('hero'), "groundOf: body must be a Body, got 'hero'"],
  ['world-groundof', () => new World().groundOf(fresh().body), 'groundOf: that body belongs to another world'],
  ['world-groundof', () => { const { world, body } = dead(); world.groundOf(body); }, 'groundOf: this body was removed'],
  ['world-touching', () => { const { world, body } = gone(); world.touching(body); }, 'touching: this world was destroyed'],
  ['world-touching', () => fresh().world.touching(null), 'touching: body must be a Body, got null'],
  ['world-touching', () => new World().touching(fresh().body), 'touching: that body belongs to another world'],
  ['world-touching', () => { const { world, body } = dead(); world.touching(body); }, 'touching: this body was removed'],

  ['world-body', () => gone().world.body(0), 'body: this world was destroyed'],
  ['world-body', () => fresh().world.body(1.5), 'body: id must be a whole number from 0 to 9007199254740991, got 1.5'],

  ...['raycast', 'raycastAll'].flatMap((call) => {
    const e = `world-${call.toLowerCase()}`, run = (...args) => () => fresh().world[call](...args);
    return [
      [e, () => gone().world[call]([0, 0, 0], [0, -1, 0]), `${call}: this world was destroyed`],
      [e, run([0, 0], [0, -1, 0]), `${call}: origin must be 3 finite numbers, got 0,0`],
      [e, run([0, 0, 0], [0, 0, 0]), `${call}: direction must be 3 finite numbers, not all zero, got 0,0,0`],
      [e, run([0, 0, 0], [0, -1, 0], 3), `${call}: options must be an object, got 3`],
      [e, run([0, 0, 0], [0, -1, 0], { maxDistanse: 1 }), `${call}: unknown option 'maxDistanse' (did you mean 'maxDistance'?)`],
      [e, run([0, 0, 0], [0, -1, 0], { layers: 1 }), `${call}: unknown option 'layers' (the options are maxDistance, mask, ignore)`],
      [e, run([0, 0, 0], [0, -1, 0], { maxDistance: -1 }), `${call}: maxDistance must be 0 or more, got -1`],
      [e, run([0, 0, 0], [0, -1, 0], { mask: 1.5 }), `${call}: mask must be a whole number that fits in 32 bits, got 1.5`],
      [e, run([0, 0, 0], [0, -1, 0], { ignore: 7 }), `${call}: ignore must be a Body of this world, got 7`],
    ];
  }),
  ...['shapeCast', 'overlap'].flatMap((call) => {
    const e = `world-${call.toLowerCase()}`, cast = call === 'shapeCast';
    const run = (shape, place = [0, 0, 0], o) => () => {
      const { world } = fresh();
      return cast ? world.shapeCast(shape, place, [0, -1, 0], o) : world.overlap(shape, place, o);
    };
    const where = cast ? 'origin' : 'position';
    return [
      [e, () => (cast ? gone().world.shapeCast({}, [0, 0, 0], [0, -1, 0]) : gone().world.overlap({}, [0, 0, 0])), `${call}: this world was destroyed`],
      [e, run('box'), `${call}: shape must be an object, got 'box'`],
      [e, run({ radus: 1 }), `${call}: unknown option 'radus' (did you mean 'radius'?)`],
      [e, run({ friction: 1 }), `${call}: unknown option 'friction' (the options are shape, size, radius, height)`],
      [e, run({ shape: 'cone' }), `${call}: shape must be 'box', 'sphere' or 'capsule', got 'cone'`],
      [e, run({ radius: 1 }), `${call}: a box has no radius`],
      [e, run({ size: [1, 0, 1] }), `${call}: size must be 3 positive numbers, got 1,0,1`],
      [e, run({ shape: 'sphere', radius: -1 }), `${call}: radius must be a positive number, got -1`],
      [e, run({ shape: 'capsule', height: 0 }), `${call}: height must be a positive number, got 0`],
      [e, run({ shape: 'capsule', radius: 1, height: 1 }), `${call}: height must be at least twice radius (2), got 1`],
      [e, run({}, [0, NaN, 0]), `${call}: ${where} must be 3 finite numbers, got 0,NaN,0`],
      [e, run({}, [0, 0, 0], []), `${call}: options must be an object, got an empty array`],
      [e, run({}, [0, 0, 0], { rotaton: [0, 0, 0, 1] }), `${call}: unknown option 'rotaton' (did you mean 'rotation'?)`],
      [e, run({}, [0, 0, 0], { speed: 1 }), `${call}: unknown option 'speed' (the options are ${cast ? 'maxDistance, ' : ''}mask, ignore, rotation)`],
      [e, run({}, [0, 0, 0], { mask: -1.5 }), `${call}: mask must be a whole number that fits in 32 bits, got -1.5`],
      [e, run({}, [0, 0, 0], { ignore: {} }), `${call}: ignore must be a Body of this world, got an object`],
      [e, run({}, [0, 0, 0], { rotation: [0, 0, 0, 0] }), `${call}: rotation must be 4 finite numbers, not all zero, got 0,0,0,0`],
      ...(cast ? [
        [e, () => fresh().world.shapeCast({}, [0, 0, 0], [0, 0, 0]), 'shapeCast: direction must be 3 finite numbers, not all zero, got 0,0,0'],
        [e, run({}, [0, 0, 0], { maxDistance: NaN }), 'shapeCast: maxDistance must be 0 or more, got NaN'],
      ] : []),
    ];
  }),
  ['world-step', () => gone().world.step(1 / 60), 'step: this world was destroyed'],
  ['world-step', () => fresh().world.step(0), 'step: dt must be a positive number of seconds, got 0'],

  ...['id', 'slot', 'world', 'alive', 'asleep'].map((p) => [`body-${p}`, () => { fresh().body[p] = 1; }, `body.${p} is read only`]),
  ['body-asleep', () => gone().body.asleep, 'asleep: this world was destroyed'],
  ['body-asleep', () => dead().body.asleep, 'asleep: this body was removed'],
  ...[['mass', 'setBody(body, { mass })'], ['inertia', 'setBody(body, { inertia })'],
    ['position', 'setPosition(x, y, z)'], ['rotation', 'setRotation(q)'],
    ['velocity', 'setVelocity(x, y, z)'], ['angularVelocity', 'setAngularVelocity(x, y, z)']].flatMap(([p, hint]) => [
    [`body-${p.toLowerCase()}`, () => gone().body[p], `${p}: this world was destroyed`],
    [`body-${p.toLowerCase()}`, () => dead().body[p], `${p}: this body was removed`],
    [`body-${p.toLowerCase()}`, () => { fresh().body[p] = 1; }, `body.${p} is read only; use ${hint}`],
  ]),

  ...[['setPosition', [0, 0, 0]], ['setRotation', [[0, 0, 0, 1]]], ['setAxisAngle', [[0, 1, 0], 1]],
    ['setEuler', [0, 0]], ['setVelocity', [0, 0, 0]], ['setAngularVelocity', [0, 0, 0]],
    ['applyForce', [[0, 0, 0]]], ['applyTorque', [[0, 0, 0]]], ['applyImpulse', [[0, 0, 0]]],
    ['applyAngularImpulse', [[0, 0, 0]]]].flatMap(([m, args]) => [
    [`body-${m.toLowerCase()}`, () => gone().body[m](...args), `${m}: this world was destroyed`],
    [`body-${m.toLowerCase()}`, () => dead().body[m](...args), `${m}: this body was removed`],
  ]),
  ['body-setposition', () => fresh().body.setPosition(1, NaN, 0), 'setPosition: x, y and z must be finite numbers, got 1,NaN,0'],
  ['body-setrotation', () => fresh().body.setRotation([0, 0, 1]), 'setRotation: q must be 4 finite numbers, not all zero, got 0,0,1'],
  ['body-setaxisangle', () => fresh().body.setAxisAngle([0, 0, 0], 1), 'setAxisAngle: axis must be 3 finite numbers, not all zero, got 0,0,0'],
  ['body-setaxisangle', () => fresh().body.setAxisAngle([0, 1, 0], '90'), "setAxisAngle: radians must be a finite number, got '90'"],
  ['body-seteuler', () => fresh().body.setEuler(0, undefined), 'setEuler: yaw, pitch and roll must be finite numbers, got 0,undefined,0'],
  ['body-setvelocity', () => fresh().body.setVelocity(1, 2), 'setVelocity: x, y and z must be finite numbers, got 1,2,undefined'],
  ['body-setvelocity', () => new World().addBody({ type: 'static' }).setVelocity(1, 0, 0), 'setVelocity: a static body has no velocity'],
  ['body-centerofmass', () => gone().body.centerOfMass, 'centerOfMass: this world was destroyed'],
  ['body-centerofmass', () => dead().body.centerOfMass, 'centerOfMass: this body was removed'],
  ['body-centerofmass', () => { fresh().body.centerOfMass = [0, 0, 0]; }, 'body.centerOfMass is read only; use setBody(body, { centerOfMass })'],
  ['body-setangularvelocity', () => fresh().body.setAngularVelocity(0, Infinity, 0), 'setAngularVelocity: x, y and z must be finite numbers, got 0,Infinity,0'],
  ['body-setangularvelocity', () => new World().addBody({ type: 'static' }).setAngularVelocity(0, 1, 0), 'setAngularVelocity: a static body has no angularVelocity'],
  ['body-applyforce', () => fresh().body.applyForce(10), 'applyForce: force must be 3 finite numbers, got 10'],
  ['body-applyforce', () => fresh().body.applyForce([0, 1, 0], [0, 0]), 'applyForce: point must be 3 finite numbers, got 0,0'],
  // Checked even on a body forces don't move, so a bad call is caught wherever it lands.
  ['body-applytorque', () => new World().addBody({ type: 'static' }).applyTorque([NaN, 0, 0]), 'applyTorque: torque must be 3 finite numbers, got NaN,0,0'],
  ['body-applyimpulse', () => fresh().body.applyImpulse(), 'applyImpulse: impulse must be 3 finite numbers, got undefined'],
  ['body-applyimpulse', () => fresh().body.applyImpulse([0, 1, 0], null), 'applyImpulse: point must be 3 finite numbers, got null'],
  ['body-applyangularimpulse', () => fresh().body.applyAngularImpulse([0, '1', 0]), "applyAngularImpulse: impulse must be 3 finite numbers, got 0,'1',0"],
  ['body-destroy', () => gone().body.destroy(), 'destroy: this world was destroyed'],

  ['readglb', () => readGlb(glb(), 5), 'readGlb: options must be an object, got 5'],
  ['readglb', () => readGlb(glb(), { nod: 'floor' }), "readGlb: unknown option 'nod' (did you mean 'node'?)"],
  ['readglb', () => readGlb(glb(), { scale: 2 }), "readGlb: unknown option 'scale' (the options are node)"],
  ['readglb', () => readGlb(glb(), { node: 0 }), 'readGlb: node must be a string, got 0'],
  ['readglb', () => readGlb(glb(), { node: 'roof' }), "readGlb: node must be the name of a node in the file, got 'roof'"],
  ['readglb', () => readGlb('level.glb'), "readGlb: bytes must be an ArrayBuffer or a typed array, got 'level.glb'"],
  ['readglb', () => readGlb(new Uint8Array(12)), "readGlb: bytes must be a binary glTF 2.0 file (.glb), got a file that doesn't start with glTF"],
  ['readglb', () => readGlb(glb({ extensionsRequired: ['EXT_meshopt_compression'] })),
    "readGlb: the file needs EXT_meshopt_compression, which Bearing doesn't read; export it without that"],
  ['readglb', () => readGlb(glb().subarray(0, 60)), 'readGlb: the file is damaged: a chunk runs past the end of the file'],
];

test('every case throws exactly its message', () => {
  for (const [entry, call, message] of cases) {
    assert.equal(thrown(call), message, `in ${entry}`);
  }
});

test('every message thrown is listed in its entry', () => {
  for (const [entry, , message] of cases) {
    const patterns = documented.get(entry);
    assert.ok(patterns, `${entry} has no Throws line, but throws '${message}'`);
    assert.ok(patterns.some((p) => p.regex.test(message)), `${entry} doesn't list '${message}'`);
  }
});

test('every message listed is thrown by some case', () => {
  for (const [entry, patterns] of documented) {
    for (const p of patterns) {
      const hit = cases.some(([e, , message]) => e === entry && p.regex.test(message));
      assert.ok(hit, `${entry} lists '${p.text}', and no case throws it`);
    }
  }
});

test('a call that throws changes nothing', () => {
  const world = new World();
  const body = world.addBody({ position: [1, 2, 3], velocity: [4, 5, 6] });
  const before = JSON.stringify(world.bodyOf(body));
  // Valid options first, the bad one last: none of them may land.
  assert.throws(() => world.setBody(body, { position: [9, 9, 9], mass: 5, velocity: [1, 2] }));
  assert.throws(() => world.setBody(body, { shape: 'sphere', radius: 1, height: 3 }));
  assert.throws(() => body.setPosition(7, 7, NaN));
  assert.equal(JSON.stringify(world.bodyOf(body)), before);
  assert.equal(world.bodyCount, 1);
  assert.throws(() => world.addBody({ position: [0, 0, 0], radius: 1 }));
  assert.equal(world.bodyCount, 1);
  // A joint the same: valid changes first, the bad one last.
  const joint = world.addJoint({ type: 'hinge', a: body, min: -1 });
  const was = JSON.stringify(world.jointOf(joint));
  assert.throws(() => world.setJoint(joint, { anchor: [5, 5, 5], motorForce: 3, min: -0.5, max: 5 }));
  assert.throws(() => world.setJoint(joint, { type: 'slider', collide: true, cone: 1 }));
  assert.equal(JSON.stringify(world.jointOf(joint)), was);
  assert.throws(() => world.addJoint({ a: body, anchor: [0, 1, 0], cone: -1 }));
  assert.equal(world.jointCount, 1);
});
