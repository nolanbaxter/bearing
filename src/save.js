// Saving a world, and loading it back exactly: world.save() and World.load().
//
// WHAT. Not the bodies as bodyOf gives them, but everything a step reads:
// every body's columns, the slots and their generations (so ids stay ids),
// the broadphase's trees and its pair list in order, the impulses each pair
// carries from step to step, the sleep timers and islands, the joints, the
// last step's events. With less, a loaded world would still load, but a pile
// would start its first step without the impulses holding it and twitch, and
// its results would drift from the saved one's. A saved world in a bug report
// has to step exactly as the one that went wrong.
//
// HOW. The world is walked as it is, field by field, rather than through a
// list of what to save, so a field added later is saved without anyone
// remembering to. Each object (the world, its storage classes, plain
// objects) goes into a table once, and a field holding one holds its place in
// the table, so objects shared stay shared and a pair list's link back to its
// world comes back as that link. Typed arrays become lists of numbers, with
// their type; numbers JSON can't hold (NaN, ±Infinity) and -0, which JSON
// writes as 0, are written as strings, so JSON.stringify and JSON.parse
// carry the data across with every bit. Functions aren't data: the one a
// storage class holds (the broadphase's visitor) is made afresh by its
// constructor as the object is loaded.
//
// FORMAT. The data is tied to the layout of the world's fields, which changes
// between versions of Bearing: FORMAT says which layout, and data of another
// throws rather than loading something subtly wrong. Bump it whenever a
// stored field is added, removed or changes meaning; test/save.test.js holds
// the world's field names to a list, so a change can't go unnoticed.

import { fail, show } from './core/check.js';
import { Slots } from './core/slots.js';
import { Broadphase } from './broadphase.js';
import { Tree } from './tree.js';
import { Pairs, Memory } from './pairs.js';
import { Contacts, RECORD } from './contact.js';
import { Joints } from './joints.js';
import { Forest } from './forest.js';
import { Events } from './events.js';
import { Mesh, Heightfield } from './mesh.js';
import { Hull } from './hull.js';
import { Parts, Part } from './collide.js';
import { COLOURS, DIGITS } from './graph.js';

/**
 * The layout of the data world.save() writes. 2 since milestone 16, when a
 * pair's remembered friction and slip went from one per point to one per
 * contact (friction solved once a manifold); data of format 1 holds them per
 * point, and is refused rather than misread. 3 since a hull pair keeps its
 * last manifold (pairs.js, cache): data of format 2 has none, and is refused
 * rather than loaded without it. 4 since a mesh keeps each edge's fold
 * (mesh.js, FOLDS): data of format 3 has none, and its meshes would stop the
 * first step that touched them. 5 since joints have friction (joints.js, the
 * friction column): data of format 4 has none, and its joints would load
 * without a value for it.
 */
export const FORMAT = 5;

/** The classes a world holds, by the name their objects are written under. */
const CLASSES = { Slots, Broadphase, Tree, Pairs, Memory, Contacts, Joints, Forest, Events, Mesh, Heightfield, Hull, Parts, Part };
/** Typed arrays, by the short name their lists are written under. */
const TYPED = {
  f64: Float64Array, f32: Float32Array, i32: Int32Array, u32: Uint32Array, i16: Int16Array, u16: Uint16Array,
  i8: Int8Array, u8: Uint8Array,
};
const TYPED_NAME = new Map(Object.entries(TYPED).map(([k, v]) => [v, k]));
/**
 * Fields that are scratch, not state: the debug lines' buffer, made afresh by
 * the next call, and the contacts' turning terms (contact.js, J), worked out
 * afresh before each step reads them, 36 numbers a point, which would nearly
 * double a pile's save; and the step's colours and solve order, and each
 * body's colour mask (graph.js), made afresh by each step; and what the
 * workers found for each pair and its records (contact.js, LOOKED), which
 * last one narrowphase pass. The workers themselves (core/jobs.js) change
 * how fast a world steps, never what it does: a world loaded has the
 * default, 'auto', as a new one does.
 */
const SCRATCH = {
  World: new Set(['_lines', '_placed', '_colours', '_workers', '_jobs']),
  Contacts: new Set(['jac', 'fjac', 'block', 'colour', 'order', 'row', 'scratch', 'starts', 'digits', 'onParts', 'plain', 'wasPlain', 'records', 'taken']),
  Pairs: new Set(['verdict']),
};

/** The world as plain, JSON-safe data. */
export function save(world, World) {
  const objects = [], index = new Map();
  const ref = (o) => {
    let i = index.get(o);
    if (i !== undefined) return i;
    i = objects.length;
    index.set(o, i);
    objects.push(null);
    const name = o instanceof World ? 'World' : o.constructor === Object ? 'Object' : className(o);
    const fields = {}, scratch = SCRATCH[name];
    for (const key of Object.keys(o)) {
      if (scratch?.has(key) || typeof o[key] === 'function') continue;
      fields[key] = value(o[key]);
    }
    objects[i] = { c: name, f: fields };
    return i;
  };
  const value = (v) => {
    if (typeof v === 'number') return Number.isFinite(v) && !Object.is(v, -0) ? v : { n: String(Object.is(v, -0) ? '-0' : v) };
    if (v === null || typeof v === 'boolean' || typeof v === 'string') return v;
    if (v === undefined) return { u: 1 };
    if (ArrayBuffer.isView(v)) return { t: TYPED_NAME.get(v.constructor), v: Array.from(v, number) };
    if (Array.isArray(v)) return v.map(value);
    return { r: ref(v) };
  };
  ref(world);
  return { format: FORMAT, objects };
}

/** A number as JSON can hold it, in a list of numbers: special ones as strings. */
function number(x) {
  return Number.isFinite(x) && !Object.is(x, -0) ? x : Object.is(x, -0) ? '-0' : String(x);
}

function className(o) {
  for (const [name, C] of Object.entries(CLASSES)) if (o.constructor === C) return name;
  // A class the world holds that isn't listed: Bearing's own mistake, never the data's.
  throw new Error(`save: a ${o.constructor.name} in the world has no place in saved data`);
}

/** The world that save() wrote `data` from. */
export function load(data, World) {
  if (data === null || typeof data !== 'object' || !('format' in data) || !Array.isArray(data.objects)) {
    fail('load', `data must be what world.save() returned, got ${show(data)}`);
  }
  if (data.format !== FORMAT) fail('load', `data is format ${show(data.format)}; this Bearing reads format ${FORMAT}`);
  const table = data.objects;
  try {
    // Every object first, empty, so a field can refer to any of them; then their fields.
    const made = table.map((o) => {
      if (o.c === 'World') return Object.create(World.prototype);
      if (o.c === 'Object') return {};
      if (o.c === 'Broadphase') return new Broadphase();
      const C = CLASSES[o.c];
      if (!C) throw new Error(`an object of an unknown kind, ${show(o.c)}`);
      return Object.create(C.prototype);
    });
    const value = (v) => {
      if (v === null || typeof v !== 'object') return v;
      if (Array.isArray(v)) return v.map(value);
      if ('n' in v) return Number(v.n === '-0' ? -0 : v.n);
      if ('u' in v) return undefined;
      if ('t' in v) {
        const T = TYPED[v.t];
        if (!T) throw new Error(`a list of an unknown type, ${show(v.t)}`);
        return T.from(v.v, (x) => (typeof x === 'number' ? x : x === '-0' ? -0 : Number(x)));
      }
      if ('r' in v) {
        if (!(v.r in made)) throw new Error(`a link to object ${show(v.r)}, of ${made.length}`);
        return made[v.r];
      }
      throw new Error('a value of no known kind');
    };
    table.forEach((o, i) => {
      for (const key of Object.keys(o.f)) made[i][key] = value(o.f[key]);
    });
    if (!(made[0] instanceof World)) throw new Error('no world first');
    made[0]._lines = new Float32Array(0);
    made[0]._placed = [];
    made[0]._colours = new Int32Array(made[0]._type.length);
    made[0]._setWorkers(null, 'workers', 'auto');
    for (const o of made) {
      if (!(o instanceof Contacts)) continue;
      o.jac = new Float64Array(o.normalMass.length * 12); o.fjac = new Float64Array(o.friction.length * 24); o.block = new Float64Array(o.normalMass.length * 4);
      o.colour = new Int32Array(o.a.length); o.order = new Int32Array(o.a.length); o.row = new Int32Array(o.a.length);
      o.scratch = new Float64Array(o.point.length); o.starts = new Int32Array(COLOURS + 2); o.digits = new Int32Array(DIGITS + 1);
      o.onParts = 0; o.plain = 0; o.wasPlain = 0;
      o.records = new Float64Array(o.a.length * RECORD); o.taken = new Int32Array(1);
    }
    for (const o of made) if (o instanceof Pairs) o.verdict = new Int32Array(o.a.length);
    return made[0];
  } catch (e) {
    return fail('load', `data is damaged: ${e.message}`);
  }
}
