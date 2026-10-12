// The API reference is complete, every link in it goes somewhere, and every
// example in it runs.
// Run: node test/docs.test.js
//
// Written by hand, docs/API.md drifts the moment a method is added without an
// entry, an entry is renamed out from under a link, or an example stops
// matching the code. So what it must cover is read from the code itself --
// every public method and getter of every exported class -- and each example
// is run against the real package.

import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { World, Body } from '../src/bearing.js';
import * as bearing from '../src/bearing.js';
import { suite, test } from './test.js';

suite('docs');

const API = readFileSync(new URL('../docs/API.md', import.meta.url), 'utf8');
const README = readFileSync(new URL('../README.md', import.meta.url), 'utf8');
const anchors = new Set([...API.matchAll(/<a id="([^"]+)"><\/a>/g)].map((m) => m[1]));

/** Every public method and getter a class's instances have, as `prefix-name` anchors. Underscored names are internal. */
function surface(prefix, cls) {
  return Object.getOwnPropertyNames(cls.prototype)
    .filter((name) => name !== 'constructor' && !name.startsWith('_'))
    .map((name) => `${prefix}-${name.toLowerCase()}`);
}

const expected = ['world', ...surface('world', World), ...surface('body', Body)];

test('the package exports World and Body, and nothing else', () => {
  assert.deepEqual(Object.keys(bearing).sort(), ['Body', 'World']);
});

test('every public method and getter has an entry', () => {
  const missing = expected.filter((id) => !anchors.has(id));
  assert.deepEqual(missing, [], `no entry for: ${missing.join(', ')}`);
});

test('every entry is for something that exists', () => {
  const entries = [...API.matchAll(/<a id="((?:world|body)-[^"]+|world)"><\/a>\s*\n###/g)].map((m) => m[1]);
  const stale = entries.filter((id) => !expected.includes(id));
  assert.deepEqual(stale, [], `entries for nothing: ${stale.join(', ')}`);
});

test('every link to an entry lands on one', () => {
  const inPage = [...API.matchAll(/\]\(#([^)]+)\)/g)].map((m) => m[1]);
  const fromReadme = [...README.matchAll(/\]\(docs\/API\.md#([^)]+)\)/g)].map((m) => m[1]);
  const broken = [...new Set([...inPage, ...fromReadme])].filter((id) => !anchors.has(id));
  assert.deepEqual(broken, [], `links to nothing: ${broken.join(', ')}`);
});

test('the index lists every entry and section', () => {
  const start = API.indexOf('\n## Index');
  const end = API.indexOf('\n## ', start + 1);
  const index = API.slice(start, end);
  const listed = new Set([...index.matchAll(/\]\(#([^)]+)\)/g)].map((m) => m[1]));
  const entries = [...API.matchAll(/<a id="([^"]+)"><\/a>\s*\n###/g)].map((m) => m[1]);
  const missing = entries.filter((id) => !listed.has(id));
  assert.deepEqual(missing, [], `not in the index: ${missing.join(', ')}`);
});

test('every entry has its parts in order: heading, text, Throws if it throws, example', () => {
  for (const m of API.matchAll(/<a id="([^"]+)"><\/a>\n(###[^]*?)(?=\n<a id=|$)/g)) {
    const [, id, body] = m;
    assert.match(body, /^### `[^`]+`( → `[^`]+`)?\n\n\S/, `${id}: heading, then a sentence`);
    assert.match(body, /```js\n/, `${id}: an example`);
    const throws = body.indexOf('\nThrows:'), example = body.indexOf('```js');
    if (throws >= 0) assert.ok(throws < example, `${id}: Throws comes before the example`);
  }
});

/**
 * Every ```js block runs, against the real package. Each gets the same
 * starting world, so an example can use `ball` or `crate` without making
 * them, and each runs in a fresh block, so one can't spoil the next. Import
 * lines are dropped: World is passed in.
 */
const PREAMBLE = `
  const world = new World();
  const ball = world.addBody({ shape: 'sphere', radius: 0.5, position: [0, 4, 0] });
  const crate = world.addBody({ position: [2, 0.5, 0] });
  const player = world.addBody({ shape: 'capsule', radius: 0.3, height: 1.8, mass: 80 });
  const bodies = [ball, crate];
`;
const quiet = { log() {} };

function examples(markdown) {
  return [...markdown.matchAll(/```js\n([^]*?)```/g)].map((m) => m[1]);
}

test('every example in the API reference and the README runs', () => {
  const all = [...examples(API), ...examples(README)];
  // At least one per entry, or the pattern above has stopped finding them.
  assert.ok(all.length >= expected.length, `found ${all.length} examples`);
  for (const code of all) {
    const body = code.split('\n').filter((line) => !line.startsWith('import ')).join('\n');
    try {
      new Function('World', 'console', `${PREAMBLE}\n{\n${body}\n}`)(World, quiet);
    } catch (e) {
      assert.fail(`this example throws ${e.message}:\n${code}`);
    }
  }
});
