// Determinism's foundations: Bearing's own sine and cosine are right, and
// nothing in src/ calls a function the spec lets engines round differently.
// Run: node test/math.test.js

import assert from 'node:assert/strict';
import { readFileSync, readdirSync } from 'node:fs';
import { join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { sin, cos } from '../src/core/math.js';
import { suite, test } from './test.js';

suite('math');

/** Distance from x to the next double up: one unit in the last place. */
function ulp(x) {
  const f = new Float64Array([Math.abs(x)]);
  const i = new BigInt64Array(f.buffer);
  i[0] += 1n;
  return f[0] - Math.abs(x) || Number.MIN_VALUE;
}

test('sin and cos agree with Math.sin and Math.cos to 1 ulp', () => {
  // V8's Math.sin and Math.cos are fdlibm's too, so agreement this close says
  // the coefficients and the reduction are right; it is not what makes ours
  // deterministic (using only exact operations is).
  let seed = 12345;
  const random = () => (seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648;
  const xs = [0, 1e-300, 1e-8, 0.5, Math.PI / 4, Math.PI / 2, Math.PI, 2 * Math.PI, 100, 1e5, 8e5];
  for (let i = 0; i < 100000; i++) xs.push((random() * 2 - 1) * [1, 10, 1000, 8e5][i % 4]);
  // Near multiples of pi/2 is where reduction loses bits if it is going to.
  for (let k = 1; k < 10000; k++) xs.push(k * Math.PI / 2, k * Math.PI / 4);
  for (const x of xs) {
    assert.ok(Math.abs(sin(x) - Math.sin(x)) <= ulp(Math.sin(x)), `sin(${x})`);
    assert.ok(Math.abs(cos(x) - Math.cos(x)) <= ulp(Math.cos(x)), `cos(${x})`);
  }
  assert.equal(sin(0), 0);
  assert.equal(cos(0), 1);
});

test('nothing in src/ calls an implementation-approximated function', () => {
  // The spec defines + - * / and Math.sqrt exactly, so every engine agrees on
  // them. These it leaves to each engine, and one differing bit is enough for
  // two machines' worlds to part ways. Math.random is here too: it differs
  // every run, let alone every machine.
  const banned = /Math\.(sin|cos|tan|asin|acos|atan|atan2|sinh|cosh|tanh|asinh|acosh|atanh|exp|expm1|log|log1p|log2|log10|pow|cbrt|hypot|random)\b|\*\*/;
  for (const bad of ['Math.sin(a)', 'Math.atan2(y, x)', 'x ** 2', 'Math.random()']) assert.ok(banned.test(bad), bad);
  for (const fine of ['Math.sqrt(a)', 'Math.PI', 'Math.sign(x)', 'a * b']) assert.ok(!banned.test(fine), fine);
  const root = fileURLToPath(new URL('../src/', import.meta.url));
  const files = readdirSync(root, { recursive: true }).filter((f) => f.endsWith('.js'));
  assert.ok(files.length > 0);
  for (const file of files) {
    const code = readFileSync(join(root, file), 'utf8')
      .replace(/\/\*[^]*?\*\//g, '')          // block comments
      .replace(/(^|[^:])\/\/.*$/gm, '$1');    // line comments
    const hit = code.split('\n').find((line) => banned.test(line));
    assert.equal(hit, undefined, `src/${file}: ${hit?.trim()}`);
  }
});
