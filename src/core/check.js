// Checking what comes in, so nothing quietly wrong gets into a world.
//
// Every option and argument is checked at the call that took it, and a bad one
// throws naming that call, the option, what it must be and what it got:
//
//   addBody: radius must be a positive number, got -2
//
// The caller's name is passed in rather than fixed here, so setBody's errors
// say setBody: one check serves both calls, and neither borrows the other's
// name. Nothing is checked inside step(). Everything was checked on the way
// in, so a simulation that goes wrong points at the solver, never at a NaN
// that slipped past the door.

/** `caller: message`, or just the message when the call is a property assignment. */
export function fail(caller, message) {
  throw new Error(caller ? `${caller}: ${message}` : message);
}

/**
 * A property you can read but not set. It throws rather than ignoring the
 * write (rule 26): a module is strict and would throw anyway, but a classic
 * script is not, and there the write would vanish without a word.
 */
export function readOnly(name, hint) {
  throw new Error(hint ? `${name} is read only; use ${hint}` : `${name} is read only`);
}

/** How a bad value reads in a message: arrays as 1,NaN,0 (Winding's form), strings quoted. */
export function show(value) {
  if (typeof value === 'string') return `'${value}'`;
  if (typeof value === 'bigint') return `${value}n`;
  if (typeof value === 'function') return 'a function';
  if (Array.isArray(value) || ArrayBuffer.isView(value)) {
    return value.length === 0 ? 'an empty array' : Array.from(value, (x) => show(x)).join(',');
  }
  if (value !== null && typeof value === 'object') return 'an object';
  return String(value);
}

const isNumber = (x) => typeof x === 'number' && Number.isFinite(x);

/** An array of `n` items, or a typed array: rule 52 takes any array-like, a Float32Array included. */
function arrayOf(value, n) {
  return value !== null && typeof value === 'object' && value.length === n;
}

export function finite(caller, name, value) {
  if (!isNumber(value)) fail(caller, `${name} must be a finite number, got ${show(value)}`);
  return value;
}

export function positive(caller, name, value) {
  if (!isNumber(value) || value <= 0) fail(caller, `${name} must be a positive number, got ${show(value)}`);
  return value;
}

/** Three finite numbers, copied out, so a caller's array can change later without reaching in. */
export function vec3(caller, name, value) {
  if (!arrayOf(value, 3) || !isNumber(value[0]) || !isNumber(value[1]) || !isNumber(value[2])) {
    fail(caller, `${name} must be 3 finite numbers, got ${show(value)}`);
  }
  return [value[0], value[1], value[2]];
}

/** A setter's (x, y, z), each a finite number: `setPosition: x, y and z must be finite numbers, got 1,NaN,0`. */
export function xyz(caller, x, y, z) {
  if (!isNumber(x) || !isNumber(y) || !isNumber(z)) {
    fail(caller, `x, y and z must be finite numbers, got ${show([x, y, z])}`);
  }
  return [x, y, z];
}

export function positive3(caller, name, value) {
  if (!arrayOf(value, 3) || !(isNumber(value[0]) && value[0] > 0)
      || !(isNumber(value[1]) && value[1] > 0) || !(isNumber(value[2]) && value[2] > 0)) {
    fail(caller, `${name} must be 3 positive numbers, got ${show(value)}`);
  }
  return [value[0], value[1], value[2]];
}

/**
 * Three finite numbers, not all zero, scaled to unit length: a direction.
 *
 * Scaled by the largest component before squaring, so [1e200, 0, 0] is the
 * x axis instead of overflowing to Infinity and coming back as zero.
 */
export function direction(caller, name, value) {
  if (arrayOf(value, 3) && isNumber(value[0]) && isNumber(value[1]) && isNumber(value[2])) {
    const m = Math.max(Math.abs(value[0]), Math.abs(value[1]), Math.abs(value[2]));
    if (m > 0) {
      const x = value[0] / m, y = value[1] / m, z = value[2] / m;
      const s = 1 / Math.sqrt(x * x + y * y + z * z);
      return [x * s, y * s, z * s];
    }
  }
  return fail(caller, `${name} must be 3 finite numbers, not all zero, got ${show(value)}`);
}

/**
 * A rotation: four finite numbers, not all zero, normalized on the way in.
 *
 * A quaternion means the same rotation at any length, so normalizing is not a
 * clamp, it is choosing the one form the integrator needs. Zero has no
 * direction to normalize to, so it throws. Scaled first, as `direction` is.
 */
export function quat(caller, name, value) {
  if (arrayOf(value, 4) && isNumber(value[0]) && isNumber(value[1])
      && isNumber(value[2]) && isNumber(value[3])) {
    const m = Math.max(Math.abs(value[0]), Math.abs(value[1]), Math.abs(value[2]), Math.abs(value[3]));
    if (m > 0) {
      const x = value[0] / m, y = value[1] / m, z = value[2] / m, w = value[3] / m;
      const s = 1 / Math.sqrt(x * x + y * y + z * z + w * w);
      return [x * s, y * s, z * s, w * s];
    }
  }
  return fail(caller, `${name} must be 4 finite numbers, not all zero, got ${show(value)}`);
}

/** 'box', 'sphere' or 'capsule', in that phrasing. */
export function oneOf(caller, name, value, choices) {
  if (!choices.includes(value)) {
    const listed = choices.map((c) => `'${c}'`);
    const phrase = listed.length === 1 ? listed[0]
      : `${listed.slice(0, -1).join(', ')} or ${listed[listed.length - 1]}`;
    fail(caller, `${name} must be ${phrase}, got ${show(value)}`);
  }
  return value;
}

const NONE = Object.freeze({});

/** The options object itself. Left out, it is `{}`: every field takes its default. */
export function options(caller, value, name = 'options') {
  if (value === undefined) return NONE;
  if (value === null || typeof value !== 'object' || Array.isArray(value)) {
    fail(caller, `${name} must be an object, got ${show(value)}`);
  }
  return value;
}

/**
 * Throws on any option name not in `names`, suggesting the nearest.
 *
 * A misspelt option that is quietly ignored is a test that quietly tests
 * nothing: Winding ran one for weeks with `castShadows` for `castShadow`.
 */
export function known(caller, given, names) {
  for (const key of Object.keys(given)) {
    if (names.includes(key)) continue;
    const near = nearest(key, names);
    fail(caller, near
      ? `unknown option '${key}' (did you mean '${near}'?)`
      : `unknown option '${key}' (the options are ${names.join(', ')})`);
  }
}

/**
 * The known name closest to a typo, or null when none is close.
 *
 * Close means an edit distance below half the typed name's length: more of
 * what was typed agrees with the suggestion than disagrees, so it is likelier
 * a slip than a different word. Past that, the message lists every option
 * instead of guessing. Case is ignored, so 'Mass' finds 'mass'. A tie goes to
 * the name listed first, so the same typo always gets the same answer.
 */
export function nearest(typed, names) {
  let best = null, bestDistance = Infinity;
  for (const name of names) {
    const d = distance(typed.toLowerCase(), name.toLowerCase());
    if (d < bestDistance) { best = name; bestDistance = d; }
  }
  return bestDistance < typed.length / 2 ? best : null;
}

/**
 * Edit distance counting a swap of two neighbours as one edit, as well as an
 * insertion, deletion or substitution (Damerau's optimal string alignment).
 * Swaps are the commonest typo, 'raduis' for 'radius', and plain Levenshtein
 * would count one as two.
 */
function distance(a, b) {
  const rows = a.length + 1, cols = b.length + 1;
  const d = new Array(rows * cols);
  for (let i = 0; i < rows; i++) d[i * cols] = i;
  for (let j = 0; j < cols; j++) d[j] = j;
  for (let i = 1; i < rows; i++) {
    for (let j = 1; j < cols; j++) {
      const cost = a[i - 1] === b[j - 1] ? 0 : 1;
      let v = Math.min(d[(i - 1) * cols + j] + 1, d[i * cols + j - 1] + 1, d[(i - 1) * cols + j - 1] + cost);
      if (i > 1 && j > 1 && a[i - 1] === b[j - 2] && a[i - 2] === b[j - 1]) {
        v = Math.min(v, d[(i - 2) * cols + j - 2] + 1);
      }
      d[i * cols + j] = v;
    }
  }
  return d[rows * cols - 1];
}
