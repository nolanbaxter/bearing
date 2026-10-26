// The contact graph's colours: the order the solver visits a step's manifolds in.
//
// Each pass of the solver pushes at every manifold in turn, and each push
// changes its two bodies' velocities, which the next manifold on either body
// reads. Two manifolds that share no body the solver moves are independent:
// solved in either order, they give the same bits. So the manifolds are
// split into colours, no two in one colour sharing a dynamic body, and the
// colours are solved one after another, each pass (Box2D 3's constraint
// graph, from Catto's Solver2D). Within a colour the order makes no
// difference, so a colour can be cut into chunks and solved on several
// threads at once (milestone 16's workers) with the very bits one thread gets.
//
// Static, kinematic and character bodies don't count. They have no inverse
// mass, so the solver reads their velocities and never changes them, and
// every box resting on one floor can share a colour.
//
// MASKS. Each body holds, for the step, the colours its manifolds have taken,
// as the bits of one 32-bit integer, and a manifold takes the lowest colour
// neither of its dynamic bodies holds: one Math.clz32, where Box2D keeps a
// bitset over every body for each colour and tries the colours in turn. That
// is where the 32 colours come from: the width of the integers JavaScript's
// bitwise operators work on. A manifold whose bodies' other manifolds hold
// all 32 between them goes in the overflow, solved after the colours, one
// manifold at a time in pair order, as a 33rd colour would be if the integer
// were wider. On one thread that is no different from a colour; with workers
// it is the part one thread solves alone, as Box2D's overflow is. A body's
// other manifolds block at most one colour each, so a manifold between two
// dynamic bodies with at most 16 manifolds each, or on a static body with a
// dynamic one that has at most 32, always has a colour. Measured, no body
// had more than 8 manifolds in the benchmark's 1,000-box pile or 9 in a heap
// of rocks and boxes, and no step used more than 9 colours (12 with jointed
// chains draped over boxes), so the overflow is for a body many others lean
// on, such as a dynamic slab carrying a crowd.
//
// AFRESH EACH STEP. The manifolds are coloured in the order narrow() found
// them, pair order, each taking the lowest colour free on its bodies, so the
// colours depend on that step's manifolds alone, and a saved world, loaded,
// colours its next step as the original does with nothing saved for it.
// Box2D keeps a constraint's colour for as long as it lasts instead, giving
// only a new one a colour, since it keeps its colours' lists from step to
// step; Bearing makes its manifolds afresh each step anyway. Kept colours
// would hold the solve order steadier: afresh, one contact beginning moves
// the colours of manifolds after it in pair order that share its bodies, and
// so on. Both were measured (the colour kept on the pair with its impulses):
// on a 288-box heap landing, 4 to 5% of the manifolds that lasted from one
// step to the next changed colour, none once it had settled, and the time
// 16 starts each of 1,000- and 288-box heaps took to sleep, and the contact
// ids they lost while settling, were the same within what one start differs
// from the next. So nothing is kept.
//
// LAID OUT BY COLOUR. Once coloured, the manifolds' rows are moved so each
// colour's lie together (in b's slot order, see DIGIT below), colour 0 first and
// the overflow last, and every pass of the solver walks the rows straight
// through. Left where they were found and visited through a list, each pass
// swept the rows once a colour, picking out a few here and there: the
// solver's 24 passes a step then read memory out of order, and measured
// pinned on the i7-1260P, colouring that way cost 2 to 6% on a pyramid and
// terrain and 12 to 14% on a pile of 8,000 boxes. Moving them costs one copy
// a step of what narrow() wrote, before prepare() and the solver write the
// rest straight into place. What runs once a step over the manifolds in pair
// order (store, touched, restitution) goes through c.row, so a mesh pair's
// manifolds still come together and events in the order they always did.

const DYNAMIC = 0;   // World's type code

/** How many colours there are: the bits of a 32-bit integer (see MASKS). */
export const COLOURS = 32;
/** The colour number a manifold in the overflow is given, after every real one. */
export const OVERFLOW = COLOURS;

const P = 4;   // points a manifold holds (contact.js)
// Every column narrow() writes, with its numbers per manifold: what arrange()
// moves. The rest prepare() and the solver write afresh in place, so a column
// narrow() comes to fill must be listed here. By element type, each moved by
// a function of its own (see arrange).
const FLOATS = [
  ['normal', 3], ['t1', 3], ['t2', 3], ['friction', 1], ['restitution', 1], ['tolerance', 1], ['patch', 1],
  ['rollImpulse1', 1], ['rollImpulse2', 1], ['spinImpulse', 1], ['tangentImpulse1', 1], ['tangentImpulse2', 1],
  ['drift1', 1], ['drift2', 1], ['separation', P], ['normalImpulse', P], ['point', 3 * P], ['shift', 3 * P],
];
const INTS = ['a', 'b', 'pair', 'sub', 'key', 'partA', 'partB', 'colour'];   // Int32Array, one a manifold

/**
 * Within a colour, the rows go by b's slot (b is the pair's higher slot, and
 * the dynamic body whenever the other is static), sorted a digit at a time:
 * a slot is 24 bits (core/slots.js), so two digits of 12, each a table of
 * 4,096 counts rather than 16 million. A body's state lies in the world's
 * columns by slot, so a sweep of a colour then reads it at rising addresses.
 * Colouring alone scattered it: two manifolds next to each other in a colour
 * never share a body, as two found together often did, and on 8,000 boxes
 * the solve spent 15% longer reading its bodies (cpu profile, i7-1260P). The
 * order within a colour changes no result.
 */
const DIGIT = 12;
export const DIGITS = 1 << DIGIT;

/**
 * Gives each of c's manifolds a colour, in c.colour, and lays the rows out
 * colour by colour (see LAID OUT BY COLOUR): colour k's are rows
 * c.starts[k] .. c.starts[k + 1], and the overflow's the last. Uses
 * world._colours, each body's mask (see MASKS).
 */
export function colour(world, c) {
  const used = world._colours, type = world._type, A = c.a, B = c.b, colour = c.colour, n = c.count;
  // Only the masks of bodies with manifolds are read, so only theirs are cleared.
  for (let m = 0; m < n; m++) {
    used[A[m]] = 0;
    used[B[m]] = 0;
  }
  for (let m = 0; m < n; m++) {
    const a = A[m], b = B[m];
    const da = type[a] === DYNAMIC, db = type[b] === DYNAMIC;
    const held = (da ? used[a] : 0) | (db ? used[b] : 0);
    if (held === -1) {
      colour[m] = OVERFLOW;
      continue;
    }
    // The lowest bit held clear: ~held & (held + 1) keeps just that bit.
    const bit = ~held & (held + 1);
    colour[m] = 31 - Math.clz32(bit);
    if (da) used[a] |= bit;
    if (db) used[b] |= bit;
  }
  // The order: stable counting sorts by b's slot's low digit, its high one
  // (needed only past 4,096 slots), then colour, so each colour's rows go by
  // b's slot (see DIGIT). c.row holds a list on the way, until arrange()
  // makes it the way back.
  const count = c.digits, order = c.order, row = c.row, high = world._slots.top > DIGITS;
  let from = high ? order : row;
  count.fill(0);
  for (let m = 0; m < n; m++) count[(B[m] & (DIGITS - 1)) + 1]++;
  for (let k = 0; k < DIGITS; k++) count[k + 1] += count[k];
  for (let m = 0; m < n; m++) from[count[B[m] & (DIGITS - 1)]++] = m;
  if (high) {
    count.fill(0);
    for (let i = 0; i < n; i++) count[(B[order[i]] >>> DIGIT) + 1]++;
    for (let k = 0; k < DIGITS; k++) count[k + 1] += count[k];
    for (let i = 0; i < n; i++) {
      const m = order[i];
      row[count[B[m] >>> DIGIT]++] = m;
    }
    from = row;
  }
  const starts = c.starts;
  starts.fill(0);
  for (let m = 0; m < n; m++) starts[colour[m] + 1]++;
  for (let k = 0; k <= OVERFLOW; k++) starts[k + 1] += starts[k];
  for (let i = 0; i < n; i++) {
    const m = from[i];
    order[starts[colour[m]]++] = m;
  }
  // Each run's start was moved on to its end, the next run's start: back by one run.
  for (let k = OVERFLOW; k > 0; k--) starts[k] = starts[k - 1];
  starts[0] = 0;
  arrange(c, order, n);
}

/**
 * Moves c's rows so row o holds the manifold found order[o]-th, and c.row is
 * the way back. Each element type has a mover of its own: one loop reading
 * Float64Array, Int32Array, Uint32Array and Uint8Array columns in turn made
 * every access in it generic, and took 9 to 11% of a step (cpu profile).
 */
function arrange(c, order, n) {
  const room = c.scratch;
  for (let k = 0; k < FLOATS.length; k++) moveFloats(c[FLOATS[k][0]], FLOATS[k][1], room, order, n);
  for (let k = 0; k < INTS.length; k++) moveInts(c[INTS[k]], room, order, n);
  moveIds(c.pid, room, order, n);
  moveCounts(c.points, room, order, n);
  for (let o = 0; o < n; o++) c.row[order[o]] = o;
}

// The movers, alike but for the column's type, so each stays monomorphic.
// Every value is exact in a double, so the room is one Float64Array for all.

function moveFloats(column, per, room, order, n) {
  for (let i = 0; i < n * per; i++) room[i] = column[i];
  for (let o = 0; o < n; o++) {
    const from = order[o] * per, to = o * per;
    for (let i = 0; i < per; i++) column[to + i] = room[from + i];
  }
}

function moveInts(column, room, order, n) {
  for (let i = 0; i < n; i++) room[i] = column[i];
  for (let o = 0; o < n; o++) column[o] = room[order[o]];
}

function moveIds(column, room, order, n) {
  for (let i = 0; i < n * P; i++) room[i] = column[i];
  for (let o = 0; o < n; o++) {
    const from = order[o] * P, to = o * P;
    for (let i = 0; i < P; i++) column[to + i] = room[from + i];
  }
}

function moveCounts(column, room, order, n) {
  for (let i = 0; i < n; i++) room[i] = column[i];
  for (let o = 0; o < n; o++) column[o] = room[order[o]];
}
