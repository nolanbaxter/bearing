// Workers: a parallel-for over shared memory, for the parts of a step where
// each item is independent of the others. Winding's job system
// (src/core/jobs.js there) is the model, and most of what follows is its
// design, kept as it was where nothing here asked for a change.
//
// NOT A SCHEDULER. One atomic cursor over chunks of the items, and every
// thread, the calling one included, claims the next chunk until none are
// left. A thread given slow chunks simply claims fewer. No queues, no
// stealing, nothing per worker.
//
// THE CURSOR CARRIES THE EPOCH. Each dispatch has a number, and the cursor
// holds it beside the next chunk's index, so claiming a chunk is a
// compare-exchange that only succeeds for the dispatch the thread read its
// arguments from. A thread held up between reading them and claiming wakes
// into a later dispatch's cursor, fails, and takes nothing, rather than
// running the later dispatch's chunks with the earlier one's arguments
// (Winding's bug, before its cursor was tagged).
//
// THE CALLING THREAD WORKS TOO, and waits by spinning on the count of items
// done. A browser's main thread may not block in Atomics.wait, and by the time
// it has run its share of the chunks, at most one chunk per worker is left.
//
// THE SAME FUNCTION RUNS EITHER WAY. A job is a function over a range of
// items; with no workers, or too few items to be worth waking them, the
// calling thread runs it over the whole range. Every job here is one whose
// items are independent (see contact.js, JOBS), so the bits are the same
// however the range is cut.
//
// What differs from Winding's:
//
// - Ready is a count in shared memory, not a message. A message reaches the
//   main thread only when it returns to its event loop, and a program
//   stepping in a loop (a test, a benchmark, a server) never does: its
//   workers would never be seen as ready, and every step would run on one
//   thread.
// - Workers spin between dispatches while a step's dispatches come close
//   together (STAY, from begin() to end()), and sleep in Atomics.wait the
//   rest of the time. The solve dispatches once per colour per pass, 150 to
//   200 times a step, and a worker woken from Atomics.wait takes about 9 us to
//   answer: more than a small colour's whole pass. But the joints run on the
//   stepping thread alone, between the contact passes, and workers spinning
//   through them slowed it though they touched none of it: 32 ragdolls, whose
//   joints are half the step, took up to 1.45 times as long a step with 7
//   workers as with none, their joints twice as long (Node, i7-1260P).
//   Spinning cores share the chip's power and turbo with the stepping thread,
//   and may share its core. So the stepping thread says when it goes to work
//   alone (lull), and if that stretch last took longer than a wake costs, the
//   workers sleep through it; the next dispatch wakes them.
// - Workers start only when a step first has enough work for them (see
//   begin), not when the world is made: a world of a few bodies never needs
//   them, and a test suite making hundreds of small worlds would otherwise
//   start hundreds of threads.
// - The buffers are sent again when storage moves (see send), since a world's
//   columns grow and shrink.

// The control block: Int32Array slots, so Atomics can work on them.
const EPOCH = 0;       // moved on by each dispatch: what an idle worker waits on
const CURSOR = 1;      // the dispatch's tag beside the next chunk's index (see TAG)
const COMPLETED = 2;   // items finished
const FROM = 3;        // the first item
const COUNT = 4;       // how many items
const CHUNK = 5;       // items a chunk
const KIND = 6;        // which job
const SHUTDOWN = 7;    // 1 once the world is done with its workers
const DATA = 8;        // moved on each time the buffers are sent: which ones are current
const STAY = 9;        // 1 between begin() and end(), but for lulls: workers spin instead of sleeping
const READY = 10;      // workers that have their buffers and are listening
/** Workers in Atomics.wait, or about to be: a dispatch notifies only if there are any (see run). */
export const SLEEPING = 11;
const SLOTS = 12;

/**
 * The cursor's low 10 bits are the next chunk's index, and the 21 above them
 * the dispatch's epoch (the sign bit is left alone). A dispatch has at most
 * INDEX chunks (see run), and a thread would have to be held up across
 * exactly a multiple of 2^21 dispatches, about 8,700 steps of 240 dispatches,
 * two and a half minutes at 60 Hz, to mistake an old dispatch for a live one;
 * the hang bound has long given up on it by then.
 */
const INDEX_BITS = 10;
const INDEX = (1 << INDEX_BITS) - 1;
const TAG = 0x7fffffff & ~INDEX;
const tagOf = (epoch) => (epoch << INDEX_BITS) & TAG;

/**
 * Chunks a dispatch is cut into per thread. One each would leave every thread
 * waiting on the slowest, and a P-core and an E-core differ by about two
 * (the 50-box pile: 0.85 ms a step on one, 1.55 on the other); more are
 * claimed by whichever thread is free, but each claim is an atomic the
 * threads contend for. Measured on the i7-1260P with 3 workers, steps of the
 * 1,000-box pile and the pyramid: 2, 4 and 8 a thread within 3% of each
 * other, 1 20 to 25% slower, 16 5 to 8% slower.
 */
const PER_THREAD = 4;

/**
 * How long the calling thread waits for the workers before giving them up.
 * A hang bound, not a performance one (Winding's reasoning): a chunk takes
 * well under a millisecond, but a breakpoint, a background tab or a loaded
 * machine can hold a thread far longer, and a bound near the real cost would
 * take that for a broken worker. Only a genuine failure, a worker that died
 * or never loaded, takes this long.
 */
const HANG_MS = 2000;

/**
 * How long a stretch of the stepping thread's work alone has to have lasted,
 * in ms, for the workers to sleep through it the next time (see lull): about
 * what a wake costs. Spinning through a stretch can cost the stepping thread
 * up to its whole length (the ragdolls' joints took twice as long beside
 * spinning workers), a wake about the same whatever the stretch. On the
 * i7-1260P in Node, 7 workers, a worker asleep in Atomics.wait answered in
 * 9 us (median; 12 us at the 90th percentile) against 0.5 us spinning, but a
 * wake cost a step 28 to 45 us all told, waiting on the late worker included
 * (the 1,000-box pile woken 25 times a step, 10% slower; 32 ragdolls' contact
 * passes 0.85 to 1.1 ms slower for 24 wakes). The ragdolls' joint passes are
 * 195 to 913 us (10th to 90th percentile), so they sleep through them; a
 * world of a few joints, whose passes take microseconds, keeps its workers
 * spinning.
 *
 * Why the stepping thread decides, and not a bound on how long a worker
 * spins idle: the other gaps, where the stepping thread moves the bodies,
 * grow with their number, 24 to 33 us on the 1,000-box pile and over 100 us
 * at 8,000, and a bound short enough to free the joints sent the 8,000-box
 * pile's workers to sleep at every substep, 25% slower.
 */
const LULL_MS = 0.05;

/**
 * How many stretches lull() keeps a length for: world.js's three joint
 * passes (warm start, solve, relax), whose lengths differ several times over.
 */
const LULLS = 3;

// Absent on a browser page that isn't cross-origin isolated (COOP and COEP
// headers): without it, workers can't share a world's memory, and everything
// runs on one thread, with the same results.
const SHARED = globalThis.SharedArrayBuffer;
export const sharedMemory = SHARED !== undefined
  && (typeof globalThis.crossOriginIsolated !== 'boolean' || globalThis.crossOriginIsolated);

/** `array` in shared memory: itself if it is, else a copy. */
function share(array) {
  if (array.buffer instanceof SHARED) return array;
  const next = new array.constructor(new SHARED(array.byteLength));
  next.set(array);
  return next;
}

/**
 * Every world's workers end when it is collected, if it was dropped without
 * destroy(): a worker asleep in Atomics.wait would otherwise sit there for as
 * long as the page or process lasts. One registry serves every world, as the
 * engine's own collector does; no world can reach another's workers through
 * it, so it is not state two worlds share (rule 38).
 */
const ENDS = new FinalizationRegistry((jobs) => jobs.destroy());

/** How many threads the machine runs at once, or 1 if it won't say. */
export function cores() {
  const os = globalThis.process?.getBuiltinModule?.('node:os');
  if (os) return os.availableParallelism();
  return globalThis.navigator?.hardwareConcurrency ?? 1;
}

/**
 * Starts a module worker running `url`: a Worker where there is one
 * (browsers, Deno, Bun), Node's worker_threads otherwise; or null.
 */
function spawn(url) {
  const Worker = globalThis.Worker;
  if (typeof Worker === 'function') {
    // A page may start a worker only from its own origin; Bearing imported
    // from a CDN is started through a one-line module of the page's own that
    // imports it (module imports may cross origins, a worker's script may not).
    const origin = globalThis.location?.origin;
    const src = origin === undefined || url.origin === origin ? url
      : URL.createObjectURL(new Blob([`import ${JSON.stringify(url.href)};`], { type: 'text/javascript' }));
    const worker = new Worker(src, { type: 'module' });
    worker.unref?.();
    return worker;
  }
  // Node: getBuiltinModule is synchronous, so workers start within the step
  // that asks for them. Node 20 before 20.16 lacks it, and runs on one thread.
  const threads = globalThis.process?.getBuiltinModule?.('node:worker_threads');
  if (!threads) return null;
  // Without the program's own options: a worker inherits them, and a
  // program run as `node -e …` would have its workers run that instead.
  const worker = new threads.Worker(url, { execArgv: [] });
  // A worker left running would keep Node from exiting when the program ends.
  worker.unref();
  return worker;
}

export class Jobs {
  /**
   * `work` is { jobs, least, columns, pairs } (contact.js, WORK): the job
   * functions by kind, the fewest items each is worth dispatching, and the
   * world's columns and the pair list's they read. `count` is how many
   * workers, besides the calling thread; `url` the worker module.
   */
  constructor(work, count, url) {
    this.jobs = work.jobs;
    this.least = work.least;
    this.columns = work.columns;
    this.pairs = work.pairs;
    this.count = count;
    this.url = url;
    this.control = null;
    /** Numbers a dispatch's job reads besides its range, as Float64 (contact.js says which). */
    this.args = null;
    this.workers = [];
    /** What was last sent, in order: the world's columns, then the contacts' arrays (see send). */
    this.sent = [];
    this.keys = null;
    /** True between begin() and end() when the step's dispatches may run on several threads. */
    this.live = false;
    /** The stretch lull() began, until the next dispatch, or -1; when it began; how long each last took, in ms. */
    this.lulled = -1;
    this.lullAt = 0;
    this.solo = new Float64Array(LULLS);
    this.broken = false;
    /**
     * Dispatches that ran on several threads. The plain loop gives the same
     * bits by design, so no result can tell the two apart: without this, a
     * test of the parallel path would pass whether or not it ever ran.
     */
    this.parallel = 0;
  }

  /**
   * Called once a step has its contacts, before the first dispatch: whether
   * this step's dispatches may go to the workers. `worth` is whether the step
   * has a dispatch big enough to want them. The first time it does, the
   * workers are started, and the step runs on one thread, as every step does
   * until they all report ready; then each step sends the buffers again if
   * any moved, and keeps the workers spinning until end(). A step not worth
   * them still sends: storage that shrank after a burst is then let go by the
   * workers too, rather than held until the world is next busy.
   */
  begin(world, c, worth) {
    if (this.broken) return false;
    if (this.control === null) {
      if (worth) this.start(world, c);
      return false;
    }
    this.send(world, c);
    if (!worth || Atomics.load(this.control, READY) < this.count) return false;
    Atomics.store(this.control, STAY, 1);
    this.live = true;
    return true;
  }

  /** The step's dispatches are over: the workers go back to sleep. */
  end() {
    if (!this.live) return;
    this.live = false;
    this.lulled = -1;
    Atomics.store(this.control, STAY, 0);
  }

  /**
   * The stepping thread goes to work alone, on stretch `k` (one of LULLS):
   * the workers sleep through it if it last took longer than LULL_MS, and
   * the next dispatch (run) wakes them, and times it. The time decides only
   * who waits how, never a result.
   */
  lull(k) {
    if (!this.live) return;
    this.lulled = k;
    this.lullAt = performance.now();
    if (this.solo[k] > LULL_MS) Atomics.store(this.control, STAY, 0);
  }

  /**
   * Moves what the jobs read into shared memory, and starts the workers. The
   * world's columns and the contacts' arrays are replaced by shared copies,
   * which stay shared as they grow and shrink (core/slots.js, resized), so
   * this happens once.
   */
  start(world, c) {
    for (const name of this.columns) world[name] = share(world[name]);
    for (const name of this.pairs) world._pairs[name] = share(world._pairs[name]);
    this.keys = [];
    for (const key of Object.keys(c)) {
      if (!ArrayBuffer.isView(c[key])) continue;
      this.keys.push(key);
      c[key] = share(c[key]);
    }
    this.control = new Int32Array(new SHARED(SLOTS * 4));
    // contact.js says which is which: 0 to 4 the solve's, 5 to 9 the narrowphase's.
    this.args = new Float64Array(new SHARED(10 * 8));
    for (let i = 0; i < this.count; i++) {
      let worker = null;
      // Refused, say, by Node's permission model without --allow-worker.
      try { worker = spawn(this.url); } catch { worker = null; }
      if (worker === null) break;
      // A worker that fails (its module didn't load, say) never reports ready,
      // so steps stay on one thread; and an error left unheard would end a
      // Node process.
      const lost = () => { this.broken = true; };
      if (worker.addEventListener) worker.addEventListener('error', lost);
      else worker.on('error', lost);
      this.workers.push(worker);
    }
    if (this.workers.length < this.count) this.broken = true;
    ENDS.register(world, this, this);
    this.send(world, c);
  }

  /**
   * Sends the workers the shared arrays, if any has moved since they were
   * last sent: grown or shrunk storage is a new array (resized). A worker sits
   * in its loop and reads no messages, so DATA moves on first, and every
   * waiting worker is woken to see it, go back to its event loop, and read the
   * new ones. Until it has, the other threads take its chunks. Nothing is
   * made unless something moved, so a step in steady state allocates nothing.
   */
  send(world, c) {
    const columns = this.columns, pairs = this.pairs, keys = this.keys, sent = this.sent, list = world._pairs;
    let moved = sent.length === 0, i = 0;
    for (let k = 0; k < columns.length && !moved; k++) moved = sent[i++] !== world[columns[k]];
    for (let k = 0; k < pairs.length && !moved; k++) moved = sent[i++] !== list[pairs[k]];
    for (let k = 0; k < keys.length && !moved; k++) moved = sent[i++] !== c[keys[k]];
    if (!moved) return;
    // A worker's world: the columns, the pair list's, and a gravity of its
    // own that the jobs fill in from args (contact.js, lookRows).
    const views = { _pairs: {}, _gravity: [0, 0, 0] }, rows = {};
    sent.length = 0;
    for (const name of columns) sent.push(views[name] = world[name]);
    for (const name of pairs) sent.push(views._pairs[name] = list[name]);
    for (const key of keys) sent.push(rows[key] = c[key]);
    const revision = Atomics.add(this.control, DATA, 1) + 1;
    const message = { control: this.control, args: this.args, world: views, c: rows, revision };
    for (const worker of this.workers) worker.postMessage(message);
    Atomics.notify(this.control, EPOCH);
  }

  /**
   * Runs job `kind` over items from..to-1, on every thread if the workers are
   * live and there are at least the job's least, else here. Returns when
   * every item is done.
   */
  run(kind, world, c, from, to) {
    const job = this.jobs[kind], n = to - from;
    if (!this.live || n < this.least[kind]) {
      job(world, c, this.args, from, to);
      return;
    }
    this.parallel++;
    const control = this.control;
    const chunks = Math.min((this.count + 1) * PER_THREAD, INDEX);
    const chunk = Math.ceil(n / chunks);
    if (this.lulled >= 0) {
      this.solo[this.lulled] = performance.now() - this.lullAt;
      this.lulled = -1;
      Atomics.store(control, STAY, 1);
    }
    const epoch = (Atomics.load(control, EPOCH) + 1) | 0;
    Atomics.store(control, CURSOR, tagOf(epoch));
    Atomics.store(control, COMPLETED, 0);
    Atomics.store(control, FROM, from);
    Atomics.store(control, COUNT, n);
    Atomics.store(control, CHUNK, chunk);
    Atomics.store(control, KIND, kind);
    // The epoch last: a worker reads the rest only once it sees a new epoch,
    // and the Atomics order the two.
    Atomics.store(control, EPOCH, epoch);
    // A notify takes a lock even with no one waiting, and most dispatches
    // find every worker spinning. A worker counts itself asleep before it
    // waits, and the wait returns at once if the epoch has moved on, so with
    // the Atomics' one order either this sees it counted or it sees the epoch.
    if (Atomics.load(control, SLEEPING) > 0) Atomics.notify(control, EPOCH);
    claim(control, job, world, c, this.args, epoch);
    if (Atomics.load(control, COMPLETED) < n) {
      const deadline = performance.now() + HANG_MS;
      while (Atomics.load(control, COMPLETED) < n) {
        if (performance.now() > deadline) this.lost(n);
      }
    }
  }

  /** The workers stopped answering mid-dispatch: the step can't finish. */
  lost(n) {
    const done = Atomics.load(this.control, COMPLETED);
    this.end();
    this.broken = true;
    this.destroy();
    throw new Error(`step: the workers stopped responding after ${HANG_MS} ms, with ${done} of ${n} items done; `
      + 'this step is lost, and the world runs on one thread from now on');
  }

  /** Ends the workers. Nothing keeps running (rule 39). */
  destroy() {
    ENDS.unregister(this);
    this.live = false;
    if (this.control !== null) {
      Atomics.store(this.control, SHUTDOWN, 1);
      Atomics.store(this.control, STAY, 0);
      Atomics.add(this.control, EPOCH, 1);
      Atomics.notify(this.control, EPOCH);
    }
    for (const worker of this.workers) worker.terminate();
    this.workers.length = 0;
  }

  /** For tests: whether every worker reported ready within the hang bound. */
  async ready() {
    const deadline = performance.now() + HANG_MS;
    while (this.control === null || Atomics.load(this.control, READY) < this.count) {
      if (this.broken || performance.now() > deadline) return false;
      await new Promise((resolve) => setTimeout(resolve, 1));
    }
    return true;
  }
}

/**
 * Claims and runs chunks of the dispatch `epoch` until none are left: the
 * loop every thread runs, the calling one included. Claiming and finishing
 * are counted apart: the cursor says what was handed out, COMPLETED what is
 * done, and the caller waits on the second, since the last chunk handed out
 * may still be running.
 */
function claim(control, job, world, c, args, epoch) {
  const from = Atomics.load(control, FROM), count = Atomics.load(control, COUNT), chunk = Atomics.load(control, CHUNK);
  const tag = tagOf(epoch);
  for (;;) {
    const cursor = Atomics.load(control, CURSOR);
    // Another dispatch owns the cursor now: nothing here is this thread's.
    if ((cursor & TAG) !== tag) return;
    const index = cursor & INDEX, start = index * chunk;
    if (start >= count) return;
    // Lost the race to another thread: read again and decide again.
    if (Atomics.compareExchange(control, CURSOR, cursor, tag | (index + 1)) !== cursor) continue;
    const end = Math.min(start + chunk, count);
    job(world, c, args, from + start, from + end);
    Atomics.add(control, COMPLETED, end - start);
  }
}

/**
 * A worker's loop: wait for a dispatch, take part in it, again. Returns when
 * its buffers stop being the current ones (so its event loop can deliver the
 * newer ones, already sent), or at shutdown. `world` and `c` are plain
 * objects holding the shared columns and rows under the World's and
 * Contacts' own names, so the job functions read them just as they read the
 * real ones.
 */
export function work(message, jobs) {
  const { control, args, world, c, revision } = message;
  let seen = Atomics.load(control, EPOCH);
  for (;;) {
    if (Atomics.load(control, DATA) !== revision || Atomics.load(control, SHUTDOWN) === 1) return;
    const epoch = Atomics.load(control, EPOCH);
    if (epoch === seen) {
      if (Atomics.load(control, STAY) === 1) continue;
      // Counted first, so a dispatch knows to wake it (see run).
      Atomics.add(control, SLEEPING, 1);
      Atomics.wait(control, EPOCH, seen);
      Atomics.sub(control, SLEEPING, 1);
      continue;
    }
    seen = epoch;
    const job = jobs[Atomics.load(control, KIND)];
    // `seen` is the dispatch these arguments belong to: if another has begun
    // since, claim()'s tag check takes nothing from it.
    if (job !== undefined && Atomics.load(control, DATA) === revision) claim(control, job, world, c, args, seen);
  }
}

/** A worker reports in, once, as it first takes up its buffers. */
export function report(control) {
  Atomics.add(control, READY, 1);
}
