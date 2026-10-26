// A world's worker (see core/jobs.js): it takes the shared buffers it is
// sent, reports ready once, and runs its share of each dispatch. It imports
// the very functions the calling thread runs (contact.js, JOBS), and holds no
// logic of its own: anything that lived here would be something one thread
// doesn't do, and the two would sooner or later disagree.

import { work, report } from './core/jobs.js';
import { JOBS } from './contact.js';

let reported = false;

function take(message) {
  if (!reported) {
    reported = true;
    report(message.control);
  }
  work(message, JOBS);
}

// A browser's, Deno's or Bun's worker has `self`; a Node worker has its parent's port.
if (typeof self !== 'undefined') self.onmessage = (event) => take(event.data);
else globalThis.process.getBuiltinModule('node:worker_threads').parentPort.on('message', take);
