// Minimal static server for the demo. `node serve.js`, then open
// http://localhost:8080/demo/
//
// Needed because ES modules and fetch() are blocked over file://. Node's own
// http and fs cover it, so there is no dependency here and never needs to be
// one. It is Winding's serve.js, less the cross-origin isolation headers:
// Bearing needs no SharedArrayBuffer, and those headers would stop the demo
// loading Winding from jsDelivr unless the CDN opted in. Winding's job system
// runs inline without them, with identical results.
//
// Not part of the package: package.json's `files` publishes src/ and the API
// reference only.

import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { extname, join, normalize, sep } from 'node:path';

const ROOT = import.meta.dirname;
const PORT = Number(process.argv[2] ?? 8080);

const TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.md': 'text/markdown; charset=utf-8',
  '.svg': 'image/svg+xml',
  '.png': 'image/png',
};

createServer(async (req, res) => {
  try {
    let path = decodeURIComponent(new URL(req.url, 'http://localhost').pathname);
    if (path.endsWith('/')) path += 'index.html';

    // Reject traversal before touching the filesystem: normalize resolves the
    // '..' segments, then the result must still be inside ROOT. The separator
    // is part of the test, or a sibling directory whose name merely extends
    // the root's would pass.
    const full = normalize(join(ROOT, path));
    if (full !== ROOT && !full.startsWith(ROOT + sep)) {
      res.writeHead(403).end('forbidden');
      return;
    }

    const body = await readFile(full);
    res.writeHead(200, {
      'content-type': TYPES[extname(full)] ?? 'application/octet-stream',
      'cache-control': 'no-store',
    }).end(body);
  } catch {
    res.writeHead(404).end('not found');
  }
}).listen(PORT, '127.0.0.1', () => {
  console.log(`http://localhost:${PORT}/demo/`);
});
