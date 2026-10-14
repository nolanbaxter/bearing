// Reading a model's triangles from a .glb file, for a mesh body.
//
// A .glb is glTF 2.0 in one binary file: a 12-byte header, a chunk of JSON
// that describes the scene, and a chunk of bytes that holds the numbers. The
// JSON says which nodes there are, how each is placed under its parent, and
// which mesh each shows; a mesh is primitives, each pointing at accessors,
// which say where in the bytes its corner positions and its triangles' corner
// numbers are, and in what type. Reading it needs nothing but a DataView, so
// Bearing reads it itself (rule 70) and depends on nothing.
//
// What comes back is every triangle of the scene, with each node's place
// applied, so the mesh lines up with the model as any engine draws it:
// metres, y up, as glTF and Bearing both are. Points and lines have no
// triangles and are left out. Materials, textures, normals and animation say
// nothing about shape, and are not read.
//
// Compressed geometry (Draco, meshopt) and quantized positions need a
// decoder, a dependency, so a file that needs one throws, naming it. Every
// exporter can leave them off.

import { fail, show, options, known } from './core/check.js';

const CALLER = 'readGlb';
const MAGIC = 0x46546c67;                     // 'glTF', little-endian
const JSON_CHUNK = 0x4e4f534a, BIN_CHUNK = 0x004e4942;   // 'JSON', 'BIN\0'
/** Extensions that change how geometry is stored; reading through one needs its decoder. */
const NEEDS_DECODER = ['KHR_draco_mesh_compression', 'EXT_meshopt_compression', 'KHR_mesh_quantization'];
/** glTF's component types: bytes each, and how to read one. */
const COMPONENTS = {
  5121: [1, (v, o) => v.getUint8(o)],
  5123: [2, (v, o) => v.getUint16(o, true)],
  5125: [4, (v, o) => v.getUint32(o, true)],
  5126: [4, (v, o) => v.getFloat32(o, true)],
};
const TRIANGLES = 4, STRIP = 5, FAN = 6;   // a primitive's modes; 0 to 3 are points and lines

/**
 * The triangles of a .glb file's scene, with every node's place applied:
 * { vertices, indices }, ready for addBody({ shape: 'mesh', ... }).
 * `bytes` is the file, an ArrayBuffer or a typed array (as fetch's
 * arrayBuffer() or Node's readFile give it). options.node names one node:
 * only it and the nodes under it are read, still placed as in the scene.
 */
export function readGlb(bytes, options_) {
  const o = options(CALLER, options_);
  known(CALLER, o, ['node']);
  if (o.node !== undefined && typeof o.node !== 'string') fail(CALLER, `node must be a string, got ${show(o.node)}`);
  const view = bytesOf(bytes);
  const { gltf, bin } = chunks(view);
  for (const name of gltf.extensionsRequired || []) {
    if (NEEDS_DECODER.includes(name)) needs(name);
  }
  const nodes = gltf.nodes || [];
  // Each node's parent, so a named node is placed with everything above it.
  const parent = new Int32Array(nodes.length).fill(-1);
  nodes.forEach((n, i) => {
    for (const c of n.children || []) {
      if (!(c >= 0 && c < nodes.length) || parent[c] >= 0 || c === i) damaged(`node ${i} has a bad child, ${show(c)}`);
      parent[c] = i;
    }
  });
  let roots;
  if (o.node !== undefined) {
    const at = nodes.findIndex((n) => n.name === o.node);
    if (at < 0) fail(CALLER, `node must be the name of a node in the file, got ${show(o.node)}`);
    roots = [at];
  } else if (gltf.scenes && gltf.scenes.length > 0) {
    const scene = gltf.scenes[gltf.scene || 0];
    if (!scene) damaged(`no scene ${show(gltf.scene)}`);
    roots = scene.nodes || [];
  } else {
    roots = nodes.map((_, i) => i).filter((i) => parent[i] < 0);
  }
  const vertices = [], indices = [];
  for (const r of roots) {
    if (!(r >= 0 && r < nodes.length)) damaged(`the scene lists node ${show(r)}, which isn't there`);
    walk(gltf, bin, nodes, r, placed(nodes, parent, r), vertices, indices, 0);
  }
  return { vertices: Float64Array.from(vertices), indices: Uint32Array.from(indices) };
}

function needs(what) {
  fail(CALLER, `the file needs ${what}, which Bearing doesn't read; export it without that`);
}

function damaged(why) {
  fail(CALLER, `the file is damaged: ${why}`);
}

/** The file's bytes as a DataView. */
function bytesOf(bytes) {
  if (bytes instanceof ArrayBuffer) return new DataView(bytes);
  if (ArrayBuffer.isView(bytes)) return new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  return fail(CALLER, `bytes must be an ArrayBuffer or a typed array, got ${show(bytes)}`);
}

/** The JSON, parsed, and the binary chunk, as a DataView. */
function chunks(view) {
  if (view.byteLength < 12 || view.getUint32(0, true) !== MAGIC) {
    fail(CALLER, 'bytes must be a binary glTF 2.0 file (.glb), got a file that doesn\'t start with glTF');
  }
  const version = view.getUint32(4, true);
  if (version !== 2) fail(CALLER, `bytes must be a binary glTF 2.0 file (.glb), got glTF version ${version}`);
  const length = Math.min(view.getUint32(8, true), view.byteLength);
  let gltf = null, bin = null;
  for (let at = 12; at + 8 <= length;) {
    const size = view.getUint32(at, true), type = view.getUint32(at + 4, true), start = at + 8;
    if (start + size > length) damaged('a chunk runs past the end of the file');
    if (type === JSON_CHUNK && gltf === null) {
      const text = new TextDecoder().decode(new Uint8Array(view.buffer, view.byteOffset + start, size));
      try {
        gltf = JSON.parse(text);
      } catch {
        damaged('its JSON doesn\'t parse');
      }
    } else if (type === BIN_CHUNK && bin === null) {
      bin = new DataView(view.buffer, view.byteOffset + start, size);
    }
    // Chunks are padded to 4 bytes; any other kind is skipped, as the format says.
    at = start + size + ((4 - (size % 4)) % 4);
  }
  if (gltf === null || typeof gltf !== 'object') damaged('it has no JSON chunk');
  return { gltf, bin };
}

/** Node i's place in the scene: its parents' places, then its own. A 4x4 matrix, columns first. */
function placed(nodes, parent, i) {
  let m = local(nodes[i]);
  for (let p = parent[i], steps = 0; p >= 0; p = parent[p]) {
    if (++steps > nodes.length) damaged('its nodes are their own ancestors');
    m = times(local(nodes[p]), m);
  }
  return m;
}

/** A node's own place, under its parent: its matrix, or its translation, rotation and scale. */
function local(n) {
  if (n.matrix) {
    if (n.matrix.length !== 16) damaged('a node\'s matrix isn\'t 16 numbers');
    return n.matrix.slice();
  }
  const [tx, ty, tz] = n.translation || [0, 0, 0];
  const [x, y, z, w] = n.rotation || [0, 0, 0, 1];
  const [sx, sy, sz] = n.scale || [1, 1, 1];
  return [
    (1 - 2 * (y * y + z * z)) * sx, 2 * (x * y + z * w) * sx, 2 * (x * z - y * w) * sx, 0,
    2 * (x * y - z * w) * sy, (1 - 2 * (x * x + z * z)) * sy, 2 * (y * z + x * w) * sy, 0,
    2 * (x * z + y * w) * sz, 2 * (y * z - x * w) * sz, (1 - 2 * (x * x + y * y)) * sz, 0,
    tx, ty, tz, 1,
  ];
}

/** a b, both 4x4, columns first. */
function times(a, b) {
  const out = new Array(16);
  for (let c = 0; c < 4; c++) {
    for (let r = 0; r < 4; r++) {
      out[4 * c + r] = a[r] * b[4 * c] + a[4 + r] * b[4 * c + 1] + a[8 + r] * b[4 * c + 2] + a[12 + r] * b[4 * c + 3];
    }
  }
  return out;
}

/** Node i's mesh, placed by m, then its children's, into vertices and indices. */
function walk(gltf, bin, nodes, i, m, vertices, indices, depth) {
  if (depth > nodes.length) damaged('its nodes are their own ancestors');
  const n = nodes[i];
  if (n.mesh !== undefined) {
    const mesh = (gltf.meshes || [])[n.mesh];
    if (!mesh) damaged(`node ${i} shows mesh ${show(n.mesh)}, which isn't there`);
    for (const p of mesh.primitives || []) primitive(gltf, bin, p, m, vertices, indices);
  }
  for (const c of n.children || []) walk(gltf, bin, nodes, c, times(m, local(nodes[c])), vertices, indices, depth + 1);
}

/** One primitive's triangles, placed by m. */
function primitive(gltf, bin, p, m, vertices, indices) {
  for (const name of Object.keys(p.extensions || {})) if (NEEDS_DECODER.includes(name)) needs(name);
  const mode = p.mode === undefined ? TRIANGLES : p.mode;
  if (mode !== TRIANGLES && mode !== STRIP && mode !== FAN) return;
  if (!p.attributes || p.attributes.POSITION === undefined) damaged('a primitive has no positions');
  const positions = accessor(gltf, bin, p.attributes.POSITION, 'VEC3');
  if (positions.type !== 5126) needs(`positions stored as component type ${positions.type} (KHR_mesh_quantization)`);
  const base = vertices.length / 3, count = positions.count;
  for (let k = 0; k < count; k++) {
    const x = positions.get(3 * k), y = positions.get(3 * k + 1), z = positions.get(3 * k + 2);
    vertices.push(m[0] * x + m[4] * y + m[8] * z + m[12], m[1] * x + m[5] * y + m[9] * z + m[13],
      m[2] * x + m[6] * y + m[10] * z + m[14]);
  }
  let corner;
  let corners;
  if (p.indices === undefined) {
    corners = count;
    corner = (k) => k;
  } else {
    const ix = accessor(gltf, bin, p.indices, 'SCALAR');
    if (ix.type === 5126) damaged('a primitive\'s indices are floats');
    corners = ix.count;
    corner = (k) => {
      const v = ix.get(k);
      if (v >= count) damaged(`a triangle uses vertex ${v} of ${count}`);
      return v;
    };
  }
  if (mode === TRIANGLES) {
    for (let k = 0; k + 2 < corners; k += 3) indices.push(base + corner(k), base + corner(k + 1), base + corner(k + 2));
  } else if (mode === STRIP) {
    // Every other triangle of a strip runs the other way round; with two sides, that changes nothing.
    for (let k = 0; k + 2 < corners; k++) indices.push(base + corner(k), base + corner(k + 1), base + corner(k + 2));
  } else {
    for (let k = 1; k + 1 < corners; k++) indices.push(base + corner(0), base + corner(k), base + corner(k + 1));
  }
}

/** Accessor `index`, checked against the bytes: { type, count, get(k) } for its k-th number. */
function accessor(gltf, bin, index, shape) {
  const a = (gltf.accessors || [])[index];
  if (!a) damaged(`no accessor ${show(index)}`);
  if (a.sparse) needs('sparse accessors');
  if (a.type !== shape) damaged(`accessor ${index} is ${show(a.type)}, not ${shape}`);
  const per = shape === 'VEC3' ? 3 : 1, kind = COMPONENTS[a.componentType];
  if (!kind) damaged(`accessor ${index} has component type ${show(a.componentType)}`);
  const [size, read] = kind;
  const count = a.count;
  if (!Number.isInteger(count) || count < 0) damaged(`accessor ${index} has count ${show(count)}`);
  if (a.bufferView === undefined) return { type: a.componentType, count, get: () => 0 };   // all zeros, as the format says
  const bv = (gltf.bufferViews || [])[a.bufferView];
  if (!bv) damaged(`no buffer view ${show(a.bufferView)}`);
  if ((bv.buffer || 0) !== 0 || bin === null) damaged('its numbers aren\'t in the file\'s binary chunk');
  const stride = bv.byteStride || per * size, start = (bv.byteOffset || 0) + (a.byteOffset || 0);
  if (count > 0 && (start + stride * (count - 1) + per * size > (bv.byteOffset || 0) + bv.byteLength
    || (bv.byteOffset || 0) + bv.byteLength > bin.byteLength)) damaged(`accessor ${index} runs past its bytes`);
  return { type: a.componentType, count, get: (k) => read(bin, start + stride * Math.floor(k / per) + (k % per) * size) };
}
