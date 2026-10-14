// The demo's models: a box, a sphere and a capsule, each built as a tiny .glb
// in memory, so the demo downloads nothing but Winding itself.
//
// A .glb is a 12-byte header, then a JSON chunk describing the meshes, then a
// binary chunk holding their numbers. Winding's engine.load takes the bytes
// directly. A unit box and a unit sphere are scaled to each body's size by its
// node; a capsule can't be (its round ends would stretch), so one is built for
// each radius and height the scene uses.

/** A .glb of one mesh, flat-shaded per vertex normal, in one colour; seen from both sides if `twoSided`. */
function glb(positions, normals, indices, color, twoSided = false) {
  const pos = new Float32Array(positions), nor = new Float32Array(normals), idx = new Uint32Array(indices);
  const min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < pos.length; i += 3) {
    for (let k = 0; k < 3; k++) { min[k] = Math.min(min[k], pos[i + k]); max[k] = Math.max(max[k], pos[i + k]); }
  }
  const json = {
    asset: { version: '2.0', generator: 'Bearing demo' },
    scene: 0, scenes: [{ nodes: [0] }], nodes: [{ mesh: 0 }],
    meshes: [{ primitives: [{ attributes: { POSITION: 0, NORMAL: 1 }, indices: 2, material: 0 }] }],
    materials: [{ pbrMetallicRoughness: { baseColorFactor: [...color, 1], metallicFactor: 0, roughnessFactor: 0.6 }, doubleSided: twoSided }],
    accessors: [
      { bufferView: 0, componentType: 5126, count: pos.length / 3, type: 'VEC3', min, max },
      { bufferView: 1, componentType: 5126, count: nor.length / 3, type: 'VEC3' },
      { bufferView: 2, componentType: 5125, count: idx.length, type: 'SCALAR' },
    ],
    bufferViews: [
      { buffer: 0, byteOffset: 0, byteLength: pos.byteLength, target: 34962 },
      { buffer: 0, byteOffset: pos.byteLength, byteLength: nor.byteLength, target: 34962 },
      { buffer: 0, byteOffset: pos.byteLength + nor.byteLength, byteLength: idx.byteLength, target: 34963 },
    ],
    buffers: [{ byteLength: pos.byteLength + nor.byteLength + idx.byteLength }],
  };
  let text = JSON.stringify(json);
  while (text.length % 4) text += ' ';   // chunks are 4-byte aligned
  const jsonBytes = new TextEncoder().encode(text);
  const binLength = json.buffers[0].byteLength;
  const total = 12 + 8 + jsonBytes.length + 8 + binLength;
  const out = new Uint8Array(total), view = new DataView(out.buffer);
  view.setUint32(0, 0x46546c67, true);   // 'glTF'
  view.setUint32(4, 2, true);
  view.setUint32(8, total, true);
  view.setUint32(12, jsonBytes.length, true);
  view.setUint32(16, 0x4e4f534a, true);  // 'JSON'
  out.set(jsonBytes, 20);
  const bin = 20 + jsonBytes.length;
  view.setUint32(bin, binLength, true);
  view.setUint32(bin + 4, 0x004e4942, true);   // 'BIN'
  out.set(new Uint8Array(pos.buffer), bin + 8);
  out.set(new Uint8Array(nor.buffer), bin + 8 + pos.byteLength);
  out.set(new Uint8Array(idx.buffer), bin + 8 + pos.byteLength + nor.byteLength);
  return out;
}

/** A 1 m cube, its faces each with their own normal. */
export function box(color) {
  const p = [], n = [], i = [];
  const faces = [[0, 1], [0, -1], [1, 1], [1, -1], [2, 1], [2, -1]];
  for (const [axis, s] of faces) {
    const u = (axis + 1) % 3, v = (axis + 2) % 3, base = p.length / 3;
    for (const [a, b] of [[-1, -1], [1, -1], [1, 1], [-1, 1]]) {
      const c = [0, 0, 0];
      c[axis] = 0.5 * s; c[u] = 0.5 * a * s; c[v] = 0.5 * b;
      p.push(...c);
      const d = [0, 0, 0];
      d[axis] = s;
      n.push(...d);
    }
    i.push(base, base + 1, base + 2, base, base + 2, base + 3);
  }
  return glb(p, n, i, color);
}

/** Rings of a round shape around y: a sphere when half is 0, a capsule otherwise. */
function round(radius, half, color) {
  const p = [], n = [], i = [];
  const rings = 24, around = 32;
  // Rings from the top pole to the bottom one; the top hemisphere sits up by
  // `half`, the bottom one down, and the cylinder between is the gap.
  const rows = [];
  for (let r = 0; r <= rings; r++) {
    const phi = (r / rings) * Math.PI, y = Math.cos(phi), s = Math.sin(phi);
    rows.push([y, s, r <= rings / 2 ? half : -half]);
    if (r === rings / 2 && half > 0) rows.push([y, s, -half]);
  }
  for (const [y, s, shift] of rows) {
    for (let a = 0; a <= around; a++) {
      const theta = (a / around) * 2 * Math.PI, x = Math.cos(theta) * s, z = Math.sin(theta) * s;
      p.push(x * radius, y * radius + shift, z * radius);
      n.push(x, y, z);
    }
  }
  for (let r = 0; r < rows.length - 1; r++) {
    for (let a = 0; a < around; a++) {
      const k = r * (around + 1) + a, l = k + around + 1;
      i.push(k, k + 1, l, k + 1, l + 1, l);
    }
  }
  return glb(p, n, i, color);
}

export const sphere = (color) => round(1, 0, color);
export const capsule = (radius, height, color) => round(radius, height / 2 - radius, color);

/**
 * Triangles, flat-shaded, each with its own three corners and its face's
 * normal, seen from both sides, as Bearing's triangles have two.
 */
export function triangles(vertices, indices, color) {
  const p = [], n = [], i = [];
  for (let t = 0; t < indices.length; t += 3) {
    const a = 3 * indices[t], b = 3 * indices[t + 1], c = 3 * indices[t + 2], v = vertices;
    const ux = v[b] - v[a], uy = v[b + 1] - v[a + 1], uz = v[b + 2] - v[a + 2];
    const wx = v[c] - v[a], wy = v[c + 1] - v[a + 1], wz = v[c + 2] - v[a + 2];
    const nx = uy * wz - uz * wy, ny = uz * wx - ux * wz, nz = ux * wy - uy * wx, l = Math.hypot(nx, ny, nz) || 1;
    for (const k of [a, b, c]) { p.push(v[k], v[k + 1], v[k + 2]); n.push(nx / l, ny / l, nz / l); }
    i.push(t, t + 1, t + 2);
  }
  return glb(p, n, i, color, true);
}

/** A heightfield's triangles, cut as Bearing cuts its cells: from each one's lowest x and z to its highest. */
export function heightfield(heights, columns, spacing, color) {
  const rows = heights.length / columns, x0 = -((columns - 1) * spacing) / 2, z0 = -((rows - 1) * spacing) / 2;
  const v = [], ix = [];
  for (let j = 0; j < rows; j++) for (let i = 0; i < columns; i++) v.push(x0 + i * spacing, heights[j * columns + i], z0 + j * spacing);
  for (let j = 0; j < rows - 1; j++) {
    for (let i = 0; i < columns - 1; i++) {
      const a = j * columns + i, b = a + 1, c = a + columns, d = c + 1;
      ix.push(a, d, b, a, c, d);
    }
  }
  return triangles(v, ix, color);
}

/**
 * A ramp, what a level's .glb might hold: 6 m long, rising 2 m from its far
 * end (x = 6) to its near one (x = 0), 3 m wide, with sides and a back.
 */
export function ramp(color) {
  const v = [0, 0, -1.5, 6, 0, -1.5, 6, 0, 1.5, 0, 0, 1.5, 0, 2, -1.5, 0, 2, 1.5];
  // The slope, the back (x = 0), the two sides and the bottom.
  const ix = [4, 5, 2, 4, 2, 1, 0, 3, 5, 0, 5, 4, 0, 4, 1, 3, 2, 5, 0, 1, 2, 0, 2, 3];
  return triangles(v, ix, color);
}
