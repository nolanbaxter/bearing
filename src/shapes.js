// What a shape weighs and how it turns: volume, and the moments of inertia
// about its own axes.
//
// Every shape here is symmetric about its own x, y and z axes, with its centre
// of mass at its centre, so its inertia tensor is diagonal in the body's own
// frame: three numbers, the principal moments, rather than a 3x3 matrix. A
// shape that breaks that symmetry (a compound, a hull) will need the full
// tensor and a centre-of-mass offset, and brings both with it.

/** m³. A mesh or heightfield is a surface, and encloses none. */
export function volume(spec) {
  if (spec.shape === 'mesh' || spec.shape === 'heightfield') return 0;
  const r = spec.radius;
  if (spec.shape === 'box') return spec.size[0] * spec.size[1] * spec.size[2];
  const ball = (4 / 3) * Math.PI * r * r * r;
  if (spec.shape === 'sphere') return ball;
  return Math.PI * r * r * (spec.height - 2 * r) + ball;
}

/**
 * The principal moments, kg·m², of a solid shape of mass `mass`, about its
 * own x, y and z axes through its centre. Into `out`.
 *
 * A capsule is a cylinder of length L = height - 2r along y, capped by two
 * hemispheres, its mass shared between them by volume. About y, the cylinder
 * gives m r²/2 and the two caps together a sphere's 2/5 m r². About x (and z,
 * the same by symmetry) the cylinder gives m (L²/12 + r²/4), and each cap is a
 * hemisphere whose own centre of mass sits 3r/8 from its flat face: moved by
 * the parallel-axis theorem out to L/2 + 3r/8 from the capsule's centre, the
 * two give m (2r²/5 + L²/4 + 3Lr/8). test/motion.test.js checks this by
 * slicing the capsule into discs and integrating, so it isn't the formula
 * checking itself.
 */
export function inertia(spec, mass, out) {
  if (spec.shape === 'box') {
    const [w, h, d] = spec.size;
    out[0] = mass * (h * h + d * d) / 12;
    out[1] = mass * (w * w + d * d) / 12;
    out[2] = mass * (w * w + h * h) / 12;
  } else if (spec.shape === 'sphere') {
    const r = spec.radius;
    out[0] = out[1] = out[2] = 0.4 * mass * r * r;
  } else if (spec.shape === 'mesh' || spec.shape === 'heightfield') {
    out[0] = out[1] = out[2] = 0;
  } else {
    const r = spec.radius, L = spec.height - 2 * r;
    const cylinder = Math.PI * r * r * L, ball = (4 / 3) * Math.PI * r * r * r;
    const mc = mass * cylinder / (cylinder + ball), ms = mass * ball / (cylinder + ball);
    out[1] = mc * r * r / 2 + ms * 0.4 * r * r;
    out[0] = out[2] = mc * (L * L / 12 + r * r / 4) + ms * (0.4 * r * r + L * L / 4 + 3 * L * r / 8);
  }
  return out;
}
