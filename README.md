# Bearing

A physics engine in plain JavaScript, for any renderer. No dependencies, no build step — ES modules,
used as they are.

**In development, and not released.** Nothing is on npm yet. Bodies fall, tumble, collide, rest,
stack, slide, bounce and hang from joints; see the [roadmap](ROADMAP.md) for what comes next.

```js
import { World } from 'bearing-physics';

const world = new World();
world.addBody({ type: 'static', size: [20, 1, 20], position: [0, -0.5, 0] });
const ball = world.addBody({ shape: 'sphere', position: [0, 4, 0], restitution: 0.7 });

function update(dt) {   // your loop's fixed step, such as 1 / 60 s
  world.step(dt);
}
```

That is the whole setup: the ball falls, bounces lower each time, and rests on the ground. There is
no `init()`, no broadphase to choose, no mass to work out (it comes from the shape and water's
density unless you give one), no tolerances to tune, and nothing to size in advance.

## What it does

- **Bodies**: dynamic, static and kinematic; boxes, spheres and capsules, with mass and inertia
  from the shape. Lopsided bodies tumble as real ones do, keeping their angular momentum exactly.
- **Contacts**: exact contact points for every pair of shapes, friction that holds a box on a slope
  until it is steep enough to slide, bounces that settle, and stacks that stand. A 210-box pyramid
  stands for as long as you like on the defaults, within 1.5 mm of where it was built.
- **Thousands of bodies**: only bodies whose boxes overlap are tested. Bodies at rest go to sleep
  and cost nothing until something disturbs them, and `world.moved()` lists only the bodies a
  renderer has to draw again. 32 collision layers, with a mask per body for what it collides with.
- **Queries**: raycasts (nearest, or every hit), shape casts and overlap tests, exact for every
  shape and quick through the same tree the contacts use.
- **Character bodies**: a body the game gives a velocity to, that walks rather than flies: it slides
  along walls, walks up slopes and steps up stairs to a limit, stays on the ground going down them,
  falls and lands, and rides moving platforms. Rotation locks keep a body upright, or a 2.5D game in
  its plane. See [Character bodies](docs/API.md#characters).
- **Joints**: fixed, ball, hinge, slider and distance joints, with limits, motors and springs. Chains,
  ragdolls and bridges are solved exactly, all at once, so a 100-link chain hangs at its built
  length and a slammed door stops on its stop. See [Joints](docs/API.md#joints).
- **Contact events and sensors**: which bodies began and stopped touching in each step, with where
  and how hard they met, read after the step; what touches a body now; and sensors, zones that
  report what enters them and push nothing. See [Contact events and sensors](docs/API.md#events).
- **Level geometry**: triangle meshes and heightfields, with no seams to catch on, and meshes read
  straight from a `.glb` file with [`readGlb`](docs/API.md#readglb), with no dependency. See
  [Meshes and heightfields](docs/API.md#meshes).
- **Tolerances that scale**: every tolerance comes from the size of what touches, so a 1 cm marble
  and a 50 m building behave with nothing retuned. See [Contacts](docs/API.md#contacts).

**Every call, option and setting is in the [API reference](docs/API.md)**: what it does, what it
takes, what it returns and what it throws.

## Any renderer

Bearing knows nothing about drawing. After each step, every body's position and rotation is in one
`Float32Array`, [`world.transforms`](docs/API.md#world-transforms), 7 numbers a body, so any
engine copies them in one loop. [`world.previous`](docs/API.md#world-previous) holds the step
before, for drawing smooth motion between fixed steps, and [`world.moved()`](docs/API.md#world-moved)
says which bodies changed since a frame was last drawn, so only those are copied.

## The same everywhere

The same inputs give bit-identical results on every machine, browser and runtime. JavaScript
defines its arithmetic exactly, and Bearing never calls the few functions, such as `Math.sin`, that
each engine may round its own way. See [Determinism](docs/API.md#determinism).

## Demo

```bash
node serve.js
```

Then open <http://localhost:8080/demo/>: a 210-box pyramid, a stack, bouncing balls, boxes on
slopes either side of their friction, a moving platform, a mixed pile, 2,000 balls raining down,
bodies tumbling over a heightfield of hills and down a ramp read from a `.glb`, a character walking
a route up stairs and down a ramp, and two colours on separate layers, with what each step costs, and how many bodies are asleep,
shown live. Bearing draws nothing, so the demo is drawn by
[Winding](https://github.com/nolanbaxter/winding), loaded from jsDelivr at a pinned version; it
needs a browser with WebGPU. The demo is not part of the package, and no test needs it.

## Tests

```bash
npm test
```

Plain Node scripts, no framework, Node 20 or newer. They check every documented error word for
word, that the [API reference](docs/API.md) covers every call and that its examples run, that no
id can reach the wrong body, and that a world adding and removing 10,000 bodies 300 times leaks
nothing.

## License

MIT — see [LICENSE](LICENSE).
