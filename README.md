# Bearing

A physics engine in plain JavaScript, for any renderer. No dependencies, no build step — ES modules,
used as they are.

**In development, and not released.** Nothing is on npm yet. Bodies move, fall and turn, but don't
collide yet: collision is the next piece being built.

```js
import { World } from 'bearing-physics';

const world = new World();
world.addBody({ type: 'static', size: [20, 1, 20], position: [0, -0.5, 0] });
const ball = world.addBody({ shape: 'sphere', radius: 0.5, position: [0, 4, 0] });

function update(dt) {   // your loop's fixed step, such as 1 / 60 s
  world.step(dt);
}
```

That is the whole setup. There is no `init()`, no broadphase to choose, no mass to work out (it
comes from the shape and water's density unless you give one), and nothing to size in advance.

**Every call, option and setting is in the [API reference](docs/API.md)**: what it does, what it
takes, what it returns and what it throws.

## Any renderer

Bearing knows nothing about drawing. After each step, every body's position and rotation is in one
`Float32Array`, [`world.transforms`](docs/API.md#world-transforms), 7 numbers a body, so any
engine copies them in one loop. [`world.previous`](docs/API.md#world-previous) holds the step
before, for drawing smooth motion between fixed steps.

## The same everywhere

The same inputs give bit-identical results on every machine, browser and runtime. JavaScript
defines its arithmetic exactly, and Bearing never calls the few functions, such as `Math.sin`, that
each engine may round its own way. See [Determinism](docs/API.md#determinism).

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
