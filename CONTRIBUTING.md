# Contributing to Bearing

Thank you for helping. Bearing is a physics engine in plain JavaScript for any renderer, with no
dependencies and no build step, and every change keeps it that way: plain ES modules, published
as they are. [ROADMAP.md](ROADMAP.md) says what it is for, what 1.0 has to be, and what is being
built now.

## Branches

- **`develop`** is where work lands. Branch from it, and open pull requests against it.
- **`main`** is released code only. A release is a pull request from `develop` into `main` that
  moves the changelog's `[Unreleased]` section under a version and bumps `package.json`; merging
  it tags the release and publishes it to npm. Nothing else reaches `main`.

## Running the tests

```bash
npm test
```

Plain Node scripts, no framework, no install (there is nothing to install), on Node 20 or newer.
CI runs them on Node 20, 22 and 24, and checks the TypeScript declarations against
`test/types.ts`.

The demo, which Winding draws, is not a test, and no test needs it:

```bash
node serve.js
```

then open <http://localhost:8080/demo/> in a browser with WebGPU.

## What a change includes

- **A test that fails without it.** A bug fix starts from a script that shows the bug; the fix
  comes with a test that fails on the old code. Physics is tested for what it must get right
  (momentum, energy, stacks standing, bounces settling), not only that the code runs.
- **The API reference.** `docs/API.md` changes in the same change as anything a user can see: a
  new call, a changed option or default, a new error message, behaviour a fix changes. A new entry
  gets an `<a id>` anchor, a line in the index, an example, and every message it throws, word for
  word. `npm test` checks that every public method has an entry, every link lands, every example
  runs, and every error message is listed and thrown.
- **The types**, in `src/bearing.d.ts`, written by hand.
- **The changelog.** A line under `## [Unreleased]` in `CHANGELOG.md`, in Added, Changed or Fixed.
- **The README**, when a feature or a limitation changes.

## Versions

Bearing follows [semantic versioning](https://semver.org) against `docs/API.md`:

- **patch** (`x.y.Z`): reworks of existing code that change no API: fixes, speed, docs;
- **minor** (`x.Y.0`): additions, and renames that keep the old name working for a version;
- **major** (`X.0.0`): anything that breaks code written against the API reference.

Before 1.0, breaking changes and additions both go in a minor version (`0.Y.0`), and patches stay
patches.

## The rules

`CLAUDE.md` holds the rules Bearing is built to, written for the AI sessions that build it, and
they are the rules for everyone. The ones that come up most:

- **Nothing arbitrary.** Every number in the code is derived (from the physics, the step, a
  format) or measured (with the measurement named beside it). Tolerances scale with the size of
  what is simulated.
- **Out of the way.** No `init()`, no bookkeeping calls, no setting a user must tune to get a
  correct result.
- **Deterministic on every machine.** Nothing in `src/` calls `Math.sin`, `Math.cos`, `Math.pow`
  or any other function the spec lets engines round their own way; `test/math.test.js` checks.
- **No rendering in `src/`.** Bearing hands back numbers; engines draw them.
- **No allocation in `step()`**, and nothing that grows forever (`test/soak.test.js`).

## Performance

A change made for speed comes with a measurement: the old code and the new, timed in one run in
alternating blocks, since separate runs drift by more than most changes, and a test that the
simulation's results didn't change. A change that measures no better is left out, however
sensible it looked. `bench/` holds the measurements that decided the engine's design so far.

## Style

Match the code around you. Comments say why, in full sentences: the reason for a decision, the
alternative rejected, what would break. Names are short and real. No new dependencies.

## Reporting

- **Bugs:** open an issue with the bug form. The smallest code that shows it makes most reports
  answerable at once.
- **Security issues:** privately, as [SECURITY.md](SECURITY.md) says, not in an issue.

By taking part you agree to the [code of conduct](CODE_OF_CONDUCT.md).
