## What this changes

<!-- What it does and why. For a fix, how to see the bug before it. -->

## Checklist

- [ ] Against `develop`, not `main`
- [ ] A test that fails without this change
- [ ] `npm test` passes
- [ ] `docs/API.md` and `src/bearing.d.ts` updated for anything a user can see
- [ ] A line in `CHANGELOG.md` under `[Unreleased]`
- [ ] For a speed-up: before and after, measured in one run, and results unchanged
- [ ] Nothing in `src/` calls `Math.sin`, `Math.pow` or the like, draws, or allocates in `step()`
- [ ] No new dependencies, no build step
