# Unidirectional A* Alternates — Spike Report

**Branch:** `spike/unidirectional-astar-alternates`
**Date:** 2026-04-19
**Status:** Done-criteria met. Not production-ready; see follow-ups.

## Goal

Bidirectional A* returns alternates when `options.alternates()` is set.
Unidirectional A* (used for time-dependent routes when `date_time` is set)
returned exactly one path. Goal: make unidirectional return up to
`1 + options.alternates()` paths, so Joyride's motorcycle routes with
`date_time.type=0` can show alternates.

## What landed

Two mechanisms composed in `src/thor/unidirectional_astar.cc`:

### 1. Plateau (cheap, first)
After the first destination-edge pop, don't early-return. Record the
predecessor-label index, keep expanding, and collect additional destination
hits up to `desired_paths_count_` or a bounded extra-iteration budget
(`kMaxAlternateIterations = 200000`). `FormPaths` then filters the
candidates through the existing `validate_alternate_by_{sharing,stretch,
local_optimality}` helpers.

### 2. Penalty rerun (does most of the work)
Back in `GetBestPath`, if the first search returned fewer paths than asked
for, seed `penalized_edges_` with the accepted paths' edge ids, `Clear()`
state, and re-run. Inside `ExpandInner`, edges in `penalized_edges_` have
their `edge_cost.cost` multiplied by `kAlternatePenaltyFactor = 3.0` so A*
is steered away. Up to `kMaxAlternateReruns = 4` reruns.

The existing validators (algorithm-agnostic, PathInfo-based) gate
acceptance. No new validator code written.

### Minimal surface
- `src/thor/unidirectional_astar.cc` (+200 lines)
- `valhalla/thor/unidirectional_astar.h` (+34 lines, 2 new members and 1
  new method)

Nothing touched outside `thor/`. `route_action.cc` already reserves
`options.alternates() + 1` trip slots and iterates the returned vector, so
downstream serialization Just Worked.

## Numbers — Oslo → Bergen, motorcycle, `date_time.type=0`

| Path    | Length (km) | Duration (min) |
|---------|-------------|----------------|
| main    | 477.1       | 385.4          |
| alt[0]  | 480.7       | 433.5          |
| alt[1]  | 541.3       | 474.6          |

- `alt[0]`: almost-same distance, ~13% longer in time — different road
  mix (probably avoids the main E-road section).
- `alt[1]`: 13% longer in distance — clearly a different major-road
  routing (likely inland vs. coastal).

For context, bidirectional's alternates on the same OD pair (without
`date_time`, using `bidirectional_a*`): 462.5 / 477.1 / 476.9 km — main +
two alternates that are within 1 km of each other. The spike's diversity
is actually _better_ on this route because penalty rerun forces a larger
geographic split, while bidirectional relies on its natural multi-candidate
connection set.

## What plateau alone did

On Oslo → Bergen:
```
candidates=2 desired=3
returning=1 rejected{share=1, stretch=0, empty=0}
```
Plateau collected exactly one extra candidate beyond the optimal, and the
sharing validator rejected it — it was a micro-deviation through a
different snap-radius destination edge, >95% shared with the optimal. Not
useful. This matches the failure mode anticipated in the plan: without a
reverse tree, plateau can only find alternate destination snaps, not
alternate routes.

Penalty rerun is what produces the diverse alternates here. Plateau stays
in the code because:
- It's ~20 lines and nearly free when it doesn't fire.
- It could pay off on denser urban graphs where multiple genuinely
  different paths converge on separate destination edges.

## What didn't work / limitations

- **Plateau is nearly useless alone** for long inter-city routes on
  Norway's tile set. The snap radius keeps multiple destination edges
  close together, which the sharing filter correctly rejects.
- **`kAlternatePenaltyFactor = 3.0` is a guess.** Values 2.0–3.0 plausibly
  work; too high and the rerun returns the same route because both
  alternatives cost about the same after penalty. Not tuned.
- **Each rerun is a fresh full A* search.** Oslo → Bergen runs in
  ~150–250 ms optimal, so a 2-alternate request costs ~450–750 ms total.
  Acceptable for a backend API, borderline for an on-device use case.
- **Unit tests not run.** `ctest -R "(alternates|astar)"` wasn't exercised
  because full-image rebuilds were slow and the iteration loop was run in
  an ad-hoc `docker exec` dev container. CI should run them.
- **Reverse (`date_time.type=2`, arrive-by) not regression-tested.** The
  reverse template shares the refactored code path; should work, but
  wasn't exercised end-to-end.
- **Diagnostic `LOG_INFO` statements** (`UnidirectionalAStar::FormPaths:
  ...` and `penalty-rerun N candidate accepted/rejected`) left in. Useful
  for tuning; would strip to `LOG_DEBUG` before production.

## Follow-ups before merging to production

1. Strip diagnostic `LOG_INFO` → `LOG_DEBUG`, or keep INFO for the
   accept/reject line and demote the others.
2. Run the upstream thor + gurka alternates tests; fix any that assume
   unidirectional returns exactly one path.
3. Regression-test `date_time.type=2` (arrive-by) end-to-end.
4. Tune `kAlternatePenaltyFactor` and `kMaxAlternateReruns` against a
   battery of (urban / suburban / inter-city / long-range) queries.
5. Benchmark latency delta on the prod tile set — how much does
   `alternates=2` cost vs. `alternates=0` in p50/p95?
6. Decide whether `filter_alternates_by_local_optimality` (currently
   stubbed `return true` in `alternates.cc`) needs a real implementation
   before relying on it to gate alternates.

## Recommendation

**Iterate, then ship.** The mechanism works and produces sensibly distinct
alternates. But treating this as production-ready without the follow-ups
above would leave the thor test suite in an unknown state and bake in the
`3.0`/`4`/`200000` magic numbers un-tuned. Worth a week of tuning + test
work before enabling in the app.
