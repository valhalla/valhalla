# Unidirectional A* Alternates Spike — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `UnidirectionalAStar::GetBestPath` return up to `1 + options.alternates()` paths so time-dependent queries (`date_time` set) can show alternates, not just the single optimal.

**Architecture:** Plateau-style extension of the forward A* loop. When the first destination edge is popped, record it as the optimal and keep expanding under a bounded extra-iteration budget, collecting additional distinct destination hits. Form paths for each candidate, filter with the existing `validate_alternate_by_stretch` / `validate_alternate_by_sharing` helpers (PathInfo-based, algorithm-agnostic), return the optimal plus accepted alternates capped at the requested count. If plateau produces only trivial (near-identical) alternates — which is plausible because unidirectional A* marks popped edges kPermanent and can't re-relax them — fall back to a penalty-rerun strategy scoped inside the same method.

**Tech Stack:** C++17, Valhalla fork (`spike/unidirectional-astar-alternates` branch), Docker dev shell via `scripts/valhalla/dev.sh`, existing Norway tiles under `valhalla-data/`. Run on port 9002 (`VALHALLA_PORT=9002`) to leave 8002 free for the production server.

**Scope constraints:**
- Changes contained to `src/thor/unidirectional_astar.cc`, `valhalla/thor/unidirectional_astar.h`, and (minor) `src/thor/alternates.cc`/`valhalla/thor/alternates.h` if we need to expose something.
- Do not touch `bidirectional_astar.*`, `route_action.cc`, or costing code.
- `route_action.cc` already reserves `options.alternates() + 1` trip slots and iterates over the returned vector, so once `GetBestPath` returns multiple paths, downstream serialization works.
- This is a spike. Accept that it may be slow or produce degenerate alternates. Measure, report, iterate if time permits.

---

## File Structure

| File | Responsibility | New / Modified |
|---|---|---|
| `valhalla/thor/unidirectional_astar.h` | Add `desired_paths_count_` member; declare new `FormPaths` helper (plural) | Modified |
| `src/thor/unidirectional_astar.cc` | Collect multiple dest hits, continue past first, form+filter paths | Modified |
| `docs/superpowers/plans/2026-04-19-unidirectional-astar-alternates.md` | This plan | New |
| `docs/superpowers/specs/2026-04-19-unidirectional-astar-alternates-report.md` | Spike findings (measurement, what worked, what didn't) | New (written at end) |

---

## Task 1: Baseline — verify dev.sh builds cleanly and current behavior is 0 alternates

**Files:**
- No code changes in this task.

- [ ] **Step 1: Ensure the fork's submodules are present**

Run from repo root (sandbox-disable if hit by filesystem permissions):

```bash
cd /Users/anders/workspace/private/valhalla && git submodule update --init --recursive
```

Expected: either "Submodule path ... registered for path ..." lines, or no output if already initialized.

- [ ] **Step 2: Check that port 9002 is free; stop any conflicting containers**

```bash
docker ps --filter "publish=9002" --format '{{.Names}}'
```

Expected: empty output. If something's bound, stop it: `docker stop <name>`.

- [ ] **Step 3: Bring up the server on 9002 against existing Norway tiles**

```bash
cd /Users/anders/.warp/worktrees/joyride/dune-ember && VALHALLA_PORT=9002 scripts/valhalla/up.sh
```

Expected: container starts, `scripts/valhalla/logs.sh` shows "Running tile service..." within ~10s.

- [ ] **Step 4: Run the Oslo→Bergen baseline query, confirm 0 alternates**

```bash
curl -sS -X POST http://localhost:9002/route \
  -H 'Content-Type: application/json' \
  -d '{"locations":[{"lat":59.9139,"lon":10.7522},{"lat":60.3913,"lon":5.3221}],"costing":"motorcycle","alternates":2,"date_time":{"type":0}}' \
  | jq '{alt_count: (.alternates | length // 0), main_len_km: .trip.summary.length}'
```

Expected: `alt_count: 0` (current behavior — date_time routes unidirectional, which doesn't yield alternates). `main_len_km` around 460–500.

- [ ] **Step 5: Commit the plan before touching code**

```bash
cd /Users/anders/workspace/private/valhalla
git add docs/superpowers/plans/2026-04-19-unidirectional-astar-alternates.md
git commit -m "docs(spike): plan for unidirectional A* alternates"
```

---

## Task 2: Thread `desired_paths_count_` through the algorithm

**Files:**
- Modify: `valhalla/thor/unidirectional_astar.h` (add member + change `GetBestPath` to use `options`)
- Modify: `src/thor/unidirectional_astar.cc:476-482` (read `options.alternates()` into the member)

- [ ] **Step 1: Add the member declaration**

In `valhalla/thor/unidirectional_astar.h`, after line 172 (where `edgelabels_` is declared) and before line 174 (`edgestatus_`), add:

```cpp
// Number of paths to return (1 + alternates requested).
uint32_t desired_paths_count_ = 1;

// Indexes into edgelabels_ for each destination-edge pop that we accept as
// a candidate ending point. First entry is the optimal path's terminal label.
std::vector<uint32_t> candidate_dest_labels_;
```

- [ ] **Step 2: Rename the `options` parameter in `GetBestPath`**

In `src/thor/unidirectional_astar.cc`, change the function signature around line 476-482 from:

```cpp
std::vector<std::vector<PathInfo>> UnidirectionalAStar<expansion_direction, FORWARD>::GetBestPath(
    valhalla::Location& origin,
    valhalla::Location& destination,
    GraphReader& graphreader,
    const sif::mode_costing_t& mode_costing,
    const travel_mode_t mode,
    const Options& /*options*/) {
```

to:

```cpp
std::vector<std::vector<PathInfo>> UnidirectionalAStar<expansion_direction, FORWARD>::GetBestPath(
    valhalla::Location& origin,
    valhalla::Location& destination,
    GraphReader& graphreader,
    const sif::mode_costing_t& mode_costing,
    const travel_mode_t mode,
    const Options& options) {
```

- [ ] **Step 3: Read `options.alternates()` into the member after the mode/costing setup**

Right after the existing `access_mode_ = costing_->access_mode();` line (around line 487), add:

```cpp
  desired_paths_count_ = 1;
  if (options.has_alternates_case() && options.alternates())
    desired_paths_count_ += options.alternates();
  candidate_dest_labels_.clear();
```

- [ ] **Step 4: Also clear the new member in `Clear()`**

In `src/thor/unidirectional_astar.cc:36-51`, inside `Clear()` after `edgestatus_.clear();`, add:

```cpp
  candidate_dest_labels_.clear();
```

- [ ] **Step 5: Also update the explicit template instantiations if needed**

Check the template instantiation block around lines 615-628. The `const Options& /*options*/` parameter there is the explicit instantiation — change both `/*options*/` to `options` to keep signatures consistent. If both are already unnamed it still compiles, but matching the definition avoids confusion.

- [ ] **Step 6: Build incrementally via dev.sh**

```bash
cd /Users/anders/.warp/worktrees/joyride/dune-ember && scripts/valhalla/dev.sh
# inside the dev container:
cd build && make -j$(nproc) valhalla_service 2>&1 | tail -40
```

Expected: compiles. Just type-plumbing, no behavior change yet.

- [ ] **Step 7: Smoke-test still returns 0 alternates (we haven't changed behavior)**

Repeat the curl from Task 1 Step 4. Expected: same output as baseline.

- [ ] **Step 8: Commit**

```bash
git add valhalla/thor/unidirectional_astar.h src/thor/unidirectional_astar.cc
git commit -m "feat(thor): thread alternates option through UnidirectionalAStar"
```

---

## Task 3: Collect extra destination hits past the first

The two `return {FormPath(predindex)};` sites in `GetBestPath` (lines 558 and 589) need to be replaced with "record this candidate; continue unless we've collected enough".

**Files:**
- Modify: `src/thor/unidirectional_astar.cc` around lines 540-612 (main loop)

- [ ] **Step 1: Replace the first `return {FormPath(predindex)};` (line 558)**

Current block (lines 546-559):

```cpp
    if (pred.destination()) {
      if (expansion_callback_) {
        auto expansion_type =
            FORWARD ? Expansion_ExpansionType_forward : Expansion_ExpansionType_reverse;
        const auto prev_pred = pred.predecessor() == kInvalidLabel
                                   ? GraphId{}
                                   : edgelabels_[pred.predecessor()].edgeid();
        expansion_callback_(graphreader, pred.edgeid(), prev_pred, "unidirectional_astar",
                            Expansion_EdgeStatus_connected, pred.cost().secs, pred.path_distance(),
                            pred.cost().cost, expansion_type, kNoFlowMask,
                            TravelMode::TravelMode_INT_MAX_SENTINEL_DO_NOT_USE_);
      }
      return {FormPath(predindex)};
    }
```

Replace the `return {FormPath(predindex)};` line with:

```cpp
      candidate_dest_labels_.push_back(predindex);
      if (candidate_dest_labels_.size() >= desired_paths_count_) {
        return FormPaths(graphreader, origin, destination);
      }
      // Otherwise keep expanding — we want more candidates. Fall through so
      // this edge gets marked kPermanent and we continue the loop.
```

- [ ] **Step 2: Handle the convergence-fallback return (line 588-593)**

Current block:

```cpp
    } else if (nc++ > kMaxIterationsWithoutConvergence) {
      if (best_path.first >= 0) {
        return {FormPath(best_path.first)};
      } else {
        LOG_ERROR("No convergence to destination after = " + std::to_string(edgelabels_.size()));
        return {};
      }
    }
```

Replace with:

```cpp
    } else if (nc++ > kMaxIterationsWithoutConvergence) {
      if (!candidate_dest_labels_.empty()) {
        return FormPaths(graphreader, origin, destination);
      }
      if (best_path.first >= 0) {
        candidate_dest_labels_.push_back(static_cast<uint32_t>(best_path.first));
        return FormPaths(graphreader, origin, destination);
      }
      LOG_ERROR("No convergence to destination after = " + std::to_string(edgelabels_.size()));
      return {};
    }
```

- [ ] **Step 3: Add an extra budget so we don't run forever past the first destination**

Right below the `if (pred.destination())` block, after the fall-through, we want to bail early if we've done "enough" extra iterations without accumulating more candidates. Add at the top of the loop body (right after the `const uint32_t predindex = adjacencylist_.pop();` and the invalid-label check), a budget early-exit:

```cpp
    // If we already have the optimal and have spent a bounded extra budget
    // without collecting more distinct destination edges, bail with what we've got.
    if (!candidate_dest_labels_.empty() &&
        edgelabels_.size() > first_dest_label_count_ + kMaxAlternateIterations) {
      return FormPaths(graphreader, origin, destination);
    }
```

- [ ] **Step 4: Introduce the `first_dest_label_count_` tracking variable**

Right before the `while (true)` loop at line 526, add:

```cpp
  size_t first_dest_label_count_ = 0;  // edgelabels_.size() snapshot when first dest found
```

And inside the `if (pred.destination())` block, in Step 1 right before `candidate_dest_labels_.push_back(predindex);`, add:

```cpp
      if (candidate_dest_labels_.empty()) {
        first_dest_label_count_ = edgelabels_.size();
      }
```

- [ ] **Step 5: Define `kMaxAlternateIterations`**

At the top of the file (around line 15 where `kMaxIterationsWithoutConvergence` is defined), add:

```cpp
// Extra label-relaxation budget after the optimal destination is found, used
// to collect candidate alternate paths. Tuned small to keep latency bounded.
constexpr uint32_t kMaxAlternateIterations = 200000;
```

- [ ] **Step 6: Build and verify it still compiles**

```bash
# inside dev container
cd build && make -j$(nproc) valhalla_service 2>&1 | tail -20
```

- [ ] **Step 7: Commit (code still won't produce alternates without FormPaths — that's next)**

```bash
git add valhalla/thor/unidirectional_astar.h src/thor/unidirectional_astar.cc
git commit -m "feat(thor): collect candidate destination hits past the optimal"
```

---

## Task 4: Implement `FormPaths` — form, filter, and return the final vector

**Files:**
- Modify: `valhalla/thor/unidirectional_astar.h` (declare `FormPaths`)
- Modify: `src/thor/unidirectional_astar.cc` (implement `FormPaths` for both forward + reverse templates)

- [ ] **Step 1: Declare `FormPaths` in the header**

In `valhalla/thor/unidirectional_astar.h`, right after the existing `FormPath` declaration (around line 157), add:

```cpp
  /**
   * Form multiple paths from the recorded candidate destination labels.
   * The first entry in candidate_dest_labels_ is always the optimal path; any
   * additional entries are filtered through share+stretch validators against
   * the optimal before being included.
   */
  std::vector<std::vector<PathInfo>>
  FormPaths(baldr::GraphReader& graphreader,
            const valhalla::Location& origin,
            const valhalla::Location& destination);
```

- [ ] **Step 2: Implement `FormPaths` in the .cc**

Add the following `#include` at the top of `src/thor/unidirectional_astar.cc` (near other thor includes, after line 4):

```cpp
#include "thor/alternates.h"
```

Add the function definition right after the existing `FormPath` definitions (after line 473, before the `GetBestPath` template at 475):

```cpp
template <const ExpansionType expansion_direction, const bool FORWARD>
std::vector<std::vector<PathInfo>>
UnidirectionalAStar<expansion_direction, FORWARD>::FormPaths(
    GraphReader& /*graphreader*/,
    const valhalla::Location& origin,
    const valhalla::Location& destination) {
  std::vector<std::vector<PathInfo>> paths;
  if (candidate_dest_labels_.empty()) {
    return paths;
  }

  // First candidate is always the optimal path.
  paths.push_back(FormPath(candidate_dest_labels_.front()));
  if (candidate_dest_labels_.size() == 1 || desired_paths_count_ <= 1) {
    return paths;
  }

  // Validate subsequent candidates against the optimal via the shared helpers.
  std::vector<std::unordered_set<GraphId>> shared_edgeids;
  const float max_sharing = get_max_sharing(origin, destination);

  for (size_t i = 1; i < candidate_dest_labels_.size(); ++i) {
    if (paths.size() >= desired_paths_count_) {
      break;
    }
    auto candidate = FormPath(candidate_dest_labels_[i]);
    if (candidate.empty()) {
      continue;
    }
    if (validate_alternate_by_sharing(shared_edgeids, paths, candidate, max_sharing) &&
        validate_alternate_by_stretch(paths.front(), candidate) &&
        validate_alternate_by_local_optimality(candidate)) {
      paths.emplace_back(std::move(candidate));
    }
  }

  LOG_DEBUG("UnidirectionalAStar returning " + std::to_string(paths.size()) + " paths");
  return paths;
}

// Explicit template instantiations for both directions.
template std::vector<std::vector<PathInfo>>
UnidirectionalAStar<ExpansionType::forward>::FormPaths(GraphReader&,
                                                       const valhalla::Location&,
                                                       const valhalla::Location&);
template std::vector<std::vector<PathInfo>>
UnidirectionalAStar<ExpansionType::reverse>::FormPaths(GraphReader&,
                                                       const valhalla::Location&,
                                                       const valhalla::Location&);
```

- [ ] **Step 3: Build**

```bash
# inside dev container
cd build && make -j$(nproc) valhalla_service 2>&1 | tail -20
```

Expected: clean build. If `validate_alternate_by_*` aren't visible, verify the `#include "thor/alternates.h"` was added.

- [ ] **Step 4: Restart the server**

```bash
# exit dev container, then
cd /Users/anders/.warp/worktrees/joyride/dune-ember && scripts/valhalla/down.sh && VALHALLA_PORT=9002 scripts/valhalla/up.sh
```

Wait for "Running tile service..." in logs.

- [ ] **Step 5: Run the Oslo→Bergen query — expect ≥1 alternate**

```bash
curl -sS -X POST http://localhost:9002/route \
  -H 'Content-Type: application/json' \
  -d '{"locations":[{"lat":59.9139,"lon":10.7522},{"lat":60.3913,"lon":5.3221}],"costing":"motorcycle","alternates":2,"date_time":{"type":0}}' \
  | jq '{alt_count: (.alternates | length // 0), main_len_km: .trip.summary.length, alt_lens_km: [.alternates[]?.trip.summary.length]}'
```

Expected (success): `alt_count` is 1 or 2, `alt_lens_km` values are within ~30% of `main_len_km`. If `alt_count: 0`, see the fallback plan in Task 5.

- [ ] **Step 6: Verify single-path case is unchanged**

```bash
curl -sS -X POST http://localhost:9002/route \
  -H 'Content-Type: application/json' \
  -d '{"locations":[{"lat":59.9139,"lon":10.7522},{"lat":60.3913,"lon":5.3221}],"costing":"motorcycle","date_time":{"type":0}}' \
  | jq '{alt_count: (.alternates | length // 0), main_len_km: .trip.summary.length}'
```

Expected: `alt_count: 0`, `main_len_km` same as before (regression check).

- [ ] **Step 7: Commit**

```bash
git add valhalla/thor/unidirectional_astar.h src/thor/unidirectional_astar.cc
git commit -m "feat(thor): return multiple paths from UnidirectionalAStar via plateau"
```

---

## Task 5: Contingency — if plateau yields 0 or only trivial alternates

Plateau relies on having multiple destination edges (from snap-radius) and on A* continuing to pop non-permanent labels. If after Task 4 we see 0 or only micro-deviation alternates (e.g. last-km variations), fall back to a bounded penalty-rerun:

**Approach:**
1. After the first path, save its edge ids.
2. `Clear()` the algorithm state.
3. Re-run the main loop, but inside `Expand` / `ExpandInner`, multiply the cost of any edge whose id is in the penalized set by a factor (try 3.0x).
4. Validate the second path against the first with `validate_alternate_by_sharing` + `validate_alternate_by_stretch`.

**Files:**
- Modify: `valhalla/thor/unidirectional_astar.h` (add `std::unordered_set<GraphId> penalized_edges_;`)
- Modify: `src/thor/unidirectional_astar.cc` `ExpandInner` near where it computes the new cost — multiply by 3.0 if the edge id is penalized.

- [ ] **Step 1: Only execute this task if Task 4 Step 5 produced <1 alternate or alternates with share > 95%**

Measure share:

```bash
# Pseudo — compute via jq on edge-list or manoeuvres lists on each path.
# For now, eyeball: look at each alternate's maneuver count/summary. If they
# only differ in the last few maneuvers, it's trivial — proceed with Task 5.
```

- [ ] **Step 2: Add penalized-edge set + penalty factor constant**

In `valhalla/thor/unidirectional_astar.h` near the existing `desired_paths_count_`:

```cpp
// Edge ids whose transition cost is multiplied by kAlternatePenaltyFactor on
// re-runs to steer A* away from the optimal path.
std::unordered_set<baldr::GraphId> penalized_edges_;
```

In `src/thor/unidirectional_astar.cc` near `kMaxAlternateIterations`:

```cpp
constexpr double kAlternatePenaltyFactor = 3.0;
```

- [ ] **Step 3: Apply penalty inside the expand loop**

Find `ExpandInner` (a private helper declared in the header around line 107). It computes a `newcost` for each neighbor edge. Right after the `newcost` is computed but before it's used for sortcost/heuristic lookups, add:

```cpp
  // Penalize optimal-path edges on re-runs so alternates can diverge.
  if (!penalized_edges_.empty() && penalized_edges_.count(meta.edge_id)) {
    newcost.cost *= kAlternatePenaltyFactor;
  }
```

(Exact line will be inside `ExpandInner`; grep for `newcost` around that function.)

- [ ] **Step 4: Drive the rerun from `GetBestPath`**

Restructure `GetBestPath` so that after the first `FormPaths` return of size 1, if we still want alternates, we:
1. Populate `penalized_edges_` from that path's edge ids.
2. `Clear()` (preserving `desired_paths_count_` and `penalized_edges_`).
3. Re-run `Init` + the main loop, but this time short-circuit at the first destination hit (we only want ONE more path).
4. Append the rerun's path to the return vector if it passes the validators.

Because `Clear()` currently wipes everything including `candidate_dest_labels_`, we need to preserve `penalized_edges_` across the clear. Easiest: use a local `std::unordered_set<GraphId> local_penalty;` in `GetBestPath`, wire it through a new private member that `Clear()` doesn't touch. Add to `Clear()`:

```cpp
  // NOTE: intentionally do NOT clear penalized_edges_ — it carries across reruns.
```

- [ ] **Step 5: Build, retest, commit if the alternates are now non-trivial**

```bash
cd build && make -j$(nproc) valhalla_service 2>&1 | tail -20
# restart server, re-run the Oslo→Bergen curl.
git add valhalla/thor/unidirectional_astar.{h,cc}
git commit -m "feat(thor): penalty-rerun fallback for unidirectional alternates"
```

---

## Task 6: Regression — unit tests still pass

**Files:**
- No code changes, just tests.

- [ ] **Step 1: Build the thor tests**

```bash
# inside dev container
cd build && make -j$(nproc) 2>&1 | tail -5
```

- [ ] **Step 2: Run the alternates + astar unit tests**

```bash
cd build && ctest -R "(alternates|astar)" --output-on-failure 2>&1 | tail -30
```

Expected: all pass. If a test that exercises `UnidirectionalAStar::GetBestPath` fails (e.g. because we changed the early-exit behavior), investigate whether the assertion is structural (always expected `paths.size() == 1`) or semantic.

- [ ] **Step 3: If tests pass, commit any test-accommodating edits**

```bash
git status
# if clean, nothing to commit. Otherwise:
git add test/...
git commit -m "test(thor): adjust astar assertions for multi-path return"
```

---

## Task 7: Write the spike report

**Files:**
- Create: `docs/superpowers/specs/2026-04-19-unidirectional-astar-alternates-report.md`

- [ ] **Step 1: Write findings**

Document:
1. Which approach landed (plateau, penalty-rerun, or a mix).
2. Oslo→Bergen numbers: optimal km, alternate km(s), share %, added latency.
3. What didn't work and why.
4. Recommendation: ship / discard / iterate.

- [ ] **Step 2: Commit**

```bash
git add docs/superpowers/specs/2026-04-19-unidirectional-astar-alternates-report.md
git commit -m "docs(spike): unidirectional A* alternates — findings"
```

- [ ] **Step 3: Open a draft PR against the fork**

```bash
cd /Users/anders/workspace/private/valhalla
git push -u origin spike/unidirectional-astar-alternates
gh pr create --draft --base master --title "Spike: alternates from unidirectional A*" --body "$(cat <<'EOF'
Spike — plateau-style alternate-path extraction from unidirectional A*
(used by time-dependent queries). See
docs/superpowers/specs/2026-04-19-unidirectional-astar-alternates-report.md
for the report.

Not for merge as-is. Ship-or-discard decision pending.
EOF
)"
```

---

## Verification summary

| Check | Command | Expected |
|---|---|---|
| Builds clean | `make -j$(nproc) valhalla_service` | no errors |
| Baseline (no alts) unchanged | Task 1 Step 4 curl | `alt_count: 0`, length unchanged |
| Alternates requested produce ≥1 | Task 4 Step 5 curl | `alt_count: 1` or `2` |
| Alternates are sensibly distinct | eyeball summaries | different major roads, not just last-km wiggles |
| Thor tests green | `ctest -R "(alternates\|astar)"` | all pass |

## What could go wrong

- **All alternates are micro-deviations near dest:** expected if plateau runs out of distinct macro routes. Task 5 (penalty rerun) is the escape hatch.
- **Latency explodes:** `kMaxAlternateIterations = 200000` is a guess. If Oslo→Bergen takes >5s, lower it to 50000 and measure again.
- **Reverse direction (arrive-by) behaves differently:** the reverse template has its own `FormPath`; any logic we add that assumes forward may break reverse. Test `date_time.type=2` (arrive-by) after Task 4 as a smoke check.
- **Memory:** collecting many candidate dest labels + forming paths costs O(alternates × path_length) extra. Oslo→Bergen has ~thousands of path edges; negligible vs the edgelabel vector.
