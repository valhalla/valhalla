# Learnings

Hand-curated index of design lessons from Valhalla's GitHub issues, pull requests, and discussions. Coding agents read this before searching the full project history so maintainers and reviewers do not have to repeat settled context.

This is not a changelog. `CHANGELOG.md` is the human-facing record of what changed. This index is agent-facing prior art: only disproven hypotheses, reversed decisions, won't-fixes with rationale, non-obvious design rules, and audit summaries earn entries.

State labels are snapshots; verify the current GitHub state before acting on entries marked open, not-planned, or won't-fix.

## Entry shape

```markdown
- **#N** (state) — one-line load-bearing takeaway. Parent: #X. Winner: #Y. Superseded by: #Z. See: <pointer>. *[tags: subsystem, subsystem]*
```

- `#N` and `(state)` are required. The takeaway is one sentence, present-tense, focused on the **rule that survived**, not the diff.
- `state` ∈ {`merged`, `not-planned`, `superseded`, `won't-fix`, `partial`, `open, approach declined`} for issue entries, **or** `(PR closed)` / `(PR closed, not merged)` when the load-bearing artifact is the closed PR itself. Compound with a structural modifier when relevant, such as `(merged, umbrella)`.
- For a lesson added in its source pull request, use the terminal state it will have if merged, normally `(merged)`. The entry becomes canonical only on merge and must be removed if the pull request closes unmerged.
- Discussions use `**Discussion #N** (answered)` or `(closed)` and always include their URL in `See:` because Discussions have a separate number namespace from issues and pull requests.
- Optional tails: `Parent:` (umbrella), `Winner:` (which sibling shipped), `Superseded by:` (what replaced it), `See:` (deep-dive pointer — a file path, documentation anchor, discussion, or comment URL), `*[tags: ...]*` (subsystem tags for cross-cutting searches).
- Umbrella issues open with a leading bullet stating the decision rule, followed by nested bullets for each child. The umbrella entry is the load-bearing one; children are searchable pointers.

## Admission rule

An entry earns its place if its takeaway would change a decision in a future coding-agent investigation. Prefer entries that capture:

- A **disproven hypothesis**: measurement showed the proposed explanation or approach was wrong.
- A **reversed decision**: the project shipped one approach, learned from it, and a different approach survived.
- A **won't-fix with rationale**: future contributors need the reason before proposing the same shape.
- A **design rule**: an invariant that is not obvious from the current code but project history establishes.
- An **audit summary**: a broad review produced one or two transferable rules worth carrying forward.

Routine bug fixes, dependency bumps, release notes, and shipped features without a generalizable lesson belong in `CHANGELOG.md`, not here.

---

## Baldr & graph tiles

- **#3973** (not-planned) — `GraphId`'s apparently unused high bits are consumed when a `DirectedEdge` embeds its 46-bit end-node ID, so expanding the tile-id field requires a tile-format break rather than reclaiming spare bits. *[tags: baldr, graphid, tile-format]*
- **#5450** (merged) — OSM node IDs belong in optional delta-encoded varints in `EdgeInfo` tagged data rather than fixed graph-core fields; the measured tileset grew about 12% when storing every routable node ID and 6% for graph-node IDs only, so builders choose the storage policy. *[tags: baldr, edgeinfo, osm, tile-size]*
- **#6126** (merged) — `dataset_id` identifies the source dataset and remains idempotent for the same input; deployments that need every code/config rebuild to have a distinct global ID set `mjolnir.dataset_id` explicitly, while tile checksums describe content. *[tags: baldr, tile-header, deployment]*

## Mjolnir & OSM ingestion

- **#2976** (merged) — Link reclassification remains necessary for turn-channel guidance and long-route performance; fix bad classifications surgically from available OSM link tags instead of removing the mechanism globally. Winner: #3042. *[tags: mjolnir, reclassification, performance]*
- **#5269** (merged) — Keep link and ramp reclassification, but model ferry road class separately from hierarchy placement so ferries can move between hierarchy levels without inheriting the connected road's classification. *[tags: mjolnir, ferries, hierarchy]*
- **#2934** (merged) — Preserve private roads in the graph and enforce access during costing/correlation; removing them during tile construction in #1960 caused widespread no-route failures and was reverted. *[tags: mjolnir, access, graph-filter]*

## Sif & Thor routing

- **#3203** (open, approach declined) — Per-alternative local-optimality shortest-path checks cost about 20%, reject good alternatives because hierarchy limits hide lower-level improvements, and improve only a small minority of sampled routes; limited sharing remains the practical filter until a hierarchy-aware method exists. *[tags: thor, alternatives, performance]*
- **#3556** (won't-fix) — Large bidirectional matrices inherently allocate per-source and per-target search state, so memory scales with request size and concurrent worker count; multi-gigabyte requests are an algorithm capacity limit rather than evidence of a retained-cache leak. *[tags: thor, matrix, memory]*
- **Discussion #4621** (answered) — Centroid expansion assumes one comparable cost unit and collapsible search state; supporting mixed travel modes requires separate queues/state per costing or a shared metric such as ETA, each with quality and efficiency tradeoffs. See: https://github.com/valhalla/valhalla/discussions/4621. *[tags: thor, centroid, costing]*
- **Discussion #5779** (answered) — The tiled hierarchy accelerates goal-directed A-to-B routing, while whole-graph searches such as isochrones must jump among tiles and lose CPU-cache locality because there is no goal heuristic to constrain expansion. See: https://github.com/valhalla/valhalla/discussions/5779. *[tags: thor, isochrone, hierarchy, performance]*

## Loki, Meili & Skadi

- **#4279** (merged) — Treat EdgeInfo elevation, `/height`, and routing grade as separate data products: route/trace elevation uses 30 m encoded samples with bridge/tunnel interpolation, `/height` samples terrain, and costing reads `DirectedEdge` weighted grade. *[tags: skadi, elevation, edgeinfo, costing]*

## Odin, Tyr & bindings

## Traffic, transit & time

## Builds, performance & operations

- **#2843** (not-planned) — Arbitrary per-request graph additions require rebuilding and reloading affected tiles; known variants can be prebuilt and disabled with avoid locations, while interactive local scenarios can rebuild a tiny extract quickly. *[tags: tiles, dynamic-graph, deployment]*
- **#1705** (merged) — Tile construction remains one ordered pipeline with configurable begin/end stages, allowing restarts from durable intermediate `.bin` files without splitting stage ownership across executables. *[tags: mjolnir, build, pipeline]*
- **#3938** (won't-fix) — Tile-directory and tile-extract hot swaps are cache-unsafe because worker threads cannot detect invalidated graph caches; a safe design needs generation consistency plus cache warming or request quiescence during the swap. *[tags: tiles, cache, deployment]*
- **Discussion #3323** (answered) — Remote lazy loading uses an empty `tile_dir` plus `tile_url`, not a tar extract; requested tiles persist on disk, cold requests pay network latency, and cached tiles from different generations cannot be mixed. See: https://github.com/valhalla/valhalla/discussions/3323. *[tags: tiles, cache, deployment]*

## Testing, process & documentation

- **#6076** (merged) — Agent guidance content lives once: root runtime guide aliases point to `CLAUDE.md`, runtime-specific skill discovery aliases point to `.agents/skills`, and every existing discovery path is symlinked to prevent instructions from drifting. *[tags: agents, documentation, process]*
