# Removing `graph.lua`: migration plan

Scope: **`lua/graph.lua` only** (the way/node/relation tag transform used by `pbfgraphparser.cc`). `admin.lua` and tests are explicitly out of scope here. Goal: make OSM tag interpretation **testable in C++** and **modular enough to re-run per-entity** (future `.osc` changeset → single-tile updates), while **preserving existing tag interpretation byte-for-byte**.

---

## 1. TL;DR / recommendation

- The "Lua tag transform" is **not** a clean transform. It's a *mutate-the-tag-map-in-place + pass-through* step, and **a large fraction of the actual parsing already lives in C++** (`tag_handlers_` dispatch + an `else-if` chain in `pbfgraphparser.cc`). Lua and C++ are tightly interleaved across a **stringly-typed `map<string,string>` boundary**.
- The single most important enabler is a **differential-testing harness**: run Lua and a native C++ transform side-by-side over a large real-OSM corpus and assert identical output. Build this *first*. Everything else stages safely behind it.
- The migration is naturally stageable: introduce a `TagTransform` interface → port logic in feature slices (nodes → relations → ways sub-features), each validated against Lua behind a config flag → finally delete the stringly-typed round-trip and `graph.lua`.
- **Biggest non-obvious risk:** users can supply their own `graph_lua_name` script. Removing Lua removes a public extension point. Decide the replacement story before deleting (see §4).

---

## 2. How it actually works today (read this before planning anything)

The mental model "Lua parses tags, C++ consumes the result" is wrong. The real flow:

```
osmium::TagList  ──►  LuaTagTransform::Transform(type, osmid, tags)
                        │  pushes raw tags into a Lua table `kv`
                        │  calls ways_proc / nodes_proc / rels_proc
                        │  Lua MUTATES kv IN PLACE: adds normalized keys
                        │  (auto_forward, road_class, use, access_mask, …),
                        │  overwrites some, deletes a few (FIXME/note/source),
                        │  and LEAVES EVERY UNTOUCHED RAW TAG IN kv
                        ▼
                 Tags = map<string,string>   ← normalized keys + raw passthrough, flattened together
                        │
                        ▼
   pbfgraphparser.cc:  for (kv : tags) { tag_handlers_.find(kv.first)?() ; else-if chain }
                        │  consumes BOTH lua-produced keys AND raw OSM keys
                        │  re-parses many values in C++ (units, enums, time ranges)
                        ▼
                 OSMWay / OSMNode / OSMAccessRestriction / OSMRestriction
```

### Three categories of keys C++ reads from the `Tags` map

1. **Lua-produced normalized keys** — only Lua emits them: `auto_forward`/`*_backward`, `road_class` (int), `use` (int), `default_speed`, `access_mask` (bitmask), `tagged_access`, `cycle_lane_right/left` (int), `cycle_lane_*_opposite`, `bike_network_mask`, `shoulder_right/left`, `gate`/`bollard`/`sump_buster`, `max_speed` (+`"unlimited"` sentinel), `forward_speed`/`backward_speed`/`average_speed`/`advisory_speed`, `private`/`private_hgv`/`no_thru_traffic`, `roundabout`, `ferry`/`rail`, `hov_type` (`HOV2`/`HOV3`), `truck_route`, `*_tag` flags, the `restriction*` ints, normalized `maxheight`/`maxwidth`/`maxlength`/`maxweight`/`maxaxleload` (in **meters/tonnes**), the `~` "except-destination" marker.
2. **Raw OSM keys passed straight through, parsed entirely in C++** — Lua never touches them: `surface`, `smoothness`, `tracktype`, `sac_scale`, `layer`, `level`/`level:ref`, `indoor`, `duration`, `destination*`, `turn:lanes*`, `guidance_view:*`, `maxspeed:conditional`, **all `*:conditional` access** (`motor_vehicle:conditional`, …), **all `name:*`/`ref:*`/`int_ref:*`/`destination:*` language + pronunciation variants** (IPA/nt-sampa/katakana/jeita).
3. **Dead handlers** — registered in C++ but nothing emits the key under the default Lua: `driving_side`, `internal_intersection`, `turn_channel` (see §5).

### Key facts that constrain the design

- **Dispatch is a `tag_handlers_` map** (`pbfgraphparser.cc:190+`), one lambda per key, mutating the member `way_`/`tag_`. Plus an `else-if` chain (`:2593+`) for conditional access and (`:2677+`) for language/pronunciation names.
- **The access bitmask is a silent ABI between Lua and C++.** `graph.lua`'s node tables hardcode `auto=1, foot=2, bike=4, truck=8, emergency=16, taxi=32, bus=64, hov=128, wheelchair=256, moped=512, motorcycle=1024` — these must *exactly* equal `kAutoAccess … kMotorcycleAccess` in `valhalla/baldr/graphconstants.h:37-47`. Same pattern for `road_class` ints, `use` ints, `restriction` ints, `CycleLane` ints. **Magic numbers mirrored across the boundary with no compile-time check.**
- **Units are split across the boundary.** Lua `normalize_measurement`/`normalize_weight` do ft/in/lb/kg → meters/tonnes; C++ then multiplies by 100 → cm / centi-tonnes. Lua `normalize_speed` does mph→kph and tosses `<10`; C++ `to_float` does the rest.
- **String-protocol quirks** the boundary depends on: trailing `~` = "except destination" on dimensional restrictions; `max_speed == "unlimited"` = autobahn sentinel; `duration` as `HH:MM:SS`; restriction time ranges re-parsed by `get_time_range()`.
- **Lifecycle:** `graph.lua` is **embedded into the binary** at build time via `cmake/ValhallaBin2Header.cmake` → `graph_lua_proc.h` (`const unsigned char lua_graph_lua[]`). Overridable at runtime by config key **`graph_lua_name`** (a file path). Fallback = embedded copy.
- **Threading:** one `lua_State` per worker thread (`LuaTagTransform` is non-copyable; constructed per-thread in the Lua pool, `pbfgraphparser.cc:~5180`). No locking. `bit.bor` implies a LuaJIT/BitOp dependency.
- **Passes:** three passes over the PBF (ways, relations, nodes); each calls `Transform` once per entity that has tags. Empty-tag entities reuse a cached `empty_*_tags_`.
- **`Transform` is already nearly a pure function** of `(type, tags)` — its only state is the parsed Lua chunk. That's good news for the `.osc` per-entity goal; the obstacle to modularity is the *3-pass, in-place-mutation, stringly-typed* design around it, not `Transform` itself.

---

## 3. Why remove it (the goals, concretely)

| Goal | What blocks it today |
|------|----------------------|
| **Testable** | Tag logic is in Lua (no C++ unit test reach) *and* split with C++; the only way to test is full tile build + routing assertion. Can't unit-test "these tags → these OSMWay fields". |
| **Modular for `.osc`** | Single-tile update needs a pure `tags → typed edge/node attrs` function callable per entity. Today that means: spin a `lua_State`, marshal strings in, get a flat map out, re-parse in C++. Heavy and stateful. |
| **Maintainable** | Logic for one concept (e.g. conditional restrictions, units, oneway) is smeared across Lua *and* C++ with magic-number coupling. |

---

## 4. Pitfalls / risks (call these out to reviewers)

1. **Custom Lua is a public API.** `graph_lua_name` lets users replace tag parsing without recompiling. Removing Lua removes that. Options: (a) accept the break and document it; (b) ship a config-driven override layer for the common cases (access tables, speed defaults) that today people fork Lua to change; (c) keep a thin optional Lua hook *only* for user overrides while the default path goes native. **Decide before Stage 5.**
2. **Planet-scale behavior parity.** ~2500 lines of Lua with dense edge cases (the oneway forward/backward reset, mph conversion, hov-only-lane logic, cyclelane L1b/M2d cases, bollard/sump_buster/wall node access). A silent divergence changes routing for the whole planet. → differential testing is non-negotiable (§6).
3. **Magic-number enum mirrors.** Port must reuse the actual C++ enums/constants, not re-hardcode. Add `static_assert`s so the mapping can't drift again.
4. **Evaluation-order dependence.** Lua mutates `kv` in place; later rules read keys written earlier (e.g. `use` depends on already-computed `auto_forward`/`bike_forward`; the final construction block zeroes access *after* `use` is set). A native port must preserve order or results change.
5. **The stringly-typed boundary hides the contract.** There's no schema for the `Tags` map. Easy to miss a key during the port. Mitigate by enumerating the full key set from the differential corpus, not from reading code.
6. **Two "conditional" mechanisms.** Dimensional conditionals use the Lua `~` marker; access conditionals are parsed wholesale in C++ from raw tags; `maxspeed:conditional` is yet another C++ path. Don't unify them silently during the port — preserve each, refactor later.
7. **Don't conflate already-C++ logic.** Language/pronunciation/conditional-access parsing is *not* in Lua. Migrating Lua shouldn't touch it; just keep feeding those raw tags through.
8. **Performance is not the win.** Tag parsing isn't the planet-build bottleneck; don't justify the change on speed. The win is testability/modularity. (Native will be faster and drop the LuaJIT dep — nice, not the point.)

---

## 5. Inconsistencies between Lua logic and C++ usage (separate, as requested)

These are real today, independent of the migration. Worth fixing/confirming regardless.

1. **Dead handler: `internal_intersection`.** C++ registers `tag_handlers_["internal_intersection"]` (fires only if `!infer_internal_intersections_`), but the default `graph.lua` only ever emits **`tagged_internal_intersection`** (from `junction=intersection`), which has its own handler. `internal_intersection` is not an OSM tag → unreachable under default Lua.
2. **Dead handler: `turn_channel`.** Same story — `graph.lua` never emits `turn_channel`; C++ infers turn channels itself. Handler only reachable via a custom Lua script.
3. **Dead/vestigial handler: `driving_side`.** Not emitted by `graph.lua` and not an OSM tag. Drive-side comes from the admin DB or the `set_drive_on_right(true)` default (`:2570`). The handler only matters for a custom Lua + `use_admin_db=false`.
4. **`maxspeed:conditional` bypasses Lua speed normalization.** `maxspeed`/`maxspeed:forward`/etc. go through Lua `normalize_speed` (mph→kph, toss `<10`). `maxspeed:conditional` is parsed raw in C++ with `to_float` — **no mph conversion, no low-speed toss.** A conditional `50 mph @ …` is mishandled relative to the unconditional path.
5. **Split unit handling.** ft/in/lb/kg → metric conversion lives in Lua; the ×100 scale-to-int lives in C++. The two must stay in lockstep; nothing enforces it.
6. **Construction double-encoding.** Lua sets `use=43` (kConstruction) *and* separately force-zeroes every access flag for `highway=construction` "for backward compatibility" with old routers. Two independent mechanisms encode the same fact; easy to break one.
7. **Late unconditional overrides.** `lanes:bus`/`lanes:psv` (`graph.lua:1508+`) and the `cycleway:both`/busway blocks set `bus_forward`/`bike_*` near the end, potentially overriding the carefully computed oneway/backward logic above them. Intended? Undocumented.
8. **Divergent numeric validators.** Lua `numeric_prefix` (caps lanes `>15`→nil, decimals flag) vs C++ `to_int`/`to_float`. Edge cases (e.g. `"3;4"`, `"~"`, trailing units) can parse differently depending on which side handles the key.
9. **The `oneway==nil/false` branch resets backward = forward** for most modes (`graph.lua:1347+`) — the in-file comment literally asks "what on earth is going on here?". Behavior is load-bearing (most ways have no `oneway`); flagging because any port will have to replicate this exactly and it's the kind of thing a reader will "fix" and break the planet.

---

## 6. The linchpin: differential-testing harness (Stage 0, do first)

A pure-C++ harness that, given an OSM tag set + entity type, returns the `Tags` map, with two backends (Lua, native), plus an assert-equal.

- **Corpus**: (a) high-frequency real tag combinations mined from a planet/region PBF (or taginfo), (b) the existing `test_requests/` and gurka maps, (c) hand-written adversarial cases for every branch in §5 and every `any_in`/oneway/cyclelane case.
- **Golden snapshot**: serialize current Lua `Transform` output for the whole corpus → checked-in golden file. Any Lua-vs-native diff is a test failure with the offending key.
- **Tooling**: a small `valhalla_parse_tags` debug CLI (`tags + type → JSON map`) is independently useful for debugging and for the §2 "design the ASCII map to match real OSM" workflow.

This harness is what makes every later stage safe and reviewable in isolation.

---

## 7. Target architecture

Replace `LuaTagTransform` with a native `TagTransform`:

```cpp
struct TagTransform {                       // pure, stateless, per-entity
  Tags Transform(OSMType, uint64_t osmid, const osmium::TagList&) const;  // Stage 1-3 contract
};
```

- **Stages 1–3** keep the `Tags = map<string,string>` contract so the port can be validated key-for-key against Lua and pbfgraphparser stays untouched.
- **Stage 4** retires the stringly-typed round-trip: the native transform populates a **typed result struct** (or `OSMWay`/`OSMNode` directly), deleting the C++ re-parse layer. This is where the testability/modularity payoff lands — a pure `tags → typed attrs` function with no `lua_State`, no flat map, callable per-entity for `.osc`.
- Reuse the real enums/constants (`graphconstants.h`) with `static_assert`s; centralize unit/number parsing in one place; keep language/pronunciation/conditional-access C++ code as-is.

---

## 8. Staged migration (1 PR per stage, each independently shippable)

| Stage | PR | Risk | Validated by |
|------|-----|------|--------------|
| **0** | Differential harness + `valhalla_parse_tags` CLI + golden snapshot of current Lua output. **No behavior change.** | none | itself |
| **1** | Introduce `TagTransform` interface; make `LuaTagTransform` implement it; `pbfgraphparser` depends on the interface, not Lua directly. Add config `tag_transform: lua\|native` (default `lua`). | none | build + existing tests |
| **2** | Audit & lock the magic-number mirrors: `static_assert`s tying Lua's emitted ints to `graphconstants.h` enums; document the `~`/`unlimited`/`HH:MM:SS` string protocol. Fix the §5 dead handlers + `maxspeed:conditional` mph bug (each its own commit, each a golden-snapshot update with rationale). | low | harness |
| **3a** | Native `nodes_proc` (smallest, self-contained: `access_mask`, `tagged_access`, barriers, signals/stops, toll/border). Flip native for nodes behind the flag; differential-test on a region extract. | med | harness + region build diff |
| **3b** | Native `rels_proc` (restrictions, bike networks, conditional/probable restriction parsing). | med | harness |
| **3c…** | Native `ways_proc` in feature slices, each a PR: (i) access + oneway/forward-backward, (ii) speeds + units, (iii) `use` + `road_class`, (iv) cyclelane/shoulder, (v) truck dimensions + hov + networks. Each slice differential-tested; native stays gated until all green. | med-high | harness + planet-extract diff |
| **4** | Default `tag_transform: native`. Run a **full planet build diff** (tile-level) Lua vs native; sign off on zero/explained diffs. | high | planet diff |
| **5** | Delete the stringly-typed round-trip: native populates typed `OSMWay`/`OSMNode` directly, remove the C++ re-parse handlers that only existed to undo Lua's string encoding. | med | harness + tests |
| **6** | Remove `graph.lua`, `LuaTagTransform` (graph path), `graph_lua_proc.h` bin2header, `graph_lua_name` config, Lua dep (if `admin.lua` also gone — out of scope here, so Lua dep likely stays for admins). **Resolve the custom-Lua extension story (§4) here.** | low (mechanical) | build |

Sequencing notes:
- Stages 3a→3c can land in any order *within* themselves but ship gated; the flag flip (Stage 4) is the only behavior-changing PR and it's a single, reviewable, diffable event.
- `.osc`/single-tile work can start the moment Stage 4 lands (pure native transform exists); Stage 5 makes it clean.
- Keep `admin.lua` on the Lua path for now — it shares `LuaTagTransform` but is a separate script and separate tool, so the class and Lua dependency survive Stage 6 unless/until admins are migrated too.

---

## 9. Open questions for the team

- Custom-Lua extension point: drop, replace with config tables, or keep an optional hook? (gates Stage 6)
- Typed intermediate (Stage 4): new struct, or write straight into `OSMWay`/`OSMNode`? The latter is less code but couples the transform to the storage structs (worse for `.osc` reuse if those structs carry build-pass state).
- Acceptance bar for Stage 4 planet diff: bit-identical tiles, or "explained diffs only" (where native intentionally fixes a §5 bug)?
