# Docs restructure proposal

## TL;DR

Two independent problems, two independent fixes:

1. **The sidebar is a wall of text** — not because there are too many files, but because `mkdocs.yml` uses `navigation.sections`, which force-expands *every* group simultaneously. Fix = switch to **collapsible left-sidebar sections**. This is a config-only change, zero file moves, and does 80% of the perceived improvement.
2. **The information architecture is muddled** — 11 top-level groups, and the split between *"Internal topics"* and *"Internal components"* is arbitrary (sif, meili, thor, mjolnir each appear in both). Fix = reorganize around **audience** (use it → integrate it → understand it → hack on it) and mirror that on disk.

Then, standardize the API reference pages so every endpoint reads the same way — no per-endpoint special-casing.

---

## Ground rules (every restructure PR obeys these)

1. **Prose is never reworded.** The words describing Valhalla — concepts, behavior, API semantics — move **verbatim**. Permitted while moving a block: link/path updates, heading-level shifts, front-matter. Nav labels in `mkdocs.yml` are not prose and may be renamed freely. Any actual rewording is its own separate PR, never folded into a move.
2. **Every endpoint doc shares the same structure.** We never split or template one endpoint differently from the rest. A reader learns the layout once and it holds for all endpoints.
3. **Every move is logged.** Each PR ships a human-readable change log (`RESTRUCTURE_LOG.md` or the PR body) mapping old → new: moved / split (which block → which file) / merged. Pure renames use `git mv` so git tracks the rename; splits and merges get one explicit line each. A reviewer verifies "no prose changed" from the log + `git diff -M`, not by re-reading every page.

---

## Problem 1 — the sidebar (highest impact, lowest effort)

Current theme features:

```yaml
features:
  - navigation.sections   # <-- renders every group as an always-open bold header
```

`navigation.sections` turns each nav group into a permanently-expanded section. With 11 groups and deep nesting, the reader sees ~60 links at once with no way to collapse. It reads as noise.

**Change to:**

```yaml
features:
  - navigation.indexes       # a section's landing page attaches to its header (no orphan "Overview" row)
  - navigation.top           # back-to-top
  - navigation.instant
  - navigation.instant.prefetch
  - toc.follow
  - content.code.copy
  - content.tabs.link
```

Drop `navigation.sections`. **Do not** add:
- `navigation.tabs` — moves the top-level groups into the **header bar** as horizontal tabs. We want them in the **left sidebar**, not the header.
- `navigation.expand` — force-expands every node, i.e. the wall again.

With none of `sections`/`tabs`/`expand`, Material renders the whole nav in the **left sidebar as collapsible sections**: the section containing the current page is expanded, every other section collapses to a single clickable row with a chevron. That is the "hideable" behavior you want.

Net effect: the left menu shows the active section expanded and the rest folded away, instead of the entire site at once.

---

## Problem 2 — information architecture

Reorganize the 11 groups into **5 audience-scoped top-level sidebar sections**. The guiding question for each doc: *who is reading this and what are they trying to do?*

| Section | Audience | Answers |
|-----|----------|---------|
| **Get started** | everyone, first 10 min | what is this, how do I build/run it, what do the words mean |
| **HTTP API** | integrators | how do I call each endpoint |
| **Language bindings** | integrators | how do I embed Valhalla in Python/Node |
| **How it works** | power users & the curious | why tiles, how costing/speeds/shapes work — conceptual, no code-diving |
| **Contributing** | code contributors | module map, build stages, testing, data pipeline, release |

Key moves:

- **Kill the "Internal topics" vs "Internal components" split.** Concept pages (why-tiles, speeds, costing, decoding) go to **How it works**. The Norse-module pages (baldr, loki, midgard, …) collapse into a single **Contributing → Architecture** subtree, one entry per module with its sub-pages nested under it — instead of being scattered top-level entries named after gods nobody recognizes.
- **`decoding.md` moves to How-it-works / or API** — it's a consumer concern (decode the `shape` you got back), not an internal.
- **Consolidate each module's pages.** Today `sif.md` + `sif/dynamic-costing.md` + `sif/elevation_costing.md` live in three different groups; `meili.md`/`thor.md`/`mjolnir.md` likewise. Each module becomes one collapsible node owning its children.
- **Filesystem mirrors the nav** (see tree). Right now `docs/docs/` is a flat dump with a few subdirs; the top-level dirs should match the sections so a contributor can find the source file from the URL and vice-versa.

---

## Problem 3 — page length & endpoint consistency

Most pages are already short (median ~65 lines); only the API references run long (Route 741, map-matching 392, locate 351). The instinct to split the Route reference conflicts with a stronger requirement: **every endpoint must read the same way.** Splitting only Route is exactly the inconsistency to avoid.

Two coherent options:

- **A (recommended) — one page per endpoint, shared section skeleton.** Keep each endpoint as a single `api-reference.md`; standardize the H2 layout across all of them (e.g. `Inputs` → `Options` → `Outputs` → `Examples`). Route stays long, but a uniform heading structure plus the right-hand page TOC make it navigable, and every endpoint is laid out identically. Nothing gets split.
- **B — split every endpoint identically** into `request` / `response` / `options` sub-pages. Uniform, but overkill for the small endpoints (isochrone is 72 lines) and multiplies file count for no gain.

Recommendation: **A**. Length is a navigation problem, solved by consistent in-page structure — not by fragmenting one endpoint. Standardizing headings only reorders/relocates existing blocks (no rewording), so it obeys the ground rules.

---

## Target structure

<details>
<summary><b>Proposed <code>docs/docs/</code> tree + nav</b></summary>

```
docs/docs/
├── index.md                          # landing / what is Valhalla
│
├── start/                            # section: Get started
│   ├── introduction.md               # was valhalla-intro.md
│   ├── building.md
│   └── terminology.md
│
├── api/                              # section: HTTP API
│   ├── index.md                      # API overview (section index)
│   ├── openapi.md
│   ├── protocol-buffers.md
│   ├── decoding.md                   # moved from top level — consumer concern
│   ├── route/
│   │   ├── overview.md               # unchanged
│   │   └── api-reference.md          # NOT split — uniform H2 skeleton, shared by all endpoints
│   ├── optimized.md
│   ├── matrix.md
│   ├── isochrone.md
│   ├── map-matching.md
│   ├── locate.md
│   ├── elevation.md
│   ├── expansion.md
│   ├── status.md
│   └── tile.md
│
├── bindings/                         # section: Language bindings
│   ├── python/
│   │   ├── index.md                  # getting started — symlink → src/bindings/python/README.md
│   │   └── api/                      # API reference (mkdocstrings pages)
│   │       ├── actor.md
│   │       ├── config.md
│   │       ├── graph_utils.md
│   │       ├── predicted_speeds.md
│   │       ├── decode_polyline.md
│   │       └── exceptions.md
│   └── nodejs.md                     # symlink → src/bindings/nodejs/README.md
│
├── concepts/                         # section: How it works
│   ├── index.md                      # short conceptual overview / route pipeline (route_overview.md)
│   ├── why-tiles.md
│   ├── tiles.md                      # tile structure
│   ├── speeds.md
│   ├── historical-traffic.md         # was mjolnir/historical_traffic.md
│   ├── incidents.md
│   ├── change-identification.md
│   ├── elevation.md
│   └── costing/
│       ├── dynamic-costing.md
│       └── elevation-costing.md
│
└── contributing/                     # section: Contributing
    ├── index.md                      # was contributing.md
    ├── testing.md                    # was test/gurka.md
    ├── releasing.md
    ├── tzdb-update.md
    ├── locales.md
    ├── data/                         # data sources & provenance
    │   ├── data-sources.md
    │   ├── admins.md
    │   └── attribution.md
    └── architecture/                 # the Norse modules, all in one place
        ├── index.md                  # module map (the diagram from CLAUDE.md)
        ├── baldr.md
        ├── loki.md
        ├── midgard.md
        ├── odin.md
        ├── skadi.md
        ├── tyr.md
        ├── sif.md
        ├── thor/
        │   ├── index.md
        │   ├── path-algorithm.md
        │   └── isochrones.md
        ├── meili/
        │   ├── index.md              # overview
        │   ├── algorithms.md
        │   ├── configuration.md
        │   ├── implementation-details.md
        │   ├── library-api.md
        │   └── service-api.md
        └── mjolnir/
            ├── index.md              # + why_tiles/getting_started merged or linked
            ├── tag-parsing.md
            ├── geojson.md
            ├── map-roulette.md
            └── getting-started.md
```

Corresponding `nav:` in `mkdocs.yml`:

```yaml
nav:
  - Home: index.md
  - Get started:
      - start/introduction.md
      - start/building.md
      - start/terminology.md
  - HTTP API:
      - api/index.md
      - Formats:
          - api/openapi.md
          - api/protocol-buffers.md
          - api/decoding.md
      - Route:
          - api/route/overview.md
          - api/route/api-reference.md
      - Optimized: api/optimized.md
      - Matrix: api/matrix.md
      - Isochrone: api/isochrone.md
      - Map matching: api/map-matching.md
      - Locate: api/locate.md
      - Elevation: api/elevation.md
      - Expansion: api/expansion.md
      - Status: api/status.md
      - Tile: api/tile.md
  - Language bindings:
      - Python:
          - bindings/python/index.md
          - API reference:
              - Actor: bindings/python/api/actor.md
              - Config: bindings/python/api/config.md
              - Graph utilities: bindings/python/api/graph_utils.md
              - Predicted speeds: bindings/python/api/predicted_speeds.md
              - Polyline decoder: bindings/python/api/decode_polyline.md
              - Exceptions: bindings/python/api/exceptions.md
      - Node.js: bindings/nodejs.md
  - How it works:
      - concepts/index.md
      - concepts/why-tiles.md
      - concepts/tiles.md
      - concepts/speeds.md
      - concepts/historical-traffic.md
      - concepts/incidents.md
      - concepts/change-identification.md
      - concepts/elevation.md
      - Costing:
          - concepts/costing/dynamic-costing.md
          - concepts/costing/elevation-costing.md
  - Contributing:
      - contributing/index.md
      - contributing/testing.md
      - Architecture:
          - contributing/architecture/index.md
          - contributing/architecture/baldr.md
          - contributing/architecture/loki.md
          - contributing/architecture/midgard.md
          - contributing/architecture/odin.md
          - contributing/architecture/sif.md
          - contributing/architecture/skadi.md
          - contributing/architecture/tyr.md
          - Thor:
              - contributing/architecture/thor/index.md
              - contributing/architecture/thor/path-algorithm.md
              - contributing/architecture/thor/isochrones.md
          - Meili:
              - contributing/architecture/meili/index.md
              - contributing/architecture/meili/algorithms.md
              - contributing/architecture/meili/configuration.md
              - contributing/architecture/meili/implementation-details.md
              - contributing/architecture/meili/library-api.md
              - contributing/architecture/meili/service-api.md
          - Mjolnir:
              - contributing/architecture/mjolnir/index.md
              - contributing/architecture/mjolnir/tag-parsing.md
              - contributing/architecture/mjolnir/geojson.md
              - contributing/architecture/mjolnir/map-roulette.md
              - contributing/architecture/mjolnir/getting-started.md
      - Data:
          - contributing/data/data-sources.md
          - contributing/data/admins.md
          - contributing/data/attribution.md
      - contributing/locales.md
      - contributing/releasing.md
      - contributing/tzdb-update.md
      - Changelog: https://github.com/valhalla/valhalla/blob/master/CHANGELOG.md
```

</details>

---

## Why this ordering

- **Audience-first, in reading order.** The five sections trace a user's journey: *what is it → call it over HTTP → embed it → understand it → change it*. Nobody hits a wall of module names before they've run a route.
- **Sections collapse by default.** A Python integrator expands "Language bindings" and never sees mjolnir's tag-parsing internals. A contributor expands "Contributing" for the full architecture tree without API reference noise. The active section is open, the rest are folded to one row each — not 60 links at once.
- **One home for the Norse modules.** `baldr/loki/midgard/...` are meaningless to users and only navigational to contributors — burying them under **Contributing → Architecture** removes 8 opaque top-level entries at a stroke.
- **URL ↔ source path parity.** `/api/route/request/` lives at `docs/docs/api/route/request.md`. Today the flat layout means a URL tells you little about the file.
- **kebab-case filenames** (`historical-traffic.md`, not `historical_traffic.md`) for consistent, clean URLs. Requires redirect handling (below).

---

## Migration notes / risks

- **Broken inbound links.** Moving/renaming files changes URLs. `valhalla.github.io/valhalla` is linked from README, issues, external blogs. Add the **`mkdocs-redirects`** plugin and map every old path → new path:
  ```yaml
  plugins:
    - redirects:
        redirect_maps:
          'valhalla-intro.md': 'start/introduction.md'
          'mjolnir/tag_parsing.md': 'contributing/architecture/mjolnir/tag-parsing.md'
          # ... one line per moved file
  ```
- **In-repo cross-links.** Many docs link each other with relative paths; grep and fix (`rg -l '\]\(\.\./|\]\([a-z]' docs/docs`).
- **`CLAUDE.md` references** point at concrete paths (`docs/docs/sif/dynamic-costing.md`, `docs/docs/test/gurka.md`, etc.). The **"In-Repo Documentation"** table and every "Where to Look" path must be updated in the same PR, or every future session starts from stale paths.
- **`edit_uri`** stays `edit/master/docs/docs/` — still correct.
- **`README_python.md` / `README_nodejs.md` are symlinks** into `src/bindings/{python,nodejs}/README.md`. They stay symlinks — recreate them at their new paths (`bindings/python/index.md`, `bindings/nodejs.md`) with the target relative to the new depth (`../../../src/bindings/...`, one `..` deeper than today). `mkdocstrings` still needs `paths: [../src/bindings/python]` in `mkdocs.yml` for the `api/` pages to resolve.
- **Standardizing the endpoint section skeletons is block reordering, not rewording** — do it as its own commit, each moved block logged. This replaces the earlier "split the Route reference" idea.
- **Ship a change log with every moves PR** (`RESTRUCTURE_LOG.md` or PR body): old → new per file, split/merge blocks called out, so reviewers verify no prose changed without re-reading every page. Use `git mv` for pure moves so `git diff -M` shows renames, not delete+add.

## Suggested sequencing

1. **Config-only** PR: drop `navigation.sections` for collapsible left-sidebar sections, no file moves. Ship the perceived win immediately; regroup the *existing* files under the 5 sections in `nav:` without moving them on disk.
2. **File moves** PR: physically reorganize to the tree above, add `mkdocs-redirects`, fix cross-links + `CLAUDE.md`.
3. **Content** PR: standardize the endpoint reference section skeletons (block reordering only); optionally merge the thin mjolnir stubs. Each move logged; no prose reworded.
</content>
</invoke>
