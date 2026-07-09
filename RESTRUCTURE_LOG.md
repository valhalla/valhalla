# Docs restructure — change log (PR 2+3)

Reviewer note: all file moves are pure renames — verify with `git diff -M --stat`. No prose was reworded; every content change beyond a move is listed under "Merges" and "Heading changes" below.

## File moves (old → new, all under `docs/docs/`)

The full map is machine-readable in `mkdocs.yml` → `plugins.redirects.redirect_maps` (67 entries, one per moved page; old URLs redirect). Summary:

| Group | Move |
|---|---|
| Get started | `valhalla-intro.md` → `start/introduction.md`; `building.md`, `terminology.md` → `start/` |
| HTTP API | `api/turn-by-turn/` → `api/route/`; each `api/<endpoint>/api-reference.md` → `api/<endpoint>.md`; `decoding.md` → `api/` |
| Bindings | `python/*.md` → `bindings/python/api/`; `README_python.md` → `bindings/python/index.md` (symlink recreated); `README_nodejs.md` → `bindings/nodejs.md` (symlink recreated) |
| How it works | `route_overview.md` → `concepts/index.md`; `tiles.md`, `speeds.md`, `elevation.md`, `incidents.md`, `change_identification.md` → `concepts/`; `mjolnir/why_tiles.md`, `mjolnir/historical_traffic.md` → `concepts/`; `sif/*costing*.md` → `concepts/costing/` |
| Contributing | `contributing.md` → `contributing/index.md` (symlink recreated); `testing.md`, `test/gurka.md` (→ `gurka.md`), `locales.md`, `releasing.md`, `tzdb_update.md` → `contributing/`; `mjolnir/{data_sources,admins,attribution}.md` → `contributing/data/`; Norse module pages → `contributing/architecture/` (thor/meili/mjolnir as subdirs with `index.md`) |
| Assets | `sif/images/` → `concepts/costing/images/`; `thor/images/` → `contributing/architecture/thor/images/`; `meili/figures/` → `contributing/architecture/meili/figures/`; `mjolnir/images/` split: `why_tiles.gif` → `concepts/images/`, `{ca,us}_admin.jpg` → `contributing/data/images/`, `maproulette_*.png` → `contributing/architecture/mjolnir/images/` |

All underscore filenames renamed to kebab-case. In-repo cross-links and image paths updated accordingly (also in `README.md`, `CONTRIBUTING.md`, `docker/README.md`, `CLAUDE.md`).

## Merges (content moved verbatim unless noted)

- `meili/overview.md` → merged into `contributing/architecture/meili/index.md`. Heading levels shifted one down (`# Overview` → `##`, etc.). Dropped from the old index: the "Documentation" link list (redundant with the nav). Added `# Meili` H1 (page had none).
- `mjolnir/getting_started_guide.md` → merged into `contributing/architecture/mjolnir/index.md` under a new `## Getting started guide` heading. Dropped: the guide's first paragraph (near-verbatim duplicate of the index intro) and the old index's dangling `## Components` stub (heading + one lead-in sentence, no content).

## Heading changes (endpoint skeleton standardization)

- `api/matrix.md`: `## Demonstration` heading removed; its demo link moved into the intro (matching route/optimized/tile).
- `api/status.md`: added `## Inputs of the Status service` above the existing verbose-parameter paragraph.
- `api/tile.md`: `## Request options` → `## Inputs of the tile service`; `## Attribute filters` demoted to `###` under it.
- `api/expansion.md`: `## Credits` → `## Data credits`; `api/elevation.md`: `## Data sources` → `## Data credits` (matches isochrone).

## Infrastructure

- `mkdocs-redirects` added to `docs/requirements.txt` and `pyproject.toml` `docs` group.
- `validation: anchors: warn` enabled in `mkdocs.yml` — with `--strict` CI now fails on broken `#fragment` links (baseline was zero).
- `mkdocs build --strict` passes.

Known anchor breakage (external links only, page-level redirects still land): fragments on renamed headings, e.g. `#request-options` on the tile page. `#inputs-of-a-route` etc. on the route page were deliberately NOT renamed to preserve the most-linked anchors.
