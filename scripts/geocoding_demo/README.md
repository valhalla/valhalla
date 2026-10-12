# `street_name` hint prototype

Prototype for the waypoint `street_name` hint described in `geocoding_problem_description.md` and
designed in `geocoding_design_notes.md`. It exists to put numbers on τ, α and β before any C++ gets
written in `src/loki/search.cc`.

Stdlib only — no venv, no `pip install`. Run from the repo root:

```bash
python3 scripts/geocoding_demo/main.py --name "Doblerstr" --lat 48.5216 --lon 9.0576
```

Responses are cached under `.cache/`, so add `--offline` after the first run and nothing touches the
network.

## Shape

Each module stands in for one eventual C++ translation unit, which is why the Python is explicit
rather than idiomatic — plain structs, plain loops, no comprehension tricks in the scoring path.

| File | Becomes, in C++ |
|---|---|
| `main.py` | nothing — CLI driver |
| `locate.py` | nothing — loki's bin search already holds these candidates |
| `candidate.py` | `PathEdge` and friends |
| `gen_translit.py` | nothing — dev tool that writes `translit.py` |
| `translit.py` | a `constexpr` fold table |
| `normalize.py` | `midgard/normalize.cc` |
| `similarity.py` | `midgard/similarity.cc` |
| `generics.py` | `constexpr` generic/abbreviation tables |
| `score.py` | the scoring added to loki |
| `rank.py` | the hook in `search.cc` |

## Stages

Built one at a time; `--show` selects which one reports.

| `--show` | Stage | Status |
|---|---|---|
| `pool` | candidate pool from `/locate` | done |
| `normalize` | transliteration and normalization passes | done |
| `grams` | trigram bags, IDF, the scoring formula | todo |
| `scores` | per-candidate name score and aggregation | todo |
| — | `effective_distance` ranking, τ/α guards | todo |

## Normalization

`translit.py` is generated, not written. `gen_translit.py` builds it from three sources in
precedence order — a hand-written `EXCEPTIONS` map, NFKD decomposition with combining marks dropped,
and the Unicode character name for `LATIN LETTER <X> WITH <modifier>` (the base letter is in the
name, which mechanically covers 70 stroke/hook/topbar letters NFKD leaves alone). That yields 387 of
the 401 letters in U+00A0–U+024F; the 14 left out are tone letters, clicks and glottal stops, listed
explicitly in `DELIBERATELY_UNMAPPED`.

```bash
python3 scripts/geocoding_demo/gen_translit.py           # rewrite translit.py
python3 scripts/geocoding_demo/gen_translit.py --check    # exits 1 if stale or newly unmapped
```

NFKD alone would not be enough even with ICU: `ß`, `ø`, `ł`, `æ`, `œ`, `þ` are not decomposable, so
transliteration is a deliberate per-codepoint choice. Since Valhalla has no ICU and header-only Boost
cannot normalize, the C++ side needs a literal table regardless — which is why the Python uses the
same table and keeps `unicodedata` confined to the generator.

Two passes, both applied to hint and candidate alike:

- **light** — lowercase, fold, split on anything non-alphanumeric. Resolves hyphen-vs-space, glued
  compounds (`Hermann Straße` and `Hermannstraße` both become `hermannstrasse`), and dropped
  diacritics.
- **aggressive** — additionally collapses `ue→u oe→o ae→a ss→s`, which is what makes `Müller`,
  `Mueller` and `Muller` agree. It discards real information, so it is a retry that carries a
  penalty, not the default.

`Normalized` exposes both `tokens` (word boundaries kept, for the token-level rules) and `stripped`
(boundaries discarded, what the trigram bag consumes).

Abbreviation is deliberately *not* normalization's job: `primustruberstr` stays distinct from
`primustruberstrasse`, and is a prefix of it. Turning that prefix relationship into a score is what
Stage 3 does.

Codepoints outside the table — Greek, Cyrillic, CJK — pass through unchanged, so normalized-exact and
trigram comparison still work within one script. There is no case folding for them, since that needs
Unicode data we do not ship; the stated target is Latin scripts with diacritics.

`--show normalize` checks the cases from the problem description, each declaring what normalization
owes it: `converge` (all variants reduce to one string), `prefix` (the short form must be a prefix of
the long one), or `fold` (a transliteration spot check with no relationship expected).

## Why `/locate`

`/locate?verbose=true` returns every edge the bin search projected onto, with `edge_info.names`,
`distance` and `edge_id` — which is exactly the input the real scorer sees. It is the only part of
this prototype that gets thrown away in the port.

One candidate per directed edge, as loki works on them: a two-way road appears twice, same distance,
`percent_along` of 0.0 and 1.0, headings 180° apart. `/locate` cannot report the same directed edge
twice, so there is nothing to deduplicate.

Two properties of the real data drive the design:

- **A street spans many edges.** In a 1 km pool around Tübingen, `Calwerstraße` covers 77 of them.
  IDF documents are therefore *distinct names*, not edges; counting edges would demote a street's own
  trigrams to "common, ignore" purely because OSM split the way into segments.
- **~40% of edges are unnamed.** They can never match a hint, so they skip normalization entirely.

The endpoint clamps `radius` to `service_limits.max_radius`, silently. The pool summary warns when
the farthest candidate sits well inside the requested radius, which is what that clamp looks like.
