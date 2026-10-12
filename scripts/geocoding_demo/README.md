# `street_name` hint prototype

Prototype for the waypoint `street_name` hint described in `geocoding_problem_description.md` and
designed in `geocoding_design_notes.md`. It exists to put numbers on τ, α, β and γ before any C++
gets written in `src/loki/search.cc`. The design notes record the decisions and measured values; this
file describes how the prototype implements them.

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
| `fixtures/` | nothing — real name pools the fixtures run against |

## Stages

Built one at a time; `--show` selects which one reports.

| `--show` | Stage | Status |
|---|---|---|
| `pool` | candidate pool from `/locate` | done |
| `normalize` | transliteration and normalization passes | done |
| `grams` | trigram bags, IDF, the scoring formula | done |
| `scores` | per-candidate name score and aggregation | done |
| `rank` | soft filter into `edges` / `filtered_edges`, thor's origin score | done |

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

## Scoring

`similarity.py` scores a hint against every distinct name in the pool, once per normalization pass:

```
weight(g) = ln(N / max(df(g), 1))           N = distinct normalized names in the pool
score     = Σ weight(g) · shared(g)  /  Σ weight(g) · hint(g)
```

- **Divided by the hint, not the smaller bag.** Hint grams missing from a candidate count against it;
  candidate grams missing from the hint don't, which is what lets `Doblerstr` score 1.0 against
  `Doblerstraße`.
- **A hint gram no nearby name contains gets the maximum weight.** Dropping it instead would let
  `Mozartstr` score 1.0 against every `-straße` in the pool.
- **Documents are distinct normalized names**, so a street split into 40 edges, or spelt two ways, is
  one document.
- Fewer than two documents means every weight is 0 and the score is undefined (`-`).

`--show grams` prints the hint's grams with df and weight, then plain (unweighted), light and
aggressive scores per name. `main.py` checks the worked example from the design notes
(`Bergstr` → 1.00 / 0.38 / 0.07 / 0.00) on every run and raises if it drifts.

## Per-name and per-edge score

`score.py` turns a hint into one score per name, then per edge (max over the names it carries):

1. **Ref hint** (`B28`, `L 1208`: up to three letters, digits, one optional letter) - near-exact match
   on the light form, 1.0 or 0.0, nothing else runs. A street hint never scores a ref name.
2. **Evidence floor** - the hint's total gram weight must reach `EVIDENCE_K · ln(N)`, i.e. ~1.5 grams
   of maximal rarity. `Str` alone carries 0.45 against a floor of 7.6 in Tübingen and scores nothing.
3. **Light containment**, times `1 − GAMMA · uncovered` - the share of the candidate's weight the hint
   doesn't cover. Generics are cheap, so abbreviations barely pay; `Silbersteinstraße` pays for
   `silberst` when the hint is `leinstr`, which is what keeps it below TAU.
4. **Below TAU only**, two retries, best one wins: the aggressive pass × `BETA`, and edit distance
   (OSA, transposition = 1 edit) between the **cores** - the name with its generic removed - ×
   `FUZZY_DISCOUNT`.
5. **Generic mismatch** - both sides carry a generic and they share none: × `GENERIC_MISMATCH`. This
   is what keeps `Hauptweg` from scoring `Hauptstraße` via the core retry.

`generics.py` holds the generic forms (prefix, suffix, glued) and the abbreviation table; a leading or
closing abbreviation is also expanded into extra hint variants (`Main Rd` → `Main Road`), scored as
the best variant.

`--show scores` prints the hint's core, generics and evidence, then light / aggressive / fuzzy /
final per name and how many edges clear TAU. `WINNER_FIXTURES` in `main.py` pin nine cases
(abbreviation, transposition, umlaut, generic mismatch, ref, no-evidence, absent street, hint inside a
longer name) and raise on every run if one flips. Each requires its winner to be the *only* name at or
above TAU, since the soft filter keeps every edge that clears it. The `leinstr` case runs against
`fixtures/neukoelln.txt`, a snapshot of the 78 real names within 1 km. Small synthetic pools don't
reproduce its gram weights, and it is the case that fails at `GAMMA=0.15`.

| constant | value | |
|---|---|---|
| `TAU` | 0.6 | retry trigger here, hint threshold in stage 5 |
| `BETA` | 0.9 | aggressive pass discount |
| `FUZZY_DISCOUNT` | 0.85 | core edit-distance discount |
| `GAMMA` | 0.3 | candidate surplus weight |
| `EVIDENCE_K` | 1.5 | evidence floor, in units of `ln(N)` |
| `GENERIC_MISMATCH` | 0.6 | |

All guesses until the validation corpus exists.

## Ranking

`rank.py` stands in for the hook in `src/loki/search.cc` and ends where thor's
`thor_worker_t` scoring of `PathEdge.distance` ends.

- **Hint radius** (`HINT_RADIUS`, 100 m; per location in C++). Loki keeps only the closest edge at
  `radius=0`, so a hint has to collect candidates over this radius first. The names inside it are also
  the IDF corpus, and it caps how far a hint can move the correlation.
- **Hint used** when any edge inside the hint radius scores ≥ `TAU`. If none does, the output is
  exactly today's.
- **Soft filter.** Matching edges become `edges`, with distance replaced by `d · (1 − ALPHA · score)`,
  then loki's own rule runs on them: everything inside the location radius, otherwise the closest
  edge and its ties. What loki would have picked without the hint becomes `filtered_edges` -
  thor's existing fallback, retried only when `edges` find no route.
- **Thor score** is reproduced as `10 · d²`, minus the minimum, capped per costing (43200 for auto,
  which saturates at ~66 m), and filtered edges get `+3600 + worst kept`.

Only scaling distances (the design notes' formula alone) bounds a hint to 5× the closest edge's
distance at `ALPHA=0.8`. `Pfleghofstr` in the Tübingen pool is 27.3 m away against 4.5 m: the
effective distance is 5.7 m, which loses, so the filter is what lets the hint win.

`--show rank` prints `edges`, `filtered_edges` and the no-hint selection with raw distance, effective
distance, name score and thor score. `--location-radius` and `--hint-radius` set the two radii.
`RANK_FIXTURES` in `main.py` pin the side-street case and the no-match case.
