#!/usr/bin/env python3
"""Entry point for the street_name hint prototype.

Run from the repo root:

    python3 scripts/geocoding_demo/main.py --name "Doblerstr" --lat 48.5216 --lon 9.0576

Stages land behind --show; see README.md.
"""

import argparse
import os
import sys

from locate import LocateError, load_pool
from normalize import normalize_aggressive, normalize_light, split_hint
from similarity import plain_containment, score_names, trigrams

DEFAULT_URL = "https://valhalla1.openstreetmap.de/locate"
DEFAULT_CACHE = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".cache")

# the cases named in geocoding_problem_description.md, each with what normalization owes it:
#   converge - every variant must reduce to one string, so they compare as equal
#   prefix   - the abbreviated variant must be a prefix of the full one, which is the property the
#              trigram scorer leans on; making them *equal* is not normalization's job
#   fold     - a spot check on the transliteration table, no relationship between the entries
FIXTURE_GROUPS = (
    (
        "hyphen vs space",
        "converge",
        ("Primus-Truber Straße", "Primus Truber Straße", "Primus-Truber-Straße"),
    ),
    ("abbreviated generic", "prefix", ("Primus-Truber-Str", "Primus Truber Straße")),
    ("glued vs split compound", "converge", ("Hermannstraße", "Hermann Straße", "Hermann-Straße")),
    ("umlaut typed three ways", "converge", ("Müller", "Mueller", "Muller")),
    ("diacritic dropped", "converge", ("Österbergstraße", "Osterbergstrasse", "Oesterbergstrasse")),
    ("generic before the name", "converge", ("Via Roma", "via roma")),
    (
        "non-decomposable letters",
        "fold",
        ("Łąkowa", "Nørregade", "Þingholtsstræti", "Ulica Świętokrzyska", "Çarşı Caddesi"),
    ),
)

# the worked example in geocoding_design_notes.md section 3: (hint, {name: expected light score})
SCORE_FIXTURES = (
    (
        "Bergstr",
        {"Bergstraße": 1.00, "Burgstraße": 0.38, "Bahnhofstraße": 0.07, "Schulweg": 0.0},
    ),
)
SCORE_TOLERANCE = 0.01


def parse_args(argv):
    parser = argparse.ArgumentParser(
        prog="geocoding_demo",
        description="prototype for the street_name waypoint hint",
    )
    parser.add_argument("--name", required=True, help="the street name hint, as a human would type it")
    parser.add_argument("--lat", type=float, required=True)
    parser.add_argument("--lon", type=float, required=True)
    parser.add_argument("--radius", type=int, default=1000, help="search radius in metres")
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--costing", default="auto")
    parser.add_argument("--cache", default=DEFAULT_CACHE, help="response cache dir, empty to disable")
    parser.add_argument("--offline", action="store_true", help="fail rather than issue a request")
    parser.add_argument(
        "--show",
        default="pool",
        choices=["pool", "normalize", "grams"],
        help="which stage to report",
    )
    parser.add_argument("--limit", type=int, default=20, help="rows to print in candidate tables")
    return parser.parse_args(argv)


def print_pool_summary(pool):
    total = len(pool.candidates)
    unnamed_share = (100.0 * pool.unnamed_count / total) if total else 0.0
    source = "cached" if pool.from_cache else "fetched"
    print("pool  lat={} lon={} radius={} m  ({})".format(pool.lat, pool.lon, pool.radius, source))
    print(
        "      {} edges, {} distinct names, {} unnamed ({:.1f}%), max distance {:.1f} m".format(
            total, len(pool.distinct_names), pool.unnamed_count, unnamed_share, pool.max_distance()
        )
    )
    # a server whose service_limits.max_radius is below the request clamps it silently
    if pool.radius > 60 and pool.max_distance() < 0.8 * pool.radius:
        print(
            "      warning: farthest candidate is well inside the requested radius - the server "
            "may be clamping to service_limits.max_radius"
        )


def print_candidate_table(pool, limit):
    rows = pool.candidates[:limit]
    print()
    print(
        "{:>8}  {:<18}  {:>10}  {:<12}  {:<12}  {}".format(
            "dist m", "graph id", "way id", "class", "use", "names"
        )
    )
    for candidate in rows:
        names = ", ".join(candidate.names) if candidate.names else "-"
        if len(names) > 40:
            names = names[:37] + "..."
        print(
            "{:>8.1f}  {:<18}  {:>10}  {:<12}  {:<12}  {}".format(
                candidate.distance,
                candidate.graph_id_str(),
                candidate.way_id,
                candidate.classification,
                candidate.use,
                names,
            )
        )
    if len(pool.candidates) > limit:
        print("... {} more (raise --limit)".format(len(pool.candidates) - limit))


def print_normalize_row(label, text):
    light = normalize_light(text)
    aggressive = normalize_aggressive(text)
    print(
        "  {:<24}  {:<28}  {:<26}  {}".format(label, light.joined, light.stripped, aggressive.stripped)
    )


def check_fixture(expectation, variants):
    """Return why normalization failed this group, or None if it delivered."""
    light = [normalize_light(v).stripped for v in variants]
    aggressive = [normalize_aggressive(v).stripped for v in variants]

    if expectation == "fold":
        return None

    if expectation == "prefix":
        longest = max(light, key=len)
        missed = [v for v, s in zip(variants, light) if not longest.startswith(s)]
        if missed:
            return "not a prefix of {!r}: {}".format(longest, ", ".join(missed))
        return None

    # converging only on the aggressive pass is still a pass, it just carries a penalty
    if len(set(light)) == 1 or len(set(aggressive)) == 1:
        return None
    return "{} distinct light, {} distinct aggressive".format(len(set(light)), len(set(aggressive)))


def check_fixtures():
    failures = []
    for label, expectation, variants in FIXTURE_GROUPS:
        reason = check_fixture(expectation, variants)
        if reason:
            failures.append("  {} [{}]: {}".format(label, expectation, reason))
    for hint, expected in SCORE_FIXTURES:
        _, _, scores = score_names(hint, list(expected), normalize_light)
        for name, want in expected.items():
            got = scores[name]
            if got is None or abs(got - want) > SCORE_TOLERANCE:
                failures.append(
                    "  score {!r} vs {!r}: got {}, want {:.2f}".format(hint, name, got, want)
                )
    if failures:
        raise RuntimeError("normalization fixtures failed:\n" + "\n".join(failures))


def print_normalize_report(hint, pool, limit):
    print()
    print(
        "  {:<24}  {:<28}  {:<26}  {}".format(
            "input", "light tokens", "light stripped", "aggressive stripped"
        )
    )

    street = print_hint_split(hint)
    print_normalize_row("HINT " + hint, street)

    print()
    print("candidate names ({} distinct, showing {})".format(len(pool.distinct_names), limit))
    for name in pool.distinct_names[:limit]:
        print_normalize_row(name, name)


def print_hint_split(hint):
    street, house, rest = split_hint(hint)
    if rest or house:
        print("  hint split: street={!r} house={!r} discarded={!r}".format(street, house, rest))
    return street


def format_score(score):
    return "    -" if score is None else "{:.3f}".format(score)


def print_grams_report(hint, pool, limit):
    print()
    street = print_hint_split(hint)
    names = pool.distinct_names
    index, hint_bag, light = score_names(street, names, normalize_light)
    aggressive_index, _, aggressive = score_names(street, names, normalize_aggressive)
    print(
        "  corpus: {} light / {} aggressive documents from {} distinct names".format(
            index.document_count, aggressive_index.document_count, len(names)
        )
    )
    if index.document_count < 2:
        print("  warning: fewer than two documents - every gram weighs 0, scores are undefined")

    print()
    print("  {:<6}  {:>3}  {:>6}   (light pass, hint {!r})".format("gram", "df", "weight", street))
    total = 0.0
    for gram in sorted(hint_bag):
        weight = index.weight(gram)
        total += weight * hint_bag[gram]
        count = "" if hint_bag[gram] == 1 else " x{}".format(hint_bag[gram])
        print(
            "  {:<6}  {:>3}  {:>6.2f}{}".format(
                gram, index.document_frequency.get(gram, 0), weight, count
            )
        )
    print("  {:<6}  {:>3}  {:>6.2f}".format("total", "", total))
    if not hint_bag:
        print("  hint is shorter than one gram - nothing to score")

    plain_hint = trigrams(normalize_light(street).stripped)
    rows = []
    for name in names:
        plain = plain_containment(plain_hint, trigrams(normalize_light(name).stripped))
        rows.append((name, plain, light[name], aggressive[name]))
    # best light score first, unscored names last
    rows.sort(key=lambda row: (row[2] is None, -(row[2] or 0.0), row[0]))

    print()
    print("  {:>5}  {:>5}  {:>5}  {}".format("plain", "light", "aggr", "name"))
    for name, plain, light_score, aggressive_score in rows[:limit]:
        print(
            "  {}  {}  {}  {}".format(
                format_score(plain), format_score(light_score), format_score(aggressive_score), name
            )
        )
    if len(rows) > limit:
        print("  ... {} more (raise --limit)".format(len(rows) - limit))


def main(argv):
    args = parse_args(argv)
    check_fixtures()
    try:
        pool = load_pool(
            args.lat,
            args.lon,
            args.radius,
            args.url,
            args.costing,
            args.cache or None,
            args.offline,
        )
    except LocateError as err:
        print("error: {}".format(err), file=sys.stderr)
        return 1

    print('hint  "{}"'.format(args.name))
    print_pool_summary(pool)

    if args.show == "pool":
        print_candidate_table(pool, args.limit)
    elif args.show == "normalize":
        print_normalize_report(args.name, pool, args.limit)
    elif args.show == "grams":
        print_grams_report(args.name, pool, args.limit)

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
