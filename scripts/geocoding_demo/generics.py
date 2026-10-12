"""The small explicit tables Design B still needs, keyed on light-normalized tokens.

Two jobs only:
- spotting a name's generic, so `Hauptweg` vs `Hauptstrasse` can be penalized - IDF weighs both
  generics down to nothing and cannot tell those streets apart;
- expanding abbreviations that are not a prefix of their full form (`rd` -> `road`). Truncating
  ones (`str`, `ave`) need no entry for scoring, trigram containment already covers them.

Not meant to be complete. A generic missing here costs the mismatch penalty, nothing else.
"""

# lead the name as their own token: `Via Roma`, `Rue de Rivoli`, `Ulica Dluga`
PREFIX_GENERICS = frozenset(
    (
        "rue", "avenue", "boulevard", "place", "chemin", "impasse", "allee", "quai", "route", "cours",
        "via", "viale", "piazza", "corso", "vicolo", "largo", "strada",
        "calle", "avenida", "plaza", "paseo", "camino", "carrera", "carretera", "ronda",
        "rua", "praca", "travessa",
        "ulica", "aleja", "plac", "osiedle", "rondo",
        "namesti", "trida",
    )
)  # fmt: skip

# close the name as their own token: `Main Street`, `Ataturk Caddesi`, `Hermann Strasse`
SUFFIX_GENERICS = frozenset(
    (
        "street", "road", "avenue", "lane", "drive", "boulevard", "place", "court", "way", "close",
        "crescent", "terrace", "square", "highway", "parkway",
        "strasse", "weg", "gasse", "platz", "allee", "damm", "ufer", "pfad", "steig", "chaussee",
        "straat", "laan", "plein", "gracht", "kade", "singel", "dijk",
        "gata", "gatan", "vagen", "vegen", "gate", "vei", "veien", "vej", "gade", "torg", "torget",
        "caddesi", "sokak", "sokagi", "bulvari",
        "utca", "ut", "ter", "korut",
    )
)  # fmt: skip

# glued onto the last token: `Hauptstrasse`, `Drottninggatan`, `Strandvej`. Longest match wins
GLUED_GENERICS = tuple(
    sorted(
        (
            "strasse", "weg", "gasse", "platz", "allee", "damm", "ufer", "pfad", "steig", "chaussee",
            "straat", "laan", "plein", "gracht", "kade", "singel", "dijk",
            "gatan", "gata", "vagen", "vegen", "veien", "vej", "gade", "torget", "torg",
        ),
        key=len,
        reverse=True,
    )
)  # fmt: skip

# abbreviation -> the generics it may stand for, by where it sits
PREFIX_ABBREVIATIONS = {
    "av": ("avenue", "avenida"),
    "avda": ("avenida",),
    "bd": ("boulevard",),
    "bld": ("boulevard",),
    "bvd": ("boulevard",),
    "pl": ("place", "plac", "plaza"),
    "pza": ("piazza", "plaza"),
    "ch": ("chemin",),
    "imp": ("impasse",),
    "rte": ("route",),
    "vle": ("viale",),
    "cl": ("calle",),
    "ctra": ("carretera",),
    "cra": ("carrera",),
    "tv": ("travessa",),
    "ul": ("ulica",),
    "al": ("aleja",),
    "os": ("osiedle",),
    "nam": ("namesti",),
}

SUFFIX_ABBREVIATIONS = {
    "st": ("street",),
    "rd": ("road",),
    "av": ("avenue",),
    "ave": ("avenue",),
    "ln": ("lane",),
    "dr": ("drive",),
    "blvd": ("boulevard",),
    "pl": ("place", "platz", "plein"),
    "ct": ("court",),
    "cres": ("crescent",),
    "tce": ("terrace",),
    "terr": ("terrace",),
    "sq": ("square",),
    "hwy": ("highway",),
    "pkwy": ("parkway",),
    "str": ("strasse", "straat"),
    "cad": ("caddesi",),
    "cd": ("caddesi",),
    "sk": ("sokak",),
    "sok": ("sokak",),
    "blv": ("bulvari",),
    "gt": ("gate",),
}

GLUED_ABBREVIATIONS = {"str": ("strasse", "straat")}

# a glued generic only counts when it leaves a stem this long: `Weg` alone is a name, not a generic
MIN_STEM = 3


class Generics:
    """What `find_generics` read off a name: the generic forms it carries and what remains."""

    __slots__ = ("forms", "core")

    def __init__(self, forms, core):
        self.forms = forms
        self.core = core


def find_generics(tokens):
    """Split light tokens into generic forms and the core, the part that actually names the street."""
    forms = set()
    tokens = list(tokens)

    if len(tokens) >= 2:
        first = tokens[0]
        if first in PREFIX_GENERICS:
            forms.add(first)
            tokens = tokens[1:]
        elif first in PREFIX_ABBREVIATIONS:
            forms.update(PREFIX_ABBREVIATIONS[first])
            tokens = tokens[1:]

    if len(tokens) >= 2:
        last = tokens[-1]
        if last in SUFFIX_GENERICS:
            forms.add(last)
            tokens = tokens[:-1]
        elif last in SUFFIX_ABBREVIATIONS:
            forms.update(SUFFIX_ABBREVIATIONS[last])
            tokens = tokens[:-1]

    if tokens and not forms:
        last = tokens[-1]
        stem = None
        for generic in GLUED_GENERICS:
            if last.endswith(generic) and len(last) - len(generic) >= MIN_STEM:
                forms.add(generic)
                stem = last[: -len(generic)]
                break
        if stem is None:
            for abbreviation, expansions in GLUED_ABBREVIATIONS.items():
                if last.endswith(abbreviation) and len(last) - len(abbreviation) >= MIN_STEM:
                    forms.update(expansions)
                    stem = last[: -len(abbreviation)]
                    break
        if stem is not None:
            tokens = tokens[:-1] + [stem]

    return Generics(forms, "".join(tokens))


def expand_abbreviations(tokens):
    """Return token lists with a leading or closing abbreviation spelt out, the original first."""
    variants = [list(tokens)]
    if len(tokens) < 2:
        return variants

    firsts = [tokens[0]] + list(PREFIX_ABBREVIATIONS.get(tokens[0], ()))
    lasts = [tokens[-1]] + list(SUFFIX_ABBREVIATIONS.get(tokens[-1], ()))
    for first in firsts:
        for last in lasts:
            if first == tokens[0] and last == tokens[-1]:
                continue
            variants.append([first] + list(tokens[1:-1]) + [last])
    return variants
