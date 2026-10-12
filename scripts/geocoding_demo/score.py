"""Stage 4: one hint score per name, then per candidate edge.

Per name: a ref hint is matched near-exactly and nothing else. Otherwise light containment, and only
when that misses TAU the aggressive pass and the edit-distance retry each get a go at a discount.
A generic mismatch scales whatever won.
"""

import math

from generics import expand_abbreviations, find_generics
from normalize import normalize_aggressive, normalize_light, split_hint
from similarity import (
    GramIndex,
    bag_weight,
    containment,
    edit_similarity,
    trigrams,
    uncovered_share,
)

TAU = 0.6  # a light score below this tries the retries; stage 5 ignores the hint below it
BETA = 0.9  # aggressive pass discount
FUZZY_DISCOUNT = 0.85  # edit-distance retry discount
GAMMA = 0.15  # how much candidate weight the hint does not cover may cost
EVIDENCE_K = 1.5  # minimum hint weight, in units of the rarest possible gram
GENERIC_MISMATCH = 0.6  # both sides carry a generic and they differ
MIN_FUZZY_CORE = 3


def looks_like_ref(stripped):
    """`b28`, `l1208`, `us101`, `a1a`: one to three letters, digits, at most one trailing letter."""
    length = len(stripped)
    position = 0
    while position < length and "a" <= stripped[position] <= "z":
        position += 1
    letters = position
    while position < length and "0" <= stripped[position] <= "9":
        position += 1
    digits = position - letters
    if position < length and "a" <= stripped[position] <= "z":
        position += 1
    return position == length and 1 <= letters <= 3 and 1 <= digits <= 5


class Hint:
    __slots__ = ("text", "street", "house", "rest", "ref", "generics", "variants")

    def __init__(self, text):
        self.text = text
        self.street, self.house, self.rest = split_hint(text)
        tokens = normalize_light(self.street).tokens
        stripped = "".join(tokens)
        self.ref = stripped if looks_like_ref(stripped) else None
        self.generics = find_generics(tokens)
        self.variants = expand_abbreviations(tokens)


class NameForms:
    """Everything the scorer needs from one candidate name, computed once per pool."""

    __slots__ = ("light_bag", "aggressive_bag", "ref", "generics")

    def __init__(self, name):
        light = normalize_light(name)
        self.light_bag = trigrams(light.stripped)
        self.aggressive_bag = trigrams(normalize_aggressive(name).stripped)
        self.ref = light.stripped if looks_like_ref(light.stripped) else None
        self.generics = find_generics(light.tokens)


class PassState:
    """One normalization pass: its corpus, the hint variants' bags, and whether they carry evidence."""

    __slots__ = ("index", "hint_bags", "evidence", "floor")

    def __init__(self, documents, hint_bags):
        self.index = GramIndex(documents)
        self.hint_bags = hint_bags
        self.evidence = 0.0
        for bag in hint_bags:
            self.evidence = max(self.evidence, bag_weight(bag, self.index))
        # with fewer than two documents every gram weighs 0, so no hint can clear the floor
        count = self.index.document_count
        self.floor = EVIDENCE_K * math.log(count) if count >= 2 else math.inf

    def has_evidence(self):
        return self.evidence > 0.0 and self.evidence >= self.floor

    def score(self, candidate_bag):
        """Best surplus-adjusted containment over the hint variants."""
        best = None
        for hint_bag in self.hint_bags:
            value = containment(hint_bag, candidate_bag, self.index)
            if value is None:
                continue
            value *= 1.0 - GAMMA * uncovered_share(hint_bag, candidate_bag, self.index)
            if best is None or value > best:
                best = value
        return best


class NameScore:
    __slots__ = ("name", "score", "via", "light", "aggressive", "fuzzy", "generic_mismatch")

    def __init__(self, name):
        self.name = name
        self.score = None
        self.via = ""
        self.light = None
        self.aggressive = None
        self.fuzzy = None
        self.generic_mismatch = False


def _documents(bags):
    """Distinct bags long enough to hold a gram - one document per distinct normalized name."""
    seen = {}
    for bag in bags:
        if bag:
            seen[tuple(sorted(bag.items()))] = bag
    return list(seen.values())


class HintScorer:
    __slots__ = ("hint", "forms", "light", "aggressive")

    def __init__(self, hint, names):
        self.hint = hint
        self.forms = {}
        for name in names:
            self.forms[name] = NameForms(name)

        light_bags = []
        aggressive_bags = []
        for tokens in hint.variants:
            light_bags.append(trigrams("".join(tokens)))
            aggressive_bags.append(trigrams(normalize_aggressive(" ".join(tokens)).stripped))
        light_documents = []
        aggressive_documents = []
        for forms in self.forms.values():
            light_documents.append(forms.light_bag)
            aggressive_documents.append(forms.aggressive_bag)
        self.light = PassState(_documents(light_documents), light_bags)
        self.aggressive = PassState(_documents(aggressive_documents), aggressive_bags)

    def score_name(self, name):
        forms = self.forms[name]
        result = NameScore(name)

        if self.hint.ref is not None:
            result.via = "ref"
            result.score = 1.0 if forms.ref == self.hint.ref else 0.0
            return result
        # a street hint says nothing about a ref, and a hint without rare grams says nothing at all
        if forms.ref is not None or not self.light.has_evidence():
            return result

        result.light = self.light.score(forms.light_bag)
        best, via = result.light, "light"
        if best is None or best < TAU:
            if self.aggressive.has_evidence():
                result.aggressive = self.aggressive.score(forms.aggressive_bag)
                if result.aggressive is not None:
                    discounted = BETA * result.aggressive
                    if best is None or discounted > best:
                        best, via = discounted, "aggressive"

            hint_core = self.hint.generics.core
            if len(hint_core) >= MIN_FUZZY_CORE and forms.generics.core:
                result.fuzzy = edit_similarity(hint_core, forms.generics.core)
                discounted = FUZZY_DISCOUNT * result.fuzzy
                if best is None or discounted > best:
                    best, via = discounted, "fuzzy"

        if best is None:
            return result
        hint_forms = self.hint.generics.forms
        name_forms = forms.generics.forms
        if hint_forms and name_forms and not (hint_forms & name_forms):
            result.generic_mismatch = True
            best *= GENERIC_MISMATCH
        result.score = best
        result.via = via
        return result


def score_pool(pool, hint):
    """Return (scorer, {name: NameScore}, [best NameScore or None per candidate, pool order])."""
    scorer = HintScorer(hint, pool.distinct_names)
    by_name = {}
    for name in pool.distinct_names:
        by_name[name] = scorer.score_name(name)

    per_candidate = []
    for candidate in pool.candidates:
        best = None
        for name in candidate.names:
            score = by_name[name]
            if score.score is not None and (best is None or score.score > best.score):
                best = score
        per_candidate.append(best)
    return scorer, by_name, per_candidate


def score_loose(hint_text, names):
    """Score loose names without a pool - for fixtures."""
    scorer = HintScorer(Hint(hint_text), names)
    scores = {}
    for name in names:
        scores[name] = scorer.score_name(name)
    return scores
