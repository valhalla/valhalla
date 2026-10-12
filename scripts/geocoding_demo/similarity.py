"""IDF-weighted trigram containment - Design B from geocoding_design_notes.md.

Grams are taken over codepoints of `Normalized.stripped`, so they ignore word boundaries: that is
what makes compounding and word order irrelevant. In C++ the normalized form is UTF-8, so the window
has to step over codepoints, not bytes, for anything outside the fold table.
"""

import math

GRAM_SIZE = 3


def trigrams(text):
    """Return the gram bag as {gram: count}. Empty for text shorter than one gram."""
    bag = {}
    for start in range(len(text) - GRAM_SIZE + 1):
        gram = text[start : start + GRAM_SIZE]
        bag[gram] = bag.get(gram, 0) + 1
    return bag


class GramIndex:
    """Document frequencies over the names being compared, one document per distinct name.

    Built per pass: the aggressive pass collapses digraphs, so its grams form their own corpus.
    """

    __slots__ = ("document_count", "document_frequency")

    def __init__(self, bags):
        self.document_count = len(bags)
        self.document_frequency = {}
        for bag in bags:
            for gram in bag:
                self.document_frequency[gram] = self.document_frequency.get(gram, 0) + 1

    def weight(self, gram):
        # a hint gram no nearby name contains is as rare as it gets; dropping it instead would let
        # `mozartstr` score 1.0 against every `-strasse` around it
        frequency = max(self.document_frequency.get(gram, 0), 1)
        return math.log(self.document_count / frequency)


def containment(hint_bag, candidate_bag, index):
    """Share of the hint's gram weight that the candidate covers, or None with no weight to share.

    Divided by the hint, not the smaller bag: hint grams missing from the candidate are evidence
    against it, candidate grams missing from the hint are not (abbreviations, dropped words).
    """
    total = 0.0
    shared = 0.0
    for gram, count in hint_bag.items():
        weight = index.weight(gram)
        total += weight * count
        shared += weight * min(count, candidate_bag.get(gram, 0))
    if total <= 0.0:
        return None
    return shared / total


def bag_weight(bag, index):
    total = 0.0
    for gram, count in bag.items():
        total += index.weight(gram) * count
    return total


def uncovered_share(hint_bag, candidate_bag, index):
    """Share of the candidate's gram weight the hint does not cover - the weak, surplus evidence."""
    total = 0.0
    uncovered = 0.0
    for gram, count in candidate_bag.items():
        weight = index.weight(gram)
        total += weight * count
        uncovered += weight * max(0, count - hint_bag.get(gram, 0))
    if total <= 0.0:
        return 0.0
    return uncovered / total


def osa_distance(a, b):
    """Edit distance counting an adjacent transposition as one edit (optimal string alignment)."""
    rows = len(a) + 1
    cols = len(b) + 1
    table = [[0] * cols for _ in range(rows)]
    for i in range(rows):
        table[i][0] = i
    for j in range(cols):
        table[0][j] = j
    for i in range(1, rows):
        for j in range(1, cols):
            cost = 0 if a[i - 1] == b[j - 1] else 1
            best = min(table[i - 1][j] + 1, table[i][j - 1] + 1, table[i - 1][j - 1] + cost)
            if i > 1 and j > 1 and a[i - 1] == b[j - 2] and a[i - 2] == b[j - 1]:
                best = min(best, table[i - 2][j - 2] + 1)
            table[i][j] = best
    return table[rows - 1][cols - 1]


def edit_similarity(a, b):
    longest = max(len(a), len(b))
    if longest == 0:
        return None
    return 1.0 - osa_distance(a, b) / longest


def score_names(hint, names, normalize):
    """Score each name against the hint under one normalization pass.

    Returns (index, hint_bag, {name: score}). The corpus is the distinct normalized forms, so
    `Hermannstrasse` and `Hermann Strasse` in one pool count as one document, not two.
    """
    stripped_by_name = {}
    for name in names:
        stripped_by_name[name] = normalize(name).stripped

    corpus = []
    for stripped in set(stripped_by_name.values()):
        if len(stripped) >= GRAM_SIZE:
            corpus.append(trigrams(stripped))
    index = GramIndex(corpus)

    hint_bag = trigrams(normalize(hint).stripped)
    scores = {}
    for name, stripped in stripped_by_name.items():
        scores[name] = containment(hint_bag, trigrams(stripped), index)
    return index, hint_bag, scores


def plain_containment(hint_bag, candidate_bag):
    """Unweighted containment, kept only to show what IDF buys."""
    total = 0
    shared = 0
    for gram, count in hint_bag.items():
        total += count
        shared += min(count, candidate_bag.get(gram, 0))
    if total == 0:
        return None
    return shared / total
