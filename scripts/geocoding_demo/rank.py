"""Stage 5: the hook in search.cc - hint scores in, the PathEdges loki hands thor out.

A hint that clears TAU anywhere within HINT_RADIUS is a soft filter: matching edges become `edges`,
their distance replaced by d * (1 - ALPHA * score); what loki would have picked without the hint
becomes `filtered_edges`, the fallback thor already retries when `edges` yield no route. When nothing
clears TAU the result is exactly today's.
"""

from locate import restrict_pool
from score import TAU, score_pool

ALPHA = 0.8
HINT_RADIUS = 100  # metres; also the cap on how far a hint can move the correlation
TIE = 0.01  # metres; every edge at a snapped node shares one distance

# thor/worker.cc: kDistanceScale, kMaxDistances, and the filtered-edge penalty
DISTANCE_SCALE = 10.0
FILTERED_PENALTY = 3600.0
MAX_SCORE = {
    "auto": 43200.0,
    "truck": 43200.0,
    "bus": 43200.0,
    "taxi": 43200.0,
    "motor_scooter": 14400.0,
    "motorcycle": 14400.0,
    "bicycle": 7200.0,
    "pedestrian": 7200.0,
}


class Ranked:
    """One PathEdge as thor receives it."""

    __slots__ = ("candidate", "name_score", "effective", "thor_score")

    def __init__(self, candidate, name_score):
        self.candidate = candidate
        self.name_score = name_score
        self.effective = candidate.distance
        self.thor_score = 0.0


class Ranking:
    __slots__ = ("hint_used", "near", "best", "edges", "filtered", "baseline")

    def __init__(self):
        self.hint_used = False
        # the pool within HINT_RADIUS, which is also the IDF corpus
        self.near = None
        self.best = None
        self.edges = []
        self.filtered = []
        # what loki picks without a hint, for comparison
        self.baseline = []


def select(entries, radius):
    """Loki's rule: every entry inside the radius, or failing that the closest one and its ties."""
    inside = []
    for entry in entries:
        if entry.effective < radius:
            inside.append(entry)
    if inside:
        return inside

    closest = None
    for entry in entries:
        if closest is None or entry.effective < closest:
            closest = entry.effective
    ties = []
    for entry in entries:
        if entry.effective - closest <= TIE:
            ties.append(entry)
    return ties


def _apply_thor_scores(edges, filtered, costing):
    """thor_worker_t: penalize filtered edges, square and scale, subtract the minimum, cap."""
    worst = 0.0
    for entry in edges:
        worst = max(worst, entry.effective)

    lowest = None
    for group, penalty in ((edges, 0.0), (filtered, FILTERED_PENALTY + worst)):
        for entry in group:
            distance = entry.effective + penalty
            entry.thor_score = DISTANCE_SCALE * distance * distance
            if lowest is None or entry.thor_score < lowest:
                lowest = entry.thor_score

    cap = MAX_SCORE.get(costing, MAX_SCORE["auto"])
    for entry in edges + filtered:
        entry.thor_score = min(entry.thor_score - lowest, cap)


def rank(pool, hint, location_radius=0, hint_radius=HINT_RADIUS, costing="auto"):
    ranking = Ranking()

    everything = []
    for candidate in pool.candidates:
        everything.append(Ranked(candidate, None))
    ranking.baseline = select(everything, location_radius)
    _apply_thor_scores(ranking.baseline, [], costing)

    ranking.near = restrict_pool(pool, hint_radius)
    _, _, per_candidate = score_pool(ranking.near, hint)
    matched = []
    for candidate, name_score in zip(ranking.near.candidates, per_candidate):
        if name_score is None:
            continue
        if ranking.best is None or name_score.score > ranking.best.score:
            ranking.best = name_score
        if name_score.score >= TAU:
            entry = Ranked(candidate, name_score)
            entry.effective = candidate.distance * (1.0 - ALPHA * name_score.score)
            matched.append(entry)

    if not matched:
        ranking.edges = ranking.baseline
        return ranking

    ranking.hint_used = True
    ranking.edges = select(matched, location_radius)
    kept = set()
    for entry in ranking.edges:
        kept.add(entry.candidate.edge_id)
    for entry in ranking.baseline:
        if entry.candidate.edge_id not in kept:
            ranking.filtered.append(Ranked(entry.candidate, None))
    _apply_thor_scores(ranking.edges, ranking.filtered, costing)
    return ranking
