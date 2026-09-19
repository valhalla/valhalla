"""Structs passed between the demo stages.

Deliberately plain classes with explicit fields: each one stands in for a C++ struct, so no
dataclasses, no dicts-as-records, no attribute magic.
"""


class Candidate:
    """One directed edge, as loki hands it to the scorer.

    A two-way road therefore appears twice, once per direction, exactly as in a PathEdge list.
    """

    __slots__ = (
        "edge_id",
        "level",
        "tile_id",
        "index",
        "way_id",
        "names",
        "distance",
        "correlated_lat",
        "correlated_lon",
        "percent_along",
        "classification",
        "use",
        "forward",
        "heading",
        "inbound_reach",
        "outbound_reach",
    )

    def __init__(self):
        self.edge_id = 0
        self.level = 0
        self.tile_id = 0
        self.index = 0
        self.way_id = 0
        self.names = []
        self.distance = 0.0
        self.correlated_lat = 0.0
        self.correlated_lon = 0.0
        self.percent_along = 0.0
        self.classification = ""
        self.use = ""
        self.forward = True
        self.heading = 0.0
        self.inbound_reach = 0
        self.outbound_reach = 0

    def graph_id_str(self):
        return "{}/{}/{}".format(self.level, self.tile_id, self.index)

    def is_named(self):
        return len(self.names) > 0


class CandidatePool:
    """The whole candidate set for one location, plus the name index the scorer works over.

    Scoring runs over `distinct_names`, not over candidates: in the real data a single street is
    spread over dozens of edges, and counting each of them as a document would demote that street's
    own trigrams to "common, ignore" purely because OSM split the way into segments.
    """

    __slots__ = (
        "lat",
        "lon",
        "radius",
        "candidates",
        "distinct_names",
        "name_to_candidates",
        "unnamed_count",
        "from_cache",
    )

    def __init__(self):
        self.lat = 0.0
        self.lon = 0.0
        self.radius = 0
        self.candidates = []
        self.distinct_names = []
        self.name_to_candidates = {}
        self.unnamed_count = 0
        self.from_cache = False

    def max_distance(self):
        if not self.candidates:
            return 0.0
        return max(c.distance for c in self.candidates)
