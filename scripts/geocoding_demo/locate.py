"""Demo-only: fetch a candidate pool from a Valhalla /locate endpoint.

This is the one module with no C++ counterpart. In the real implementation loki's bin search already
holds these candidates by the time the hint needs scoring; /locate is just a convenient way to get an
equivalent list out of a running server.
"""

import hashlib
import json
import os
import urllib.error
import urllib.request

from candidate import Candidate, CandidatePool

USER_AGENT = "valhalla-street-name-hint-demo/0.1"
TIMEOUT_SECONDS = 180


class LocateError(Exception):
    pass


def _request_body(lat, lon, radius, costing):
    # sort_keys so the cache key is stable
    return json.dumps(
        {
            "locations": [{"lat": lat, "lon": lon, "radius": radius}],
            "costing": costing,
            "verbose": True,
        },
        sort_keys=True,
    )


def _cache_key(url, body):
    return hashlib.sha256((url + "\n" + body).encode("utf-8")).hexdigest()[:16]


def _post(url, body):
    request = urllib.request.Request(
        url,
        data=body.encode("utf-8"),
        headers={"Content-Type": "application/json", "User-Agent": USER_AGENT},
    )
    try:
        with urllib.request.urlopen(request, timeout=TIMEOUT_SECONDS) as response:
            return response.read().decode("utf-8")
    except urllib.error.HTTPError as err:
        raise LocateError("{} returned HTTP {}: {}".format(url, err.code, err.read()[:400]))
    except urllib.error.URLError as err:
        raise LocateError("could not reach {}: {}".format(url, err.reason))


def fetch(lat, lon, radius, url, costing, cache_dir, offline):
    """Return (raw_response_list, came_from_cache)."""
    body = _request_body(lat, lon, radius, costing)
    path = None
    if cache_dir:
        path = os.path.join(cache_dir, _cache_key(url, body) + ".json")
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as handle:
                return json.load(handle), True

    if offline:
        raise LocateError(
            "--offline was given but no cached response exists for "
            "lat={} lon={} radius={} costing={} at {}\n"
            "run the same command once without --offline to populate the cache".format(
                lat, lon, radius, costing, url
            )
        )

    text = _post(url, body)
    parsed = json.loads(text)
    # valhalla reports errors as an object, successful /locate as an array of locations
    if not isinstance(parsed, list):
        raise LocateError("{} returned an error: {}".format(url, json.dumps(parsed)[:400]))

    if path:
        os.makedirs(cache_dir, exist_ok=True)
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(parsed, handle)

    return parsed, False


def _to_candidate(raw):
    candidate = Candidate()

    edge_id = raw["edge_id"]
    candidate.edge_id = edge_id["value"]
    candidate.level = edge_id["level"]
    candidate.tile_id = edge_id["tile_id"]
    candidate.index = edge_id["id"]

    edge_info = raw["edge_info"]
    candidate.way_id = edge_info["way_id"]
    candidate.names = list(edge_info.get("names") or [])

    # "classification" is an object holding classification/use/surface/link/internal
    edge_class = raw["edge"]["classification"]
    candidate.classification = edge_class["classification"]
    candidate.use = edge_class["use"]
    candidate.forward = bool(raw["edge"]["forward"])

    candidate.distance = raw["distance"]
    candidate.correlated_lat = raw["correlated_lat"]
    candidate.correlated_lon = raw["correlated_lon"]
    candidate.percent_along = raw["percent_along"]
    candidate.heading = raw.get("heading", 0.0)
    candidate.inbound_reach = raw.get("inbound_reach", 0)
    candidate.outbound_reach = raw.get("outbound_reach", 0)
    return candidate


def build_pool(raw_response, lat, lon, radius, from_cache):
    """Turn a /locate response into a CandidatePool, name-indexed.

    One candidate per directed edge, so a two-way road appears twice - which is what loki works on.
    """
    if not raw_response:
        raise LocateError("/locate returned no locations")

    edges = raw_response[0].get("edges") or []
    if not edges:
        raise LocateError("/locate found no edges - is the coordinate inside the tileset?")

    pool = CandidatePool()
    pool.lat = lat
    pool.lon = lon
    pool.radius = radius
    pool.from_cache = from_cache

    for raw in edges:
        pool.candidates.append(_to_candidate(raw))
    pool.candidates.sort(key=lambda c: (c.distance, c.edge_id))

    for position, candidate in enumerate(pool.candidates):
        if not candidate.is_named():
            pool.unnamed_count += 1
            continue
        for name in candidate.names:
            pool.name_to_candidates.setdefault(name, []).append(position)

    pool.distinct_names = sorted(pool.name_to_candidates)
    return pool


def load_pool(lat, lon, radius, url, costing, cache_dir, offline):
    raw, from_cache = fetch(lat, lon, radius, url, costing, cache_dir, offline)
    return build_pool(raw, lat, lon, radius, from_cache)
