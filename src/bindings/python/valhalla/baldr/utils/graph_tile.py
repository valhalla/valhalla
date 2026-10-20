"""Python wrapper for tile-id helpers bound in C++.

``get_tile_ids_from_bbox`` keeps the historical composite signature and loops
over the requested levels in Python, calling the single-level
``TileHierarchy.GetGraphIds(bbox, level)`` C++ binding per level.
"""

from collections.abc import Sequence
from typing import Union

from ..._valhalla import AABB2, TileHierarchy

__all__ = ["get_tile_ids_from_bbox"]

_DEFAULT_LEVELS: tuple = (0, 1, 2)


def get_tile_ids_from_bbox(
    minx: float,
    miny: float,
    maxx: float,
    maxy: float,
    levels: Union[Sequence[int], None] = None,
) -> list:
    """Returns all tile GraphIds for the specified levels (default: all),
    which intersect the bbox.

    :param minx: The bbox's minimum longitude.
    :param miny: The bbox's minimum latitude.
    :param maxx: The bbox's maximum longitude.
    :param maxy: The bbox's maximum latitude.
    :param levels: The hierarchy levels for which to find tiles.
    :returns: The list of tile GraphIds which intersect the bounding box
    :raises ValueError: When the level(s) or coord are invalid.
    """
    if minx < -180.0 or maxx > 180.0 or miny < -90.0 or maxy > 90.0:
        raise ValueError("Invalid coordinate, remember it's (lon, lat)")

    if levels is None or len(levels) == 0:
        requested_levels = _DEFAULT_LEVELS
    else:
        requested_levels = tuple(levels)

    bbox = AABB2(minx, miny, maxx, maxy)

    tile_ids: list = []
    for level in requested_levels:
        if isinstance(level, bool) or not isinstance(level, int):
            raise ValueError(f"Invalid hierarchy level: {level!r}")
        # range check happens in the C++ binding (check_level -> ValueError),
        # but guard the uint32_t boundary here so huge ints fail cleanly
        if level < 0 or level > 0xFFFFFFFF:
            raise ValueError(f"Invalid hierarchy level: {level!r}")
        tile_ids.extend(TileHierarchy.GetGraphIds(bbox, level))

    return tile_ids
