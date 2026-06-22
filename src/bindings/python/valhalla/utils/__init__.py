"""Deprecated shim. ``valhalla.utils`` moved to ``valhalla.baldr`` / ``valhalla.baldr.utils``
and ``valhalla.midgard.utils``.

Re-exports the relocated symbols and warns on import. Will be removed in a future release.
"""

import warnings

from ..baldr import GraphId
from ..baldr.utils import (
    BUCKETS_PER_WEEK,
    COEFFICIENT_COUNT,
    SPEED_BUCKET_SIZE_MINUTES,
    SPEED_BUCKET_SIZE_SECONDS,
    GraphUtils,
    compress_speed_buckets,
    decode_compressed_speeds,
    decompress_speed_bucket,
    encode_compressed_speeds,
    get_tile_base_lon_lat,
    get_tile_id_from_lon_lat,
    get_tile_ids_from_bbox,
    get_tile_ids_from_ring,
)
from ..midgard.utils import decode_polyline

warnings.warn(
    "`valhalla.utils` is deprecated and will be removed in a future release: "
    "https://github.com/valhalla/valhalla/pull/6133",
    DeprecationWarning,
    stacklevel=2,
)

__all__ = [
    "GraphId",
    "GraphUtils",
    "get_tile_base_lon_lat",
    "get_tile_id_from_lon_lat",
    "get_tile_ids_from_bbox",
    "get_tile_ids_from_ring",
    "compress_speed_buckets",
    "decompress_speed_bucket",
    "encode_compressed_speeds",
    "decode_compressed_speeds",
    "BUCKETS_PER_WEEK",
    "COEFFICIENT_COUNT",
    "SPEED_BUCKET_SIZE_MINUTES",
    "SPEED_BUCKET_SIZE_SECONDS",
    "decode_polyline",
]
