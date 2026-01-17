"""Valhalla utilities: graph access and predicted speeds compression."""

from .graph_utils import (
    GraphId,
    GraphUtils,
    get_tile_base_lon_lat,
    get_tile_id_from_lon_lat,
    get_tile_ids_from_bbox,
)
from ._predicted_speeds import (
    BUCKETS_PER_WEEK,
    COEFFICIENT_COUNT,
    SPEED_BUCKET_SIZE_MINUTES,
    SPEED_BUCKET_SIZE_SECONDS,
    compress_speed_buckets,
    decode_compressed_speeds,
    decompress_speed_bucket,
    encode_compressed_speeds,
)

__all__ = [
    # graph_utils
    "GraphId",
    "GraphUtils",
    "get_tile_base_lon_lat",
    "get_tile_id_from_lon_lat",
    "get_tile_ids_from_bbox",
    # predicted_speeds
    "compress_speed_buckets",
    "decompress_speed_bucket",
    "encode_compressed_speeds",
    "decode_compressed_speeds",
    "BUCKETS_PER_WEEK",
    "COEFFICIENT_COUNT",
    "SPEED_BUCKET_SIZE_MINUTES",
    "SPEED_BUCKET_SIZE_SECONDS",
]
