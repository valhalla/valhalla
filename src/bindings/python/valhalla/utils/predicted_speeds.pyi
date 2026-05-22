"""Valhalla DCT-2 speed compression utilities"""

from collections.abc import Sequence
from typing import Annotated

import numpy
from numpy.typing import NDArray

BUCKETS_PER_WEEK: int = 2016

COEFFICIENT_COUNT: int = 200

SPEED_BUCKET_SIZE_MINUTES: int = 5

SPEED_BUCKET_SIZE_SECONDS: int = 300

def compress_speed_buckets(
    speeds: Annotated[NDArray[numpy.float32], dict(shape=(2016))],
) -> Annotated[NDArray[numpy.int16], dict(shape=(200))]:
    """
    Compress 2016 speed buckets into 200 DCT-II coefficients.

    :param speeds: NumPy array of 2016 float values (one per 5-minute bucket)
    :returns: NumPy array of 200 int16 coefficients
    """

def decompress_speed_bucket(coefficients: Sequence[int], bucket_idx: int) -> float:
    """
    Decompress a single speed bucket using DCT-III.

    :param coefficients: Array of 200 int16 coefficients
    :param bucket_idx: Bucket index (0 to 2015)
    :returns: Speed in KPH for the specified bucket
    :raises ValueError: If bucket_idx is out of range
    """

def encode_compressed_speeds(coefficients: Sequence[int]) -> str:
    """
    Encode 200 coefficients as base64 string.

    :param coefficients: Array of 200 int16 coefficients
    :returns: Base64-encoded string
    """

def decode_compressed_speeds(encoded: str) -> Annotated[NDArray[numpy.int16], dict(shape=(200))]:
    """
    Decode base64 string to 200 coefficients.

    :param encoded: Base64-encoded string (536 characters)
    :returns: Array of 200 int16 coefficients
    :raises RuntimeError: If decoded size is incorrect
    """
