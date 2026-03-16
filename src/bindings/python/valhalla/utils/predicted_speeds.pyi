import numpy as np
from numpy.typing import NDArray

BUCKETS_PER_WEEK: int
COEFFICIENT_COUNT: int
SPEED_BUCKET_SIZE_MINUTES: int
SPEED_BUCKET_SIZE_SECONDS: int

def compress_speed_buckets(speeds: NDArray[np.float32]) -> NDArray[np.int16]:
    """Compress 2016 speed buckets into 200 DCT-II coefficients.

    :param speeds: NumPy array of 2016 float values (one per 5-minute bucket)
    :returns: NumPy array of 200 int16 coefficients
    """

def decompress_speed_bucket(coefficients: NDArray[np.int16], bucket_idx: int) -> float:
    """Decompress a single speed bucket using DCT-III.

    :param coefficients: Array of 200 int16 coefficients
    :param bucket_idx: Bucket index (0 to 2015)
    :returns: Speed in KPH for the specified bucket
    :raises ValueError: If bucket_idx is out of range
    """

def encode_compressed_speeds(coefficients: NDArray[np.int16]) -> str:
    """Encode 200 coefficients as base64 string.

    :param coefficients: Array of 200 int16 coefficients
    :returns: Base64-encoded string
    """

def decode_compressed_speeds(encoded: str) -> NDArray[np.int16]:
    """Decode base64 string to 200 coefficients.

    :param encoded: Base64-encoded string (536 characters)
    :returns: Array of 200 int16 coefficients
    :raises RuntimeError: If decoded size is incorrect
    """
