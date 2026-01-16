import numpy as np
from numpy.typing import NDArray

BUCKETS_PER_WEEK: int
COEFFICIENT_COUNT: int
SPEED_BUCKET_SIZE_MINUTES: int
SPEED_BUCKET_SIZE_SECONDS: int

def compress_speed_buckets(speeds: NDArray[np.float32]) -> NDArray[np.int16]:
    """Compress 2016 speed buckets into 200 DCT-II coefficients.

    Args:
        speeds: NumPy array of 2016 float values (one per 5-minute bucket)

    Returns:
        NumPy array of 200 int16 coefficients
    """

def decompress_speed_bucket(coefficients: NDArray[np.int16], bucket_idx: int) -> float:
    """Decompress a single speed bucket using DCT-III.

    Args:
        coefficients: Array of 200 int16 coefficients
        bucket_idx: Bucket index (0 to 2015)

    Returns:
        Speed in KPH for the specified bucket

    Raises:
        ValueError: If bucket_idx is out of range
    """

def encode_compressed_speeds(coefficients: NDArray[np.int16]) -> str:
    """Encode 200 coefficients as base64 string.

    Args:
        coefficients: Array of 200 int16 coefficients

    Returns:
        Base64-encoded string
    """

def decode_compressed_speeds(encoded: str) -> NDArray[np.int16]:
    """Decode base64 string to 200 coefficients.

    Args:
        encoded: Base64-encoded string (400 characters)

    Returns:
        Array of 200 int16 coefficients

    Raises:
        RuntimeError: If decoded size is incorrect
    """
