# Add predicted_speeds to exports
try:
    from .predicted_speeds import (
        compress_speed_buckets,
        decompress_speed_bucket,
        encode_compressed_speeds,
        decode_compressed_speeds,
        BUCKETS_PER_WEEK,
        COEFFICIENT_COUNT,
        SPEED_BUCKET_SIZE_MINUTES,
        SPEED_BUCKET_SIZE_SECONDS,
    )

    __all__ = [
        "compress_speed_buckets",
        "decompress_speed_bucket",
        "encode_compressed_speeds",
        "decode_compressed_speeds",
        "BUCKETS_PER_WEEK",
        "COEFFICIENT_COUNT",
        "SPEED_BUCKET_SIZE_MINUTES",
        "SPEED_BUCKET_SIZE_SECONDS",
    ]
except ImportError:
    # Module not built yet
    __all__ = []
