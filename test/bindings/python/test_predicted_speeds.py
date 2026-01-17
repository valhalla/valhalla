# -*- coding: utf-8 -*-

import unittest

import numpy as np
from valhalla.utils.predicted_speeds import (
    BUCKETS_PER_WEEK,
    COEFFICIENT_COUNT,
    compress_speed_buckets,
    decode_compressed_speeds,
    decompress_speed_bucket,
    encode_compressed_speeds,
)


class TestPredictedSpeeds(unittest.TestCase):
    """Test DCT-2 compression/decompression functions."""

    def test_constants(self):
        """Verify module constants are correct."""
        self.assertEqual(BUCKETS_PER_WEEK, 2016)
        self.assertEqual(COEFFICIENT_COUNT, 200)

    def test_compress_shape(self):
        """Test compress_speed_buckets returns correct shape."""
        speeds = np.full(BUCKETS_PER_WEEK, 50.0, dtype=np.float32)
        coefficients = compress_speed_buckets(speeds)

        self.assertIsInstance(coefficients, np.ndarray)
        self.assertEqual(coefficients.shape, (COEFFICIENT_COUNT,))
        self.assertEqual(coefficients.dtype, np.int16)

    def test_compress_decompress_constant(self):
        """Test round-trip with constant speed (50 KPH)."""
        speeds = np.full(BUCKETS_PER_WEEK, 50.0, dtype=np.float32)
        coefficients = compress_speed_buckets(speeds)

        # Decompress all buckets
        for bucket_idx in range(BUCKETS_PER_WEEK):
            decompressed = decompress_speed_bucket(coefficients, bucket_idx)
            self.assertAlmostEqual(
                decompressed,
                50.0,
                delta=0.5,
                msg=f"Bucket {bucket_idx} decompression error",
            )

    def test_compress_decompress_sine_wave(self):
        """Test round-trip with sine wave pattern (like C++ test)."""
        # Generate synthetic speeds: 30 + 15 * sin(x/20)
        speeds = np.array(
            [round(30.0 + 15.0 * np.sin(i / 20.0)) for i in range(BUCKETS_PER_WEEK)],
            dtype=np.float32,
        )

        coefficients = compress_speed_buckets(speeds)

        # Calculate decompression errors
        errors = []
        for bucket_idx in range(BUCKETS_PER_WEEK):
            decompressed = decompress_speed_bucket(coefficients, bucket_idx)
            error = abs(decompressed - speeds[bucket_idx])
            errors.append(error)

        # Validate accuracy (same thresholds as C++ test)
        mean_error = np.mean(errors)
        max_error = np.max(errors)

        self.assertLessEqual(
            mean_error, 1.0, f"Mean error {mean_error:.2f} KPH exceeds threshold"
        )
        self.assertLessEqual(
            max_error, 2.0, f"Max error {max_error:.2f} KPH exceeds threshold"
        )

    def test_encode_decode_round_trip(self):
        """Test encode/decode round-trip preserves coefficients."""
        # Create test coefficients (alternating positive/negative)
        coefficients_in = np.array(
            [(10 * i) if i % 2 == 0 else (-10 * i) for i in range(COEFFICIENT_COUNT)],
            dtype=np.int16,
        )

        # Encode and decode
        encoded = encode_compressed_speeds(coefficients_in)
        coefficients_out = decode_compressed_speeds(encoded)

        # Verify encoding format
        self.assertIsInstance(encoded, str)
        self.assertEqual(len(encoded), 536)  # Base64 of 200 * 2 bytes = 400 bytes → 536 chars

        # Verify coefficients match exactly
        np.testing.assert_array_equal(
            coefficients_in, coefficients_out, err_msg="Encode/decode round-trip failed"
        )

    def test_decompress_bucket_validation(self):
        """Test bucket_idx validation."""
        coefficients = np.zeros(COEFFICIENT_COUNT, dtype=np.int16)

        # Valid indices should work
        speed = decompress_speed_bucket(coefficients, 0)
        self.assertIsInstance(speed, float)

        speed = decompress_speed_bucket(coefficients, BUCKETS_PER_WEEK - 1)
        self.assertIsInstance(speed, float)

        # Invalid indices should raise
        with self.assertRaises(ValueError):
            decompress_speed_bucket(coefficients, BUCKETS_PER_WEEK)

        with self.assertRaises(ValueError):
            decompress_speed_bucket(coefficients, 10000)

    def test_decode_invalid_string(self):
        """Test decode_compressed_speeds validates input."""
        with self.assertRaises(RuntimeError):
            decode_compressed_speeds("invalid_base64")

        with self.assertRaises(RuntimeError):
            decode_compressed_speeds("AA==")  # Too short

    def test_compress_wrong_size(self):
        """Test compress_speed_buckets validates array size."""
        # Too few elements (nanobind validates this as TypeError)
        with self.assertRaises((ValueError, RuntimeError, TypeError)):
            compress_speed_buckets(np.array([50.0] * 100, dtype=np.float32))

        # Too many elements (nanobind validates this as TypeError)
        with self.assertRaises((ValueError, RuntimeError, TypeError)):
            compress_speed_buckets(np.array([50.0] * 3000, dtype=np.float32))


if __name__ == "__main__":
    unittest.main()
