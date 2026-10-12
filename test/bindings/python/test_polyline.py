# -*- coding: utf-8 -*-

import unittest

from valhalla.midgard.utils import decode_polyline

# [(lng, lat)] = [(5.03231, 52.08813), (5.14913, 52.09987)]
POLYLINE6 = "csejbBkvcrHw|UgdcF"
POLYLINE5 = "ym||H}zu]khAcyU"


class TestDecodePolyline(unittest.TestCase):
    def assertCoordsAlmostEqual(self, actual, expected):
        self.assertEqual(len(actual), len(expected))
        for (ax, ay), (ex, ey) in zip(actual, expected):
            self.assertAlmostEqual(ax, ex, places=5)
            self.assertAlmostEqual(ay, ey, places=5)

    def test_default_precision_and_order(self):
        self.assertCoordsAlmostEqual(
            decode_polyline(POLYLINE6), [(5.03231, 52.08813), (5.14913, 52.09987)]
        )

    def test_latlng_order(self):
        self.assertCoordsAlmostEqual(
            decode_polyline(POLYLINE6, order="latlng"), [(52.08813, 5.03231), (52.09987, 5.14913)]
        )

    def test_precision(self):
        self.assertCoordsAlmostEqual(
            decode_polyline(POLYLINE5, precision=5), [(5.03231, 52.08813), (5.14913, 52.09987)]
        )
        # the same string decoded with the wrong precision lands somewhere else
        self.assertNotAlmostEqual(decode_polyline(POLYLINE5)[0][0], 5.03231, places=3)

    def test_empty(self):
        self.assertEqual(decode_polyline(""), [])


if __name__ == "__main__":
    unittest.main()
