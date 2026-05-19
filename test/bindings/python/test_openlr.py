# -*- coding: utf-8 -*-

import unittest
import valhalla.utils.openlr as olr

class TestOpenlr(unittest.TestCase):

    def setUp(self):
        self.lrp1 = olr.LocationReferencePoint(
            longitude=-73.9857,
            latitude=40.7484,
            bearing=90.0,
            frc=1,
            fow=olr.FormOfWay.MULTIPLE_CARRIAGEWAY,
            distance=527.4
        )

        self.lrp2 = olr.LocationReferencePoint(
            longitude=-73.9815,
            latitude=40.7527,
            bearing=45.0,
            frc=1,
            fow=olr.FormOfWay.MULTIPLE_CARRIAGEWAY,
            prev=self.lrp1,
            distance=0.0,
            lfrcnp=2,
        )

        self.openlr_obj = olr.OpenLr(
            lrps=[self.lrp1, self.lrp2],
            positive_offset_bucket=5,
            negative_offset_bucket=0,
            point_along_line=True,
            orientation=olr.Orientation.FirstLrpTowardsSecond,
            side_of_the_road=olr.SideOfTheRoad.RightSideOfRoad,
        )

    """Module import tests"""

    def test_module_imports_openlr(self):
        self.assertTrue(hasattr(olr, "OpenLr"))

    def test_module_imports_location_reference_point(self):
        self.assertTrue(hasattr(olr, "LocationReferencePoint"))

    def test_module_imports_form_of_way(self):
        self.assertTrue(hasattr(olr, "FormOfWay"))

    def test_module_imports_orientation(self):
        self.assertTrue(hasattr(olr, "Orientation"))

    def test_module_imports_side_of_the_road(self):
        self.assertTrue(hasattr(olr, "SideOfTheRoad"))

    """LocationReferencePoint tests"""

    def test_lrp1_longitude(self):
        self.assertAlmostEqual(self.lrp1.longitude, -73.9857, places=4)

    def test_lrp1_latitude(self):
        self.assertAlmostEqual(self.lrp1.latitude, 40.7484, places=4)

    def test_lrp1_distance(self):
        self.assertEqual(self.lrp1.distance, 527.4)

    def test_lrp2_lfrcnp(self):
        self.assertEqual(self.lrp2.lfrcnp, 2)

    def test_lrp_repr_contains_lon(self):
        self.assertIn("lon=", repr(self.lrp1))

    def test_lrp_repr_contains_lat(self):
        self.assertIn("lat=", repr(self.lrp1))

    """OpenLr property tests"""

    def test_openlr_positive_offset(self):
        self.assertEqual(self.openlr_obj.poff, 5)

    def test_openlr_negative_offset(self):
        self.assertEqual(self.openlr_obj.noff, 0)

    def test_openlr_point_along_line(self):
        self.assertTrue(self.openlr_obj.is_point_along_line)

    def test_openlr_orientation(self):
        self.assertEqual(
            self.openlr_obj.orientation,
            olr.Orientation.FirstLrpTowardsSecond
        )

    def test_openlr_side_of_the_road(self):
        self.assertEqual(
            self.openlr_obj.side_of_the_road,
            olr.SideOfTheRoad.RightSideOfRoad
        )

    """Coordinate and geometry tests"""

    def test_first_coordinate_longitude(self):
        self.assertAlmostEqual(
            self.openlr_obj.first_coordinate.lng,
            -73.9857,
            places=4
        )

    def test_first_coordinate_latitude(self):
        self.assertAlmostEqual(
            self.openlr_obj.first_coordinate.lat,
            40.7484,
            places=4
        )

    def test_last_coordinate_longitude(self):
        self.assertAlmostEqual(
            self.openlr_obj.last_coordinate.lng,
            -73.9815,
            places=4
        )

    def test_last_coordinate_latitude(self):
        self.assertAlmostEqual(
            self.openlr_obj.last_coordinate.lat,
            40.7527,
            places=4
        )

    def test_openlr_length_positive(self):
        self.assertGreater(self.openlr_obj.length, 0.0)

    def test_pointll_is_valid(self):
        self.assertEqual(
            self.openlr_obj.first_coordinate.is_valid(),
            True
        )

    """Serialization tests"""

    def test_to_binary_returns_bytes(self):
        binary_data = self.openlr_obj.to_binary()
        self.assertIsInstance(binary_data, (bytes, bytearray))

    def test_to_base64_returns_string(self):
        base64_data = self.openlr_obj.to_base64()
        self.assertIsInstance(base64_data, str)

    """Base64 reconstruction tests"""

    def test_from_base64_preserves_offsets(self):
        reconstructed = olr.OpenLr.from_base64(
            self.openlr_obj.to_base64()
        )

        self.assertEqual(reconstructed.poff, self.openlr_obj.poff)
        self.assertEqual(reconstructed.noff, self.openlr_obj.noff)

    def test_from_base64_preserves_coordinates(self):
        reconstructed = olr.OpenLr.from_base64(
            self.openlr_obj.to_base64()
        )

        self.assertEqual(
            reconstructed.first_coordinate,
            self.openlr_obj.first_coordinate
        )

        self.assertEqual(
            reconstructed.last_coordinate,
            self.openlr_obj.last_coordinate
        )

    def test_from_base64_preserves_orientation(self):
        reconstructed = olr.OpenLr.from_base64(
            self.openlr_obj.to_base64()
        )

        self.assertEqual(
            reconstructed.orientation,
            self.openlr_obj.orientation
        )

    def test_from_base64_preserves_side_of_the_road(self):
        reconstructed = olr.OpenLr.from_base64(
            self.openlr_obj.to_base64()
        )

        self.assertEqual(
            reconstructed.side_of_the_road,
            self.openlr_obj.side_of_the_road
        )

    """Binary reconstruction tests"""

    def test_from_binary_preserves_offsets(self):
        reconstructed = olr.OpenLr.from_binary(
            self.openlr_obj.to_binary()
        )

        self.assertEqual(reconstructed.poff, self.openlr_obj.poff)
        self.assertEqual(reconstructed.noff, self.openlr_obj.noff)

    def test_from_binary_preserves_coordinates(self):
        reconstructed = olr.OpenLr.from_binary(
            self.openlr_obj.to_binary()
        )

        self.assertEqual(
            reconstructed.first_coordinate,
            self.openlr_obj.first_coordinate
        )

        self.assertEqual(
            reconstructed.last_coordinate,
            self.openlr_obj.last_coordinate
        )

    def test_from_binary_preserves_orientation(self):
        reconstructed = olr.OpenLr.from_binary(
            self.openlr_obj.to_binary()
        )

        self.assertEqual(
            reconstructed.orientation,
            self.openlr_obj.orientation
        )

    def test_from_binary_preserves_side_of_the_road(self):
        reconstructed = olr.OpenLr.from_binary(
            self.openlr_obj.to_binary()
        )

        self.assertEqual(
            reconstructed.side_of_the_road,
            self.openlr_obj.side_of_the_road
        )

    """Representation tests"""

    def test_openlr_repr_contains_class_name(self):
        self.assertIn("OpenLr(", repr(self.openlr_obj))


if __name__ == "__main__":
    unittest.main()