// -*- mode: c++ -*-
#include <string>

#include <valhalla/midgard/pointll.h>

#include "meili/measurement.h"
#include "meili/geojson_reader.h"
#include "test.h"


using namespace valhalla;

void TestGeoJSONReader()
{
  meili::GeoJSONReader reader(10, 40);
  test::assert_bool(reader.default_gps_accuracy() == 10,
                    "gps should be read correctly");
  test::assert_bool(reader.default_search_radius() == 40,
                    "radius should be read correctly");

  // It should read multipoint geometry
  {
    std::vector<std::vector<meili::Measurement>> sequences;
    bool is_collection = reader.Read("{\"type\": \"MultiPoint\", \"coordinates\": [[1,2], [3,4]]}", sequences);
    test::assert_bool(!is_collection, "should not be collection");
    const auto& measurements = sequences.front();
    test::assert_bool(measurements.size() == 2,
                      "measurement size should be 2");
    test::assert_bool(measurements[0].lnglat() == midgard::PointLL(1,2),
                      "coordinates should be correct");
    test::assert_bool(measurements[1].lnglat() == midgard::PointLL(3,4),
                      "coordinates should be correct");
    for (const auto& measurement: measurements) {
      test::assert_bool(measurement.gps_accuracy() == reader.default_gps_accuracy(),
                        "gps should be correct");
      test::assert_bool(measurement.search_radius() == reader.default_search_radius(),
                        "radius should be correct");
    }
  }

  // It should read linestring geometry
  {
    std::vector<std::vector<meili::Measurement>> sequences;
    bool is_collection = reader.Read("{\"type\": \"LineString\", \"coordinates\": [[1,2], [3,4]]}", sequences);
    test::assert_bool(!is_collection,
                      "should nbot be collection");
    test::assert_bool(sequences.size() == 1,
                      "should be 1 sequence");
    const auto& measurements = sequences.front();
    test::assert_bool(measurements.size() == 2,
                      "should be 2 measurements");
    test::assert_bool(measurements[0].lnglat() == midgard::PointLL(1,2),
                      "coordinates should be correct");
    test::assert_bool(measurements[1].lnglat() == midgard::PointLL(3,4),
                      "coordinates should be correct");
    for (const auto& measurement: measurements) {
      test::assert_bool(measurement.gps_accuracy() == reader.default_gps_accuracy(),
                        "gps should be correct");
      test::assert_bool(measurement.search_radius() == reader.default_search_radius(),
                        "radius should be correct");
    }
  }

  // It should read geometry collection
  {
    std::vector<std::vector<meili::Measurement>> sequences;
    bool is_collection = reader.Read("{\"type\": \"GeometryCollection\", \"geometries\": [{\"type\": \"LineString\", \"coordinates\": [[1,2], [3,4]]}, {\"type\": \"MultiPoint\", \"coordinates\": [[2,3], [4,5]]}]}", sequences);
    test::assert_bool(is_collection,
                      "should be collection");
    test::assert_bool(sequences.size() == 2, "should be 2 sequences");
    test::assert_bool(sequences[0].size() == 2, "should be 2 measurements");
    test::assert_bool(sequences[0][0].lnglat() == midgard::PointLL(1,2),
                      "coordinates should be correct");
    test::assert_bool(sequences[0][1].lnglat() == midgard::PointLL(3,4),
                      "coordinates should be correct");
    test::assert_bool(sequences[1].size() == 2,
                      "should be 2 measurements");
    test::assert_bool(sequences[1][0].lnglat() == midgard::PointLL(2,3),
                      "coordinates should be correct");
    test::assert_bool(sequences[1][1].lnglat() == midgard::PointLL(4,5),
                      "coordinates should be correct");
    for (const auto& sequence: sequences) {
      for (const auto& measurement: sequence) {
        test::assert_bool(measurement.gps_accuracy() == reader.default_gps_accuracy(),
                          "gps should be correct");
        test::assert_bool(measurement.search_radius() == reader.default_search_radius(),
                          "radius should be correct");
      }
    }
  }

  // It should read feature
  {
    std::vector<std::vector<meili::Measurement>> sequences;
    bool is_collection = reader.Read("{\"type\": \"Feature\", \"properties\":{\"gps_accuracy\": 5, \"search_radius\": [20]}, \"geometry\": {\"type\": \"LineString\", \"coordinates\": [[1,2], [3,4]]}}", sequences);
    test::assert_bool(!is_collection,
                      "should not be collection");
    test::assert_bool(sequences.size() == 1,
                      "should be 1 sequence");
    const auto& measurements = sequences.front();
    test::assert_bool(measurements.size() == 2,
                      "shuold be 2 measurements");
    test::assert_bool(measurements[0].lnglat() == midgard::PointLL(1,2),
                      "coordinates should be correct");
    test::assert_bool(measurements[1].lnglat() == midgard::PointLL(3,4),
                      "coordinates should be correct");
    test::assert_bool(measurements[0].gps_accuracy() == 5,
                      "gps should be correct");
    test::assert_bool(measurements[1].gps_accuracy() == 5,
                      "gps should be correct");
    test::assert_bool(measurements[0].search_radius() == 20,
                      "search radius should be correct");
    test::assert_bool(measurements[1].search_radius() == reader.default_search_radius(),
                      "radius should be default");
  }

  // It should read feature collection
  {
    std::vector<std::vector<meili::Measurement>> sequences;
    bool is_collection = reader.Read("{\"type\": \"FeatureCollection\", \"features\": [{\"type\": \"Feature\", \"properties\":{\"gps_accuracy\": 5, \"search_radius\": [20]}, \"geometry\": {\"type\": \"LineString\", \"coordinates\": [[1,2], [3,4]]}}, {\"type\": \"Feature\", \"properties\":{\"gps_accuracy\": [5, 6, 7], \"search_radius\": [20, 21, 22]}, \"geometry\": {\"type\": \"LineString\", \"coordinates\": [[2,3], [4,5]]}}]}", sequences);
    test::assert_bool(is_collection, "should be collection");
    test::assert_bool(sequences.size() == 2, "should be 2 sequences");
    test::assert_bool(sequences[0].size() == 2, "should be 2 measurements");
    test::assert_bool(sequences[0][0].lnglat() == midgard::PointLL(1,2),
                      "coordinates should be correct");
    test::assert_bool(sequences[0][1].lnglat() == midgard::PointLL(3,4),
                      "coordinates should be correct");
    test::assert_bool(sequences[1].size() == 2,
                      "should be 2 measurements");
    test::assert_bool(sequences[1][0].lnglat() == midgard::PointLL(2,3),
                      "coordinates should be correct");
    test::assert_bool(sequences[1][1].lnglat() == midgard::PointLL(4,5),
                      "coordinates should be correct");
  }

  // It should throw parse error
  {
    std::vector<std::vector<meili::Measurement>> sequences;

    test::assert_throw<meili::SequenceParseError>([&reader, &sequences]() {
        reader.Read("", sequences);
        test::assert_bool(sequences.empty(), "sequences should be empty");
      }, "empty json can not be parsed");

    test::assert_throw<meili::SequenceParseError>([&reader, &sequences]() {
        reader.Read("hello my friend", sequences);
        test::assert_bool(sequences.empty(), "sequences should be empty");
      }, "your friend can not be parsed");

    test::assert_throw<meili::SequenceParseError>([&reader, &sequences]() {
        reader.Read("[1,2,3]", sequences);
        test::assert_bool(sequences.empty(), "sequences should be empty");
      }, "[1,2,3] is not a valid GeoJSON");

    test::assert_throw<meili::SequenceParseError>([&reader, &sequences]() {
        reader.Read("{\"type\": \"LineString\"}", sequences);
        test::assert_bool(sequences.empty(), "sequences should be empty");
      }, "{\"type\": \"LineString\"} is not a valid GeoJSON");

    test::assert_throw<meili::SequenceParseError>([&reader, &sequences]() {
        reader.Read("{\"type\": \"LineString\", \"coordinates\":[1]}", sequences);
        test::assert_bool(sequences.empty(), "sequences should be empty");
      }, "invalid coordinates");
  }
}


int main(int argc, char *argv[])
{
  test::suite suite("geojson reader");

  suite.test(TEST_CASE(TestGeoJSONReader));

  return suite.tear_down();
}
