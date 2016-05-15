// -*- mode: c++ -*-
#undef NDEBUG

#include <iostream>
#include <cassert>

#include <valhalla/midgard/pointll.h>

#include "mmp/measurement.h"
#include "mmp/geojson_reader.h"

using namespace valhalla;

void TestGeoJSONReader()
{
  mmp::GeoJSONReader reader(10, 40);
  assert(reader.default_gps_accuracy() == 10);
  assert(reader.default_search_radius() == 40);

  // It should read multipoint geometry
  {
    std::vector<std::vector<mmp::Measurement>> sequences;
    bool is_collection = reader.Read("{\"type\": \"MultiPoint\", \"coordinates\": [[1,2], [3,4]]}", sequences);
    assert(!is_collection);
    const auto& measurements = sequences.front();
    assert(measurements.size() == 2);
    assert(measurements[0].lnglat() == midgard::PointLL(1,2));
    assert(measurements[1].lnglat() == midgard::PointLL(3,4));
    for (const auto& measurement: measurements) {
      assert(measurement.gps_accuracy() == reader.default_gps_accuracy());
      assert(measurement.search_radius() == reader.default_search_radius());
    }
  }

  // It should read linestring geometry
  {
    std::vector<std::vector<mmp::Measurement>> sequences;
    bool is_collection = reader.Read("{\"type\": \"LineString\", \"coordinates\": [[1,2], [3,4]]}", sequences);
    assert(!is_collection);
    assert(sequences.size() == 1);
    const auto& measurements = sequences.front();
    assert(measurements.size() == 2);
    assert(measurements[0].lnglat() == midgard::PointLL(1,2));
    assert(measurements[1].lnglat() == midgard::PointLL(3,4));
    for (const auto& measurement: measurements) {
      assert(measurement.gps_accuracy() == reader.default_gps_accuracy());
      assert(measurement.search_radius() == reader.default_search_radius());
    }
  }

  // It should read geometry collection
  {
    std::vector<std::vector<mmp::Measurement>> sequences;
    bool is_collection = reader.Read("{\"type\": \"GeometryCollection\", \"geometries\": [{\"type\": \"LineString\", \"coordinates\": [[1,2], [3,4]]}, {\"type\": \"MultiPoint\", \"coordinates\": [[2,3], [4,5]]}]}", sequences);
    assert(is_collection);
    assert(sequences.size() == 2);
    assert(sequences[0].size() == 2);
    assert(sequences[0][0].lnglat() == midgard::PointLL(1,2));
    assert(sequences[0][1].lnglat() == midgard::PointLL(3,4));
    assert(sequences[1].size() == 2);
    assert(sequences[1][0].lnglat() == midgard::PointLL(2,3));
    assert(sequences[1][1].lnglat() == midgard::PointLL(4,5));
    for (const auto& sequence: sequences) {
      for (const auto& measurement: sequence) {
        assert(measurement.gps_accuracy() == reader.default_gps_accuracy());
        assert(measurement.search_radius() == reader.default_search_radius());
      }
    }
  }

  // It should read feature
  {
    std::vector<std::vector<mmp::Measurement>> sequences;
    bool is_collection = reader.Read("{\"type\": \"Feature\", \"properties\":{\"gps_accuracy\": 5, \"search_radius\": [20]}, \"geometry\": {\"type\": \"LineString\", \"coordinates\": [[1,2], [3,4]]}}", sequences);
    assert(!is_collection);
    assert(sequences.size() == 1);
    const auto& measurements = sequences.front();
    assert(measurements.size() == 2);
    assert(measurements[0].lnglat() == midgard::PointLL(1,2));
    assert(measurements[1].lnglat() == midgard::PointLL(3,4));
    assert(measurements[0].gps_accuracy() == 5);
    assert(measurements[1].gps_accuracy() == 5);
    assert(measurements[0].search_radius() == 20);
    assert(measurements[1].search_radius() == reader.default_search_radius());
  }

  // It should read feature collection
  {
    std::vector<std::vector<mmp::Measurement>> sequences;
    bool is_collection = reader.Read("{\"type\": \"FeatureCollection\", \"features\": [{\"type\": \"Feature\", \"properties\":{\"gps_accuracy\": 5, \"search_radius\": [20]}, \"geometry\": {\"type\": \"LineString\", \"coordinates\": [[1,2], [3,4]]}}, {\"type\": \"Feature\", \"properties\":{\"gps_accuracy\": [5, 6, 7], \"search_radius\": [20, 21, 22]}, \"geometry\": {\"type\": \"LineString\", \"coordinates\": [[2,3], [4,5]]}}]}", sequences);
    assert(is_collection);
    assert(sequences.size() == 2);
    assert(sequences[0].size() == 2);
    assert(sequences[0][0].lnglat() == midgard::PointLL(1,2));
    assert(sequences[0][1].lnglat() == midgard::PointLL(3,4));
    assert(sequences[1].size() == 2);
    assert(sequences[1][0].lnglat() == midgard::PointLL(2,3));
    assert(sequences[1][1].lnglat() == midgard::PointLL(4,5));
  }

  // It should throw parse error
  {
    bool error = false;
    std::vector<std::vector<mmp::Measurement>> sequences;
    try {
      bool is_collection = reader.Read("", sequences);
    } catch (const mmp::SequenceParseError& ex) {
      error = true;
    }
    assert(error);

    error = false;
    try {
      bool is_collection = reader.Read("hello my friend", sequences);
    } catch (const mmp::SequenceParseError& ex) {
      error = true;
    }
    assert(error);

    error = false;
    try {
      bool is_collection = reader.Read("[1,2,3]", sequences);
    } catch (const mmp::SequenceParseError& ex) {
      error = true;
    }
    assert(error);

    error = false;
    try {
      bool is_collection = reader.Read("{\"type\": \"LineString\"}", sequences);
    } catch (const mmp::SequenceParseError& ex) {
      error = true;
    }
    assert(sequences.empty());
    assert(error);

    error = false;
    try {
      bool is_collection = reader.Read("{\"type\": \"LineString\"}", sequences);
    } catch (const mmp::SequenceParseError& ex) {
      error = true;
    }
    assert(sequences.empty());
    assert(error);

    error = false;
    try {
      bool is_collection = reader.Read("{\"type\": \"LineString\", \"coordinates\":[1]}", sequences);
    } catch (const mmp::SequenceParseError& ex) {
      error = true;
    }
    assert(sequences.empty());
    assert(error);
  }
}


int main(int argc, char *argv[])
{
  TestGeoJSONReader();

  std::cout << "all tests passed" << std::endl;
  return 0;
}
