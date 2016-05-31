// -*- mode: c++ -*-
#undef NDEBUG

#include <iostream>
#include <cassert>

#include <rapidjson/stringbuffer.h>

#include "meili/geojson_writer.h"


void TestGeoJSONWriter()
{
  using buffer_t = rapidjson::StringBuffer;

  // TODO: testing is not easy here as it needs mocking tile data
  meili::GeoJSONRouteWriter<buffer_t> route_writer;
  meili::GeoJSONMatchedPointsWriter<buffer_t> matched_points_writer;
}


int main(int argc, char *argv[])
{
  TestGeoJSONWriter();

  std::cout << "all tests passed" << std::endl;

  return 0;
}
