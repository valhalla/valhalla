// -*- mode: c++ -*-

#ifndef MMP_GEOJSON_READER_H_
#define MMP_GEOJSON_READER_H_

#include <string>
#include <vector>

#include <mmp/measurement.h>

namespace mmp {

using namespace valhalla;

class SequenceParseError: public std::runtime_error {
  // Need its constructor
  using std::runtime_error::runtime_error;
};


class GeoJSONReader
{
 public:
  GeoJSONReader(float default_search_radius, float default_gps_accuracy);
  bool Read(const std::string& string, std::vector<std::vector<Measurement>>& sequences) const;

 private:
  float default_gps_accuracy_;
  float default_search_radius_;
};


}
#endif // MMP_GEOJSON_READER_H_
