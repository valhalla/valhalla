// -*- mode: c++ -*-
#ifndef MMP_GEOJSON_READER_H_
#define MMP_GEOJSON_READER_H_

#include <string>
#include <vector>

#include <rapidjson/document.h>

#include <valhalla/meili/measurement.h>


namespace valhalla {
namespace meili {

class SequenceParseError: public std::runtime_error {
  // Need its constructor
  using std::runtime_error::runtime_error;
};


class GeoJSONReader
{
 public:
  GeoJSONReader(float default_search_radius, float default_gps_accuracy);

  bool Read(const std::string& string, std::vector<std::vector<Measurement>>& sequences) const;

  float default_gps_accuracy() const
  { return default_gps_accuracy_; }

  float default_search_radius() const
  { return default_search_radius_; }

 protected:
  std::vector<Measurement> ReadGeometry(const rapidjson::Value& value) const;

  std::vector<Measurement> ReadFeature(const rapidjson::Value& value) const;

 private:
  float default_gps_accuracy_;

  float default_search_radius_;
};

}
}
#endif // MMP_GEOJSON_READER_H_
