#include "baldr/json.h"
#include "skadi/sample.h"
#include "tyr/serializers.h"

#include <sstream>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace {

void serialize_range_height(rapidjson::writer_wrapper_t& writer,
                            const std::vector<double>& ranges,
                            const std::vector<double>& heights,
                            const uint32_t precision,
                            const double no_data_value) {
  writer.start_array("range_height");
  writer.set_precision(precision);
  // for each posting
  auto range = ranges.cbegin();

  for (const auto height : heights) {
    writer.start_array();
    if (height == no_data_value) {
      writer(nullptr);
    } else {
      writer(height);
    }
    writer.end_array();
    ++range;
  }
  writer.end_array();
}

void serialize_height(rapidjson::writer_wrapper_t& writer,
                      const std::vector<double>& heights,
                      const uint32_t precision,
                      const double no_data_value) {
  writer.start_array("height");
  writer.set_precision(precision);
  for (const auto height : heights) {
    // add all heights's to an array
    if (height == no_data_value) {
      writer(nullptr);
    } else {
      writer(height);
    }
  }
  writer.end_array();
}

void serialize_shape(rapidjson::writer_wrapper_t& writer,
                     const google::protobuf::RepeatedPtrField<valhalla::Location>& shape) {
  writer.start_array("shape");
  writer.set_precision(6);
  for (const auto& p : shape) {
    writer.start_object();
    writer("lon", p.ll().lng());
    writer("lat", p.ll().lat());
    writer.end_object();
  }
  writer.end_array();
}

} // namespace

namespace valhalla {
namespace tyr {

/* example height with range response:
{
  "shape": [ {"lat": 40.712433, "lon": -76.504913}, {"lat": 40.712276, "lon": -76.605263} ],
  "range_height": [ [0,303], [8467,275], [25380,198] ]
}
*/
std::string serializeHeight(const Api& request,
                            const std::vector<double>& heights,
                            const std::vector<double>& ranges) {
  rapidjson::writer_wrapper_t writer(4096);
  writer.start_object();

  // get the precision to use for returned heights
  uint32_t precision = request.options().height_precision();

  // get the distances between the postings
  if (ranges.size()) {
    serialize_range_height(writer, ranges, heights, precision, skadi::get_no_data_value());
  } // just the postings
  else {
    serialize_height(writer, heights, precision, skadi::get_no_data_value());
  }
  // send back the shape as well
  if (request.options().has_encoded_polyline_case()) {
    writer("encoded_polyline", request.options().encoded_polyline());
  } else {
    serialize_shape(writer, request.options().shape());
  }
  if (request.options().has_id_case()) {
    writer("id", request.options().id());
  }

  // add warnings to json response
  if (request.info().warnings_size() >= 1) {
    serializeWarnings(request, writer);
  }
  writer.end_object();
  return writer.get_buffer();
}
} // namespace tyr
} // namespace valhalla
