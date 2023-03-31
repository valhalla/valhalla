#include <sstream>

#include "baldr/json.h"
#include "skadi/sample.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace {

json::ArrayPtr serialize_range_height(const std::vector<double>& ranges,
                                      const std::vector<double>& heights,
                                      const uint32_t precision,
                                      const double no_data_value) {
  auto array = json::array({});
  // for each posting
  auto range = ranges.cbegin();

  for (const auto height : heights) {
    auto element = json::array({json::fixed_t{*range, 0}});
    if (height == no_data_value) {
      element->push_back(nullptr);
    } else {
      element->push_back({json::fixed_t{height, precision}});
    }
    array->push_back(element);
    ++range;
  }
  return array;
}

json::ArrayPtr serialize_height(const std::vector<double>& heights,
                                const uint32_t precision,
                                const double no_data_value) {
  auto array = json::array({});

  for (const auto height : heights) {
    // add all heights's to an array
    if (height == no_data_value) {
      array->push_back(nullptr);
    } else {
      array->push_back({json::fixed_t{height, precision}});
    }
  }

  return array;
}

json::ArrayPtr serialize_shape(const google::protobuf::RepeatedPtrField<valhalla::Location>& shape) {
  auto array = json::array({});
  for (const auto& p : shape) {
    array->emplace_back(json::map(
        {{"lon", json::fixed_t{p.ll().lng(), 6}}, {"lat", json::fixed_t{p.ll().lat(), 6}}}));
  }
  return array;
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
  auto json = json::map({});

  // get the precision to use for returned heights
  uint32_t precision = request.options().height_precision();

  // get the distances between the postings
  if (ranges.size()) {
    json = json::map({{"range_height", serialize_range_height(ranges, heights, precision,
                                                              skadi::get_no_data_value())}});
  } // just the postings
  else {
    json = json::map({{"height", serialize_height(heights, precision, skadi::get_no_data_value())}});
  }
  // send back the shape as well
  if (request.options().has_encoded_polyline_case()) {
    json->emplace("encoded_polyline", request.options().encoded_polyline());
  } else {
    json->emplace("shape", serialize_shape(request.options().shape()));
  }
  if (request.options().has_id_case()) {
    json->emplace("id", request.options().id());
  }

  // add warnings to json response
  if (request.info().warnings_size() >= 1) {
    json->emplace("warnings", serializeWarnings(request));
  }

  std::stringstream ss;
  ss << *json;
  return ss.str();
}
} // namespace tyr
} // namespace valhalla
