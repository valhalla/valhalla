#include <sstream>

#include "baldr/json.h"
#include "skadi/sample.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace {

json::ArrayPtr serialize_range_height(const std::vector<float>& ranges,
                                      const std::vector<double>& heights,
                                      const double no_data_value) {
  auto array = json::array({});
  // for each posting
  auto range = ranges.cbegin();

  for (const auto height : heights) {
    auto element = json::array({json::fp_t{*range, 0}});
    if (height == no_data_value) {
      element->push_back(nullptr);
    } else {
      element->push_back({json::fp_t{height, 0}});
    }
    array->push_back(element);
    ++range;
  }
  return array;
}

json::ArrayPtr serialize_height(const std::vector<double>& heights, const double no_data_value) {
  auto array = json::array({});

  for (const auto height : heights) {
    // add all heights's to an array
    if (height == no_data_value) {
      array->push_back(nullptr);
    } else {
      array->push_back({json::fp_t{height, 0}});
    }
  }

  return array;
}

json::ArrayPtr serialize_shape(const google::protobuf::RepeatedPtrField<valhalla::Location>& shape) {
  auto array = json::array({});
  for (const auto& p : shape) {
    array->emplace_back(
        json::map({{"lon", json::fp_t{p.ll().lng(), 6}}, {"lat", json::fp_t{p.ll().lat(), 6}}}));
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
                            const std::vector<float>& ranges) {
  auto json = json::map({});

  // get the distances between the postings
  if (ranges.size()) {
    json = json::map({{"range_height",
                       serialize_range_height(ranges, heights, skadi::sample::get_no_data_value())}});
  } // just the postings
  else {
    json = json::map({{"height", serialize_height(heights, skadi::sample::get_no_data_value())}});
  }
  // send back the shape as well
  if (request.options().has_encoded_polyline()) {
    json->emplace("encoded_polyline", request.options().encoded_polyline());
  } else {
    json->emplace("shape", serialize_shape(request.options().shape()));
  }
  if (request.options().has_id()) {
    json->emplace("id", request.options().id());
  }

  std::stringstream ss;
  ss << *json;
  return ss.str();
}
} // namespace tyr
} // namespace valhalla
