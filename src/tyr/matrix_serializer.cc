#include <cstdint>

#include "baldr/json.h"
#include "proto_conversions.h"
#include "thor/matrix_common.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::thor;

namespace {

json::ArrayPtr
serialize_duration(const google::protobuf::RepeatedPtrField<valhalla::Matrix_TimeDistance>& tds,
                   size_t start_td,
                   const size_t td_count) {
  auto time = json::array({});
  for (size_t i = start_td; i < start_td + td_count; ++i) {
    // check to make sure a route was found; if not, return null for time in matrix result
    if (tds[i].time() != kMaxCost) {
      time->emplace_back(static_cast<uint64_t>(tds[i].time()));
    } else {
      time->emplace_back(static_cast<std::nullptr_t>(nullptr));
    }
  }
  return time;
}

json::ArrayPtr
serialize_distance(const google::protobuf::RepeatedPtrField<valhalla::Matrix_TimeDistance>& tds,
                   size_t start_td,
                   const size_t td_count,
                   const size_t /* source_index */,
                   const size_t /* target_index */,
                   double distance_scale) {
  auto distance = json::array({});
  for (size_t i = start_td; i < start_td + td_count; ++i) {
    // check to make sure a route was found; if not, return null for distance in matrix result
    if (tds[i].time() != kMaxCost) {
      distance->emplace_back(json::fixed_t{tds[i].distance() * distance_scale, 3});
    } else {
      distance->emplace_back(static_cast<std::nullptr_t>(nullptr));
    }
  }
  return distance;
}
} // namespace

namespace osrm_serializers {

// Serialize route response in OSRM compatible format.
std::string serialize(const Api& request, double distance_scale) {
  auto json = json::map({});
  auto time = json::array({});
  auto distance = json::array({});
  const auto& options = request.options();

  // If here then the matrix succeeded. Set status code to OK and serialize
  // waypoints (locations).
  json->emplace("code", std::string("Ok"));
  json->emplace("sources", osrm::waypoints(options.sources()));
  json->emplace("destinations", osrm::waypoints(options.targets()));

  for (size_t source_index = 0; source_index < options.sources_size(); ++source_index) {
    time->emplace_back(serialize_duration(request.matrix().time_distances(),
                                          source_index * options.targets_size(),
                                          options.targets_size()));
    distance->emplace_back(
        serialize_distance(request.matrix().time_distances(), source_index * options.targets_size(),
                           options.targets_size(), source_index, 0, distance_scale));
  }
  json->emplace("durations", time);
  json->emplace("distances", distance);
  json->emplace("algorithm", MatrixAlgoToString(request.matrix().algorithm()));

  std::stringstream ss;
  ss << *json;
  return ss.str();
}
} // namespace osrm_serializers

namespace valhalla_serializers {

/*
valhalla output looks like this:

*/

json::ArrayPtr locations(const google::protobuf::RepeatedPtrField<valhalla::Location>& correlated) {
  auto input_locs = json::array({});
  for (size_t i = 0; i < correlated.size(); i++) {
    input_locs->emplace_back(json::map({{"lat", json::fixed_t{correlated.Get(i).ll().lat(), 6}},
                                        {"lon", json::fixed_t{correlated.Get(i).ll().lng(), 6}}}));
  }
  return input_locs;
}

json::ArrayPtr
serialize_row(const google::protobuf::RepeatedPtrField<valhalla::Matrix_TimeDistance>& tds,
              size_t start_td,
              const size_t td_count,
              const size_t source_index,
              const size_t target_index,
              double distance_scale) {
  auto row = json::array({});
  for (size_t i = start_td; i < start_td + td_count; ++i) {
    // check to make sure a route was found; if not, return null for distance & time in matrix
    // result
    json::MapPtr map;
    if (tds[i].time() != kMaxCost) {
      map = json::map({{"from_index", static_cast<uint64_t>(source_index)},
                       {"to_index", static_cast<uint64_t>(target_index + (i - start_td))},
                       {"time", static_cast<uint64_t>(tds[i].time())},
                       {"distance", json::fixed_t{tds[i].distance() * distance_scale, 3}}});
      if (!tds[i].date_time().empty()) {
        map->emplace("date_time", tds[i].date_time());
      }
    } else {
      map = json::map({{"from_index", static_cast<uint64_t>(source_index)},
                       {"to_index", static_cast<uint64_t>(target_index + (i - start_td))},
                       {"time", static_cast<std::nullptr_t>(nullptr)},
                       {"distance", static_cast<std::nullptr_t>(nullptr)}});
    }
    row->emplace_back(map);
  }
  return row;
}

std::string serialize(const Api& request, double distance_scale) {
  auto json = json::map({});
  const auto& options = request.options();

  if (options.verbose()) {
    json::ArrayPtr matrix = json::array({});
    for (size_t source_index = 0; source_index < options.sources_size(); ++source_index) {
      matrix->emplace_back(serialize_row(request.matrix().time_distances(),
                                         source_index * options.targets_size(),
                                         options.targets_size(), source_index, 0, distance_scale));
    }

    json->emplace("sources_to_targets", matrix);

    json->emplace("targets", json::array({locations(options.targets())}));
    json->emplace("sources", json::array({locations(options.sources())}));
  } // slim it down
  else {
    auto matrix = json::map({});
    auto time = json::array({});
    auto distance = json::array({});

    for (size_t source_index = 0; source_index < options.sources_size(); ++source_index) {
      time->emplace_back(serialize_duration(request.matrix().time_distances(),
                                            source_index * options.targets_size(),
                                            options.targets_size()));
      distance->emplace_back(
          serialize_distance(request.matrix().time_distances(), source_index * options.targets_size(),
                             options.targets_size(), source_index, 0, distance_scale));
    }
    matrix->emplace("distances", distance);
    matrix->emplace("durations", time);

    json->emplace("sources_to_targets", matrix);
  }

  json->emplace("units", Options_Units_Enum_Name(options.units()));
  json->emplace("algorithm", MatrixAlgoToString(request.matrix().algorithm()));

  if (options.has_id_case()) {
    json->emplace("id", options.id());
  }

  // add warnings to json response
  if (request.info().warnings_size() >= 1) {
    json->emplace("warnings", valhalla::tyr::serializeWarnings(request));
  }

  std::stringstream ss;
  ss << *json;
  return ss.str();
}
} // namespace valhalla_serializers

namespace valhalla {
namespace tyr {

std::string serializeMatrix(Api& request) {
  auto format = request.options().format();
  double distance_scale = (request.options().units() == Options::miles) ? kMilePerMeter : kKmPerMeter;
  switch (request.options().format()) {
    case Options_Format_osrm:
      return osrm_serializers::serialize(request, distance_scale);
    case Options_Format_json:
      return valhalla_serializers::serialize(request, distance_scale);
    case Options_Format_pbf:
      return serializePbf(request);
    default:
      throw;
  }
}

} // namespace tyr
} // namespace valhalla
