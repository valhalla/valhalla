#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "baldr/json.h"
#include "midgard/encoded.h"
#include "midgard/pointll.h"
#include "midgard/polyline2.h"
#include "midgard/util.h"
#include "odin/enhancedtrippath.h"
#include "odin/util.h"
#include "route_summary_cache.h"
#include "tyr/serializer_constants.h"
#include "tyr/serializers.h"
#include "worker.h"

#include "proto/directions.pb.h"
#include "proto/options.pb.h"
#include "proto/trip.pb.h"
#include "proto_conversions.h"
#ifdef INLINE_TEST
#include "test.h"
#endif

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::odin;
using namespace valhalla::tyr;
using namespace std;

namespace {
const std::string kSignElementDelimiter = ", ";
const std::string kDestinationsDelimiter = ": ";
const std::string kSpeedLimitSignVienna = "vienna";
const std::string kSpeedLimitSignMutcd = "mutcd";
const std::string kSpeedLimitUnitsKph = "km/h";
const std::string kSpeedLimitUnitsMph = "mph";

constexpr std::size_t MAX_USED_SEGMENTS = 2;

struct Coordinate {
  std::int32_t lng;
  std::int32_t lat;

  Coordinate(const std::int32_t lng_, const std::int32_t lat_) : lng(lng_), lat(lat_) {
  }
};

inline std::int32_t toFixed(const float floating) {
  const auto d = static_cast<double>(floating);
  const auto fixed = static_cast<std::int32_t>(std::round(d * ENCODE_PRECISION));
  return fixed;
}

inline double toFloating(const std::int32_t fixed) {
  const auto i = static_cast<std::int32_t>(fixed);
  const auto floating = static_cast<double>(i) * DECODE_PRECISION;
  return floating;
}

const constexpr double TILE_SIZE = 256.0;
static constexpr unsigned MAX_ZOOM = 18;
static constexpr unsigned MIN_ZOOM = 1;
// this is an upper bound to current display sizes
static constexpr double VIEWPORT_WIDTH = 8 * TILE_SIZE;
static constexpr double VIEWPORT_HEIGHT = 5 * TILE_SIZE;
static double INV_LOG_2 = 1. / std::log(2);
const constexpr double DEGREE_TO_RAD = 0.017453292519943295769236907684886;
const constexpr double RAD_TO_DEGREE = 1. / DEGREE_TO_RAD;
const constexpr double EPSG3857_MAX_LATITUDE = 85.051128779806592378; // 90(4*atan(exp(pi))/pi-1)

const constexpr PointLL::first_type DOUGLAS_PEUCKER_THRESHOLDS[19] = {
    703125.0, // z0
    351562.5, // z1
    175781.2, // z2
    87890.6,  // z3
    43945.3,  // z4
    21972.6,  // z5
    10986.3,  // z6
    5493.1,   // z7
    2746.5,   // z8
    1373.2,   // z9
    686.6,    // z10
    343.3,    // z11
    171.6,    // z12
    85.8,     // z13
    42.9,     // z14
    21.4,     // z15
    10.7,     // z16
    5.3,      // z17
    2.6,      // z18
};

inline double clamp(const double lat) {
  return std::max(std::min(lat, double(EPSG3857_MAX_LATITUDE)), double(-EPSG3857_MAX_LATITUDE));
}

inline double latToY(const double latitude) {
  // apparently this is the (faster) version of the canonical log(tan()) version
  const auto clamped_latitude = clamp(latitude);
  const double f = std::sin(DEGREE_TO_RAD * static_cast<double>(clamped_latitude));
  return RAD_TO_DEGREE * 0.5 * std::log((1 + f) / (1 - f));
}

inline double lngToPixel(double lon, unsigned zoom) {
  const double shift = (1u << zoom) * TILE_SIZE;
  const double b = shift / 2.0;
  const double x = b * (1 + static_cast<double>(lon) / 180.0);
  return x;
}

inline double latToPixel(double lat, unsigned zoom) {
  const double shift = (1u << zoom) * TILE_SIZE;
  const double b = shift / 2.0;
  const double y = b * (1. - latToY(lat) / 180.);
  return y;
}

inline unsigned getFittedZoom(Coordinate south_west, Coordinate north_east) {
  const auto min_x = lngToPixel(toFloating(south_west.lng), MAX_ZOOM);
  const auto max_y = latToPixel(toFloating(south_west.lat), MAX_ZOOM);
  const auto max_x = lngToPixel(toFloating(north_east.lng), MAX_ZOOM);
  const auto min_y = latToPixel(toFloating(north_east.lat), MAX_ZOOM);
  const double width_ratio = (max_x - min_x) / VIEWPORT_WIDTH;
  const double height_ratio = (max_y - min_y) / VIEWPORT_HEIGHT;
  const auto zoom = MAX_ZOOM - std::max(std::log(width_ratio), std::log(height_ratio)) * INV_LOG_2;

  if (std::isfinite(zoom))
    return std::max<unsigned>(MIN_ZOOM, zoom);
  else
    return MIN_ZOOM;
}

// Sign style and unit conventions for speed limit signs by country.
// Most countries use Vienna style and km/h, but the countries below
// use MUTCD and/or mph conventions.
std::unordered_map<std::string, std::pair<std::string, std::string>> speed_limit_info = {
    {"AG", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"AI", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"AS", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"BS", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"BZ", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"CA", {kSpeedLimitSignMutcd, kSpeedLimitUnitsKph}},
    {"DM", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"FK", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"GB", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"GD", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"GG", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"GS", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"GU", {kSpeedLimitSignMutcd, kSpeedLimitUnitsMph}},
    {"IM", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"JE", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"KN", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"KY", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"LC", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"LR", {kSpeedLimitSignMutcd, kSpeedLimitUnitsKph}},
    {"MP", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"MS", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"PR", {kSpeedLimitSignMutcd, kSpeedLimitUnitsMph}},
    {"SH", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"TC", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"US", {kSpeedLimitSignMutcd, kSpeedLimitUnitsMph}},
    {"VC", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"VG", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"VI", {kSpeedLimitSignMutcd, kSpeedLimitUnitsMph}},
    {"WS", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
};

namespace osrm_serializers {
/*
OSRM output is described in: http://project-osrm.org/docs/v5.5.1/api/
{
    "code":"Ok"
    "waypoints": [{ }, { }...],
    "routes": [
        {
            "geometry":"....."
            "distance":xxx.y
            "duration":yyy.z
            "legs":[
                {
                    "steps":[
                        "intersections":[
                        ]
                        "geometry":" "
                        "maneuver":{
                        }
                    ]
                }
            ]
        },
        ...
    ]
}
*/

std::string destinations(const valhalla::TripSign& sign);

// Add OSRM route summary information: distance, duration
void route_summary(json::MapPtr& route, const valhalla::Api& api, bool imperial, int route_index) {
  // Compute total distance and duration
  double duration = 0;
  double distance = 0;
  double weight = 0;
  auto leg_itr = api.trip().routes(route_index).legs().begin();
  std::vector<std::pair<double, double>> recosts(api.options().recostings_size(), {0, 0});
  for (const auto& leg : api.directions().routes(route_index).legs()) {
    distance += leg.summary().length();
    duration += leg.summary().time();
    weight += leg_itr->node().rbegin()->cost().elapsed_cost().cost();
    for (int i = 0; i < leg_itr->node().rbegin()->recosts_size(); ++i) {
      const auto& recost = leg_itr->node().rbegin()->recosts(i);
      if (!recost.has_elapsed_cost() || recosts[i].first < 0) {
        recosts[i].first = -1;
        recosts[i].second = -1;
      } else {
        recosts[i].first += recost.elapsed_cost().seconds();
        recosts[i].second += recost.elapsed_cost().cost();
      }
    }
    ++leg_itr;
  }

  // Convert distance to meters. Output distance and duration.
  distance = units_to_meters(distance, !imperial);
  route->emplace("distance", json::fixed_t{distance, 3});
  route->emplace("duration", json::fixed_t{duration, 3});

  route->emplace("weight", json::fixed_t{weight, 3});
  assert(api.options().costing_options(api.options().costing()).has_name());
  route->emplace("weight_name", api.options().costing_options(api.options().costing()).name());

  auto recosting_itr = api.options().recostings().begin();
  for (const auto& recost : recosts) {
    if (recost.first < 0) {
      route->emplace("duration_" + recosting_itr->name(), nullptr_t());
      route->emplace("weight_" + recosting_itr->name(), nullptr_t());
    } else {
      route->emplace("duration_" + recosting_itr->name(), json::fixed_t{recost.first, 3});
      route->emplace("weight_" + recosting_itr->name(), json::fixed_t{recost.second, 3});
    }
    ++recosting_itr;
  }
}

// Generate leg shape in geojson format.
json::MapPtr geojson_shape(const std::vector<PointLL> shape) {
  auto geojson = json::map({});
  auto coords = json::array({});
  coords->reserve(shape.size());
  for (const auto& p : shape) {
    coords->emplace_back(json::array(
        {json::fixed_t{p.lng(), DIGITS_PRECISION}, json::fixed_t{p.lat(), DIGITS_PRECISION}}));
  }
  geojson->emplace("type", std::string("LineString"));
  geojson->emplace("coordinates", std::move(coords));
  return geojson;
}

// Generate full shape of the route.
std::vector<PointLL> full_shape(const valhalla::DirectionsRoute& directions,
                                const valhalla::Options& options) {
  // If just one leg and it we want polyline6 then we just return the encoded leg shape
  if (directions.legs().size() == 1 && options.shape_format() == polyline6) {
    return midgard::decode<std::vector<PointLL>>(directions.legs().begin()->shape());
  }
  // TODO: there is a tricky way to do this... since the end of each leg is the same as the
  // beginning we essentially could just peel off the first encoded shape point of all the legs (but
  // the first) this way we wouldn't really have to do any decoding (would be far faster). it might
  // even be the case that the string length of the first number is a fixed length (which would be
  // great!) have to have a look should make this a function in midgard probably so the logic is all
  // in the same place
  std::vector<PointLL> decoded;
  for (const auto& leg : directions.legs()) {
    auto decoded_leg = midgard::decode<std::vector<PointLL>>(leg.shape());
    decoded.insert(decoded.end(), decoded.size() ? decoded_leg.begin() + 1 : decoded_leg.begin(),
                   decoded_leg.end());
  }
  return decoded;
}

// Generate simplified shape of the route.
std::vector<PointLL> simplified_shape(const valhalla::DirectionsRoute& directions) {
  Coordinate south_west(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
  Coordinate north_east(std::numeric_limits<int>::min(), std::numeric_limits<int>::min());
  std::vector<PointLL> simple_shape;
  std::unordered_set<size_t> indices;
  for (const auto& leg : directions.legs()) {
    auto decoded_leg = midgard::decode<std::vector<PointLL>>(leg.shape());
    for (const auto& coord : decoded_leg) {
      south_west.lng = std::min(south_west.lng, toFixed(coord.lng()));
      south_west.lat = std::min(south_west.lat, toFixed(coord.lat()));
      north_east.lng = std::max(north_east.lng, toFixed(coord.lng()));
      north_east.lat = std::max(north_east.lat, toFixed(coord.lat()));
    }

    for (const auto& maneuver : leg.maneuver()) {
      indices.emplace(simple_shape.size() ? ((simple_shape.size() - 1) + maneuver.begin_shape_index())
                                          : maneuver.begin_shape_index());
    }

    simple_shape.insert(simple_shape.end(),
                        simple_shape.size() ? decoded_leg.begin() + 1 : decoded_leg.begin(),
                        decoded_leg.end());
  }

  const auto zoom_level = std::min(MAX_ZOOM, getFittedZoom(south_west, north_east));
  Polyline2<PointLL>::Generalize(simple_shape, DOUGLAS_PEUCKER_THRESHOLDS[zoom_level], indices);
  return simple_shape;
}

void route_geometry(json::MapPtr& route,
                    const valhalla::DirectionsRoute& directions,
                    const valhalla::Options& options) {
  std::vector<PointLL> shape;
  if (options.has_generalize() && options.generalize() == 0.0f) {
    shape = simplified_shape(directions);
  } else if (!options.has_generalize() || (options.has_generalize() && options.generalize() > 0.0f)) {
    shape = full_shape(directions, options);
  }
  if (options.shape_format() == geojson) {
    route->emplace("geometry", geojson_shape(shape));
  } else {
    int precision = options.shape_format() == polyline6 ? 1e6 : 1e5;
    route->emplace("geometry", midgard::encode(shape, precision));
  }
}

json::MapPtr serialize_annotations(const valhalla::TripLeg& trip_leg) {
  auto attributes_map = json::map({});
  attributes_map->reserve(4);

  if (trip_leg.shape_attributes().time_size() > 0) {
    auto duration_array = json::array({});
    duration_array->reserve(trip_leg.shape_attributes().time_size());
    for (const auto& time : trip_leg.shape_attributes().time()) {
      // milliseconds (ms) to seconds (sec)
      duration_array->push_back(json::fixed_t{time * kSecPerMillisecond, 3});
    }
    attributes_map->emplace("duration", duration_array);
  }

  if (trip_leg.shape_attributes().length_size() > 0) {
    auto distance_array = json::array({});
    distance_array->reserve(trip_leg.shape_attributes().length_size());
    for (const auto& length : trip_leg.shape_attributes().length()) {
      // decimeters (dm) to meters (m)
      distance_array->push_back(json::fixed_t{length * kMeterPerDecimeter, 1});
    }
    attributes_map->emplace("distance", distance_array);
  }

  if (trip_leg.shape_attributes().speed_size() > 0) {
    auto speeds_array = json::array({});
    speeds_array->reserve(trip_leg.shape_attributes().speed_size());
    for (const auto& speed : trip_leg.shape_attributes().speed()) {
      // dm/s to m/s
      speeds_array->push_back(json::fixed_t{speed * kMeterPerDecimeter, 1});
    }
    attributes_map->emplace("speed", speeds_array);
  }

  if (trip_leg.shape_attributes().speed_limit_size() > 0) {
    auto speed_limits_array = json::array({});
    speed_limits_array->reserve(trip_leg.shape_attributes().speed_limit_size());
    for (const auto& speed_limit : trip_leg.shape_attributes().speed_limit()) {
      auto speed_limit_annotation = json::map({});
      if (speed_limit == kUnlimitedSpeedLimit) {
        speed_limit_annotation->emplace("none", true);
      } else if (speed_limit > 0) {
        // TODO support mph?
        speed_limit_annotation->emplace("unit", kSpeedLimitUnitsKph);
        speed_limit_annotation->emplace("speed", static_cast<uint64_t>(speed_limit));
      } else {
        speed_limit_annotation->emplace("unknown", true);
      }
      speed_limits_array->push_back(speed_limit_annotation);
    }
    attributes_map->emplace("maxspeed", speed_limits_array);
  }

  return attributes_map;
}

// Serialize waypoints for optimized route. Note that OSRM retains the
// original location order, and stores an index for the waypoint index in
// the optimized sequence.
json::ArrayPtr waypoints(google::protobuf::RepeatedPtrField<valhalla::Location>& locs) {
  // Create a vector of indexes.
  uint32_t i = 0;
  std::vector<uint32_t> indexes;
  for (const auto& loc : locs) {
    indexes.push_back(i++);
  }

  // Sort the the vector by the location's original index
  std::sort(indexes.begin(), indexes.end(), [&locs](const uint32_t a, const uint32_t b) -> bool {
    return locs.Get(a).original_index() < locs.Get(b).original_index();
  });

  // Output each location in its original index order along with its
  // waypoint index (which is the index in the optimized order).
  auto waypoints = json::array({});
  for (const auto& index : indexes) {
    locs.Mutable(index)->set_waypoint_index(index);
    waypoints->emplace_back(osrm::waypoint(locs.Get(index), false, true));
  }
  return waypoints;
}

// Simple structure for storing intersection data
struct IntersectionEdges {
  uint32_t bearing;
  bool routeable;
  bool in_edge;
  bool out_edge;

  IntersectionEdges(const uint32_t brg, const bool rte, const bool edge_in, const bool edge_out)
      : bearing(brg), routeable(rte), in_edge(edge_in), out_edge(edge_out) {
  }

  bool operator<(const IntersectionEdges& other) const {
    return bearing < other.bearing;
  }
};

// Add intersections along a step/maneuver.
json::ArrayPtr intersections(const valhalla::DirectionsLeg::Maneuver& maneuver,
                             valhalla::odin::EnhancedTripLeg* etp,
                             const std::vector<PointLL>& shape,
                             uint32_t& count,
                             const bool arrive_maneuver) {
  // Iterate through the nodes/intersections of the path for this maneuver
  count = 0;
  auto intersections = json::array({});
  uint32_t n = arrive_maneuver ? maneuver.end_path_index() + 1 : maneuver.end_path_index();
  EnhancedTripLeg_Node* prev_node = nullptr;
  for (uint32_t i = maneuver.begin_path_index(); i < n; i++) {
    auto intersection = json::map({});

    // Get the node and current edge from the enhanced trip path
    // NOTE: curr_edge does not exist for the arrive maneuver
    auto node = etp->GetEnhancedNode(i);
    auto curr_edge = etp->GetCurrEdge(i);
    auto prev_edge = etp->GetPrevEdge(i);

    // Add the node location (lon, lat). Use the last shape point for
    // the arrive step
    auto loc = json::array({});
    size_t shape_index = arrive_maneuver ? shape.size() - 1 : curr_edge->begin_shape_index();
    PointLL ll = shape[shape_index];
    loc->emplace_back(json::fixed_t{ll.lng(), 6});
    loc->emplace_back(json::fixed_t{ll.lat(), 6});
    intersection->emplace("location", loc);
    intersection->emplace("geometry_index", static_cast<uint64_t>(shape_index));

    // Add index into admin list
    if (node->has_admin_index()) {
      intersection->emplace("admin_index", static_cast<uint64_t>(node->admin_index()));
    }

    if (!arrive_maneuver) {
      if (curr_edge->has_is_urban()) {
        intersection->emplace("is_urban", curr_edge->is_urban());
      }
    }

    auto toll_collection = json::map({});
    if (node->type() == TripLeg_Node::kTollBooth) {
      toll_collection->emplace("type", std::string("toll_booth"));
    } else if (node->type() == TripLeg_Node::kTollGantry) {
      toll_collection->emplace("type", std::string("toll_gantry"));
    }
    if (!toll_collection->empty())
      intersection->emplace("toll_collection", toll_collection);

    if (node->cost().transition_cost().seconds() > 0)
      intersection->emplace("turn_duration",
                            json::fixed_t{node->cost().transition_cost().seconds(), 3});
    if (node->cost().transition_cost().cost() > 0)
      intersection->emplace("turn_weight", json::fixed_t{node->cost().transition_cost().cost(), 3});
    auto next_node = i + 1 < n ? etp->GetEnhancedNode(i + 1) : nullptr;
    if (next_node) {
      auto secs = next_node->cost().elapsed_cost().seconds() - node->cost().elapsed_cost().seconds();
      auto cost = next_node->cost().elapsed_cost().cost() - node->cost().elapsed_cost().cost();
      if (secs > 0)
        intersection->emplace("duration", json::fixed_t{secs, 3});
      if (cost > 0)
        intersection->emplace("weight", json::fixed_t{cost, 3});
    }

    // TODO: add recosted durations to the intersection?

    // Add rest_stop when passing by a rest_area or service_area
    if (i > 0 && !arrive_maneuver) {
      auto rest_stop = json::map({});
      for (uint32_t m = 0; m < node->intersecting_edge_size(); m++) {
        auto intersecting_edge = node->GetIntersectingEdge(m);
        bool routeable = intersecting_edge->IsTraversableOutbound(curr_edge->travel_mode());

        std::string sign_text;
        if (intersecting_edge->has_sign()) {
          const valhalla::TripSign& trip_leg_sign = intersecting_edge->sign();
          // I've looked at the results from guide_destinations(), destinations(), and
          // exit_destinations(). exit_destinations() does not contain rest-area names.
          // guide_destinations() and destinations() return the same string value for
          // the rest area name. So I've decided to use guide_destinations().
          sign_text = destinations(trip_leg_sign);
        }

        if (routeable && intersecting_edge->use() == TripLeg_Use_kRestAreaUse) {
          rest_stop->emplace("type", std::string("rest_area"));
          if (!sign_text.empty()) {
            rest_stop->emplace("name", sign_text);
          }
          intersection->emplace("rest_stop", rest_stop);
          break;
        } else if (routeable && intersecting_edge->use() == TripLeg_Use_kServiceAreaUse) {
          rest_stop->emplace("type", std::string("service_area"));
          if (!sign_text.empty()) {
            rest_stop->emplace("name", sign_text);
          }
          intersection->emplace("rest_stop", rest_stop);
          break;
        }
      }
    }

    // Get bearings and access to outgoing intersecting edges. Do not add
    // any intersecting edges for the first depart intersection and for
    // the arrive step.
    std::vector<IntersectionEdges> edges;

    // Add the edge departing the node
    if (!arrive_maneuver) {
      edges.emplace_back(curr_edge->begin_heading(), true, false, true);
    }

    // Add the incoming edge except for the first depart intersection.
    // Set routeable to false except for arrive.
    // TODO - what if a true U-turn - need to set it to routeable.
    if (i > 0) {
      bool entry = (arrive_maneuver) ? true : false;
      uint32_t prior_heading = prev_edge->end_heading();
      edges.emplace_back(((prior_heading + 180) % 360), entry, true, false);
    }

    // Create bearing and entry output
    auto bearings = json::array({});
    auto entries = json::array({});

    // Sort edges by increasing bearing and update the in/out edge indexes
    std::sort(edges.begin(), edges.end());
    uint32_t incoming_index, outgoing_index;
    for (uint32_t n = 0; n < edges.size(); ++n) {
      if (edges[n].in_edge) {
        incoming_index = n;
      }
      if (edges[n].out_edge) {
        outgoing_index = n;
      }
      bearings->emplace_back(static_cast<uint64_t>(edges[n].bearing));
      entries->emplace_back(edges[n].routeable);
    }

    // Add the index of the input edge and output edge
    if (i > 0) {
      intersection->emplace("in", static_cast<uint64_t>(incoming_index));
    }
    if (!arrive_maneuver) {
      intersection->emplace("out", static_cast<uint64_t>(outgoing_index));
    }

    intersection->emplace("entry", entries);
    intersection->emplace("bearings", bearings);

    // Add tunnel_name for tunnels
    if (!arrive_maneuver) {
      if (curr_edge->tunnel() && !curr_edge->tagged_value().empty()) {
        for (uint32_t t = 0; t < curr_edge->tagged_value().size(); ++t) {
          if (curr_edge->tagged_value().Get(t).type() == TaggedValue_Type_kTunnel) {
            intersection->emplace("tunnel_name", curr_edge->tagged_value().Get(t).value());
          }
        }
      }
    }

    // Add classes based on the first edge after the maneuver (not needed
    // for arrive maneuver).
    if (!arrive_maneuver) {
      std::vector<std::string> classes;
      if (curr_edge->tunnel()) {
        classes.push_back("tunnel");
      }
      if (maneuver.portions_toll() || curr_edge->toll()) {
        classes.push_back("toll");
      }
      if (curr_edge->road_class() == valhalla::RoadClass::kMotorway) {
        classes.push_back("motorway");
      }
      if (curr_edge->use() == TripLeg::Use::TripLeg_Use_kFerryUse) {
        classes.push_back("ferry");
      }

      if (curr_edge->destination_only()) {
        classes.push_back("restricted");
      }
      if (classes.size() > 0) {
        auto class_list = json::array({});
        for (const auto& cl : classes) {
          class_list->emplace_back(cl);
        }
        intersection->emplace("classes", class_list);
      }
    }

    // Process turn lanes - which are stored on the previous edge to the node
    // Check if there is an active turn lane
    // Verify that turn lanes are not non-directional
    if (prev_edge && (prev_edge->turn_lanes_size() > 0) && prev_edge->HasActiveTurnLane() &&
        !prev_edge->HasNonDirectionalTurnLane()) {
      auto lanes = json::array({});
      for (const auto& turn_lane : prev_edge->turn_lanes()) {
        auto lane = json::map({});
        // Process 'valid' & 'active' flags
        bool is_active = turn_lane.state() == TurnLane::kActive;
        // an active lane is also valid
        bool is_valid = is_active || turn_lane.state() == TurnLane::kValid;
        lane->emplace("active", is_active);
        lane->emplace("valid", is_valid);
        // Add valid_indication for a valid & active lanes
        if (turn_lane.state() != TurnLane::kInvalid) {
          lane->emplace("valid_indication", turn_lane_direction(turn_lane.active_direction()));
        }

        // Process 'indications' array - add indications from left to right
        auto indications = json::array({});
        uint16_t mask = turn_lane.directions_mask();

        // TODO make map for lane mask to osrm indication string

        // reverse (left u-turn)
        if (mask & kTurnLaneReverse && prev_edge->drive_on_right()) {
          indications->emplace_back(osrmconstants::kModifierUturn);
        }
        // sharp_left
        if (mask & kTurnLaneSharpLeft) {
          indications->emplace_back(osrmconstants::kModifierSharpLeft);
        }
        // left
        if (mask & kTurnLaneLeft) {
          indications->emplace_back(osrmconstants::kModifierLeft);
        }
        // slight_left
        if (mask & kTurnLaneSlightLeft) {
          indications->emplace_back(osrmconstants::kModifierSlightLeft);
        }
        // through
        if (mask & kTurnLaneThrough) {
          indications->emplace_back(osrmconstants::kModifierStraight);
        }
        // slight_right
        if (mask & kTurnLaneSlightRight) {
          indications->emplace_back(osrmconstants::kModifierSlightRight);
        }
        // right
        if (mask & kTurnLaneRight) {
          indications->emplace_back(osrmconstants::kModifierRight);
        }
        // sharp_right
        if (mask & kTurnLaneSharpRight) {
          indications->emplace_back(osrmconstants::kModifierSharpRight);
        }
        // reverse (right u-turn)
        if (mask & kTurnLaneReverse && !prev_edge->drive_on_right()) {
          indications->emplace_back(osrmconstants::kModifierUturn);
        }
        lane->emplace("indications", std::move(indications));
        lanes->emplace_back(std::move(lane));
      }
      intersection->emplace("lanes", std::move(lanes));
    }

    // Add the intersection to the JSON array
    intersections->emplace_back(intersection);
    count++;
  }
  return intersections;
}

// Add exits (exit numbers) along a step/maneuver.
std::string exits(const valhalla::TripSign& sign) {
  // Iterate through the exit numbers
  std::string exits;
  for (const auto& number : sign.exit_numbers()) {
    if (!exits.empty()) {
      exits += "; ";
    }
    exits += number.text();
  }
  return exits;
}

valhalla::baldr::json::RawJSON serializeIncident(const TripLeg::Incident& incident) {
  rapidjson::StringBuffer stringbuffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(stringbuffer);
  writer.StartObject();
  osrm::serializeIncidentProperties(writer, incident.metadata(),
                                    incident.has_begin_shape_index() ? incident.begin_shape_index()
                                                                     : -1,
                                    incident.has_end_shape_index() ? incident.end_shape_index() : -1,
                                    "", "");
  writer.EndObject();
  return {stringbuffer.GetString()};
}

// Serializes incidents and adds to json-document
void serializeIncidents(const google::protobuf::RepeatedPtrField<TripLeg::Incident>& incidents,
                        json::Jmap& doc) {
  if (incidents.size() == 0) {
    // No incidents, nothing to do
    return;
  }
  json::ArrayPtr serialized_incidents = std::shared_ptr<json::Jarray>(new json::Jarray());
  {
    // Bring up any already existing array
    auto existing = doc.find("incidents");
    if (existing != doc.end()) {
      if (auto* ptr = boost::get<std::shared_ptr<valhalla::baldr::json::Jarray>>(&existing->second)) {
        serialized_incidents = *ptr;
      } else {
        throw std::logic_error("Invalid state: stored ptr should not be null");
      }
    }
  }
  for (const auto& incident : incidents) {
    auto json_incident = serializeIncident(incident);
    serialized_incidents->emplace_back(json_incident);
  }
  doc.emplace("incidents", serialized_incidents);
}

void serializeClosures(const valhalla::TripLeg& leg, json::Jmap& doc) {
  if (!leg.closures_size()) {
    return;
  }
  auto closures = json::array({});
  closures->reserve(leg.closures_size());
  for (const valhalla::TripLeg_Closure& closure : leg.closures()) {
    auto closure_obj = json::map({});
    closure_obj->emplace("geometry_index_start", static_cast<uint64_t>(closure.begin_shape_index()));
    closure_obj->emplace("geometry_index_end", static_cast<uint64_t>(closure.end_shape_index()));
    closures->emplace_back(std::move(closure_obj));
  }
  doc.emplace("closures", closures);
}

// Compile and return the refs of the specified list
// TODO we could enhance by limiting results by using consecutive count
std::string get_sign_element_refs(
    const google::protobuf::RepeatedPtrField<::valhalla::TripSignElement>& sign_elements,
    const std::string& delimiter = kSignElementDelimiter) {
  std::string refs;
  for (const auto& sign_element : sign_elements) {
    // Only process refs
    if (sign_element.is_route_number()) {
      // If refs is not empty, append specified delimiter
      if (!refs.empty()) {
        refs += delimiter;
      }
      // Append sign element
      refs += sign_element.text();
    }
  }
  return refs;
}

// Compile and return the nonrefs of the specified list
// TODO we could enhance by limiting results by using consecutive count
std::string get_sign_element_nonrefs(
    const google::protobuf::RepeatedPtrField<::valhalla::TripSignElement>& sign_elements,
    const std::string& delimiter = kSignElementDelimiter) {
  std::string nonrefs;
  for (const auto& sign_element : sign_elements) {
    // Only process nonrefs
    if (!(sign_element.is_route_number())) {
      // If nonrefs is not empty, append specified delimiter
      if (!nonrefs.empty()) {
        nonrefs += delimiter;
      }
      // Append sign element
      nonrefs += sign_element.text();
    }
  }
  return nonrefs;
}

// Compile and return the sign elements of the specified list
// TODO we could enhance by limiting results by using consecutive count
std::string get_sign_elements(
    const google::protobuf::RepeatedPtrField<::valhalla::TripSignElement>& sign_elements,
    const std::string& delimiter = kSignElementDelimiter) {
  std::string sign_elements_string;
  for (const auto& sign_element : sign_elements) {
    // If the sign_elements_string is not empty, append specified delimiter
    if (!sign_elements_string.empty()) {
      sign_elements_string += delimiter;
    }
    // Append sign element
    sign_elements_string += sign_element.text();
  }
  return sign_elements_string;
}

bool exit_destinations_exist(const valhalla::TripSign& sign) {
  if ((sign.exit_onto_streets_size() > 0) || (sign.exit_toward_locations_size() > 0) ||
      (sign.exit_names_size() > 0)) {
    return true;
  }
  return false;
}

// Return the exit destinations
std::string exit_destinations(const valhalla::TripSign& sign) {

  /////////////////////////////////////////////////////////////////////////////
  // Process the refs
  // Get the branch refs
  std::string branch_refs = get_sign_element_refs(sign.exit_onto_streets());

  // Get the toward refs
  std::string toward_refs = get_sign_element_refs(sign.exit_toward_locations());

  // Create the refs by combining the branch and toward ref lists
  std::string refs = branch_refs;
  // If needed, add the delimiter between the lists
  if (!refs.empty() && !toward_refs.empty()) {
    refs += kSignElementDelimiter;
  }
  refs += toward_refs;

  /////////////////////////////////////////////////////////////////////////////
  // Process the nonrefs
  // Get the branch nonrefs
  std::string branch_nonrefs = get_sign_element_nonrefs(sign.exit_onto_streets());

  // Get the towards nonrefs
  std::string toward_nonrefs = get_sign_element_nonrefs(sign.exit_toward_locations());

  // Get the name nonrefs only if the others are empty
  std::string name_nonrefs;
  if (branch_nonrefs.empty() && toward_nonrefs.empty()) {
    name_nonrefs = get_sign_element_nonrefs(sign.exit_names());
  }

  // Create nonrefs by combining the branch, toward, name nonref lists
  std::string nonrefs = branch_nonrefs;
  // If needed, add the delimiter between the lists
  if (!nonrefs.empty() && !toward_nonrefs.empty()) {
    nonrefs += kSignElementDelimiter;
  }
  nonrefs += toward_nonrefs;
  // If needed, add the delimiter between lists
  if (!nonrefs.empty() && !name_nonrefs.empty()) {
    nonrefs += kSignElementDelimiter;
  }
  nonrefs += name_nonrefs;

  /////////////////////////////////////////////////////////////////////////////
  // Process the destinations
  std::string destinations = refs;
  if (!refs.empty() && !nonrefs.empty()) {
    destinations += kDestinationsDelimiter;
  }
  destinations += nonrefs;

  return destinations;
}

// Return the guide destinations
std::string guide_destinations(const valhalla::TripSign& sign) {

  /////////////////////////////////////////////////////////////////////////////
  // Process the refs
  // Get the branch refs
  std::string branch_refs = get_sign_element_refs(sign.guide_onto_streets());

  // Get the toward refs
  std::string toward_refs = get_sign_element_refs(sign.guide_toward_locations());

  // Create the refs by combining the branch and toward ref lists
  std::string refs = branch_refs;
  // If needed, add the delimiter between the lists
  if (!refs.empty() && !toward_refs.empty()) {
    refs += kSignElementDelimiter;
  }
  refs += toward_refs;

  /////////////////////////////////////////////////////////////////////////////
  // Process the nonrefs
  // Get the branch nonrefs
  std::string branch_nonrefs = get_sign_element_nonrefs(sign.guide_onto_streets());

  // Get the towards nonrefs
  std::string toward_nonrefs = get_sign_element_nonrefs(sign.guide_toward_locations());

  // Create nonrefs by combining the branch, toward, name nonref lists
  std::string nonrefs = branch_nonrefs;
  // If needed, add the delimiter between the lists
  if (!nonrefs.empty() && !toward_nonrefs.empty()) {
    nonrefs += kSignElementDelimiter;
  }
  nonrefs += toward_nonrefs;

  /////////////////////////////////////////////////////////////////////////////
  // Process the destinations
  std::string destinations = refs;
  if (!refs.empty() && !nonrefs.empty()) {
    destinations += kDestinationsDelimiter;
  }
  destinations += nonrefs;

  return destinations;
}

// Add destinations along a step/maneuver. Constructs a destinations string.
// Here are the destinations formats:
//   1. <ref>
//   2. <non-ref>
//   3. <ref>: <non-ref>
// Each <ref> or <non-ref> could have one or more items and will separated with ", "
//   for example: "I 99, US 220, US 30: Altoona, Johnstown"
std::string destinations(const valhalla::TripSign& sign) {
  if (exit_destinations_exist(sign)) {
    return exit_destinations(sign);
  }
  return guide_destinations(sign);
}

// Get the turn modifier based on incoming edge bearing and outgoing edge
// bearing.
std::string turn_modifier(const uint32_t in_brg, const uint32_t out_brg) {
  auto turn_degree = GetTurnDegree(in_brg, out_brg);
  auto turn_type = Turn::GetType(turn_degree);
  switch (turn_type) {
    case baldr::Turn::Type::kStraight:
      return osrmconstants::kModifierStraight;
    case baldr::Turn::Type::kSlightRight:
      return osrmconstants::kModifierSlightRight;
    case baldr::Turn::Type::kRight:
      return osrmconstants::kModifierRight;
    case baldr::Turn::Type::kSharpRight:
      return osrmconstants::kModifierSharpRight;
    case baldr::Turn::Type::kReverse:
      return osrmconstants::kModifierUturn;
    case baldr::Turn::Type::kSharpLeft:
      return osrmconstants::kModifierSharpLeft;
    case baldr::Turn::Type::kLeft:
      return osrmconstants::kModifierLeft;
    case baldr::Turn::Type::kSlightLeft:
      return osrmconstants::kModifierSlightLeft;
  }
  auto num = static_cast<uint32_t>(turn_type);
  throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) +
                           " Unhandled Turn::Type: " + std::to_string(num));
}

// Get the turn modifier based on the maneuver type
// or if needed, the incoming edge bearing and outgoing edge bearing.
std::string turn_modifier(const valhalla::DirectionsLeg::Maneuver& maneuver,
                          const uint32_t in_brg,
                          const uint32_t out_brg,
                          const bool arrive_maneuver) {
  switch (maneuver.type()) {
    case valhalla::DirectionsLeg_Maneuver_Type_kStart:
    case valhalla::DirectionsLeg_Maneuver_Type_kDestination:
      return "";
    case valhalla::DirectionsLeg_Maneuver_Type_kSlightRight:
    case valhalla::DirectionsLeg_Maneuver_Type_kStayRight:
    case valhalla::DirectionsLeg_Maneuver_Type_kExitRight:
    case valhalla::DirectionsLeg_Maneuver_Type_kMergeRight:
      return osrmconstants::kModifierSlightRight;
    case valhalla::DirectionsLeg_Maneuver_Type_kRight:
    case valhalla::DirectionsLeg_Maneuver_Type_kStartRight:
    case valhalla::DirectionsLeg_Maneuver_Type_kDestinationRight:
      return osrmconstants::kModifierRight;
    case valhalla::DirectionsLeg_Maneuver_Type_kSharpRight:
      return osrmconstants::kModifierSharpRight;
    case valhalla::DirectionsLeg_Maneuver_Type_kUturnRight:
    case valhalla::DirectionsLeg_Maneuver_Type_kUturnLeft:
      // [TODO #1789] route ending in uturn should not set modifier=uturn
      if (arrive_maneuver)
        return "";
      return osrmconstants::kModifierUturn;
    case valhalla::DirectionsLeg_Maneuver_Type_kSharpLeft:
      return osrmconstants::kModifierSharpLeft;
    case valhalla::DirectionsLeg_Maneuver_Type_kLeft:
    case valhalla::DirectionsLeg_Maneuver_Type_kStartLeft:
    case valhalla::DirectionsLeg_Maneuver_Type_kDestinationLeft:
      return osrmconstants::kModifierLeft;
    case valhalla::DirectionsLeg_Maneuver_Type_kSlightLeft:
    case valhalla::DirectionsLeg_Maneuver_Type_kStayLeft:
    case valhalla::DirectionsLeg_Maneuver_Type_kExitLeft:
    case valhalla::DirectionsLeg_Maneuver_Type_kMergeLeft:
      return osrmconstants::kModifierSlightLeft;
    case valhalla::DirectionsLeg_Maneuver_Type_kRampRight:
      if (Turn::GetType(GetTurnDegree(in_brg, out_brg)) == baldr::Turn::Type::kRight)
        return osrmconstants::kModifierRight;
      else
        return osrmconstants::kModifierSlightRight;
    case valhalla::DirectionsLeg_Maneuver_Type_kRampLeft:
      if (Turn::GetType(GetTurnDegree(in_brg, out_brg)) == baldr::Turn::Type::kLeft)
        return osrmconstants::kModifierLeft;
      else
        return osrmconstants::kModifierSlightLeft;
    case valhalla::DirectionsLeg_Maneuver_Type_kRoundaboutEnter:
    case valhalla::DirectionsLeg_Maneuver_Type_kRoundaboutExit:
    case valhalla::DirectionsLeg_Maneuver_Type_kFerryEnter:
    case valhalla::DirectionsLeg_Maneuver_Type_kFerryExit:
      return turn_modifier(in_brg, out_brg);
    default:
      return osrmconstants::kModifierStraight;
  }
}

// Ramp cases - off ramp transitions from a motorway.
// On ramp ends in a motorway.
std::string ramp_type(const valhalla::DirectionsLeg::Maneuver& maneuver) {
  if ((maneuver.type() == DirectionsLeg_Maneuver_Type_kExitRight) ||
      (maneuver.type() == DirectionsLeg_Maneuver_Type_kExitLeft)) {
    return "off ramp";
  } else if ((maneuver.type() == DirectionsLeg_Maneuver_Type_kRampStraight) ||
             (maneuver.type() == DirectionsLeg_Maneuver_Type_kRampRight) ||
             (maneuver.type() == DirectionsLeg_Maneuver_Type_kRampLeft)) {

    // If slight turn
    uint32_t turn_degree = maneuver.turn_degree();
    if ((turn_degree > 329) || (turn_degree < 31)) {
      return "on ramp";
    } else {
      return "turn";
    }
  }
  return "";
}

// Populate the OSRM maneuver record within a step.
json::MapPtr osrm_maneuver(const valhalla::DirectionsLeg::Maneuver& maneuver,
                           valhalla::odin::EnhancedTripLeg* etp,
                           const PointLL& man_ll,
                           const bool depart_maneuver,
                           const bool arrive_maneuver,
                           const uint32_t prev_intersection_count,
                           const std::string& mode,
                           const std::string& prev_mode,
                           const bool rotary,
                           const bool prev_rotary,
                           const valhalla::Options& options) {
  auto osrm_man = json::map({});

  // Set the location
  auto loc = json::array({});
  loc->emplace_back(json::fixed_t{man_ll.lng(), 6});
  loc->emplace_back(json::fixed_t{man_ll.lat(), 6});
  osrm_man->emplace("location", loc);

  // Get incoming and outgoing bearing. For the incoming heading, use the
  // prior edge from the TripLeg. Compute turn modifier. TODO - reconcile
  // turn degrees between Valhalla and OSRM
  uint32_t idx = maneuver.begin_path_index();
  uint32_t in_brg = (idx > 0) ? etp->GetPrevEdge(idx)->end_heading() : 0;
  uint32_t out_brg = maneuver.begin_heading();
  osrm_man->emplace("bearing_before", static_cast<uint64_t>(in_brg));
  osrm_man->emplace("bearing_after", static_cast<uint64_t>(out_brg));

  std::string modifier;
  if (!depart_maneuver) {
    modifier = turn_modifier(maneuver, in_brg, out_brg, arrive_maneuver);
    if (!modifier.empty())
      osrm_man->emplace("modifier", modifier);
  }

  if (options.directions_type() == DirectionsType::instructions) {
    osrm_man->emplace("instruction", maneuver.text_instruction());
  }

  // TODO - logic to convert maneuver types from Valhalla into OSRM maneuver types.
  std::string maneuver_type;
  if (depart_maneuver) {
    maneuver_type = "depart";
  } else if (arrive_maneuver) {
    maneuver_type = "arrive";
  } else if (mode != prev_mode) {
    maneuver_type = "notification";
  } else if (maneuver.type() == DirectionsLeg_Maneuver_Type_kRoundaboutEnter) {
    if (rotary) {
      maneuver_type = "rotary";
    } else {
      maneuver_type = "roundabout";
    }
    // Roundabout count
    if (maneuver.has_roundabout_exit_count()) {
      osrm_man->emplace("exit", static_cast<uint64_t>(maneuver.roundabout_exit_count()));
    }
  } else if (maneuver.type() == DirectionsLeg_Maneuver_Type_kRoundaboutExit) {
    if (prev_rotary) {
      maneuver_type = "exit rotary";
    } else {
      maneuver_type = "exit roundabout";
    }
  } else {
    // Special cases
    auto prev_edge = etp->GetPrevEdge(idx);
    auto curr_edge = etp->GetCurrEdge(idx);
    bool new_name = maneuver.type() == DirectionsLeg_Maneuver_Type_kContinue ||
                    maneuver.type() == DirectionsLeg_Maneuver_Type_kBecomes;
    bool ramp = ((maneuver.type() == DirectionsLeg_Maneuver_Type_kRampStraight) ||
                 (maneuver.type() == DirectionsLeg_Maneuver_Type_kRampRight) ||
                 (maneuver.type() == DirectionsLeg_Maneuver_Type_kRampLeft) ||
                 (maneuver.type() == DirectionsLeg_Maneuver_Type_kExitRight) ||
                 (maneuver.type() == DirectionsLeg_Maneuver_Type_kExitLeft));
    bool fork = ((maneuver.type() == DirectionsLeg_Maneuver_Type_kStayStraight) ||
                 (maneuver.type() == DirectionsLeg_Maneuver_Type_kStayRight) ||
                 (maneuver.type() == DirectionsLeg_Maneuver_Type_kStayLeft));
    if ((maneuver.type() == DirectionsLeg_Maneuver_Type_kMerge) ||
        (maneuver.type() == DirectionsLeg_Maneuver_Type_kMergeLeft) ||
        (maneuver.type() == DirectionsLeg_Maneuver_Type_kMergeRight)) {
      maneuver_type = "merge";
    } else if (fork) {
      maneuver_type = "fork";
    } else if (ramp) {
      maneuver_type = ramp_type(maneuver);
    } else if (new_name) {
      maneuver_type = "new name";
    }

    // Are there any intersecting edges
    bool false_node = etp->node(idx).intersecting_edge_size() == 0;

    // Fall through case if maneuver not set by special cases above
    new_name = false;
    if (maneuver_type.empty()) {
      // Check for end of road if prior edge is not a ramp. Road ends if
      // the current road name ends and more than 1 intersection has been
      // passed. Description is: at t-intersections, when you’re turning
      // onto a new road name, and have passed at least 1 intersection to
      // get there.
      bool road_ends = (prev_intersection_count > 1 && prev_edge->use() != TripLeg_Use_kRampUse &&
                        etp->node(idx).intersecting_edge_size() == 1);
      if (road_ends) {
        // TODO what about a doubly digitized road ending at a T (would be
        // 2 intersecting edges)? What if there is a driveway or path as
        // an intersecting edge?
        const auto& intsct_edge = etp->node(idx).intersecting_edge(0);
        if (intsct_edge.prev_name_consistency()) {
          road_ends = false;
        } else {
          // Get turn types to see if this is a turn at a "T"
          // (opposing right/left turns).
          uint32_t turn_degree1 = GetTurnDegree(prev_edge->end_heading(), curr_edge->begin_heading());
          uint32_t turn_degree2 =
              GetTurnDegree(prev_edge->end_heading(), intsct_edge.begin_heading());
          Turn::Type turn_type1 = Turn::GetType(turn_degree1);
          Turn::Type turn_type2 = Turn::GetType(turn_degree2);
          if (!(turn_type1 == Turn::Type::kRight && turn_type2 == Turn::Type::kLeft) &&
              !(turn_type2 == Turn::Type::kRight && turn_type1 == Turn::Type::kLeft)) {
            road_ends = false;
          }
        }
      }

      // Check if previous maneuver and current maneuver have same name
      // TODO - more extensive name comparison method?
      if (prev_edge->name_size() > 0 && prev_edge->name_size() == curr_edge->name_size() &&
          (prev_edge->name(0).value() != curr_edge->name(0).value())) {
        new_name = true;
      }

      if (prev_intersection_count > 1 && road_ends) {
        maneuver_type = "end of road";
      } else if (false_node && new_name) {
        maneuver_type = "new name";
      } else {
        if ((modifier != osrmconstants::kModifierUturn) && (!maneuver.to_stay_on())) {
          maneuver_type = "turn";
        } else {
          maneuver_type = "continue";
        }
      }
    }
  }
  osrm_man->emplace("type", maneuver_type);

  return osrm_man;
}

// Method to get the geometry string for a maneuver.
void maneuver_geometry(json::MapPtr& step,
                       const uint32_t begin_idx,
                       const uint32_t end_idx,
                       const std::vector<PointLL>& shape,
                       bool is_arrive_maneuver,
                       const valhalla::Options& options) {
  // Must add one to the end range since maneuver end shape index is exclusive
  std::vector<PointLL> maneuver_shape(shape.begin() + begin_idx, shape.begin() + end_idx + 1);
  // Last maneuver shape is a linestring with two identical points at the destination
  if (is_arrive_maneuver) {
    maneuver_shape.push_back(shape.back());
  }

  if (options.shape_format() == geojson) {
    step->emplace("geometry", geojson_shape(maneuver_shape));
  } else {
    int precision = options.shape_format() == polyline6 ? 1e6 : 1e5;
    step->emplace("geometry", midgard::encode(maneuver_shape, precision));
  }
}

// Get the mode
std::string get_mode(const valhalla::DirectionsLeg::Maneuver& maneuver,
                     const bool arrive_maneuver,
                     valhalla::odin::EnhancedTripLeg* etp) {
  // Return ferry if not last maneuver and the edge use is Ferry
  if (!arrive_maneuver &&
      (etp->GetCurrEdge(maneuver.begin_path_index())->use() == TripLeg::Use::TripLeg_Use_kFerryUse)) {
    return "ferry";
  }

  // Otherwise return based on the travel mode
  switch (maneuver.travel_mode()) {
    case DirectionsLeg_TravelMode_kDrive: {
      return "driving";
    }
    case DirectionsLeg_TravelMode_kPedestrian: {
      return "walking";
    }
    case DirectionsLeg_TravelMode_kBicycle: {
      return "cycling";
    }
    case DirectionsLeg_TravelMode_kTransit: {
      return "transit";
    }
  }
  auto num = static_cast<int>(maneuver.travel_mode());
  throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) +
                           " Unhandled travel_mode: " + std::to_string(num));
}

// Get the names and ref names
std::pair<std::string, std::string>
names_and_refs(const valhalla::DirectionsLeg::Maneuver& maneuver) {
  std::string names, refs;

  // Roundabouts need to use the roundabout_exit_street_names
  // if a maneuver begin street name exists then use it otherwise use the maneuver street name
  // TODO: in the future we may switch to use both
  auto& street_names = (maneuver.type() == DirectionsLeg_Maneuver_Type_kRoundaboutEnter)
                           ? maneuver.roundabout_exit_street_names()
                           : (maneuver.begin_street_name_size() > 0) ? maneuver.begin_street_name()
                                                                     : maneuver.street_name();

  for (const auto& name : street_names) {
    // Check if the name is a ref
    if (name.is_route_number()) {
      if (!refs.empty()) {
        refs += "; ";
      }
      refs += name.value();
    } else {
      if (!names.empty()) {
        names += "; ";
      }
      names += name.value();
    }
  }

  return std::make_pair(names, refs);
}

// Serialize each leg
json::ArrayPtr serialize_legs(const google::protobuf::RepeatedPtrField<valhalla::DirectionsLeg>& legs,
                              const std::vector<std::string>& leg_summaries,
                              google::protobuf::RepeatedPtrField<valhalla::TripLeg>& path_legs,
                              bool imperial,
                              const valhalla::Options& options) {
  auto output_legs = json::array({});
  output_legs->reserve(path_legs.size());

  // Verify that the path_legs list is the same size as the legs list
  if (legs.size() != path_legs.size()) {
    throw valhalla_exception_t{503};
  }

  // Iterate through the legs in DirectionsLeg and TripLeg
  int leg_index = 0;
  auto leg = legs.begin();

  for (auto& path_leg : path_legs) {
    valhalla::odin::EnhancedTripLeg etp(path_leg);
    auto output_leg = json::map({});
    output_leg->reserve(10);

    // Get the full shape for the leg. We want to use this for serializing
    // encoded shape for each step (maneuver) in OSRM output.
    auto shape = midgard::decode<std::vector<PointLL>>(leg->shape());

    //#########################################################################
    // Iterate through maneuvers - convert to OSRM steps
    uint32_t maneuver_index = 0;
    uint32_t prev_intersection_count = 0;
    std::string drive_side = "right";
    std::string name = "";
    std::string ref = "";
    std::string mode = "";
    std::string prev_mode = "";
    bool rotary = false;
    bool prev_rotary = false;
    auto steps = json::array({});
    const DirectionsLeg_Maneuver* prev_maneuver = nullptr;
    json::MapPtr prev_step;
    for (const auto& maneuver : leg->maneuver()) {
      auto step = json::map({});
      step->reserve(15); // lots of conditional stuff here
      bool depart_maneuver = (maneuver_index == 0);
      bool arrive_maneuver = (maneuver_index == leg->maneuver_size() - 1);

      // TODO - iterate through TripLeg from prior maneuver end to
      // end of this maneuver - perhaps insert OSRM specific steps such as
      // name change

      // Add geometry for this maneuver
      maneuver_geometry(step, maneuver.begin_shape_index(), maneuver.end_shape_index(), shape,
                        arrive_maneuver, options);

      // Add mode, driving side, weight, distance, duration, name
      double distance = units_to_meters(maneuver.length(), !imperial);
      double duration = maneuver.time();

      // Process drive_side, name, ref, mode, and prev_mode attributes if not the arrive maneuver
      if (!arrive_maneuver) {
        drive_side =
            (etp.GetCurrEdge(maneuver.begin_path_index())->drive_on_right()) ? "right" : "left";
        auto name_ref_pair = names_and_refs(maneuver);
        name = name_ref_pair.first;
        ref = name_ref_pair.second;
        mode = get_mode(maneuver, arrive_maneuver, &etp);
        if (prev_mode.empty())
          prev_mode = mode;
      }

      step->emplace("mode", mode);
      step->emplace("driving_side", drive_side);
      step->emplace("distance", json::fixed_t{distance, 3});
      step->emplace("duration", json::fixed_t{duration, 3});
      const auto& end_node = path_leg.node(maneuver.end_path_index());
      const auto& begin_node = path_leg.node(maneuver.begin_path_index());
      auto weight = end_node.cost().elapsed_cost().cost() - begin_node.cost().elapsed_cost().cost();
      step->emplace("weight", json::fixed_t{weight, 3});
      auto recost_itr = options.recostings().begin();
      auto begin_recost_itr = begin_node.recosts().begin();
      for (const auto& end_recost : end_node.recosts()) {
        if (end_recost.has_elapsed_cost()) {
          step->emplace("duration_" + recost_itr->name(),
                        json::fixed_t{end_recost.elapsed_cost().seconds() -
                                          begin_recost_itr->elapsed_cost().seconds(),
                                      3});
          step->emplace("weight_" + recost_itr->name(),
                        json::fixed_t{end_recost.elapsed_cost().cost() -
                                          begin_recost_itr->elapsed_cost().cost(),
                                      3});
        } else {
          step->emplace("duration_" + recost_itr->name(), nullptr_t());
          step->emplace("weight_" + recost_itr->name(), nullptr_t());
        }
        ++recost_itr;
        ++begin_recost_itr;
      }

      step->emplace("name", name);
      if (!ref.empty()) {
        step->emplace("ref", ref);
      }

      // Check if speed limits were requested
      if (path_leg.shape_attributes().speed_limit_size() > 0) {
        // Lookup speed limit info for this country
        auto country_code = etp.GetCountryCode(maneuver.begin_path_index());
        auto country = speed_limit_info.find(country_code);
        if (country != speed_limit_info.end()) {
          // Some countries have different speed limit sign types and speed units
          step->emplace("speedLimitSign", country->second.first);
          step->emplace("speedLimitUnit", country->second.second);
        } else {
          // Otherwise use the defaults (vienna convention style and km/h)
          step->emplace("speedLimitSign", kSpeedLimitSignVienna);
          step->emplace("speedLimitUnit", kSpeedLimitUnitsKph);
        }
      }

      rotary = ((maneuver.type() == DirectionsLeg_Maneuver_Type_kRoundaboutEnter) &&
                (maneuver.street_name_size() > 0));
      if (rotary) {
        step->emplace("rotary_name", maneuver.street_name(0).value());
      }

      // Add OSRM maneuver
      step->emplace("maneuver",
                    osrm_maneuver(maneuver, &etp, shape[maneuver.begin_shape_index()],
                                  depart_maneuver, arrive_maneuver, prev_intersection_count, mode,
                                  prev_mode, rotary, prev_rotary, options));

      // Add destinations
      const auto& sign = maneuver.sign();
      std::string dest = destinations(sign);
      if (!dest.empty()) {
        step->emplace("destinations", dest);
        // If the maneuver is an exit roundabout
        // and the previous maneuver is an enter roundabout
        // then set the destinations on the previous step
        if ((maneuver.type() == DirectionsLeg_Maneuver_Type_kRoundaboutExit) && prev_maneuver &&
            (prev_maneuver->type() == DirectionsLeg_Maneuver_Type_kRoundaboutEnter) && prev_step) {
          prev_step->emplace("destinations", dest);
        }
      }

      // Add exits
      std::string ex = exits(sign);
      if (!ex.empty()) {
        step->emplace("exits", ex);
      }

      // Add junction_name if not the start maneuver
      std::string junction_name = get_sign_elements(sign.junction_names());
      if (!depart_maneuver && !junction_name.empty()) {
        step->emplace("junction_name", junction_name);
      }

      // If the user requested guidance_views
      if (options.guidance_views()) {
        // Add guidance_views if not the start maneuver
        if (!depart_maneuver && (maneuver.guidance_views_size() > 0)) {
          auto guidance_views = json::array({});
          guidance_views->reserve(maneuver.guidance_views_size());
          for (const auto& gv : maneuver.guidance_views()) {
            auto guidance_view = json::map({});
            guidance_view->emplace("data_id", gv.data_id());
            guidance_view->emplace("type", GuidanceViewTypeToString(gv.type()));
            guidance_view->emplace("base_id", gv.base_id());
            auto overlay_ids = json::array({});
            overlay_ids->reserve(gv.overlay_ids_size());
            for (const auto& overlay : gv.overlay_ids()) {
              overlay_ids->emplace_back(overlay);
            }
            guidance_view->emplace("overlay_ids", std::move(overlay_ids));

            // Append to guidance view list
            guidance_views->emplace_back(std::move(guidance_view));
          }
          // Add guidance views to step
          step->emplace("guidance_views", std::move(guidance_views));
        }
      }

      // Add intersections
      step->emplace("intersections",
                    intersections(maneuver, &etp, shape, prev_intersection_count, arrive_maneuver));

      // Add step
      prev_rotary = rotary;
      prev_mode = mode;
      prev_step = step;
      prev_maneuver = &maneuver;
      maneuver_index++;
      steps->emplace_back(std::move(step));
    } // end maneuver loop
    //#########################################################################

    // Add distance, duration, weight, and summary
    // Get a summary based on longest maneuvers.
    double duration = leg->summary().time();
    double distance = units_to_meters(leg->summary().length(), !imperial);
    output_leg->emplace("summary", leg_summaries[leg_index]);
    output_leg->emplace("distance", json::fixed_t{distance, 3});
    output_leg->emplace("duration", json::fixed_t{duration, 3});
    output_leg->emplace("weight",
                        json::fixed_t{path_leg.node().rbegin()->cost().elapsed_cost().cost(), 3});
    auto recost_itr = options.recostings().begin();
    for (const auto& recost : path_leg.node().rbegin()->recosts()) {
      if (recost.has_elapsed_cost()) {
        output_leg->emplace("duration_" + recost_itr->name(),
                            json::fixed_t{recost.elapsed_cost().seconds(), 3});
        output_leg->emplace("weight_" + recost_itr->name(),
                            json::fixed_t{recost.elapsed_cost().cost(), 3});
      } else {
        output_leg->emplace("duration_" + recost_itr->name(), nullptr_t());
        output_leg->emplace("weight_" + recost_itr->name(), nullptr_t());
      }
      ++recost_itr;
    }

    // Add admin country codes to leg json
    auto admins = json::array({});
    admins->reserve(path_leg.admin_size());
    for (const auto& admin : path_leg.admin()) {
      auto admin_map = json::map({});
      if (admin.has_country_code()) {
        admin_map->emplace("iso_3166_1", admin.country_code());
        auto country_iso3 = valhalla::baldr::get_iso_3166_1_alpha3(admin.country_code());
        if (!country_iso3.empty()) {
          admin_map->emplace("iso_3166_1_alpha3", country_iso3);
        }
      }
      // TODO: iso_3166_2 state code
      admins->push_back(admin_map);
    }
    output_leg->emplace("admins", std::move(admins));

    // Add steps to the leg
    output_leg->emplace("steps", std::move(steps));

    // Add shape_attributes, if requested
    if (path_leg.has_shape_attributes()) {
      output_leg->emplace("annotation", serialize_annotations(path_leg));
    }

    // Add via waypoints to the leg
    output_leg->emplace("via_waypoints", osrm::intermediate_waypoints(path_leg));

    // Add incidents to the leg
    serializeIncidents(path_leg.incidents(), *output_leg);

    // Add closures
    serializeClosures(path_leg, *output_leg);

    // Keep the leg
    output_legs->emplace_back(std::move(output_leg));
    leg++;
    leg_index++;
  }
  return output_legs;
}

std::vector<std::vector<std::string>>
summarize_route_legs(const google::protobuf::RepeatedPtrField<DirectionsRoute>& routes) {

  route_summary_cache rscache(routes);

  // vector 1: routes
  // vector 2: legs
  // string: unique summary for the route/leg
  std::vector<std::vector<std::string>> all_summaries;
  all_summaries.reserve(routes.size());

  // Find the simplest summary for every leg of every route. Important note:
  // each route should have the same number of legs. Hence, we only need to make
  // unique the same leg (leg_idx) between all routes.
  for (size_t route_i = 0; route_i < routes.size(); route_i++) {

    size_t num_legs_i = routes.Get(route_i).legs_size();
    std::vector<std::string> leg_summaries;
    leg_summaries.reserve(num_legs_i);

    for (size_t leg_idx = 0; leg_idx < num_legs_i; leg_idx++) {

      // we desire each summary to be comprised of at least two named segments.
      // however, if only one is available that's all we can use.
      size_t num_named_segments_i = rscache.num_named_segments_for_route_leg(route_i, leg_idx);
      size_t num_named_segs_needed = std::min(MAX_USED_SEGMENTS, num_named_segments_i);

      // Compare every jth route/leg summary vs the current ith route/leg summary.
      // We desire to compute num_named_segs_needed, which is the number of named
      // segments needed to uniquely identify the ith's summary.
      for (size_t route_j = 0; route_j < routes.size(); route_j++) {

        // avoid self
        if (route_i == route_j)
          continue;

        size_t num_legs_j = routes.Get(route_j).legs_size();

        // there should be the same number of legs in every route. however, some
        // unit tests break this rule, so we cannot enable this assert.
        // assert(num_legs_i == num_legs_j);
        if (leg_idx >= num_legs_j)
          continue;

        size_t num_named_segments_j = rscache.num_named_segments_for_route_leg(route_j, leg_idx);

        size_t num_comparable = std::min(num_named_segments_i, num_named_segments_j);

        // k is the number of named segments in the summary. It keeps going
        // up by 1 until route_i's summary is different than the route_j's.
        size_t k = std::min(num_named_segs_needed, num_comparable);
        for (; (k < num_comparable); k++) {
          const std::string& summary_i = rscache.get_n_segment_summary(route_i, leg_idx, k);
          const std::string& summary_j = rscache.get_n_segment_summary(route_j, leg_idx, k);
          if (summary_i != summary_j)
            break;
        }

        if (k > num_named_segs_needed) {
          num_named_segs_needed = k;
        }
      }

      std::string leg_summary =
          rscache.get_n_segment_summary(route_i, leg_idx, num_named_segs_needed);

      leg_summaries.emplace_back(std::move(leg_summary));
    }

    all_summaries.emplace_back(std::move(leg_summaries));
  }

  return all_summaries;
}

// Serialize route response in OSRM compatible format.
// Inputs are:
//     directions options
//     TripLeg protocol buffer
//     DirectionsLeg protocol buffer
std::string serialize(valhalla::Api& api) {
  auto& options = *api.mutable_options();
  auto json = json::map({});

  // If here then the route succeeded. Set status code to OK and serialize waypoints (locations).
  std::string status("Ok");
  json->emplace("code", status);
  switch (options.action()) {
    case valhalla::Options::trace_route:
      json->emplace("tracepoints", osrm::waypoints(options.shape(), true));
      break;
    case valhalla::Options::route:
      json->emplace("waypoints", osrm::waypoints(api.trip()));
      break;
    case valhalla::Options::optimized_route:
      json->emplace("waypoints", waypoints(*options.mutable_locations()));
      break;
    default:
      throw std::runtime_error("Unknown route serialization action");
  }

  // Add each route
  auto routes = json::array({});
  routes->reserve(api.trip().routes_size());

  // OSRM is always using metric for non narrative stuff
  bool imperial = options.units() == Options::miles;

  // 2D (by route, by leg) vector of every route and leg summarized. We cannot
  // generate a route/leg summary in isolation, since this can lead to the same
  // summary for different route/legs. We instead make each summary as simple
  // as possible while also make sure they are unique.
  std::vector<std::vector<std::string>> route_leg_summaries =
      summarize_route_legs(api.directions().routes());

  // For each route...
  for (int i = 0; i < api.trip().routes_size(); ++i) {
    // Create a route to add to the array
    auto route = json::map({});
    route->reserve(10); // some of the things are conditional so we take a swag here

    if (options.action() == Options::trace_route) {
      // NOTE(mookerji): confidence value here is a placeholder for future implementation.
      route->emplace("confidence", json::fixed_t{1, 1});
    }
    // Add linear references, if applicable
    route_references(route, api.trip().routes(i), options);

    // Concatenated route geometry
    route_geometry(route, api.directions().routes(i), options);

    // Other route summary information
    route_summary(route, api, imperial, i);

    // Serialize route legs
    route->emplace("legs", serialize_legs(api.directions().routes(i).legs(), route_leg_summaries[i],
                                          *api.mutable_trip()->mutable_routes(i)->mutable_legs(),
                                          imperial, options));

    routes->emplace_back(std::move(route));
  }

  // Routes are called matchings in osrm map matching mode
  json->emplace(options.action() == valhalla::Options::trace_route ? "matchings" : "routes",
                std::move(routes));

  std::stringstream ss;
  ss << *json;
  return ss.str();
}

} // namespace osrm_serializers

#ifdef INLINE_TEST

using namespace osrm_serializers;

/// Assert equality of two json documents
//
// TODO Improve the diffed view of mis-matching documents
void assert_json_equality(const rapidjson::Document& doc1, const rapidjson::Document& doc2) {
  if (doc1 != doc2) {
    ASSERT_STREQ(rapidjson::serialize(doc1).c_str(), rapidjson::serialize(doc2).c_str());
  }
}

TEST(RouteSerializerOsrm, testserializeIncidents) {
  // Test that an incident is added correctly to the intersections-json

  rapidjson::Document serialized_to_json;
  {
    auto intersection_doc = json::Jmap();
    auto leg = TripLeg();
    // Sets up the incident
    auto incidents = leg.mutable_incidents();
    auto* incident = incidents->Add();
    incident->set_begin_shape_index(42);
    incident->set_end_shape_index(42);

    valhalla::IncidentsTile::Metadata meta;
    meta.set_id(
        // Set a large id that excercises the uint64 serialization
        18446744073709551615u);
    uint64_t creation_time = 1597241829;
    meta.set_creation_time(creation_time);
    meta.set_start_time(creation_time + 100);
    meta.set_end_time(creation_time + 1800);
    meta.set_type(valhalla::IncidentsTile::Metadata::WEATHER);
    meta.set_impact(valhalla::IncidentsTile::Metadata::MAJOR);
    meta.set_description("fooing foo");
    meta.set_long_description("long fooing foo");
    meta.set_sub_type("foo");
    meta.set_sub_type_description("foobar");
    meta.set_road_closed(true);
    meta.set_num_lanes_blocked(2);
    meta.set_length(1337);
    meta.set_clear_lanes("many lanes clear");
    meta.mutable_congestion()->set_value(33);
    meta.add_alertc_codes(11);
    meta.set_iso_3166_1_alpha2("AU");
    meta.set_iso_3166_1_alpha3("AUS");
    *incident->mutable_metadata() = meta;

    // Finally call the function under test to serialize to json
    serializeIncidents(*incidents, intersection_doc);

    // Lastly, convert to rapidjson
    std::stringstream ss;
    ss << intersection_doc;
    serialized_to_json.Parse(ss.str().c_str());
  }

  rapidjson::Document expected_json;
  {
    expected_json.Parse(R"({
      "incidents": [
        {
          "id": "18446744073709551615",
          "type": "weather",
          "iso_3166_1_alpha2": "AU",
          "iso_3166_1_alpha3": "AUS",
          "creation_time": "2020-08-12T14:17:09Z",
          "start_time": "2020-08-12T14:18:49Z",
          "end_time": "2020-08-12T14:47:09Z",
          "impact": "major",
          "description": "fooing foo",
          "long_description": "long fooing foo",
          "sub_type": "foo",
          "sub_type_description": "foobar",
          "alertc_codes": [ 11 ],
          "lanes_blocked": [],
          "num_lanes_blocked": 2,
          "clear_lanes": "many lanes clear",
          "length": 1337,
          "closed": true,
          "congestion": {
            "value": 33
          },
          "geometry_index_start": 42,
          "geometry_index_end": 42
        }
      ]
    })");
    ASSERT_TRUE(expected_json.IsObject());
  }

  assert_json_equality(serialized_to_json, expected_json);
}

TEST(RouteSerializerOsrm, testserializeIncidentsMultipleIncidentsSingleEdge) {
  // Test that multiple incidents on an edge are serialized correctly
  // that only the incident-id is stored in subsequent intersections
  // after the first

  rapidjson::Document serialized_to_json;
  {
    auto intersection_doc = json::Jmap();
    auto leg = TripLeg();
    // Sets up the incident
    auto* incidents = leg.mutable_incidents();
    {
      // First incident
      auto incident = incidents->Add();
      uint64_t creation_time = 1597241829;
      incident->set_begin_shape_index(87);
      incident->set_end_shape_index(92);

      valhalla::IncidentsTile::Metadata meta;
      meta.set_id(1337);
      meta.set_description("Fooo");
      meta.set_creation_time(creation_time);
      meta.set_type(valhalla::IncidentsTile::Metadata::WEATHER);
      meta.set_iso_3166_1_alpha2("SE");
      meta.set_iso_3166_1_alpha3("SWE");
      *incident->mutable_metadata() = meta;
    }
    {
      // second incident
      auto incident = incidents->Add();
      uint64_t creation_time = 1597241800;
      incident->set_begin_shape_index(21);
      incident->set_end_shape_index(104);

      valhalla::IncidentsTile::Metadata meta;
      meta.set_id(2448);
      meta.set_creation_time(creation_time);
      meta.set_start_time(creation_time + 100);
      meta.set_end_time(creation_time + 1800);
      meta.set_type(valhalla::IncidentsTile::Metadata::ACCIDENT);
      meta.set_iso_3166_1_alpha2("SE");
      meta.set_iso_3166_1_alpha3("SWE");
      *incident->mutable_metadata() = meta;
    }

    // Finally call the function under test to serialize to json
    serializeIncidents(*incidents, intersection_doc);

    // Lastly, convert to rapidjson
    std::stringstream ss;
    ss << intersection_doc;
    serialized_to_json.Parse(ss.str().c_str());
  }

  rapidjson::Document expected_json;
  {
    expected_json.Parse(R"({
      "incidents": [
        {
          "id": "1337",
          "description": "Fooo",
          "creation_time": "2020-08-12T14:17:09Z",
          "type": "weather",
          "iso_3166_1_alpha2": "SE",
          "iso_3166_1_alpha3": "SWE",
          "lanes_blocked": [],
          "geometry_index_start": 87,
          "geometry_index_end": 92
        },
        {
          "id": "2448",
          "creation_time": "2020-08-12T14:16:40Z",
          "start_time": "2020-08-12T14:18:20Z",
          "end_time": "2020-08-12T14:46:40Z",
          "type": "accident",
          "iso_3166_1_alpha2": "SE",
          "iso_3166_1_alpha3": "SWE",
          "lanes_blocked": [],
          "geometry_index_start": 21,
          "geometry_index_end": 104
        }
      ]
    })");
    // Ensure the json was parsed
    ASSERT_TRUE(expected_json.IsObject());
  }

  assert_json_equality(serialized_to_json, expected_json);
}

TEST(RouteSerializerOsrm, testserializeIncidentsNothingToAdd) {

  rapidjson::Document serialized_to_json;
  {
    auto intersection_doc = json::Jmap();
    auto leg = TripLeg();

    // Finally call the function under test to serialize to json
    serializeIncidents(leg.incidents(), intersection_doc);

    // Lastly, convert to rapidjson
    std::stringstream ss;
    ss << intersection_doc;
    serialized_to_json.Parse(ss.str().c_str());
  }

  rapidjson::Document expected_json;
  {
    expected_json.Parse("{}");
    ASSERT_TRUE(expected_json.IsObject());
  }

  assert_json_equality(serialized_to_json, expected_json);
}

TEST(RouteSerializerOsrm, testserializeAnnotationsEmpty) {
  rapidjson::Document serialized_to_json;
  {
    auto leg = TripLeg();
    json::MapPtr annotations = serialize_annotations(leg);

    std::stringstream ss;
    ss << *annotations;
    serialized_to_json.Parse(ss.str().c_str());
    std::cout << *annotations << std::endl;
  }
  rapidjson::Document expected_json;
  { expected_json.Parse(R"({})"); }

  assert_json_equality(serialized_to_json, expected_json);
}

TEST(RouteSerializerOsrm, testserializeAnnotations) {
  rapidjson::Document serialized_to_json;
  {
    auto leg = TripLeg();
    leg.mutable_shape_attributes()->add_time(1);
    leg.mutable_shape_attributes()->add_length(2);
    leg.mutable_shape_attributes()->add_speed(3);
    auto annotations = serialize_annotations(leg);

    std::stringstream ss;
    ss << *annotations;
    serialized_to_json.Parse(ss.str().c_str());
  }
  rapidjson::Document expected_json;
  {
    expected_json.Parse(R"({
      "duration": [0.001],
      "distance": [0.2],
      "speed": [0.3]
    })");
    ASSERT_TRUE(expected_json.IsObject());
  }

  assert_json_equality(serialized_to_json, expected_json);
}

TEST(RouteSerializerOsrm, testserializeAnnotationsSpeedLimits) {
  rapidjson::Document serialized_to_json;
  {
    auto leg = TripLeg();
    leg.mutable_shape_attributes()->add_speed_limit(30);
    leg.mutable_shape_attributes()->add_speed_limit(255);
    leg.mutable_shape_attributes()->add_speed_limit(0);
    auto annotations = serialize_annotations(leg);

    std::stringstream ss;
    ss << *annotations;
    serialized_to_json.Parse(ss.str().c_str());
  }
  rapidjson::Document expected_json;
  {
    expected_json.Parse(R"({
      "maxspeed": [
        { "speed": 30, "unit": "km/h" },
        { "none": true },
        { "unknown": true }
      ]
    })");
    ASSERT_TRUE(expected_json.IsObject());
  }

  assert_json_equality(serialized_to_json, expected_json);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#else

} // namespace

#endif
