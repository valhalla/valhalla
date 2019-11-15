#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "baldr/json.h"
#include "midgard/encoded.h"
#include "midgard/pointll.h"
#include "midgard/polyline2.h"

#include "odin/enhancedtrippath.h"
#include "odin/util.h"
#include "tyr/serializers.h"

#include "proto/directions.pb.h"
#include "proto/options.pb.h"
#include "proto/trip.pb.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::odin;
using namespace valhalla::tyr;
using namespace std;

namespace {
const std::string kSignElementDelimiter = ", ";
const std::string kDestinationsDelimiter = ": ";

constexpr std::size_t MAX_USED_SEGMENTS = 2;
struct NamedSegment {
  std::string name;
  uint32_t index;
  float distance;
};

constexpr const double COORDINATE_PRECISION = 1e6;
struct Coordinate {
  std::int32_t lng;
  std::int32_t lat;

  Coordinate(const std::int32_t lng_, const std::int32_t lat_) : lng(lng_), lat(lat_) {
  }
};

inline std::int32_t toFixed(const float floating) {
  const auto d = static_cast<double>(floating);
  const auto fixed = static_cast<std::int32_t>(std::round(d * COORDINATE_PRECISION));
  return fixed;
}

inline double toFloating(const std::int32_t fixed) {
  const auto i = static_cast<std::int32_t>(fixed);
  const auto floating = static_cast<double>(i) / COORDINATE_PRECISION;
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

const constexpr float DOUGLAS_PEUCKER_THRESHOLDS[19] = {
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

/**********OLD OSRM CODE - delete
    const std::unordered_map<int, std::string> maneuver_type = {
        { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kNone),             "0"
},//NoTurn = 0, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kContinue), "1"
},//GoStraight, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kBecomes), "1"
},//GoStraight, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kRampStraight), "1"
},//GoStraight, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kStayStraight), "1"
},//GoStraight, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kMerge), "1"
},//GoStraight, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kFerryEnter), "1"
},//GoStraight, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kFerryExit), "1"
},//GoStraight, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kSlightRight), "2"
},//TurnSlightRight, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kRight), "3"
},//TurnRight, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kRampRight), "3"
},//TurnRight, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kExitRight), "3"
},//TurnRight, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kStayRight), "3"
},//TurnRight, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kSharpRight), "4"
},//TurnSharpRight, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kUturnLeft), "5"
},//UTurn, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kUturnRight),       "5"
},//UTurn, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kSharpLeft),        "6"
},//TurnSharpLeft, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kLeft), "7"
},//TurnLeft, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kRampLeft), "7"
},//TurnLeft, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kExitLeft), "7"
},//TurnLeft, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kStayLeft), "7"
},//TurnLeft, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kSlightLeft), "8"
},//TurnSlightLeft,
        //{ static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_k),               "9"
},//ReachViaLocation, {
static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kRoundaboutEnter),  "11"
},//EnterRoundAbout, {
static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kRoundaboutExit),   "12"
},//LeaveRoundAbout,
        //{ static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_k),               "13"
},//StayOnRoundAbout, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kStart), "14"
},//StartAtEndOfStreet, {
static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kStartRight),       "14"
},//StartAtEndOfStreet, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kStartLeft),
"14" },//StartAtEndOfStreet, {
static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kDestination),      "15"
},//ReachedYourDestination, {
static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kDestinationRight), "15"
},//ReachedYourDestination, {
static_cast<int>(valhalla::DirectionsLeg_Maneuver_Type_kDestinationLeft),  "15"
},//ReachedYourDestination,
        //{ static_cast<int>valhalla::DirectionsLeg_Maneuver_Type_k),                "16"
},//EnterAgainstAllowedDirection,
        //{ static_cast<int>valhalla::DirectionsLeg_Maneuver_Type_k),                "17"
},//LeaveAgainstAllowedDirection
    };

    const std::unordered_map<int, std::string> cardinal_direction_string = {
      { static_cast<int>(valhalla::DirectionsLeg_Maneuver_CardinalDirection_kNorth),     "N"
}, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_CardinalDirection_kNorthEast), "NE" },
      { static_cast<int>(valhalla::DirectionsLeg_Maneuver_CardinalDirection_kEast),      "E"
}, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_CardinalDirection_kSouthEast), "SE" },
      { static_cast<int>(valhalla::DirectionsLeg_Maneuver_CardinalDirection_kSouth),     "S"
}, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_CardinalDirection_kSouthWest), "SW" },
      { static_cast<int>(valhalla::DirectionsLeg_Maneuver_CardinalDirection_kWest),      "W"
}, { static_cast<int>(valhalla::DirectionsLeg_Maneuver_CardinalDirection_kNorthWest), "NW" }
    };

    json::ArrayPtr route_instructions(const
google::protobuf::RepeatedPtrField<valhalla::DirectionsLeg>& legs){ auto route_instructions =
json::array({}); for(const auto& leg : legs) { for(const auto& maneuver : leg.maneuver()) {
          //if we dont know the type of maneuver then skip it
          auto maneuver_text = maneuver_type.find(static_cast<int>(maneuver.type()));
          if(maneuver_text == maneuver_type.end())
            continue;

          //length
          std::ostringstream length;
          length << static_cast<uint64_t>(maneuver.length()*1000.f) << "m";

          //json
          route_instructions->emplace_back(json::array({
            maneuver_text->second, //maneuver type
            (maneuver.street_name_size() ? maneuver.street_name(0) : string("")), //street name
            static_cast<uint64_t>(maneuver.length() * 1000.f), //length in meters
            static_cast<uint64_t>(maneuver.begin_shape_index()), //index in the shape
            static_cast<uint64_t>(maneuver.time()), //time in seconds
            length.str(), //length as a string with a unit suffix
            cardinal_direction_string.find(static_cast<int>(maneuver.begin_cardinal_direction()))->second,
// one of: N S E W NW NE SW SE static_cast<uint64_t>(maneuver.begin_heading())
          }));
        }
      }
      return route_instructions;
    }
**/

// Add OSRM route summary information: distance, duration
void route_summary(json::MapPtr& route,
                   const google::protobuf::RepeatedPtrField<valhalla::DirectionsLeg>& legs,
                   bool imperial) {
  // Compute total distance and duration
  float duration = 0.0f;
  float distance = 0.0f;
  for (const auto& leg : legs) {
    distance += leg.summary().length();
    duration += leg.summary().time();
  }

  // Convert distance to meters. Output distance and duration.
  distance *= imperial ? 1609.34f : 1000.0f;
  route->emplace("distance", json::fp_t{distance, 1});
  route->emplace("duration", json::fp_t{duration, 1});

  // TODO - support returning weight based on costing method
  // as well as returning the costing method
  float weight = duration;
  route->emplace("weight", json::fp_t{weight, 1});
  route->emplace("weight_name", std::string("Valhalla default"));
}

// Generate leg shape in geojson format.
json::MapPtr geojson_shape(const std::vector<PointLL> shape) {
  auto geojson = json::map({});
  auto coords = json::array({});
  for (auto p : shape) {
    coords->emplace_back(json::array({json::fp_t{p.lng(), 6}, json::fp_t{p.lat(), 6}}));
  }
  geojson->emplace("type", std::string("LineString"));
  geojson->emplace("coordinates", coords);
  return geojson;
}

// Generate full shape of the route.
std::vector<PointLL>
full_shape(const google::protobuf::RepeatedPtrField<valhalla::DirectionsLeg>& legs,
           const valhalla::Options& options) {
  // If just one leg and it we want polyline6 then we just return the encoded leg shape
  if (legs.size() == 1 && options.shape_format() == polyline6) {
    return midgard::decode<std::vector<PointLL>>(legs.begin()->shape());
  }

  // TODO: there is a tricky way to do this... since the end of each leg is the same as the
  // beginning we essentially could just peel off the first encoded shape point of all the legs (but
  // the first) this way we wouldn't really have to do any decoding (would be far faster). it might
  // even be the case that the string length of the first number is a fixed length (which would be
  // great!) have to have a look should make this a function in midgard probably so the logic is all
  // in the same place
  std::vector<PointLL> decoded;
  for (const auto& leg : legs) {
    auto decoded_leg = midgard::decode<std::vector<PointLL>>(leg.shape());
    decoded.insert(decoded.end(), decoded.size() ? decoded_leg.begin() + 1 : decoded_leg.begin(),
                   decoded_leg.end());
  }
  return decoded;
}

// Generate simplified shape of the route.
std::vector<PointLL>
simplified_shape(const google::protobuf::RepeatedPtrField<valhalla::DirectionsLeg>& legs,
                 const valhalla::Options& options) {
  Coordinate south_west(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
  Coordinate north_east(std::numeric_limits<int>::min(), std::numeric_limits<int>::min());

  std::vector<PointLL> simple_shape;
  std::unordered_set<size_t> indices;

  for (const auto& leg : legs) {
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
                    const google::protobuf::RepeatedPtrField<valhalla::DirectionsLeg>& legs,
                    const valhalla::Options& options) {
  // full geom = !has_generalize()
  // simplified geom = has_generalize && generalize == 0
  // no geom = has_generalize && generalize == -1
  std::vector<PointLL> shape;
  if (options.has_generalize() && options.generalize() == 0.0f) {
    shape = simplified_shape(legs, options);
  } else if (!options.has_generalize() || (options.has_generalize() && options.generalize() > 0.0f)) {
    shape = full_shape(legs, options);
  }

  if (options.shape_format() == geojson) {
    route->emplace("geometry", geojson_shape(shape));
  } else {
    int precision = options.shape_format() == polyline6 ? 1e6 : 1e5;
    route->emplace("geometry", midgard::encode(shape, precision));
  }
}

// Serialize waypoints for optimized route. Note that OSRM retains the
// original location order, and stores an index for the waypoint index in
// the optimized sequence.
json::ArrayPtr waypoints(const google::protobuf::RepeatedPtrField<valhalla::Location>& locs) {
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
    waypoints->emplace_back(osrm::waypoint(locs.Get(index), false, true, index));
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
    PointLL ll = arrive_maneuver ? shape[shape.size() - 1] : shape[curr_edge->begin_shape_index()];
    loc->emplace_back(json::fp_t{ll.lng(), 6});
    loc->emplace_back(json::fp_t{ll.lat(), 6});
    intersection->emplace("location", loc);

    // Get bearings and access to outgoing intersecting edges. Do not add
    // any intersecting edges for the first depart intersection and for
    // the arrive step.
    std::vector<IntersectionEdges> edges;
    if (i > 0 && !arrive_maneuver) {
      for (uint32_t m = 0; m < node->intersecting_edge_size(); m++) {
        auto intersecting_edge = node->GetIntersectingEdge(m);
        bool routeable = intersecting_edge->IsTraversableOutbound(curr_edge->travel_mode());
        uint32_t bearing = static_cast<uint32_t>(intersecting_edge->begin_heading());
        edges.emplace_back(bearing, routeable, false, false);
      }
    }

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
      // TODO We might want to have more specific logic for ramp
      if ((curr_edge->road_class() == TripLeg_RoadClass_kMotorway) || curr_edge->IsRampUse()) {
        classes.push_back("motorway");
      }
      if (curr_edge->use() == TripLeg::Use::TripLeg_Use_kFerryUse) {
        classes.push_back("ferry");
      }

      /** TODO
      if ( ) {
        classes.push_back("restricted");
      } */
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
        // Process 'valid' flag
        lane->emplace("valid", turn_lane.is_active());

        // Process 'indications' array - add indications from left to right
        auto indications = json::array({});
        uint16_t mask = turn_lane.directions_mask();

        // TODO make map for lane mask to osrm indication string
        // reverse (left u-turn)
        if ((mask & kTurnLaneReverse) &&
            (maneuver.type() == DirectionsLeg_Maneuver_Type_kUturnLeft)) {
          indications->emplace_back(std::string("uturn"));
        }
        // sharp_left
        if (mask & kTurnLaneSharpLeft) {
          indications->emplace_back(std::string("sharp left"));
        }
        // left
        if (mask & kTurnLaneLeft) {
          indications->emplace_back(std::string("left"));
        }
        // slight_left
        if (mask & kTurnLaneSlightLeft) {
          indications->emplace_back(std::string("slight left"));
        }
        // through
        if (mask & kTurnLaneThrough) {
          indications->emplace_back(std::string("straight"));
        }
        // slight_right
        if (mask & kTurnLaneSlightRight) {
          indications->emplace_back(std::string("slight right"));
        }
        // right
        if (mask & kTurnLaneRight) {
          indications->emplace_back(std::string("right"));
        }
        // sharp_right
        if (mask & kTurnLaneSharpRight) {
          indications->emplace_back(std::string("sharp right"));
        }
        // reverse (right u-turn)
        if ((mask & kTurnLaneReverse) &&
            (maneuver.type() == DirectionsLeg_Maneuver_Type_kUturnRight)) {
          indications->emplace_back(std::string("uturn"));
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
std::string exits(const valhalla::DirectionsLeg_Maneuver_Sign& sign) {
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

// Compile and return the refs of the specified list
// TODO we could enhance by limiting results by using consecutive count
std::string get_sign_element_refs(const google::protobuf::RepeatedPtrField<
                                      ::valhalla::DirectionsLeg_Maneuver_SignElement>& sign_elements,
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
    const google::protobuf::RepeatedPtrField<::valhalla::DirectionsLeg_Maneuver_SignElement>&
        sign_elements,
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

// Add destinations along a step/maneuver. Constructs a destinations string.
// Here are the destinations formats:
//   1. <ref>
//   2. <non-ref>
//   3. <ref>: <non-ref>
// Each <ref> or <non-ref> could have one or more items and will separated with ", "
//   for example: "I 99, US 220, US 30: Altoona, Johnstown"
std::string destinations(const valhalla::DirectionsLeg_Maneuver_Sign& sign) {

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

// Get the turn modifier based on incoming edge bearing and outgoing edge
// bearing.
std::string turn_modifier(const uint32_t in_brg, const uint32_t out_brg) {
  auto turn_degree = GetTurnDegree(in_brg, out_brg);
  auto turn_type = Turn::GetType(turn_degree);
  switch (turn_type) {
    case baldr::Turn::Type::kStraight:
      return "straight";
    case baldr::Turn::Type::kSlightRight:
      return "slight right";
    case baldr::Turn::Type::kRight:
      return "right";
    case baldr::Turn::Type::kSharpRight:
      return "sharp right";
    case baldr::Turn::Type::kReverse:
      return "uturn";
    case baldr::Turn::Type::kSharpLeft:
      return "sharp left";
    case baldr::Turn::Type::kLeft:
      return "left";
    case baldr::Turn::Type::kSlightLeft:
      return "slight left";
  }
  auto num = static_cast<uint32_t>(turn_type);
  throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) +
                           " Unhandled Turn::Type: " + std::to_string(num));
}

// Get the turn modifier based on the maneuver type
// or if needed, the incoming edge bearing and outgoing edge bearing.
std::string turn_modifier(const valhalla::DirectionsLeg::Maneuver& maneuver,
                          valhalla::odin::EnhancedTripLeg* etp,
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
      return "slight right";
    case valhalla::DirectionsLeg_Maneuver_Type_kRight:
    case valhalla::DirectionsLeg_Maneuver_Type_kStartRight:
    case valhalla::DirectionsLeg_Maneuver_Type_kDestinationRight:
      return "right";
    case valhalla::DirectionsLeg_Maneuver_Type_kSharpRight:
      return "sharp right";
    case valhalla::DirectionsLeg_Maneuver_Type_kUturnRight:
    case valhalla::DirectionsLeg_Maneuver_Type_kUturnLeft:
      // [TODO #1789] route ending in uturn should not set modifier=uturn
      if (arrive_maneuver)
        return "";
      return "uturn";
    case valhalla::DirectionsLeg_Maneuver_Type_kSharpLeft:
      return "sharp left";
    case valhalla::DirectionsLeg_Maneuver_Type_kLeft:
    case valhalla::DirectionsLeg_Maneuver_Type_kStartLeft:
    case valhalla::DirectionsLeg_Maneuver_Type_kDestinationLeft:
      return "left";
    case valhalla::DirectionsLeg_Maneuver_Type_kSlightLeft:
    case valhalla::DirectionsLeg_Maneuver_Type_kStayLeft:
    case valhalla::DirectionsLeg_Maneuver_Type_kExitLeft:
    case valhalla::DirectionsLeg_Maneuver_Type_kMergeLeft:
      return "slight left";
    case valhalla::DirectionsLeg_Maneuver_Type_kRampRight:
      if (Turn::GetType(GetTurnDegree(in_brg, out_brg)) == baldr::Turn::Type::kRight)
        return "right";
      else
        return "slight right";
    case valhalla::DirectionsLeg_Maneuver_Type_kRampLeft:
      if (Turn::GetType(GetTurnDegree(in_brg, out_brg)) == baldr::Turn::Type::kLeft)
        return "left";
      else
        return "slight left";
    case valhalla::DirectionsLeg_Maneuver_Type_kRoundaboutEnter:
    case valhalla::DirectionsLeg_Maneuver_Type_kRoundaboutExit:
    case valhalla::DirectionsLeg_Maneuver_Type_kFerryEnter:
    case valhalla::DirectionsLeg_Maneuver_Type_kFerryExit:
      return turn_modifier(in_brg, out_brg);
    default:
      return "straight";
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
                           const bool prev_rotary) {
  auto osrm_man = json::map({});

  // Set the location
  auto loc = json::array({});
  loc->emplace_back(json::fp_t{man_ll.lng(), 6});
  loc->emplace_back(json::fp_t{man_ll.lat(), 6});
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
    modifier = turn_modifier(maneuver, etp, in_brg, out_brg, arrive_maneuver);
    if (!modifier.empty())
      osrm_man->emplace("modifier", modifier);
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
        if ((modifier != "uturn") && (!maneuver.to_stay_on())) {
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

// This is an initial implementation to be as good or better than the current OSRM response
// In the future we shall use the percent of name distance as compared to the total distance
// to determine how many named segments to display.
// Also, might need to combine some similar named segments
std::string
summarize_leg(google::protobuf::RepeatedPtrField<valhalla::DirectionsLeg>::const_iterator leg) {
  // Create a map of maneuver names to index,distance pairs
  std::unordered_map<std::string, std::pair<uint32_t, float>> maneuver_summary_map;
  uint32_t maneuver_index = 0;
  for (const auto& maneuver : leg->maneuver()) {
    if (maneuver.street_name_size() > 0) {
      const std::string& name = maneuver.street_name(0).value();
      auto maneuver_summary = maneuver_summary_map.find(name);
      if (maneuver_summary == maneuver_summary_map.end()) {
        maneuver_summary_map[name] = std::make_pair(maneuver_index, maneuver.length());
      } else {
        maneuver_summary->second.second += maneuver.length();
      }
    }
    // Increment maneuver index
    ++maneuver_index;
  }

  // Create a list of named segments (maneuver name, index, distance items)
  std::vector<NamedSegment> named_segments;
  for (const auto map_item : maneuver_summary_map) {
    named_segments.emplace_back(
        NamedSegment{map_item.first, map_item.second.first, map_item.second.second});
  }

  // Sort list by descending maneuver distance
  std::sort(named_segments.begin(), named_segments.end(),
            [](const NamedSegment& a, const NamedSegment& b) { return b.distance < a.distance; });

  // Reduce the list size to the summary list max
  named_segments.resize(std::min(named_segments.size(), MAX_USED_SEGMENTS));

  // Sort final list by ascending maneuver index
  std::sort(named_segments.begin(), named_segments.end(),
            [](const NamedSegment& a, const NamedSegment& b) { return a.index < b.index; });

  // Create single summary string from list
  std::stringstream ss;
  for (size_t i = 0; i < named_segments.size(); ++i) {
    if (i != 0)
      ss << ", ";
    ss << named_segments[i].name;
  }

  return ss.str();
}

// Serialize each leg
json::ArrayPtr serialize_legs(const google::protobuf::RepeatedPtrField<valhalla::DirectionsLeg>& legs,
                              google::protobuf::RepeatedPtrField<valhalla::TripLeg>& path_legs,
                              bool imperial,
                              const valhalla::Options& options) {
  auto output_legs = json::array({});

  // Verify that the path_legs list is the same size as the legs list
  if (legs.size() != path_legs.size()) {
    throw valhalla_exception_t{503};
  }

  // Iterate through the legs in DirectionsLeg and TripLeg
  auto leg = legs.begin();
  for (auto& path_leg : path_legs) {
    valhalla::odin::EnhancedTripLeg etp(path_leg);
    auto output_leg = json::map({});

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
    for (const auto& maneuver : leg->maneuver()) {
      auto step = json::map({});
      bool depart_maneuver = (maneuver_index == 0);
      bool arrive_maneuver = (maneuver_index == leg->maneuver_size() - 1);

      // TODO - iterate through TripLeg from prior maneuver end to
      // end of this maneuver - perhaps insert OSRM specific steps such as
      // name change

      // Add geometry for this maneuver
      maneuver_geometry(step, maneuver.begin_shape_index(), maneuver.end_shape_index(), shape,
                        arrive_maneuver, options);

      // Add mode, driving side, weight, distance, duration, name
      float distance = maneuver.length() * (imperial ? 1609.34f : 1000.0f);
      float duration = maneuver.time();

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
      step->emplace("duration", json::fp_t{duration, 1});
      step->emplace("weight", json::fp_t{duration, 1});
      step->emplace("distance", json::fp_t{distance, 1});
      step->emplace("name", name);
      if (!ref.empty()) {
        step->emplace("ref", ref);
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
                                  prev_mode, rotary, prev_rotary));

      // Add destinations and exits
      const auto& sign = maneuver.sign();
      std::string dest = destinations(sign);
      if (!dest.empty()) {
        step->emplace("destinations", dest);
      }
      std::string ex = exits(sign);
      if (!ex.empty()) {
        step->emplace("exits", ex);
      }

      // Add intersections
      step->emplace("intersections",
                    intersections(maneuver, &etp, shape, prev_intersection_count, arrive_maneuver));

      // Add step
      steps->emplace_back(step);
      prev_rotary = rotary;
      prev_mode = mode;
      maneuver_index++;
    } // end maneuver loop
    //#########################################################################

    // Add distance, duration, weight, and summary
    // Get a summary based on longest maneuvers.
    float duration = leg->summary().time();
    float distance = leg->summary().length() * (imperial ? 1609.34f : 1000.0f);
    output_leg->emplace("summary", summarize_leg(leg));
    output_leg->emplace("distance", json::fp_t{distance, 1});
    output_leg->emplace("duration", json::fp_t{duration, 1});
    output_leg->emplace("weight", json::fp_t{duration, 1});

    // Add steps to the leg
    output_leg->emplace("steps", steps);
    output_legs->emplace_back(output_leg);
    leg++;
  }
  return output_legs;
}

// Serialize route response in OSRM compatible format.
// Inputs are:
//     directions options
//     TripLeg protocol buffer
//     DirectionsLeg protocol buffer
std::string serialize(valhalla::Api& api) {
  const auto& options = api.options();
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
      json->emplace("waypoints", waypoints(options.locations()));
      break;
  }

  // Add each route
  auto routes = json::array({});

  // OSRM is always using metric for non narrative stuff
  bool imperial = options.units() == Options::miles;

  // For each route...
  for (int i = 0; i < api.trip().routes_size(); ++i) {
    // Create a route to add to the array
    auto route = json::map({});

    // TODO: phase 1, just hardcode score. phase 2: do real implementation
    if (options.action() == valhalla::Options::trace_route)
      route->emplace("confidence", json::fp_t{1, 1});

    // Concatenated route geometry
    route_geometry(route, api.directions().routes(i).legs(), options);

    // Other route summary information
    route_summary(route, api.directions().routes(i).legs(), imperial);

    // Serialize route legs
    route->emplace("legs", serialize_legs(api.directions().routes(i).legs(),
                                          *api.mutable_trip()->mutable_routes(i)->mutable_legs(),
                                          imperial, options));

    routes->emplace_back(route);
  }

  // Routes are called matchings in osrm map matching mode
  json->emplace(options.action() == valhalla::Options::trace_route ? "matchings" : "routes", routes);

  std::stringstream ss;
  ss << *json;
  return ss.str();
}

} // namespace osrm_serializers
} // namespace
