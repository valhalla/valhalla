#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "baldr/json.h"
#include "midgard/encoded.h"
#include "midgard/pointll.h"
#include "odin/enhancedtrippath.h"
#include "odin/util.h"
#include "tyr/serializers.h"

#include "proto/directions_options.pb.h"
#include "proto/tripdirections.pb.h"
#include "proto/trippath.pb.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::odin;
using namespace valhalla::tyr;
using namespace std;

namespace {
const std::string kSignElementDelimiter = ", ";
const std::string kDestinationsDelimiter = ": ";

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
        { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kNone),             "0"
},//NoTurn = 0, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kContinue), "1"
},//GoStraight, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kBecomes), "1"
},//GoStraight, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kRampStraight), "1"
},//GoStraight, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kStayStraight), "1"
},//GoStraight, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kMerge), "1"
},//GoStraight, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kFerryEnter), "1"
},//GoStraight, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kFerryExit), "1"
},//GoStraight, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kSlightRight), "2"
},//TurnSlightRight, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kRight), "3"
},//TurnRight, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kRampRight), "3"
},//TurnRight, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kExitRight), "3"
},//TurnRight, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kStayRight), "3"
},//TurnRight, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kSharpRight), "4"
},//TurnSharpRight, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kUturnLeft), "5"
},//UTurn, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kUturnRight),       "5"
},//UTurn, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kSharpLeft),        "6"
},//TurnSharpLeft, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kLeft), "7"
},//TurnLeft, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kRampLeft), "7"
},//TurnLeft, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kExitLeft), "7"
},//TurnLeft, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kStayLeft), "7"
},//TurnLeft, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kSlightLeft), "8"
},//TurnSlightLeft,
        //{ static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_k),               "9"
},//ReachViaLocation, {
static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kRoundaboutEnter),  "11"
},//EnterRoundAbout, {
static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kRoundaboutExit),   "12"
},//LeaveRoundAbout,
        //{ static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_k),               "13"
},//StayOnRoundAbout, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kStart), "14"
},//StartAtEndOfStreet, {
static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kStartRight),       "14"
},//StartAtEndOfStreet, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kStartLeft),
"14" },//StartAtEndOfStreet, {
static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kDestination),      "15"
},//ReachedYourDestination, {
static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kDestinationRight), "15"
},//ReachedYourDestination, {
static_cast<int>(valhalla::odin::TripDirections_Maneuver_Type_kDestinationLeft),  "15"
},//ReachedYourDestination,
        //{ static_cast<int>valhalla::odin::TripDirections_Maneuver_Type_k),                "16"
},//EnterAgainstAllowedDirection,
        //{ static_cast<int>valhalla::odin::TripDirections_Maneuver_Type_k),                "17"
},//LeaveAgainstAllowedDirection
    };

    const std::unordered_map<int, std::string> cardinal_direction_string = {
      { static_cast<int>(valhalla::odin::TripDirections_Maneuver_CardinalDirection_kNorth),     "N"
}, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_CardinalDirection_kNorthEast), "NE" },
      { static_cast<int>(valhalla::odin::TripDirections_Maneuver_CardinalDirection_kEast),      "E"
}, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_CardinalDirection_kSouthEast), "SE" },
      { static_cast<int>(valhalla::odin::TripDirections_Maneuver_CardinalDirection_kSouth),     "S"
}, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_CardinalDirection_kSouthWest), "SW" },
      { static_cast<int>(valhalla::odin::TripDirections_Maneuver_CardinalDirection_kWest),      "W"
}, { static_cast<int>(valhalla::odin::TripDirections_Maneuver_CardinalDirection_kNorthWest), "NW" }
    };

    json::ArrayPtr route_instructions(const std::list<valhalla::odin::TripDirections>& legs){
      auto route_instructions = json::array({});
      for(const auto& leg : legs) {
        for(const auto& maneuver : leg.maneuver()) {
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
                   const std::list<valhalla::odin::TripDirections>& legs,
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

// Generate full shape of the route. TODO - different encodings, generalization
std::string full_shape(const std::list<valhalla::odin::TripDirections>& legs,
                       const valhalla::odin::DirectionsOptions& directions_options) {
  // TODO - support generalization

  // If just one leg and it we want polyline6 then we just return the encoded leg shape
  if (legs.size() == 1 && directions_options.shape_format() == odin::polyline6) {
    return legs.front().shape();
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
  int precision = directions_options.shape_format() == odin::polyline6 ? 1e6 : 1e5;
  return midgard::encode(decoded, precision);
}

// Serialize waypoints for optimized route. Note that OSRM retains the
// original location order, and stores an index for the waypoint index in
// the optimized sequence.
json::ArrayPtr waypoints(const google::protobuf::RepeatedPtrField<odin::Location>& locs) {
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
json::ArrayPtr intersections(const valhalla::odin::TripDirections::Maneuver& maneuver,
                             valhalla::odin::EnhancedTripPath* etp,
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
    auto* node = etp->GetEnhancedNode(i);
    auto* curr_edge = etp->GetCurrEdge(i);

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
      for (uint32_t n = 0; n < node->intersecting_edge_size(); n++) {
        auto* intersecting_edge = node->GetIntersectingEdge(n);
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
      uint32_t prior_heading = etp->GetPrevEdge(i)->end_heading();
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
      if ((curr_edge->road_class() == odin::TripPath_RoadClass_kMotorway) || curr_edge->IsRampUse()) {
        classes.push_back("motorway");
      }
      if (curr_edge->use() == odin::TripPath::Use::TripPath_Use_kFerryUse) {
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

    // Add the intersection to the JSON array
    intersections->emplace_back(intersection);
    count++;
  }
  return intersections;
}

// Add exits (exit numbers) along a step/maneuver.
std::string exits(const valhalla::odin::TripDirections_Maneuver_Sign& sign) {
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
std::string get_sign_element_refs(
    const google::protobuf::RepeatedPtrField<::valhalla::odin::TripDirections_Maneuver_SignElement>&
        sign_elements,
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
    const google::protobuf::RepeatedPtrField<::valhalla::odin::TripDirections_Maneuver_SignElement>&
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
std::string destinations(const valhalla::odin::TripDirections_Maneuver_Sign& sign) {

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
}

// Get the turn modifier based on the maneuver type
// or if needed, the incoming edge bearing and outgoing edge bearing.
std::string turn_modifier(const valhalla::odin::TripDirections::Maneuver& maneuver,
                          const uint32_t in_brg,
                          const uint32_t out_brg) {
  switch (maneuver.type()) {
    case valhalla::odin::TripDirections_Maneuver_Type_kStart:
    case valhalla::odin::TripDirections_Maneuver_Type_kDestination:
      return "";
    case valhalla::odin::TripDirections_Maneuver_Type_kSlightRight:
    case valhalla::odin::TripDirections_Maneuver_Type_kStayRight:
    case valhalla::odin::TripDirections_Maneuver_Type_kExitRight:
      return "slight right";
    case valhalla::odin::TripDirections_Maneuver_Type_kRight:
    case valhalla::odin::TripDirections_Maneuver_Type_kStartRight:
    case valhalla::odin::TripDirections_Maneuver_Type_kDestinationRight:
      return "right";
    case valhalla::odin::TripDirections_Maneuver_Type_kSharpRight:
      return "sharp right";
    case valhalla::odin::TripDirections_Maneuver_Type_kUturnRight:
    case valhalla::odin::TripDirections_Maneuver_Type_kUturnLeft:
      return "uturn";
    case valhalla::odin::TripDirections_Maneuver_Type_kSharpLeft:
      return "sharp left";
    case valhalla::odin::TripDirections_Maneuver_Type_kLeft:
    case valhalla::odin::TripDirections_Maneuver_Type_kStartLeft:
    case valhalla::odin::TripDirections_Maneuver_Type_kDestinationLeft:
      return "left";
    case valhalla::odin::TripDirections_Maneuver_Type_kSlightLeft:
    case valhalla::odin::TripDirections_Maneuver_Type_kExitLeft:
    case valhalla::odin::TripDirections_Maneuver_Type_kStayLeft:
      return "slight left";
    case valhalla::odin::TripDirections_Maneuver_Type_kRampRight:
      if (Turn::GetType(GetTurnDegree(in_brg, out_brg)) == baldr::Turn::Type::kRight)
        return "right";
      else
        return "slight right";
    case valhalla::odin::TripDirections_Maneuver_Type_kRampLeft:
      if (Turn::GetType(GetTurnDegree(in_brg, out_brg)) == baldr::Turn::Type::kLeft)
        return "left";
      else
        return "slight left";
    case valhalla::odin::TripDirections_Maneuver_Type_kMerge:
    case valhalla::odin::TripDirections_Maneuver_Type_kRoundaboutEnter:
    case valhalla::odin::TripDirections_Maneuver_Type_kRoundaboutExit:
    case valhalla::odin::TripDirections_Maneuver_Type_kFerryEnter:
    case valhalla::odin::TripDirections_Maneuver_Type_kFerryExit:
      return turn_modifier(in_brg, out_brg);
    default:
      return "straight";
  }
}

// Ramp cases - off ramp transitions from a motorway. On ramp ends
// in a motorway.
// TODO are we able to use the TripDirections_Maneuver_Type ramp/exit classification
std::string ramp_type(valhalla::odin::EnhancedTripPath_Edge* prev_edge,
                      const uint32_t idx,
                      valhalla::odin::EnhancedTripPath* etp) {
  if (prev_edge->use() == odin::TripPath_Use_kRoadUse) {
    if (prev_edge->road_class() == odin::TripPath_RoadClass_kMotorway) {
      return std::string("off ramp");
    } else if (prev_edge->road_class() != odin::TripPath_RoadClass_kMotorway) {
      // Check that next road is a motorway
      for (uint32_t i = idx + 1; i < (etp->node_size() - 1); ++i) {
        auto* curr_edge = etp->GetCurrEdge(i);
        if (curr_edge->use() == odin::TripPath_Use_kRoadUse) {
          if (curr_edge->road_class() == odin::TripPath_RoadClass_kMotorway) {
            return std::string("on ramp");
          }
          break;
        }
      }
    }
  }
  return "";
}

// Populate the OSRM maneuver record within a step.
json::MapPtr osrm_maneuver(const valhalla::odin::TripDirections::Maneuver& maneuver,
                           valhalla::odin::EnhancedTripPath* etp,
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
  // prior edge from the TripPath. Compute turn modifier. TODO - reconcile
  // turn degrees between Valhalla and OSRM
  uint32_t idx = maneuver.begin_path_index();
  uint32_t in_brg = (idx > 0) ? etp->GetPrevEdge(idx)->end_heading() : 0;
  uint32_t out_brg = maneuver.begin_heading();
  osrm_man->emplace("bearing_before", static_cast<uint64_t>(in_brg));
  osrm_man->emplace("bearing_after", static_cast<uint64_t>(out_brg));

  std::string modifier;
  if (!depart_maneuver) {
    modifier = turn_modifier(maneuver, in_brg, out_brg);
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
  } else if (maneuver.type() == odin::TripDirections_Maneuver_Type_kRoundaboutEnter) {
    if (rotary) {
      maneuver_type = "rotary";
    } else {
      maneuver_type = "roundabout";
    }
    // Roundabout count
    if (maneuver.has_roundabout_exit_count()) {
      osrm_man->emplace("exit", static_cast<uint64_t>(maneuver.roundabout_exit_count()));
    }
  } else if (maneuver.type() == odin::TripDirections_Maneuver_Type_kRoundaboutExit) {
    if (prev_rotary) {
      maneuver_type = "exit rotary";
    } else {
      maneuver_type = "exit roundabout";
    }
  } else {
    // Special cases
    auto* prev_edge = etp->GetPrevEdge(idx);
    auto* curr_edge = etp->GetCurrEdge(idx);
    bool new_name = maneuver.type() == odin::TripDirections_Maneuver_Type_kContinue ||
                    maneuver.type() == odin::TripDirections_Maneuver_Type_kBecomes;
    bool ramp = curr_edge->use() == odin::TripPath_Use_kRampUse;
    bool fork = etp->node(idx).fork();
    if (maneuver.type() == odin::TripDirections_Maneuver_Type_kMerge) {
      maneuver_type = "merge";
    } else if (fork) {
      maneuver_type = "fork";
    } else if (ramp) {
      maneuver_type = ramp_type(prev_edge, idx, etp);
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
      bool road_ends =
          (prev_intersection_count > 1 && prev_edge->use() != odin::TripPath_Use_kRampUse &&
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
// TODO - encoding options
std::string maneuver_geometry(const uint32_t begin_idx,
                              const uint32_t end_idx,
                              const std::vector<PointLL>& shape,
                              const valhalla::odin::DirectionsOptions& directions_options) {
  // Must add one to the end range since maneuver end shape index is exclusive
  std::vector<PointLL> maneuver_shape(shape.begin() + begin_idx, shape.begin() + end_idx + 1);
  int precision = directions_options.shape_format() == odin::polyline6 ? 1e6 : 1e5;
  return midgard::encode(maneuver_shape, precision);
}

// Get the mode
std::string get_mode(const valhalla::odin::TripDirections::Maneuver& maneuver,
                     const bool arrive_maneuver,
                     valhalla::odin::EnhancedTripPath* etp) {
  // Return ferry if not last maneuver and the edge use is Ferry
  if (!arrive_maneuver && (etp->GetCurrEdge(maneuver.begin_path_index())->use() ==
                           odin::TripPath::Use::TripPath_Use_kFerryUse)) {
    return "ferry";
  }

  // Otherwise return based on the travel mode
  switch (maneuver.travel_mode()) {
    case TripDirections_TravelMode_kDrive: {
      return "driving";
    }
    case TripDirections_TravelMode_kPedestrian: {
      return "walking";
    }
    case TripDirections_TravelMode_kBicycle: {
      return "cycling";
    }
    case TripDirections_TravelMode_kTransit: {
      return "transit";
    }
  }
}

// Get the names and ref names
std::pair<std::string, std::string>
names_and_refs(const valhalla::odin::TripDirections::Maneuver& maneuver) {
  std::string names, refs;

  // Roundabouts need to use the roundabout_exit_street_names
  auto& street_names = (maneuver.type() == odin::TripDirections_Maneuver_Type_kRoundaboutEnter)
                           ? maneuver.roundabout_exit_street_names()
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

// Add annotations to the leg
json::MapPtr annotations(valhalla::odin::EnhancedTripPath* etp) {
  auto annotations = json::map({});

  // Create distance and duration arrays. Iterate through trip edges and
  // form distance and duration.
  // NOTE: if we need to do per node Id pair we could walk the shape,
  // compute distances, and interpolate durations.
  uint32_t elapsed_time = 0;
  auto distances = json::array({});
  auto durations = json::array({});
  for (uint32_t idx = 0; idx < etp->node_size() - 1; ++idx) {
    distances->emplace_back(json::fp_t{etp->GetCurrEdge(idx)->length() * 1000.0f, 1});
    uint32_t t = etp->node(idx + 1).elapsed_time() > etp->node(idx).elapsed_time()
                     ? etp->node(idx + 1).elapsed_time() - etp->node(idx).elapsed_time()
                     : 0;
    durations->emplace_back(static_cast<uint64_t>(t));
  }

  // Add arrays and return
  annotations->emplace("duration", durations);
  annotations->emplace("distance", distances);
  return annotations;
}

// Serialize each leg
json::ArrayPtr serialize_legs(const std::list<valhalla::odin::TripDirections>& legs,
                              std::list<valhalla::odin::TripPath>& path_legs,
                              bool imperial,
                              const valhalla::odin::DirectionsOptions& directions_options) {
  auto output_legs = json::array({});

  // Verify that the path_legs list is the same size as the legs list
  if (legs.size() != path_legs.size()) {
    throw valhalla_exception_t{503};
  }

  // Iterate through the legs in TripDirections and TripPath
  auto leg = legs.begin();
  for (auto& path_leg : path_legs) {
    valhalla::odin::EnhancedTripPath* etp = static_cast<valhalla::odin::EnhancedTripPath*>(&path_leg);
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
    std::unordered_map<std::string, float> maneuvers;
    for (const auto& maneuver : leg->maneuver()) {
      auto step = json::map({});
      bool depart_maneuver = (maneuver_index == 0);
      bool arrive_maneuver = (maneuver_index == leg->maneuver_size() - 1);

      // TODO - iterate through TripPath from prior maneuver end to
      // end of this maneuver - perhaps insert OSRM specific steps such as
      // name change

      // Add geometry for this maneuver
      step->emplace("geometry",
                    maneuver_geometry(maneuver.begin_shape_index(), maneuver.end_shape_index(), shape,
                                      directions_options));

      // Add mode, driving side, weight, distance, duration, name
      float distance = maneuver.length() * (imperial ? 1609.34f : 1000.0f);
      float duration = maneuver.time();

      // Process drive_side, name, ref, mode, and prev_mode attributes if not the arrive maneuver
      if (!arrive_maneuver) {
        drive_side =
            (etp->GetCurrEdge(maneuver.begin_path_index())->drive_on_right()) ? "right" : "left";
        auto name_ref_pair = names_and_refs(maneuver);
        name = name_ref_pair.first;
        ref = name_ref_pair.second;
        rotary = ((maneuver.type() == TripDirections_Maneuver_Type_kRoundaboutEnter) &&
                  (maneuver.street_name_size() > 0));
        mode = get_mode(maneuver, arrive_maneuver, etp);
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
      if (rotary) {
        step->emplace("rotary_name", maneuver.street_name(0).value());
      }

      // Record street name and distance.. TODO - need to also worry about order
      if (maneuver.street_name_size() > 0) {
        const std::string& name = maneuver.street_name(0).value();
        auto man = maneuvers.find(name);
        if (man == maneuvers.end()) {
          maneuvers[name] = distance;
        } else {
          man->second += distance;
        }
      }

      // Add OSRM maneuver
      step->emplace("maneuver",
                    osrm_maneuver(maneuver, etp, shape[maneuver.begin_shape_index()], depart_maneuver,
                                  arrive_maneuver, prev_intersection_count, mode, prev_mode, rotary,
                                  prev_rotary));

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
                    intersections(maneuver, etp, shape, prev_intersection_count, arrive_maneuver));

      // Add step
      steps->emplace_back(step);
      prev_rotary = rotary;
      prev_mode = mode;
      maneuver_index++;
    } // end maneuver loop
    //#########################################################################

    // Add annotations. Valhalla cannot support node Ids (Valhalla does
    // not store node Ids) but can support distance, duration, speed.
    // NOTE: Valhalla outputs annotations per edge not between node Id
    // pairs like OSRM does.
    // Protect against empty trip path
    if (etp->node_size() > 0) {
      output_leg->emplace("annotation", annotations(etp));
    }

    // Add distance, duration, weight, and summary
    // Get a summary based on longest maneuvers.
    std::string summary = "TODO"; // Form summary from longest maneuvers?
    float duration = leg->summary().time();
    float distance = leg->summary().length() * (imperial ? 1609.34f : 1000.0f);
    output_leg->emplace("summary", summary);
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
//     TripPath protocol buffer
//     TripDirections protocol buffer
std::string serialize(const valhalla::odin::DirectionsOptions& directions_options,
                      std::list<valhalla::odin::TripPath>& path_legs,
                      const std::list<valhalla::odin::TripDirections>& legs) {
  auto json = json::map({});

  // If here then the route succeeded. Set status code to OK and serialize
  // waypoints (locations).
  std::string status("Ok");
  json->emplace("code", status);
  switch (directions_options.action()) {
    case valhalla::odin::DirectionsOptions::trace_route:
      json->emplace("tracepoints", osrm::waypoints(directions_options.locations(), true));
      break;
    case valhalla::odin::DirectionsOptions::route:
      json->emplace("waypoints", osrm::waypoints(directions_options.locations()));
      break;
    case valhalla::odin::DirectionsOptions::optimized_route:
      json->emplace("waypoints", waypoints(directions_options.locations()));
      break;
  }

  // Add each route
  // TODO - alternate routes (currently Valhalla only has 1 route)
  auto routes = json::array({});

  // OSRM is always using metric for non narrative stuff
  bool imperial = directions_options.units() == DirectionsOptions::miles;

  // For each route...
  for (int i = 0; i < 1; ++i) {
    // Create a route to add to the array
    auto route = json::map({});

    // Get full shape for the route.
    route->emplace("geometry", full_shape(legs, directions_options));

    // Other route summary information
    route_summary(route, legs, imperial);

    // Serialize route legs
    route->emplace("legs", serialize_legs(legs, path_legs, imperial, directions_options));

    routes->emplace_back(route);
  }

  // Routes are called matchings in osrm map matching mode
  json->emplace(directions_options.action() == valhalla::odin::DirectionsOptions::trace_route
                    ? "matchings"
                    : "routes",
                routes);

  std::stringstream ss;
  ss << *json;
  return ss.str();
}

} // namespace osrm_serializers
} // namespace
