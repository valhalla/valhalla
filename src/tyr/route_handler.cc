#include "tyr/route_handler.h"

#include <stdexcept>
#include <ostream>
#include <valhalla/proto/trippath.pb.h>
#include <valhalla/proto/tripdirections.pb.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/python/str.hpp>
#include <boost/python/extract.hpp>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/loki/search.h>
#include <valhalla/thor/costfactory.h>
#include <valhalla/thor/autocost.h>
#include <valhalla/thor/bicyclecost.h>
#include <valhalla/thor/pedestriancost.h>
#include <valhalla/thor/pathalgorithm.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/thor/trippathbuilder.h>
#include <valhalla/odin/directionsbuilder.h>
#include <boost/algorithm/string/replace.hpp>

#include "tyr/json.h"


namespace {

/*
OSRM output looks like this:
{
    "hint_data": {
        "locations": [
            "_____38_SADaFQQAKwEAABEAAAAAAAAAdgAAAFfLwga4tW0C4P6W-wAARAA",
            "fzhIAP____8wFAQA1AAAAC8BAAAAAAAAAAAAAP____9Uu20CGAiX-wAAAAA"
        ],
        "checksum": 2875622111
    },
    "route_name": [ "West 26th Street", "Madison Avenue" ],
    "via_indices": [ 0, 9 ],
    "found_alternative": false,
    "route_summary": {
        "end_point": "West 29th Street",
        "start_point": "West 26th Street",
        "total_time": 145,
        "total_distance": 878
    },
    "via_points": [ [ 40.744377, -73.990433 ], [40.745811, -73.988075 ] ],
    "route_instructions": [
        [ "10", "West 26th Street", 216, 0, 52, "215m", "SE", 118 ],
        [ "1", "East 26th Street", 153, 2, 29, "153m", "SE", 120 ],
        [ "7", "Madison Avenue", 237, 3, 25, "236m", "NE", 29 ],
        [ "7", "East 29th Street", 155, 6, 29, "154m", "NW", 299 ],
        [ "1", "West 29th Street", 118, 7, 21, "117m", "NW", 299 ],
        [ "15", "", 0, 8, 0, "0m", "N", 0 ]
    ],
    "route_geometry": "ozyulA~p_clCfc@ywApTar@li@ybBqe@c[ue@e[ue@i[ci@dcB}^rkA",
    "status_message": "Found route between points",
    "status": 0
}
*/
using namespace valhalla::tyr;
using namespace std;

json::JsonObjectPtr route_name(const valhalla::odin::TripDirections& trip_directions){
  auto route_name = json::array({});
  if(trip_directions.maneuver_size() > 0) {
    if(trip_directions.maneuver(0).street_name_size() > 0) {
      route_name->push_back(trip_directions.maneuver(0).street_name(0));
    }
    if(trip_directions.maneuver(trip_directions.maneuver_size() - 1).street_name_size() > 0) {
      route_name->push_back(trip_directions.maneuver(trip_directions.maneuver_size() - 1).street_name(0));
    }
  }
  return route_name;
}

json::JsonObjectPtr via_indices(const valhalla::odin::TripDirections& trip_directions){
  auto via_indices = json::array({});
  if(trip_directions.maneuver_size() > 0) {
    via_indices->push_back(static_cast<uint64_t>(0));
    via_indices->push_back(static_cast<uint64_t>(trip_directions.maneuver_size() - 1));
  }
  return via_indices;
}

json::JsonObjectPtr route_summary(const valhalla::odin::TripDirections& trip_directions){
  auto route_summary = json::map({});
  if(trip_directions.maneuver_size() > 0) {
    if(trip_directions.maneuver(0).street_name_size() > 0)
      route_summary->emplace("start_point", trip_directions.maneuver(0).street_name(0));
    else
      route_summary->emplace("start_point", string(""));
    if(trip_directions.maneuver(trip_directions.maneuver_size() - 1).street_name_size() > 0)
      route_summary->emplace("end_point", trip_directions.maneuver(trip_directions.maneuver_size() - 1).street_name(0));
    else
      route_summary->emplace("end_point", string(""));
  }
  uint64_t seconds = 0, meters = 0;
  for(const auto& maneuver : trip_directions.maneuver()) {
    meters += static_cast<uint64_t>(maneuver.length() * 1000.f);
    seconds + static_cast<uint64_t>(maneuver.time());
  }
  route_summary->emplace("total_time", seconds);
  route_summary->emplace("total_distance", meters);
  return route_summary;
}

json::JsonObjectPtr via_points(const valhalla::odin::TripPath& trip_path){
  auto via_points = json::array({});
  for(const auto& location : trip_path.location()) {
    via_points->emplace_back(json::array({(long double)(location.ll().lat()), (long double)(location.ll().lng())}));
  }
  return via_points;
}

std::string escape(const std::string& unescaped) {
  //these replacements are only useful for polyline encoded shape right now
  std::string escaped = boost::replace_all_copy(unescaped, "\\", "\\\\");
  boost::replace_all(escaped, "\"", "\\\"");
  return escaped;
}

const std::unordered_map<unsigned int, std::string> maneuver_type = {
    { static_cast<unsigned int>(valhalla::odin::TripDirections_Type_kNone),            "0" },//NoTurn = 0,
    { static_cast<unsigned int>(valhalla::odin::TripDirections_Type_kContinue),        "1" },//GoStraight,
    { static_cast<unsigned int>(valhalla::odin::TripDirections_Type_kSlightRight),     "2" },//TurnSlightRight,
    { static_cast<unsigned int>(valhalla::odin::TripDirections_Type_kRight),           "3" },//TurnRight,
    { static_cast<unsigned int>(valhalla::odin::TripDirections_Type_kSharpRight),      "4" },//TurnSharpRight,
    { static_cast<unsigned int>(valhalla::odin::TripDirections_Type_kUturnLeft),       "5" },//UTurn,
    { static_cast<unsigned int>(valhalla::odin::TripDirections_Type_kUturnRight),      "5" },//UTurn,
    { static_cast<unsigned int>(valhalla::odin::TripDirections_Type_kSharpLeft),       "6" },//TurnSharpLeft,
    { static_cast<unsigned int>(valhalla::odin::TripDirections_Type_kLeft),            "7" },//TurnLeft,
    { static_cast<unsigned int>(valhalla::odin::TripDirections_Type_kSlightLeft),      "8" },//TurnSlightLeft,
    //{ static_cast<unsigned int>(valhalla::odin::TripDirections_Type_k),              "9" },//ReachViaLocation,
    { static_cast<unsigned int>(valhalla::odin::TripDirections_Type_kContinue),        "10" },//HeadOn,
    { static_cast<unsigned int>(valhalla::odin::TripDirections_Type_kBecomes),         "10" },//HeadOn,
    //{ static_cast<unsigned int>(valhalla::odin::TripDirections_Type_k),              "11" },//EnterRoundAbout,
    //{ static_cast<unsigned int>(valhalla::odin::TripDirections_Type_k),              "12" },//LeaveRoundAbout,
    //{ static_cast<unsigned int>(valhalla::odin::TripDirections_Type_k),              "13" },//StayOnRoundAbout,
    { static_cast<unsigned int>(valhalla::odin::TripDirections_Type_kStart),           "14" },//StartAtEndOfStreet,
    { static_cast<unsigned int>(valhalla::odin::TripDirections_Type_kStartRight),      "14" },//StartAtEndOfStreet,
    { static_cast<unsigned int>(valhalla::odin::TripDirections_Type_kStartLeft),       "14" },//StartAtEndOfStreet,
    { static_cast<unsigned int>(valhalla::odin::TripDirections_Type_kDestination),     "15" },//ReachedYourDestination,
    { static_cast<unsigned int>(valhalla::odin::TripDirections_Type_kDestinationRight),"15" },//ReachedYourDestination,
    { static_cast<unsigned int>(valhalla::odin::TripDirections_Type_kDestinationLeft), "15" },//ReachedYourDestination,
    //{ static_cast<unsigned int>valhalla::odin::TripDirections_Type_k), 16 },//EnterAgainstAllowedDirection,
    //{ static_cast<unsigned int>valhalla::odin::TripDirections_Type_k), 17 },//LeaveAgainstAllowedDirection
};

json::JsonObjectPtr route_instructions(const valhalla::odin::TripDirections& trip_directions){
  auto route_instructions = json::array({});
  for(const auto& maneuver : trip_directions.maneuver()) {
    //if we dont know the type of maneuver then skip it
    auto maneuver_text = maneuver_type.find(static_cast<unsigned int>(maneuver.type()));
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
      string(""), //TODO: heading as one of: N S E W NW NE SW SE
      static_cast<uint64_t>(maneuver.begin_heading())
    }));
  }
  return route_instructions;
}

std::string serialize(const valhalla::odin::TripPath& trip_path,
  const valhalla::odin::TripDirections& trip_directions) {

  //TODO: worry about multipoint routes


  //build up the json object
  auto json = json::map
  ({
    {"hint_data", json::map
      ({
        {"locations", json::array({ string(""), string("") })}, //TODO: are these internal ids?
        {"checksum", static_cast<uint64_t>(0)} //TODO: what is this exactly?
      })
    },
    {"route_name", route_name(trip_directions)}, //TODO: list of all of the streets or just the via points?
    {"via_indices", via_indices(trip_directions)}, //maneuver index
    {"found_alternative", static_cast<bool>(false)}, //no alt route support
    {"route_summary", route_summary(trip_directions)}, //start/end name, total time/distance
    {"via_points", via_points(trip_path)}, //array of lat,lng pairs
    {"route_instructions", route_instructions(trip_directions)}, //array of maneuvers
    {"route_geometry", escape(trip_path.shape())}, //polyline encoded shape
    {"status_message", string("Found route between points")}, //found route between points OR cannot find route between points
    {"status", static_cast<uint64_t>(0)} //0 success or 207 no route
  });

  //serialize it
  ostringstream stream;
  stream << *json;
  return stream.str();
}

}

namespace valhalla {
namespace tyr {

RouteHandler::RouteHandler(const boost::python::dict& dict_request) : Handler(dict_request) {
  //parse out the type of route
  if(!dict_request.has_key("costing_method"))
    throw std::runtime_error("No edge/node costing method provided");
  //register edge/node costing methods
  valhalla::thor::CostFactory<valhalla::thor::DynamicCost> factory;
  factory.Register("auto", valhalla::thor::CreateAutoCost);
  factory.Register("bicycle", valhalla::thor::CreateBicycleCost);
  factory.Register("pedestrian", valhalla::thor::CreatePedestrianCost);
  //get the costing method
  std::string costing_method = boost::python::extract<std::string>(boost::python::str(dict_request["costing_method"]));
  cost_ = factory.Create(costing_method);
  //get the config for the graph reader
  boost::property_tree::ptree pt;
  std::string config_file = boost::python::extract<std::string>(boost::python::str(dict_request["config"]));
  boost::property_tree::read_json(config_file, pt);
  reader_.reset(new valhalla::baldr::GraphReader(pt));

  //TODO: we get other info such as: z (zoom level), output (format), instructions (text)
}

RouteHandler::~RouteHandler() {

}

std::string RouteHandler::Action() {
  //where to
  valhalla::baldr::PathLocation origin = valhalla::loki::Search(locations_[0], *reader_);
  valhalla::baldr::PathLocation destination = valhalla::loki::Search(locations_[1], *reader_);

  //get the path
  valhalla::thor::PathAlgorithm path_algorithm;
  std::vector<valhalla::baldr::GraphId> path_edges;
  path_edges = path_algorithm.GetBestPath(origin, destination, *reader_, cost_);

  //get some pbf
  valhalla::odin::TripPath trip_path = valhalla::thor::TripPathBuilder::Build(*reader_, path_edges);
  path_algorithm.Clear();

  //get some annotated instructions
  valhalla::odin::DirectionsBuilder directions_builder;
  valhalla::odin::TripDirections trip_directions = directions_builder.BuildSimple(trip_path);

  //make some json
  return serialize(trip_path, trip_directions);
}


}
}
