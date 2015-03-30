#include "tyr/custom_route_handler.h"

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
valhalla output looks like this:
{
    "trip":
{
    "status": 0,
    "locations": [ ],
    "summary":
{
    "distance": 4973,
    "time": 325
},
"legs":
[
  {
      "summary":
  {
      "distance": 4973,
      "time": 325
  },
  "maneuvers":
  [
    {
        "beginShapeIndex": 0,
        "distance": 633,
        "writtenInstruction": "Start out going west on West Market Street.",
        "streetNames":
        [
            "West Market Street"
        ],
        "type": 1,
        "time": 41
    },
    {
        "beginShapeIndex": 7,
        "distance": 4340,
        "writtenInstruction": "Continue onto Jonestown Road.",
        "streetNames":
        [
            "Jonestown Road"
        ],
        "type": 8,
        "time": 284
    },
    {
        "beginShapeIndex": 40,
        "distance": 0,
        "writtenInstruction": "You have arrived at your destination.",
        "type": 4,
        "time": 0
    }
],
"shape": "gysalAlg|zpC~Clt@tDtx@hHfaBdKl{BrKbnApGro@tJrz@jBbQj@zVt@lTjFnnCrBz}BmFnoB]pHwCvm@eJxtATvXTnfAk@|^z@rGxGre@nTpnBhBbQvXduCrUr`Edd@naEja@~gAhk@nzBxf@byAfm@tuCvDtOvNzi@|jCvkKngAl`HlI|}@`N`{Adx@pjE??xB|J"
}
],
"status_message": "Found route between points"
}
}
*/
using namespace valhalla::tyr;
using namespace std;

json::MapPtr summary(const valhalla::odin::TripDirections& trip_directions){

  // TODO: multiple legs.

  auto route_summary = json::map({});
  uint64_t seconds = 0, meters = 0;
  for(const auto& maneuver : trip_directions.maneuver()) {
    meters += static_cast<uint64_t>(maneuver.length() * 1000.f);
    seconds += static_cast<uint64_t>(maneuver.time());
  }
  route_summary->emplace("time", seconds);
  route_summary->emplace("distance", meters);
  return route_summary;
}

json::ArrayPtr locations(const valhalla::odin::TripPath& trip_path){
  auto locations = json::array({});
  for(const auto& location : trip_path.location()) {

    auto loc = json::map({});

    //loc->emplace("type", location.type());
    loc->emplace("latitude", (long double)(location.ll().lat()));
    loc->emplace("longitude",(long double)(location.ll().lng()));
    loc->emplace("name",location.name());
    loc->emplace("phone",location.phone());
    loc->emplace("street",location.street());
    loc->emplace("city",location.city());
    loc->emplace("state",location.state());
    loc->emplace("postalCode",location.postal_code());
    loc->emplace("country",location.country());
    // loc->emplace("url",location.url());
    //loc->emplace("sideOfStreet",location.side_of_street());
    locations->emplace_back(loc);
  }

  return locations;
}


json::ArrayPtr legs(const valhalla::odin::TripPath& trip_path,
                    const valhalla::odin::TripDirections& trip_directions){

  // TODO: multiple legs.
  auto legs = json::array({});
  auto leg = json::map({});
  auto summary = json::map({});
  uint64_t seconds = 0, meters = 0;
  auto maneuvers = json::array({});

  for(const auto& maneuver : trip_directions.maneuver()) {

    meters += static_cast<uint64_t>(maneuver.length() * 1000.f);
    seconds += static_cast<uint64_t>(maneuver.time());
    leg->emplace("shape", trip_path.shape());
    auto man = json::map({});

    man->emplace("type", static_cast<uint64_t>(maneuver.type()));
    man->emplace("writtenInstruction", maneuver.text_instruction());
    //“verbalTransitionAlertInstruction” : “<verbalTransitionAlertInstruction>”,
    //“verbalPreTransitionInstruction” : “<verbalPreTransitionInstruction>”,
    //“verbalPostTransitionInstruction” : “<verbalPostTransitionInstruction>”,
    auto street_names = json::array({});

    for (int i = 0; i < maneuver.street_name_size(); i++)
      street_names->emplace_back(maneuver.street_name(i));

    if (street_names->size())
      man->emplace("streetNames", street_names);
    man->emplace("time", static_cast<uint64_t>(maneuver.time()));
    man->emplace("distance", static_cast<uint64_t>(maneuver.length() * 1000.f));
    man->emplace("beginShapeIndex", static_cast<uint64_t>(maneuver.begin_shape_index()));
    man->emplace("endShapeIndex", static_cast<uint64_t>(maneuver.end_shape_index()));

    if (maneuver.portions_toll())
      man->emplace("hasPortionsToll", maneuver.portions_toll());
    if (maneuver.portions_unpaved())
      man->emplace("hasPortionsUnpaved", maneuver.portions_unpaved());

    //  man->emplace("hasGate", maneuver.);
    //  man->emplace("hasFerry", maneuver.);
    //“portionsTollNote” : “<portionsTollNote>”,
    //“portionsUnpavedNote” : “<portionsUnpavedNote>”,
    //“gateAccessRequiredNote” : “<gateAccessRequiredNote>”,
    //“checkFerryInfoNote” : “<checkFerryInfoNote>”
    maneuvers->emplace_back(man);

  }
  leg->emplace("maneuvers", maneuvers);
  summary->emplace("time", seconds);
  summary->emplace("distance", meters);
  leg->emplace("summary",summary);

  legs->emplace_back(leg);
  return legs;
}

void serialize(const valhalla::odin::TripPath& trip_path,
  const valhalla::odin::TripDirections& trip_directions, std::ostringstream& stream) {

  //TODO: worry about multipoint routes

  //build up the json object
  auto json = json::map
      ({
      {"trip", json::map
      ({
          {"locations", locations(trip_path)},
          {"summary", summary(trip_directions)},
          {"legs", legs(trip_path,trip_directions)},
          {"status_message", string("Found route between points")}, //found route between points OR cannot find route between points
          {"status", static_cast<uint64_t>(0)} //0 success or 207 no route
      })
    }
  });

  //serialize it
  stream << *json;
}

}

namespace valhalla {
namespace tyr {

CustomRouteHandler::CustomRouteHandler(const std::string& config, const boost::python::dict& dict_request) : Handler(config, dict_request) {
  //parse out the type of route
  if(!dict_request.has_key("costing_method"))
    throw std::runtime_error("No edge/node costing method provided");
  //register edge/node costing methods
  valhalla::thor::CostFactory<valhalla::thor::DynamicCost> factory;
  factory.Register("auto", valhalla::thor::CreateAutoCost);
  factory.Register("bicycle", valhalla::thor::CreateBicycleCost);
  factory.Register("pedestrian", valhalla::thor::CreatePedestrianCost);

  boost::property_tree::ptree pt;
  boost::property_tree::read_json(config, pt);

  //get the costing method
  std::string costing_method = boost::python::extract<std::string>(boost::python::str(dict_request["costing_method"]));
  cost_ = factory.Create(costing_method, pt.get_child("thor"));

  //get the config for the graph reader
  reader_.reset(new valhalla::baldr::GraphReader(pt.get_child("mjolnir.hierarchy")));

  //TODO: we get other info such as: z (zoom level), output (format), instructions (text)
}

CustomRouteHandler::~CustomRouteHandler() {

}

std::string CustomRouteHandler::Action() {
  //where to
  valhalla::baldr::PathLocation origin = valhalla::loki::Search(locations_[0], *reader_, cost_->GetFilter());
  valhalla::baldr::PathLocation destination = valhalla::loki::Search(locations_[1], *reader_, cost_->GetFilter());

  //get the path
  valhalla::thor::PathAlgorithm path_algorithm;
  std::vector<valhalla::baldr::GraphId> path_edges;
  path_edges = path_algorithm.GetBestPath(origin, destination, *reader_, cost_);

  // Check if failure.
  if (path_edges.size() == 0) {
    // If costing allows multiple passes - relax the hierarchy limits.
    // TODO - configuration control of # of passes and relaxation factor
    if (cost_->AllowMultiPass()) {
      uint32_t n = 0;
      while (path_edges.size() == 0 && n++ < 4) {
        path_algorithm.Clear();
        cost_->RelaxHierarchyLimits(4.0f);
        path_edges = path_algorithm.GetBestPath(origin, destination, *reader_, cost_);
      }
      if (path_edges.size() == 0) {
        throw std::runtime_error("Route failed after 4 passes");
      }
    } else {
      throw std::runtime_error("Route failure");
    }
  }

  //get some pbf
  valhalla::odin::TripPath trip_path = valhalla::thor::TripPathBuilder::Build(*reader_, path_edges, origin, destination);
  path_algorithm.Clear();

  //get some annotated instructions
  valhalla::odin::DirectionsBuilder directions_builder;
  valhalla::odin::TripDirections trip_directions = directions_builder.Build(trip_path);

  //make some json
  std::ostringstream stream;
  if(jsonp_)
    stream << *jsonp_ << '(';
  serialize(trip_path, trip_directions, stream);
  if(jsonp_)
    stream << ')';
  return stream.str();
}


}
}
