#include "tyr/custom_route_handler.h"

#include <stdexcept>
#include <ostream>
#include <valhalla/proto/trippath.pb.h>
#include <valhalla/proto/tripdirections.pb.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/python/str.hpp>
#include <boost/python/extract.hpp>
#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/loki/search.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/sif/autocost.h>
#include <valhalla/sif/bicyclecost.h>
#include <valhalla/sif/pedestriancost.h>
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
    "locations": [
       {
        "longitude": -76.4791,
        "latitude": 40.4136,
         "stopType": 0
       },
       {
        "longitude": -76.5352,
        "latitude": 40.4029,
        "stopType": 0
       }
     ],
    "units": "kilometers"
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
  float length = 0.0f;
  uint64_t seconds = 0, meters = 0;
  for(const auto& maneuver : trip_directions.maneuver()) {
    length += maneuver.length();
    seconds += static_cast<uint64_t>(maneuver.time());
  }
  route_summary->emplace("time", seconds);
  route_summary->emplace("length", static_cast<long double>(length));
  return route_summary;
}

json::ArrayPtr locations(const valhalla::odin::TripDirections& trip_directions){
  auto locations = json::array({});
  for(const auto& location : trip_directions.location()) {

    auto loc = json::map({});

    if (location.type() == valhalla::odin::TripDirections_Location_Type_kThrough) {
      loc->emplace("type", std::string("through"));
    } else {
      loc->emplace("type", std::string("break"));
    }
    loc->emplace("latitude", static_cast<long double>(location.ll().lat()));
    loc->emplace("longitude",static_cast<long double>(location.ll().lng()));
    if (!location.name().empty())
      loc->emplace("name",location.name());
    if (!location.street().empty())
      loc->emplace("street",location.street());
    if (!location.city().empty())
      loc->emplace("city",location.city());
    if (!location.state().empty())
      loc->emplace("state",location.state());
    if (!location.postal_code().empty())
      loc->emplace("postal_code",location.postal_code());
    if (!location.country().empty())
      loc->emplace("country",location.country());
    if (location.has_heading())
      loc->emplace("heading",static_cast<uint64_t>(location.heading()));

    //loc->emplace("sideOfStreet",location.side_of_street());

    locations->emplace_back(loc);
  }

  return locations;
}


json::ArrayPtr legs(const valhalla::odin::TripDirections& trip_directions){

  // TODO: multiple legs.
  auto legs = json::array({});
  auto leg = json::map({});
  auto summary = json::map({});
  float length = 0.0f;
  uint64_t seconds = 0;
  auto maneuvers = json::array({});

  for(const auto& maneuver : trip_directions.maneuver()) {

    length += maneuver.length();
    seconds += static_cast<uint64_t>(maneuver.time());
    auto man = json::map({});

    man->emplace("type", static_cast<uint64_t>(maneuver.type()));
    man->emplace("instruction", maneuver.text_instruction());
    //“verbalTransitionAlertInstruction” : “<verbalTransitionAlertInstruction>”,
    //“verbalPreTransitionInstruction” : “<verbalPreTransitionInstruction>”,
    //“verbalPostTransitionInstruction” : “<verbalPostTransitionInstruction>”,
    auto street_names = json::array({});

    for (int i = 0; i < maneuver.street_name_size(); i++)
      street_names->emplace_back(maneuver.street_name(i));

    if (street_names->size())
      man->emplace("street_names", street_names);
    man->emplace("time", static_cast<uint64_t>(maneuver.time()));
    man->emplace("length", static_cast<long double>(maneuver.length()));
    man->emplace("begin_shape_index", static_cast<uint64_t>(maneuver.begin_shape_index()));
    man->emplace("end_shape_index", static_cast<uint64_t>(maneuver.end_shape_index()));

    if (maneuver.portions_toll())
      man->emplace("toll", maneuver.portions_toll());
    if (maneuver.portions_unpaved())
      man->emplace("rough", maneuver.portions_unpaved());

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
  summary->emplace("length", static_cast<long double>(length));
  leg->emplace("summary",summary);
  leg->emplace("shape", trip_directions.shape());

  legs->emplace_back(leg);
  return legs;
}

void serialize(const valhalla::odin::TripDirections& trip_directions,
               const std::string& units, std::ostringstream& stream) {

  //TODO: worry about multipoint routes

  //build up the json object
  auto json = json::map
      ({
      {"trip", json::map
      ({
          {"locations", locations(trip_directions)},
          {"summary", summary(trip_directions)},
          {"legs", legs(trip_directions)},
          {"status_message", string("Found route between points")}, //found route between points OR cannot find route between points
          {"status", static_cast<uint64_t>(0)}, //0 success or 207 no route
          {"units", units}
      })
    }
  });

  //serialize it
  stream << *json;
}

}

namespace valhalla {
namespace tyr {

CustomRouteHandler::CustomRouteHandler(const boost::property_tree::ptree& config,
                                       const boost::property_tree::ptree& request) {
  try {
    for(const auto& location : request.get_child("locations"))
      locations_.emplace_back(std::move(baldr::Location::FromPtree(location.second)));
    if(locations_.size() < 2)
      throw;
  }
  catch(...) {
    throw std::runtime_error("insufficiently specified required parameter 'locations'");
  }

  // Parse out the type of route - this provides the costing method to use
  std::string costing;
  try {
    costing = request.get<std::string>("costing");
  }
  catch(...) {
    throw std::runtime_error("No edge/node costing provided");
  }

  // Register edge/node costing methods
  valhalla::sif::CostFactory<valhalla::sif::DynamicCost> factory;
  factory.Register("auto", valhalla::sif::CreateAutoCost);
  factory.Register("auto_shorter", valhalla::sif::CreateAutoShorterCost);
  factory.Register("bicycle", valhalla::sif::CreateBicycleCost);
  factory.Register("pedestrian", valhalla::sif::CreatePedestrianCost);

  // Get the costing options. Get the base options from the config and the
  // options for the specified costing method
  std::string method_options = "costing_options." + costing;
  boost::property_tree::ptree config_costing = config.get_child(method_options);
  auto request_costing = request.get_child_optional(method_options);
  if (request_costing) {
    // If the request has any options for this costing type, merge the 2
    // costing options - override any config options that are in the request.
    // and  add any request options not in the config.
    for (auto r : *request_costing) {
      config_costing.put_child(r.first, r.second);
    }
  }
  cost_ = factory.Create(costing, config_costing);

  // Get the config for the graph reader
  reader_.reset(new valhalla::baldr::GraphReader(config.get_child("mjolnir.hierarchy")));

  // TODO - replace this below when we pass down to Odin and get back info
  // in the proto
  // Get the units (defaults to kilometers)
  std::string units = "k";
  auto s = request.get_optional<std::string>("units");
  if (s) {
    units = *s;
  }
  km_units_ = (units == "miles" || units == "m") ? false : true;
  units_ = (km_units_) ? "kilometers" : "miles";

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
  serialize(trip_directions, units_, stream);
  if(jsonp_)
    stream << ')';
  return stream.str();
}


}
}
