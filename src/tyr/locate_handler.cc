#include "tyr/locate_handler.h"
#include "tyr/json.h"

#include <valhalla/baldr/pathlocation.h>
#include <valhalla/loki/search.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/sif/autocost.h>
#include <valhalla/sif/bicyclecost.h>
#include <valhalla/sif/pedestriancost.h>

using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::loki;
using namespace valhalla::tyr;

namespace {

  //TODO: move json header to baldr
  //TODO: make objects serialize themselves


  json::Value serialize_node(const PathLocation& location, GraphReader& reader) {
    if(location.IsNode()) {
      auto mp = json::map({});
      //TODO: get the osm node id
      //TODO: get the coordinate
      return mp;
    }
    else {
      return static_cast<nullptr_t>(nullptr);
    }
  }

  json::ArrayPtr serialize_edges(const std::vector<PathLocation::PathEdge>& edges, GraphReader& reader) {
    auto array = json::array({});
    for(const auto& edge : edges) {
      //TODO: get the osm way id
      //TODO: crack open the shape, use the distance to get the coordinate
    }
    return array;
  }

  json::MapPtr serialize(const PathLocation& location, GraphReader& reader) {
    return json::map({
      {"node", serialize_node(location, reader)},
      {"ways", serialize_edges(location.edges(), reader)}
    });
  }

}

namespace valhalla {
namespace tyr {

LocateHandler::LocateHandler(const boost::property_tree::ptree& config, const boost::property_tree::ptree& request)
  : Handler(config, request) {

  //we require locations
  try {
    for(const auto& loc : request.get_child("locations"))
      locations_.emplace_back(std::move(baldr::Location::FromCsv(loc.second.get_value<std::string>())));
    if(locations_.size() < 2)
      throw;
    //TODO: bail if this is too many
  }
  catch(...) {
    throw std::runtime_error("insufficiently specified required parameter `locations'");
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
  sif::CostFactory<sif::DynamicCost> factory;
  factory.Register("auto", sif::CreateAutoCost);
  factory.Register("auto_shorter", sif::CreateAutoShorterCost);
  factory.Register("bicycle", sif::CreateBicycleCost);
  factory.Register("pedestrian", sif::CreatePedestrianCost);

  // Get the costing options. Get the base options from the config and the
  // options for the specified costing method
  std::string method_options = "costing_options." + costing;
  boost::property_tree::ptree config_costing = config.get_child(method_options);
  auto request_costing = request.get_child_optional(method_options);
  if (request_costing) {
    // If the request has any options for this costing type, merge the 2
    // costing options - override any config options that are in the request.
    // and add any request options not in the config.
    for (auto r : *request_costing) {
      config_costing.put_child(r.first, r.second);
    }
  }
  cost_ = factory.Create(costing, config_costing);

  // Get the config for the graph reader
  reader_.reset(new baldr::GraphReader(config.get_child("mjolnir.hierarchy")));

}

LocateHandler::~LocateHandler() {

}

std::string LocateHandler::Action() {
  //find the correlate the various locations to the underlying graph
  std::list<baldr::PathLocation> correlated;
  for(const auto& location : locations_)
    correlated.emplace_back(loki::Search(locations_[0], *reader_, cost_->GetFilter()));

  //rip through the correlated ones to create the json array
  auto json = json::array({});
  for(const auto& location : correlated)
    json->emplace_back(serialize(location, *reader_));

  //make some json
  std::ostringstream stream;
  if(jsonp_)
    stream << *jsonp_ << '(';
  stream << *json;
  if(jsonp_)
    stream << ')';
  return stream.str();
}

}
}
