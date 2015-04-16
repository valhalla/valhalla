#include "tyr/locate_handler.h"
#include "tyr/json.h"

#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/loki/search.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/sif/autocost.h>
#include <valhalla/sif/bicyclecost.h>
#include <valhalla/sif/pedestriancost.h>

#include <utility>

using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::loki;
using namespace valhalla::tyr;

namespace {

  //TODO: move json header to baldr
  //TODO: make objects serialize themselves

  json::ArrayPtr serialize_ll(const PointLL& ll) {
    return json::array({
      static_cast<long double>(ll.lat()), static_cast<long double>(ll.lng())
    });
  }

  json::ArrayPtr serialize_edges(const PathLocation& location, GraphReader& reader) {
    auto array = json::array({});
    for(const auto& edge : location.edges()) {
      try {
        //get the osm way id
        auto tile = reader.GetGraphTile(edge.id);
        auto* directed_edge = tile->directededge(edge.id);
        auto edge_info = tile->edgeinfo(directed_edge->edgeinfo_offset());
        array->emplace_back(
          json::map({
            {"way_id", edge_info->wayid()},
            {"correlated_lat_lon", serialize_ll(location.vertex())}
          })
        );
      }
      catch(...) {
        //this really shouldnt ever get hit
        LOG_WARN("Expected edge no found in graph but found by loki::search!");
      }
    }
    return array;
  }

  json::MapPtr serialize(const PathLocation& location, GraphReader& reader) {
    return json::map({
      {"ways", serialize_edges(location, reader)},
      {"input_lat_lon", serialize_ll(location.latlng_)}
    });
  }

  json::MapPtr serialize(const std::pair<Location, std::string>& skipped) {
    return json::map({
      {"ways", static_cast<nullptr_t>(nullptr)},
      {"input_lat_lon", serialize_ll(skipped.first.latlng_)},
      {"reason", skipped.second}
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
  std::list<std::pair<PointLL, std::string> > skipped;
  for(const auto& location : locations_) {
    try {
      correlated.emplace_back(loki::Search(locations_[0], *reader_, cost_->GetFilter()));
    }
    catch(const std::exception& e) {
      skipped.emplace_back(std::make_pair(location.latlng_, e.what()));
    }
  }

  //rip through the correlated ones to create the json array
  auto correlated_json = json::array({});
  for(const auto& location : correlated)
    correlated_json->emplace_back(serialize(location, *reader_));

  //rip through the skipped ones to create the json array
  auto skipped_json = json::array({});
  for(const auto& skip : skipped)
    skipped_json->emplace_back(serialize(skip));

  //jsonp callback
  std::ostringstream stream;
  if(jsonp_)
    stream << *jsonp_ << '(';

  //top level object
  stream << *json::map({
    {"correlated", correlated_json},
    {"skipped", skipped_json}
  });

  //finish up jsonp
  if(jsonp_)
    stream << ')';
  return stream.str();
}

}
}
