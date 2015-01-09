#include "tyr/route_handler.h"

#include <stdexcept>
#include <valhalla/proto/trippath.pb.h>
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
#include <valhalla/odin/narrativebuilder.h>

namespace {

std::string serialize(const valhalla::odin::TripPath& trip_path) {
  return "blah";
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
  valhalla::thor::cost_ptr_t cost_ = factory.Create(costing_method);
  //get the config for the graph reader
  boost::property_tree::ptree pt;
  std::string config_file = boost::python::extract<std::string>(boost::python::str(dict_request["config"]));
  boost::property_tree::read_json(config_file, pt);
  reader_.reset(new valhalla::baldr::GraphReader(pt));

  //TODO: we get other info such as: z (zoom level), output (format), instructions (text)
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
  valhalla::odin::NarrativeBuilder narrative_builder(trip_path);
  narrative_builder.Build();

  //make some json
  return serialize(trip_path);
}


}
}
