#include <pybind11/pybind11.h>

#include "baldr/rapidjson_utils.h"
#include <boost/make_shared.hpp>
#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <sstream>
#include <string>

#include "baldr/rapidjson_utils.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "tyr/actor.h"

namespace vt = valhalla::tyr;
namespace {

// configuring multiple times is wasteful/ineffectual but not harmful
// TODO: make this threadsafe just in case its abused
const boost::property_tree::ptree configure(const std::string& config) {
  boost::property_tree::ptree pt;
  try {
    // parse the config and configure logging
    rapidjson::read_json(config, pt);

    boost::optional<boost::property_tree::ptree&> logging_subtree =
        pt.get_child_optional("mjolnir.logging");
    if (logging_subtree) {
      auto logging_config = valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                                     std::unordered_map<std::string, std::string>>(
          logging_subtree.get());
      valhalla::midgard::logging::Configure(logging_config);
    }
  } catch (...) { throw std::runtime_error("Failed to load config from: " + config); }

  return pt;
}
} // namespace

namespace py = pybind11;

PYBIND11_MODULE(python_valhalla, m) {
  py::class_<vt::actor_t>(m, "_Actor", "Valhalla Actor class")
      .def(py::init<>([](std::string config) { return vt::actor_t(configure(config), true); }))
      .def(
          "route", [](vt::actor_t& self, std::string& req) { return self.route(req); },
          "Calculates a route.")
      .def(
          "locate", [](vt::actor_t& self, std::string& req) { return self.locate(req); },
          "Provides information about nodes and edges.")
      .def(
          "optimized_route",
          [](vt::actor_t& self, std::string& req) { return self.optimized_route(req); },
          "Optimizes the order of a set of waypoints by time.")
      .def(
          "matrix", [](vt::actor_t& self, std::string& req) { return self.matrix(req); },
          "Computes the time and distance between a set of locations and returns them as a matrix table.")
      .def(
          "isochrone", [](vt::actor_t& self, std::string& req) { return self.isochrone(req); },
          "Calculates isochrones and isodistances.")
      .def(
          "trace_route", [](vt::actor_t& self, std::string& req) { return self.trace_route(req); },
          "Map-matching for a set of input locations, e.g. from a GPS.")
      .def(
          "trace_attributes",
          [](vt::actor_t& self, std::string& req) { return self.trace_attributes(req); },
          "Returns detailed attribution along each portion of a route calculated from a set of input locations, e.g. from a GPS trace.")
      .def(
          "height", [](vt::actor_t& self, std::string& req) { return self.height(req); },
          "Provides elevation data for a set of input geometries.")
      .def(
          "transit_available",
          [](vt::actor_t& self, std::string& req) { return self.transit_available(req); },
          "Lookup if transit stops are available in a defined radius around a set of input locations.")
      .def(
          "expansion", [](vt::actor_t& self, std::string& req) { return self.expansion(req); },
          "Returns all road segments which were touched by the routing algorithm during the graph traversal.")
      .def(
          "centroid", [](vt::actor_t& self, std::string& req) { return self.centroid(req); },
          "Returns routes from all the input locations to the minimum cost meeting point of those paths.")
      .def(
          "status", [](vt::actor_t& self, std::string& req) { return self.status(req); },
          "Returns nothing or optionally details about Valhalla's configuration.");
}
