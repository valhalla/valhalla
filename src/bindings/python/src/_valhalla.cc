#include "baldr/rapidjson_utils.h"
#include "config.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "tyr/actor.h"

#include <boost/property_tree/ptree.hpp>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include <string>

namespace vt = valhalla::tyr;
namespace nb = nanobind;
namespace {

// configuring multiple times is wasteful/ineffectual but not harmful
// TODO: make this threadsafe just in case its abused
const boost::property_tree::ptree configure(const std::string& config) {
  boost::property_tree::ptree pt;
  try {
    // parse the config and configure logging
    rapidjson::read_json(config, pt);

    auto logging_subtree = pt.get_child_optional("mjolnir.logging");
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

NB_MODULE(_valhalla, m) {
  // Add these constants in C++ to avoid creating another shim python module, as they
  // are needed at runtime of the python library and need to be set during the build
  m.attr("VALHALLA_PRINT_VERSION") = VALHALLA_PRINT_VERSION;

  nb::class_<vt::actor_t>(m, "_Actor", "Valhalla Actor class")
      .def(
          "__init__",
          [](vt::actor_t* self, const std::string& config) {
            new (self) vt::actor_t(configure(config), true);
          },
          nb::arg("config"))
      .def(
          "route", [](vt::actor_t& self, std::string& req) { return self.route(req); },
          "Calculates a route.", nb::call_guard<nb::gil_scoped_release>())
      .def(
          "locate", [](vt::actor_t& self, std::string& req) { return self.locate(req); },
          "Provides information about nodes and edges.", nb::call_guard<nb::gil_scoped_release>())
      .def(
          "optimized_route",
          [](vt::actor_t& self, std::string& req) { return self.optimized_route(req); },
          "Optimizes the order of a set of waypoints by time.",
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "matrix", [](vt::actor_t& self, std::string& req) { return self.matrix(req); },
          "Computes the time and distance between a set of locations and returns them as a matrix table.",
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "isochrone", [](vt::actor_t& self, std::string& req) { return self.isochrone(req); },
          "Calculates isochrones and isodistances.", nb::call_guard<nb::gil_scoped_release>())
      .def(
          "trace_route", [](vt::actor_t& self, std::string& req) { return self.trace_route(req); },
          "Map-matching for a set of input locations, e.g. from a GPS.",
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "trace_attributes",
          [](vt::actor_t& self, std::string& req) { return self.trace_attributes(req); },
          "Returns detailed attribution along each portion of a route calculated from a set of input locations, e.g. from a GPS trace.",
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "height", [](vt::actor_t& self, std::string& req) { return self.height(req); },
          "Computes the height for a set of input geometries.",
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "transit_available",
          [](vt::actor_t& self, std::string& req) { return self.transit_available(req); },
          "Lookup if transit stops are available in a defined radius around a set of input locations.",
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "expansion", [](vt::actor_t& self, std::string& req) { return self.expansion(req); },
          "Returns all road segments which were touched by the routing algorithm during the graph traversal.",
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "centroid", [](vt::actor_t& self, std::string& req) { return self.centroid(req); },
          "Returns routes from all the input locations to the minimum cost meeting point of those paths.",
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "status", [](vt::actor_t& self, std::string& req) { return self.status(req); },
          "Returns nothing or optionally details about Valhalla's configuration.",
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "tile",
          [](vt::actor_t& self, std::string& req) -> nb::bytes {
            std::string result;
            {
              nb::gil_scoped_release release;
              result = self.tile(req);
            }
            return nb::bytes(result.c_str(), result.size());
          },
          "Returns a vector tile (MVT binary data) with a bounding box feature for the given z/x/y tile coordinates.");
}
