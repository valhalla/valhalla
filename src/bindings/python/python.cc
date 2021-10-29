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

namespace {

// configuring multiple times is wasteful/ineffectual but not harmful
// TODO: make this threadsafe just in case its abused
const boost::property_tree::ptree configure(const std::string& config) {
  boost::property_tree::ptree pt;
  try {
    // parse the config and configure logging
    rapidjson::read_json(config, pt);

    boost::optional<boost::property_tree::ptree&> logging_subtree =
        pt.get_child_optional("tyr.logging");
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

struct simplified_actor_t : public valhalla::tyr::actor_t {
  simplified_actor_t(const boost::property_tree::ptree& config)
      : valhalla::tyr::actor_t::actor_t(config, true) {
  }

  std::string route(const std::string& request_str) {
    return valhalla::tyr::actor_t::route(request_str, nullptr, nullptr);
  };
  std::string locate(const std::string& request_str) {
    return valhalla::tyr::actor_t::locate(request_str, nullptr, nullptr);
  };
  std::string optimized_route(const std::string& request_str) {
    return valhalla::tyr::actor_t::optimized_route(request_str, nullptr, nullptr);
  };
  std::string matrix(const std::string& request_str) {
    return valhalla::tyr::actor_t::matrix(request_str, nullptr, nullptr);
  };
  std::string isochrone(const std::string& request_str) {
    return valhalla::tyr::actor_t::isochrone(request_str, nullptr, nullptr);
  };
  std::string trace_route(const std::string& request_str) {
    return valhalla::tyr::actor_t::trace_route(request_str, nullptr, nullptr);
  };
  std::string trace_attributes(const std::string& request_str) {
    return valhalla::tyr::actor_t::trace_attributes(request_str, nullptr, nullptr);
  };
  std::string height(const std::string& request_str) {
    return valhalla::tyr::actor_t::height(request_str, nullptr, nullptr);
  };
  std::string transit_available(const std::string& request_str) {
    return valhalla::tyr::actor_t::transit_available(request_str, nullptr, nullptr);
  };
  std::string expansion(const std::string& request_str) {
    return valhalla::tyr::actor_t::expansion(request_str, nullptr, nullptr);
  };
  std::string centroid(const std::string& request_str) {
    return valhalla::tyr::actor_t::centroid(request_str, nullptr, nullptr);
  };
  std::string status(const std::string& request_str) {
    return valhalla::tyr::actor_t::status(request_str, nullptr, nullptr);
  }
};

namespace py = pybind11;

PYBIND11_MODULE(python_valhalla, m) {
  py::class_<simplified_actor_t>(m, "_Actor", "Valhalla Actor class")
      .def(py::init<>([](std::string config) { return simplified_actor_t(configure(config)); }))
      .def("route", &simplified_actor_t::route, "Calculates a route.")
      .def("locate", &simplified_actor_t::locate, "Provides information about nodes and edges.")
      .def("optimized_route", &simplified_actor_t::optimized_route,
           "Optimizes the order of a set of waypoints by time.")
      .def(
          "matrix", &simplified_actor_t::matrix,
          "Computes the time and distance between a set of locations and returns them as a matrix table.")
      .def("isochrone", &simplified_actor_t::isochrone, "Calculates isochrones and isodistances.")
      .def("trace_route", &simplified_actor_t::trace_route,
           "Map-matching for a set of input locations, e.g. from a GPS.")
      .def(
          "trace_attributes", &simplified_actor_t::trace_attributes,
          "Returns detailed attribution along each portion of a route calculated from a set of input locations, e.g. from a GPS trace.")
      .def("height", &simplified_actor_t::height,
           "Provides elevation data for a set of input geometries.")
      .def(
          "transit_available", &simplified_actor_t::transit_available,
          "Lookup if transit stops are available in a defined radius around a set of input locations.")
      .def(
          "expansion", &simplified_actor_t::expansion,
          "Returns all road segments which were touched by the routing algorithm during the graph traversal.")
      .def(
          "centroid", &simplified_actor_t::centroid,
          "Returns routes from all the input locations to the minimum cost meeting point of those paths.")
      .def("status", &simplified_actor_t::status,
           "Returns nothing or optionally details about Valhalla's configuration.");
}
