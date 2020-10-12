#include <pybind11/pybind11.h>

#include "baldr/rapidjson_utils.h"
#include <boost/make_shared.hpp>
#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>
#include <sstream>
#include <string>

#include "baldr/rapidjson_utils.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "tyr/actor.h"

namespace {

// statically set the config file and configure logging, throw if you never configured
// configuring multiple times is wasteful/ineffectual but not harmful
// TODO: make this threadsafe just in case its abused
const boost::property_tree::ptree&
configure(const boost::optional<std::string>& config = boost::none) {
  static boost::optional<boost::property_tree::ptree> pt;
  // if we haven't already loaded one
  if (config && !pt) {
    try {
      // parse the config
      boost::property_tree::ptree temp_pt;
      rapidjson::read_json(config.get(), temp_pt);
      pt = temp_pt;

      // configure logging
      boost::optional<boost::property_tree::ptree&> logging_subtree =
          pt->get_child_optional("tyr.logging");
      if (logging_subtree) {
        auto logging_config = valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                                       std::unordered_map<std::string, std::string>>(
            logging_subtree.get());
        valhalla::midgard::logging::Configure(logging_config);
      }
    } catch (...) { throw std::runtime_error("Failed to load config from: " + config.get()); }
  }

  // if it turned out no one ever configured us we throw
  if (!pt) {
    throw std::runtime_error("The service was not configured");
  }
  return *pt;
}
void py_configure(const std::string& config_file) {
  configure(config_file);
}
} // namespace

namespace py = pybind11;

template <typename... Args> using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;

PYBIND11_MODULE(valhalla_python, m) {
  m.def("Configure", py_configure);

  py::class_<valhalla::tyr::actor_t, std::shared_ptr<valhalla::tyr::actor_t>>(m, "Actor")
      .def(py::init<>([]() { return std::make_shared<valhalla::tyr::actor_t>(configure(), true); }))
      .def("Route", overload_cast_<const std::string&>()(&valhalla::tyr::actor_t::route),
           "Calculates a route.")
      .def("Locate", overload_cast_<const std::string&>()(&valhalla::tyr::actor_t::locate),
           "Provides information about nodes and edges.")
      .def("OptimizedRoute",
           overload_cast_<const std::string&>()(&valhalla::tyr::actor_t::optimized_route),
           "Optimizes the order of a set of waypoints by time.")
      .def(
          "Matrix", overload_cast_<const std::string&>()(&valhalla::tyr::actor_t::matrix),
          "Computes the time and distance between a set of locations and returns them as a matrix table.")
      .def("Isochrone", overload_cast_<const std::string&>()(&valhalla::tyr::actor_t::isochrone),
           "Calculates isochrones and isodistances.")
      .def("TraceRoute", overload_cast_<const std::string&>()(&valhalla::tyr::actor_t::trace_route),
           "Map-matching for a set of input locations, e.g. from a GPS.")
      .def(
          "TraceAttributes",
          overload_cast_<const std::string&>()(&valhalla::tyr::actor_t::trace_attributes),
          "Returns detailed attribution along each portion of a route calculate from a set of input locations, e.g. from a GPS trace.")
      .def("Height", overload_cast_<const std::string&>()(&valhalla::tyr::actor_t::height),
           "Provides elevation data for a set of input geometries.")
      .def(
          "TransitAvailable",
          overload_cast_<const std::string&>()(&valhalla::tyr::actor_t::transit_available),
          "Lookup if transit stops are available in a defined radius around a set of input locations.")
      .def("Expansion", overload_cast_<const std::string&>()(&valhalla::tyr::actor_t::expansion),
           "Returns all road segments which were touched by the routing algorithm during the search.");
}
