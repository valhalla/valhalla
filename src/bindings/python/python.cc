// Note: keep include boost/python.hpp first - fixes https://bugs.python.org/issue10910 on
// FreeBSD/macOS
#include <boost/python.hpp>

#include "baldr/rapidjson_utils.h"
#include <boost/make_shared.hpp>
#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>
#include <sstream>
#include <string>

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

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(route_overloads, route, 1, 1);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(locate_overloads, locate, 1, 1);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(optimized_route_overloads, optimized_route, 1, 1);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(matrix_overloads, matrix, 1, 1);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(isochrone_overloads, isochrone, 1, 1);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(trace_route_overloads, trace_route, 1, 1);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(trace_attributes_overloads, trace_attributes, 1, 1);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(height_overloads, height, 1, 1);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(transit_available_overloads, transit_available, 1, 1);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(expansion_overloads, expansion, 1, 1);
} // namespace

BOOST_PYTHON_MODULE(valhalla) {

  // python interface for configuring the system, always call this first in your python program
  boost::python::def("Configure", py_configure);

  boost::python::class_<valhalla::tyr::actor_t, boost::noncopyable,
                        boost::shared_ptr<valhalla::tyr::actor_t>>("Actor", boost::python::no_init)
      .def("__init__", boost::python::make_constructor(+[]() {
             return boost::make_shared<valhalla::tyr::actor_t>(configure(), true);
           }))
      .def("Route", &valhalla::tyr::actor_t::route, route_overloads())
      .def("Locate", &valhalla::tyr::actor_t::route, locate_overloads())
      .def("OptimizedRoute", &valhalla::tyr::actor_t::route, optimized_route_overloads())
      .def("Matrix", &valhalla::tyr::actor_t::route, matrix_overloads())
      .def("Isochrone", &valhalla::tyr::actor_t::route, isochrone_overloads())
      .def("TraceRoute", &valhalla::tyr::actor_t::route, trace_route_overloads())
      .def("TraceAttributes", &valhalla::tyr::actor_t::route, trace_attributes_overloads())
      .def("Height", &valhalla::tyr::actor_t::route, height_overloads())
      .def("TransitAvailable", &valhalla::tyr::actor_t::route, transit_available_overloads())
      .def("Expansion", &valhalla::tyr::actor_t::route, expansion_overloads())

      ;
}
