#include "baldr/rapidjson_utils.h"
#include "midgard/logging.h"
#include "module.h"
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

    valhalla::midgard::logging::ConfigureFromPtree(pt);
  } catch (...) { throw std::runtime_error("Failed to load config from: " + config); }

  return pt;
}
} // namespace

namespace pyvalhalla {

void init_actor(nb::module_& m) {
  // User-facing docstrings (class + each method) live on the Python `Actor`
  // wrapper in actor.py as string literals — Pyright/Pylance reads them
  // statically and does not walk the MRO, so the C++ docstrings would not
  // show up on hover in VSCode.
  nb::class_<vt::actor_t>(m, "_Actor")
      .def(
          "__init__",
          [](vt::actor_t* self, const std::string& config) {
            new (self) vt::actor_t(configure(config), true);
          },
          nb::arg("config"))
      .def(
          "route", [](vt::actor_t& self, std::string& req) { return self.route(req); },
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "locate", [](vt::actor_t& self, std::string& req) { return self.locate(req); },
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "optimized_route",
          [](vt::actor_t& self, std::string& req) { return self.optimized_route(req); },
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "matrix", [](vt::actor_t& self, std::string& req) { return self.matrix(req); },
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "isochrone", [](vt::actor_t& self, std::string& req) { return self.isochrone(req); },
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "trace_route", [](vt::actor_t& self, std::string& req) { return self.trace_route(req); },
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "trace_attributes",
          [](vt::actor_t& self, std::string& req) { return self.trace_attributes(req); },
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "height", [](vt::actor_t& self, std::string& req) { return self.height(req); },
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "transit_available",
          [](vt::actor_t& self, std::string& req) { return self.transit_available(req); },
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "expansion", [](vt::actor_t& self, std::string& req) { return self.expansion(req); },
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "centroid", [](vt::actor_t& self, std::string& req) { return self.centroid(req); },
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "status", [](vt::actor_t& self, std::string& req) { return self.status(req); },
          nb::call_guard<nb::gil_scoped_release>())
      .def("tile", [](vt::actor_t& self, std::string& req) -> nb::bytes {
        std::string result;
        {
          nb::gil_scoped_release release;
          result = self.tile(req);
        }
        return nb::bytes(result.c_str(), result.size());
      });
}
} // namespace pyvalhalla
