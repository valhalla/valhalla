#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

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
#include "mjolnir/util.h"
#include "tyr/actor.h"

using namespace valhalla::mjolnir;
namespace py = pybind11;

namespace {

// statically set the config file and configure logging, throw if you never configured
// configuring multiple times is wasteful/ineffectual but not harmful
// TODO: make this threadsafe just in case its abused
const boost::property_tree::ptree&
configure(const boost::optional<std::string>& config_path = boost::none,
          py::dict config = {},
          std::string tile_dir = "",
          std::string tile_extract = "",
          bool verbose = true) {
  static boost::optional<boost::property_tree::ptree> pt;
  if (config_path) {
    // create the config via python
    py::object create_config = py::module::import("valhalla.config").attr("_create_config");
    create_config(config_path.get(), config, tile_dir, tile_extract);
    try {
      // parse the config
      boost::property_tree::ptree temp_pt;
      rapidjson::read_json(config_path.get(), temp_pt);
      pt = temp_pt;

      // configure logging
      boost::optional<boost::property_tree::ptree&> logging_subtree =
          pt->get_child_optional("loki.logging");
      if (logging_subtree) {
        // No logging if not wanted
        if (!verbose) {
          logging_subtree->put("type", "");
        }
        auto logging_config = valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                                       std::unordered_map<std::string, std::string>>(
            logging_subtree.get());
        valhalla::midgard::logging::Configure(logging_config);
      }
    } catch (...) { throw std::runtime_error("Failed to load config from: " + config_path.get()); }
  }

  // if it turned out no one ever configured us we throw
  if (!pt) {
    throw std::runtime_error("The service was not configured");
  }
  return *pt;
}

void py_configure(const std::string& config_file,
                  py::dict config,
                  std::string tile_dir,
                  std::string tile_extract,
                  bool verbose) {
  configure(config_file, config, tile_dir, tile_extract, verbose);
}

bool py_build_tiles(std::vector<std::string> input_pbfs) {
  // make sure the service is configured
  auto pt = configure();

  pt.get_child("mjolnir").erase("tile_extract");
  pt.get_child("mjolnir").erase("tile_url");

  if (input_pbfs.empty()) {
    throw std::invalid_argument("No PBF files specified");
  }

  return build_tile_set(pt, input_pbfs, BuildStage::kInitialize, BuildStage::kCleanup, false);
}

void py_tar_tiles(const boost::property_tree::ptree& pt) {
  // delegate tar balling to python
  py::object tar_tiles = py::module::import("valhalla._utils").attr("_tar_tiles");
  tar_tiles(pt.get("mjolnir.tile_dir", ""), pt.get("mjolnir.tile_extract", ""));
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
};

PYBIND11_MODULE(python_valhalla, m) {
  m.def("Configure", py_configure, py::arg("config_file"), py::arg("config") = py::dict(),
        py::arg("tile_dir") = "", py::arg("tile_extract") = "", py::arg("verbose") = true);

  py::class_<simplified_actor_t, std::shared_ptr<simplified_actor_t>>(m, "Actor")
      .def(py::init<>([]() {
        auto pt = configure();
        std::cout << "Tile extract: " << pt.get("mjolnir.tile_extract", "") << std::endl;
        return std::make_shared<simplified_actor_t>(pt);
      }))
      .def("Route", &simplified_actor_t::route, "Calculates a route.")
      .def("Locate", &simplified_actor_t::locate, "Provides information about nodes and edges.")
      .def("OptimizedRoute", &simplified_actor_t::optimized_route,
           "Optimizes the order of a set of waypoints by time.")
      .def(
          "Matrix", &simplified_actor_t::matrix,
          "Computes the time and distance between a set of locations and returns them as a matrix table.")
      .def("Isochrone", &simplified_actor_t::isochrone, "Calculates isochrones and isodistances.")
      .def("TraceRoute", &simplified_actor_t::trace_route,
           "Map-matching for a set of input locations, e.g. from a GPS.")
      .def(
          "TraceAttributes", &simplified_actor_t::trace_attributes,
          "Returns detailed attribution along each portion of a route calculated from a set of input locations, e.g. from a GPS trace.")
      .def("Height", &simplified_actor_t::height,
           "Provides elevation data for a set of input geometries.")
      .def(
          "TransitAvailable", &simplified_actor_t::transit_available,
          "Lookup if transit stops are available in a defined radius around a set of input locations.")
      .def(
          "Expansion", &simplified_actor_t::expansion,
          "Returns all road segments which were touched by the routing algorithm during the graph traversal.");

  m.def("BuildTiles", py_build_tiles);

  m.def("TarTiles", []() { return py_tar_tiles(configure()); });
}
