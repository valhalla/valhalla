#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/rapidjson_utils.h"
#include "midgard/logging.h"
#include "module.h"

#include <boost/property_tree/ptree.hpp>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include <algorithm>
#include <fstream>
#include <sstream>

namespace nb = nanobind;
namespace vb = valhalla::baldr;
namespace vm = valhalla::midgard;

namespace pyvalhalla::baldr::utils {

void init_graphreader(nb::module_& m) {
  // GraphUtils class - manages GraphReader for efficient edge access
  nb::class_<
      vb::GraphReader>(m, "_GraphUtils",
                       "C++ binding for GraphUtils (internal use - prefer GraphUtils wrapper).\n\n"
                       "Manages a GraphReader for efficient access to tiles and edges.\n"
                       "Initialize once and reuse for multiple edge queries.")
      .def(
          "__init__",
          [](vb::GraphReader* self, const std::string& config) {
            // Parse the config - handle both file paths and JSON strings
            boost::property_tree::ptree pt;
            try {
              std::ifstream file(config);
              if (file.good()) {
                rapidjson::read_json(file, pt);
              } else {
                std::istringstream stream(config);
                rapidjson::read_json(stream, pt);
              }
            } catch (...) { throw std::runtime_error("Failed to parse config JSON"); }

            // Configure logging
            vm::logging::ConfigureFromPtree(pt);

            // Create GraphReader with mjolnir subtree
            new (self) vb::GraphReader(pt.get_child("mjolnir"));
          },
          nb::arg("config"),
          "Initialize _GraphUtils with Valhalla configuration.\n\n"
          ":param config: Valhalla configuration as JSON string or path to config file\n"
          ":raises RuntimeError: When config is invalid")
      .def(
          "get_edge_shape",
          [](vb::GraphReader& self,
             const vb::GraphId& edge_id) -> std::vector<std::tuple<double, double>> {
            // Get the tile containing this edge
            auto tile = self.GetGraphTile(edge_id);
            if (!tile) {
              throw std::runtime_error("Tile not found for edge " + std::to_string(edge_id));
            }

            // Get the directed edge
            const auto* directed_edge = tile->directededge(edge_id);
            if (!directed_edge) {
              throw std::runtime_error("Edge not found: " + std::to_string(edge_id));
            }

            // Get the edge info and shape
            auto edge_info = tile->edgeinfo(directed_edge);
            auto shape = edge_info.shape();

            // Reverse the shape if the edge is not forward
            if (!directed_edge->forward()) {
              std::reverse(shape.begin(), shape.end());
            }

            // Convert to list of (lon, lat) tuples
            std::vector<std::tuple<double, double>> result;
            result.reserve(shape.size());
            for (const auto& pt : shape) {
              result.emplace_back(pt.lng(), pt.lat());
            }

            return result;
          },
          nb::arg("edge_id"), nb::call_guard<nb::gil_scoped_release>(),
          "Get the shape (polyline) for an edge as a list of (lon, lat) tuples.\n\n"
          ":param edge_id: GraphId of the edge\n"
          ":returns: List of (lon, lat) tuples representing the edge geometry\n"
          ":raises RuntimeError: When the tile or edge is not found");
}
} // namespace pyvalhalla::baldr::utils
