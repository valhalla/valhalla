#include "baldr/graphtileheader.h"
#include "module.h"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>

#include <string>

namespace nb = nanobind;
namespace vb = valhalla::baldr;

namespace pyvalhalla::baldr {

void init_graphtileheader(nb::module_& m) {
  nb::class_<vb::GraphTileHeader>(
      m, "GraphTileHeader",
      "Read-only information about a graph tile. Obtain one via GraphUtils.get_graph_tile_header().")
      .def_prop_ro("graphid", &vb::GraphTileHeader::graphid,
                   "GraphId (tile id + level) of this tile.")
      .def_prop_ro(
          "base_ll",
          [](const vb::GraphTileHeader& h) {
            const auto ll = h.base_ll();
            return std::make_tuple(ll.lng(), ll.lat());
          },
          "(lon, lat) of the tile's south-western corner, in degrees.")
      .def_prop_ro("version", &vb::GraphTileHeader::version, "Tile format version string.")
      .def_prop_ro("dataset_id", &vb::GraphTileHeader::dataset_id,
                   "Data set id (e.g. latest OSM changeset id).")
      .def_prop_ro("density", &vb::GraphTileHeader::density, "Relative road density (0-15).")
      .def_prop_ro("has_elevation", &vb::GraphTileHeader::has_elevation,
                   "True if the tile carries edge elevation data.")
      .def_prop_ro("has_ext_directededge", &vb::GraphTileHeader::has_ext_directededge,
                   "True if the tile carries extended directed-edge attributes.")
      .def_prop_ro("has_bounding_circles", &vb::GraphTileHeader::has_bounding_circles,
                   "True if the tile carries node bounding circles.")
      .def_prop_ro("nodecount", &vb::GraphTileHeader::nodecount, "Number of nodes.")
      .def_prop_ro("directededgecount", &vb::GraphTileHeader::directededgecount,
                   "Number of directed edges.")
      .def_prop_ro("transitioncount", &vb::GraphTileHeader::transitioncount,
                   "Number of node transitions.")
      .def_prop_ro("signcount", &vb::GraphTileHeader::signcount, "Number of signs.")
      .def_prop_ro("access_restriction_count", &vb::GraphTileHeader::access_restriction_count,
                   "Number of access restriction records.")
      .def_prop_ro("admincount", &vb::GraphTileHeader::admincount, "Number of admin records.")
      .def_prop_ro("turnlane_count", &vb::GraphTileHeader::turnlane_count,
                   "Number of turn lane records.")
      .def_prop_ro("predictedspeeds_count", &vb::GraphTileHeader::predictedspeeds_count,
                   "Number of predicted speed records.")
      .def_prop_ro("departurecount", &vb::GraphTileHeader::departurecount,
                   "Number of transit departures.")
      .def_prop_ro("stopcount", &vb::GraphTileHeader::stopcount, "Number of transit stops.")
      .def_prop_ro("routecount", &vb::GraphTileHeader::routecount, "Number of transit routes.")
      .def_prop_ro("schedulecount", &vb::GraphTileHeader::schedulecount,
                   "Number of transit schedules.")
      .def_prop_ro("transfercount", &vb::GraphTileHeader::transfercount,
                   "Number of transit transfers.")
      .def_prop_ro("date_created", &vb::GraphTileHeader::date_created,
                   "Tile creation date (days since the pivot date).")
      .def_prop_ro("end_offset", &vb::GraphTileHeader::end_offset, "Tile size in bytes.")
      .def_prop_ro("tile_checksum", &vb::GraphTileHeader::tile_checksum,
                   "Integer checksum (48 bit) of the tile's data.")
      .def_prop_ro("build_id", &vb::GraphTileHeader::build_id,
                   "Integer additive checksum (16 bit) of the tileset.")
      .def("__repr__", [](const vb::GraphTileHeader& h) {
        return "<GraphTileHeader " + std::to_string(h.graphid()) +
               " nodes=" + std::to_string(h.nodecount()) +
               " edges=" + std::to_string(h.directededgecount()) + ">";
      });
}
} // namespace pyvalhalla::baldr
