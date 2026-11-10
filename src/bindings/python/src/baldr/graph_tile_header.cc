#include "baldr/graphtileheader.h"
#include "module.h"

#include <nanobind/nanobind.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>

#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace nb = nanobind;
namespace vb = valhalla::baldr;

namespace {

constexpr size_t kHeaderSize = sizeof(vb::GraphTileHeader);

vb::GraphTileHeader from_bytes(const uint8_t* data, size_t size) {
  if (size < kHeaderSize)
    throw std::invalid_argument("buffer too small for a graph tile header (" + std::to_string(size) +
                                " < " + std::to_string(kHeaderSize) + ")");
  vb::GraphTileHeader h;
  std::memcpy(&h, data, kHeaderSize);
  return h;
}

vb::GraphTileHeader from_file(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in)
    throw std::runtime_error("cannot open " + path.string());
  uint8_t buf[kHeaderSize];
  in.read(reinterpret_cast<char*>(buf), kHeaderSize);
  if (static_cast<size_t>(in.gcount()) < kHeaderSize)
    throw std::invalid_argument("file too small for a graph tile header: " + path.string());
  return from_bytes(buf, kHeaderSize);
}

void save(const vb::GraphTileHeader& h, const std::filesystem::path& path) {
  // Patch an existing tile's header in place — never create or grow a file:
  // a header without tile data behind it is not a tile.
  std::error_code ec;
  const auto size = std::filesystem::file_size(path, ec);
  if (ec || size < kHeaderSize)
    throw std::invalid_argument("not an existing graph tile file: " + path.string());
  std::fstream out(path, std::ios::binary | std::ios::in | std::ios::out);
  if (!out)
    throw std::runtime_error("cannot open " + path.string());
  out.write(reinterpret_cast<const char*>(&h), kHeaderSize);
  if (!out)
    throw std::runtime_error("failed writing " + path.string());
}

} // namespace

namespace pyvalhalla::baldr {

void init_graphtileheader(nb::module_& m) {
  nb::class_<vb::GraphTileHeader>(
      m, "GraphTileHeader",
      "Header of a graph tile. Read one via GraphUtils.get_graph_tile_header(), from_file() or "
      "from_bytes(), or default-construct one. The tileset identity fields (dataset_id, "
      "date_created, raw_checksum) are writable; everything else is read-only — it identifies "
      "the tile or is derived from its data. save() patches the header span of an existing tile "
      "file in place.")
      .def(nb::init<>())
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
      .def_prop_rw("dataset_id", &vb::GraphTileHeader::dataset_id,
                   &vb::GraphTileHeader::set_dataset_id,
                   "Data set id (e.g. latest OSM changeset id).")
      .def_prop_rw("date_created", &vb::GraphTileHeader::date_created,
                   &vb::GraphTileHeader::set_date_created,
                   "Tile creation date (days since the pivot date).")
      .def_prop_rw(
          "raw_checksum",
          [](const vb::GraphTileHeader& h) {
            return (static_cast<uint64_t>(h.build_id()) << vb::kTileHashBits) | h.tile_checksum();
          },
          &vb::GraphTileHeader::set_raw_checksum,
          "Raw 64-bit checksum field: tileset build id in the high 16 bits, the tile's 48-bit "
          "data hash in the low bits. See also the derived tile_checksum and build_id.")
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
      .def_prop_ro("end_offset", &vb::GraphTileHeader::end_offset, "Tile size in bytes.")
      .def_prop_rw(
          "tile_checksum", &vb::GraphTileHeader::tile_checksum,
          [](vb::GraphTileHeader& h, uint64_t checksum) {
            if (checksum >> vb::kTileHashBits)
              throw std::invalid_argument("tile_checksum exceeds 48 bits");
            h.set_raw_checksum((static_cast<uint64_t>(h.build_id()) << vb::kTileHashBits) | checksum);
          },
          "Integer checksum (48 bit) of the tile's data. Setting it keeps the build_id.")
      .def_prop_rw(
          "build_id", &vb::GraphTileHeader::build_id,
          [](vb::GraphTileHeader& h, uint16_t build_id) {
            h.set_raw_checksum((static_cast<uint64_t>(build_id) << vb::kTileHashBits) |
                               h.tile_checksum());
          },
          "Integer additive checksum (16 bit) of the tileset. Setting it keeps the "
          "tile_checksum.")
      .def_static(
          "byte_size", []() { return kHeaderSize; },
          "Size of the on-disk header in bytes (the header span at the start of a tile).")
      .def_static(
          "from_bytes",
          [](nb::bytes data) {
            return from_bytes(reinterpret_cast<const uint8_t*>(data.c_str()), data.size());
          },
          nb::arg("data"),
          "Read a header from a buffer (a whole tile or just its first byte_size() bytes).")
      .def_static("from_file", &from_file, nb::arg("path"), "Read the header of a graph tile file.")
      .def(
          "to_bytes",
          [](const vb::GraphTileHeader& h) {
            return nb::bytes(reinterpret_cast<const char*>(&h), kHeaderSize);
          },
          "Serialize the header to its byte_size() on-disk bytes.")
      .def("save", &save, nb::arg("path"),
           "Patch this header over the header span of an existing graph tile file, in place. "
           "The file must already exist and hold at least byte_size() bytes.")
      .def("__repr__", [](const vb::GraphTileHeader& h) {
        return "<GraphTileHeader " + std::to_string(h.graphid()) +
               " nodes=" + std::to_string(h.nodecount()) +
               " edges=" + std::to_string(h.directededgecount()) + ">";
      });
}
} // namespace pyvalhalla::baldr
