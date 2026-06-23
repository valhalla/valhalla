#include "baldr/graphid.h"
#include "baldr/graphtile.h"
#include "module.h"

#include <nanobind/nanobind.h>
#include <nanobind/operators.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>

namespace nb = nanobind;
namespace vb = valhalla::baldr;

namespace pyvalhalla::baldr {

void init_graphid(nb::module_& m) {
  nb::class_<vb::GraphId>(m, "GraphId",
                          "Identifier of a node or an edge within the tiled, hierarchical graph.\n"
                          "Includes the tile Id, hierarchy level, and a unique identifier within\n"
                          "the tile/level.")
      .def(nb::init(), "Constructs an invalid GraphId")
      .def(nb::init<uint32_t, uint32_t, uint32_t>(), nb::arg("tileid"), nb::arg("level"),
           nb::arg("id"), "Constructs a GraphId from its portions.")
      .def(nb::init<uint64_t>(),
           "Constructs a GraphId from its integer value, e.g. 118931 == 3/14866/0.")
      .def(nb::init<std::string>(),
           "Constructs a GraphId from its string representation, e.g. \"2/71944/0\".")
      .def_ro("value", &vb::GraphId::value, "The integer representation of the bit-fielded GraphId.")
      .def("tileid", &vb::GraphId::tileid, "Gets the tile Id.")
      .def("level", &vb::GraphId::level, "Gets the hierarchy level.")
      .def("id", &vb::GraphId::id, "Gets the identifier within the hierarchy level.")
      .def("is_valid", &vb::GraphId::is_valid, "Returns true if the id is valid.")
      .def("tile_base", &vb::GraphId::tile_base,
           "Returns a GraphId omitting the id of the of the object within the level.\n"
           "Construct a new GraphId with the Id portion omitted.")
      .def("tile_value", &vb::GraphId::tile_value,
           "Returns a value indicating the tile (level and tile id) of the graph Id.")
      .def(nb::self + uint64_t(), "Increments the id portion by value")
      .def(nb::self += uint32_t(), "Increments the id portion by value")
      .def(nb::self == nb::self, "Equality operator")
      .def(nb::self != nb::self, "Inequality operator")
      .def("__bool__", &vb::GraphId::is_valid, "True if is_valid().")
      .def("__str__", [](const vb::GraphId& graph_id) { return std::to_string(graph_id); })
      .def("__fspath__",
           [](const vb::GraphId& graph_id) { return vb::GraphTile::FileSuffix(graph_id); })
      .def("__repr__",
           [](const vb::GraphId& graph_id) { return "<GraphId(" + std::to_string(graph_id) + ")>"; })
      // pickling support auto-provides copy/deepcopy support
      .def("__getstate__", [](const vb::GraphId& gid) { return std::make_tuple(gid.value); })
      .def("__setstate__", [](vb::GraphId& self, const std::tuple<uint64_t>& state) {
        new (&self) vb::GraphId(std::get<0>(state));
      });
}
} // namespace pyvalhalla::baldr
