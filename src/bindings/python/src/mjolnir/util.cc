#include "mjolnir/util.h"
#include "module.h"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;
namespace vj = valhalla::mjolnir;

namespace pyvalhalla::mjolnir {

void init_util(nb::module_& m) {
  m.def("compute_tileset_build_id", &vj::compute_tileset_build_id, nb::arg("tile_dir"),
        nb::call_guard<nb::gil_scoped_release>(),
        "Compute the tileset-wide 16-bit build id from the per-tile content hashes\n"
        "already stored in each tile header (their sum, folded to 16 bits; no\n"
        "re-hashing). Read-only companion of set_tileset_build_id.\n\n"
        ":param tile_dir: Directory holding the .gph tiles.\n"
        ":returns: The 16-bit tileset build id.");

  m.def("set_tileset_build_id", &vj::set_tileset_build_id, nb::arg("tile_dir"),
        nb::call_guard<nb::gil_scoped_release>(),
        "Recompute the tileset-wide build id and stamp it into the high bits of every\n"
        "tile's checksum, in place. Call this after a tool has rewritten a subset of\n"
        "tiles (e.g. adding predicted traffic) so URL clients see a changed tileset.\n\n"
        ":param tile_dir: Directory holding the .gph tiles; headers are patched in place.");
}

} // namespace pyvalhalla::mjolnir
