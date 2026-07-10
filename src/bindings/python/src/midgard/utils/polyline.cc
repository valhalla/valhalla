#include "midgard/encoded.h"
#include "midgard/pointll.h"
#include "module.h"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include <algorithm>
#include <cmath>

namespace nb = nanobind;
namespace vm = valhalla::midgard;

namespace pyvalhalla::midgard::utils {

void init_polyline(nb::module_& m) {
  using coord_t = std::tuple<double, double>;

  // maintains signature backward compatibility with an old python implementation
  m.def(
      "decode_polyline",
      [](const std::string& polyline, int precision,
         const std::string& order) -> std::vector<coord_t> {
        const double prec = std::pow(10.0, -precision);
        const auto pts = vm::decode<std::vector<vm::PointLL>>(polyline, prec);

        const auto project = order == "latlng" ? +[](const vm::PointLL& p) -> coord_t {
          return {p.lat(), p.lng()};
        }
        : +[](const vm::PointLL& p) -> coord_t {
            return {p.lng(), p.lat()};
          };

        std::vector<coord_t> out(pts.size());
        std::transform(pts.begin(), pts.end(), out.begin(), project);
        return out;
      },
      nb::arg("polyline"), nb::arg("precision") = 6, nb::arg("order") = "lnglat",
      "Decodes an encoded polyline string with precision to a list of coordinate tuples.\n"
      "The coordinate order of the output can be lnglat or latlng.");
}
} // namespace pyvalhalla::midgard::utils
