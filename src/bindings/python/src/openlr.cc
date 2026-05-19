#include "valhalla/baldr/openlr.h"
#include "midgard/pointll.h"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <string>

namespace nb = nanobind;
namespace volr = valhalla::baldr::OpenLR;

namespace pyvalhalla {

void init_openlr(nb::module_& m) {
  // enums
  nb::enum_<volr::LocationReferencePoint::FormOfWay>(m, "FormOfWay")
      .value("UNDEFINED", volr::LocationReferencePoint::UNDEFINED)
      .value("MOTORWAY", volr::LocationReferencePoint::MOTORWAY)
      .value("MULTIPLE_CARRIAGEWAY", volr::LocationReferencePoint::MULTIPLE_CARRIAGEWAY)
      .value("SINGLE_CARRIAGEWAY", volr::LocationReferencePoint::SINGLE_CARRIAGEWAY)
      .value("ROUNDABOUT", volr::LocationReferencePoint::ROUNDABOUT)
      .value("TRAFFICSQUARE", volr::LocationReferencePoint::TRAFFICSQUARE)
      .value("SLIPROAD", volr::LocationReferencePoint::SLIPROAD)
      .value("OTHER", volr::LocationReferencePoint::OTHER);

  nb::enum_<volr::Orientation>(m, "Orientation")
      .value("NoOrientation", volr::Orientation::NoOrientation)
      .value("FirstLrpTowardsSecond", volr::Orientation::FirstLrpTowardsSecond)
      .value("SecondLrpTowardsFirst", volr::Orientation::SecondLrpTowardsFirst)
      .value("BothDirections", volr::Orientation::BothDirections);

  nb::enum_<volr::SideOfTheRoad>(m, "SideOfTheRoad")
      .value("DirectlyOnRoadOrNotApplicable", volr::SideOfTheRoad::DirectlyOnRoadOrNotApplicable)
      .value("RightSideOfRoad", volr::SideOfTheRoad::RightSideOfRoad)
      .value("LeftSideOfRoad", volr::SideOfTheRoad::LeftSideOfRoad)
      .value("BothSidesOfRoad", volr::SideOfTheRoad::BothSidesOfRoad);

  nb::class_<volr::LocationReferencePoint>(m, "LocationReferencePoint")
      // explicitly handles optional 'prev' pointer and default arguments
      .def(
          "__init__",
          [](volr::LocationReferencePoint* self, double longitude, double latitude, double bearing,
             unsigned char frc, volr::LocationReferencePoint::FormOfWay fow,
             volr::LocationReferencePoint* prev, double distance, unsigned char lfrcnp) {
            new (self) volr::LocationReferencePoint(longitude, latitude, bearing, frc, fow, prev,
                                                    distance, lfrcnp);
          },
          nb::arg("longitude"), nb::arg("latitude"), nb::arg("bearing"), nb::arg("frc"),
          nb::arg("fow"), nb::arg("prev") = nullptr, nb::arg("distance") = 0.0, nb::arg("lfrcnp") = 0)

      .def_ro("longitude", &volr::LocationReferencePoint::longitude)
      .def_ro("latitude", &volr::LocationReferencePoint::latitude)
      .def_ro("bearing", &volr::LocationReferencePoint::bearing)
      .def_ro("distance", &volr::LocationReferencePoint::distance)
      .def_ro("frc", &volr::LocationReferencePoint::frc)
      .def_ro("lfrcnp", &volr::LocationReferencePoint::lfrcnp)
      .def_ro("fow", &volr::LocationReferencePoint::fow)

      // print formatting
      .def("__repr__", [](const volr::LocationReferencePoint& lrp) {
        return "<LocationReferencePoint lon=" + std::to_string(lrp.longitude) +
               " lat=" + std::to_string(lrp.latitude) + " bearing=" + std::to_string(lrp.bearing) +
               ">";
      });

  nb::class_<valhalla::midgard::PointLL>(m, "PointLL")
      .def(nb::init<double, double>())
      .def_prop_ro("lng", &valhalla::midgard::PointLL::lng)
      .def_prop_ro("lat", &valhalla::midgard::PointLL::lat)
      .def("is_valid", &valhalla::midgard::PointLL::IsValid)
      .def("__repr__",
           [](const valhalla::midgard::PointLL& p) {
             return "<PointLL lng=" + std::to_string(p.lng()) + " lat=" + std::to_string(p.lat()) +
                    ">";
           })
      .def("__eq__", [](const valhalla::midgard::PointLL& a, const valhalla::midgard::PointLL& b) {
        return a == b;
      });

  nb::class_<volr::OpenLr>(m, "OpenLr")
      // semantic constructor
      .def(nb::init<const std::vector<volr::LocationReferencePoint>&, uint8_t, uint8_t, bool,
                    const volr::Orientation&, const volr::SideOfTheRoad&>(),
           nb::arg("lrps"), nb::arg("positive_offset_bucket") = 0,
           nb::arg("negative_offset_bucket") = 0, nb::arg("point_along_line") = false,
           nb::arg("orientation") = volr::Orientation::NoOrientation,
           nb::arg("side_of_the_road") = volr::SideOfTheRoad::DirectlyOnRoadOrNotApplicable)

      // static parsing constructors
      .def_static(
          "from_binary",
          [](nb::bytes b) {
            std::string s(static_cast<const char*>(b.data()), b.size());
            return volr::OpenLr(s, false);
          },
          nb::arg("binary"))
      .def_static(
          "from_base64", [](const std::string& encoded) { return volr::OpenLr(encoded, true); },
          nb::arg("encoded"))

      .def_ro("lrps", &volr::OpenLr::lrps)
      .def_ro("poff", &volr::OpenLr::poff)
      .def_ro("noff", &volr::OpenLr::noff)
      .def_ro("is_point_along_line", &volr::OpenLr::isPointAlongLine)
      .def_ro("orientation", &volr::OpenLr::orientation)
      .def_ro("side_of_the_road", &volr::OpenLr::sideOfTheRoad)

      // convenience properties
      .def_prop_ro("first_coordinate", &volr::OpenLr::getFirstCoordinate)
      .def_prop_ro("last_coordinate", &volr::OpenLr::getLastCoordinate)
      .def_prop_ro("length", &volr::OpenLr::getLength)

      // serialization
      .def("to_binary",
           [](const volr::OpenLr& olr) {
             auto data = olr.toBinary();
             return nb::bytes(data.data(), data.size());
           })
      .def("to_base64", &volr::OpenLr::toBase64)
      .def("__repr__", [](const volr::OpenLr& olr) {
        return "OpenLr("
               "lrps=" +
               std::to_string(olr.lrps.size()) + ", poff=" + std::to_string(olr.poff) +
               ", noff=" + std::to_string(olr.noff) +
               ", point_along_line=" + std::string(olr.isPointAlongLine ? "True" : "False") + ")";
      });
}

NB_MODULE(openlr, m) {
  init_openlr(m);
  m.doc() = "Valhalla OpenLR (v3) utilities";
}

} // namespace pyvalhalla
