#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/multi/geometries/register/multi_polygon.hpp>
#include <gtest/gtest.h>

#include "gurka.h"
#include "test.h"

#include "baldr/graphconstants.h"
#include "baldr/graphreader.h"
#include "loki/polygon_search.h"
#include "midgard/pointll.h"
#include "mjolnir/graphtilebuilder.h"

#include "proto/options.pb.h"

using namespace valhalla;
namespace bg = boost::geometry;
namespace vm = valhalla::midgard;
namespace vl = valhalla::loki;

BOOST_GEOMETRY_REGISTER_POINT_2D(PointLL, double, bg::cs::geographic<bg::degree>, first, second)
BOOST_GEOMETRY_REGISTER_MULTI_POLYGON(std::vector<bg::model::polygon<PointLL>>)

using multipolygon_t = std::vector<bg::model::polygon<PointLL>>;

void config_poly_options(Options* options, const multipolygon_t& polys) {
  auto avoid_polygons = options->mutable_avoid_polygons();
  for (auto& poly : polys) {
    auto* polygon = avoid_polygons->Add();
    for (auto& coord : poly.outer()) {
      auto* ll = polygon->add_coords()->mutable_ll();
      ll->set_lng(coord.lng());
      ll->set_lat(coord.lat());
    }
  }
}

namespace {
// TODO: make loki_worker_test_t and make few functions public
class loki_worker_test_t : vl::loki_worker_t {
public:
  loki_worker_test_t(const boost::property_tree::ptree& config,
                     const std::shared_ptr<baldr::GraphReader>& graph_reader)
      : vl::loki_worker_t(config, reader) {
  }
  void parse_costing(Api& api, bool allow_none) {
    vl::loki_worker_t::parse_costing(api, allow_none);
  };
};
} // namespace

TEST(AvoidPolygons, TestCorrectAreaCalc) {
  // define multipolygon with ~ 9 sqkm area
  multipolygon_t polys(2);
  bg::read_wkt(
      "POLYGON ((13.38625361 52.4652558, 13.38625361 52.48000128, 13.4181769 52.48000128, 13.4181769 52.4652558, 13.38625361 52.4652558))",
      polys[0]);
  bg::read_wkt(
      "POLYGON ((13.36862753 52.43148012, 13.36862753 52.4555968, 13.39944328 52.4555968, 13.39944328 52.43148012, 13.36862753 52.43148012))",
      polys[1]);
  double actual_area = bg::area(polys);

  // Add polygons to PBF
  Options* options;
  config_poly_options(options, polys);
  auto calc_area = loki::GetArea(options->avoid_polygons());

  EXPECT_EQ(actual_area, calc_area);
}

TEST(AvoidPolygons, TestBla) {
  // not actually building any tiles..
  const boost::property_tree::ptree conf =
      gurka::detail::build_config("test/data/utrecht_tiles",
                                  {{"service_limits.max_avoid_polygons_sqkm", "1"}});
  loki_worker_test_t loki_worker(conf, std::make_shared<GraphReader>(conf.get_child("mjolnir")));
  Api request;
  Options* options = request.mutable_options();
  options->set_costing(Costing::auto_);

  // define multipolygon with ~ 9 sqkm area
  multipolygon_t polys(2);
  bg::read_wkt(
      "POLYGON ((13.38625361 52.4652558, 13.38625361 52.48000128, 13.4181769 52.48000128, 13.4181769 52.4652558, 13.38625361 52.4652558))",
      polys[0]);
  bg::read_wkt(
      "POLYGON ((13.36862753 52.43148012, 13.36862753 52.4555968, 13.39944328 52.4555968, 13.39944328 52.43148012, 13.36862753 52.43148012))",
      polys[1]);
  config_poly_options(options, polys);

  loki_worker.parse_costing(request, false);
}
