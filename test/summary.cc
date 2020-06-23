#include "test.h"

#include <iostream>
#include <string>
#include <vector>

#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "loki/worker.h"
#include "odin/worker.h"
#include "thor/worker.h"
#include "tyr/serializers.h"
#include <boost/property_tree/ptree.hpp>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::thor;
using namespace valhalla::odin;
using namespace valhalla::loki;
using namespace valhalla::tyr;

namespace {

boost::property_tree::ptree get_conf() {
  std::stringstream ss;
  ss << R"({
      "mjolnir":{"tile_dir":"test/data/utrecht_tiles", "concurrency": 1},
      "loki":{
        "actions":["route"],
        "logging":{"long_request": 100},
        "service_defaults":{"minimum_reachability": 50,"radius": 0,"search_cutoff": 35000, "node_snap_tolerance": 5, "street_side_tolerance": 5, "street_side_max_distance": 1000, "heading_tolerance": 60}
      },
      "thor":{"logging":{"long_request": 100}},
      "odin":{"logging":{"long_request": 100}},
      "skadi":{"actons":["height"],"logging":{"long_request": 5}},
      "meili":{"customizable": ["turn_penalty_factor","max_route_distance_factor","max_route_time_factor","search_radius"],
              "mode":"auto","grid":{"cache_size":100240,"size":500},
              "default":{"beta":3,"breakage_distance":2000,"geometry":false,"gps_accuracy":5.0,"interpolation_distance":10,
              "max_route_distance_factor":5,"max_route_time_factor":5,"max_search_radius":200,"route":true,
              "search_radius":15.0,"sigma_z":4.07,"turn_penalty_factor":200}},
      "service_limits": {
        "auto": {"max_distance": 5000000.0, "max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "auto_shorter": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "bicycle": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
        "bus": {"max_distance": 5000000.0,"max_locations": 50,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "hov": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
        "isochrone": {"max_contours": 4,"max_distance": 25000.0,"max_locations": 1,"max_time": 120},
        "max_avoid_locations": 50,"max_radius": 200,"max_reachability": 100,"max_alternates":2,
        "multimodal": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 0.0,"max_matrix_locations": 0},
        "pedestrian": {"max_distance": 250000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50,"max_transit_walking_distance": 10000,"min_transit_walking_distance": 1},
        "skadi": {"max_shape": 750000,"min_resample": 10.0},
        "trace": {"max_distance": 200000.0,"max_gps_accuracy": 100.0,"max_search_radius": 100,"max_shape": 16000,"max_best_paths":4,"max_best_paths_shape":100},
        "transit": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
        "truck": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50}
      }
    })";
  boost::property_tree::ptree conf;
  rapidjson::read_json(ss, conf);
  return conf;
}

struct route_tester {
  route_tester()
      : conf(get_conf()), reader(new GraphReader(conf.get_child("mjolnir"))),
        loki_worker(conf, reader), thor_worker(conf, reader), odin_worker(conf) {
  }
  Api test(const std::string& request_json) {
    Api request;
    ParseApi(request_json, Options::route, request);
    loki_worker.route(request);
    thor_worker.route(request);
    odin_worker.narrate(request);
    loki_worker.cleanup();
    thor_worker.cleanup();
    odin_worker.cleanup();
    return request;
  }
  boost::property_tree::ptree conf;
  std::shared_ptr<GraphReader> reader;
  loki_worker_t loki_worker;
  thor_worker_t thor_worker;
  odin_worker_t odin_worker;
};

TEST(Summary, test_time_summary) {
  route_tester tester;
  std::string request =
      R"({"locations":[{"lat":52.114622,"lon":5.131816},{"lat":52.048267,"lon":5.074825}],"costing":"auto",
          "filters":{"attributes":["shape_attributes.time"],"action":"include"}})";
  auto response = tester.test(request);
  const auto& leg = response.trip().routes(0).legs(0);
  EXPECT_EQ(
      leg.shape(),
      "kbykbB{afxHWee@kAuh@{@qZmP{[_IsMaHoMgDcSu@mFM{@wDoIcAx@iR`LqHlE}IhFaShF{P{@eJAqL{MoNkLcRgSsUc^_IcPwIcUmOkf@qRss@sKig@{Po~@iM}|@wHe{@yCa_@s@{_@z@i^dEo_@nI}^lJcX~NoWhM{Npa@gYbk@yUro@_WnvAmn@~l@wVnc@kRhGcDnNaHxa@mShg@aUlqBq{@hdB_x@prDuyAt`Cy|@dZsKhWuLb|B_`Ajf@wOfgAy`@nr@q^bj@cY`q@qa@h]_UzZcTz`@i[tmAmbAfXwNp\\aPdPoAtTpAzUrJnNlKvMbQhM~WzKtZx\\p_BxQfbA~NjcAdJdj@|~@poEzn@fmC~DdPjKiJ~N_MfNuJdOkIfNkFv^kDjP`@fO`AdOtD~CnApCdApGhC~NxJpQhPbLdLfYjZlgAjnAh|@r{@zZnSje@x\\|aBfdAld@xZ~uBjtArKhHfkAhw@bWzNzJpF~IfGxLpIrLpInShQbPbR`NlTjP|ZzPhc@dKbZ`g@d{AbQnh@fXxv@vM`^j[~s@zQx]nMnTpRz[nh@pu@bVzZ|TtZt^rc@ra@vf@dYrZz`@h^l_@~W~]xSxyAxl@~|@j]b~Axm@xjAdd@zy@hZfgAra@nStJdJfIfElGtDpHfI|TvHfXrB`Mz@~ZGxb@{@hXcQ~qBm_@`vDcp@`aFoHxm@uE|SsFzX}Jjd@gIj\\oHlXqH|Z}DtQoDvS{ApLaCr\\UdE]vDGtCd@jEz@xCxBvDxHhBjo@nOr[jEdT~AbLYbGYdJeB`CY~R_FpWoKxHwDfeB_u@jViFrKm@xMvAvSdHxVdS`Xjd@bKhSlJfWvIna@`\\`zBxCjQhGhVpH`UpLpV|@lBvCpG~HnUjGfZzEld@zAna@dAva@Df_@aBpc@kB~^aCx^cAdKiBzKqBbMyClQwH`]aHxWqHbUy[rz@mE`L}O~`@zF|GtDdFxl@~{@zn@``Ani@`w@jUr_@hg@l~@`{@faBh\\hp@fYhg@`Wbd@xGnL}OvnAoCtd@yBbOaRzuAm@nDqCfM_C~MeAlFsAlKs@pHaCtUWpJwChXqBdH}@jFUjFOzHt@nHvCpK~DtGlDlC`XdO~CzBG~T}Jh}@yVaLqBbAsFhh@G`FOt`@e@xC");

  // loop over all routes all legs
  auto trip_route = response.trip().routes().begin();
  for (const auto& route : response.directions().routes()) {
    auto trip_leg = trip_route->legs().begin();
    for (const auto& leg : route.legs()) {
      // accumulate the maneuvers for a leg
      double accumulated_time = 0;
      double accumulated_transition_time = 0;
      double accumulated_edge_time = 0;
      for (const auto& maneuver : leg.maneuver()) {
        accumulated_time += maneuver.time();
        // check the transition times should be non-zero and less than equal to the maneuver time
        double transition_time = 0;
        for (auto n = maneuver.begin_path_index(); n < maneuver.end_path_index(); ++n) {
          transition_time += trip_leg->node(n).transition_time();
        }
        EXPECT_LE(transition_time, maneuver.time());
        // check the on edge times plus the transition times add up to the maneuver time
        double edge_time_ms = 0;
        for (auto s = maneuver.begin_shape_index(); s < maneuver.end_shape_index(); ++s) {
          edge_time_ms += trip_leg->shape_attributes().time(s);
        }
        EXPECT_NEAR(edge_time_ms / 1000.0 + transition_time, maneuver.time(), .2);
        accumulated_transition_time += transition_time;
        accumulated_edge_time += edge_time_ms / 1000.0;
      }
      // make sure the end of the trip path is the same as the legs
      EXPECT_EQ(trip_leg->node().rbegin()->elapsed_time(), leg.summary().time());
      // make sure the maneuvers add up to the leg as well
      EXPECT_EQ(accumulated_time, leg.summary().time());
      // we should have had some transition costs along the way
      EXPECT_GT(accumulated_transition_time, 0);
      // we should have the edge time plus the transition time add up to the leg time
      EXPECT_NEAR(accumulated_edge_time + accumulated_transition_time, accumulated_time, .15);
      ++trip_leg;
    }
    ++trip_route;
  }

  // get the json
  auto json_str = serializeDirections(response);
  rapidjson::Document json;
  json.Parse(json_str);
  ASSERT_FALSE(json.HasParseError());

  // loop over all routes all legs
  for (const auto& leg : json["trip"].GetObject()["legs"].GetArray()) {
    // accumulate the maneuvers for a leg
    double accumulated_time = 0;
    for (const auto& maneuver : leg["maneuvers"].GetArray()) {
      accumulated_time += maneuver["time"].GetDouble();
    }
    // make sure the end of the trip path is the same as the legs
    EXPECT_NEAR(accumulated_time, leg["summary"].GetObject()["time"].GetDouble(), .01);
  }

  // get the osrm json
  response.mutable_options()->set_format(Options::osrm);
  json_str = serializeDirections(response);
  json.Parse(json_str);
  ASSERT_FALSE(json.HasParseError());

  // loop over all routes all legs
  for (const auto& route : json["routes"].GetArray()) {
    // accumulate the steps over all the legs
    double accumulated_time = 0;
    double accumulated_transition_time = 0;
    for (const auto& leg : route["legs"].GetArray()) {
      for (const auto& step : leg["steps"].GetArray()) {
        accumulated_time += step["duration"].GetDouble();
        // check that the accumulated transition durations are less than the step duration
        double transition_time = 0;
        for (const auto& intersection : step["intersections"].GetArray()) {
          if (intersection.HasMember("duration")) {
            transition_time += intersection["duration"].GetDouble();
          }
        }
        EXPECT_LE(transition_time, step["duration"].GetDouble());
        accumulated_transition_time += transition_time;
      }
    }
    // make sure the end of the trip path is the same as the legs
    EXPECT_NEAR(accumulated_time, route.GetObject()["duration"].GetDouble(), .01);
    // we should have had some transition costs along the way
    EXPECT_GT(accumulated_transition_time, 0);
  }
}

} // namespace

int main(int argc, char* argv[]) {
  valhalla::midgard::logging::Configure({{"type", ""}});
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
