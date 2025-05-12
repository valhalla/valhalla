#include "test.h"

#include <string>

#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "loki/worker.h"
#include "odin/worker.h"
#include "thor/worker.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::thor;
using namespace valhalla::odin;
using namespace valhalla::loki;
using namespace valhalla::tyr;

namespace {

const auto conf = test::make_config("test/data/utrecht_tiles");

struct route_tester {
  route_tester()
      : reader(new GraphReader(conf.get_child("mjolnir"))), loki_worker(conf, reader),
        thor_worker(conf, reader), odin_worker(conf) {
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
  EXPECT_TRUE(
      test::
          encoded_shape_equality(
              leg.shape(),
              "ibykbB}afxHWge@oAqh@y@sZoPy[{HuMeHmMgDgSq@iFS{@sDsIgAz@iR`LoHlEyIjFcSfF{P}@gJAmL{MqNkLaReSuUc^cIcPsIeUoOkf@sRqs@mKgg@_Qs~@kMy|@wHi{@sC__@y@{_@|@g^fEs_@lI}^lJaX|NqWjMyNpa@gYbk@yUto@_WnvAmn@~l@uVnc@mRdGcDnNaHza@kSfg@cUjqBs{@pdB}w@jrDsyAv`C}|@dZqKlWuL~{B}_Ajf@yOhgAy`@lr@q^fj@cYzp@qa@j]aU|ZaTv`@g[vmAobAjXyNp\\_PbPmAtTnA|UrJjNjKzMbQfM`XzKtZz\\p_BtQfbA~NhcAdJfj@~~@roEvn@dmC~DdPpKiJ~N_MdNsJdOkIbNmFx^iDlP\\hOdAdOpDzCnApChAnGhCbOvJlQjPdL`LfYnZpgAhnAd|@t{@xZlSne@x\\xaBfdAnd@xZ|uBjtAvKjHdkAfw@fWzNvJrF~IdGxLrIpLlIrSjQ`PdRbNlTlPzZvPjc@fK`Zbg@d{A~Pph@hXvv@tMb^j[zs@~Qz]jMnTtRx[nh@pu@~UzZ`UtZr^tc@na@tf@fYrZx`@j^n_@`X~]tSzyAzl@~|@l]~}Avm@zjAdd@|y@hZbgApa@rStJbJjIjElGpDnHhIzTxHjXrB`Mv@zZIzb@u@hXeQ~qBm_@`vDap@baFsHvm@sE|SsFzX_Kjd@eIj\\mHnXwHzZ{DrQqDzS{ApLyBp\\]dEYtDGtCb@jE|@zCxBxDtHfBno@pOr[hEbT|A`LWbGWhJeB`C]`S{EpWqKtHyDjeB}t@hViFpKk@|MtAnSdH~VdS~Wld@dKfSnJfWtIpa@b\\~yBrChQhGlVrH`UpLpV~@jBrCnGdInUdGjZ|Ejd@|Ala@dAxa@@d_@{Apc@kB`_@aCx^iAfKiBzKqB~LwCnQwHb]cHvWiHdU}[nz@kEbLaP`a@|F|GxDdFrl@~{@|n@``Ahi@~v@pUt_@dg@h~@b{@jaBj\\dp@bYjg@fWdd@vGnL}OtnAqCtd@uB`OaR~uAq@lDoCfMaC`NaAjFsAjKu@pHcCxUSlJ{ClXmB`H}@nF[fFIzHp@nH|CtKxDrGpDlC~WdO|CxBE~TaKh}@sVaLyBbAsFlh@A`FMt`@c@xC"));

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
          transition_time += trip_leg->node(n).cost().transition_cost().seconds();
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
      EXPECT_EQ(trip_leg->node().rbegin()->cost().elapsed_cost().seconds(), leg.summary().time());
      // make sure the maneuvers add up to the leg as well
      EXPECT_EQ(accumulated_time, leg.summary().time());
      // we should have had some transition costs along the way
      EXPECT_GT(accumulated_transition_time, 0);
      // we should have the edge time plus the transition time add up to the leg time
      EXPECT_NEAR(accumulated_edge_time + accumulated_transition_time, accumulated_time, .16);
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
