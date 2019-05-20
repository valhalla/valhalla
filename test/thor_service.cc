#include "test.h"

#include "midgard/logging.h"
#include "thor/worker.h"
#include <unistd.h>

#include <thread>

#include <boost/property_tree/ptree.hpp>
#include <prime_server/http_protocol.hpp>
#include <prime_server/prime_server.hpp>

using namespace prime_server;
using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::thor;

// TODO: loki already validates: number of locations, invalid locations, invalid action, no or
// invalid costing essentially all well formed requestes are validated in loki. here we need to
// check
// for error conditions that happen as a result of data (no path) and also success conditions. to do
// this we should derive a thor_worker that has some canned results. we'll need to break apart some
// of the functions though in the base class to do it so its a reasonable amount of work

namespace {

std::list<std::pair<std::string, std::string>>
    failure_request_responses{
        /* {"{\"action\":0,\"locations\":[{\"lat\":\"40.743355\",\"lon\":\"-73.998182\",\"type\":\"break\"}],\"costing\":\"auto\",\"api_key\":\"valhalla-UdVXVeg\",\"correlated_1\":{\"edges\":[{\"id\":\"320546241000\",\"dist\":\"0.5481633\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-73.99814\",\"lat\":\"40.74347\"},\"location_index\":\"1\"}}","Insufficient
         number of locations provided"},
         {"{\"action\":-1,\"locations\":[{\"lat\":\"40.751158\",\"lon\":\"-74.000816\"},{\"lat\":\"40.745696\",\"lon\":\"-73.985023\"},{\"lat\":\"40.739193\",\"lon\":\"-73.980732\"},{\"lat\":\"40.73269\",\"lon\":\"-73.98468\"},{\"lat\":\"40.737893\",\"lon\":\"-73.99189\"}
         ],\"costing\":\"auto\",\"units\":\"mi\",\"correlated_0\":{\"edges\":[{\"id\":\"705396615\",\"dist\":\"0\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"0\"},{\"id\":\"839614343\",\"dist\":\"0\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"0\"},{\"id\":\"15737782151\",\"dist\":\"1\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"0\"},{\"id\":\"136667955080\",\"dist\":\"1\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"0\"}
         ],\"is_node\":\"true\",\"vertex\":{\"lon\":\"-74.00143\",\"lat\":\"40.75145\"},\"location_index\":\"0\"},\"correlated_1\":{\"edges\":[{\"id\":\"268738197992\",\"dist\":\"0.7749391\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-73.98515\",\"lat\":\"40.7454\"},\"location_index\":\"1\"},\"correlated_2\":{\"edges\":[{\"id\":\"358127204840\",\"dist\":\"0.288961\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-73.98063\",\"lat\":\"40.73945\"},\"location_index\":\"2\"},\"correlated_3\":{\"edges\":[{\"id\":\"120561826280\",\"dist\":\"0.4931332\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"0\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-73.98469\",\"lat\":\"40.73269\"},\"location_index\":\"3\"},\"correlated_4\":{\"edges\":[{\"id\":\"36944181736\",\"dist\":\"0.8432447\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"2\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-73.99194\",\"lat\":\"40.73779\"},\"location_index\":\"4\"}}","Unknown
         action"},
         {"{\"action\":0,\"locations\":[{\"lat\":\"40.04405976651413\",\"lon\":\"-76.29813373088837\",\"type\":\"break\"},{\"lat\":\"40.04260596866566\",\"lon\":\"-76.2991851568222\",\"type\":\"break\"}
         ],\"costing\":\"walk\",\"correlated_0\":{\"edges\":[{\"id\":\"1343687978654\",\"dist\":\"0.7936895\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"2\"},{\"id\":\"3091605450398\",\"dist\":\"0.2063105\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29807\",\"lat\":\"40.04407\"},\"location_index\":\"0\"},\"correlated_1\":{\"edges\":[{\"id\":\"2258650230430\",\"dist\":\"0.7210155\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"},{\"id\":\"2259052883614\",\"dist\":\"0.2789845\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"2\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29918\",\"lat\":\"40.04251\"},\"location_index\":\"1\"}}","No
         costing method found for 'walk'"},
         {"{\"action\":0,\"costing\":\"auto\",\"correlated_0\":{\"edges\":[{\"id\":\"1343687978654\",\"dist\":\"0.7936895\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"2\"},{\"id\":\"3091605450398\",\"dist\":\"0.2063105\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29807\",\"lat\":\"40.04407\"},\"location_index\":\"0\"},\"correlated_1\":{\"edges\":[{\"id\":\"2258650230430\",\"dist\":\"0.7210155\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"},{\"id\":\"2259052883614\",\"dist\":\"0.2789845\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"2\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29918\",\"lat\":\"40.04251\"},\"location_index\":\"1\"}}","Insufficiently
         specified required parameter 'locations'"},
         {"{\"action\":0,\"locations\":[{\"lat\":\"40.04405976651413\",\"lon\":\"-76.29813373088837\",\"type\":\"break\"},{\"lat\":\"40.04260596866566\"}],\"costing\":\"auto\",\"correlated_0\":{\"edges\":[{\"id\":\"1343687978654\",\"dist\":\"0.7936895\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"2\"},{\"id\":\"3091605450398\",\"dist\":\"0.2063105\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29807\",\"lat\":\"40.04407\"},\"location_index\":\"0\"},\"correlated_1\":{\"edges\":[{\"id\":\"2258650230430\",\"dist\":\"0.7210155\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"},{\"id\":\"2259052883614\",\"dist\":\"0.2789845\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"2\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29918\",\"lat\":\"40.04251\"},\"location_index\":\"1\"}}","Failed
         to parse location"},
         {"{\"action\":0,\"locations\":[{\"lat\":\"40.743355347975395\",\"lon\":\"-73.99818241596222\",\"type\":\"break\"}],\"costing\":\"auto\",\"api_key\":\"valhalla-UdVXVeg\",\"correlated_0\":{\"edges\":[{\"id\":\"14024310487527\",\"dist\":\"0\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"0\"},{\"id\":\"3523968903\",\"dist\":\"1\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"0\"}
         ],\"is_node\":\"true\",\"vertex\":{\"lon\":\"-74.0033\",\"lat\":\"40.74981\"},\"location_index\":\"0\"},\"correlated_1\":{\"edges\":[{\"id\":\"320546241000\",\"dist\":\"0.5481633\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"}
         ]}}","Failed to parse correlated location"},
         {"{\"action\":0,\"locations\":[{\"lat\":\"40.04405976651413\",\"lon\":\"-76.29813373088837\",\"type\":\"break\"},{\"lat\":\"40.04260596866566\",\"lon\":\"-76.2991851568222\",\"type\":\"break\"}
         ],\"correlated_0\":{\"edges\":[{\"id\":\"1343687978654\",\"dist\":\"0.7936895\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"2\"},{\"id\":\"3091605450398\",\"dist\":\"0.2063105\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29807\",\"lat\":\"40.04407\"},\"location_index\":\"0\"},\"correlated_1\":{\"edges\":[{\"id\":\"2258650230430\",\"dist\":\"0.7210155\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"},{\"id\":\"2259052883614\",\"dist\":\"0.2789845\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"2\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29918\",\"lat\":\"40.04251\"},\"location_index\":\"1\"}}","No
         edge/node costing provided"},
         {"{\"locations\":[{\"lat\":\"44.41416430998939\",\"lon\":\"-99.80682377703488\",\"type\":\"break\",\"date_time\":\"current\"},{\"lat\":\"44.35331432151491\",\"lon\":\"-99.57611088640988\",\"type\":\"break\"}
         ],\"costing\":\"multimodal\",\"date_time\":{\"type\":\"0\"},\"api_key\":\"valhalla-t_16n1c\",\"correlated_0\":{\"edges\":[{\"id\":\"58687475168\",\"dist\":\"0.2245807\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"},{\"id\":\"59090128352\",\"dist\":\"0.7754193\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"2\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-99.80527\",\"lat\":\"44.41417\"},\"location_index\":\"0\"},\"correlated_1\":{\"edges\":[{\"id\":\"28354268641\",\"dist\":\"0.3254639\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"2\"},{\"id\":\"28756921825\",\"dist\":\"0.6745361\",\"projected\":{\"lat\":0,\"lon\":0},\"sos\":\"1\"}
         ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-99.57611\",\"lat\":\"44.35286\"},\"location_index\":\"1\"}}","Cannot
         reach destination - too far from a transit stop"},
         */
    };

void test_failure_requests() {
  // service worker
  boost::property_tree::ptree config;
  config.add("mjolnir.tile_dir", "test/data/thor_service");
  config.add_child("costing_options.auto", {});
  config.add_child("costing_options.bicycle", {});
  config.add_child("costing_options.pedestrian", {});
  config.add_child("costing_options.transit", {});
  config.add_child("service_limits", {});
  config.add("thor.logging.long_request", "110.0");
  config.add("meili.default.gps_accuracy", "4.07");
  config.add("meili.default.search_radius", "40");
  config.add("meili.grid.size", "500");
  config.add("meili.grid.cache_size", "64");

  boost::property_tree::ptree customizable;
  boost::property_tree::ptree mode;
  boost::property_tree::ptree search_radius;
  mode.put("", "mode");
  search_radius.put("", "search_radius");
  customizable.push_back(std::make_pair("", mode));
  customizable.push_back(std::make_pair("", search_radius));
  config.add_child("meili.customizable", customizable);
  thor_worker_t worker(config);
  for (auto& req_resp : failure_request_responses) {
    std::list<zmq::message_t> messages;
    http_request_info_t request_info;

    messages.emplace_back(zmq::message_t(static_cast<void*>(&req_resp.first[0]),
                                         req_resp.first.size(), [](void*, void*) {}));

    auto result = worker.work(messages, &request_info, []() {});
    http_response_t response(400, "Bad Request", req_resp.second,
                             headers_t{{"Access-Control-Allow-Origin", "*"}});
    response.from_info(request_info);
    auto response_str = response.to_string();

    if (result.intermediate)
      throw std::logic_error(
          "This cant be intermediate right now we are only testing error scenarios");
    if (result.messages.front() != response_str)
      throw std::runtime_error("Expected Response: '" + response_str +
                               ",\n\n Actual Response: " + result.messages.front());
  }
}

} // namespace

int main(void) {
  // make this whole thing bail if it doesnt finish fast
  alarm(120);

  test::suite suite("Thor Service");

  suite.test(TEST_CASE(test_failure_requests));

  return suite.tear_down();
}
