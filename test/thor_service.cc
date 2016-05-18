#include "test.h"

#include "thor/service.h"
#include <valhalla/midgard/logging.h>
#include <unistd.h>

#include <thread>

#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using namespace prime_server;
using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::thor;


namespace {

  std::list<std::pair<std::string, std::string>> failure_request_responses {
    {"{\"locations\":[{\"lat\":\"40.743355\",\"lon\":\"-73.998182\",\"type\":\"break\"}],\"costing\":\"auto\",\"api_key\":\"valhalla-UdVXVeg\",\"correlated_1\":{\"edges\":[{\"id\":\"320546241000\",\"dist\":\"0.5481633\",\"sos\":\"1\"}        ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-73.99814\",\"lat\":\"40.74347\"},\"location_index\":\"1\"}}","Insufficient number of locations provided"},
    {"{\"locations\":[{\"lat\":\"40.751158\",\"lon\":\"-74.000816\"},{\"lat\":\"40.745696\",\"lon\":\"-73.985023\"},{\"lat\":\"40.739193\",\"lon\":\"-73.980732\"},{\"lat\":\"40.73269\",\"lon\":\"-73.98468\"},{\"lat\":\"40.737893\",\"lon\":\"-73.99189\"}    ],\"costing\":\"auto\",\"units\":\"mi\",\"correlated_0\":{\"edges\":[{\"id\":\"705396615\",\"dist\":\"0\",\"sos\":\"0\"},{\"id\":\"839614343\",\"dist\":\"0\",\"sos\":\"0\"},{\"id\":\"15737782151\",\"dist\":\"1\",\"sos\":\"0\"},{\"id\":\"136667955080\",\"dist\":\"1\",\"sos\":\"0\"}        ],\"is_node\":\"true\",\"vertex\":{\"lon\":\"-74.00143\",\"lat\":\"40.75145\"},\"location_index\":\"0\"},\"correlated_1\":{\"edges\":[{\"id\":\"268738197992\",\"dist\":\"0.7749391\",\"sos\":\"1\"}        ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-73.98515\",\"lat\":\"40.7454\"},\"location_index\":\"1\"},\"correlated_2\":{\"edges\":[{\"id\":\"358127204840\",\"dist\":\"0.288961\",\"sos\":\"1\"}        ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-73.98063\",\"lat\":\"40.73945\"},\"location_index\":\"2\"},\"correlated_3\":{\"edges\":[{\"id\":\"120561826280\",\"dist\":\"0.4931332\",\"sos\":\"0\"}        ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-73.98469\",\"lat\":\"40.73269\"},\"location_index\":\"3\"},\"correlated_4\":{\"edges\":[{\"id\":\"36944181736\",\"dist\":\"0.8432447\",\"sos\":\"2\"}        ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-73.99194\",\"lat\":\"40.73779\"},\"location_index\":\"4\"},\"matrix_type\":\"wrong_type\"}","Incorrect type provided:: wrong_type  Accepted types are 'one_to_many', 'many_to_one', 'many_to_many' or 'optimized'."},
    {"{\"locations\":[{\"lat\":\"40.04405976651413\",\"lon\":\"-76.29813373088837\",\"type\":\"break\"},{\"lat\":\"40.04260596866566\",\"lon\":\"-76.2991851568222\",\"type\":\"break\"}    ],\"costing\":\"walk\",\"correlated_0\":{\"edges\":[{\"id\":\"1343687978654\",\"dist\":\"0.7936895\",\"sos\":\"2\"},{\"id\":\"3091605450398\",\"dist\":\"0.2063105\",\"sos\":\"1\"}        ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29807\",\"lat\":\"40.04407\"},\"location_index\":\"0\"},\"correlated_1\":{\"edges\":[{\"id\":\"2258650230430\",\"dist\":\"0.7210155\",\"sos\":\"1\"},{\"id\":\"2259052883614\",\"dist\":\"0.2789845\",\"sos\":\"2\"}        ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29918\",\"lat\":\"40.04251\"},\"location_index\":\"1\"}}","No costing method found for 'walk'"},
    {"{\"costing\":\"auto\",\"correlated_0\":{\"edges\":[{\"id\":\"1343687978654\",\"dist\":\"0.7936895\",\"sos\":\"2\"},{\"id\":\"3091605450398\",\"dist\":\"0.2063105\",\"sos\":\"1\"}        ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29807\",\"lat\":\"40.04407\"},\"location_index\":\"0\"},\"correlated_1\":{\"edges\":[{\"id\":\"2258650230430\",\"dist\":\"0.7210155\",\"sos\":\"1\"},{\"id\":\"2259052883614\",\"dist\":\"0.2789845\",\"sos\":\"2\"}        ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29918\",\"lat\":\"40.04251\"},\"location_index\":\"1\"}}","Insufficiently specified required parameter 'locations'"},
    {"{\"locations\":[{\"lat\":\"40.04405976651413\",\"lon\":\"-76.29813373088837\",\"type\":\"break\"},{\"lat\":\"40.04260596866566\"}],\"costing\":\"auto\",\"correlated_0\":{\"edges\":[{\"id\":\"1343687978654\",\"dist\":\"0.7936895\",\"sos\":\"2\"},{\"id\":\"3091605450398\",\"dist\":\"0.2063105\",\"sos\":\"1\"}        ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29807\",\"lat\":\"40.04407\"},\"location_index\":\"0\"},\"correlated_1\":{\"edges\":[{\"id\":\"2258650230430\",\"dist\":\"0.7210155\",\"sos\":\"1\"},{\"id\":\"2259052883614\",\"dist\":\"0.2789845\",\"sos\":\"2\"}        ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29918\",\"lat\":\"40.04251\"},\"location_index\":\"1\"}}","Failed to parse location"},
    {"{\"locations\":[{\"lat\":\"40.743355347975395\",\"lon\":\"-73.99818241596222\",\"type\":\"break\"}],\"costing\":\"auto\",\"api_key\":\"valhalla-UdVXVeg\",\"correlated_0\":{\"edges\":[{\"id\":\"14024310487527\",\"dist\":\"0\",\"sos\":\"0\"},{\"id\":\"3523968903\",\"dist\":\"1\",\"sos\":\"0\"}        ],\"is_node\":\"true\",\"vertex\":{\"lon\":\"-74.0033\",\"lat\":\"40.74981\"},\"location_index\":\"0\"},\"correlated_1\":{\"edges\":[{\"id\":\"320546241000\",\"dist\":\"0.5481633\",\"sos\":\"1\"}        ]}}","Failed to parse correlated location"},
    {"{\"locations\":[{\"lat\":\"40.04405976651413\",\"lon\":\"-76.29813373088837\",\"type\":\"break\"},{\"lat\":\"40.04260596866566\",\"lon\":\"-76.2991851568222\",\"type\":\"break\"}    ],\"correlated_0\":{\"edges\":[{\"id\":\"1343687978654\",\"dist\":\"0.7936895\",\"sos\":\"2\"},{\"id\":\"3091605450398\",\"dist\":\"0.2063105\",\"sos\":\"1\"}        ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29807\",\"lat\":\"40.04407\"},\"location_index\":\"0\"},\"correlated_1\":{\"edges\":[{\"id\":\"2258650230430\",\"dist\":\"0.7210155\",\"sos\":\"1\"},{\"id\":\"2259052883614\",\"dist\":\"0.2789845\",\"sos\":\"2\"}        ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-76.29918\",\"lat\":\"40.04251\"},\"location_index\":\"1\"}}","No edge/node costing provided"}
    //transit
   // {"{\"locations\":[{\"lat\":\"44.41416430998939\",\"lon\":\"-99.80682377703488\",\"type\":\"break\",\"date_time\":\"current\"},{\"lat\":\"44.35331432151491\",\"lon\":\"-99.57611088640988\",\"type\":\"break\"}    ],\"costing\":\"multimodal\",\"date_time\":{\"type\":\"0\"},\"api_key\":\"valhalla-t_16n1c\",\"correlated_0\":{\"edges\":[{\"id\":\"58687475168\",\"dist\":\"0.2245807\",\"sos\":\"1\"},{\"id\":\"59090128352\",\"dist\":\"0.7754193\",\"sos\":\"2\"}        ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-99.80527\",\"lat\":\"44.41417\"},\"location_index\":\"0\"},\"correlated_1\":{\"edges\":[{\"id\":\"28354268641\",\"dist\":\"0.3254639\",\"sos\":\"2\"},{\"id\":\"28756921825\",\"dist\":\"0.6745361\",\"sos\":\"1\"}        ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-99.57611\",\"lat\":\"44.35286\"},\"location_index\":\"1\"}}","Cannot reach destination - too far from a transit stop"}
  };

  std::list<std::pair<std::string, std::string>> success_request_responses {
    {"locations\":[{\"lat\":\"40.71955054111941\",\"lon\":\"-74.00016546947882\",\"type\":\"break\"},{\"lat\":\"40.737892702684064\",\"lon\":\"-73.98368597729132\",\"type\":\"through\"},{\"lat\":\"40.748557450215635\",\"lon\":\"-73.977506167721\",\"type\":\"through\"},{\"lat\":\"40.727486422997785\",\"lon\":\"-73.98574591381475\",\"type\":\"through\"},{\"lat\":\"40.75661990450192\",\"lon\":\"-73.97407294018194\",\"type\":\"through\"},{\"lat\":\"40.75661990450192\",\"lon\":\"-73.97407294018194\",\"type\":\"break\"}    ],\"costing\":\"auto\",\"correlated_0\":{\"edges\":[{\"id\":\"268201327079\",\"dist\":\"0.2520433\",\"sos\":\"2\"}        ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-74.00043\",\"lat\":\"40.71978\"},\"location_index\":\"0\"},\"correlated_1\":{\"edges\":[{\"id\":\"324035901928\",\"dist\":\"0.3327826\",\"sos\":\"1\"},{\"id\":\"12523219417576\",\"dist\":\"0.6672174\",\"sos\":\"2\"}        ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-73.98378\",\"lat\":\"40.73796\"},\"location_index\":\"1\"},\"correlated_2\":{\"edges\":[{\"id\":\"432215390696\",\"dist\":\"0.2814217\",\"sos\":\"2\"}        ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-73.97744\",\"lat\":\"40.74873\"},\"location_index\":\"2\"},\"correlated_3\":{\"edges\":[{\"id\":\"178812320232\",\"dist\":\"0.5289862\",\"sos\":\"1\"}        ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-73.98552\",\"lat\":\"40.72731\"},\"location_index\":\"3\"},\"correlated_4\":{\"edges\":[{\"id\":\"871644233608\",\"dist\":\"0.3149269\",\"sos\":\"2\"}        ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-73.97421\",\"lat\":\"40.75672\"},\"location_index\":\"4\"},\"correlated_5\":{\"edges\":[{\"id\":\"871644233608\",\"dist\":\"0.3149269\",\"sos\":\"2\"}        ],\"is_node\":\"false\",\"vertex\":{\"lon\":\"-73.97421\",\"lat\":\"40.75672\"},\"location_index\":\"5\"},\"matrix_type\":\"optimized","Found route between points"},
  };

  class thor_testable_worker_t : public thor_worker_t {
   protected:
    virtual std::list<valhalla::odin::TripPath> path_arrive_by(std::vector<baldr::PathLocation>& correlated, const std::string &costing, const std::string &request_str) {
      std::list<valhalla::odin::TripPath> tps;
      //TODO:
      return tps;
    }
    virtual std::list<valhalla::odin::TripPath> path_depart_from(std::vector<baldr::PathLocation>& correlated, const std::string &costing, const boost::optional<int> &date_time_type, const std::string &request_str) {
      std::list<valhalla::odin::TripPath> tps;
      //TODO:
      return tps;
    }
    virtual prime_server::worker_t::result_t matrix(const MATRIX_TYPE matrix_type, const std::string &costing, const boost::property_tree::ptree &request, prime_server::http_request_t::info_t& request_info) {
      worker_t::result_t result{false};
      //TODO:
      return result;
    }
  };

  void test_failure_requests() {
    //service worker
    boost::property_tree::ptree config;
    config.add("mjolnir.tile_dir", "test/data/tiles");
    config.add_child("costing_options.auto", {});
    config.add_child("costing_options.bicycle", {});
    config.add_child("costing_options.pedestrian", {});
    config.add_child("costing_options.transit", {});
    config.add("thor.logging.long_request_route", "110.0");
    config.add("thor.logging.long_request_manytomany", "15000.0");

    thor_worker_t worker(config);
    for (auto& req_resp : failure_request_responses) {
      std::list<zmq::message_t> messages;
      http_request_t::info_t request_info;
      messages.emplace_back(zmq::message_t(static_cast<void*>(&req_resp.first[0]), req_resp.first.size(), [](void*, void*){}));

      auto result = worker.work(messages, &request_info);
      http_response_t response(400, "Bad Request", req_resp.second, headers_t{{"Access-Control-Allow-Origin", "*"}});
      response.from_info(request_info);
      auto response_str = response.to_string();

      if (result.intermediate)
        throw std::logic_error("This cant be intermediate right now we are only testing error scenarios");
      if(result.messages.front() != response_str)
        throw std::runtime_error("Expected Response: '" + response_str +",\n\n Actual Response: " + result.messages.front());
    }
  }

  void test_success_requests() {
    boost::property_tree::ptree config;
    config.add("mjolnir.tile_dir", "test/data/tiles");
    config.add_child("costing_options.auto", {});
    config.add_child("costing_options.bicycle", {});
    config.add_child("costing_options.pedestrian", {});
    config.add_child("costing_options.transit", {});
    config.add("thor.logging.long_request_route", "110.0");
    config.add("thor.logging.long_request_manytomany", "15000.0");

    thor_worker_t worker(config);
    for (auto& req_resp : failure_request_responses) {
      std::list<zmq::message_t> messages;
      http_request_t::info_t request_info;
      messages.emplace_back(zmq::message_t(static_cast<void*>(&req_resp.first[0]), req_resp.first.size(), [](void*, void*){}));

      auto result = worker.work(messages, &request_info);
      http_response_t response(200, "OK", req_resp.second, headers_t{{"Access-Control-Allow-Origin", "*"}});
      response.from_info(request_info);
      auto response_str = response.to_string();

      if (result.intermediate)
        throw std::logic_error("This cant be intermediate right now we are only testing error scenarios");
      if(result.messages.front() != response_str)
        throw std::runtime_error("Expected Response: '" + response_str +",\n\n Actual Response: " + result.messages.front());
    }
  }
}

int main(void) {
  //make this whole thing bail if it doesnt finish fast
  alarm(30);

  test::suite suite("Thor Service");

  suite.test(TEST_CASE(test_failure_requests));
  //suite.test(TEST_CASE(test_success_requests));

  return suite.tear_down();
}


