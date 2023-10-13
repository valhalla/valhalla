#include "gurka.h"
#include "loki/worker.h"
#include "midgard/pointll.h"
#include "pixels.h"
#include "test.h"

#include <gtest/gtest.h>
#include <prime_server/http_protocol.hpp>
#include <prime_server/prime_server.hpp>

using namespace valhalla;
using namespace valhalla::gurka;
using namespace prime_server;

namespace {
std::string shape;
std::vector<http_request_t> requests;
std::vector<std::string> responses;
const std::string workdir = "test/data/gurka_elevation_in_tiles";

zmq::context_t context;
void start_service(const boost::property_tree::ptree config) {
  // need a place to drop our sockets
  auto run_dir = config.get<std::string>("mjolnir.tile_dir");
  if (!filesystem::is_directory(run_dir) && !filesystem::create_directories(run_dir))
    throw std::runtime_error("Couldnt make directory to run from");

  // server
  std::thread server(std::bind(&http_server_t::serve,
                               http_server_t(context, config.get<std::string>("httpd.service.listen"),
                                             config.get<std::string>("loki.service.proxy") + "_in",
                                             config.get<std::string>("httpd.service.loopback"),
                                             config.get<std::string>("httpd.service.interrupt"))));
  server.detach();

  // load balancer
  std::thread proxy(std::bind(&proxy_t::forward,
                              proxy_t(context, config.get<std::string>("loki.service.proxy") + "_in",
                                      config.get<std::string>("loki.service.proxy") + "_out")));
  proxy.detach();

  // service worker
  std::thread worker(valhalla::loki::run_service, config);
  worker.detach();
}

} // namespace

TEST(Standalone, ElevationRealWorld) {
  const std::string ascii_map = R"(
                 A---B
                 |   |
                 |   C
                 |
              D--E-----F------G
              |        |
              |        |
              |        1
              H--I-----J
                       2
                       |
                       |
             K---L-----M
             |         |
             |         |
             N---O-----P
             |   |     |
             |   |     |
             Q---R-----S
  )";

  const gurka::ways ways = {
      {"KNQ", {{"highway", "service"}, {"service", "alley"}}},
      {"NOP", {{"highway", "residential"}, {"name", "East Chestnut Street"}}},
      {"QRS", {{"highway", "service"}, {"service", "alley"}}},
      {"OR", {{"highway", "service"}, {"service", "alley"}}},
      {"SPM2J1F", {{"highway", "service"}, {"service", "alley"}}},
      {"KLM", {{"highway", "service"}, {"service", "alley"}, {"name", "East Center Alley"}}},
      {"DEFG", {{"highway", "service"}, {"service", "alley"}, {"name", "North Alley"}}},
      {"DH", {{"highway", "service"}, {"service", "alley"}}},
      {"HIJ", {{"highway", "secondary"}, {"name", "East Main Street"}}},
      {"EABC", {{"highway", "service"}, {"service", "driveway"}}},
  };

  // Create our layout based on real world data.
  // Really the only one we care about is SPM2J1F or omsid 326371867
  using nodelayout = std::map<std::string, midgard::PointLL>;
  nodelayout layout;

  layout.insert({"1", {-76.4945823, 40.6517478}});
  layout.insert({"2", {-76.4945322, 40.6516377}});
  layout.insert({"A", {-76.4951627, 40.6526995}});
  layout.insert({"B", {-76.4948476, 40.6527545}});
  layout.insert({"C", {-76.4948142, 40.6526154}});
  layout.insert({"D", {-76.4956555, 40.6521481}});
  layout.insert({"E", {-76.4951254, 40.6521625}});
  layout.insert({"F", {-76.4946354, 40.6521889}});
  layout.insert({"G", {-76.494053, 40.6522276}});
  layout.insert({"H", {-76.4956165, 40.6516304}});
  layout.insert({"I", {-76.4950803, 40.6516647}});
  layout.insert({"J", {-76.4945597, 40.651698}});
  layout.insert({"K", {-76.4958143, 40.6511348}});
  layout.insert({"L", {-76.4951349, 40.6506908}});
  layout.insert({"M", {-76.4944987, 40.6512172}});
  layout.insert({"N", {-76.4957653, 40.6506541}});
  layout.insert({"O", {-76.4951349, 40.6506908}});
  layout.insert({"P", {-76.4944599, 40.6507301}});
  layout.insert({"Q", {-76.4957235, 40.6502434}});
  layout.insert({"R", {-76.4950865, 40.6501919}});
  layout.insert({"S", {-76.4944069, 40.6502916}});

  // create the test elevation tile
  std::vector<int16_t> tile(3601 * 3601, 0);
  for (const auto& p : pixels)
    tile[p.first] = p.second;

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  // actually store it
  std::ofstream file(workdir + "/N40W077.hgt", std::ios::binary | std::ios::trunc);
  file.write(static_cast<const char*>(static_cast<void*>(tile.data())),
             sizeof(int16_t) * tile.size());
  ASSERT_TRUE(file.good()) << "File stream is not good";

  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map map;
  map.nodes = layout;
  map.config = test::make_config(workdir, {});
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("additional_data.elevation", workdir);

  std::vector<std::string> input_files = {pbf_filename};
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  std::string route_json;
  // ~60m test
  auto route = gurka::do_action(valhalla::Options::route, map, {"1", "M"}, "bicycle",
                                {{"/elevation_interval", "30"}}, {}, &route_json);

  rapidjson::Document result;
  result.Parse(route_json.c_str());

  auto s = rapidjson::get_child_optional(result, "/trip/legs/0/shape");
  EXPECT_TRUE(s && s->IsString());
  shape = s->GetString();

  auto elevation = rapidjson::get_child_optional(result, "/trip/legs/0/elevation");
  EXPECT_TRUE(elevation && elevation->IsArray());

  std::vector<float> expected_elevation = {252.9, 257.9, 258};

  size_t i = 0;
  if (elevation && elevation->IsArray()) {
    for (const auto& e : elevation->GetArray()) {
      EXPECT_EQ(e.GetFloat(), expected_elevation[i++]);
    }
  }

  requests.push_back(
      http_request_t(POST, "/height",
                     "{\"resample_distance\":30,\"height_precision\":1,\"encoded_polyline\":\"" +
                         shape + "\"}"));

  std::string res = std::string("{\"encoded_polyline\":\"g}dplAjnz{pClO{BvOi@\",\"height\":[");
  for (const auto& e : expected_elevation) {
    res += std::to_string(e) + ",";
  }
  res += "]}";
  std::cout << res << std::endl;

  responses.push_back(res);
}

/*
 * sigh. more bad data

TEST(Standalone, ElevationFakeEdge) {
  const std::string ascii_map = R"(
                 A
                 |
                 |
                 |
                 B
                 |
                 |
                 |
                 C
                 |
                 |
                 |
                 D
                 |
                 |
                 |
                 E

  )";

  const gurka::ways ways = {
      {"ABCDE", {{"highway", "residential"}, {"name", "East Chestnut Street"}}},
  };

  // Create our layout
  using nodelayout = std::map<std::string, midgard::PointLL>;
  nodelayout layout;

  layout.insert({"A", {-76.537011, 40.723872}});// 300m
  layout.insert({"B", {-76.537011, 40.726872}});// 350m
  layout.insert({"C", {-76.537011, 40.729872}});// 450m
  layout.insert({"D", {-76.537011, 40.732872}});// 325m
  layout.insert({"E", {-76.537011, 40.735872}});// 250m

  // create the test elevation tile
  std::vector<int16_t> tile(3601 * 3601, 0);
  for (const auto& p : pixels)
    tile[p.first] = p.second;

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  // actually store it
  std::ofstream file(workdir + "/N40W077.hgt", std::ios::binary | std::ios::trunc);
  file.write(static_cast<const char*>(static_cast<void*>(tile.data())),
             sizeof(int16_t) * tile.size());
  ASSERT_TRUE(file.good()) << "File stream is not good";

  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, {}, {}, pbf_filename);

  valhalla::gurka::map map;
  map.nodes = layout;
  map.config = test::make_config(workdir, {});
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("additional_data.elevation", workdir);

  std::vector<std::string> input_files = {pbf_filename};
  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  std::string route_json;
  auto route = gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "bicycle",
                                {{"/elevation_interval", "30"}}, {}, &route_json);

  rapidjson::Document result;
  result.Parse(route_json.c_str());

  auto s = rapidjson::get_child_optional(result, "/trip/legs/0/shape");
  EXPECT_TRUE(s && s->IsString());
  shape = s->GetString();

  auto elevation = rapidjson::get_child_optional(result, "/trip/legs/0/elevation");
  EXPECT_TRUE(elevation && elevation->IsArray());

  std::vector<float> expected_elevation = {252.9, 257.9, 258};

  size_t i = 0;
  if (elevation && elevation->IsArray()) {
    for (const auto& e : elevation->GetArray()) {
      EXPECT_EQ(e.GetFloat(), expected_elevation[i++]);
    }
  }

  requests.push_back(
  http_request_t(
      POST,
      "/height",
      "{\"resample_distance\":30,\"height_precision\":1,\"encoded_polyline\":\"" + shape + "\"}" ));

  std::string res = std::string(
      "{\"encoded_polyline\":\"g}dplAjnz{pClO{BvOi@\",\"height\":[");
  for (const auto& e : expected_elevation)
          res += std::to_string(e) + ",";
  res += "]}";
  responses.push_back(res);
}
*/

TEST(Standalone, ElevationCompareToSkadi) {

  auto config = test::make_config(workdir, {});
  config.put("additional_data.elevation", workdir);

  // start up the service
  start_service(config);

  // client makes requests and gets back responses in a batch fashion
  auto request = requests.cbegin();
  std::string request_str;
  http_client_t client(
      context, config.get<std::string>("httpd.service.listen"),
      [&request, &request_str]() {
        // we dont have any more requests so bail
        if (request == requests.cend())
          return std::make_pair<const void*, size_t>(nullptr, 0);
        // get the string of bytes to send formatted for http protocol
        request_str = request->to_string();
        ++request;
        return std::make_pair<const void*, size_t>(request_str.c_str(), request_str.size());
      },
      [&request](const void* data, size_t size) {
        auto response = http_response_t::from_string(static_cast<const char*>(data), size);
        EXPECT_EQ(response.body, responses[request - requests.cbegin() - 1]);
        return request != requests.cend();
      },
      1);

  // make this whole thing bail if it doesnt finish fast
  alarm(120);

  // request and receive
  client.batch();
}
