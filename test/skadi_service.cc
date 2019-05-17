#include "loki/worker.h"
#include "pixels.h"
#include "test.h"

#include <boost/property_tree/ptree.hpp>
#include <prime_server/http_protocol.hpp>
#include <prime_server/prime_server.hpp>

#include <fstream>
#include <thread>
#include <unistd.h>

using namespace valhalla;
using namespace prime_server;

namespace {
const std::vector<http_request_t> requests{
    http_request_t(
        POST,
        "/height",
        "{\"range\":true,\"shape\":[{\"lat\":40.712431, \"lon\":-76.504916},{\"lat\":40.712275, "
        "\"lon\":-76.605259},{\"lat\":40.712122, \"lon\":-76.805694},{\"lat\":40.722431, "
        "\"lon\":-76.884916},{\"lat\":40.812275, \"lon\":-76.905259},{\"lat\":40.912122, "
        "\"lon\":-76.965694}]}"),
    http_request_t(
        POST,
        "/height",
        "{\"range\":true,\"encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@"
        "rFodDbRaG\\\\ypAfDec@l@mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@"
        "?ofA_Tus@c[qfAgw@s_Agc@}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_"
        "IxaIcF~|@cBngJiMjj@_I`HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@"
        "jVd@rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_"
        "IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\"}"),
    http_request_t(GET,
                   "/height?json={\"range\":true,\"shape\":[{\"lat\":40.712431, "
                   "\"lon\":-76.504916},{\"lat\":40.712275, \"lon\":-76.605259},{\"lat\":40.712122, "
                   "\"lon\":-76.805694},{\"lat\":40.722431, \"lon\":-76.884916},{\"lat\":40.812275, "
                   "\"lon\":-76.905259},{\"lat\":40.912122, \"lon\":-76.965694}]}"),
    http_request_t(
        GET,
        "/height?json={\"range\":true,\"encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{["
        "hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_"
        "SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@"
        "eZpuE_IxaIcF~|@cBngJiMjj@_I`HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@"
        "mOjUaCvDqBrEcAbGWbG|@jVd@rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_"
        "I`HwDpGoH|T_IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\"}"),
    http_request_t(POST,
                   "/height",
                   "{\"shape\":[{\"lat\":40.712431, \"lon\":-76.504916},{\"lat\":40.712275, "
                   "\"lon\":-76.605259},{\"lat\":40.712122, \"lon\":-76.805694},{\"lat\":40.722431, "
                   "\"lon\":-76.884916},{\"lat\":40.812275, \"lon\":-76.905259},{\"lat\":40.912122, "
                   "\"lon\":-76.965694}]}"),
    http_request_t(
        POST,
        "/height",
        "{\"encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@"
        "mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@"
        "}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`"
        "HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@"
        "rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_"
        "IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\"}"),
    http_request_t(
        GET,
        "/height?json={\"shape\":[{\"lat\":40.712431, \"lon\":-76.504916},{\"lat\":40.712275, "
        "\"lon\":-76.605259},{\"lat\":40.712122, \"lon\":-76.805694},{\"lat\":40.722431, "
        "\"lon\":-76.884916},{\"lat\":40.812275, \"lon\":-76.905259},{\"lat\":40.912122, "
        "\"lon\":-76.965694}]}"),
    http_request_t(
        GET,
        "/height?json={\"encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@"
        "rFodDbRaG\\\\ypAfDec@l@mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@"
        "?ofA_Tus@c[qfAgw@s_Agc@}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_"
        "IxaIcF~|@cBngJiMjj@_I`HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@"
        "jVd@rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_"
        "IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\"}"),
};

const std::vector<std::string> responses{
    std::string(
        "{\"shape\":[{\"lat\":40.712433,\"lon\":-76.504913},{\"lat\":40.712276,\"lon\":-76.605263},"
        "{\"lat\":40.712124,\"lon\":-76.805695},{\"lat\":40.722431,\"lon\":-76.884918},{\"lat\":40."
        "812275,\"lon\":-76.905258},{\"lat\":40.912121,\"lon\":-76.965691}],\"range_height\":[[0,"
        "307],[8467,272],[25380,204],[32162,204],[42309,180],[54533,198]]}"),
    std::string(
        "{\"encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@"
        "mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@"
        "}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`"
        "HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@"
        "rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_"
        "IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\",\"range_height\":[[0,258],[54,258],[143,259],[242,257],["
        "323,255],[374,253],[424,246],[489,245],[784,233],[799,233],[945,221],[1009,216],[1215,223]"
        ",[1324,227],[1364,227],[1426,228],[1485,230],[1533,232],[1598,236],[1817,251],[1836,251],["
        "1983,251],[2259,251],[2275,251],[2313,254],[2339,258],[2440,267],[2571,283],[2672,289],["
        "2821,298],[2946,308],[3005,316],[3018,318],[3027,320],[3042,322],[3052,323],[3066,324],["
        "3120,328],[3397,359],[4322,445],[4382,452],[4499,463],[4562,463],[4856,448],[5292,405],["
        "5377,393],[5864,336],[5928,329],[5949,326],[5998,316],[6028,311],[6050,309],[6070,308],["
        "6266,289],[6296,291],[6307,292],[6318,292],[6329,291],[6340,289],[6441,278],[6483,279],["
        "6494,279],[6505,280],[6516,281],[6527,281],[6559,280],[6583,281],[6595,281],[6603,282],["
        "6612,282],[6633,282],[6674,280],[6717,276],[7022,251],[7070,248],[7113,247],[7125,246],["
        "7140,244],[7150,243],[7173,240],[7185,239],[7203,239],[7225,238],[7240,239],[7274,241],["
        "7300,241],[7331,239],[7362,236],[7492,221],[7500,221],[7576,225],[7609,224]]}"),
    std::string(
        "{\"shape\":[{\"lat\":40.712433,\"lon\":-76.504913},{\"lat\":40.712276,\"lon\":-76.605263},"
        "{\"lat\":40.712124,\"lon\":-76.805695},{\"lat\":40.722431,\"lon\":-76.884918},{\"lat\":40."
        "812275,\"lon\":-76.905258},{\"lat\":40.912121,\"lon\":-76.965691}],\"range_height\":[[0,"
        "307],[8467,272],[25380,204],[32162,204],[42309,180],[54533,198]]}"),
    std::string(
        "{\"encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@"
        "mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@"
        "}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`"
        "HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@"
        "rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_"
        "IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\",\"range_height\":[[0,258],[54,258],[143,259],[242,257],["
        "323,255],[374,253],[424,246],[489,245],[784,233],[799,233],[945,221],[1009,216],[1215,223]"
        ",[1324,227],[1364,227],[1426,228],[1485,230],[1533,232],[1598,236],[1817,251],[1836,251],["
        "1983,251],[2259,251],[2275,251],[2313,254],[2339,258],[2440,267],[2571,283],[2672,289],["
        "2821,298],[2946,308],[3005,316],[3018,318],[3027,320],[3042,322],[3052,323],[3066,324],["
        "3120,328],[3397,359],[4322,445],[4382,452],[4499,463],[4562,463],[4856,448],[5292,405],["
        "5377,393],[5864,336],[5928,329],[5949,326],[5998,316],[6028,311],[6050,309],[6070,308],["
        "6266,289],[6296,291],[6307,292],[6318,292],[6329,291],[6340,289],[6441,278],[6483,279],["
        "6494,279],[6505,280],[6516,281],[6527,281],[6559,280],[6583,281],[6595,281],[6603,282],["
        "6612,282],[6633,282],[6674,280],[6717,276],[7022,251],[7070,248],[7113,247],[7125,246],["
        "7140,244],[7150,243],[7173,240],[7185,239],[7203,239],[7225,238],[7240,239],[7274,241],["
        "7300,241],[7331,239],[7362,236],[7492,221],[7500,221],[7576,225],[7609,224]]}"),
    std::string("{\"shape\":[{\"lat\":40.712433,\"lon\":-76.504913},{\"lat\":40.712276,\"lon\":-76."
                "605263},{\"lat\":40.712124,\"lon\":-76.805695},{\"lat\":40.722431,\"lon\":-76."
                "884918},{\"lat\":40.812275,\"lon\":-76.905258},{\"lat\":40.912121,\"lon\":-76."
                "965691}],\"height\":[307,272,204,204,180,198]}"),
    std::string(
        "{\"encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@"
        "mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@"
        "}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`"
        "HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@"
        "rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_"
        "IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\",\"height\":[258,258,259,257,255,253,246,245,233,233,221,"
        "216,223,227,227,228,230,232,236,251,251,251,251,251,254,258,267,283,289,298,308,316,318,"
        "320,322,323,324,328,359,445,452,463,463,448,405,393,336,329,326,316,311,309,308,289,291,"
        "292,292,291,289,278,279,279,280,281,281,280,281,281,282,282,282,280,276,251,248,247,246,"
        "244,243,240,239,239,238,239,241,241,239,236,221,221,225,224]}"),
    std::string("{\"shape\":[{\"lat\":40.712433,\"lon\":-76.504913},{\"lat\":40.712276,\"lon\":-76."
                "605263},{\"lat\":40.712124,\"lon\":-76.805695},{\"lat\":40.722431,\"lon\":-76."
                "884918},{\"lat\":40.812275,\"lon\":-76.905258},{\"lat\":40.912121,\"lon\":-76."
                "965691}],\"height\":[307,272,204,204,180,198]}"),
    std::string(
        "{\"encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@"
        "mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@"
        "}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`"
        "HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@"
        "rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_"
        "IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\",\"height\":[258,258,259,257,255,253,246,245,233,233,221,"
        "216,223,227,227,228,230,232,236,251,251,251,251,251,254,258,267,283,289,298,308,316,318,"
        "320,322,323,324,328,359,445,452,463,463,448,405,393,336,329,326,316,311,309,308,289,291,"
        "292,292,291,289,278,279,279,280,281,281,280,281,281,282,282,282,280,276,251,248,247,246,"
        "244,243,240,239,239,238,239,241,241,239,236,221,221,225,224]}")};

// TODO: add tests that do resampling as well

void create_tile() {
  // its annoying to have to get actual data but its also very boring to test with fake data
  // so we get some real data build the tests and then create the data on the fly
  // get the real tile and run the tests against it uncommenting the log infos in src/sample.cc
  // wget -q -O - http://s3.amazonaws.com/mapzen.valhalla/elevation/N40/N40W077.hgt.gz | gunzip >
  // test/data/N40W077.hgt hack the tests to run against that and turn that into something we can
  // build a tile out of grep -E '{[0-9,]+}' test/*.log | sed -e "s/.*{/{/g" | sort -n | tr '\n' ','
  // | sed -e "s/^/#include<cstdint>\n#include<unordered_map>\nstd::unordered_map<size_t,int16_t>
  // pixels {/g" -e "s/$/};/g" > test/pixels.h
  std::vector<int16_t> tile(3601 * 3601, 0);
  for (const auto& p : pixels)
    tile[p.first] = p.second;
  std::ofstream file("test/data/service/N40W077.hgt", std::ios::binary | std::ios::trunc);
  file.write(static_cast<const char*>(static_cast<void*>(tile.data())),
             sizeof(int16_t) * tile.size());
  if (!file.good()) {
    throw std::runtime_error("File stream is not good");
  }
}

void start_service(zmq::context_t& context) {
  // server
  std::thread server(
      std::bind(&http_server_t::serve,
                http_server_t(context, "ipc:///tmp/test_skadi_server",
                              "ipc:///tmp/test_skadi_proxy_upstream", "ipc:///tmp/test_skadi_results",
                              "ipc:///tmp/test_skadi_interrupt")));
  server.detach();

  // load balancer
  std::thread proxy(
      std::bind(&proxy_t::forward, proxy_t(context, "ipc:///tmp/test_skadi_proxy_upstream",
                                           "ipc:///tmp/test_skadi_proxy_out")));
  proxy.detach();

  // service worker
  boost::property_tree::ptree config;
  std::stringstream json;
  json << R"({
      "mjolnir": { "tile_dir": "test/tiles" },
      "loki": { "actions": [ "height" ],
                  "logging": { "long_request": 100.0 },
                  "service": { "proxy": "ipc:///tmp/test_skadi_proxy" },
                "service_defaults": { "minimum_reachability": 50, "radius": 0,"search_cutoff": 35000, "node_snap_tolerance": 5, "street_side_tolerance": 5, "heading_tolerance": 60} },
      "thor": { "service": { "proxy": "ipc:///tmp/test_skadi_thor_proxy" } },
      "httpd": { "service": { "loopback": "ipc:///tmp/test_skadi_results", "interrupt": "ipc:///tmp/test_skadi_interrupt" } },
      "additional_data": { "elevation": "test/data/service" },
      "service_limits": {
        "skadi": { "max_shape": 100, "min_resample": "10"},
        "auto": { "max_distance": 5000000.0, "max_locations": 20,
                  "max_matrix_distance": 400000.0, "max_matrix_locations": 50 },
        "pedestrian": { "max_distance": 250000.0, "max_locations": 50,
                        "max_matrix_distance": 200000.0, "max_matrix_locations": 50,
                        "min_transit_walking_distance": 1, "max_transit_walking_distance": 10000 },
        "isochrone": { "max_contours": 4, "max_time": 120, "max_distance": 25000, "max_locations": 1},
        "trace": { "max_best_paths": 4, "max_best_paths_shape": 100, "max_distance": 200000.0, "max_gps_accuracy": 100.0, "max_search_radius": 100, "max_shape": 16000 },
        "max_avoid_locations": 0,
        "max_reachability": 100,
        "max_radius": 200,
        "max_alternates":2
      },
      "costing_options": { "auto": {}, "pedestrian": {} }
    })";
  rapidjson::read_json(json, config);

  std::thread worker(valhalla::loki::run_service, config);
  worker.detach();
}

void test_requests() {
  // start up the service
  zmq::context_t context;
  start_service(context);

  // client makes requests and gets back responses in a batch fashion
  auto request = requests.cbegin();
  std::string request_str;
  http_client_t client(context, "ipc:///tmp/test_skadi_server",
                       [&request, &request_str]() {
                         // we dont have any more requests so bail
                         if (request == requests.cend())
                           return std::make_pair<const void*, size_t>(nullptr, 0);
                         // get the string of bytes to send formatted for http protocol
                         request_str = request->to_string();
                         ++request;
                         return std::make_pair<const void*, size_t>(request_str.c_str(),
                                                                    request_str.size());
                       },
                       [&request](const void* data, size_t size) {
                         auto response =
                             http_response_t::from_string(static_cast<const char*>(data), size);
                         if (response.body != responses[request - requests.cbegin() - 1])
                           throw std::runtime_error("Unexpected response body: " + response.body);

                         return request != requests.cend();
                       },
                       1);
  // request and receive
  client.batch();
}
} // namespace

int main(void) {
  // make this whole thing bail if it doesnt finish fast
  alarm(120);

  test::suite suite("Elevation Service");

  suite.test(TEST_CASE(create_tile));

  suite.test(TEST_CASE(test_requests));

  return suite.tear_down();
}
