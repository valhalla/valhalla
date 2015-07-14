#include "test.h"

#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>
#include <boost/property_tree/ptree.hpp>
#include <thread>
#include <unistd.h>

#include "skadi/service.h"

using namespace valhalla;
using namespace prime_server;

namespace {
  const std::vector<http_request_t> requests {
    http_request_t(POST, "/elevation", "{\"shape\":[{\"lat\":40.712431, \"lon\":-76.504916},{\"lat\":40.712275, \"lon\":-76.605259},{\"lat\":40.712122, \"lon\":-76.805694},{\"lat\":40.722431, \"lon\":-76.884916},{\"lat\":40.812275, \"lon\":-76.905259},{\"lat\":40.912122, \"lon\":-76.965694}]}"),
    http_request_t(POST, "/elevation", "{\"encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\"}"),
    http_request_t(POST, "/elevation", "{\"shape\":[{\"lat\":40.712431, \"lon\":-76.504916},{\"lat\":40.712275, \"lon\":-76.605259},{\"lat\":40.712122, \"lon\":-76.805694}]}")
  };

  const std::vector<std::string> responses {
    std::string("{\"input_shape\":[{\"lat\":40.712433,\"lon\":-76.504913},{\"lat\":40.712276,\"lon\":-76.605263},{\"lat\":40.712124,\"lon\":-76.805695},{\"lat\":40.722431,\"lon\":-76.884918},{\"lat\":40.812275,\"lon\":-76.905258},{\"lat\":40.912121,\"lon\":-76.965691}],\"elevation\":[{\"height\":303,\"range\":0},{\"height\":275,\"range\":8410},{\"height\":198,\"range\":16798},{\"height\":197,\"range\":6736},{\"height\":181,\"range\":10079},{\"height\":201,\"range\":12141}]}"),
    std::string("{\"input_encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\",\"elevation\":[{\"height\":null,\"range\":0},{\"height\":null,\"range\":53},{\"height\":null,\"range\":89},{\"height\":null,\"range\":98},{\"height\":null,\"range\":80},{\"height\":null,\"range\":51},{\"height\":null,\"range\":49},{\"height\":null,\"range\":65},{\"height\":null,\"range\":294},{\"height\":null,\"range\":14},{\"height\":null,\"range\":145},{\"height\":null,\"range\":64},{\"height\":null,\"range\":205},{\"height\":null,\"range\":108},{\"height\":null,\"range\":39},{\"height\":null,\"range\":61},{\"height\":null,\"range\":59},{\"height\":null,\"range\":48},{\"height\":null,\"range\":64},{\"height\":248,\"range\":218},{\"height\":250,\"range\":18},{\"height\":250,\"range\":146},{\"height\":252,\"range\":274},{\"height\":252,\"range\":16},{\"height\":253,\"range\":38},{\"height\":256,\"range\":25},{\"height\":268,\"range\":101},{\"height\":280,\"range\":130},{\"height\":289,\"range\":101},{\"height\":296,\"range\":147},{\"height\":304,\"range\":124},{\"height\":313,\"range\":59},{\"height\":313,\"range\":13},{\"height\":319,\"range\":9},{\"height\":319,\"range\":15},{\"height\":319,\"range\":10},{\"height\":320,\"range\":13},{\"height\":322,\"range\":54},{\"height\":356,\"range\":275},{\"height\":438,\"range\":919},{\"height\":446,\"range\":59},{\"height\":461,\"range\":116},{\"height\":464,\"range\":63},{\"height\":450,\"range\":292},{\"height\":414,\"range\":433},{\"height\":397,\"range\":84},{\"height\":341,\"range\":484},{\"height\":331,\"range\":63},{\"height\":331,\"range\":21},{\"height\":320,\"range\":48},{\"height\":314,\"range\":30},{\"height\":310,\"range\":22},{\"height\":310,\"range\":20},{\"height\":291,\"range\":195},{\"height\":292,\"range\":30},{\"height\":292,\"range\":11},{\"height\":292,\"range\":11},{\"height\":291,\"range\":10},{\"height\":291,\"range\":12},{\"height\":279,\"range\":100},{\"height\":279,\"range\":42},{\"height\":279,\"range\":11},{\"height\":279,\"range\":11},{\"height\":279,\"range\":12},{\"height\":279,\"range\":11},{\"height\":279,\"range\":32},{\"height\":279,\"range\":24},{\"height\":280,\"range\":12},{\"height\":280,\"range\":8},{\"height\":280,\"range\":9},{\"height\":282,\"range\":20},{\"height\":281,\"range\":41},{\"height\":278,\"range\":43},{\"height\":253,\"range\":302},{\"height\":249,\"range\":48},{\"height\":247,\"range\":44},{\"height\":248,\"range\":11},{\"height\":248,\"range\":15},{\"height\":244,\"range\":10},{\"height\":243,\"range\":22},{\"height\":243,\"range\":12},{\"height\":240,\"range\":18},{\"height\":240,\"range\":21},{\"height\":240,\"range\":15},{\"height\":241,\"range\":34},{\"height\":242,\"range\":26},{\"height\":241,\"range\":30},{\"height\":238,\"range\":31},{\"height\":222,\"range\":129},{\"height\":222,\"range\":9},{\"height\":223,\"range\":75},{\"height\":224,\"range\":33}]}"),
    std::string("{\"input_shape\":[{\"lat\":40.712433,\"lon\":-76.504913},{\"lat\":40.712276,\"lon\":-76.605263},{\"lat\":40.712124,\"lon\":-76.805695}],\"elevation\":[{\"height\":303,\"range\":0},{\"height\":275,\"range\":8410},{\"height\":198,\"range\":16798}]}")
  };


  void start_service(zmq::context_t& context) {
    //server
    std::thread server(std::bind(&http_server_t::serve,
      http_server_t(context, "ipc://test_skadi_server", "ipc://test_skadi_proxy_upstream", "ipc://test_skadi_results")));
    server.detach();

    //load balancer
    std::thread proxy(std::bind(&proxy_t::forward,
      proxy_t(context, "ipc://test_skadi_proxy_upstream", "ipc://test_skadi_proxy_out")));
    proxy.detach();

    //service worker
    boost::property_tree::ptree config;
    config.add("skadi.service.proxy", "ipc://test_skadi_proxy");
    config.add("httpd.service.loopback", "ipc://test_skadi_results");
    config.add("additional_data.elevation", "test/data/appalachian.vrt");

    std::thread worker(valhalla::skadi::run_service, config);
    worker.detach();
  }

  void test_requests() {
    //start up the service
    zmq::context_t context;
    start_service(context);

    //client makes requests and gets back responses in a batch fashion
    auto request = requests.cbegin();
    std::string request_str;
    http_client_t client(context, "ipc://test_skadi_server",
      [&request, &request_str]() {
        //we dont have any more requests so bail
        if(request == requests.cend())
          return std::make_pair<const void*, size_t>(nullptr, 0);
        //get the string of bytes to send formatted for http protocol
        request_str = request->to_string();
        ++request;
        return std::make_pair<const void*, size_t>(request_str.c_str(), request_str.size());
      },
      [&request](const void* data, size_t size) {
        auto response = http_response_t::from_string(static_cast<const char*>(data), size);
        if (response.body != responses[request-requests.cbegin()-1])
          throw std::runtime_error("Unexpected response body");

        return request != requests.cend();
      }, 1
    );
    //request and receive
    client.batch();
  }
}

int main(void) {
  //make this whole thing bail if it doesnt finish fast
  alarm(30);

  test::suite suite("Elevation Service");

  suite.test(TEST_CASE(test_requests));

  return suite.tear_down();
}

