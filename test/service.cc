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
    http_request_t(POST, "/plot", "{\"shape\":[{\"lat\":40.712431, \"lon\":-76.504916},{\"lat\":40.712275, \"lon\":-76.605259},{\"lat\":40.712122, \"lon\":-76.805694},{\"lat\":40.722431, \"lon\":-76.884916},{\"lat\":40.812275, \"lon\":-76.905259},{\"lat\":40.912122, \"lon\":-76.965694}]}"),
    http_request_t(POST, "/plot", "{\"encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\"}"),
    http_request_t(POST, "/plot", "{\"shape\":[{\"lat\":40.712431, \"lon\":-76.504916},{\"lat\":40.712275, \"lon\":-76.605259},{\"lat\":40.712122, \"lon\":-76.805694}]}"),
    http_request_t(GET, "/plot?json={\"shape\":[{\"lat\":40.712431,\"lon\":-76.504916},{\"lat\":40.712275,\"lon\":-76.605259},{\"lat\":40.712122,\"lon\":-76.805694},{\"lat\":40.722431,\"lon\":-76.884916},{\"lat\":40.812275,\"lon\":-76.905259},{\"lat\":40.912122,\"lon\":-76.965694}]}"),
    http_request_t(GET, "/plot?json={\"encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\"}"),
    http_request_t(POST, "/elevation", "{\"shape\":[{\"lat\":40.712431, \"lon\":-76.504916},{\"lat\":40.712275, \"lon\":-76.605259},{\"lat\":40.712122, \"lon\":-76.805694},{\"lat\":40.722431, \"lon\":-76.884916},{\"lat\":40.812275, \"lon\":-76.905259},{\"lat\":40.912122, \"lon\":-76.965694}]}"),
    http_request_t(POST, "/elevation", "{\"encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\"}"),
    http_request_t(GET, "/elevation?json={\"shape\":[{\"lat\":40.712431,\"lon\":-76.504916},{\"lat\":40.712275,\"lon\":-76.605259},{\"lat\":40.712122,\"lon\":-76.805694},{\"lat\":40.722431,\"lon\":-76.884916},{\"lat\":40.812275,\"lon\":-76.905259},{\"lat\":40.912122,\"lon\":-76.965694}]}"),
    http_request_t(GET, "/elevation?json={\"encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\"}"),
  };

  const std::vector<std::string> responses {
    std::string("{\"input_shape\":[{\"lat\":40.712433,\"lon\":-76.504913},{\"lat\":40.712276,\"lon\":-76.605263},{\"lat\":40.712124,\"lon\":-76.805695},{\"lat\":40.722431,\"lon\":-76.884918},{\"lat\":40.812275,\"lon\":-76.905258},{\"lat\":40.912121,\"lon\":-76.965691}],\"plot\":[[0,8410,25208,31945,42023,54165],[303,275,198,197,181,201]]}"),
    std::string("{\"input_encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\",\"plot\":[[0,53,142,240,320,372,421,485,779,794,938,1003,1207,1315,1354,1416,1475,1523,1587,1805,1824,1970,2243,2260,2298,2323,2424,2554,2654,2801,2926,2984,2997,3007,3022,3032,3045,3099,3374,4293,4352,4468,4531,4823,5256,5341,5824,5888,5909,5957,5987,6009,6029,6224,6254,6264,6275,6286,6297,6397,6439,6450,6461,6472,6483,6515,6539,6550,6559,6568,6588,6629,6672,6974,7022,7065,7077,7092,7102,7124,7136,7154,7176,7191,7225,7251,7281,7312,7441,7450,7525,7558],[null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,248,250,250,252,252,253,256,268,280,289,296,304,313,313,319,319,319,320,322,356,438,446,461,464,450,414,397,341,331,331,320,314,310,310,291,292,292,292,291,291,279,279,279,279,279,279,279,279,280,280,280,282,281,278,253,249,247,248,248,244,243,243,240,240,240,241,242,241,238,222,222,223,224]]}"),
    std::string("{\"input_shape\":[{\"lat\":40.712433,\"lon\":-76.504913},{\"lat\":40.712276,\"lon\":-76.605263},{\"lat\":40.712124,\"lon\":-76.805695}],\"plot\":[[0,8410,25208],[303,275,198]]}"),
    std::string("{\"input_shape\":[{\"lat\":40.712433,\"lon\":-76.504913},{\"lat\":40.712276,\"lon\":-76.605263},{\"lat\":40.712124,\"lon\":-76.805695},{\"lat\":40.722431,\"lon\":-76.884918},{\"lat\":40.812275,\"lon\":-76.905258},{\"lat\":40.912121,\"lon\":-76.965691}],\"plot\":[[0,8410,25208,31945,42023,54165],[303,275,198,197,181,201]]}"),
    std::string("{\"input_encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\",\"plot\":[[0,53,142,240,320,372,421,485,779,794,938,1003,1207,1315,1354,1416,1475,1523,1587,1805,1824,1970,2243,2260,2298,2323,2424,2554,2654,2801,2926,2984,2997,3007,3022,3032,3045,3099,3374,4293,4352,4468,4531,4823,5256,5341,5824,5888,5909,5957,5987,6009,6029,6224,6254,6264,6275,6286,6297,6397,6439,6450,6461,6472,6483,6515,6539,6550,6559,6568,6588,6629,6672,6974,7022,7065,7077,7092,7102,7124,7136,7154,7176,7191,7225,7251,7281,7312,7441,7450,7525,7558],[null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,248,250,250,252,252,253,256,268,280,289,296,304,313,313,319,319,319,320,322,356,438,446,461,464,450,414,397,341,331,331,320,314,310,310,291,292,292,292,291,291,279,279,279,279,279,279,279,279,280,280,280,282,281,278,253,249,247,248,248,244,243,243,240,240,240,241,242,241,238,222,222,223,224]]}"),
    std::string("{\"input_shape\":[{\"lat\":40.712433,\"lon\":-76.504913},{\"lat\":40.712276,\"lon\":-76.605263},{\"lat\":40.712124,\"lon\":-76.805695},{\"lat\":40.722431,\"lon\":-76.884918},{\"lat\":40.812275,\"lon\":-76.905258},{\"lat\":40.912121,\"lon\":-76.965691}],\"elevation\":[303,275,198,197,181,201]}"),
    std::string("{\"input_encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\",\"elevation\":[null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,248,250,250,252,252,253,256,268,280,289,296,304,313,313,319,319,319,320,322,356,438,446,461,464,450,414,397,341,331,331,320,314,310,310,291,292,292,292,291,291,279,279,279,279,279,279,279,279,280,280,280,282,281,278,253,249,247,248,248,244,243,243,240,240,240,241,242,241,238,222,222,223,224]}"),
    std::string("{\"input_shape\":[{\"lat\":40.712433,\"lon\":-76.504913},{\"lat\":40.712276,\"lon\":-76.605263},{\"lat\":40.712124,\"lon\":-76.805695},{\"lat\":40.722431,\"lon\":-76.884918},{\"lat\":40.812275,\"lon\":-76.905258},{\"lat\":40.912121,\"lon\":-76.965691}],\"elevation\":[303,275,198,197,181,201]}"),
    std::string("{\"input_encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\",\"elevation\":[null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,248,250,250,252,252,253,256,268,280,289,296,304,313,313,319,319,319,320,322,356,438,446,461,464,450,414,397,341,331,331,320,314,310,310,291,292,292,292,291,291,279,279,279,279,279,279,279,279,280,280,280,282,281,278,253,249,247,248,248,244,243,243,240,240,240,241,242,241,238,222,222,223,224]}")
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

