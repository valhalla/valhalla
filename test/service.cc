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
    http_request_t(POST, "/profile", "{\"shape\":[{\"lat\":40.712431, \"lon\":-76.504916},{\"lat\":40.712275, \"lon\":-76.605259},{\"lat\":40.712122, \"lon\":-76.805694},{\"lat\":40.722431, \"lon\":-76.884916},{\"lat\":40.812275, \"lon\":-76.905259},{\"lat\":40.912122, \"lon\":-76.965694}]}"),
    http_request_t(POST, "/profile", "{\"encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\"}"),
    http_request_t(POST, "/profile", "{\"shape\":[{\"lat\":40.712431, \"lon\":-76.504916},{\"lat\":40.712275, \"lon\":-76.605259},{\"lat\":40.712122, \"lon\":-76.805694}]}"),
    http_request_t(GET, "/profile?json={\"shape\":[{\"lat\":40.712431,\"lon\":-76.504916},{\"lat\":40.712275,\"lon\":-76.605259},{\"lat\":40.712122,\"lon\":-76.805694},{\"lat\":40.722431,\"lon\":-76.884916},{\"lat\":40.812275,\"lon\":-76.905259},{\"lat\":40.912122,\"lon\":-76.965694}]}"),
    http_request_t(GET, "/profile?json={\"encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\"}"),
    http_request_t(POST, "/elevation", "{\"shape\":[{\"lat\":40.712431, \"lon\":-76.504916},{\"lat\":40.712275, \"lon\":-76.605259},{\"lat\":40.712122, \"lon\":-76.805694},{\"lat\":40.722431, \"lon\":-76.884916},{\"lat\":40.812275, \"lon\":-76.905259},{\"lat\":40.912122, \"lon\":-76.965694}]}"),
    http_request_t(POST, "/elevation", "{\"encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\"}"),
    http_request_t(GET, "/elevation?json={\"shape\":[{\"lat\":40.712431,\"lon\":-76.504916},{\"lat\":40.712275,\"lon\":-76.605259},{\"lat\":40.712122,\"lon\":-76.805694},{\"lat\":40.722431,\"lon\":-76.884916},{\"lat\":40.812275,\"lon\":-76.905259},{\"lat\":40.912122,\"lon\":-76.965694}]}"),
    http_request_t(GET, "/elevation?json={\"encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\"}"),
  };

  const std::vector<std::string> responses {
    std::string("{\"input_shape\":[{\"lat\":40.712433,\"lon\":-76.504913},{\"lat\":40.712276,\"lon\":-76.605263},{\"lat\":40.712124,\"lon\":-76.805695},{\"lat\":40.722431,\"lon\":-76.884918},{\"lat\":40.812275,\"lon\":-76.905258},{\"lat\":40.912121,\"lon\":-76.965691}],\"profile\":[[0,303],[8467,275],[25380,198],[32162,197],[42309,181],[54533,201]]}"),
    std::string("{\"input_encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\",\"profile\":[[0,null],[54,null],[143,null],[242,null],[323,null],[374,null],[424,null],[489,null],[785,null],[799,null],[945,null],[1010,null],[1215,null],[1324,null],[1364,null],[1426,null],[1485,null],[1533,null],[1598,null],[1818,248],[1836,250],[1983,250],[2259,252],[2275,252],[2314,253],[2339,256],[2441,268],[2571,280],[2672,289],[2821,296],[2946,304],[3005,313],[3018,313],[3027,319],[3042,319],[3053,319],[3066,320],[3121,322],[3397,356],[4322,438],[4382,446],[4499,461],[4562,464],[4856,450],[5292,414],[5377,397],[5864,341],[5928,331],[5950,331],[5998,320],[6028,314],[6050,310],[6070,310],[6267,291],[6296,292],[6307,292],[6318,292],[6329,291],[6341,291],[6441,279],[6483,279],[6494,279],[6505,279],[6517,279],[6528,279],[6559,279],[6583,279],[6595,280],[6603,280],[6613,280],[6633,282],[6675,281],[6718,278],[7022,253],[7070,249],[7114,247],[7125,248],[7140,248],[7150,244],[7173,243],[7185,243],[7203,240],[7225,240],[7240,240],[7275,241],[7300,242],[7331,241],[7362,238],[7492,222],[7500,222],[7576,223],[7610,224]]}"),
    std::string("{\"input_shape\":[{\"lat\":40.712433,\"lon\":-76.504913},{\"lat\":40.712276,\"lon\":-76.605263},{\"lat\":40.712124,\"lon\":-76.805695}],\"profile\":[[0,303],[8467,275],[25380,198]]}"),
    std::string("{\"input_shape\":[{\"lat\":40.712433,\"lon\":-76.504913},{\"lat\":40.712276,\"lon\":-76.605263},{\"lat\":40.712124,\"lon\":-76.805695},{\"lat\":40.722431,\"lon\":-76.884918},{\"lat\":40.812275,\"lon\":-76.905258},{\"lat\":40.912121,\"lon\":-76.965691}],\"profile\":[[0,303],[8467,275],[25380,198],[32162,197],[42309,181],[54533,201]]}"),
    std::string("{\"input_encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\",\"profile\":[[0,null],[54,null],[143,null],[242,null],[323,null],[374,null],[424,null],[489,null],[785,null],[799,null],[945,null],[1010,null],[1215,null],[1324,null],[1364,null],[1426,null],[1485,null],[1533,null],[1598,null],[1818,248],[1836,250],[1983,250],[2259,252],[2275,252],[2314,253],[2339,256],[2441,268],[2571,280],[2672,289],[2821,296],[2946,304],[3005,313],[3018,313],[3027,319],[3042,319],[3053,319],[3066,320],[3121,322],[3397,356],[4322,438],[4382,446],[4499,461],[4562,464],[4856,450],[5292,414],[5377,397],[5864,341],[5928,331],[5950,331],[5998,320],[6028,314],[6050,310],[6070,310],[6267,291],[6296,292],[6307,292],[6318,292],[6329,291],[6341,291],[6441,279],[6483,279],[6494,279],[6505,279],[6517,279],[6528,279],[6559,279],[6583,279],[6595,280],[6603,280],[6613,280],[6633,282],[6675,281],[6718,278],[7022,253],[7070,249],[7114,247],[7125,248],[7140,248],[7150,244],[7173,243],[7185,243],[7203,240],[7225,240],[7240,240],[7275,241],[7300,242],[7331,241],[7362,238],[7492,222],[7500,222],[7576,223],[7610,224]]}"),
    std::string("{\"input_shape\":[{\"lat\":40.712433,\"lon\":-76.504913},{\"lat\":40.712276,\"lon\":-76.605263},{\"lat\":40.712124,\"lon\":-76.805695},{\"lat\":40.722431,\"lon\":-76.884918},{\"lat\":40.812275,\"lon\":-76.905258},{\"lat\":40.912121,\"lon\":-76.965691}],\"profile\":[[0,303],[8467,275],[25380,198],[32162,197],[42309,181],[54533,201]]}"),
    std::string("{\"input_encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\",\"profile\":[[0,null],[54,null],[143,null],[242,null],[323,null],[374,null],[424,null],[489,null],[785,null],[799,null],[945,null],[1010,null],[1215,null],[1324,null],[1364,null],[1426,null],[1485,null],[1533,null],[1598,null],[1818,248],[1836,250],[1983,250],[2259,252],[2275,252],[2314,253],[2339,256],[2441,268],[2571,280],[2672,289],[2821,296],[2946,304],[3005,313],[3018,313],[3027,319],[3042,319],[3053,319],[3066,320],[3121,322],[3397,356],[4322,438],[4382,446],[4499,461],[4562,464],[4856,450],[5292,414],[5377,397],[5864,341],[5928,331],[5950,331],[5998,320],[6028,314],[6050,310],[6070,310],[6267,291],[6296,292],[6307,292],[6318,292],[6329,291],[6341,291],[6441,279],[6483,279],[6494,279],[6505,279],[6517,279],[6528,279],[6559,279],[6583,279],[6595,280],[6603,280],[6613,280],[6633,282],[6675,281],[6718,278],[7022,253],[7070,249],[7114,247],[7125,248],[7140,248],[7150,244],[7173,243],[7185,243],[7203,240],[7225,240],[7240,240],[7275,241],[7300,242],[7331,241],[7362,238],[7492,222],[7500,222],[7576,223],[7610,224]]}"),
    std::string("{\"input_shape\":[{\"lat\":40.712433,\"lon\":-76.504913},{\"lat\":40.712276,\"lon\":-76.605263},{\"lat\":40.712124,\"lon\":-76.805695},{\"lat\":40.722431,\"lon\":-76.884918},{\"lat\":40.812275,\"lon\":-76.905258},{\"lat\":40.912121,\"lon\":-76.965691}],\"profile\":[[0,303],[8467,275],[25380,198],[32162,197],[42309,181],[54533,201]]}"),
    std::string("{\"input_encoded_polyline\":\"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\\\ypAfDec@l@mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_IzLaMzKuOrFqfAbPwCl@_h@fN}OnI\",\"profile\":[[0,null],[54,null],[143,null],[242,null],[323,null],[374,null],[424,null],[489,null],[785,null],[799,null],[945,null],[1010,null],[1215,null],[1324,null],[1364,null],[1426,null],[1485,null],[1533,null],[1598,null],[1818,248],[1836,250],[1983,250],[2259,252],[2275,252],[2314,253],[2339,256],[2441,268],[2571,280],[2672,289],[2821,296],[2946,304],[3005,313],[3018,313],[3027,319],[3042,319],[3053,319],[3066,320],[3121,322],[3397,356],[4322,438],[4382,446],[4499,461],[4562,464],[4856,450],[5292,414],[5377,397],[5864,341],[5928,331],[5950,331],[5998,320],[6028,314],[6050,310],[6070,310],[6267,291],[6296,292],[6307,292],[6318,292],[6329,291],[6341,291],[6441,279],[6483,279],[6494,279],[6505,279],[6517,279],[6528,279],[6559,279],[6583,279],[6595,280],[6603,280],[6613,280],[6633,282],[6675,281],[6718,278],[7022,253],[7070,249],[7114,247],[7125,248],[7140,248],[7150,244],[7173,243],[7185,243],[7203,240],[7225,240],[7240,240],[7275,241],[7300,242],[7331,241],[7362,238],[7492,222],[7500,222],[7576,223],[7610,224]]}")
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

