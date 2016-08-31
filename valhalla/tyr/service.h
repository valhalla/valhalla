#ifndef __VALHALLA_TYR_SERVICE_H__
#define __VALHALLA_TYR_SERVICE_H__

#include <boost/property_tree/ptree.hpp>

#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>

#include <valhalla/baldr/json.h>


namespace valhalla {
  namespace tyr {

    void run_service(const boost::property_tree::ptree& config);

    class tyr_worker_t {
     public:
      tyr_worker_t(const boost::property_tree::ptree& config);
      virtual ~tyr_worker_t();
      prime_server::worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info);
      void cleanup();

     protected:

      prime_server::worker_t::result_t jsonify_error(uint64_t code, const std::string& status, const std::string& error, prime_server::http_request_t::info_t& request_info) const;

      boost::property_tree::ptree config;
      float long_request;
    };
  }
}


#endif //__VALHALLA_TYR_SERVICE_H__
