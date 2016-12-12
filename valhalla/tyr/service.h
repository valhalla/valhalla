#ifndef __VALHALLA_TYR_SERVICE_H__
#define __VALHALLA_TYR_SERVICE_H__

#include <boost/property_tree/ptree.hpp>

#include <prime_server/prime_server.hpp>

namespace valhalla {
  namespace tyr {

    void run_service(const boost::property_tree::ptree& config);

    class tyr_worker_t {
     public:
      tyr_worker_t(const boost::property_tree::ptree& config);
      virtual ~tyr_worker_t();
      prime_server::worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info, const prime_server::worker_t::interrupt_function_t&);
      void cleanup();

     protected:

      boost::property_tree::ptree config;
      boost::optional<std::string> jsonp;
      float long_request;
    };
  }
}


#endif //__VALHALLA_TYR_SERVICE_H__
