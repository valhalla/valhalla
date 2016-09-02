#ifndef __VALHALLA_ODIN_SERVICE_H__
#define __VALHALLA_ODIN_SERVICE_H__

#include <boost/property_tree/ptree.hpp>


namespace valhalla {
  namespace odin {

    void run_service(const boost::property_tree::ptree& config);

    class odin_worker_t {
     public:
      odin_worker_t(const boost::property_tree::ptree& config);
      virtual ~odin_worker_t();
      prime_server::worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info);
      void cleanup();

     protected:

      prime_server::worker_t::result_t jsonify_error(uint64_t code, const std::string& status, const std::string& error, prime_server::http_request_t::info_t& request_info) const;

      boost::property_tree::ptree config;
    };
  }
}

#endif //__VALHALLA_ODIN_SERVICE_H__
