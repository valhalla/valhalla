#ifndef __VALHALLA_SKADI_SERVICE_H__
#define __VALHALLA_SKADI_SERVICE_H__

#include <vector>

#include <boost/property_tree/ptree.hpp>

#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>

#include <valhalla/baldr/location.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/json.h>
#include "skadi/sample.h"


namespace valhalla {
  namespace skadi {

    void run_service(const boost::property_tree::ptree& config);

    class skadi_worker_t {
     public:
      enum ACTION_TYPE {HEIGHT = 0};
      skadi_worker_t(const boost::property_tree::ptree& config);
      virtual ~skadi_worker_t();
      prime_server::worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info);
      void cleanup();

     protected:

      prime_server::worker_t::result_t jsonify_error(uint64_t code, const std::string& status, const std::string& error, prime_server::http_request_t::info_t& request_info) const;
      void init_request(const ACTION_TYPE& action, const boost::property_tree::ptree& request);

      prime_server::worker_t::result_t elevation(const boost::property_tree::ptree& request, prime_server::http_request_t::info_t& request_info);

      std::list<midgard::PointLL> shape;
      boost::optional<std::string> encoded_polyline;
      bool range;
      skadi::sample sample;
      size_t max_shape;
      float min_resample;
      float long_request;
  };
 }
}

#endif //__VALHALLA_SKADI_SERVICE_H__
