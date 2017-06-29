#ifndef __VALHALLA_SKADI_SERVICE_H__
#define __VALHALLA_SKADI_SERVICE_H__

#include <cstdint>
#include <list>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>

#include <valhalla/baldr/json.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/skadi/sample.h>
#include <valhalla/worker.h>

namespace valhalla {
  namespace skadi {

#ifdef HAVE_HTTP
    void run_service(const boost::property_tree::ptree& config);
#endif

    class skadi_worker_t : public service_worker_t{
     public:
      skadi_worker_t(const boost::property_tree::ptree& config);
      virtual ~skadi_worker_t();
#ifdef HAVE_HTTP
      virtual prime_server::worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info, const prime_server::worker_t::interrupt_function_t& interrupt) override;
#endif
      virtual void cleanup() override;

      baldr::json::MapPtr elevation(const boost::property_tree::ptree& request);

     protected:

      void init_request(const boost::property_tree::ptree& request);

      std::list<midgard::PointLL> shape;
      boost::optional<std::string> encoded_polyline;
      bool range;
      skadi::sample sample;
      size_t max_shape;
      float min_resample;
      float long_request;
      bool healthcheck;
  };
 }
}

#endif //__VALHALLA_SKADI_SERVICE_H__
