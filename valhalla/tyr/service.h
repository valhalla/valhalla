#ifndef __VALHALLA_TYR_SERVICE_H__
#define __VALHALLA_TYR_SERVICE_H__

#include <cstdint>
#include <list>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>

#include <valhalla/service.h>
#include <valhalla/baldr/json.h>
#include <valhalla/proto/tripdirections.pb.h>
#include <valhalla/proto/directions_options.pb.h>

namespace valhalla {
  namespace tyr {

#ifdef HAVE_HTTP
    void run_service(const boost::property_tree::ptree& config);
#endif

    class tyr_worker_t : public service::service_worker_t{
     public:
      tyr_worker_t(const boost::property_tree::ptree& config);
      virtual ~tyr_worker_t();
#ifdef HAVE_HTTP
      virtual prime_server::worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info, const prime_server::worker_t::interrupt_function_t& interrupt) override;
#endif
      virtual void cleanup() override;

      baldr::json::MapPtr serialize(service::ACTION_TYPE action, const boost::property_tree::ptree& request, const std::list<odin::TripDirections>& directions_legs) const;

     protected:

      boost::property_tree::ptree config;
      boost::optional<std::string> jsonp;
      float long_request;
      bool healthcheck;
    };
  }
}


#endif //__VALHALLA_TYR_SERVICE_H__
