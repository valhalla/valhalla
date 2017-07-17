#ifndef __VALHALLA_SKADI_SERVICE_H__
#define __VALHALLA_SKADI_SERVICE_H__

#include <cstdint>
#include <list>
#include <string>
#include <boost/property_tree/ptree.hpp>

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
      virtual worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info, const std::function<void ()>& interrupt) override;
#endif
      virtual void cleanup() override;

      baldr::json::MapPtr height(rapidjson::Document& request);

     protected:

      void init_request(rapidjson::Document& request);

      std::vector<midgard::PointLL> shape;
      boost::optional<std::string> encoded_polyline;
      bool range;
      skadi::sample sample;
      size_t max_shape;
      float min_resample;
      float long_request;
      bool healthcheck;
      std::string action_str;
  };
 }
}

#endif //__VALHALLA_SKADI_SERVICE_H__
