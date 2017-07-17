#include <functional>
#include <string>
#include <stdexcept>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <sstream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "baldr/json.h"
#include "midgard/logging.h"

#include "proto/directions_options.pb.h"
#include "proto/trippath.pb.h"
#include "odin/worker.h"
#include "odin/util.h"
#include "odin/directionsbuilder.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
  namespace odin {

    odin_worker_t::odin_worker_t(const boost::property_tree::ptree& config){}

    odin_worker_t::~odin_worker_t(){}

    void odin_worker_t::cleanup(){}

    std::list<TripDirections> odin_worker_t::narrate(boost::property_tree::ptree& request, std::list<TripPath>& legs) const {
      // Grab language from options and set
      auto language = request.get_optional<std::string>("directions_options.language");
      // If language is not found then set to the default language (en-US)
      if (!language || (odin::get_locales().find(*language) == odin::get_locales().end()))
        request.put<std::string>("directions_options.language", odin::DirectionsOptions::default_instance().language());

      //see if we can get some options
      valhalla::odin::DirectionsOptions directions_options;
      auto options = request.get_child_optional("directions_options");
      if(options)
        directions_options = valhalla::odin::GetDirectionsOptions(*options);

      //get some annotated directions
      std::list<TripDirections> narrated;
      try{
        for(auto& leg : legs) {
          narrated.emplace_back(odin::DirectionsBuilder().Build(directions_options, leg));
          LOG_INFO("maneuver_count::" + std::to_string(narrated.back().maneuver_size()));
        }
      }
      catch(...) {
        throw valhalla_exception_t{202};
      }
      return narrated;
    }

#ifdef HAVE_HTTP
    worker_t::result_t odin_worker_t::work(const std::list<zmq::message_t>& job, void* request_info, const std::function<void ()>& interrupt_function) {
      auto& info = *static_cast<http_request_info_t*>(request_info);
      LOG_INFO("Got Odin Request " + std::to_string(info.id));
      boost::optional<std::string> jsonp;
      try{
        //crack open the original request
        std::string request_str(static_cast<const char*>(job.front().data()), job.front().size());
        std::stringstream stream(request_str);
        boost::property_tree::ptree request;
        try{
          boost::property_tree::read_json(stream, request);
          jsonp = request.get_optional<std::string>("jsonp");
        }
        catch(...) {
          return jsonify_error({200}, info, jsonp);
        }

        // Set the interrupt function
        service_worker_t::set_interrupt(interrupt_function);

        //parse each leg
        std::list<TripPath> legs;
        for(auto leg = ++job.cbegin(); leg != job.cend(); ++leg) {
          //crack open the path
          legs.emplace_back();
          try {
            legs.back().ParseFromArray(leg->data(), static_cast<int>(leg->size()));
          }
          catch(...) {
            return jsonify_error({201}, info, jsonp);
          }
        }

        //narrate them and serialize them along
        auto narrated = narrate(request, legs);
        ACTION_TYPE action = static_cast<ACTION_TYPE>(request.get<int>("action"));
        return to_response(tyr::serializeDirections(action, request, narrated), jsonp, info);
      }
      catch(const std::exception& e) {
        return jsonify_error({299, std::string(e.what())}, info, jsonp);
      }
    }

    void run_service(const boost::property_tree::ptree& config) {
      //gets requests from odin proxy
      auto upstream_endpoint = config.get<std::string>("odin.service.proxy") + "_out";
      //or returns just location information back to the server
      auto loopback_endpoint = config.get<std::string>("httpd.service.loopback");
      auto interrupt_endpoint = config.get<std::string>("httpd.service.interrupt");

      //listen for requests
      zmq::context_t context;
      prime_server::worker_t worker(context, upstream_endpoint, "ipc:///dev/null", loopback_endpoint, interrupt_endpoint,
        std::bind(&odin_worker_t::work, odin_worker_t(config), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
      worker.work();

      //TODO: should we listen for SIGINT and terminate gracefully/exit(0)?
    }

#endif
  }
}
