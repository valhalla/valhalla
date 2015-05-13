#include <functional>
#include <string>
#include <stdexcept>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <sstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>
using namespace prime_server;

#include <valhalla/midgard/logging.h>

#include "odin/service.h"
#include "odin/util.h"
#include "proto/directions_options.pb.h"
#include "proto/trippath.pb.h"
#include "odin/directionsbuilder.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace {
  //TODO: throw this in the header to make it testable?
  class odin_worker_t {
   public:
    odin_worker_t(const boost::property_tree::ptree& config):config(config) {

    }
    worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info) {
      auto info = *static_cast<http_request_t::info_t*>(request_info);
      LOG_INFO("Got Odin Request " + std::to_string(info.id));
      try{
        //crack open the original request
        std::string request_str(static_cast<const char*>(job.front().data()), job.front().size());
        std::stringstream stream(request_str);
        boost::property_tree::ptree request;
        boost::property_tree::read_info(stream, request);

        //see if we can get some options
        valhalla::odin::DirectionsOptions directions_options;
        auto options = request.get_child_optional("directions_options");
        if(options)
          directions_options = valhalla::odin::GetDirectionsOptions(*options);

        //crack open the path
        odin::TripPath trip_path;
        trip_path.ParseFromArray(job.back().data(), static_cast<int>(job.back().size()));

        //get some annotated directions
        odin::DirectionsBuilder directions;
        odin::TripDirections trip_directions = directions.Build(directions_options, trip_path);

        //pass it on
        worker_t::result_t result{true};
        result.messages.emplace_back(std::move(request_str)); //the original request
        result.messages.emplace_back(trip_directions.SerializeAsString()); //the protobuf directions
        return result;
      }
      catch(const std::exception& e) {
        worker_t::result_t result{false};
        http_response_t response(400, "Bad Request", e.what());
        response.from_info(info);
        result.messages.emplace_back(response.to_string());
        return result;
      }
    }
   protected:
    boost::property_tree::ptree config;
  };
}

namespace valhalla {
  namespace odin {
    void run_service(const boost::property_tree::ptree& config) {
      //gets requests from odin proxy
      auto upstream_endpoint = config.get<std::string>("odin.service.proxy") + "_out";
      //sends them on to tyr
      auto downstream_endpoint = config.get<std::string>("tyr.service.proxy") + "_in";
      //or returns just location information back to the server
      auto loopback_endpoint = config.get<std::string>("httpd.service.loopback");

      //listen for requests
      zmq::context_t context;
      prime_server::worker_t worker(context, upstream_endpoint, downstream_endpoint, loopback_endpoint,
        std::bind(&odin_worker_t::work, odin_worker_t(config), std::placeholders::_1, std::placeholders::_2));
      worker.work();

      //TODO: should we listen for SIGINT and terminate gracefully/exit(0)?
    }
  }
}
