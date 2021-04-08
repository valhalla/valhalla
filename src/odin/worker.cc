#include <cstdint>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "baldr/json.h"
#include "midgard/logging.h"

#include "midgard/util.h"
#include "odin/directionsbuilder.h"
#include "odin/util.h"
#include "odin/worker.h"
#include "tyr/serializers.h"

#include "proto/trip.pb.h"

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace odin {

odin_worker_t::odin_worker_t(const boost::property_tree::ptree&) {
}

odin_worker_t::~odin_worker_t() {
}

void odin_worker_t::cleanup() {
}

void odin_worker_t::narrate(Api& request) const {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request, "odin_worker_t::narrate");

  // get some annotated directions
  try {
    odin::DirectionsBuilder().Build(request);
  } catch (...) { throw valhalla_exception_t{202}; }
}

void odin_worker_t::status(Api&) const {
#ifdef HAVE_HTTP
  // if we are in the process of shutting down we signal that here
  // should react by draining traffic (though they are likely doing this as they are usually the ones
  // who sent us the request to shutdown)
  if (prime_server::draining() || prime_server::shutting_down()) {
    throw valhalla_exception_t{203};
  }
#endif
}

#ifdef HAVE_HTTP
prime_server::worker_t::result_t
odin_worker_t::work(const std::list<zmq::message_t>& job,
                    void* request_info,
                    const std::function<void()>& interrupt_function) {
  auto& info = *static_cast<prime_server::http_request_info_t*>(request_info);
  LOG_INFO("Got Odin Request " + std::to_string(info.id));
  Api request;
  try {
    // Set the interrupt function
    service_worker_t::set_interrupt(&interrupt_function);

    // crack open the in progress request
    bool success = request.ParseFromArray(job.front().data(), job.front().size());
    if (!success) {
      LOG_ERROR("Failed parsing pbf in Odin::Worker");
      throw valhalla_exception_t{200,
                                 boost::optional<std::string>("Failed parsing pbf in Odin::Worker")};
    }

    // its either a simple status request or its a route to narrate
    switch (request.options().action()) {
      case Options::status: {
        status(request);
        auto response = tyr::serializeStatus(request);
        return to_response(response, info, request);
      }
      default: {
        // narrate them and serialize them along
        narrate(request);
        auto response = tyr::serializeDirections(request);
        const bool as_gpx = request.options().format() == Options::gpx;
        return to_response(response, info, request, as_gpx ? worker::GPX_MIME : worker::JSON_MIME,
                           as_gpx);
      }
    }
  } catch (const std::exception& e) {
    return jsonify_error({299, std::string(e.what())}, info, request);
  }
}

void run_service(const boost::property_tree::ptree& config) {
  // gracefully shutdown when asked via SIGTERM
  prime_server::quiesce(config.get<unsigned int>("httpd.service.drain_seconds", 28),
                        config.get<unsigned int>("httpd.service.shutting_seconds", 1));

  // gets requests from odin proxy
  auto upstream_endpoint = config.get<std::string>("odin.service.proxy") + "_out";
  // or returns just location information back to the server
  auto loopback_endpoint = config.get<std::string>("httpd.service.loopback");
  auto interrupt_endpoint = config.get<std::string>("httpd.service.interrupt");

  // listen for requests
  zmq::context_t context;
  prime_server::worker_t worker(context, upstream_endpoint, "ipc:///dev/null", loopback_endpoint,
                                interrupt_endpoint,
                                std::bind(&odin_worker_t::work, odin_worker_t(config),
                                          std::placeholders::_1, std::placeholders::_2,
                                          std::placeholders::_3));
  worker.work();

  // TODO: should we listen for SIGINT and terminate gracefully/exit(0)?
}

#endif
} // namespace odin
} // namespace valhalla
