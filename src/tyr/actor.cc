#include "tyr/actor.h"
#include "baldr/rapidjson_utils.h"
#include "loki/worker.h"
#include "odin/worker.h"
#include "thor/worker.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::loki;
using namespace valhalla::thor;
using namespace valhalla::odin;

namespace valhalla {
namespace tyr {

struct actor_t::pimpl_t {
  pimpl_t(const boost::property_tree::ptree& config)
      : reader(new baldr::GraphReader(config.get_child("mjolnir"))), loki_worker(config, reader),
        thor_worker(config, reader), odin_worker(config) {
  }
  void set_interrupts(const std::function<void()>& interrupt_function) {
    loki_worker.set_interrupt(interrupt_function);
    thor_worker.set_interrupt(interrupt_function);
    odin_worker.set_interrupt(interrupt_function);
  }
  void cleanup() {
    loki_worker.cleanup();
    thor_worker.cleanup();
    odin_worker.cleanup();
  }
  std::shared_ptr<baldr::GraphReader> reader;
  loki::loki_worker_t loki_worker;
  thor::thor_worker_t thor_worker;
  odin::odin_worker_t odin_worker;
};

actor_t::actor_t(const boost::property_tree::ptree& config, bool auto_cleanup)
    : pimpl(new pimpl_t(config)), auto_cleanup(auto_cleanup) {
}

void actor_t::cleanup() {
  pimpl->cleanup();
}

std::string actor_t::route(const std::string& request_str, const std::function<void()>& interrupt) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // parse the request
  valhalla_request_t request;
  request.parse(request_str, odin::DirectionsOptions::route);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.route(request);
  auto legs = pimpl->thor_worker.route(request);
  // get some directions back from them
  auto directions = pimpl->odin_worker.narrate(request, legs);
  // serialize them out to json string
  auto bytes = tyr::serializeDirections(request, legs, directions);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  return bytes;
}

std::string actor_t::locate(const std::string& request_str, const std::function<void()>& interrupt) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // parse the request
  valhalla_request_t request;
  request.parse(request_str, odin::DirectionsOptions::locate);
  // check the request and locate the locations in the graph
  auto json = pimpl->loki_worker.locate(request);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  return json;
}

std::string actor_t::matrix(const std::string& request_str, const std::function<void()>& interrupt) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // parse the request
  valhalla_request_t request;
  request.parse(request_str, odin::DirectionsOptions::sources_to_targets);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.matrix(request);
  // compute the matrix
  auto json = pimpl->thor_worker.matrix(request);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  return json;
}

std::string actor_t::optimized_route(const std::string& request_str,
                                     const std::function<void()>& interrupt) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // parse the request
  valhalla_request_t request;
  request.parse(request_str, odin::DirectionsOptions::optimized_route);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.matrix(request);
  // compute compute all pairs and then the shortest path through them all
  auto legs = pimpl->thor_worker.optimized_route(request);
  // get some directions back from them
  auto directions = pimpl->odin_worker.narrate(request, legs);
  // serialize them out to json string
  auto bytes = tyr::serializeDirections(request, legs, directions);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  return bytes;
}

std::string actor_t::isochrone(const std::string& request_str,
                               const std::function<void()>& interrupt) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // parse the request
  valhalla_request_t request;
  request.parse(request_str, odin::DirectionsOptions::isochrone);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.isochrones(request);
  // compute the isochrones
  auto json = pimpl->thor_worker.isochrones(request);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  return json;
}

std::string actor_t::trace_route(const std::string& request_str,
                                 const std::function<void()>& interrupt) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // parse the request
  valhalla_request_t request;
  request.parse(request_str, odin::DirectionsOptions::trace_route);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.trace(request);
  // route between the locations in the graph to find the best path
  std::list<TripPath> legs{pimpl->thor_worker.trace_route(request)};
  // get some directions back from them
  auto directions = pimpl->odin_worker.narrate(request, legs);
  // serialize them out to json string
  auto bytes = tyr::serializeDirections(request, legs, directions);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  return bytes;
}

std::string actor_t::trace_attributes(const std::string& request_str,
                                      const std::function<void()>& interrupt) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // parse the request
  valhalla_request_t request;
  request.parse(request_str, odin::DirectionsOptions::trace_attributes);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.trace(request);
  // get the path and turn it into attribution along it
  auto json = pimpl->thor_worker.trace_attributes(request);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  return json;
}

std::string actor_t::height(const std::string& request_str, const std::function<void()>& interrupt) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // parse the request
  valhalla_request_t request;
  request.parse(request_str, odin::DirectionsOptions::height);
  // get the height at each point
  auto json = pimpl->loki_worker.height(request);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  return json;
}

std::string actor_t::transit_available(const std::string& request_str,
                                       const std::function<void()>& interrupt) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // parse the request
  valhalla_request_t request;
  request.parse(request_str, odin::DirectionsOptions::transit_available);
  // check the request and locate the locations in the graph
  auto json = pimpl->loki_worker.transit_available(request);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  return json;
}

} // namespace tyr
} // namespace valhalla
