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
  pimpl_t(const boost::property_tree::ptree& config, baldr::GraphReader& graph_reader)
      : reader(&graph_reader, [](baldr::GraphReader*) {}), loki_worker(config, reader),
        thor_worker(config, reader), odin_worker(config) {
  }
  void set_interrupts(const std::function<void()>* interrupt_function) {
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
  odin_worker_t odin_worker;
};

actor_t::actor_t(const boost::property_tree::ptree& config, bool auto_cleanup)
    : pimpl(new pimpl_t(config)), auto_cleanup(auto_cleanup) {
}
actor_t::actor_t(const boost::property_tree::ptree& config,
                 baldr::GraphReader& reader,
                 bool auto_cleanup)
    : pimpl(new pimpl_t(config, reader)), auto_cleanup(auto_cleanup) {
}

void actor_t::cleanup() {
  pimpl->cleanup();
}

std::string actor_t::act(Api& api, const std::function<void()>* interrupt) {
  if (!api.options().has_action_case())
    throw std::runtime_error("No action was specified");

  switch (api.options().action()) {
    case Options::route:
      return route("", interrupt, &api);
    case Options::locate:
      return locate("", interrupt, &api);
    case Options::sources_to_targets:
      return matrix("", interrupt, &api);
    case Options::optimized_route:
      return optimized_route("", interrupt, &api);
    case Options::isochrone:
      return isochrone("", interrupt, &api);
    case Options::trace_route:
      return trace_route("", interrupt, &api);
    case Options::trace_attributes:
      return trace_attributes("", interrupt, &api);
    case Options::height:
      return height("", interrupt, &api);
    case Options::transit_available:
      return transit_available("", interrupt, &api);
    case Options::expansion:
      return expansion("", interrupt, &api);
    case Options::centroid:
      return centroid("", interrupt, &api);
    case Options::status:
      return status("", interrupt, &api);
    default:
      throw std::runtime_error("Unknown action");
  }
}

std::string
actor_t::route(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // parse the request
  Api request;
  ParseApi(request_str, Options::route, request);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.route(request);
  // route between the locations in the graph to find the best path
  pimpl->thor_worker.route(request);
  // get some directions back from them and serialize
  auto bytes = pimpl->odin_worker.narrate(request);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  // give the caller a copy
  if (api) {
    api->Swap(&request);
  }
  return bytes;
}

std::string
actor_t::locate(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // parse the request
  Api request;
  ParseApi(request_str, Options::locate, request);
  // check the request and locate the locations in the graph
  auto json = pimpl->loki_worker.locate(request);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  // give the caller a copy
  if (api) {
    api->Swap(&request);
  }
  return json;
}

std::string
actor_t::matrix(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // parse the request
  Api request;
  ParseApi(request_str, Options::sources_to_targets, request);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.matrix(request);
  // compute the matrix
  auto json = pimpl->thor_worker.matrix(request);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  // give the caller a copy
  if (api) {
    api->Swap(&request);
  }
  return json;
}

std::string actor_t::optimized_route(const std::string& request_str,
                                     const std::function<void()>* interrupt,
                                     Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // parse the request
  Api request;
  ParseApi(request_str, Options::optimized_route, request);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.matrix(request);
  // compute compute all pairs and then the shortest path through them all
  pimpl->thor_worker.optimized_route(request);
  // get some directions back from them and serialize
  auto bytes = pimpl->odin_worker.narrate(request);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  // give the caller a copy
  if (api) {
    api->Swap(&request);
  }
  return bytes;
}

std::string
actor_t::isochrone(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // parse the request
  Api request;
  ParseApi(request_str, Options::isochrone, request);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.isochrones(request);
  // compute the isochrones
  auto json = pimpl->thor_worker.isochrones(request);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  // give the caller a copy
  if (api) {
    api->Swap(&request);
  }
  return json;
}

std::string actor_t::trace_route(const std::string& request_str,
                                 const std::function<void()>* interrupt,
                                 Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // parse the request
  Api request;
  ParseApi(request_str, Options::trace_route, request);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.trace(request);
  // route between the locations in the graph to find the best path
  pimpl->thor_worker.trace_route(request);
  // get some directions back from them
  auto bytes = pimpl->odin_worker.narrate(request);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  // give the caller a copy
  if (api) {
    api->Swap(&request);
  }
  return bytes;
}

std::string actor_t::trace_attributes(const std::string& request_str,
                                      const std::function<void()>* interrupt,
                                      Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // parse the request
  Api request;
  ParseApi(request_str, Options::trace_attributes, request);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.trace(request);
  // get the path and turn it into attribution along it
  auto json = pimpl->thor_worker.trace_attributes(request);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  // give the caller a copy
  if (api) {
    api->Swap(&request);
  }
  return json;
}

std::string
actor_t::height(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // parse the request
  Api request;
  ParseApi(request_str, Options::height, request);
  // get the height at each point
  auto json = pimpl->loki_worker.height(request);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  // give the caller a copy
  if (api) {
    api->Swap(&request);
  }
  return json;
}

std::string actor_t::transit_available(const std::string& request_str,
                                       const std::function<void()>* interrupt,
                                       Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // parse the request
  Api request;
  ParseApi(request_str, Options::transit_available, request);
  // check the request and locate the locations in the graph
  auto json = pimpl->loki_worker.transit_available(request);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  // give the caller a copy
  if (api) {
    api->Swap(&request);
  }
  return json;
}

std::string
actor_t::expansion(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // parse the request
  Api request;
  ParseApi(request_str, Options::expansion, request);
  // check the request and locate the locations in the graph
  if (request.options().expansion_action() == Options::route) {
    pimpl->loki_worker.route(request);
  } else {
    pimpl->loki_worker.isochrones(request);
  }
  // route between the locations in the graph to find the best path
  auto json = pimpl->thor_worker.expansion(request);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  // give the caller a copy
  if (api) {
    api->Swap(&request);
  }
  return json;
}

std::string
actor_t::centroid(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this temp one
  Api request;
  if (!api) {
    api = &request;
  }
  // parse the request
  ParseApi(request_str, Options::centroid, *api);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.route(*api);
  // route between the locations in the graph to find the best path
  pimpl->thor_worker.centroid(*api);
  // get some directions back from them and serialize
  auto bytes = pimpl->odin_worker.narrate(*api);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  return bytes;
}

std::string
actor_t::status(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // parse the request
  Api request;
  ParseApi(request_str, Options::status, request);
  // check lokis status
  pimpl->loki_worker.status(request);
  // check thors status
  pimpl->thor_worker.status(request);
  // check odins status
  pimpl->odin_worker.status(request);
  // get the json
  auto json = tyr::serializeStatus(request);
  // if they want you do to do the cleanup automatically
  if (auto_cleanup) {
    cleanup();
  }
  // give the caller a copy
  if (api) {
    api->Swap(&request);
  }
  return json;
}

} // namespace tyr
} // namespace valhalla
