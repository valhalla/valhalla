#include "tyr/actor.h"
#include "loki/worker.h"
#include "midgard/logging.h"
#include "odin/worker.h"
#include "thor/worker.h"
#include "tile/worker.h"
#include "tyr/serializers.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <sstream>

using namespace valhalla;
using namespace valhalla::loki;
using namespace valhalla::midgard;
using namespace valhalla::odin;
using namespace valhalla::thor;

namespace valhalla {
namespace tyr {

struct actor_t::pimpl_t {
  pimpl_t(const boost::property_tree::ptree& config)
      : reader(new baldr::GraphReader(config.get_child("mjolnir"))), loki_worker(config, reader),
        thor_worker(config, reader), odin_worker(config), tile_worker(config, reader) {
  }
  pimpl_t(const boost::property_tree::ptree& config, baldr::GraphReader& graph_reader)
      : reader(&graph_reader, [](baldr::GraphReader*) {}), loki_worker(config, reader),
        thor_worker(config, reader), odin_worker(config), tile_worker(config, reader) {
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
  tile::tile_worker_t tile_worker;
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
  if (api.options().action() == Options::no_action)
    throw valhalla_exception_t{106};

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
      throw valhalla_exception_t{106};
  }
}

std::string
actor_t::route(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  auto scoped_cleaner = make_finally([this]() {
    if (auto_cleanup)
      cleanup();
  });
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::route, *api);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.route(*api);
  // route between the locations in the graph to find the best path
  pimpl->thor_worker.route(*api);
  // get some directions back from them and serialize
  auto bytes = pimpl->odin_worker.narrate(*api);
  return bytes;
}

std::string
actor_t::locate(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  auto scoped_cleaner = make_finally([this]() {
    if (auto_cleanup)
      cleanup();
  });
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::locate, *api);
  // check the request and locate the locations in the graph
  auto json = pimpl->loki_worker.locate(*api);
  return json;
}

std::string
actor_t::matrix(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  auto scoped_cleaner = make_finally([this]() {
    if (auto_cleanup)
      cleanup();
  });
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::sources_to_targets, *api);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.matrix(*api);
  // compute the matrix
  auto bytes = pimpl->thor_worker.matrix(*api);
  return bytes;
}

std::string actor_t::optimized_route(const std::string& request_str,
                                     const std::function<void()>* interrupt,
                                     Api* api) {
  auto scoped_cleaner = make_finally([this]() {
    if (auto_cleanup)
      cleanup();
  });
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::optimized_route, *api);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.matrix(*api);
  // compute compute all pairs and then the shortest path through them all
  pimpl->thor_worker.optimized_route(*api);
  // get some directions back from them and serialize
  auto bytes = pimpl->odin_worker.narrate(*api);
  return bytes;
}

std::string
actor_t::isochrone(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  auto scoped_cleaner = make_finally([this]() {
    if (auto_cleanup)
      cleanup();
  });
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::isochrone, *api);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.isochrones(*api);
  // compute the isochrones
  auto json = pimpl->thor_worker.isochrones(*api);
  return json;
}

std::string actor_t::trace_route(const std::string& request_str,
                                 const std::function<void()>* interrupt,
                                 Api* api) {
  auto scoped_cleaner = make_finally([this]() {
    if (auto_cleanup)
      cleanup();
  });
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::trace_route, *api);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.trace(*api);
  // route between the locations in the graph to find the best path
  pimpl->thor_worker.trace_route(*api);
  // get some directions back from them
  auto bytes = pimpl->odin_worker.narrate(*api);
  return bytes;
}

std::string actor_t::trace_attributes(const std::string& request_str,
                                      const std::function<void()>* interrupt,
                                      Api* api) {
  auto scoped_cleaner = make_finally([this]() {
    if (auto_cleanup)
      cleanup();
  });
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::trace_attributes, *api);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.trace(*api);
  // get the path and turn it into attribution along it
  auto json = pimpl->thor_worker.trace_attributes(*api);
  return json;
}

std::string
actor_t::height(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  auto scoped_cleaner = make_finally([this]() {
    if (auto_cleanup)
      cleanup();
  });
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::height, *api);
  // get the height at each point
  auto json = pimpl->loki_worker.height(*api);
  return json;
}

std::string actor_t::transit_available(const std::string& request_str,
                                       const std::function<void()>* interrupt,
                                       Api* api) {
  auto scoped_cleaner = make_finally([this]() {
    if (auto_cleanup)
      cleanup();
  });
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::transit_available, *api);
  // check the request and locate the locations in the graph
  auto json = pimpl->loki_worker.transit_available(*api);
  return json;
}

std::string
actor_t::expansion(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  auto scoped_cleaner = make_finally([this]() {
    if (auto_cleanup)
      cleanup();
  });
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::expansion, *api);
  // check the request and locate the locations in the graph
  if (api->options().expansion_action() == Options::route) {
    pimpl->loki_worker.route(*api);
  } else if (api->options().expansion_action() == Options::isochrone) {
    pimpl->loki_worker.isochrones(*api);
  } else {
    pimpl->loki_worker.matrix(*api);
  }
  // route between the locations in the graph to find the best path
  auto json = pimpl->thor_worker.expansion(*api);
  return json;
}

std::string
actor_t::centroid(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  auto scoped_cleaner = make_finally([this]() {
    if (auto_cleanup)
      cleanup();
  });
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::centroid, *api);
  // check the request and locate the locations in the graph
  pimpl->loki_worker.route(*api);
  // route between the locations in the graph to find the best path
  pimpl->thor_worker.centroid(*api);
  // get some directions back from them and serialize
  auto bytes = pimpl->odin_worker.narrate(*api);
  return bytes;
}

std::string
actor_t::status(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
  auto scoped_cleaner = make_finally([this]() {
    if (auto_cleanup)
      cleanup();
  });
  // set the interrupts
  pimpl->set_interrupts(interrupt);
  // if the caller doesn't want a copy we'll use this dummy
  Api dummy;
  if (!api) {
    api = &dummy;
  }
  // parse the request
  ParseApi(request_str, Options::status, *api);
  // check lokis status
  pimpl->loki_worker.status(*api);
  // check thors status
  pimpl->thor_worker.status(*api);
  // check odins status
  pimpl->odin_worker.status(*api);
  // get the json
  auto json = tyr::serializeStatus(*api);
  return json;
}

std::string actor_t::tile(const std::string& request_str) {
  // Parse z/x/y from request string
  // Expected format: {"z": 10, "x": 123, "y": 456}
  uint32_t z = 0, x = 0, y = 0;

  try {
    std::stringstream ss(request_str);
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(ss, pt);

    z = pt.get<uint32_t>("z");
    x = pt.get<uint32_t>("x");
    y = pt.get<uint32_t>("y");
  } catch (const std::exception& e) {
    throw valhalla_exception_t{400, "Invalid tile request: " + std::string(e.what())};
  }

  // Delegate to tile worker
  return pimpl->tile_worker.render_tile(z, x, y);
}

} // namespace tyr
} // namespace valhalla
