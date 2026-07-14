// Byte-faithful copy of actor_t::pimpl_t + actor_t::route into a single test TU, using the REAL
// types (loki/thor/odin workers, shared GraphReader, proto Api, ParseApi, midgard::Finally) and run
// through gtest exactly like the real actor test. The real actor_t::route AVs on Windows; every
// less-faithful slice was clean. If this AVs, the crash lives entirely in this function's shape and
// we have a self-contained repro; if not, the only remaining difference is that actor.cc is a
// different TU (with identical Windows flags), which would itself be the finding.
#include "baldr/graphreader.h"
#include "loki/worker.h"
#include "midgard/util.h"
#include "odin/worker.h"
#include "test.h"
#include "thor/worker.h"
#include "worker.h" // ParseApi, Api

#include <boost/property_tree/ptree.hpp>
#include <gtest/gtest.h>

#include <functional>
#include <memory>
#include <string>

using namespace valhalla;

// verbatim actor_t::pimpl_t
struct slice_pimpl {
  slice_pimpl(const boost::property_tree::ptree& config)
      : reader(new baldr::GraphReader(config.get_child("mjolnir"))), loki_worker(config, reader),
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
  odin::odin_worker_t odin_worker;
};

// verbatim actor_t + actor_t::route
struct slice_actor {
  std::unique_ptr<slice_pimpl> pimpl;
  bool auto_cleanup;
  slice_actor(const boost::property_tree::ptree& config, bool ac)
      : pimpl(new slice_pimpl(config)), auto_cleanup(ac) {
  }
  void cleanup() {
    pimpl->cleanup();
  }
  std::string route(const std::string& request_str, const std::function<void()>* interrupt, Api* api) {
    auto scoped_cleaner = midgard::make_finally([this]() {
      if (auto_cleanup)
        cleanup();
    });
    pimpl->set_interrupts(interrupt);
    Api dummy;
    if (!api) {
      api = &dummy;
    }
    ParseApi(request_str, Options::route, *api);
    pimpl->loki_worker.route(*api);
    pimpl->thor_worker.route(*api);
    auto bytes = pimpl->odin_worker.narrate(*api);
    return bytes;
  }
  // the other 12 actor_t methods each open with the IDENTICAL make_finally funclet. defining them
  // gives the TU 13 identical cleanup funclets (candidates for linker ICF folding) like actor.cc,
  // instead of the slice's single one. never called -- just present to be folded.
#define SLICE_METHOD(name)                                                                           \
  std::string name(const std::string&, const std::function<void()>*, Api*) {                         \
    auto scoped_cleaner = midgard::make_finally([this]() {                                            \
      if (auto_cleanup)                                                                               \
        cleanup();                                                                                    \
    });                                                                                              \
    return "";                                                                                        \
  }
  SLICE_METHOD(locate)
  SLICE_METHOD(matrix)
  SLICE_METHOD(optimized_route)
  SLICE_METHOD(isochrone)
  SLICE_METHOD(trace_route)
  SLICE_METHOD(trace_attributes)
  SLICE_METHOD(height)
  SLICE_METHOD(transit_available)
  SLICE_METHOD(expansion)
  SLICE_METHOD(centroid)
  SLICE_METHOD(status)
  SLICE_METHOD(tile)
#undef SLICE_METHOD
};

TEST(Slice, NoData) {
  auto cfg = test::make_config("no_such_tiles");
  slice_actor act(cfg, true);
  const std::string request =
      R"({"locations":[{"lat":52.0,"lon":5.0},{"lat":52.1,"lon":5.1}],"costing":"auto"})";
  try {
    act.route(request, nullptr, nullptr);
  } catch (const std::exception& e) {
    std::printf("caught: %s\n", e.what());
  }
}
