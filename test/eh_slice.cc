// Vertical slice, grown to the real thing: construct an actual loki_worker_t and call its cleanup()
// from a midgard::Finally funclet during exception unwind, in a fully-linked binary (valhalla_test).
// The generic/zmq slices were clean on Windows; this isolates whether the REAL worker type + full
// libvalhalla link reproduces the AV. No tiles/tz.sqlite needed (the config points at a missing dir).
#include "loki/worker.h"
#include "midgard/util.h"
#include "test.h"

#include <boost/property_tree/ptree.hpp>

#include <cstdio>

using namespace valhalla;

struct my_error {
  const char* msg;
};

struct slice_actor {
  bool auto_cleanup;
  loki::loki_worker_t worker;
  slice_actor(bool ac, const boost::property_tree::ptree& cfg) : auto_cleanup(ac), worker(cfg) {
  }
  void cleanup() {
    std::fprintf(stderr, "[DBG] slice_actor::cleanup -> worker.cleanup\n");
    std::fflush(stderr);
    worker.cleanup();
    std::fprintf(stderr, "[DBG] slice_actor::cleanup done\n");
    std::fflush(stderr);
  }
  void route() {
    auto scoped = midgard::make_finally([this]() {
      if (auto_cleanup)
        cleanup();
    });
    throw my_error{"no edges"};
  }
};

int main() {
  auto cfg = test::make_config("no_such_tiles");
  slice_actor act(true, cfg);
  try {
    act.route();
  } catch (const my_error& e) {
    std::printf("caught: %s\n", e.msg);
  }
  std::puts("OK");
  return 0;
}
