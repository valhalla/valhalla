// Vertical slice mirroring actor_t as faithfully as possible in one file: a unique_ptr-held pimpl
// owning several real loki_worker_t's, cleanup delegating actor->pimpl->workers, driven by a
// midgard::Finally funclet capturing `this` (the ORIGINAL pattern) that throws through it. Linked
// against full libvalhalla. Isolates whether the pimpl indirection + multiple real workers is what
// the earlier (single-worker) slices lacked to reproduce the Windows unwind AV. No tiles needed.
#include "loki/worker.h"
#include "midgard/util.h"
#include "test.h"

#include <boost/property_tree/ptree.hpp>

#include <cstdio>
#include <memory>

using namespace valhalla;

struct my_error {
  const char* msg;
};

// mirror actor_t::pimpl_t: heap-allocated, owns the workers, cleanup calls each in turn.
struct slice_pimpl {
  loki::loki_worker_t a, b, c;
  explicit slice_pimpl(const boost::property_tree::ptree& cfg) : a(cfg), b(cfg), c(cfg) {
  }
  void cleanup() {
    std::fprintf(stderr, "[DBG] slice_pimpl::cleanup enter\n");
    std::fflush(stderr);
    a.cleanup();
    b.cleanup();
    c.cleanup();
    std::fprintf(stderr, "[DBG] slice_pimpl::cleanup done\n");
    std::fflush(stderr);
  }
};

// mirror actor_t: unique_ptr pimpl, cleanup delegates through it, route arms a Finally then throws.
struct slice_actor {
  std::unique_ptr<slice_pimpl> pimpl;
  bool auto_cleanup;
  slice_actor(const boost::property_tree::ptree& cfg, bool ac)
      : pimpl(new slice_pimpl(cfg)), auto_cleanup(ac) {
  }
  void cleanup() {
    std::fprintf(stderr, "[DBG] slice_actor::cleanup -> pimpl->cleanup\n");
    std::fflush(stderr);
    pimpl->cleanup();
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
  slice_actor act(cfg, true);
  try {
    act.route();
  } catch (const my_error& e) {
    std::printf("caught: %s\n", e.msg);
  }
  std::puts("OK");
  return 0;
}
