// Vertical slice reproducing the Windows unwind-funclet AV in isolation: a Finally (copied from
// midgard) runs a this-capturing cleanup during exception unwind, and that cleanup makes nested
// virtual member calls into polymorphic worker objects. Built as a normal test target so it uses
// the same compiler/linker flags as the rest of the suite. Names are generic on purpose.
//
// Trivial versions (raw lambda + one plain cleanup) do NOT crash, so this adds, incrementally, the
// shape of the real code: a virtual base cleanup, several workers, a message-list member. If this
// AVs on Windows the same way actor.exe does, it's a self-contained MSVC repro.
#include <cstdio>
#include <list>
#include <string>
#include <utility>

template <typename T> struct Finally {
  T t;
  explicit Finally(T t) : t(std::move(t)) {}
  Finally() = delete;
  Finally(Finally&&) = default;
  Finally(const Finally&) = delete;
  Finally& operator=(const Finally&) = delete;
  Finally& operator=(Finally&&) = delete;
  ~Finally() {
    t();
  }
};
template <typename T> Finally<T> make_finally(T t) {
  return Finally<T>(std::move(t));
}

struct my_error {
  std::string msg;
};

// stand-in for service_worker_t: virtual cleanup + a message-list member like the zmq one.
struct base_worker {
  std::list<std::string> messages;
  virtual ~base_worker() = default;
  virtual void cleanup() {
    std::fprintf(stderr, "[DBG] base_worker::cleanup this=%p\n", (void*)this);
    std::fflush(stderr);
  }
};
struct derived_worker : base_worker {
  void cleanup() override {
    std::fprintf(stderr, "[DBG] derived_worker::cleanup enter this=%p\n", (void*)this);
    std::fflush(stderr);
    base_worker::cleanup();
  }
};

struct slice_actor {
  bool auto_cleanup;
  derived_worker a, b, c;
  explicit slice_actor(bool ac) : auto_cleanup(ac) {}
  void cleanup() {
    std::fprintf(stderr, "[DBG] slice_actor::cleanup enter\n");
    std::fflush(stderr);
    a.cleanup();
    b.cleanup();
    c.cleanup();
    std::fprintf(stderr, "[DBG] slice_actor::cleanup done\n");
    std::fflush(stderr);
  }
  void route() {
    auto scoped = make_finally([this]() {
      if (auto_cleanup)
        cleanup();
    });
    throw my_error{"no edges"};
  }
};

int main() {
  slice_actor act(true);
  try {
    act.route();
  } catch (const my_error& e) {
    std::printf("caught: %s\n", e.msg.c_str());
  }
  std::puts("OK");
  return 0;
}
