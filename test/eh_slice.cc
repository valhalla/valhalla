// Minimal repro attempt going the RIGHT way: NO valhalla. Generic Finally + this-capturing cleanup
// + nested virtual worker cleanups (the shape that was clean as a bare main), now wrapped in a gtest
// TEST and run through gtest exactly like the crashing actor test -- so the throw unwinds through
// gtest's frames (Test::Run -> HandleExceptionsInMethodIfSupported). Isolates whether the gtest
// wrapper is the missing variable. Links only gtest.
#include <gtest/gtest.h>

#include <cstdio>
#include <list>
#include <memory>
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
  const char* msg;
};

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
    std::fprintf(stderr, "[DBG] derived_worker::cleanup this=%p\n", (void*)this);
    std::fflush(stderr);
    base_worker::cleanup();
  }
};

struct slice_pimpl {
  derived_worker a, b, c;
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

struct slice_actor {
  std::unique_ptr<slice_pimpl> pimpl;
  bool auto_cleanup;
  explicit slice_actor(bool ac) : pimpl(new slice_pimpl()), auto_cleanup(ac) {}
  void cleanup() {
    pimpl->cleanup();
  }
  void route() {
    auto scoped = make_finally([this]() {
      if (auto_cleanup)
        cleanup();
    });
    throw my_error{"no edges"};
  }
};

TEST(Slice, NoData) {
  slice_actor act(true);
  try {
    act.route();
  } catch (const my_error& e) {
    std::printf("caught: %s\n", e.msg);
  }
}
