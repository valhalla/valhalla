#pragma once

#include <array>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace valhalla {
namespace baldr {

struct curler_t {
  /**
   * Constructor
   *
   * @param user_agent User-Agent header
   */
  explicit curler_t(const std::string& user_agent);

  /**
   * Fetch a url and return the bytes that we got
   *
   * @param  url                the url to fetch
   * @param  http_code          the code we got back when fetching
   * @param  gzipped            whether to request for gzip compressed data
   * @return the bytes we fetched
   */
  std::vector<char> operator()(const std::string& url, long& http_code, bool gzipped);

  /**
   * Allow only moves and forbid copies. We don't want
   * several curlers to share the same state to completely exclude
   * situations when there are 2 copies of the same curler in a pool
   */
  curler_t(curler_t&& other) = default;
  curler_t& operator=(curler_t&& other) = default;

protected:
  struct pimpl_t;
  std::shared_ptr<pimpl_t> pimpl;
};

/**
 * Represents a synchronized pool of curlers with a fixed size.
 * When all curlers in the pool are busy, the allocate() will block
 */
class curler_pool_t {
public:
  /**
   * The only way a pool is meant to be constructed
   *
   * @param pool_size   the number of curler instances in the pool
   * @param user_agent  user agent to use by curlers for HTTP requests
   */
  curler_pool_t(const size_t pool_size, const std::string& user_agent);

  /**
   * @return The size of the pool (never changes from creation)
   */
  size_t size() const {
    return size_;
  }

  /**
   * Allocates a curler instance from pool. If none are available, blocks
   *
   * @return  curler instance that is available only in current thread.
   */
  curler_t acquire();

  /**
   * Return a curler instance to the pool (should be from current pool)
   *
   * @param curler  curler instance being released
   */
  void release(curler_t&& curler);

  curler_pool_t(const curler_pool_t&) = delete;
  curler_pool_t& operator=(const curler_pool_t&) = delete;

private:
  const size_t size_;
  std::mutex curler_pool_lock_;
  std::condition_variable curler_pool_empty_cond_;
  std::vector<curler_t> curlers_;
};

/**
 * Represents a curl entry allocated from a curler_pool_t
 */
class scoped_curler_t {
public:
  explicit scoped_curler_t(curler_pool_t& owner)
      : owner_(owner), curler_(std::move(owner.acquire())) {
  }

  ~scoped_curler_t() {
    owner_.release(std::move(curler_));
  }

  curler_t& get() {
    return curler_;
  }

  scoped_curler_t(const scoped_curler_t&) = delete;
  scoped_curler_t& operator=(const scoped_curler_t&) = delete;

private:
  curler_pool_t& owner_;
  curler_t curler_;
};

} // namespace baldr
} // namespace valhalla
