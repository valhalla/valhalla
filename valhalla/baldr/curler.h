#pragma once

#include <valhalla/baldr/tilegetter.h>

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
   * @param user_pw  the "user:pwd" for HTTP basic auth
   */
  explicit curler_t(const std::string& user_agent, const std::string& user_pw);

  using interrupt_t = tile_getter_t::interrupt_t;
  using HEAD_response_t = tile_getter_t::HEAD_response_t;
  using GET_response_t = tile_getter_t::GET_response_t;
  using header_mask_t = tile_getter_t::header_mask_t;

  /**
   * Fetch a url and return the bytes that we got
   *
   * @param  url                the url to fetch
   * @param  gzipped            whether to request for gzip compressed data
   * @param  interrupt          throws if request should be interrupted
   * @param  range_offset       the HTTP Range start offset
   * @param  range_size       the HTTP Range size
   * @return the bytes we fetched
   */
  GET_response_t get(const std::string& url,
                     bool gzipped,
                     const interrupt_t* interrupt,
                     const uint64_t range_offset,
                     const uint64_t range_size) const;
  /**
   * Performs a HEAD request.
   *
   * @param url         The URL to query
   * @param http_code   What we got back from the server
   * @param header_mask Which response headers should be recorded
   */
  HEAD_response_t head(const std::string& url, header_mask_t header_mask);

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
  curler_pool_t(const size_t pool_size, const std::string& user_agent, const std::string& user_pw);

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
  explicit scoped_curler_t(curler_pool_t& owner) : owner_(owner), curler_(owner.acquire()) {
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
