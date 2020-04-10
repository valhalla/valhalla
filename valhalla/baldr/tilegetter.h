#pragma once

#include <valhalla/baldr/curler.h>

#include <string>
#include <utility>
#include <vector>

namespace valhalla {
namespace baldr {

/**
 * Synchronous interface for getting tiles.
 */
class tile_getter_t {
public:
  // TODO: Consider other error codes.
  /**
   * Operation status code.
   */
  enum class status_code_t { SUCCESS, FAILURE };

  /**
   * Raw bytes we get as a response.
   */
  using bytes_t = std::vector<char>;

  /**
   * The result of synchronous operation. Contains raw data and operations result code.
   */
  struct response_t {
    bytes_t bytes_;
    status_code_t status_ = status_code_t::FAILURE;
  };

  /**
   * Makes a synchronous request to the corresponding url and returns response_t object.
   * */
  virtual response_t get(const std::string& url) = 0;

  /**
   * Whether tiles are with .gz extension.
   */
  virtual bool gzipped() const {
    return false;
  }

  virtual ~tile_getter_t() = default;
};

/**
 * Default implementation which uses libcurl and curler_pool_t.
 */
class curl_tile_getter_t : public tile_getter_t {
public:
  /**
   * @param pool_size  the number of curler instances in the pool
   * @param user_agent  user agent to use by curlers for HTTP requests
   * @param gzipped  whether to request for gzip compressed data
   */
  curl_tile_getter_t(const size_t pool_size, const std::string& user_agent, bool gzipped)
      : curlers_(pool_size, user_agent), gzipped_(gzipped) {
  }

  using response_t = tile_getter_t::response_t;

  response_t get(const std::string& url) override {
    scoped_curler_t curler(curlers_);
    long http_code = 0;
    auto tile_data = curler.get()(url, http_code, gzipped_);
    response_t result;
    // TODO: Check other codes.
    if (http_code == 200) {
      result.bytes_ = std::move(tile_data);
      result.status_ = tile_getter_t::status_code_t::SUCCESS;
    }

    return result;
  }

  bool gzipped() const override {
    return gzipped_;
  }

private:
  curler_pool_t curlers_;
  const bool gzipped_;
};

} // namespace baldr
} // namespace valhalla
