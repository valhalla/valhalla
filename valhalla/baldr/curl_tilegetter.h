#pragma once

#include <string>
#include <utility>

#include <valhalla/baldr/curler.h>
#include <valhalla/baldr/tilegetter.h>

namespace valhalla {
namespace baldr {

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
    auto tile_data = curler.get()(url, http_code, gzipped_, interrupt_);
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

  using interrupt_t = tile_getter_t::interrupt_t;

  void set_interrupt(const interrupt_t* interrupt) override {
    interrupt_ = interrupt;
  }

private:
  curler_pool_t curlers_;
  const bool gzipped_;
  const interrupt_t* interrupt_ = nullptr;
};

/**
 * @brief Build uri address to make remote call
 * @name[in] tile_url Base url address
 * @name[in] fname File to call for
 * @name[in] remote_path This parameter is used only in testing
 * @return full uri address
 */
inline std::string make_single_point_url(const std::string& tile_url,
                                         const std::string& fname,
                                         const std::string& remote_path = {}) {
  static const std::string path_pattern{"{tilePath}"};
  auto id_pos = tile_url.find(path_pattern);
  return tile_url.substr(0, id_pos) + remote_path + fname +
         tile_url.substr(id_pos + path_pattern.size());
}

} // namespace baldr
} // namespace valhalla
