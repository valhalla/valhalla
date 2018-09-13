#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace valhalla {
namespace baldr {

struct curler_t {
  /**
   * Constructor
   */
  curler_t();

  /**
   * Fetch a url and return the bytes that we got
   *
   * @param  url                the url to fetch
   * @param  http_code          the code we got back when fetching
   * @param  allow_compression  whether to allow and automatically handle compressed content types
   * @return the bytes we fetched
   */
  std::vector<char>
  operator()(const std::string& url, long& http_code, bool allow_compression = true);

protected:
  struct pimpl_t;
  std::shared_ptr<pimpl_t> pimpl;
};

} // namespace baldr
} // namespace valhalla