#pragma once

#include <functional>
#include <string>
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

  /**
   * A callback which is called to check if the request should be interrupted.
   */
  using interrupt_t = std::function<void()>;

  /**
   * Allows users to interrupt downloading requests.
   * Returns true if the request should be interrupted.
   */
  virtual void set_interrupt(const interrupt_t*){};

  virtual ~tile_getter_t() = default;
};

} // namespace baldr
} // namespace valhalla
