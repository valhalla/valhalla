#pragma once

#include <cstdint>
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
   * Mask for response headers
   */
  using header_mask_t = uint16_t;
  const static header_mask_t kHeaderNone = 0;
  const static header_mask_t kHeaderLastModified = 1;
  // const static header_mask_t kHeaderContentLength = 2;

  /**
   * Raw bytes we get as a response.
   */
  using bytes_t = std::vector<char>;

  /**
   * The result of synchronous operation. Contains raw data and operations result code.
   */
  struct GET_response_t {
    bytes_t bytes_;
    status_code_t status_ = status_code_t::FAILURE;
    long http_code_;
  };

  struct HEAD_response_t {
    uint64_t last_modified_time_ = 0;
    status_code_t status_ = status_code_t::FAILURE;
    long http_code_ = 0;
  };

  /**
   * Makes a synchronous request to the corresponding url and returns response_t object.
   * */
  virtual GET_response_t
  get(const std::string& url, const uint64_t offset = 0, const uint64_t size = 0) = 0;

  virtual HEAD_response_t head(const std::string& url, header_mask_t header_mask) = 0;

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
