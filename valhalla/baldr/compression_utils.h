#pragma once

#include <functional>
#include <zlib.h>

namespace valhalla {
namespace baldr {

/* Deflates data with gzip or zlib wrapper
 * @param src_func  function which modifies the stream to read more input
 * @param dst_func  function which modifies the stream to write more output
 * @param level     what compression level to use
 * @param gzip      whether or not to write a gzip header instead of a zlib one
 * @return          returns true if the stream was successfully inflated, false otherwise
 */
bool deflate(const std::function<int(z_stream&)>& src_func,
             const std::function<void(z_stream&)>& dst_func,
             int level = Z_BEST_COMPRESSION,
             bool gzip = true);

/* Inflates gzip or zlib wrapped deflated data
 * @param src_func  function which modifies the stream to read more input
 * @param dst_func  function which modifies the stream to write more output
 * @return          returns true if the stream was successfully inflated, false otherwise
 */
bool inflate(const std::function<void(z_stream&)>& src_func,
             const std::function<int(z_stream&)>& dst_func);

} // namespace baldr
} // namespace valhalla