#include "baldr/compression_utils.h"

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
             int level,
             bool gzip) {
  // initialize the stream
  // add 16 to window bits for gzip header instead of zlib header, 9 is max speed
  z_stream stream{};
  if (deflateInit2(&stream, level, Z_DEFLATED, gzip ? 15 + 16 : 15, 9, Z_DEFAULT_STRATEGY) != Z_OK)
    return false;

  int flush = Z_NO_FLUSH;
  int code = Z_OK;
  do {
    // if we need more src material
    try {
      if (stream.avail_in == 0)
        flush = src_func(stream);
    } catch (...) {
      deflateEnd(&stream);
      return false;
    }

    do {
      // if we need more space in the dst
      try {
        if (stream.avail_out == 0)
          dst_func(stream);
      } catch (...) {
        deflateEnd(&stream);
        return false;
      }

      // only one fatal error to worry about
      code = deflate(&stream, flush);
      if (code == Z_STREAM_ERROR) {
        deflateEnd(&stream);
        return false;
      }
      // only stop when we've got nothing more to put in the dst buffer
    } while (stream.avail_out == 0);
    // only stop when we signaled that we have no more input
  } while (flush != Z_FINISH);

  // hand back the final buffer
  dst_func(stream);
  // if we got here we expected to finish but weren't thats not good
  deflateEnd(&stream);
  return true;
}

/* Inflates gzip or zlib wrapped deflated data
 * @param src_func  function which modifies the stream to read more input
 * @param dst_func  function which modifies the stream to write more output
 * @return          returns true if the stream was successfully inflated, false otherwise
 */
bool inflate(const std::function<void(z_stream&)>& src_func,
             const std::function<int(z_stream&)>& dst_func) {

  // initialize the stream
  // MAX_WBITS is the max size of the window and should be 15, this will work with headerless
  // defalted streams to work with gzip add 16, to work with both gzip and libz add 32
  z_stream stream{};
  if (inflateInit2(&stream, MAX_WBITS + 32) != Z_OK)
    return false;

  int flush = Z_NO_FLUSH;
  int code = Z_OK;
  do {
    // if we need more src material
    try {
      if (stream.avail_in == 0)
        src_func(stream);
      if (stream.avail_in == 0)
        throw;
    } catch (...) {
      inflateEnd(&stream);
      return false;
    }

    do {
      // if we need more space in the dst
      try {
        if (stream.avail_out == 0)
          flush = dst_func(stream);
      } catch (...) {
        inflateEnd(&stream);
        return false;
      }

      // several fatal errors to worry about
      code = inflate(&stream, flush);
      switch (code) {
        case Z_STREAM_ERROR:
        case Z_NEED_DICT:
        case Z_DATA_ERROR:
        case Z_MEM_ERROR:
          inflateEnd(&stream);
          return false;
      }
      // only stop when we've got nothing more to put in the dst buffer
    } while (stream.avail_out == 0);
    // only stop when we reached the end of the stream
  } while (code != Z_STREAM_END);

  // hand back the final buffer
  dst_func(stream);
  // if we got here we expected to finish but weren't thats not good
  inflateEnd(&stream);
  return true;
}

} // namespace baldr
} // namespace valhalla