#pragma once

#include <functional>
#include <zlib.h>

namespace valhalla {
namespace baldr {

std::vector<Byte> deflate(std::vector<int16_t>& in) {
  auto in_size = static_cast<unsigned int>(in.size() * sizeof(int16_t));
  size_t max_compressed_size = in_size + ((5 * in_size + 16383) / 16384) + 6;
  std::vector<Byte> out(max_compressed_size);

  z_stream stream{static_cast<Byte*>(static_cast<void*>(&in[0])),
                  in_size,
                  in_size,
                  &out[0],
                  static_cast<unsigned int>(out.size()),
                  static_cast<unsigned int>(out.size())};

  // use gzip standard headers (15 | 16)
  int err = deflateInit2(&stream, Z_BEST_COMPRESSION, Z_DEFLATED, 15 | 16, 9, Z_DEFAULT_STRATEGY);
  if (err != Z_OK)
    throw std::runtime_error("Couldn't compress the file");
  err = deflate(&stream, Z_FINISH);
  if (err != Z_STREAM_END)
    throw std::runtime_error("Didn't reach end of compression input stream");
  out.resize(stream.total_out);
  deflateEnd(&stream);

  return out;
}

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
             bool gzip = true) {
  // initialize the stream
  // add 16 to window bits for gzip header instead of zlib header, 9 is max speed
  z_stream stream{};
  if (deflateInit2(&stream, level, Z_DEFLATED, gzip ? 15 + 16 : 15, 9, Z_DEFAULT_STRATEGY) != Z_OK)
    return false;

  int flush = Z_NO_FLUSH;
  do {
    // if we need more input, get it and fail otherwise
    try {
      if (stream.avail_in == 0)
        flush = src_func(stream);
    } catch (...) {
      deflateEnd(&stream);
      return false;
    }

    // if we need more output, get it and fail otherwise
    try {
      if (stream.avail_out == 0)
        out_func(stream);
    } catch (...) {
      deflateEnd(&stream);
      return false;
    }

    // deflate some bytes
    switch (deflate(&stream, flush)) {
      // theres still more
      case Z_OK:
        continue;
      // we are done
      case Z_STREAM_END:
        deflateEnd(&stream);
        return true;
      // some error happened
      default:
        deflateEnd(&stream);
        return false;
    }
    // we keep deflating until our src is empty
  } while (flush != Z_FINISH);

  // if we got here we expected to finish but weren't thats not good
  deflateEnd(&stream);
  return false;
}

/* Inflates gzip or zlib wrapped deflated data
 * @param src_func  function which modifies the stream to read more input
 * @param dst_func  function which modifies the stream to write more output
 * @return          returns true if the stream was successfully inflated, false otherwise
 */
bool inflate(const std::function<int(z_stream&)>& src_func,
             const std::function<void(z_stream&)>& dst_func) {

  // initialize the stream
  // MAX_WBITS is the max size of the window and should be 15, this will work with headerless
  // defalted streams to work with gzip add 16, to work with both gzip and libz add 32
  z_stream stream{};
  if (inflateInit2(&stream, MAX_WBITS + 32) != Z_OK)
    return false;

  int flush = Z_NO_FLUSH;
  do {
    // if we need more input, get it and fail otherwise
    try {
      if (stream.avail_in == 0)
        flush = src_func(stream);
    } catch (...) {
      inflateEnd(&stream);
      return false;
    }

    // if we need more output, get it and fail otherwise
    try {
      if (stream.avail_out == 0)
        dst_func(stream);
    } catch (...) {
      inflateEnd(&stream);
      return false;
    }

    // inflate some bytes
    switch (inflate(&stream, flush)) {
      // theres still more
      case Z_OK:
        continue;
      // we are done
      case Z_STREAM_END:
        inflateEnd(&stream);
        return true;
      // some error happened
      default:
        inflateEnd(&stream);
        return false;
    }
    // we keep inflating until our src is empty
  } while (flush != Z_FINISH);

  // if we got here we expected to finish but weren't thats not good
  inflateEnd(&stream);
  return false;
}

} // namespace baldr
} // namespace valhalla