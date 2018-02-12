#include <vector>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdint>
#include <fstream>

#include <zlib.h>
#include <lz4.h>
#include <lz4hc.h>
#include <lz4frame.h>

constexpr size_t HGT_BYTES = 3601 * 3601 * 2;

long file_size(const std::string& file_name) {
  //TODO: detect gzip and actually validate the uncompressed size?
  struct stat s;
  int rc = stat(file_name.c_str(), &s);
  return rc == 0 ? s.st_size : -1;
}

std::vector<char> read_file(const std::string& file_name, long size) {
  int fd = open(file_name.c_str(), O_RDONLY);
  if(fd == -1)
    throw std::runtime_error("Could not open: " + file_name);
#ifndef __APPLE__
  posix_fadvise(fd, 0, 0, POSIX_FADV_SEQUENTIAL);
#endif
  std::vector<char> in(size);
  if(read(fd, in.data(), size) != size)
    throw std::runtime_error("Could not open: " + file_name);
  close(fd);
  return in;
}

void write_file(const std::string& file_name, const std::vector<char>& out) {
  std::ofstream file(file_name, std::ios::binary | std::ios::trunc);
  file.write(out.data(), out.size());
}

enum class algorithm_t { DEFAULT, HIGH, FRAME };

std::vector<char> lzip(const std::vector<char>& in, int compression_level, algorithm_t algorithm) {
  LZ4F_preferences_t frame_prefs{LZ4F_frameInfo_t{}, compression_level};

  //prepair for destination in memory
  size_t max_compressed_size = algorithm == algorithm_t::FRAME ?
      LZ4F_compressFrameBound(in.size(), &frame_prefs) : LZ4_compressBound(in.size());
  std::vector<char> out(max_compressed_size);

  //compress it in memory
  int compressed_size = 0;
  switch(algorithm) {
    case algorithm_t::DEFAULT:
      compressed_size = LZ4_compress_fast(in.data(), out.data(), in.size(), max_compressed_size, compression_level);
      break;
    case algorithm_t::HIGH:
      compressed_size = LZ4_compress_HC(in.data(), out.data(), in.size(), max_compressed_size, compression_level);
      break;
    case algorithm_t::FRAME:
      compressed_size = LZ4F_compressFrame(out.data(), max_compressed_size, in.data(), in.size(), &frame_prefs);
      break;
    default:
      throw std::runtime_error("Wrong compression type");
  }
  if(compressed_size <= 0)
    throw std::runtime_error("Compression failed: " + std::to_string(compressed_size));
  out.resize(compressed_size);

  return out;
}

std::vector<char> gunzip(std::vector<char>& in) {
  //make a zstream with in and out
  std::vector<char> out(HGT_BYTES);
  z_stream stream = {
    static_cast<Byte*>(static_cast<void*>(in.data())),
      static_cast<unsigned int>(in.size()), static_cast<unsigned int>(in.size()), //the input stream
    static_cast<Byte*>(static_cast<void*>(out.data())), HGT_BYTES, HGT_BYTES //the output stream
  };

  //decompress the file
  if(inflateInit2(&stream, 16 + MAX_WBITS) != Z_OK)
    throw std::runtime_error("gzip decompression init failed");
  auto e = inflate(&stream, Z_FINISH);
  if(e != Z_STREAM_END || stream.total_out != HGT_BYTES)
    throw std::runtime_error("Corrupt gzip elevation data");
  inflateEnd(&stream);
  return out;
}

std::vector<char> lunzip(const std::vector<char>& in, algorithm_t algorithm) {
  size_t in_size = in.size();
  auto* in_ptr = in.data();
  auto* in_end = in.data() + in_size;

  LZ4F_decompressionContext_t context = nullptr;
  LZ4F_frameInfo_t info;
  int hint = -1;

  size_t out_size = HGT_BYTES;
  auto decompressed_size = 0;
  std::vector<char> out(out_size);
  switch(algorithm) {
    case algorithm_t::DEFAULT:
    case algorithm_t::HIGH:
      decompressed_size = LZ4_decompress_fast(in.data(), out.data(), out_size);
      break;
    case algorithm_t::FRAME:
      hint = LZ4F_createDecompressionContext(&context, LZ4F_VERSION);
      if(LZ4F_isError(hint))
        throw std::runtime_error("Decompression initialization failed " + std::string(LZ4F_getErrorName(hint)));

      /*
      //cant get the info to tell us how much to reserve in total so we'll just have it known for now..
      out_size = LZ4F_getFrameInfo(context, &info, in_ptr, &in_size);
      if(LZ4F_isError(hint))
        throw std::runtime_error("Decompression header parsing failed " + std::string(LZ4F_getErrorName(hint)));
      in_ptr += in_size;
      in_size = in_end - in_ptr;
      */

      //decompress
      do {
        hint = LZ4F_decompress(context, out.data(), &out_size, in_ptr, &in_size, nullptr);
        if(LZ4F_isError(hint))
          throw std::runtime_error("Decompression of frame failed " + std::string(LZ4F_getErrorName(hint)));
        in_ptr += in_size;
        in_size = in_end - in_ptr;
      } while(hint && in_size);

      LZ4F_freeDecompressionContext(context);
      break;
    default:
      throw std::runtime_error("Wrong decompression algorithm");
  }

  if(decompressed_size < 0)
    throw std::runtime_error("Deompression failed: " + std::to_string(decompressed_size));

  return out;
}

int main(int argc, char** argv){
  if(argc < 2)
    return 0;
  //TODO: add arguments to this

  std::string file_name(argv[1]);
  auto tile = read_file(file_name, file_size(file_name));
  //test the file for proper lz4hc encoding
  if(file_name.find(".lz4") == file_name.size() - 4) {
    tile = lunzip(tile, algorithm_t::HIGH);
    write_file(file_name.substr(0, file_name.size() - 4), tile);
  }//compress an existing file
  else {
    //gunzip it
    if(file_name.find(".gz") == file_name.size() - 3) {
      tile = gunzip(tile);
      file_name = file_name.substr(0, file_name.size() - 3);
    }
    //lzip it
    tile = lzip(tile, 9, algorithm_t::HIGH);
    write_file(file_name + ".lz4", tile);
  }

  return 0;
}
