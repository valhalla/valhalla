#include "skadi/sample.h"

#include <cstddef>
#include <cmath>
#include <stdexcept>
#include <limits>
#include <list>
#include <fstream>
#include <boost/regex.hpp>
#include <sys/stat.h>
#include <zlib.h>
#include <lz4.h>
#include <lz4hc.h>

#include <boost/filesystem.hpp>
#include <boost/optional.hpp>

#include "midgard/logging.h"
#include "midgard/pointll.h"

namespace {
  //srtmgl1 holds 1x1 degree tiles but oversamples the egde of the tile
  //by .5 seconds on all sides. that means that the center of pixel 0 is
  //located at the tiles lat,lon (which is important for bilinear filtering)
  //it also means that there are 3601 pixels per row and per column
  constexpr size_t HGT_DIM = 3601;
  constexpr size_t HGT_PIXELS = HGT_DIM * HGT_DIM;
  constexpr size_t HGT_BYTES = sizeof(int16_t) * HGT_PIXELS;
  constexpr int16_t NO_DATA_VALUE = -32768;
  constexpr int16_t NO_DATA_HIGH = 16384;
  constexpr int16_t NO_DATA_LOW = -16384;
  constexpr size_t TILE_COUNT = 180 * 360;

  //macro is faster than inline funciton for this..
  #define out_of_range(v) v > NO_DATA_HIGH || v < NO_DATA_LOW

  std::list<std::string> get_files(const std::string& root_dir) {
    std::list<std::string> files;
    try {
      for (boost::filesystem::recursive_directory_iterator i(root_dir), end; i != end; ++i)
        if (!is_directory(i->path()))
          files.push_back(i->path().string());
    }//couldn't get data
    catch(...) {
      LOG_WARN(root_dir + " currently has no elevation tiles");
      files.clear();
    }
    return files;
  }

  std::string name_hgt(int16_t index) {
    auto x = (index % 360) - 180;
    auto y = (index / 360) - 90;

    std::string name(y < 0 ? "/S" : "/N");
    y = std::abs(y);
    if(y < 10)
      name.push_back('0');
    name.append(std::to_string(y));
    name.append(name);

    name.append(x < 0 ? "W" : "E");
    x = std::abs(x);
    if(x < 100)
      name.push_back('0');
    if(x < 10)
      name.push_back('0');
    name.append(std::to_string(x));
    name.append(".hgt");

    return name;
  }

  template<typename fmt_t>
  uint16_t is_hgt(const std::string& name, fmt_t& fmt) {
    boost::smatch m;
    boost::regex e(".*/([NS])([0-9]{2})([WE])([0-9]{3})\\.hgt(\\.gz|\\.lz4)?$");
    if(boost::regex_search(name, m, e)) {
      //enum class format_t{ UNKNOWN = 0, GZIP = 1, LZ4 = 2, RAW = 3 };
      fmt = static_cast<fmt_t>(m[5].length() ? (m[5] == ".lz4" ? 2 : (m[5] == ".gz" ? 1 : 0)) : 3);
      auto lon = std::stoul(m[4]) * (m[3] == "E" ? 1 : -1) + 180;
      auto lat = std::stoul(m[2]) * (m[1] == "N" ? 1 : -1) + 90;
      if(lon >= 0 && lon < 360 && lat >=0 && lat < 180)
        return lat * 360 + lon;
    }
    return -1;
  }

  int16_t flip(int16_t value) {
    return ((value & 0xFF) << 8) | ((value >> 8) & 0xFF);
  }

  uint64_t file_size(const std::string& file_name) {
    //TODO: detect gzip and actually validate the uncompressed size?
    struct stat s;
    int rc = stat(file_name.c_str(), &s);
    return rc == 0 ? s.st_size : -1;
  }

  void gunzip(const valhalla::midgard::mem_map<char>& in, int16_t* tile) {
    //make a zstream with in and out
    z_stream stream = {
      static_cast<Byte*>(static_cast<void*>(in.get())),
        static_cast<unsigned int>(in.size()), static_cast<unsigned int>(in.size()), //the input stream
      static_cast<Byte*>(static_cast<void*>(tile)), HGT_BYTES, HGT_BYTES //the output stream
    };

    //decompress the file
    if(inflateInit2(&stream, 16 + MAX_WBITS) != Z_OK)
      throw std::runtime_error("gzip decompression init failed");
    auto e = inflate(&stream, Z_FINISH);
    if(e != Z_STREAM_END || stream.total_out != HGT_BYTES)
      throw std::runtime_error("Corrupt gzip elevation data");
    inflateEnd(&stream);
  }

  void lunzip(const valhalla::midgard::mem_map<char>& in, int16_t* tile) {
    auto compressed_size = LZ4_decompress_fast(in.get(), static_cast<char*>(static_cast<void*>(tile)), HGT_BYTES);
    if(compressed_size != in.size())
      throw std::runtime_error("Corrupt lz4 elevation data");
  }

}

namespace valhalla {
namespace skadi {

  sample::sample(const std::string& data_source):
      mapped_cache(TILE_COUNT), unzipped_cache(-1, std::vector<int16_t>(HGT_PIXELS)), data_source(data_source) {
    //messy but needed
    while(this->data_source.size() && this->data_source.back() == '/')
      this->data_source.pop_back();

    //check the directory for files that look like what we need
    auto files = get_files(data_source);
    for(const auto& f : files) {
      //make sure its a valid index
      format_t format;
      auto index = is_hgt(f, format);
      if(index < mapped_cache.size() && format > mapped_cache[index].first) {
        auto size = file_size(f);
        if(format == format_t::RAW && size != HGT_BYTES) {
          LOG_WARN("Corrupt elevation data: " + f);
          continue;
        }
        mapped_cache[index].first = format;
        mapped_cache[index].second.map(f, size, POSIX_MADV_SEQUENTIAL);
      }
    }
  }

  const int16_t* sample::source(uint16_t index) const {
    //bail if its out of bounds
    if(index >= TILE_COUNT)
      return nullptr;

    //if we dont have anything maybe its lazy loaded
    auto& mapped = mapped_cache[index];
    if(mapped.second.get() == nullptr) {
      auto f = data_source + name_hgt(index);
      auto size = file_size(f);
      if(size != HGT_BYTES)
        return nullptr;
      mapped.first = format_t::RAW;
      mapped.second.map(f, size, POSIX_MADV_SEQUENTIAL);
    }

    //we have it raw or we dont
    if(mapped.first == format_t::RAW)
      return static_cast<const int16_t*>(static_cast<const void*>(mapped.second.get()));

    //if we have it already unzipped
    if(unzipped_cache.first == index)
      return unzipped_cache.second.data();

    //we have to unzip it
    try {
      if(mapped.first == format_t::LZ4HC)
        lunzip(mapped.second, unzipped_cache.second.data());
      else
        gunzip(mapped.second, unzipped_cache.second.data());
    }//failed to unzip
    catch(...) {
      LOG_WARN("Corrupt compressed elevation data");
      unzipped_cache.first = -1;
      return nullptr;
    }

    //update the simple cache
    //TODO: LRU
    unzipped_cache.first = index;
    return unzipped_cache.second.data();
  }

  template <class coord_t>
  double sample::get(const coord_t& coord) const {
    //check the cache and load
    auto lon = std::floor(coord.first);
    auto lat = std::floor(coord.second);
    auto index = static_cast<uint16_t>(lat + 90) * 360 + static_cast<uint16_t>(lon + 180);

    //get the proper source of the data
    const auto* t = source(index);
    if(t == nullptr)
      return NO_DATA_VALUE;

    //figure out what row and column we need from the array of data
    //NOTE: data is arranged from upper left to bottom right, so y is flipped

    //fractional pixel
    double u = (coord.first - lon) * (HGT_DIM - 1);
    double v = (1.0 - (coord.second - lat)) * (HGT_DIM - 1);

    //integer pixel
    size_t x = std::floor(u);
    size_t y = std::floor(v);

    //coefficients
    double u_ratio = u - x;
    double v_ratio = v - y;
    double u_inv = 1 - u_ratio;
    double v_inv = 1 - v_ratio;
    double a_coef = u_inv * v_inv;
    double b_coef = u_ratio * v_inv;
    double c_coef = u_inv * v_ratio;
    double d_coef = u_ratio * v_ratio;

    //values
    double adjust = 0;
    auto a = flip(t[y * HGT_DIM + x]);
    auto b = flip(t[y * HGT_DIM + x + 1]);
    if(out_of_range(a)) a_coef = 0;
    if(out_of_range(b)) b_coef = 0;

    //first part of the bilinear interpolation
    auto value = a * a_coef + b * b_coef;
    adjust += a_coef + b_coef;
    //LOG_INFO('{' + std::to_string(y * HGT_DIM + x) + ',' + std::to_string(a) + '}');
    //LOG_INFO('{' + std::to_string(y * HGT_DIM + x + 1) + ',' + std::to_string(b) + '}');
    //only need the second part if you aren't right on the row
    //this also protects from a corner case where you sample past the end of the image
    if(y < HGT_DIM - 1) {
      auto c = flip(t[(y + 1) * HGT_DIM + x]);
      auto d = flip(t[(y + 1) * HGT_DIM + x + 1]);
      if(out_of_range(c)) c_coef = 0;
      if(out_of_range(d)) d_coef = 0;
      //LOG_INFO('{' + std::to_string((y + 1) * HGT_DIM + x) + ',' + std::to_string(c) + '}');
      //LOG_INFO('{' + std::to_string((y + 1) * HGT_DIM + x + 1) + ',' + std::to_string(d) + '}');
      value += c * c_coef + d * d_coef;
      adjust += c_coef + d_coef;
    }
    //if we are missing everything then give up
    if(adjust == 0)
      return NO_DATA_VALUE;
    //if we were missing some we need to adjust by that
    return value / adjust;
  }

  template <class coords_t>
  std::vector<double> sample::get_all(const coords_t& coords) const {
    std::vector<double> values;
    values.reserve(coords.size());
    for(const auto& coord : coords)
      values.emplace_back(get(coord));
    return values;
  }

  double sample::get_no_data_value() const {
    return NO_DATA_VALUE;
  }

  //explicit instantiations for templated get
  template double sample::get<std::pair<double, double> >(const std::pair<double, double>&) const;
  template double sample::get<std::pair<float, float> >(const std::pair<float, float>&) const;
  template double sample::get<midgard::PointLL>(const midgard::PointLL&) const;
  template double sample::get<midgard::Point2>(const midgard::Point2&) const;
  template std::vector<double> sample::get_all<std::list<std::pair<double, double> > >(const std::list<std::pair<double, double> >&) const;
  template std::vector<double> sample::get_all<std::vector<std::pair<double, double> > >(const std::vector<std::pair<double, double> >&) const;
  template std::vector<double> sample::get_all<std::list<std::pair<float, float> > >(const std::list<std::pair<float, float> >&) const;
  template std::vector<double> sample::get_all<std::vector<std::pair<float, float> > >(const std::vector<std::pair<float, float> >&) const;
  template std::vector<double> sample::get_all<std::list<midgard::PointLL> >(const std::list<midgard::PointLL>&) const;
  template std::vector<double> sample::get_all<std::vector<midgard::PointLL> >(const std::vector<midgard::PointLL>&) const;
  template std::vector<double> sample::get_all<std::list<midgard::Point2> >(const std::list<midgard::Point2>&) const;
  template std::vector<double> sample::get_all<std::vector<midgard::Point2> >(const std::vector<midgard::Point2>&) const;

}
}
