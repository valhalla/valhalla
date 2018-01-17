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
      LOG_WARN(root_dir + " has no elevation tiles");
      files.clear();
    }
    return files;
  }

  std::string hgt_name(int lon, int lat) {
    std::string name(lat < 0 ? "S" : "N");
    name.append(std::to_string(std::abs(lat)));
    name.insert(1, 3 - name.size(), '0');
    name.push_back(lon < 0 ? 'W' : 'E');
    name.append(std::to_string(std::abs(lon)));
    name.append(".hgt");
    name.insert(4, 11 - name.size(), '0');
    return name;
  }

  uint16_t is_hgt(const std::string& name, bool& zipped_extension) {
    boost::smatch m;
    boost::regex e(".*/([NS])([0-9]{2})([WE])([0-9]{3})\\.hgt(\\.gz|\\.lz4)?$");
    if(boost::regex_search(name, m, e)) {
      zipped_extension = m[5].length();
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

  void gunzip(const std::pair<std::string, uint64_t>& file, std::vector<int16_t>& tile) {
    //slurp in the file
    int fd = open(file.first.c_str(), O_RDONLY);
    if(fd == -1)
      throw std::runtime_error("Could not open: " + file.first);
    posix_fadvise(fd, 0, 0, 1);  // FDADVICE_SEQUENTIAL
    std::vector<Byte> in(file.second);
    if(read(fd, &in[0], file.second) != file.second)
      throw std::runtime_error("Could not open: " + file.first);
    close(fd);

    //make a zstream with in and out
    z_stream stream = {
      &in[0], static_cast<unsigned int>(file.second), static_cast<unsigned int>(file.second), //the input stream
      static_cast<Byte*>(static_cast<void*>(tile.data())), HGT_BYTES, HGT_BYTES //the output stream
    };

    //decompress the file
    int err = inflateInit2(&stream, 16 + MAX_WBITS);
    if(err == Z_OK) {
      err = inflate(&stream, Z_FINISH);
      if(err != Z_STREAM_END || stream.total_out != HGT_PIXELS * sizeof(int16_t))
        throw std::runtime_error("Corrupt elevation data: " + file.first);
    }
    inflateEnd(&stream);
  }

  void lunzip(const std::pair<std::string, uint64_t>& file, std::vector<int16_t>& tile) {
    //slurp in the file
    int fd = open(file.first.c_str(), O_RDONLY);
    if(fd == -1)
      throw std::runtime_error("Could not open: " + file.first);
    posix_fadvise(fd, 0, 0, 1);  // FDADVICE_SEQUENTIAL
    std::vector<char> in(file.second);
    if(read(fd, &in[0], file.second) != file.second)
      throw std::runtime_error("Could not open: " + file.first);
    close(fd);

    auto decompressed_size = LZ4_decompress_safe(
      in.data(), static_cast<char*>(static_cast<void*>(tile.data())), file.second, HGT_BYTES);

    if(decompressed_size != HGT_BYTES)
      throw std::runtime_error("Corrupt elevation data: " + file.first + std::to_string(decompressed_size));
  }

}

namespace valhalla {
namespace skadi {

  sample::sample(const std::string& data_source):
      mapped_cache(TILE_COUNT), unzipped(-1, std::vector<int16_t>(HGT_PIXELS)) {
    //check the directory for files that look like what we need
    auto files = get_files(data_source);
    for(const auto& f : files) {
      //make sure its a valid index
      bool zipped_extension;
      auto index = is_hgt(f, zipped_extension);
      if(index < mapped_cache.size()) {
        auto size = file_size(f);
        if(zipped_extension)
          zipped[index] = std::make_pair(f, size);
        else if(size == HGT_BYTES)
          mapped_cache[index].map(f, HGT_PIXELS * sizeof(int16_t));
        else
          LOG_WARN("Corrupt elevation data: " + f);
      }
    }
  }

  const int16_t* sample::source(uint16_t index) const {
    //bail if its out of bounds
    if(index >= TILE_COUNT)
      return nullptr;

    //try the memmap cache
    const auto* s = mapped_cache[index].get();

    //if it failed try gzipped sources
    if(s == nullptr) {
      //we have it cached
      if(unzipped.first == index)
        return unzipped.second.data();

      //we cant get it from disk
      auto found = zipped.find(index);
      if(found == zipped.cend())
        return nullptr;

      //get it off disk gz
      if(found->second.first.back() == 'z')
        gunzip(found->second, unzipped.second);
      //get it off disk lz4
      else
        lunzip(found->second, unzipped.second);
      unzipped.first = index;
      s = unzipped.second.data();

      //TODO: LRU stuff
    }

    return s;
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
