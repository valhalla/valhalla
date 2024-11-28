#include "skadi/sample.h"

#include <cmath>
#include <cstddef>
#include <future>
#include <list>
#include <optional>
#include <regex>
#include <unordered_map>
#include <unordered_set>

#include <lz4frame.h>
#include <sys/stat.h>

#include "baldr/compression_utils.h"
#include "filesystem.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/sequence.h"
#include "valhalla/baldr/curl_tilegetter.h"

namespace {
// srtmgl1 holds 1x1 degree tiles but oversamples the edge of the tile
// by .5 seconds on all sides. that means that the center of pixel 0 is
// located at the tiles lat,lon (which is important for bilinear filtering)
// it also means that there are 3601 pixels per row and per column
constexpr size_t HGT_DIM = 3601;
constexpr size_t HGT_PIXELS = HGT_DIM * HGT_DIM;
constexpr size_t HGT_BYTES = sizeof(int16_t) * HGT_PIXELS;
constexpr int16_t NO_DATA_VALUE = -32768;
constexpr int16_t NO_DATA_HIGH = 16384;
constexpr int16_t NO_DATA_LOW = -16384;
constexpr size_t TILE_COUNT = 180 * 360;
constexpr int8_t UNPACKED_TILES_COUNT = 50;

// macro is faster than inline function for this...
#define out_of_range(v) v > NO_DATA_HIGH || v < NO_DATA_LOW

int16_t flip(int16_t value) {
  return ((value & 0xFF) << 8) | ((value >> 8) & 0xFF);
}

uint64_t file_size(const std::string& file_name) {
  // TODO: detect gzip and actually validate the uncompressed size?
  struct stat s {};
  int rc = stat(file_name.c_str(), &s);
  return rc == 0 ? s.st_size : -1;
}

} // namespace

namespace valhalla {
namespace skadi {

enum class format_t { UNKNOWN = 0, RAW = 1, GZIP = 2, LZ4 = 3 };

class cache_item_t {
private:
  format_t format;
  valhalla::midgard::mem_map<char> data;
  int usages;
  const char* unpacked;

public:
  cache_item_t() : format(format_t::UNKNOWN), usages(0), unpacked(nullptr) {
  }
  cache_item_t(cache_item_t&&) = default;
  ~cache_item_t() {
    free((void*)unpacked);
  }

  bool init(const std::string& path, format_t format) {
    auto size = file_size(path);
    if (format == format_t::RAW && size != HGT_BYTES) {
      return false;
    }
    this->format = format;
    data.map(path, size, POSIX_MADV_SEQUENTIAL, true);
    return true;
  }

  inline const char* get_data() const {
    return data.get();
  }

  inline format_t get_format() const {
    return format;
  }

  inline int& get_usages() {
    return usages;
  }

  inline const char* get_unpacked() {
    return unpacked;
  }

  inline const char* detach_unpacked() {
    auto rv = unpacked;
    unpacked = nullptr;
    return rv;
  }

  bool unpack(const char* unpacked) {
    this->unpacked = unpacked;

    if (format == format_t::GZIP) {
      // for setting where to read compressed data from
      auto src_func = [this](z_stream& s) -> void {
        s.next_in = static_cast<Byte*>(static_cast<void*>(data.get()));
        s.avail_in = static_cast<unsigned int>(data.size());
      };

      // for setting where to write the uncompressed data to
      auto dst_func = [this](z_stream& s) -> int {
        s.next_out = (Byte*)(this->unpacked);
        s.avail_out = HGT_BYTES;
        return Z_FINISH; // we know the output will hold all the input
      };

      // we have to unzip it
      if (!baldr::inflate(src_func, dst_func)) {
        LOG_WARN("Corrupt gzip elevation data");
        format = format_t::UNKNOWN;
        return false;
      }
    } else if (format == format_t::LZ4) {
      LZ4F_decompressionContext_t decode;
      LZ4F_decompressOptions_t options;
      LZ4F_createDecompressionContext(&decode, LZ4F_VERSION);

      // Take these two values locally, since LZ4F_decompress expects pointers...
      size_t src_size = data.size();
      size_t dest_size = HGT_BYTES;
      size_t result;

      do {
        result = LZ4F_decompress(decode, const_cast<char*>(this->unpacked), &dest_size, data.get(),
                                 &src_size, &options);
        if (LZ4F_isError(result)) {
          LZ4F_freeDecompressionContext(decode);
          LOG_WARN("Corrupt lz4 elevation data");
          format = format_t::UNKNOWN;
          return false;
        }
      } while (result != 0);

      LZ4F_freeDecompressionContext(decode);
    } else {
      LOG_WARN("Corrupt elevation data of unknown type");
      format = format_t::UNKNOWN;
      return false;
    }

    return true;
  }

  static std::optional<std::pair<uint16_t, format_t>> parse_hgt_name(const std::string& name) {
    std::smatch m;
    std::regex e(".*/([NS])([0-9]{2})([WE])([0-9]{3})\\.hgt(\\.(gz|lz4))?$");
    if (std::regex_search(name, m, e)) {
      // enum class format_t{ UNKNOWN = 0, GZIP = 1, RAW = 3, LZ4 = 4 };
      format_t fmt;
      if (m[5].matched) {
        if (m[5] == ".gz") {
          fmt = format_t::GZIP;
        } else if (m[5] == ".lz4") {
          fmt = format_t::LZ4;
        } else {
          fmt = format_t::UNKNOWN;
        }
      } else {
        fmt = format_t::RAW;
      }

      auto lon = std::stoi(m[4]) * (m[3] == "E" ? 1 : -1) + 180;
      auto lat = std::stoi(m[2]) * (m[1] == "N" ? 1 : -1) + 90;
      if (lon >= 0 && lon < 360 && lat >= 0 && lat < 180) {
        return std::make_pair(uint16_t(lat * 360 + lon), fmt);
      }
    }
    return std::nullopt;
  }
};

// tile_data object holds unpacked elevation tile data
class tile_data {
private:
  cache_t* c;
  const int16_t* data;
  uint16_t index;
  bool reusable;

public:
  tile_data() : c(nullptr), data(nullptr), index(TILE_COUNT), reusable(false) {
  }

  tile_data(const tile_data& other) : c(nullptr) {
    *this = other;
  }

  tile_data(cache_t* c, uint16_t index, bool reusable, const int16_t* data);
  ~tile_data();
  tile_data& operator=(const tile_data& other);

  tile_data& operator=(tile_data&& other) {
    std::swap(c, other.c);
    std::swap(data, other.data);
    std::swap(index, other.index);
    std::swap(reusable, other.reusable);

    return *this;
  }

  inline explicit operator bool() const {
    return data != nullptr;
  }

  uint16_t get_index() const {
    return index;
  }

  double get(double u, double v) const {
    // integer pixel
    size_t x = std::floor(u);
    size_t y = std::floor(v);

    // coefficients
    double u_ratio = u - x;
    double v_ratio = v - y;
    double u_inv = 1 - u_ratio;
    double v_inv = 1 - v_ratio;
    double a_coef = u_inv * v_inv;
    double b_coef = u_ratio * v_inv;
    double c_coef = u_inv * v_ratio;
    double d_coef = u_ratio * v_ratio;

    // values
    double adjust = 0;
    auto a = flip(data[y * HGT_DIM + x]);
    auto b = flip(data[y * HGT_DIM + x + 1]);
    if (out_of_range(a)) {
      a_coef = 0;
    }
    if (out_of_range(b)) {
      b_coef = 0;
    }

    // first part of the bilinear interpolation
    auto value = a * a_coef + b * b_coef;
    adjust += a_coef + b_coef;
    // LOG_INFO('{' + std::to_string(y * HGT_DIM + x) + ',' + std::to_string(a) + '}');
    // LOG_INFO('{' + std::to_string(y * HGT_DIM + x + 1) + ',' + std::to_string(b) + '}');
    // only need the second part if you aren't right on the row
    // this also protects from a corner case where you sample past the end of the image
    if (y < HGT_DIM - 1) {
      auto c = flip(data[(y + 1) * HGT_DIM + x]);
      auto d = flip(data[(y + 1) * HGT_DIM + x + 1]);
      if (out_of_range(c)) {
        c_coef = 0;
      }
      if (out_of_range(d)) {
        d_coef = 0;
      }
      // LOG_INFO('{' + std::to_string((y + 1) * HGT_DIM + x) + ',' + std::to_string(c) + '}');
      // LOG_INFO('{' + std::to_string((y + 1) * HGT_DIM + x + 1) + ',' + std::to_string(d) + '}');
      value += c * c_coef + d * d_coef;
      adjust += c_coef + d_coef;
    }
    // if we are missing everything then give up
    if (adjust == 0) {
      return get_no_data_value();
    }
    // if we were missing some we need to adjust by that
    return value / adjust;
  }
};

struct cache_t {
  // Cached tiles
  std::vector<cache_item_t> cache;
  // Set of reusable tile indexes
  std::unordered_set<uint16_t> reusable;
  // Map of pending tiles. No matter how many requests received, only one inflate job per tile
  // started.
  std::unordered_map<uint16_t, std::shared_future<tile_data>> pending_tiles;
  // Guards access to the pending_tiles
  std::recursive_mutex mutex;
  // Elevation tile path
  std::string data_source;

  void increment_usages(uint16_t index) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    cache[index].get_usages()++;
  }

  void decrement_usages(uint16_t index) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    cache[index].get_usages()--;
  }

  // no need for synchronization as size is constant(set in constructor
  // and never change after that)
  std::size_t size() const noexcept {
    return cache.size();
  }

  bool insert(size_t pos, const std::string& path, format_t format);

  tile_data source(uint16_t index);
};

bool cache_t::insert(size_t pos, const std::string& path, format_t format) {
  if (pos >= cache.size())
    return false;

  std::lock_guard<std::recursive_mutex> lock(mutex);
  return cache[pos].init(path, format);
}

tile_data cache_t::source(uint16_t index) {
  // bail if it's out of bounds
  if (index >= TILE_COUNT) {
    return {};
  }

  // if we don't have anything maybe it's lazy loaded
  auto& item = cache[index];
  if (item.get_data() == nullptr) {
    auto f = data_source + get_hgt_file_name(index);
    item.init(f, format_t::RAW);
  }

  // it wasn't in cache and when we tried to load it the file was of unknown type
  if (item.get_format() == format_t::UNKNOWN) {
    return {};
  }

  // we have it raw or we don't
  if (item.get_format() == format_t::RAW) {
    return {this, index, false, (const int16_t*)item.get_data()};
  }

  // we were able to load it but the format wasn't RAW, which only leaves compressed formats
  mutex.lock();
  auto it = pending_tiles.find(index);
  if (it != pending_tiles.end()) {
    auto future = it->second;
    mutex.unlock();
    return future.get();
  }

  // item in cache is already unpacked
  const char* unpacked = item.get_unpacked();
  if (unpacked) {
    auto rv = tile_data(this, index, true, (const int16_t*)unpacked);
    mutex.unlock();
    return rv;
  }

  std::promise<tile_data> promise;
  it = pending_tiles.emplace(index, promise.get_future()).first;

  if (reusable.size() >= UNPACKED_TILES_COUNT) {
    for (auto i : reusable) {
      if (cache[i].get_usages() <= 0) {
        reusable.erase(i);
        unpacked = cache[i].detach_unpacked();
        break;
      }
    }
  }
  if (!unpacked) {
    unpacked = (char*)malloc(HGT_BYTES);
  }
  reusable.insert(index);
  auto rv = tile_data(this, index, true, (const int16_t*)unpacked);
  mutex.unlock();

  if (!item.unpack(unpacked)) {
    rv = tile_data();
  }

  mutex.lock();
  promise.set_value(rv);
  pending_tiles.erase(it);
  mutex.unlock();
  return rv;
}

tile_data::tile_data(cache_t* c, uint16_t index, bool reusable, const int16_t* data)
    : c(c), data(data), index(index), reusable(reusable) {
  if (reusable)
    c->increment_usages(index);
}

tile_data::~tile_data() {
  if (reusable)
    c->decrement_usages(index);
}

tile_data& tile_data::operator=(const tile_data& other) {
  if (c && reusable)
    c->decrement_usages(index);

  c = other.c;
  data = other.data;
  index = other.index;
  reusable = other.reusable;

  if (c && reusable)
    c->increment_usages(index);
  return *this;
}

sample::sample(const boost::property_tree::ptree& pt)
    : sample(pt.get<std::string>("additional_data.elevation", "")) {
  url_ = pt.get<std::string>("additional_data.elevation_url", "");

  auto max_concurrent_users = pt.get<size_t>("mjolnir.max_concurrent_reader_users", 1);
  remote_loader_ =
      std::make_unique<baldr::curl_tile_getter_t>(max_concurrent_users,
                                                  pt.get<std::string>("mjolnir.user_agent", ""),
                                                  false);

  // this line used only for testing, for more details check elevation_builder.cc
  remote_path_ = pt.get<std::string>("additional_data.elevation_dir", "");
}

sample::sample(const std::string& data_source) {
  // cache initialization logic moved to different a method
  // to make future constructor merging easier, see sample.h
  cache_initialisation(data_source);
}

sample::~sample() {
}

template <class coord_t> double sample::get(const coord_t& coord, tile_data& tile) {
  // check the cache and load
  auto lon = std::floor(coord.first);
  auto lat = std::floor(coord.second);
  auto index = static_cast<uint16_t>(lat + 90) * 360 + static_cast<uint16_t>(lon + 180);

  // the caller can pass a cached tile, so we only fetch one if its not the one they already have
  if (index != tile.get_index()) {
    {
      std::lock_guard<std::mutex> _(cache_lck);
      tile = cache_->source(index);
    }
    if (!tile) {
      if (!fetch(index))
        return get_no_data_value();

      if (!(tile = cache_->source(index)))
        return get_no_data_value();
    }
  }

  // figure out what row and column we need from the array of data
  // NOTE: data is arranged from upper left to bottom right, so y is flipped

  // fractional pixel
  double u = (coord.first - lon) * (HGT_DIM - 1);
  double v = (1.0 - (coord.second - lat)) * (HGT_DIM - 1);

  return tile.get(u, v);
}

template <class coord_t> double sample::get(const coord_t& coord) {
  tile_data tile;
  return get(coord, tile);
}

template <class coords_t> std::vector<double> sample::get_all(const coords_t& coords) {
  std::vector<double> values;
  values.reserve(coords.size());

  tile_data tile;
  for (const auto& coord : coords) {
    values.emplace_back(get(coord, tile));
  }

  return values;
}

bool sample::store(const std::string& elev, const std::vector<char>& raw_data) {
  // data_source never changes so we do not lock it. it is set only in sample constructor
  auto fpath = cache_->data_source + elev;
  if (filesystem::exists(fpath))
    return true;

  auto data = cache_item_t::parse_hgt_name(elev);
  if (!data || data->second == format_t::UNKNOWN)
    return false;

  // cache size is always the same.
  if (data->first >= cache_->size())
    return false;

  // thread-safe by implementation
  if (!filesystem::save(fpath, raw_data))
    return false;

  std::lock_guard<std::mutex> _(cache_lck);
  return cache_->insert(data->first, fpath, data->second);
}

bool sample::fetch(uint16_t index) {
  if (url_.empty() || !remote_loader_)
    return false;

  auto elev = get_hgt_file_name(index);
  // we assume that tile_url already has valid format
  // and no additional '/' needed
  auto uri = baldr::make_single_point_url(url_, elev.substr(1), remote_path_);

  LOG_INFO("Start loading data from remote server address: " + uri);
  auto result = remote_loader_->get(uri);

  if (result.status_ != baldr::tile_getter_t::status_code_t::SUCCESS) {
    LOG_WARN("Fail to load data from remote server address: " + uri);
    return false;
  }

  if (!store(elev, result.bytes_)) {
    LOG_WARN("Fail to save data loaded from remote server address: " + uri);
    return false;
  }

  LOG_INFO("Data loaded from remote server address: " + uri);
  return true;
}

template <class coord_t> uint16_t sample::get_tile_index(const coord_t& coord) {
  auto lon = std::floor(coord.first);
  auto lat = std::floor(coord.second);
  return static_cast<uint16_t>(lat + 90) * 360 + static_cast<uint16_t>(lon + 180);
}

void sample::add_single_tile(const std::string& path) {
  std::lock_guard<std::mutex> _(cache_lck);
  cache_->insert(0, path, format_t::RAW);
}

// explicit instantiations for templated get
template double sample::get<std::pair<double, double>>(const std::pair<double, double>&);
template double sample::get<std::pair<float, float>>(const std::pair<float, float>&);
template double sample::get<midgard::PointLL>(const midgard::PointLL&);
template double sample::get<midgard::Point2>(const midgard::Point2&);

template std::vector<double>
sample::get_all<std::list<std::pair<double, double>>>(const std::list<std::pair<double, double>>&);
template std::vector<double> sample::get_all<std::vector<std::pair<double, double>>>(
    const std::vector<std::pair<double, double>>&);
template std::vector<double>
sample::get_all<std::list<std::pair<float, float>>>(const std::list<std::pair<float, float>>&);
template std::vector<double>
sample::get_all<std::vector<std::pair<float, float>>>(const std::vector<std::pair<float, float>>&);
template std::vector<double>
sample::get_all<std::list<midgard::PointLL>>(const std::list<midgard::PointLL>&);
template std::vector<double>
sample::get_all<std::vector<midgard::PointLL>>(const std::vector<midgard::PointLL>&);
template std::vector<double>
sample::get_all<std::list<midgard::Point2>>(const std::list<midgard::Point2>&);
template std::vector<double>
sample::get_all<std::vector<midgard::Point2>>(const std::vector<midgard::Point2>&);
template uint16_t
sample::get_tile_index<std::pair<double, double>>(const std::pair<double, double>& coord);
template uint16_t
sample::get_tile_index<std::pair<float, float>>(const std::pair<float, float>& coord);
template uint16_t sample::get_tile_index<midgard::PointLL>(const midgard::PointLL& coord);
template uint16_t sample::get_tile_index<midgard::Point2>(const midgard::Point2& coord);

std::string get_hgt_file_name(uint16_t index) {
  auto x = (index % 360) - 180;
  auto y = (index / 360) - 90;

  std::string name(y < 0 ? "/S" : "/N");
  y = std::abs(y);
  if (y < 10) {
    name.push_back('0');
  }
  name.append(std::to_string(y));
  name.append(name);

  name.append(x < 0 ? "W" : "E");
  x = std::abs(x);
  if (x < 100) {
    name.push_back('0');
  }
  if (x < 10) {
    name.push_back('0');
  }
  name.append(std::to_string(x));
  name.append(".hgt");

  return name;
}

// we don't need lock as this method is called in constructor only
void sample::cache_initialisation(const std::string& data_source) {
  cache_ = std::make_unique<cache_t>();
  cache_->data_source = data_source;

  // messy but needed
  while (cache_->data_source.size() &&
         cache_->data_source.back() == filesystem::path::preferred_separator) {
    cache_->data_source.pop_back();
  }

  // If data_source is empty, do not allocate/resize mapped cache.
  if (cache_->data_source.empty()) {
    LOG_DEBUG("No elevation data_source was provided");
    return;
  }
  cache_->cache.resize(TILE_COUNT);

  // check the directory for files that look like what we need
  auto files = filesystem::get_files(cache_->data_source);
  for (const auto& f : files) {
    // make sure its a valid index
    auto data = cache_item_t::parse_hgt_name(f);
    if (data && data->second != format_t::UNKNOWN) {
      if (!cache_->insert(data->first, f, data->second)) {
        LOG_WARN("Corrupt elevation data: " + f);
      }
    }
  }
}

double get_no_data_value() {
  return NO_DATA_VALUE;
}

} // namespace skadi
} // namespace valhalla
