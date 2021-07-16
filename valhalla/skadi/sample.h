#ifndef __VALHALLA_SAMPLE_H__
#define __VALHALLA_SAMPLE_H__

#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <valhalla/midgard/sequence.h>

#include <boost/optional.hpp>

namespace valhalla {
namespace skadi {

class sample {
public:
  // non-default-constructable and non-copyable
  sample() = delete;
  sample(sample&&) = delete;
  sample& operator=(sample&&) = delete;
  sample(const sample&) = delete;
  sample& operator=(const sample&) = delete;

  /**
   * Constructor
   * @param data_source  directory name of the datasource from which to sample
   */
  sample(const std::string& data_source);

  /**
   * Get a single sample from the datasource
   * @param coord  the single posting at which to sample the datasource
   */
  template <class coord_t> double get(const coord_t& coord);

  /**
   * Get multiple samples from the datasource
   * @param coords  the list of postings at which to sample the datasource
   */
  template <class coords_t> std::vector<double> get_all(const coords_t& coords);

  /**
   * @return the no data value for this data source
   */
  static double get_no_data_value();

protected:
  /**
   * @return A tile index value from a coordinate
   */
  template <class coord_t> static uint16_t get_tile_index(const coord_t& coord);

  /**
   * @return The file name of a tile for a given index
   */
  static std::string get_hgt_file_name(uint16_t index);

  enum class format_t { UNKNOWN = 0, RAW = 1, GZIP = 2 };

  class tile_data {
  private:
    sample* s;
    const int16_t* data;
    uint16_t index;
    bool reusable;

  public:
    tile_data(sample* s, uint16_t index, bool reusable, const int16_t* data);
    tile_data(const tile_data& other);
    ~tile_data();
    //      tile_data& operator= (tile_data &other);
    tile_data& operator=(tile_data&& other);

    inline operator bool() const {
      return data != nullptr;
    }
    double get(double u, double v);
  };

  /**
   * @param  index  the index of the data tile being requested
   * @return the array of data or nullptr if there was none
   */
  sample::tile_data source(uint16_t index);

  void increment_usages(uint16_t index);
  void decrement_usages(uint16_t index);

  class cache_item_t {
  private:
    format_t format;
    midgard::mem_map<char> data;
    int usages;
    const char* unpacked;

  public:
    cache_item_t() : format(format_t::UNKNOWN), usages(0), unpacked(nullptr) {
    }
    cache_item_t(cache_item_t&&) = default;
    ~cache_item_t() {
      free((void*)unpacked);
    }
    bool init(const std::string& file, format_t format);

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

    bool unpack(const char* unpacked);
    static boost::optional<std::pair<uint16_t, sample::format_t>>
    parse_hgt_name(const std::string& name);
  };

  // using memory maps
  std::vector<cache_item_t> cache;
  std::unordered_set<uint16_t> reusable;
  std::unordered_map<uint16_t, std::shared_future<tile_data>> pending_tiles;
  std::recursive_mutex mutex;
  std::string data_source;
};

} // namespace skadi
} // namespace valhalla

#endif //__VALHALLA_SAMPLE_H__
