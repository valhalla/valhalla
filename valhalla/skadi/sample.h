#ifndef __VALHALLA_SAMPLE_H__
#define __VALHALLA_SAMPLE_H__

#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "valhalla/baldr/tilegetter.h"

namespace valhalla {
namespace skadi {

struct cache_t;
class tile_data;

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
  sample(const boost::property_tree::ptree& pt);
  ~sample();

  /**
   * Get a single sample from the datasource
   * @param coord  the single posting at which to sample the datasource
   */
  template <class coord_t> double get(const coord_t& coord);

  /**
   * Get a single sample from a remote source
   * @param coord the single posting at which to sample the datasource
   */
  template <class coord_t> double get_from_remote(const coord_t& coord);

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
   * Get a single sample from the datasource
   * supplies the current tile - so if the same tile is requested in succession it does not have to
   * look up the tile in the cache.
   * @param coord  the single posting at which to sample the datasource
   * @param tile the tile pointer that may already contain a tile, output value
   * @return a single sample
   */
  template <class coord_t> double get(const coord_t& coord, tile_data& tile);

  /**
   * @return A tile index value from a coordinate
   */
  template <class coord_t> static uint16_t get_tile_index(const coord_t& coord);

  /**
   * @return The file name of a tile for a given index
   */
  static std::string get_hgt_file_name(uint16_t index);

  /**
   * Adds single tile in cache. Used only in tests
   * @param path path to the tile
   */
  void add_single_tile(const std::string& path);

  cache_t* cache_;
  friend cache_t;

private:
  /**
   * Get a single sample from cache
   * @param coord the single posting at which to sample the datasource
   */
  template <class coord_t> double get_from_cache(const coord_t& coord);
  bool store(const std::string& elev, const std::vector<char>& raw_data);

  std::string url_;
  std::unique_ptr<baldr::tile_getter_t> remote_loader_;
  std::unordered_set<std::string> st_;
  std::mutex st_lck_;
  std::uint32_t num_threads_{1};
};

} // namespace skadi
} // namespace valhalla

#endif //__VALHALLA_SAMPLE_H__
