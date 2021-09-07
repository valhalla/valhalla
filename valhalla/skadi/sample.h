#ifndef __VALHALLA_SAMPLE_H__
#define __VALHALLA_SAMPLE_H__

#include <string>
#include <vector>

namespace valhalla {
namespace skadi {

struct cache_t;

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
  ~sample();

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

  /**
   * Adds single tile in cache. Used only in tests
   * @param path path to the tile
   */
  void add_single_tile(const std::string& path);

  cache_t* cache_;
  friend cache_t;
};

} // namespace skadi
} // namespace valhalla

#endif //__VALHALLA_SAMPLE_H__
