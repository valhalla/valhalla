#include "skadi/sample.h"

#include <cstddef>
#include <cmath>
#include <stdexcept>
#include <limits>
#include <list>
#include <fstream>

#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/pointll.h>

namespace {
  //srtmgl1 holds 1x1 degree tiles but oversamples the egde of the tile
  //by .5 seconds. that means that the center of pixel 0 is located at
  //the tiles lat,lon (which is important for bilinear filtering)
  constexpr size_t HGT_DIM = 3601;
  constexpr size_t HGT_PIXELS = HGT_DIM * HGT_DIM;
  constexpr double NO_DATA_VALUE = -32768;

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

  std::shared_ptr<int16_t> read_hgt(const std::string& file_name) {
    std::ifstream f(file_name);
    if(!f.is_open())
      return nullptr;
    //make some memory
    auto values = std::shared_ptr<int16_t>(new int16_t[HGT_PIXELS], [](int16_t* p){delete [] p;});
    //copy the data
    f.read(static_cast<char*>(static_cast<void*>(values.get())), HGT_PIXELS * (sizeof(int16_t)/sizeof(char)));
    return values;
  }

}

namespace valhalla {
namespace skadi {

  sample::sample(const std::string& data_source): data_source(data_source) {
  }

  template <class coord_t>
  double sample::get(const coord_t& coord) {
    //check the cache and load
    auto cache_id = (static_cast<int>(std::floor(coord.second) + 90) << 9) | static_cast<int>(std::floor(coord.first) + 180);
    auto cached_tile = cache.find(cache_id);
    if(cached_tile == cache.cend()) {
      //pull out a tile
      auto data = read_hgt("scripts/srtm/" + hgt_name(static_cast<int>(std::floor(coord.first)),
                                                      static_cast<int>(std::floor(coord.second))));
      //cache it
      cached_tile = cache.insert(std::make_pair(cache_id,
        srtm_tile_t{static_cast<int>(std::floor(coord.first)), static_cast<int>(std::floor(coord.second)), data})).first;
    }

    //TODO: bilinear
    const auto& tile = cached_tile->second;
    if(!tile.data)
      return NO_DATA_VALUE;
    auto c = coord.first - tile.lon;
    auto r = coord.second - tile.lat;
    return tile.data.get()[static_cast<size_t>(std::floor(r)) * HGT_DIM + static_cast<size_t>(std::floor(c))];
  }

  template <class coords_t>
  std::vector<double> sample::get_all(const coords_t& coords) {
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
  template double sample::get<std::pair<double, double> >(const std::pair<double, double>&);
  template double sample::get<std::pair<float, float> >(const std::pair<float, float>&);
  template double sample::get<midgard::PointLL>(const midgard::PointLL&);
  template double sample::get<midgard::Point2>(const midgard::Point2&);
  template std::vector<double> sample::get_all<std::list<std::pair<double, double> > >(const std::list<std::pair<double, double> >&);
  template std::vector<double> sample::get_all<std::vector<std::pair<double, double> > >(const std::vector<std::pair<double, double> >&);
  template std::vector<double> sample::get_all<std::list<std::pair<float, float> > >(const std::list<std::pair<float, float> >&);
  template std::vector<double> sample::get_all<std::vector<std::pair<float, float> > >(const std::vector<std::pair<float, float> >&);
  template std::vector<double> sample::get_all<std::list<midgard::PointLL> >(const std::list<midgard::PointLL>&);
  template std::vector<double> sample::get_all<std::vector<midgard::PointLL> >(const std::vector<midgard::PointLL>&);
  template std::vector<double> sample::get_all<std::list<midgard::Point2> >(const std::list<midgard::Point2>&);
  template std::vector<double> sample::get_all<std::vector<midgard::Point2> >(const std::vector<midgard::Point2>&);

}
}
