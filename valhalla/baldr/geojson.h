#ifndef VALHALLA_BALDR_GEOJSON_H_
#define VALHALLA_BALDR_GEOJSON_H_

#include <valhalla/baldr/json.h>
#include <valhalla/midgard/gridded_data.h>

#include <vector>
#include <utility>

namespace valhalla {
namespace baldr {
namespace json {

/**
 * Turn grid data contours into geojson
 *
 * @param grid_contours    the contours generated from the grid
 */
using rgba_t = std::tuple<float,float,float,float>;
template <class coord_t>
MapPtr to_geojson(const typename midgard::GriddedData<coord_t>::contours_t& grid_contours, const std::vector<rgba_t>& colors = {});

}
}
}

#endif //VALHALLA_BALDR_GEOJSON_H_
