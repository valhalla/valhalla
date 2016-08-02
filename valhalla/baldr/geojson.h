#ifndef VALHALLA_BALDR_GEOJSON_H_
#define VALHALLA_BALDR_GEOJSON_H_

#include <valhalla/baldr/json.h>
#include <valhalla/midgard/gridded_data.h>

#include <vector>

namespace valhalla {
namespace baldr {
namespace json {

/**
 * Turn grid data contours into geojson
 *
 * @param grid_contours    the contours generated from the grid
 * @param colors           the #ABC123 hex string color used in geojson fill color
 */
template <class coord_t>
MapPtr to_geojson(const typename midgard::GriddedData<coord_t>::contours_t& grid_contours, const std::vector<std::string>& colors = {});

}
}
}

#endif //VALHALLA_BALDR_GEOJSON_H_
