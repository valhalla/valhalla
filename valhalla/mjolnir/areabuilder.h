#ifndef VALHALLA_MJOLNIR_AREABUILDER_H
#define VALHALLA_MJOLNIR_AREABUILDER_H

#include <valhalla/midgard/pointll.h>
#include <valhalla/mjolnir/osmdata.h>

#include <boost/property_tree/ptree_fwd.hpp>

#include <string>

typedef struct GEOSGeom_t GEOSGeometry;

namespace valhalla {
namespace mjolnir {

class AreaBuilder {
public:
  /**
   * Builds routable traversals through pedestrian areas. For each area, assembles
   * its polygon(s) from the collected member ways, computes a medial-axis skeleton,
   * connects the area's entrances to it, and materialises the result as synthetic
   * footways so later stages turn them into routable edges.
   * @param  pt                             properties file
   * @param  ways_file                      where to store the ways so they are not in memory
   * @param  way_nodes_file                 where to store the nodes so they are not in memory
   * @param  osmdata                        OSM data
   *
   */
  static void BuildAreas(const boost::property_tree::ptree& pt,
                         const std::string& ways_file,
                         const std::string& way_nodes_file,
                         OSMData& osmdata);
};
} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_AREABUILDER_H