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
  // TODO: doxygen comment
  static void BuildAreas(const boost::property_tree::ptree& pt,
                         const std::string& ways_file,
                         const std::string& way_nodes_file,
                         OSMData& osmdata);

private:
  // TODO: doxygen comment
  static std::vector<std::vector<midgard::Point2d>> GenerateMedialAxis(const GEOSGeometry* polygon);
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_AREABUILDER_H