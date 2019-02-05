#ifndef VALHALLA_MJOLNIR_GRAPHBUILDER_H
#define VALHALLA_MJOLNIR_GRAPHBUILDER_H

#include <boost/property_tree/ptree.hpp>
#include <cstdint>
#include <string>
#include <vector>

#include <valhalla/baldr/signinfo.h>

#include <valhalla/mjolnir/osmdata.h>
#include <valhalla/mjolnir/osmnode.h>
#include <valhalla/mjolnir/osmway.h>

namespace valhalla {
namespace mjolnir {

/**
 * Class used to construct temporary data used to build the initial graph.
 */
class GraphBuilder {
public:
  /**
   * Tell the builder to build the tiles from the provided datasource
   * and configs
   * @param  config                         properties file
   * @param  osmdata                        OSM data used to build the graph.
   * @param  ways_file                      where to store the ways so they are not in memory
   * @param  way_nodes_file                 where to store the nodes so they are not in memory
   * @param  nodes_file                     where to store node information so it isn't in memory
   * @param  edges_file                     where to store edge information so it isn't in memory
   * @param  complex_from_restriction_file  where to store the from complex restrictions so they are
   * not in memory
   * @param  complex_to_restriction_file    where to store the to complex restrictions so they are not
   * in memory
   */
  static void Build(const boost::property_tree::ptree& pt,
                    const OSMData& osmdata,
                    const std::string& ways_file,
                    const std::string& way_nodes_file,
                    const std::string& nodes_file,
                    const std::string& edges_file,
                    const std::string& complex_from_restriction_file,
                    const std::string& complex_to_restriction_file);

  static std::string GetRef(const std::string& way_ref, const std::string& relation_ref);

  static std::vector<baldr::SignInfo> CreateExitSignInfoList(const OSMNode& node,
                                                             const OSMWay& way,
                                                             const OSMData& osmdata,
                                                             bool fork,
                                                             bool forward);
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_GRAPHBUILDER_H
