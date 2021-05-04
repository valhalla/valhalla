#pragma once

#include "baldr/traffictile.h"
#include "baldr/predictedspeeds.h"
#include <boost/property_tree/ptree.hpp>

namespace valhalla {
namespace baldr {
class GraphReader;
class DirectedEdge;
struct GraphId;
}
}

namespace test {

/**
 * Generate a new GraphReader that doesn't re-use a previously
 * statically initizalized tile_extract member variable.
 *
 * Useful if you need to reload a tile extract within the same
 * process
 */
std::shared_ptr<valhalla::baldr::GraphReader>
make_clean_graphreader(const boost::property_tree::ptree& mjolnir_conf);

/*************************************************************/
// Creates an empty traffic file
//
// To actually customize the traffic data, use `customize_live_traffic_data`
//
/*************************************************************/
void build_live_traffic_data(const boost::property_tree::ptree& config,
                             uint32_t traffic_tile_version = valhalla::baldr::TRAFFIC_TILE_VERSION);

/*************************************************************/
// Helper function for customizing traffic data in unit-tests
//
// `setter_cb` is a callback that can modify traffic for each edge
// when building traffic data
/*************************************************************/
using LiveTrafficCustomize = std::function<void(valhalla::baldr::GraphReader&,
                                                valhalla::baldr::TrafficTile&,
                                                int,
                                                valhalla::baldr::TrafficSpeed*)>;

void customize_live_traffic_data(const boost::property_tree::ptree& config,
                                 const LiveTrafficCustomize& setter_cb);

using HistoricalTrafficCustomize =
std::function<
    boost::optional<std::array<float, valhalla::baldr::kBucketsPerWeek>>(
            valhalla::baldr::DirectedEdge&)>;
void customize_historical_traffic(const boost::property_tree::ptree& config,
                                  const HistoricalTrafficCustomize& cb);

using EdgesCustomize = std::function<void(const valhalla::baldr::GraphId&,
                                          valhalla::baldr::DirectedEdge&)>;
void customize_edges(const boost::property_tree::ptree& config, const EdgesCustomize& setter_cb);

}
