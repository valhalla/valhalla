// -*- mode: c++ -*-
#pragma once

#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/traffictile.h"
#include "config.h"
#include "mjolnir/graphtilebuilder.h"

#include <cmath>
#include <fstream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <boost/algorithm/string/replace.hpp>
#include <boost/property_tree/ptree.hpp>

namespace test {

// Return a random number inside [0, 1)
inline float rand01(std::mt19937& gen) {
  std::uniform_real_distribution<float> dis(0, 1);
  return dis(gen);
}

std::string load_binary_file(const std::string& filename);

MATCHER_P2(IsBetween,
           a,
           b,
           std::string(negation ? "isn't" : "is") + " between " + ::testing::PrintToString(a) +
               " and " + ::testing::PrintToString(b)) {
  return a <= arg && arg <= b;
}

template <typename pbf_message_t> bool pbf_equals(const pbf_message_t& a, const pbf_message_t& b) {
  return a.SerializeAsString() == b.SerializeAsString();
}

boost::property_tree::ptree json_to_pt(const std::string& json);

boost::property_tree::ptree
make_config(const std::string& path_prefix,
            const std::unordered_map<std::string, std::string>& overrides = {},
            const std::unordered_set<std::string>& removes = {});

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

#ifdef DATA_TOOLS
using HistoricalTrafficCustomize =
    std::function<boost::optional<std::array<float, kBucketsPerWeek>>(DirectedEdge&)>;
void customize_historical_traffic(const boost::property_tree::ptree& config,
                                  const HistoricalTrafficCustomize& cb);

using EdgesCustomize = std::function<void(const GraphId&, DirectedEdge&)>;
void customize_edges(const boost::property_tree::ptree& config, const EdgesCustomize& setter_cb);
#endif

// provides us an easy way to mock having incident tiles, each test can override the tile in question
// a bit more work is needed if we want to do it for more than one tile at a time
struct IncidentsReader : public valhalla::baldr::GraphReader {
  IncidentsReader(const boost::property_tree::ptree& pt) : valhalla::baldr::GraphReader(pt) {
    tile_extract_.reset(new valhalla::baldr::GraphReader::tile_extract_t(pt));
    enable_incidents_ = true;
  }
  virtual std::shared_ptr<const valhalla::IncidentsTile>
  GetIncidentTile(const valhalla::baldr::GraphId& tile_id) const override {
    auto i = incidents.find(tile_id.Tile_Base());
    if (i == incidents.cend())
      return {};
    return std::shared_ptr<valhalla::IncidentsTile>(&i->second, [](valhalla::IncidentsTile*) {});
  }
  void add_incident(const valhalla::baldr::GraphId& id,
                    valhalla::IncidentsTile::Location&& _incident_location,
                    uint64_t incident_id,
                    const std::string& incident_description = "") {

    // Grab the incidents-tile we need to add to
    valhalla::IncidentsTile& incidents_tile = incidents[id.Tile_Base()];

    // See if we have this incident metadata already
    int metadata_index =
        std::find_if(incidents_tile.metadata().begin(), incidents_tile.metadata().end(),
                     [incident_id](const valhalla::IncidentsTile::Metadata& m) {
                       return incident_id == m.id();
                     }) -
        incidents_tile.metadata().begin();

    // We're not yet tracking this incident, so add it new
    if (metadata_index == incidents_tile.metadata_size()) {
      auto* metadata = incidents_tile.mutable_metadata()->Add();
      metadata->set_id(incident_id);
      metadata->set_description(incident_description);
    }

    // Finally, add the relation from edge to the incident metadata
    auto* locations = incidents_tile.mutable_locations()->Add();
    locations->Swap(&_incident_location);
    locations->set_metadata_index(metadata_index);
  }
  void sort_incidents() {
    for (auto& kv : incidents) {
      std::sort(kv.second.mutable_locations()->begin(), kv.second.mutable_locations()->end(),
                [](const valhalla::IncidentsTile::Location& a,
                   const valhalla::IncidentsTile::Location& b) {
                  if (a.edge_index() == b.edge_index()) {
                    if (a.start_offset() == b.start_offset()) {
                      if (a.end_offset() == b.end_offset()) {
                        return a.metadata_index() < b.metadata_index();
                      }
                      return a.end_offset() < b.end_offset();
                    }
                    return a.start_offset() < b.start_offset();
                  }
                  return a.edge_index() < b.edge_index();
                });
    }
  }
  mutable std::unordered_map<valhalla::baldr::GraphId, valhalla::IncidentsTile> incidents;
};

} // namespace test
