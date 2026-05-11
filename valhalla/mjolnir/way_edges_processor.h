#pragma once

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace valhalla {
namespace mjolnir {

struct way_edge {
  baldr::GraphId edgeid;
  uint32_t length;
  bool forward;
};

/**
 * Collects edges for ways in the graph.
 *
 * @param reader GraphReader to access graph data
 * @param filename Optional file path for additional edge information
 * @param access_mask the access mask to filter edges by
 * @return Map of way IDs to their associated edges and directions
 */
std::unordered_map<uint64_t, std::vector<way_edge>>
collect_way_edges(baldr::GraphReader& reader,
                  const std::string& filename = "",
                  uint32_t access_mask = baldr::kAutoAccess,
                  bool skip_backward = false);

/**
 * Collects edges for ways in the graph and sorts edges topologically per way.
 *
 * The vector of edges is sorted by direction, then by topological order (from the start of the OSM
 * way to the end for the forward edges, and from the end of the OSM way to the start for the backward
 * edges).
 *
 * Consider this example way:
 *
 * A-----B-----C-----D----E----F
 *
 * assuming all nodes A-F are graph nodes, the edges vector will look like this:
 *
 * AB, BC, CD, DE, EF, FE, FD, DC, CB, BA
 *
 * @param osm_filename File name of the OSM PBF file used to create the Valhalla tile set
 * @param reader GraphReader to access graph data
 * @param include_length if true, the CSV will contain an additional column per edge containing the
 * length in meters
 * @param filename Optional file path for additional edge information
 * @param access_mask the access mask to filter edges by
 * @return Map of way IDs to their associated edges and directions
 */
std::unordered_map<uint64_t, std::vector<way_edge>>
collect_way_edges_sorted(const std::string& osm_filename,
                         baldr::GraphReader& reader,
                         bool include_length,
                         const std::string& filename = "",
                         uint32_t access_mask = baldr::kAutoAccess);

} // namespace mjolnir
} // namespace valhalla
