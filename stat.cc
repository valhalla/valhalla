// -*- mode: c++ -*-

#include <unordered_map>
#include <sstream>
#include <cassert>

#include <sqlite3.h>

// boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

// valhalla
#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>

using namespace valhalla;


class SqliteQueryError: public std::runtime_error
{
  using std::runtime_error::runtime_error;
};


using Range = std::pair<float, float>;


// Merge a list of ranges, e.g. MergeRange({{0.1, 0.4}, {0.3, 0.5},
// {0.7, 0.8}}) will get you {{0.1, 0.5}, {0.7, 0.8}} in-place
void MergeRanges(std::vector<Range>& ranges)
{
  if (ranges.size() < 2) {
    return;
  }

  // Sort ranges by start value
  std::sort(ranges.begin(), ranges.end(), [](const Range& lhs, const Range& rhs) {
      return lhs.first < rhs.first;
    });

  auto merged_range = ranges.begin();
  for (auto range = std::next(ranges.begin()); range != ranges.end(); range++) {
    // Invariant: [begin, merged_range] are guaranteed to be merged
    assert(merged_range < range);
    assert(merged_range->first <= range->first);  // Make sure it's sorted

    if (merged_range->second >= range->first) { // Two ranges are intersected
      merged_range->second = std::max(merged_range->second, range->second);
    } else {  // Two ranges are disjoint
      std::advance(merged_range, 1);
      if (merged_range != range) {
        merged_range->first = range->first;
        merged_range->second = range->second;
      }
    }
  }

  ranges.erase(std::next(merged_range), ranges.end());
  if (ranges.back().first == 0.f && ranges.back().second == 1.f) {
    assert(ranges.size() == 1);
  }
}


// A complete edge is an edge that is fullly covered. The
// CompleteEdgeMap is to map each complete edge (GraphId) to the
// number of times (unsigned int) it's been covered.
using CompleteEdgeMap = std::unordered_map<baldr::GraphId, unsigned int>;

// A incomplete edge is an edge that is partially covered. The
// IncompleteEdgeMap is to map each incomplete edge (GraphId) to the
// ranges (std::vector<Range>) it's been covered.
using IncompleteEdgeMap = std::unordered_map<baldr::GraphId, std::vector<Range>>;


void aggregate_routes(sqlite3* db_handle,
                      CompleteEdgeMap& complete_edges,
                      IncompleteEdgeMap& incomplete_edges)
{
  sqlite3_stmt* stmt;
  std::string sql = "SELECT sequence_id, route_index, edgeid, source, target FROM routes";
  int ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, nullptr);
  if (SQLITE_OK != ret) {
    sqlite3_finalize(stmt);
    throw SqliteQueryError("Failed to prepare " + sql);
  }
  do {
    ret = sqlite3_step(stmt);
    if (SQLITE_ROW == ret) {
      // Don't need them for now
      // uint32_t sid = sqlite3_column_int(stmt, 0);
      // uint32_t route_idx = sqlite3_column_int(stmt, 1);
      baldr::GraphId edgeid(static_cast<uint64_t>(sqlite3_column_int64(stmt, 2)));
      assert(edgeid.Is_Valid());
      float source = static_cast<float>(sqlite3_column_double(stmt, 3));
      float target = static_cast<float>(sqlite3_column_double(stmt, 4));
      assert(0.f <= source && source <= target && target <= 1.f);

      if (source == 0.f && target == 1.f) {
        complete_edges[edgeid]++;
      } else {
        auto& ranges = incomplete_edges[edgeid];
        ranges.emplace_back(source, target);
        // If these ranges can cover the whole edge, then we move this
        // edge to the complete edge set. NOTE an edge is either in
        // the complete edge set or in the incomplete edge set
        MergeRanges(ranges);
        if (ranges.back().first == 0.f && ranges.back().second == 1.f) {
          complete_edges[edgeid]++;
          incomplete_edges.erase(edgeid);
        }
      }
    }
  } while (SQLITE_ROW == ret || SQLITE_BUSY == ret);

  if (SQLITE_DONE != ret) {
    sqlite3_finalize(stmt);
    throw SqliteQueryError("Expect SQLITE_DONE to return, but you got " + std::to_string(ret));
  }

  sqlite3_finalize(stmt);
}


// Group directed edges by tileid.
template <typename edge_map_t>
std::unordered_map<uint32_t, edge_map_t>
group_directededges_by_tileid(const edge_map_t& edges)
{
  std::unordered_map<uint32_t, edge_map_t> map;
  for (const auto& pair : edges) {
    map[pair.first.tileid()].insert(pair);
  }
  return map;
}


std::unordered_map<uint32_t, CompleteEdgeMap>
directed_to_bidirected_map(baldr::GraphReader& graphreader,
                           const std::unordered_map<uint32_t, CompleteEdgeMap>& complete_tiles,
                           const CompleteEdgeMap& complete_edges,
                           const IncompleteEdgeMap& incomplete_edges)
{
  std::unordered_map<uint32_t, CompleteEdgeMap> map;
  std::unordered_set<baldr::GraphId> visited;

  for (const auto& tile_pair : complete_tiles) {
    for (const auto& pair : tile_pair.second) {
      const auto edgeid = pair.first,
              oppedgeid = graphreader.GetOpposingEdgeId(edgeid);
      if (edgeid == oppedgeid) {
        std::ostringstream is;
        is << "Found an edge that its opposite edge is itself ";
        is << edgeid;
        LOG_ERROR(is.str());
      }
      if (graphreader.GetOpposingEdgeId(oppedgeid) != edgeid) {
        std::ostringstream is;
        is << "Found an edge " << edgeid;
        is << " has opposite edge to be " << oppedgeid << ",";
        is << " but which has another opposite edge to be " << graphreader.GetOpposingEdgeId(oppedgeid);
        LOG_ERROR(is.str());
      }

      visited.insert(oppedgeid);

      if (visited.insert(edgeid).second) {
        // Merge coverage counts
        unsigned int count = 0;
        auto it = complete_edges.find(edgeid);
        if (it != complete_edges.end()) {
          count += it->second;
        }
        it = complete_edges.find(oppedgeid);
        if (it != complete_edges.end()) {
          count += it->second;
        }
        if (count > 0) {
          map[edgeid.tileid()].emplace(edgeid, count);
        }
      }
    }
    if (graphreader.OverCommitted()) {
      graphreader.Clear();
    }
  }

  return map;
}


std::unordered_map<uint32_t, IncompleteEdgeMap>
directed_to_bidirected_map(baldr::GraphReader& graphreader,
                           const std::unordered_map<uint32_t, IncompleteEdgeMap>& incomplete_tiles,
                           const CompleteEdgeMap& complete_edges,
                           const IncompleteEdgeMap& incomplete_edges)
{
  std::unordered_map<uint32_t, IncompleteEdgeMap> map;
  std::unordered_set<baldr::GraphId> visited;

  for (const auto& tile_pair : incomplete_tiles) {
    for (const auto& pair : tile_pair.second) {
      const auto edgeid = pair.first,
              oppedgeid = graphreader.GetOpposingEdgeId(edgeid);
      // Log some impossible cases
      if (edgeid == oppedgeid) {
        std::ostringstream is;
        is << "Found an edge that its opposite edge is itself ";
        is << edgeid;
        LOG_ERROR(is.str());
      }
      if (graphreader.GetOpposingEdgeId(oppedgeid) != edgeid) {
        std::ostringstream is;
        is << "Found an edge " << edgeid;
        is << " has opposite edge to be " << oppedgeid << ",";
        is << " but which has another opposite edge to be " << graphreader.GetOpposingEdgeId(oppedgeid);
        LOG_ERROR(is.str());
      }

      visited.insert(oppedgeid);

      if (visited.insert(edgeid).second
          && complete_edges.find(edgeid) == complete_edges.end()
          && complete_edges.find(oppedgeid) == complete_edges.end()) {
        std::vector<Range> ranges;
        auto it = incomplete_edges.find(edgeid);
        if (it != incomplete_edges.end()) {
          ranges.insert(ranges.end(), it->second.begin(), it->second.end());
        }
        if (it != incomplete_edges.end()) {
          for (const auto& range : it->second) {
            ranges.emplace_back(1.f - range.second, 1.f - range.first);
          }
        }
        MergeRanges(ranges);
        if (!ranges.empty()) {
          map[edgeid.tileid()].emplace(edgeid, ranges);
        }
      }
    }
    if (graphreader.OverCommitted()) {
      graphreader.Clear();
    }
  }

  return map;
}


uint32_t get_edge_length(baldr::GraphReader& graphreader,
                         const baldr::GraphId& edgeid,
                         const baldr::GraphTile* tile = nullptr)
{
  if (!tile) {
    tile = graphreader.GetGraphTile(edgeid);
    if (!tile) return 0;
  }
  // If wrong tile is given, it's possible to throw exception here
  auto directededge = tile->directededge(edgeid);
  if (!directededge) return 0;
  return directededge->length();
}


double complete_coverage(baldr::GraphReader& graphreader,
                         const std::unordered_map<uint32_t, CompleteEdgeMap>& complete_tile_map)
{
  double total_length = 0.f;

  CompleteEdgeMap::size_type total_size = 0;
  for (const auto& pair : complete_tile_map) {
    total_size += pair.second.size();
  }
  CompleteEdgeMap::size_type processed = 0;

  for (const auto& tile_pair : complete_tile_map) {
    const baldr::GraphTile* tile = nullptr;
    for (const auto& pair : tile_pair.second) {
      if (!tile) {
        tile = graphreader.GetGraphTile(pair.first);
      }
      total_length += static_cast<double>(get_edge_length(graphreader, pair.first, tile));

      // Write progress to log
      processed++;
      if (processed % std::max(static_cast<unsigned int>(total_size / 100), 1u) == 0) {
        auto percent = static_cast<unsigned int>((processed / static_cast<float>(total_size)) * 100);
        LOG_INFO("Processed " + std::to_string(processed) + "/" + std::to_string(total_size)
                 + "(" + std::to_string(percent) + "%)"
                 + " complete edges");
      }
    }

    if (graphreader.OverCommitted()) {
      graphreader.Clear();
    }
  }

  return total_length;
}


double incomplete_coverage(baldr::GraphReader& graphreader,
                           const std::unordered_map<uint32_t, IncompleteEdgeMap>& incomplete_tile_map)
{
  double total_length = 0;

  IncompleteEdgeMap::size_type total_size = 0;
  for (const auto& pair : incomplete_tile_map) {
    total_size += pair.second.size();
  }
  IncompleteEdgeMap::size_type processed = 0;

  for (const auto& tile_pair : incomplete_tile_map) {
    const baldr::GraphTile* tile = nullptr;
    for (const auto& pair : tile_pair.second) {
      if (!tile) {
        tile = graphreader.GetGraphTile(pair.first);
      }
      float partial_length = 0.f;
      auto edge_length = get_edge_length(graphreader, pair.first, tile);
      for (const auto& range : pair.second) {
        assert(range.first <= range.second);
        partial_length += (range.second - range.first) * edge_length;
      }
      total_length += partial_length;

      // Write progress to log
      processed++;
      if (processed % std::max(static_cast<unsigned int>(total_size / 100), 1u) == 0) {
        auto percent = static_cast<unsigned int>((processed / static_cast<float>(total_size)) * 100);
        LOG_INFO("Processed " + std::to_string(processed) + "/" + std::to_string(total_size)
                 + "(" + std::to_string(percent) + "%)"
                 + " incomplete edges");
      }
    }

    if (graphreader.OverCommitted()) {
      graphreader.Clear();
    }
  }

  return total_length;
}


int main(int argc, char *argv[])
{
  if (argc < 4) {
    std::cerr << "usage: attacher ACTION CONFIG_FILENAME SQLITE3_FILENAME" << std::endl;
    return 1;
  }

  std::string action = argv[1],
     config_filename = argv[2],
   sqlite3_filename  = argv[3];

  sqlite3* db_handle;
  int ret = sqlite3_open_v2(sqlite3_filename.c_str(), &db_handle, SQLITE_OPEN_READONLY, nullptr);
  if (SQLITE_OK != ret) {
    LOG_ERROR("Failed to open sqlite3 database at " + sqlite3_filename);
    return 2;
  }

  boost::property_tree::ptree pt;
  boost::property_tree::read_json(config_filename.c_str(), pt);
  baldr::GraphReader graphreader(pt.get_child("mjolnir.hierarchy"));

  if (action == "coverage") {
    CompleteEdgeMap complete_edges;
    IncompleteEdgeMap incomplete_edges;

    LOG_INFO("Aggregating edges");
    aggregate_routes(db_handle, complete_edges, incomplete_edges);
    LOG_INFO("Aggregated " + std::to_string(complete_edges.size()) + " complete edges");
    LOG_INFO("Aggregated " + std::to_string(incomplete_edges.size()) + " incomplete edges");

    auto complete_tile_map = group_directededges_by_tileid(complete_edges);
    auto incomplete_tile_map = group_directededges_by_tileid(incomplete_edges);
    auto complete_total_length = complete_coverage(graphreader, complete_tile_map),
       incomplete_total_length = incomplete_coverage(graphreader, incomplete_tile_map),
                  total_length = complete_total_length + incomplete_total_length;
    std::cout << "Fully covered street length: " << std::to_string(complete_total_length) << "meters" << std::endl;
    std::cout << "Partially covered street length: " << std::to_string(incomplete_total_length) << "meters" << std::endl;
    std::cout << "Totally covered street length: " + std::to_string(total_length) + " meters" << std::endl;

    auto bid_complete_tile_map = directed_to_bidirected_map(graphreader, complete_tile_map,
                                                            complete_edges, incomplete_edges);
    auto bid_incomplete_tile_map = directed_to_bidirected_map(graphreader, incomplete_tile_map,
                                                              complete_edges, incomplete_edges);
    auto bid_complete_total_length = complete_coverage(graphreader, bid_complete_tile_map),
       bid_incomplete_total_length = incomplete_coverage(graphreader, bid_incomplete_tile_map),
                  bid_total_length = bid_complete_total_length + bid_incomplete_total_length;
    std::cout << "Fully covered street length: " << std::to_string(bid_complete_total_length) << "meters" << std::endl;
    std::cout << "Partially covered street length: " << std::to_string(bid_incomplete_total_length) << "meters" << std::endl;
    std::cout << "Totally covered street length (bidirectionally): " + std::to_string(bid_total_length) + " meters" << std::endl;
  } else {
    LOG_ERROR("Unsupported action");
    return 3;
  }

  return 0;
}
