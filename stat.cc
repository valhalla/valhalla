// -*- mode: c++ -*-

#include <unordered_map>
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
  std::sort(ranges.begin(), ranges.end(), [](const Range& rhs, const Range& lhs) {
      return rhs.first < rhs.first;
    });

  auto merged_range = ranges.begin();
  for (auto range = std::next(ranges.begin()); range != ranges.end(); range++) {
    assert(merged_range < range);
    // [begin, merged_range] are guaranteed to be merged
    assert(merged_range->first <= range->first);  // since it's sorted

    if (range->first <= merged_range->second) {
      merged_range->second = std::max(merged_range->second, range->second);
    } else {
      std::advance(merged_range, 1);
      if (merged_range != range) {
        merged_range->first = range->first;
        merged_range->second = range->second;
      }
    }
  }

  ranges.erase(std::next(merged_range), ranges.end());
}


void aggregate_routes(sqlite3* db_handle,
                      std::unordered_map<baldr::GraphId, size_t>& complete_edges,
                      std::unordered_map<baldr::GraphId, std::vector<Range>>& incomplete_edges)
{
  sqlite3_stmt* stmt;
  std::string sql = "SELECT sequence_id, coordinate_index, graphid, graphtype FROM routes";
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
        MergeRanges(ranges);
        if (ranges.back().first == 0.f && ranges.back().second == 1.f) {
          complete_edges.emplace(edgeid, 1);
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


uint32_t get_edge_length(baldr::GraphReader& graphreader,
                         const baldr::GraphId& edgeid,
                         const baldr::GraphTile* tile = nullptr)
{
  if (!tile) {
    tile = graphreader.GetGraphTile(edgeid);
    if (!tile) return 0;
  }
  auto directededge = tile->directededge(edgeid);
  if (!directededge) return 0;
  return directededge->length();
}


double coverage(baldr::GraphReader& graphreader,
                const std::unordered_map<baldr::GraphId, size_t>& complete_edges,
                const std::unordered_map<baldr::GraphId, std::vector<Range>>& incomplete_edges)
{
  double total_length = 0;

  for (const auto& pair : complete_edges) {
    total_length += static_cast<double>(get_edge_length(graphreader, pair.first));
  }

  for (const auto& pair : incomplete_edges) {
    if (complete_edges.find(pair.first) == complete_edges.end()) {
      continue;
    }
    float partial_length = 0.f;
    auto edge_length = get_edge_length(graphreader, pair.first);
    for (const auto& range : pair.second) {
      assert(range.first <= range.second);
      partial_length += (range.second - range.first) * static_cast<float>(edge_length);
    }
    total_length += static_cast<double>(partial_length);
  }

  return total_length;
}


int main(int argc, char *argv[])
{
  if (argc < 4) {
    std::cerr << "usage: attacher ACTION CONFIG_FILENAME SQLITE3_FILENAME" << std::endl;
    return 1;
  }

  std::string action = argv[0],
     config_filename = argv[1],
   sqlite3_filename  = argv[2];

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
    std::unordered_map<baldr::GraphId, size_t> complete_edges;
    std::unordered_map<baldr::GraphId, std::vector<Range>> incomplete_edges;
    aggregate_routes(db_handle, complete_edges, incomplete_edges);

    auto total_length = coverage(graphreader, complete_edges, incomplete_edges);
    LOG_INFO("Total covered street length: " + std::to_string(total_length) + " meters");
  } else {
    LOG_ERROR("Unsupported action");
    return 3;
  }

  return 0;
}
