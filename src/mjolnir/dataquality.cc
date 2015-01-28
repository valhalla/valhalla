#include "mjolnir/dataquality.h"
#include <fstream>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Constructor
DataQuality::DataQuality()
    : not_thru_count_(0),
      internal_count_(0) {
}

// Add statistics
void DataQuality::AddStats(const GraphId& tileid,
                           const DirectedEdge& directededge) {
  if (directededge.not_thru()) {
    not_thru_count_++;
  }
  if (directededge.internal()) {
    internal_count_++;
  }
}

// Adds an issue.
void DataQuality::AddIssue(const DataIssueType issuetype, const GraphId& graphid,
            const uint64_t wayid1, const uint64_t wayid2) {
  if (issuetype == kDuplicateWays) {
    std::pair<uint64_t, uint64_t> wayids = std::make_pair(wayid1, wayid2);
    auto it = duplicateways_.find(wayids);
    if (it == duplicateways_.end()) {
      duplicateways_.emplace(wayids, 1);
    } else {
      it->second++;
    }
  } else if (issuetype == kUnconnectedLinkEdge) {
    unconnectedlinks_.insert(wayid1);
  } else if (issuetype == kIncompatibleLinkUse) {
    incompatiblelinkuse_.insert(wayid1);
  }
}

// Logs statistics and issues
void DataQuality::Log() {
  LOG_INFO("Not thru edgecount = " + std::to_string(not_thru_count_));
  LOG_INFO("Internal edgecount = " + std::to_string(internal_count_));

  // Log the duplicate ways - sort by number of duplicate edges

  uint32_t duplicates = 0;
  std::vector<DuplicateWay> dups;
  LOG_WARN("Duplicate Ways: count = " + std::to_string(duplicateways_.size()));
  for (const auto& dup : duplicateways_) {
    dups.emplace_back(DuplicateWay(dup.first.first, dup.first.second,
                                    dup.second));
    duplicates += dup.second;
  }
  LOG_WARN("Duplicate ways " + std::to_string(duplicateways_.size()) +
           " duplicate edges = " + std::to_string(duplicates));

  // Sort by edgecount and write to separate file
  std::ofstream dupfile;
  std::sort(dups.begin(), dups.end());
  dupfile.open("duplicateways.txt", std::ofstream::out | std::ofstream::trunc);
  dupfile << "WayID1   WayID2    DuplicateEdges" << std::endl;
  for (const auto& dupway : dups) {
    dupfile << dupway.wayid1 << "," << dupway.wayid2 << ","
            << dupway.edgecount << std::endl;
  }
  dupfile.close();

  // Log the unconnected link edges
  LOG_WARN("Link edges that are not connected. OSM Way Ids");
  for (const auto& wayid : unconnectedlinks_) {
    LOG_WARN(std::to_string(wayid));
  }

  // Log the links with incompatible use
  LOG_WARN("Link edges that have incompatible use. OSM Way Ids:");
  for (const auto& wayid : incompatiblelinkuse_) {
    LOG_WARN(std::to_string(wayid));
  }
}

}
}
