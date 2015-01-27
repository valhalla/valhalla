#include "mjolnir/dataquality.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Constructor
DataQuality::DataQuality()
    : not_thru_count_(0) {
}

// Add statistics
void DataQuality::AddStats(const GraphId& tileid,
                           const DirectedEdge& directededge) {
  if (directededge.not_thru()) {
    not_thru_count_++;
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

  // Log the duplicate ways
  uint32_t duplicates = 0;
  LOG_WARN("Duplicate Ways: count = " + std::to_string(duplicateways_.size()));
  for (const auto& dup : duplicateways_) {
    LOG_WARN("Duplicate: OSM Way Ids = " + std::to_string(dup.first.first) +
        " and " + std::to_string(dup.first.second) + " Created " +
        std::to_string(dup.second) + " duplicate edges");
    duplicates += dup.second;
  }
  LOG_WARN("Duplicate edgecount = " + std::to_string(duplicates));

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
