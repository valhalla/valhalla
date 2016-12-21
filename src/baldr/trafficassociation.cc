#include <string.h>
#include <valhalla/midgard/logging.h>
#include "baldr/trafficassociation.h"

namespace valhalla {
namespace baldr {

/**
 * Default constructor.
 */
TrafficAssociation::TrafficAssociation()
    : segment_id_(0),
      begin_percent_(0),
      end_percent_(0),
      starts_segment_(false),
      ends_segment_(false),
      chunk_(false),
      spare_(0) {
}

// Constructor with arguments.
TrafficAssociation::TrafficAssociation(const GraphId& segment_id,
                     const float begin_percent,
                     const float end_percent)
    : segment_id_(segment_id.value),
      chunk_(false),
      spare_(0) {
  starts_segment_ = false;
  if (begin_percent <= 0.0f) {
    begin_percent_ = 0;
    starts_segment_ = true;
  } else if (begin_percent >= 1.0f) {
    LOG_WARN("TrafficAssociation: begin percent must be < 1.0");
    begin_percent_ = static_cast<uint32_t>(kPercentFactor);
  } else {
    begin_percent_ = static_cast<uint32_t>(begin_percent * kPercentFactor);
  }

  ends_segment_ = false;
  if (end_percent <= 0.0f) {
    LOG_WARN("TrafficAssociation: end percent must be > 0.0");
    end_percent_ = 0;
  } else if (end_percent >= 1.0f) {
    end_percent_  = static_cast<uint32_t>(kPercentFactor);
    ends_segment_ = true;
  } else {
    end_percent_ = static_cast<uint32_t>(end_percent * kPercentFactor);
  }
}

// Constructor with arguments. Validate input is within bounds of the
// name offset fields.
// Constructor with arguments
TrafficAssociation::TrafficAssociation(
                     const float begin_percent, const float end_percent,
                     const uint32_t chunk_count, const uint32_t chunk_index)
      : chunk_(true),
        spare_(0) {
  // TODO - set chunk count and chunk index
  starts_segment_ = false;
  if (begin_percent <= 0.0f) {
    begin_percent_ = 0;
    starts_segment_ = true;
  } else if (begin_percent >= 1.0f) {
    LOG_WARN("TrafficAssociation: begin percent must be <= 1.0");
    begin_percent_ = static_cast<uint32_t>(kPercentFactor);
  } else {
    begin_percent_ = static_cast<uint32_t>(begin_percent * kPercentFactor);
  }

  ends_segment_ = false;
  if (begin_percent <= 0.0f) {
    LOG_WARN("TrafficAssociation: end percent must be >= 0.0");
    end_percent_ = 0;
  } else if (begin_percent >= 1.0f) {
    end_percent_ = static_cast<uint32_t>(kPercentFactor);
    ends_segment_ = true;
  } else {
    end_percent_ = static_cast<uint32_t>(end_percent * kPercentFactor);
  }

  segment_id_ = 0; // TODO
}

// Get the traffic segment Id.
GraphId TrafficAssociation::segment_id() const {
  return GraphId(segment_id_);
}

// Set the traffic segment Id.
void TrafficAssociation::set_segment_id(const GraphId& id) {
  segment_id_ = id.value;
}


// Get the percentage along the traffic segment for the beginning of the
// directed edge.
float TrafficAssociation::begin_percent() const {
  return static_cast<float>(begin_percent_ * kInvPercentFactor);
}

// Get the percentage along the traffic segment for the end of the
// directed edge.
float TrafficAssociation::end_percent() const {
  return static_cast<float>(end_percent_ * kInvPercentFactor);
}

// Does this directed edge start at the beginning of the traffic segment.
bool TrafficAssociation::starts_segment() const {
 return starts_segment_;
}

// Does this directed edge ends at the end of the traffic segment.
bool TrafficAssociation::ends_segment() const {
  return ends_segment_;
}

// Does this edge associate to a traffic chunk (rather than a single
// traffic segment).
bool TrafficAssociation::chunk() const {
  return chunk_;
}

// Gets the chunk count and index.
std::pair<uint64_t, uint64_t> TrafficAssociation::GetChunkCountAndIndex() const {
  uint64_t v = segment_id_;
  return std::make_pair((v & kChunkCountMask) >> kChunkCountShift,
                        (v & kChunkIndexMask));
}

}
}
