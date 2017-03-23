#ifndef VALHALLA_BALDR_TRAFFICASSOCIATION_H_
#define VALHALLA_BALDR_TRAFFICASSOCIATION_H_

#include <valhalla/baldr/graphid.h>

namespace valhalla {
namespace baldr {

// 8 bits are used for percentages
constexpr float kPercentFactor    = 255.0f;
constexpr float kInvPercentFactor = 1.0f / 255.0f;

// Structure used to return information
struct TrafficSegment {
  GraphId segment_id_;
  float begin_percent_;
  float end_percent_;
  bool starts_segment_;
  bool ends_segment_;

  TrafficSegment(const GraphId& id, const float begin_pct,
                 const float end_pct, const bool starts, const bool ends)
      : segment_id_(id),
        begin_percent_(begin_pct),
        end_percent_(end_pct),
        starts_segment_(starts),
        ends_segment_(ends) {
  }
  json::MapPtr json() const {
    return json::map({
      {"segment_id", segment_id_.value},
      {"begin_percent", json::fp_t{begin_percent_, 3}},
      {"end_percent", json::fp_t{end_percent_, 3}},
      {"starts_segment", starts_segment_},
      {"ends_segment", ends_segment_},
    });
  }
};

/**
 * TrafficChunk describes many to one and many to many associations of
 * traffic segments to edges. They are also used for cases where a segment
 * is not within the same tile as the edge (at tile boundaries or if segment
 * is associated to an edge on a different hierarchy level). There can be
 * multiple chunks per edge.
 */
class TrafficChunk {
 public:
  /**
   * Constructor with arguments
   */
  TrafficChunk(const GraphId& segment_id, const float begin_percent,
               const float end_percent, const bool starts, const bool ends)
      : segment_id_(segment_id.value),
        begin_percent_((begin_percent  * kPercentFactor) + 0.5f),
        end_percent_((end_percent * kPercentFactor) + 0.5f),
        starts_segment_(starts),
        ends_segment_(ends) {
  }

  /**
   * Get the traffic segment Id.
   * @return  Returns the GraphId of the traffic segment for this chunk.
   */
  GraphId segment_id() const {
    return GraphId(segment_id_);
  }

  /**
   * Get the begin percent of this traffic segment along the edge.
   * @return  Returns percent (floating point)
   */
  float begin_percent() const {
    return begin_percent_ * kInvPercentFactor;
  }

  /**
   * Get the end  percent of this traffic segment along the edge.
   * @return  Returns percent (floating point)
   */
  float end_percent() const {
    return end_percent_ * kInvPercentFactor;
  }

  /**
   * Does this directed edge start at the beginning of the traffic segment.
   * @return  Returns true if this edge starts at the beginning of the
   *          traffic segment.
   */
  bool starts_segment() const {
    return starts_segment_;
  }

  /**
   * Does this directed edge ends at the end of the traffic segment.
   * @return  Returns true if this edge ends at the end of the
   *          traffic segment.
   */
  bool ends_segment() const {
    return ends_segment_;
  }

 private:
  uint64_t segment_id_     : 46; // Traffic segment Id
  uint64_t begin_percent_  : 8;  // Begin percent of the segment along the edge
  uint64_t end_percent_    : 8;  // End percent of the segment along the edge
  uint64_t starts_segment_ : 1;  // Edge starts this traffic segment
  uint64_t ends_segment_   : 1;  // Edge ends this traffic segment
};

/**
 * Information to associate a Valhalla directed edge to OSMLR traffic
 * segments. Includes the segment Id as well as information to localize
 * it to all or part of the traffic segment. TrafficAssociation structure
 * is used for 1:1 and 1:many cases (traffic segment to edge association)
 * AND the associated segment is in the same tile. A chunk is used if the
 * segment is in a different tile.
 */
class TrafficAssociation {
 public:
  /**
   * Default constructor.
   */
  TrafficAssociation()
     : id_(0),
       count_(0),
       starts_segment_(false),
       ends_segment_(false),
       chunk_(false) {
  }

  /**
   * Constructor with arguments.
   * @param  id      Traffic segment Id (within this tile)
   * @param  starts  Percentage along the segment at the beginning
   *                        of the edge.
   * @param  ends    Percentage along the segment at the end
   *                        of the edge.
   */
  TrafficAssociation(const uint32_t id, const bool starts,
                     const bool ends)
      : id_(id),
        count_(1),
        starts_segment_(starts),
        ends_segment_(ends),
        chunk_(false) {
  }

  /**
   * Constructor with arguments. Used when the edge associates to a traffic
   * chunk.
   * @param  chunk_count    Count of chunks associated to this edge.
   * @param  chunk_index    Index into the list of chunks.
   */
  TrafficAssociation(const size_t chunk_count, const size_t chunk_index)
    :  id_(static_cast<uint32_t>(chunk_index)),
       count_(static_cast<uint32_t>(chunk_count)),
       starts_segment_(false),
       ends_segment_(false),
       chunk_(true) {
  }

  /**
   * Get the traffic segment Id.
   * @return  Returns the segment Id within the current tile.
   */
  uint32_t id() const {
    return id_;
  }

  /**
   * Get the traffic segment count. This will be 0 if there is no associated
   * traffic segment.
   * @return  Returns the count (0 if no associated segment, 1 if a valid
   *          association within this tile, and >0 if associated to a chunk).
   */
  uint32_t count() const {
    return count_;
  }

  /**
   * Does this directed edge start at the beginning of the traffic segment.
   * @return  Returns true if this edge starts at the beginning of the
   *          traffic segment.
   */
  bool starts_segment() const {
    return starts_segment_;
  }

  /**
   * Does this directed edge ends at the end of the traffic segment.
   * @return  Returns true if this edge ends at the end of the
   *          traffic segment.
   */
  bool ends_segment() const {
    return ends_segment_;
  }

  /**
   * Does this edge associate to a traffic chunk (rather than a single
   * traffic segment).
   * @return  Returns true if this edge associates to a chunk.
   */
  bool chunk() const {
    return chunk_;
  }

  /**
   * Gets the chunk count and index.
   * @return  Returns a pair which includes the chunk count and index.
   */
  std::pair<uint32_t, uint32_t> GetChunkCountAndIndex() const {
    return std::make_pair(count_, id_);
  }

 protected:
  uint32_t id_              : 21; // Traffic segment Id (within same tile)
                                  // Chunk index if a chunk
  uint32_t count_           : 8;  // If a chunk
  uint32_t starts_segment_  : 1;  // Start of the traffic segment
  uint32_t ends_segment_    : 1;  // End of the traffic segment
  uint32_t chunk_           : 1;  // This is part of a chunk
};

}
}

#endif  // VALHALLA_BALDR_TRAFFICASSOCIATION_H_
