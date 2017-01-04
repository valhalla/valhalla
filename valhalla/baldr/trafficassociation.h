#ifndef VALHALLA_BALDR_TRAFFICASSOCIATION_H_
#define VALHALLA_BALDR_TRAFFICASSOCIATION_H_

#include <valhalla/baldr/graphid.h>

namespace valhalla {
namespace baldr {

constexpr float kPercentFactor    = 127.0f;
constexpr float kInvPercentFactor = 1.0f / 127.0f;

// TODO - need to redesign the traffic chunks...

// These are used to indicate
constexpr uint64_t kChunkCountMask  = 0x00000fff00000000;
constexpr uint64_t kChunkCountShift = 32;
constexpr uint64_t kChunkIndexMask  = 0x00000000ffffffff;


struct TrafficChunk {
  uint64_t segment_id_              : 46;  // Internal route type
  uint64_t begin_percent_           : 7;   // Weight.
  uint64_t spare_                   : 13;
};

/**
 * Information to associate a Valhalla directed edge to OSMLR traffic
 * segments. Includes the segment Id as well as information to localize
 * it to all or part of the traffic segment.
 */
class TrafficAssociation {
 public:
  /**
   * Default constructor.
   */
  TrafficAssociation();

  /**
   * Constructor with arguments.
   * @param  segment_id     Traffic segment Id.
   * @param  begin_percent  Percentage along the segment at the beginning
   *                        of the edge.
   * @param  end_percent    Percentage along the segment at the end
   *                        of the edge.
   */
  TrafficAssociation(const GraphId& segment_id, const float begin_percent,
                     const float end_percent);

  /**
   * Constructor with arguments. Used when the edge associates to a traffic
   * chunk.
   * @param  begin_percent  Percentage along the segment at the beginning
   *                        of the edge.
   * @param  end_percent    Percentage along the segment at the end
   *                        of the edge.
   * @param  chunk_count    Count of chunks associated to this edge.
   * @param  chunk_index    Index into the list of chunks.
   */
  TrafficAssociation(const float begin_percent, const float end_percent,
                     const uint32_t chunk_count, const uint32_t chunk_index);

  /**
   * Get the traffic segment Id.
   * @return  Returns the OSMLR traffic segment Id.
   */
  GraphId segment_id() const;

  /**
   * Set the traffic segment Id.
   * @param  id  Segment Id.
   */
  void set_segment_id(const GraphId& id);

  /**
   * Get the percentage along the traffic segment for the beginning of the
   * directed edge.
   * @return  Returns the percent along the traffic segment (0-1).
   */
  float begin_percent() const;

  /**
   * Get the percentage along the traffic segment for the end of the
   * directed edge.
   * @return  Returns the percent along the traffic segment (0-1).
   */
  float end_percent() const;

  /**
   * Does this directed edge start at the beginning of the traffic segment.
   * @return  Returns true if this edge starts at the beginning of the
   *          traffic segment.
   */
  bool starts_segment() const;

  /**
   * Does this directed edge ends at the end of the traffic segment.
   * @return  Returns true if this edge ends at the end of the
   *          traffic segment.
   */
  bool ends_segment() const;

  /**
   * Does this edge associate to a traffic chunk (rather than a single
   * traffic segment).
   * @return  Returns true if this edge associates to a chunk.
   */
  bool chunk() const;

  /**
   * Gets the chunk count and index.
   * @return  Returns a pair which includes the chunk count and index.
   */
  std::pair<uint64_t, uint64_t> GetChunkCountAndIndex() const;

 protected:

  // NOTE: If this association is part of a chunk (chunk_ flag is set), then
  // the segment_id points to an index and count of chunks.
  uint64_t segment_id_              : 46;  // Traffic segment Id
  uint64_t begin_percent_           : 7;   // Begin percent
  uint64_t end_percent_             : 7;   // End percent
  uint64_t starts_segment_          : 1;   // Start of the traffic segment
  uint64_t ends_segment_            : 1;   // End of the traffic segment
  uint64_t chunk_                   : 1;   // This is part of a chunk
  uint64_t spare_                   : 1;
};

}
}

#endif  // VALHALLA_BALDR_TRAFFICASSOCIATION_H_
