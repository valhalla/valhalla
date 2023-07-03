#ifndef VALHALLA_BALDR_TURNLANES_H_
#define VALHALLA_BALDR_TURNLANES_H_

#include <cstdint>
#include <sstream>
#include <stdint.h>
#include <string>
#include <unordered_map>
#include <valhalla/midgard/util.h>
#include <vector>

namespace valhalla {
namespace baldr {

// Lane delimiter
constexpr char kLaneDelimiter = '|';

// Turn lane delimiter (within a lane)
constexpr char kTurnLaneDelimiter = ';';

// Turn lane masks
constexpr uint16_t kTurnLaneTypeCount = 11;
constexpr uint16_t kTurnLaneEmpty = 0u;
constexpr uint16_t kTurnLaneNone = 1u << 0u;
constexpr uint16_t kTurnLaneThrough = 1u << 1u;
constexpr uint16_t kTurnLaneSharpLeft = 1u << 2u;
constexpr uint16_t kTurnLaneLeft = 1u << 3u;
constexpr uint16_t kTurnLaneSlightLeft = 1u << 4u;
constexpr uint16_t kTurnLaneSlightRight = 1u << 5u;
constexpr uint16_t kTurnLaneRight = 1u << 6u;
constexpr uint16_t kTurnLaneSharpRight = 1u << 7u;
constexpr uint16_t kTurnLaneReverse = 1u << 8u;
constexpr uint16_t kTurnLaneMergeToLeft = 1u << 9u;
constexpr uint16_t kTurnLaneMergeToRight = 1u << 10u;

const static std::unordered_map<uint16_t, std::string> kTurnLaneNames =
    {{0, "|"},
     {kTurnLaneNone, "none"},
     {kTurnLaneThrough, "through"},
     {kTurnLaneSharpLeft, "sharp_left"},
     {kTurnLaneLeft, "left"},
     {kTurnLaneSlightLeft, "slight_left"},
     {kTurnLaneSlightRight, "slight_right"},
     {kTurnLaneRight, "right"},
     {kTurnLaneSharpRight, "sharp_right"},
     {kTurnLaneReverse, "reverse"},
     {kTurnLaneMergeToLeft, "merge_to_left"},
     {kTurnLaneMergeToRight, "merge_to_right"}};

const static std::unordered_map<std::string, uint16_t> kTurnLaneMasks =
    {{"|", kTurnLaneEmpty},
     {"none", kTurnLaneNone},
     {"through", kTurnLaneThrough},
     {"sharp_left", kTurnLaneSharpLeft},
     {"left", kTurnLaneLeft},
     {"slight_left", kTurnLaneSlightLeft},
     {"slight_right", kTurnLaneSlightRight},
     {"right", kTurnLaneRight},
     {"sharp_right", kTurnLaneSharpRight},
     {"reverse", kTurnLaneReverse},
     {"merge_to_left", kTurnLaneMergeToLeft},
     {"merge_to_right", kTurnLaneMergeToRight}};

/**
 * Holds turn lane information at the end of a directed edge. Turn lane text is stored
 * in the GraphTile text list and the offset is stored within the TurnLane structure.
 * The directed edge index within the tile is also stored so that turn lanes can be found
 * via the directed edge index.
 */
class TurnLanes {
public:
  /**
   * Constructor given arguments.
   * @param  idx  Directed edge index to which this trurn lane applies.
   * @param  text_offset  Offset to text in the names/text table.
   */
  TurnLanes(const uint32_t idx, const uint32_t text_offset)
      : edgeindex_(idx), spare_(0), text_offset_(text_offset) {
  }

  /**
   * Get the index of the directed edge this sign applies to.
   * @return  Returns the directed edge index (within the same tile
   *          as the sign information).
   */
  uint32_t edgeindex() const {
    return edgeindex_;
  }

  /**
   * Set the directed edge index.
   * @param  idx  Directed edge index.
   */
  void set_edgeindex(const uint32_t idx) {
    edgeindex_ = idx;
  }

  /**
   * Get the offset into the GraphTile text list for the text associated
   * with the sign.
   * @return  Returns the text offset.
   */
  uint32_t text_offset() const {
    return text_offset_;
  }

  /**
   * Convert a stored string into a vector of turn lane masks.
   * @param str  Stored string to convert into a vector of turn lanes masks.
   * @return  Returns a vector a turn lane masks.
   */
  static std::vector<uint16_t> lanemasks(const std::string& str) {
    // Convert the pipe separated string into lane masks
    std::vector<uint16_t> masks;
    std::stringstream ss(str);
    std::string item;
    while (std::getline(ss, item, kLaneDelimiter)) {
      masks.push_back(stoi(item));
    }
    return masks;
  }

  /**
   * Get a string with turn lanes - lanes are pipe delimited and turn lanes within
   * a lane are ';' delimited.
   * @param lanemasks List of lane masks.
   * @return Returns a string depicting the turn lanes.
   */
  static std::string turnlane_string(const std::vector<uint16_t>& lanemasks) {
    std::string turnlanes;
    if (lanemasks.size() == 0) {
      return turnlanes;
    }
    for (const auto& m : lanemasks) {
      if (turnlanes.empty() && m == 0) {
        turnlanes += kLaneDelimiter;
      } else {
        if (m > 0) {
          std::string tl;
          for (uint16_t i = 0; i < kTurnLaneTypeCount; ++i) {
            if (m & (1u << i)) {
              auto str = kTurnLaneNames.find(1u << i);
              if (str != kTurnLaneNames.end()) {
                if (!tl.empty()) {
                  tl += kTurnLaneDelimiter;
                }
                tl += str->second;
              }
            }
          }
          turnlanes += tl;
        }
        turnlanes += kLaneDelimiter;
      }
    }
    turnlanes.pop_back();
    return turnlanes;
  }

  /**
   * Get the pipe separated Valhalla turn lane string from the OSM turn lane.
   * @param  osmstr  OSM turn lane string.
   * @return Returns pipe separated turn lane string (stored in Valhalla tiles).
   */
  static std::string GetTurnLaneString(const std::string& osmstr) {
    std::string tl;
    std::stringstream ss(osmstr);
    std::string item;
    while (std::getline(ss, item, kLaneDelimiter)) {
      if (!tl.empty()) {
        tl += kLaneDelimiter;
      }
      std::stringstream ss2(item);
      std::string item2;
      uint16_t lanemask = 0;
      while (std::getline(ss2, item2, kTurnLaneDelimiter)) {
        const auto turnlane_mask = kTurnLaneMasks.find(item2);
        if (turnlane_mask != kTurnLaneMasks.end()) {
          lanemask |= turnlane_mask->second;
        }
      }

      // Append to the string
      tl += std::to_string(lanemask);
    }

    // Add an empty lane if the string ends with a delimiter
    if (osmstr.back() == kLaneDelimiter) {
      tl += kLaneDelimiter;
      tl += '0';
    }
    return tl;
  }

  /**
   * < operator for use in std::lower_bound.
   * @param  other  Other object to compare against.
   * @return  Returns true if this object is < the other object.
   */
  bool operator<(const TurnLanes& other) const {
    return edgeindex_ < other.edgeindex();
  }

protected:
  uint32_t edgeindex_ : 22; // kMaxTileEdgeCount in nodeinfo.h: 22 bits
  uint32_t spare_ : 10;

  uint32_t text_offset_;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_TURNLANES_H_
