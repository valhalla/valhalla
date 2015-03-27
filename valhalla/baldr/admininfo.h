#ifndef VALHALLA_BALDR_ADMININFO_H_
#define VALHALLA_BALDR_ADMININFO_H_

#include <vector>
#include <string>
#include <ostream>
#include <iostream>

#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/util.h>
#include <valhalla/baldr/graphid.h>

using namespace valhalla::midgard;

namespace valhalla {
namespace baldr {

constexpr size_t kMaxNames = 255;
constexpr size_t kDstSize = 9;

/**
 * Admin information
 */
class AdminInfo {
 public:
  AdminInfo() = delete;
  AdminInfo(const AdminInfo& other) = delete;

  /**
   * Constructor
   *
   * @param pointer to a bit of memory that has the info for this admin
   */
  AdminInfo(char* ptr, const char* names_list, const size_t names_list_length);

  /**
   * Destructor
   *
   */
  virtual ~AdminInfo();

  // Returns the name count
  const uint32_t name_count() const;

  // Returns the admin id
  const uint32_t admin_id() const;

  // Returns the parent admin id
  const uint32_t parent_admin_id() const;

  // Returns the iso code index
  const uint32_t iso_code_index() const;

  // Set the iso code index.
  void set_iso_code_index(const uint32_t iso_code_index);

  // When does daylight saving time start?
  const char* StartDST() const;

  // When does daylight saving time end?
  const char* EndDST() const;

  // Set when daylight saving time starts.
  void SetStartDST(const std::string& start_dst);

  // Set when daylight saving time ends.
  void SetEndDST(const std::string& end_dst);

  // Returns the name index at the specified index.
  const uint32_t GetNameOffset(uint8_t index) const;

  /**
   * Convenience method to get the names for an admin
   * @return   Returns a list (vector) of names.
   */
  const std::vector<std::string> GetNames() const;

  // Operator EqualTo.
  bool operator ==(const AdminInfo& rhs) const;

  // Packed items: counts for names and level
  union PackedItem {
    struct Fields {
      uint32_t name_count           :8;  //name count
      uint32_t admin_id             :6;  //admin id
      uint32_t parent_admin_id      :6;  //parent admin id
      uint32_t spare                :12;
    } fields;
    uint32_t value;
  };

 protected:

  // Where we keep the statistics about how large the vectors below are
  PackedItem* item_;

  // DST start date and time.
  char start_dst_[kDstSize];

  // DST end date and time.
  char end_dst_[kDstSize];

  // ISO 3166-2 code.  Index into textlist
  uint32_t iso_code_index_;

  // List of name indexes
  uint32_t* name_offset_list_;

  // The list of names within the tile
  const char* names_list_;

  // The size of the names list
  const size_t names_list_length_;

};

}
}

#endif  // VALHALLA_BALDR_ADMININFO_H_
