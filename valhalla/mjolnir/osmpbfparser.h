#ifndef __OSMPBFPARSER__
#define __OSMPBFPARSER__

#include <robin_hood.h>

#include <string>

// extend the protobuf osmpbf namespace
namespace OSMPBF {

// Which callbacks you want to be called
enum Interest {
  NONE = 0x0,
  NODES = 0x01,
  WAYS = 0x02,
  RELATIONS = 0x04,
  CHANGESETS = 0x8,
  ALL = 0x15,
};

namespace Relation {
enum class MemberType : uint8_t {
  NODE = 0,
  WAY = 1,
  RELATION = 2,
};
}

// Represents the key/values of an object
using Tags = robin_hood::unordered_map<std::string, std::string>;

// Member of a relation
struct Member {
  Relation::MemberType member_type;
  uint64_t member_id;
  std::string role;
  Member() = delete;
  Member(const Relation::MemberType type, const uint64_t id, const std::string& role)
      : member_type(type), member_id(id), role(role) {
  }
  Member(Member&& other) noexcept
      : member_type(other.member_type), member_id(other.member_id), role(std::move(other.role)) {
  }
};

// pure virtual interface for consumers to implement
struct Callback {
  virtual ~Callback(){};
  virtual void
  node_callback(const uint64_t osmid, const double lng, const double lat, const Tags& tags) = 0;
  virtual void
  way_callback(const uint64_t osmid, const Tags& tags, const std::vector<uint64_t>& nodes) = 0;
  virtual void
  relation_callback(const uint64_t osmid, const Tags& tags, const std::vector<Member>& members) = 0;
  virtual void changeset_callback(const uint64_t changeset_id) = 0;
};

// parse the pbf file for the things you are interested in
void parse(const std::string& file, const Interest interest, Callback& callback);

} // namespace OSMPBF

#endif //__OSMPBFPARSER__
