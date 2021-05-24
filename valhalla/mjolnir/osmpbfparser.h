/*
 Copyright (c) 2012, Canal TP
 All rights reserved.
#include <cstdint>

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the Canal TP nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL CANAL TP BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// This is largely based off of: https://github.com/CanalTP/libosmpbfreader
// there have been some minor changes for our own purposes but its largely the same
#ifndef __OSMPBFPARSER__
#define __OSMPBFPARSER__

#include <fstream>
#include <string>

// this describes the low-level blob storage
#include <valhalla/proto/fileformat.pb.h>
// this describes the high-level OSM objects
#include <valhalla/proto/osmformat.pb.h>

// extend the protobuf osmpbf namespace
namespace OSMPBF {

// Which callbacks you want to be called
enum Interest {
  NONE = 0x0,
  NODES = 0x01,
  WAYS = 0x02,
  RELATIONS = 0x04,
  CHANGESETS = 0x8,
  ALL = 0x15
};

// Represents the key/values of an object
using Tags = std::unordered_map<std::string, std::string>;

// Member of a relation
struct Member {
  Relation::MemberType member_type;
  uint64_t member_id;
  std::string role;
  Member() = delete;
  Member(const Relation::MemberType type, const uint64_t id, const std::string& role);
  Member(Member&& other);
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

// the parser used to get data out of the osmpbf file
class Parser {
public:
  Parser() = delete;
  // parse the pbf file for the things you are interested in
  static void parse(std::ifstream& file, const Interest interest, Callback& callback);
  // clean up protobuf library level memory, this will make protobuf unusable after its called
  static void free();
};

} // namespace OSMPBF

#endif //__OSMPBFPARSER__
