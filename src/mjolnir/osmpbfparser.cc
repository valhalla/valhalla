/*
 Copyright (c) 2012, Canal TP
 All rights reserved.

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
#include <cstdint>
#include <netinet/in.h>
#include <zlib.h>
#include <vector>
#include <unordered_map>

#include "mjolnir/osmpbfparser.h"
#include <valhalla/midgard/logging.h>

using namespace OSMPBF;

namespace {

// the maximum size of a blob header in bytes 64 kB
#define MAX_BLOB_HEADER_SIZE 65536
// the maximum size of an uncompressed blob in bytes 32 MB
#define MAX_UNCOMPRESSED_BLOB_SIZE 33554432

BlobHeader read_header(char* buffer, std::ifstream& file, bool& finished) {
  BlobHeader result;

  //read the first 4 bytes of the file, this is the size of the blob-header
  int32_t sz;
  if (!file.read(static_cast<char*>(static_cast<void*>(&sz)), 4)) {
    finished = true;
    return result;
  }

  //convert the size from network byte-order to host byte-order and check its sane
  sz = ntohl(sz);
  if (sz > MAX_BLOB_HEADER_SIZE)
    throw std::runtime_error("blob-header-size is bigger than allowed " + std::to_string(sz) + " > " + std::to_string(MAX_BLOB_HEADER_SIZE));

  //grab the blob header bytes
  file.read(buffer, sz);
  if (!file.good())
    throw std::runtime_error("unable to read blob-header from file");

  //turn the bytes into a protobuf object
  if (!result.ParseFromArray(buffer, sz))
    throw std::runtime_error("unable to parse blob header");

  finished = false;
  return result;
}

int32_t read_blob(char* buffer, char* unpack_buffer, std::ifstream& file, const BlobHeader & header) {
  Blob blob;

  //is the size of the following blob sane
  int32_t sz = header.datasize();
  if (sz > MAX_UNCOMPRESSED_BLOB_SIZE)
    throw std::runtime_error("blob-size is bigger than allowed");

  //pull out the bytes
  if (!file.read(buffer, sz))
    throw std::runtime_error("unable to read blob from file");

  //turn it into a protobuf object
  if (!blob.ParseFromArray(buffer, sz))
    throw std::runtime_error("unable to parse blob");

  //if the blob was uncompressed
  if (blob.has_raw()) {
    //check that raw_size is set correctly and move it to the final buffer
    sz = blob.raw().size();
    if (sz != blob.raw_size())
      LOG_WARN("blob reports wrong raw_size: " + std::to_string(blob.raw_size()) + " bytes");
    memcpy(unpack_buffer, buffer, sz);
    return sz;
  }//if the blob was zlib compressed
  else if (blob.has_zlib_data()) {
    sz = blob.zlib_data().size();
    z_stream z;
    z.next_in = (unsigned char*) blob.zlib_data().c_str();
    z.avail_in = sz;
    z.next_out = (unsigned char*) unpack_buffer;
    z.avail_out = blob.raw_size();
    z.zalloc = Z_NULL;
    z.zfree = Z_NULL;
    z.opaque = Z_NULL;
    if (inflateInit(&z) != Z_OK)
      throw std::runtime_error("failed to init zlib stream");
    if (inflate(&z, Z_FINISH) != Z_STREAM_END)
      throw std::runtime_error("failed to inflate zlib stream");
    if (inflateEnd(&z) != Z_OK)
      throw std::runtime_error("failed to deinit zlib stream");
    return z.total_out;
  }

  //if the blob was lzma compressed
  //if (blob.has_lzma_data())
  //  throw std::runtime_error("lzma-decompression is not supported");

  throw std::runtime_error("Unsupported blob data format");
}

template <class T>
OSMPBF::Tags get_tags(const T& object, const OSMPBF::PrimitiveBlock &primblock) {
  OSMPBF::Tags result(object.keys_size());
  for (int i = 0; i < object.keys_size(); ++i) {
    uint64_t key = object.keys(i);
    uint64_t val = object.vals(i);
    std::string key_string = primblock.stringtable().s(key);
    std::string val_string = primblock.stringtable().s(val);
    result[key_string] = val_string;
  }
  return result;
}

void parse_primitiveblock(char* unpack_buffer, int32_t sz, const Interest interest, Callback& callback) {
  //turn the blob bytes into a protobuf object
  PrimitiveBlock primblock;
  if (!primblock.ParseFromArray(unpack_buffer, sz))
    throw std::runtime_error("unable to parse primitive block");

  //for each primitive group
  for (int i = 0, l = primblock.primitivegroup_size(); i < l; i++) {
    const PrimitiveGroup& primitive_group = primblock.primitivegroup(i);

    //do the nodes
    if ((interest & NODES) == NODES) {
      // Simple Nodes
      for (int i = 0; i < primitive_group.nodes_size(); ++i) {
        const Node& n = primitive_group.nodes(i);

        double lon = 0.000000001 * (primblock.lon_offset() + (primblock.granularity() * n.lon()));
        double lat = 0.000000001 * (primblock.lat_offset() + (primblock.granularity() * n.lat()));
        callback.node_callback(n.id(), lon, lat, get_tags<Node>(n, primblock));
      }

      // Dense Nodes
      if (primitive_group.has_dense()) {
        const DenseNodes& dn = primitive_group.dense();
        uint64_t id = 0;
        double lon = 0;
        double lat = 0;

        int current_kv = 0;
        for (int i = 0; i < dn.id_size(); ++i) {
          id += dn.id(i);
          lon += 0.000000001 * (primblock.lon_offset() + (primblock.granularity() * dn.lon(i)));
          lat += 0.000000001 * (primblock.lat_offset() + (primblock.granularity() * dn.lat(i)));

          //can't exactly preallocate because you don't know how many there are
          Tags tags;
          while (current_kv < dn.keys_vals_size() && dn.keys_vals(current_kv) != 0) {
            uint64_t key = dn.keys_vals(current_kv);
            uint64_t val = dn.keys_vals(current_kv + 1);
            std::string key_string = primblock.stringtable().s(key);
            std::string val_string = primblock.stringtable().s(val);
            current_kv += 2;
            tags[key_string] = val_string;
          }
          ++current_kv;
          callback.node_callback(id, lon, lat, tags);
        }
      }
    }

    //do the ways
    if ((interest & WAYS) == WAYS) {
      for (int i = 0; i < primitive_group.ways_size(); ++i) {
        const Way& w = primitive_group.ways(i);

        uint64_t node = 0;
        std::vector<uint64_t> nodes;
        nodes.reserve(w.refs_size());
        for (int j = 0; j < w.refs_size(); ++j) {
          node += w.refs(j);
          nodes.push_back(node);
        }
        uint64_t id = w.id();
        callback.way_callback(id, get_tags<Way>(w, primblock), nodes);
      }
    }

    //do the relations
    if ((interest & RELATIONS) == RELATIONS) {
      for (int i = 0; i < primitive_group.relations_size(); ++i) {
        const Relation& rel = primitive_group.relations(i);
        uint64_t id = 0;
        std::vector<Member> members;
        members.reserve(rel.memids_size());
        for (int l = 0; l < rel.memids_size(); ++l) {
          id += rel.memids(l);
          members.emplace_back(rel.types(l), id, primblock.stringtable().s(rel.roles_sid(l)));
        }
        callback.relation_callback(rel.id(), get_tags<Relation>(rel, primblock), members);
      }
    }
  }
}

}

// extend the protobuf osmpbf namespace
namespace OSMPBF {

Member::Member(const Relation::MemberType type, const uint64_t id, const std::string& role): member_type(type), member_id(id), role(role) {
}

Member::Member(Member&& other): member_type(other.member_type), member_id(other.member_id), role(std::move(other.role)) {
}

void Parser::parse(const std::string& filename, const Interest interest, Callback& callback) {
  char* buffer = new char[MAX_UNCOMPRESSED_BLOB_SIZE];
  char* unpack_buffer = new char[MAX_UNCOMPRESSED_BLOB_SIZE];

  //check if the file is open
  std::ifstream file(filename, std::ios::binary);
  if (!file.is_open())
    throw std::runtime_error("Unable to open: " + filename);

  //while there is more to read
  while (!file.eof()) {
    //grab the blob header
    bool finished = false;
    BlobHeader header = read_header(buffer, file, finished);
    //if we didnt hit the end
    if (!finished) {
      //grab the blob that goes with the blob header
      int32_t sz = read_blob(buffer, unpack_buffer, file, header);
      //if its data parse it
      if (header.type() == "OSMData")
        parse_primitiveblock(unpack_buffer, sz, interest, callback);
      //if its something other than a header
      else if (header.type() != "OSMHeader")
        LOG_WARN("Unknown blob type: " + header.type());
    }
  }

  //done with protobuf and buffers
  delete [] buffer;
  delete [] unpack_buffer;
}

void Parser::free() {
  google::protobuf::ShutdownProtobufLibrary();
}

}
