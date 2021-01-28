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
#ifdef _MSC_VER
#include <winsock2.h> // ntohl
#else
#include <netinet/in.h>
#endif
#include <unordered_map>
#include <vector>
#include <zlib.h>

#include "midgard/logging.h"
#include "mjolnir/osmpbfparser.h"

using namespace OSMPBF;

namespace {

// the maximum size of a blob header in bytes 64 kB
#define MAX_BLOB_HEADER_SIZE 65536
// the maximum size of an uncompressed blob in bytes 32 MB
#define MAX_UNCOMPRESSED_BLOB_SIZE 33554432

BlobHeader read_header(char* buffer, std::ifstream& file, bool& finished) {
  BlobHeader result;

  // read the first 4 bytes of the file, this is the size of the blob-header
  int32_t sz;
  if (!file.read(static_cast<char*>(static_cast<void*>(&sz)), 4)) {
    finished = true;
    return result;
  }

  // convert the size from network byte-order to host byte-order and check its sane
  sz = ntohl(sz);
  if (sz > MAX_BLOB_HEADER_SIZE) {
    throw std::runtime_error("blob-header-size is bigger than allowed " + std::to_string(sz) + " > " +
                             std::to_string(MAX_BLOB_HEADER_SIZE));
  }

  // grab the blob header bytes
  file.read(buffer, sz);
  if (!file.good()) {
    throw std::runtime_error("unable to read blob-header from file");
  }

  // turn the bytes into a protobuf object
  if (!result.ParseFromArray(buffer, sz)) {
    throw std::runtime_error("unable to parse blob header");
  }

  finished = false;
  return result;
}

int32_t read_blob(char* buffer, char* unpack_buffer, std::ifstream& file, const BlobHeader& header) {
  Blob blob;

  // is the size of the following blob sane
  int32_t sz = header.datasize();
  if (sz > MAX_UNCOMPRESSED_BLOB_SIZE) {
    throw std::runtime_error("blob-size is bigger than allowed");
  }

  // pull out the bytes
  if (!file.read(buffer, sz)) {
    throw std::runtime_error("unable to read blob from file");
  }

  // turn it into a protobuf object
  if (!blob.ParseFromArray(buffer, sz)) {
    throw std::runtime_error("unable to parse blob");
  }

  // if the blob was uncompressed
  if (blob.has_raw()) {
    // check that raw_size is set correctly and move it to the final buffer
    sz = blob.raw().size();
    if (sz != blob.raw_size()) {
      LOG_WARN("blob reports wrong raw_size: " + std::to_string(blob.raw_size()) + " bytes");
    }
    memcpy(unpack_buffer, buffer, sz);
    return sz;
  } // if the blob was zlib compressed
  else if (blob.has_zlib_data()) {
    sz = blob.zlib_data().size();
    z_stream z;
    z.next_in = (unsigned char*)blob.zlib_data().c_str();
    z.avail_in = sz;
    z.next_out = (unsigned char*)unpack_buffer;
    z.avail_out = blob.raw_size();
    z.zalloc = Z_NULL;
    z.zfree = Z_NULL;
    z.opaque = Z_NULL;
    if (inflateInit(&z) != Z_OK) {
      throw std::runtime_error("failed to init zlib stream");
    }
    if (inflate(&z, Z_FINISH) != Z_STREAM_END) {
      throw std::runtime_error("failed to inflate zlib stream");
    }
    if (inflateEnd(&z) != Z_OK) {
      throw std::runtime_error("failed to deinit zlib stream");
    }
    return z.total_out;
  }

  // if the blob was lzma compressed
  // if (blob.has_lzma_data())
  //  throw std::runtime_error("lzma-decompression is not supported");

  throw std::runtime_error("Unsupported blob data format");
}

template <class T> OSMPBF::Tags get_tags(const T& object, const OSMPBF::PrimitiveBlock& primblock) {
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

void parse_primitive_block(char* unpack_buffer,
                           int32_t sz,
                           const Interest interest,
                           Callback& callback) {
  // turn the blob bytes into a protobuf object
  PrimitiveBlock primblock;
  if (!primblock.ParseFromArray(unpack_buffer, sz)) {
    throw std::runtime_error("unable to parse primitive block");
  }

  // for each primitive group
  for (const auto& primitive_group : primblock.primitivegroup()) {

    // do the nodes
    if ((interest & NODES) == NODES) {
      // Simple Nodes
      for (const auto& node : primitive_group.nodes()) {
        double lon = 0.000000001 * (primblock.lon_offset() + (primblock.granularity() * node.lon()));
        double lat = 0.000000001 * (primblock.lat_offset() + (primblock.granularity() * node.lat()));
        callback.node_callback(node.id(), lon, lat, get_tags<Node>(node, primblock));
        if (node.has_info() && node.info().has_changeset() && (interest & CHANGESETS) == CHANGESETS) {
          callback.changeset_callback(node.info().changeset());
        }
      }

      // Dense Nodes
      if (primitive_group.has_dense()) {
        const DenseNodes& dense_nodes = primitive_group.dense();
        uint64_t id = 0;
        double lon = 0;
        double lat = 0;

        int current_kv = 0;
        for (int i = 0; i < dense_nodes.id_size(); ++i) {
          id += dense_nodes.id(i);
          lon +=
              0.000000001 * (primblock.lon_offset() + (primblock.granularity() * dense_nodes.lon(i)));
          lat +=
              0.000000001 * (primblock.lat_offset() + (primblock.granularity() * dense_nodes.lat(i)));

          // can't exactly preallocate because you don't know how many there are
          Tags tags;
          while (current_kv < dense_nodes.keys_vals_size() &&
                 dense_nodes.keys_vals(current_kv) != 0) {
            uint64_t key = dense_nodes.keys_vals(current_kv);
            uint64_t val = dense_nodes.keys_vals(current_kv + 1);
            std::string key_string = primblock.stringtable().s(key);
            std::string val_string = primblock.stringtable().s(val);
            current_kv += 2;
            tags[key_string] = val_string;
          }
          ++current_kv;
          callback.node_callback(id, lon, lat, tags);
        }
        if (dense_nodes.has_denseinfo() && (interest & CHANGESETS) == CHANGESETS) {
          uint64_t changeset = 0;
          for (auto changeset_id_offset : dense_nodes.denseinfo().changeset()) {
            callback.changeset_callback(changeset += changeset_id_offset);
          }
        }
      }
    }

    // do the ways
    if ((interest & WAYS) == WAYS) {
      for (const auto& way : primitive_group.ways()) {
        uint64_t node = 0;
        std::vector<uint64_t> nodes;
        nodes.reserve(way.refs_size());
        for (auto node_id_offset : way.refs()) {
          node += node_id_offset;
          // TODO: skip consecutive duplicates, make this configurable
          if (nodes.size() == 0 || node != nodes.back()) {
            nodes.push_back(node);
          }
        }
        callback.way_callback(way.id(), get_tags<Way>(way, primblock), nodes);
        if (way.has_info() && way.info().has_changeset() && (interest & CHANGESETS) == CHANGESETS) {
          callback.changeset_callback(way.info().changeset());
        }
      }
    }

    // do the relations
    if ((interest & RELATIONS) == RELATIONS) {
      for (const auto& relation : primitive_group.relations()) {
        uint64_t member = 0;
        std::vector<Member> members;
        members.reserve(relation.memids_size());
        for (int l = 0; l < relation.memids_size(); ++l) {
          member += relation.memids(l);
          members.emplace_back(relation.types(l), member,
                               primblock.stringtable().s(relation.roles_sid(l)));
        }
        callback.relation_callback(relation.id(), get_tags<Relation>(relation, primblock), members);
        if (relation.has_info() && relation.info().has_changeset() &&
            (interest & CHANGESETS) == CHANGESETS) {
          callback.changeset_callback(relation.info().changeset());
        }
      }
    }

    // do the changesets
    if ((interest & CHANGESETS) == CHANGESETS) {
      for (const auto& changeset : primitive_group.changesets()) {
        callback.changeset_callback(changeset.id());
      }
    }
  }
}

void parse_header_block(char* unpack_buffer, int32_t sz) {
  // turn the blob bytes into a protobuf object
  HeaderBlock header_block;
  if (!header_block.ParseFromArray(unpack_buffer, sz)) {
    throw std::runtime_error("unable to parse header block");
  }

  // TODO: do something with replication information?
}

} // namespace

// extend the protobuf osmpbf namespace
namespace OSMPBF {

Member::Member(const Relation::MemberType type, const uint64_t id, const std::string& role)
    : member_type(type), member_id(id), role(role) {
}

Member::Member(Member&& other)
    : member_type(other.member_type), member_id(other.member_id), role(std::move(other.role)) {
}

void Parser::parse(std::ifstream& file, const Interest interest, Callback& callback) {
  char* buffer = new char[MAX_UNCOMPRESSED_BLOB_SIZE];
  char* unpack_buffer = new char[MAX_UNCOMPRESSED_BLOB_SIZE];

  // start from the top
  file.clear();
  file.seekg(0, std::ios::beg);

  // while there is more to read
  while (!file.eof()) {
    // grab the blob header
    bool finished = false;
    BlobHeader header = read_header(buffer, file, finished);
    // if we didnt hit the end
    if (!finished) {
      // grab the blob that goes with the blob header
      int32_t sz = read_blob(buffer, unpack_buffer, file, header);
      // if its data parse it
      if (header.type() == "OSMData") {
        parse_primitive_block(unpack_buffer, sz, interest, callback);
        // if its something other than a header
      } else if (header.type() == "OSMHeader") {
        parse_header_block(unpack_buffer, sz);
      } else {
        LOG_WARN("Unknown blob type: " + header.type());
      }
    }
  }

  // done with protobuf and buffers
  delete[] buffer;
  delete[] unpack_buffer;
}

void Parser::free() {
  google::protobuf::ShutdownProtobufLibrary();
}

} // namespace OSMPBF
