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
#pragma once

#include <stdint.h>
#include <netinet/in.h>
#include <zlib.h>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>

// this describes the low-level blob storage
#include "include/proto/fileformat.pb.h"
// this describes the high-level OSM objects
#include "include/proto/osmformat.pb.h"
// the maximum size of a blob header in bytes
const int max_blob_header_size = 64 * 1024; // 64 kB
// the maximum size of an uncompressed blob in bytes
const int max_uncompressed_blob_size = 32 * 1024 * 1024; // 32 MB
// resolution for longitude/latitude used for conversion
// between representation as double and as int
const int lonlat_resolution = 1000 * 1000 * 1000; 

namespace CanalTP {

// Represents the key/values of an object
typedef std::map<std::string, std::string> Tags;

// References of a relation
struct Reference {
    OSMPBF::Relation::MemberType member_type; // type de la relation
    uint64_t member_id; // OSMID
    std::string role; // le role

    Reference() {}
    Reference(OSMPBF::Relation::MemberType member_type, uint64_t member_id, std::string role) :
        member_type(member_type), member_id(member_id), role(role)
    {}
};

typedef std::vector<Reference> References;

// Main function
template<typename Visitor>
void read_osm_pbf(const std::string & filename, Visitor & visitor);

struct warn {
    warn() {std::cout << "\033[33m[WARN] ";}
    template<typename T>warn & operator<<(const T & t){ std::cout << t; return *this;}
    ~warn() {std::cout << "\033[0m" << std::endl;}
};

struct info {
    info() {std::cout << "\033[32m[INFO] ";}
    template<typename T>info & operator<<(const T & t){ std::cout << t; return *this;}
    ~info() {std::cout << "\033[0m" << std::endl;}
};

struct fatal {
    fatal() {std::cout << "\033[31m[FATAL] ";}
    template<typename T>fatal & operator<<(const T & t){ std::cout << t; return *this;}
    ~fatal() {std::cout << "\033[0m" << std::endl; exit(1);}
};



template<typename T>
Tags get_tags(T object, const OSMPBF::PrimitiveBlock &primblock){
    Tags result;
    for(int i = 0; i < object.keys_size(); ++i){
        uint64_t key = object.keys(i);
        uint64_t val = object.vals(i);
        std::string key_string = primblock.stringtable().s(key);
        std::string val_string = primblock.stringtable().s(val);
        result[key_string] = val_string;
    }
    return result;
}

template<typename Visitor>
struct Parser {

    void parse(){
        while(!this->file.eof() && !finished) {
            OSMPBF::BlobHeader header = this->read_header();
            if(!this->finished){
                int32_t sz = this->read_blob(header);
                if(header.type() == "OSMData") {
                    this->parse_primitiveblock(sz);
                }
                else if(header.type() == "OSMHeader"){
                }
                else {
                    warn() << "  unknown blob type: " << header.type();
                }
            }
        }
    }

    Parser(const std::string & filename, Visitor & visitor)
        : visitor(visitor), file(filename.c_str(), std::ios::binary ), finished(false)
    {
        if(!file.is_open())
            fatal() << "Unable to open the file " << filename;
        buffer = new char[max_uncompressed_blob_size];
        unpack_buffer = new char[max_uncompressed_blob_size];
        info() << "Reading the file" << filename;
    }

    ~Parser(){
        delete[] buffer;
        delete[] unpack_buffer;
        google::protobuf::ShutdownProtobufLibrary();
    }

private:
    Visitor & visitor;
    std::ifstream file;
    char* buffer;
    char* unpack_buffer;
    bool finished;

    OSMPBF::BlobHeader read_header(){
        int32_t sz;
        OSMPBF::BlobHeader result;

        // read the first 4 bytes of the file, this is the size of the blob-header
        if( !file.read((char*)&sz, 4) ){
            info() << "We finished reading the file";
            this->finished = true;
            return result;
        }

        sz = ntohl(sz);// convert the size from network byte-order to host byte-order

        if(sz > max_blob_header_size)
            fatal() << "blob-header-size is bigger then allowed " << sz << " > " << max_blob_header_size;

        this->file.read(this->buffer, sz);
        if(!this->file.good())
            fatal() << "unable to read blob-header from file";

        // parse the blob-header from the read-buffer
        if(!result.ParseFromArray(this->buffer, sz))
            fatal() << "unable to parse blob header";
        return result;
    }

    int32_t read_blob(const OSMPBF::BlobHeader & header){
        OSMPBF::Blob blob;
        // size of the following blob
        int32_t sz = header.datasize();

        if(sz > max_uncompressed_blob_size)
            fatal() << "blob-size is bigger then allowed";

        if(!this->file.read(buffer, sz))
            fatal() << "unable to read blob from file";
        if(!blob.ParseFromArray(this->buffer, sz))
            fatal() << "unable to parse blob";

        // if the blob has uncompressed data
        if(blob.has_raw()) {
            // size of the blob-data
            sz = blob.raw().size();

            // check that raw_size is set correctly
            if(sz != blob.raw_size())
                warn() << "  reports wrong raw_size: " << blob.raw_size() << " bytes";

            memcpy(unpack_buffer, buffer, sz);
            return sz;
        }


        if(blob.has_zlib_data()) {
            sz = blob.zlib_data().size();

            z_stream z;
            z.next_in   = (unsigned char*) blob.zlib_data().c_str();
            z.avail_in  = sz;
            z.next_out  = (unsigned char*) unpack_buffer;
            z.avail_out = blob.raw_size();
            z.zalloc    = Z_NULL;
            z.zfree     = Z_NULL;
            z.opaque    = Z_NULL;

            if(inflateInit(&z) != Z_OK) {
                fatal() << "failed to init zlib stream";
            }
            if(inflate(&z, Z_FINISH) != Z_STREAM_END) {
                fatal() << "failed to inflate zlib stream";
            }
            if(inflateEnd(&z) != Z_OK) {
                fatal() << "failed to deinit zlib stream";
            }
            return z.total_out;
        }

        if(blob.has_lzma_data()) {
            fatal() << "lzma-decompression is not supported";
        }
        return 0;
    }

    void parse_primitiveblock(int32_t sz) {
        OSMPBF::PrimitiveBlock primblock;
        if(!primblock.ParseFromArray(this->unpack_buffer, sz))
            fatal() << "unable to parse primitive block";

        for(int i = 0, l = primblock.primitivegroup_size(); i < l; i++) {
            OSMPBF::PrimitiveGroup pg = primblock.primitivegroup(i);

            // Simple Nodes
            for(int i = 0; i < pg.nodes_size(); ++i) {
                OSMPBF::Node n = pg.nodes(i);

                double lon = 0.000000001 * (primblock.lon_offset() + (primblock.granularity() * n.lon())) ;
                double lat = 0.000000001 * (primblock.lat_offset() + (primblock.granularity() * n.lat())) ;
                visitor.node_callback(n.id(), lon, lat, get_tags(n, primblock));
            }

            // Dense Nodes
            if(pg.has_dense()) {
                OSMPBF::DenseNodes dn = pg.dense();
                uint64_t id = 0;
                double lon = 0;
                double lat = 0;

                int current_kv = 0;

                for(int i = 0; i < dn.id_size(); ++i) {
                    id += dn.id(i);
                    lon +=  0.000000001 * (primblock.lon_offset() + (primblock.granularity() * dn.lon(i)));
                    lat +=  0.000000001 * (primblock.lat_offset() + (primblock.granularity() * dn.lat(i)));

                    Tags tags;
                    while (current_kv < dn.keys_vals_size() && dn.keys_vals(current_kv) != 0){
                        uint64_t key = dn.keys_vals(current_kv);
                        uint64_t val = dn.keys_vals(current_kv + 1);
                        std::string key_string = primblock.stringtable().s(key);
                        std::string val_string = primblock.stringtable().s(val);
                        current_kv += 2;
                        tags[key_string] = val_string;
                    }
                    ++current_kv;
                    visitor.node_callback(id, lon, lat, tags);
                }
            }

            for(int i = 0; i < pg.ways_size(); ++i) {
                OSMPBF::Way w = pg.ways(i);

                uint64_t ref = 0;
                std::vector<uint64_t> refs;
                for(int j = 0; j < w.refs_size(); ++j){
                    ref += w.refs(j);
                    refs.push_back(ref);
                }
                uint64_t id = w.id();
                visitor.way_callback(id, get_tags(w, primblock), refs);
            }


            for(int i=0; i < pg.relations_size(); ++i){
                OSMPBF::Relation rel = pg.relations(i);
                uint64_t id = 0;
                References refs;

                for(int l = 0; l < rel.memids_size(); ++l){
                    id += rel.memids(l);
                    refs.push_back(Reference(rel.types(l), id, primblock.stringtable().s(rel.roles_sid(l))));
                }

                visitor.relation_callback(rel.id(), get_tags(rel, primblock), refs);
            }
        }
    }
};

template<typename Visitor>
void read_osm_pbf(const std::string & filename, Visitor & visitor){
    Parser<Visitor> p(filename, visitor);
    p.parse();
}

}
