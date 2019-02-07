#include <cstdint>
#include <fstream>
#include <iostream>

#include <boost/filesystem/operations.hpp>

#include "midgard/logging.h"
#include "mjolnir/osmdata.h"
#include "mjolnir/util.h"

using namespace valhalla::mjolnir;

namespace {

// Temporary files used during tile building
const std::string count_file = "osmdata_counts.bin";
const std::string restrictions_file = "osmdata_restrictions.bin";
const std::string viaset_file = "osmdata_viaset.bin";
const std::string access_restrictions_file = "osmdata_access_restrictions.bin";
const std::string bike_relations_file = "osmdata_bike_relations.bin";
const std::string wayref_file = "osmdata_wayrefs.bin";
const std::string unique_names_file = "osmdata_unique_strings.bin";
const std::string lane_connectivity_file = "osmdata_lane_connectivity.bin";

// Data structures to assist writing and reading data
struct BikeRelation {
  uint32_t way_id;
  OSMBike bike_relation;
};

bool write_restrictions(const std::string& filename, const RestrictionsMultiMap& res) {
  // Open file and truncate
  std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
  if (!file.is_open()) {
    LOG_ERROR("write_restrictions failed to open output file: " + filename);
    return false;
  }

  return true;
}

bool write_viaset(const std::string& filename, const ViaSet& vias) {
  // Open file and truncate
  std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
  if (!file.is_open()) {
    LOG_ERROR("write_viaset failed to open output file: " + filename);
    return false;
  }

  // TODO!

  return true;
}

bool write_access_restrictions(const std::string& filename, const AccessRestrictionsMultiMap& access) {
  // Open file and truncate
  std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
  if (!file.is_open()) {
    LOG_ERROR("write_access_restrictions failed to open output file: " + filename);
    return false;
  }

  // TODO!

  return true;
}

bool write_bike_relations(const std::string& filename, const BikeMultiMap& bike_relations) {
  // Open file and truncate
  std::stringstream in_mem;
  std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
  if (!file.is_open()) {
    LOG_ERROR("write_bike_relations failed to open output file: " + filename);
    return false;
  }

  // Create a vector of bike relations from the multimap
  std::vector<BikeRelation> relations;
  for (const auto& r : bike_relations) {
    // TODO - iterate through the multimap

  }

  // Write the count and then the wayids (keys)
  in_mem.write(reinterpret_cast<const char*>(relations.size()), sizeof(uint32_t));
  in_mem.write(reinterpret_cast<const char*>(relations.data()), relations.size() * sizeof(BikeRelation));
  file << in_mem.rdbuf();
  file.close();
  return true;
}

bool write_way_refs(const std::string& filename, const OSMStringMap& string_map) {
  // Open file and truncate
  std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
  if (!file.is_open()) {
    LOG_ERROR("write_way_refs failed to open output file: " + filename);
    return false;
  }

  // Store the way Id keys in a vector and the strings in ????
  uint32_t i = 0;
  std::vector<uint32_t> wayids(string_map.size());
  std::vector<char> strings;
  for (const auto& s : string_map) {
    wayids[i] = s.first;
  }

  // TODO - store the strings

  // Write the count and then the wayids (keys)
  std::stringstream in_mem;
  in_mem.write(reinterpret_cast<const char*>(wayids.size()), sizeof(uint32_t));
  in_mem.write(reinterpret_cast<const char*>(wayids.data()), wayids.size() * sizeof(uint32_t));

  // TODO write the strings to in_mem

  file << in_mem.rdbuf();
  file.close();
  return true;
}

bool write_unique_names(const std::string& filename, const UniqueNames& names) {
  // Open file and truncate
  std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
  if (!file.is_open()) {
    LOG_ERROR("write_unique_names failed to open output file: " + filename);
    return false;
  }

  // TODO!

  return true;
}

bool write_lane_connectivity(const std::string& filename, const OSMLaneConnectivityMultiMap& lanes) {
  // Open file and truncate
  std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
  if (!file.is_open()) {
    LOG_ERROR("write_lane_connectivity failed to open output file: " + filename);
    return false;
  }

  // TODO!

  return true;
}

bool read_restrictions(const std::string& filename, RestrictionsMultiMap& res) {
  // Open file and truncate
  std::ifstream file(filename, std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    LOG_ERROR("read_restrictions failed to open input file: " + filename);
    return false;
  }

  // TODO!!

  return true;
}

bool read_viaset(const std::string& filename, ViaSet& res) {
  // Open file and truncate
  std::ifstream file(filename, std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    LOG_ERROR("read_viaset failed to open input file: " + filename);
    return false;
  }

  // TODO!!

  return true;
}

bool read_access_restrictions(const std::string& filename, AccessRestrictionsMultiMap& res) {
  // Open file and truncate
  std::ifstream file(filename, std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    LOG_ERROR("read_access_restrictions failed to open input file: " + filename);
    return false;
  }

  // TODO!!

  return true;
}

bool read_bike_relations(const std::string& filename, const BikeMultiMap& string_map) {
  // Open file and truncate
  std::ifstream file(filename, std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    LOG_ERROR("read_bike_relations failed to open input file: " + filename);
    return false;
  }

  // Read the wayids (keys)
  uint32_t count;
  file.read(reinterpret_cast<char*>(&count), sizeof(uint32_t));
  std::vector<uint32_t> wayids(count);
  file.read(reinterpret_cast<char*>(wayids.data()), sizeof(uint32_t) * count);

  // TODO - read the strings

  // TODO - add the strings into string_map using the way Ids as keys

  return true;
}

bool read_way_refs(const std::string& filename, const OSMStringMap& string_map) {
  // Open file and truncate
  std::ifstream file(filename, std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    LOG_ERROR("read_way_refs failed to open input file: " + filename);
    return false;
  }

  // Read the wayids (keys)
  uint32_t count;
  file.read(reinterpret_cast<char*>(&count), sizeof(uint32_t));
  std::vector<uint32_t> wayids(count);
  file.read(reinterpret_cast<char*>(wayids.data()), sizeof(uint32_t) * count);

  // TODO - read the strings

  // TODO - add the strings into string_map using the way Ids as keys

  return true;
}


bool read_unique_names(const std::string& filename, UniqueNames& names) {
  // Open file and truncate
  std::ifstream file(filename, std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    LOG_ERROR("read_unique_names failed to open input file: " + filename);
    return false;
  }

  // TODO!!

  return true;
}

bool read_lane_connectivity(const std::string& filename, OSMLaneConnectivityMultiMap& lanes) {
  // Open file and truncate
  std::ifstream file(filename, std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    LOG_ERROR("read_lane_connectivity failed to open input file: " + filename);
    return false;
  }

  // TODO!!

  return true;
}

} // namespace

namespace valhalla {
namespace mjolnir {

// Write OSMData to temporary files
bool OSMData::write_to_temp_files(const std::string& tile_dir) {
  // Write counts
  std::string countfile = tile_dir + count_file;
  std::ofstream file(countfile, std::ios::out | std::ios::binary | std::ios::trunc);
  if (!file.is_open()) {
    LOG_ERROR("Failed to open output file: " + countfile);
    return false;
  }
  file.write(reinterpret_cast<const char*>(&max_changeset_id_), sizeof(uint64_t));
  file.write(reinterpret_cast<const char*>(&osm_node_count), sizeof(size_t));
  file.write(reinterpret_cast<const char*>(&osm_way_count), sizeof(size_t));
  file.write(reinterpret_cast<const char*>(&osm_way_node_count), sizeof(size_t));
  file.write(reinterpret_cast<const char*>(&intersection_count), sizeof(size_t));
  file.write(reinterpret_cast<const char*>(&node_count), sizeof(size_t));
  file.write(reinterpret_cast<const char*>(&edge_count), sizeof(size_t));
  file.write(reinterpret_cast<const char*>(&node_ref_count), sizeof(size_t));
  file.write(reinterpret_cast<const char*>(&node_name_count), sizeof(size_t));
  file.write(reinterpret_cast<const char*>(&node_exit_to_count), sizeof(size_t));
  file.close();

  // Write the rest of OSMData
  return write_restrictions(tile_dir + restrictions_file, restrictions) &&
         write_viaset(tile_dir + viaset_file, via_set) &&
         write_access_restrictions(tile_dir + access_restrictions_file, access_restrictions) &&
         write_bike_relations(tile_dir + bike_relations_file, bike_relations) &&
         write_way_refs(tile_dir + wayref_file, way_ref) &&
         write_unique_names(tile_dir + unique_names_file, name_offset_map) &&
         write_lane_connectivity(tile_dir + lane_connectivity_file, lane_connectivity_map);
}

// Read OSMData from temporary files
bool OSMData::read_from_temp_files(const std::string& tile_dir) {
  // Open the count file
  std::string countfile = tile_dir + count_file;
  std::ifstream file(countfile, std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    LOG_ERROR("Failed to open input file: " + countfile);
    return false;
  }
  file.read(reinterpret_cast<char*>(&max_changeset_id_), sizeof(uint64_t));
  file.read(reinterpret_cast<char*>(&osm_node_count), sizeof(size_t));
  file.read(reinterpret_cast<char*>(&osm_way_count), sizeof(size_t));
  file.read(reinterpret_cast<char*>(&osm_way_node_count), sizeof(size_t));
  file.read(reinterpret_cast<char*>(&intersection_count), sizeof(size_t));
  file.read(reinterpret_cast<char*>(&node_count), sizeof(size_t));
  file.read(reinterpret_cast<char*>(&edge_count), sizeof(size_t));
  file.read(reinterpret_cast<char*>(&node_ref_count), sizeof(size_t));
  file.read(reinterpret_cast<char*>(&node_name_count), sizeof(size_t));
  file.read(reinterpret_cast<char*>(&node_exit_to_count), sizeof(size_t));
  file.close();

  // Read the other data
  return read_restrictions(tile_dir + restrictions_file, restrictions) &&
         read_viaset(tile_dir + viaset_file, via_set) &&
         read_access_restrictions(tile_dir + access_restrictions_file, access_restrictions) &&
         read_bike_relations(tile_dir + bike_relations_file, bike_relations) &&
         read_way_refs(tile_dir + wayref_file, way_ref) &&
         read_unique_names(tile_dir + unique_names_file, name_offset_map) &&
         read_lane_connectivity(tile_dir + lane_connectivity_file, lane_connectivity_map);
}

void OSMData::cleanup_temp_files(const std::string& tile_dir) {
  auto remove_temp_file = [](const std::string& fname) {
    if (boost::filesystem::exists(fname)) {
      boost::filesystem::remove(fname);
    }
  };

  remove_temp_file(tile_dir + count_file);
  remove_temp_file(tile_dir + restrictions_file);
  remove_temp_file(tile_dir + viaset_file);
  remove_temp_file(tile_dir + access_restrictions_file);
  remove_temp_file(tile_dir + bike_relations_file);
  remove_temp_file(tile_dir + wayref_file);
  remove_temp_file(tile_dir + unique_names_file);
  remove_temp_file(tile_dir + lane_connectivity_file);
}

} // namespace mjolnir
} // namespace valhalla
