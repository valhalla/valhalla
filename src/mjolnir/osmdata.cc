#include <cctype>
#include <cstdint>
#include <fstream>
#include <iostream>

#include <boost/algorithm/string.hpp>
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
const std::string way_ref_file = "osmdata_way_refs.bin";
const std::string way_ref_rev_file = "osmdata_way_refs_rev.bin";
const std::string node_names_file = "osmdata_node_names.bin";
const std::string unique_names_file = "osmdata_unique_strings.bin";
const std::string lane_connectivity_file = "osmdata_lane_connectivity.bin";

// Data structures to assist writing and reading data
struct TempRestriction {
  uint32_t way_id;
  OSMRestriction restriction;
  TempRestriction() : way_id(0), restriction(OSMRestriction()) {
  }
  TempRestriction(const uint32_t w, const OSMRestriction& r) : way_id(w), restriction(r) {
  }
};

struct TempWayRef {
  uint32_t way_id;
  uint32_t name_index;
  TempWayRef() : way_id(0), name_index(0) {
  }
  TempWayRef(const uint32_t w, const uint32_t& index) : way_id(w), name_index(index) {
  }
};

struct BikeRelation {
  uint32_t way_id;
  OSMBike relation;
  BikeRelation() : way_id(0), relation(OSMBike()) {
  }
  BikeRelation(const uint32_t w, const OSMBike& r) : way_id(w), relation(r) {
  }
};

struct TempAccessRestriction {
  uint32_t way_id;
  OSMAccessRestriction restriction;
  TempAccessRestriction() : way_id(0), restriction(OSMAccessRestriction()) {
  }
  TempAccessRestriction(const uint32_t w, const OSMAccessRestriction& r) : way_id(w), restriction(r) {
  }
};

struct TempLaneConnectivity {
  uint32_t way_id;
  OSMLaneConnectivity lane;
  TempLaneConnectivity() : way_id(0), lane(OSMLaneConnectivity()) {
  }
  TempLaneConnectivity(const uint32_t w, const OSMLaneConnectivity& l) : way_id(w), lane(l) {
  }
};

bool write_restrictions(const std::string& filename, const RestrictionsMultiMap& res_map) {
  // Open file and truncate
  std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
  if (!file.is_open()) {
    LOG_ERROR("write_restrictions failed to open output file: " + filename);
    return false;
  }

  // Convert the multi map into a vector of TempRestriction
  std::vector<TempRestriction> res;
  for (auto it = res_map.cbegin(); it != res_map.cend(); ++it) {
    res.emplace_back(it->first, it->second);
  }

  // Write the count and then the via ids
  uint32_t sz = res.size();
  file.write(reinterpret_cast<const char*>(&sz), sizeof(uint32_t));
  file.write(reinterpret_cast<const char*>(res.data()), res.size() * sizeof(TempRestriction));
  file.close();
  return true;
}

bool write_viaset(const std::string& filename, const ViaSet& via_set) {
  // Open file and truncate
  std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
  if (!file.is_open()) {
    LOG_ERROR("write_viaset failed to open output file: " + filename);
    return false;
  }

  // Create a vector to hold the elements of the via set
  uint32_t i = 0;
  std::vector<uint32_t> via_vector(via_set.size());
  for (const auto v : via_set) {
    via_vector[i] = v;
    ++i;
  }

  // Write the count and then the via ids
  uint32_t sz = via_vector.size();
  file.write(reinterpret_cast<const char*>(&sz), sizeof(uint32_t));
  file.write(reinterpret_cast<const char*>(via_vector.data()), via_vector.size() * sizeof(uint32_t));
  file.close();
  return true;
}

bool write_access_restrictions(const std::string& filename,
                               const AccessRestrictionsMultiMap& access_map) {
  // Open file and truncate
  std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
  if (!file.is_open()) {
    LOG_ERROR("write_access_restrictions failed to open output file: " + filename);
    return false;
  }

  // Convert the multi map into a vector of TempAccessRestriction
  std::vector<TempAccessRestriction> res;
  for (auto it = access_map.cbegin(); it != access_map.cend(); ++it) {
    res.emplace_back(it->first, it->second);
  }

  // Write the count and then the via ids
  uint32_t sz = res.size();
  file.write(reinterpret_cast<const char*>(&sz), sizeof(uint32_t));
  file.write(reinterpret_cast<const char*>(res.data()), res.size() * sizeof(TempAccessRestriction));
  file.close();
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
  for (auto it = bike_relations.cbegin(); it != bike_relations.cend(); ++it) {
    relations.emplace_back(it->first, it->second);
  }

  // Write the count and then the bike relations
  uint32_t sz = relations.size();
  file.write(reinterpret_cast<const char*>(&sz), sizeof(uint32_t));
  file.write(reinterpret_cast<const char*>(relations.data()),
             relations.size() * sizeof(BikeRelation));
  file.close();
  return true;
}

bool write_way_refs(const std::string& filename, const OSMStringMap& way_refs) {
  // Open file and truncate
  std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
  if (!file.is_open()) {
    LOG_ERROR("write_way_refs failed to open output file: " + filename);
    return false;
  }

  // Store the way Id keys and name indexes in a TempWayRef vector
  uint32_t i = 0;
  std::vector<TempWayRef> temp_wayrefs(way_refs.size());
  std::vector<char> strings;
  for (const auto& s : way_refs) {
    temp_wayrefs[i] = {s.first, s.second};
    ++i;
  }

  // Write the count and then the TempWayRefs
  uint32_t sz = temp_wayrefs.size();
  file.write(reinterpret_cast<const char*>(&sz), sizeof(uint32_t));
  file.write(reinterpret_cast<const char*>(temp_wayrefs.data()),
             temp_wayrefs.size() * sizeof(TempWayRef));
  file.close();
  return true;
}

bool write_node_names(const std::string& filename, const UniqueNames& names) {
  // Open file and truncate
  std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
  if (!file.is_open()) {
    LOG_ERROR("write_node_names failed to open output file: " + filename);
    return false;
  }

  // Store a count of strings followed by an array of string lengths
  uint32_t offset = 0;
  uint32_t name_count = names.Size();
  std::vector<uint32_t> lengths(name_count);
  std::vector<char> namebuf;
  for (uint32_t n = 0; n < name_count; ++n) {
    const auto str = names.name(n + 1); // Add 1 since the first name is blank
    lengths[n] = str.length() + 1;      // Add 1 for the null terminator

    // Copy the string to the namebuf and add a terminator
    std::copy(str.c_str(), str.c_str() + str.length(), back_inserter(namebuf));
    namebuf.push_back(0);
  }

  // Write to file
  file.write(reinterpret_cast<const char*>(&name_count), sizeof(uint32_t));
  file.write(reinterpret_cast<const char*>(lengths.data()), lengths.size() * sizeof(uint32_t));
  uint32_t sz = namebuf.size();
  file.write(reinterpret_cast<const char*>(&sz), sizeof(uint32_t));
  file.write(reinterpret_cast<const char*>(namebuf.data()), namebuf.size());
  return true;
}

bool write_unique_names(const std::string& filename, const UniqueNames& names) {
  // Open file and truncate
  std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
  if (!file.is_open()) {
    LOG_ERROR("write_unique_names failed to open output file: " + filename);
    return false;
  }

  // Store a count of strings followed by an array of string lengths
  uint32_t offset = 0;
  uint32_t name_count = names.Size();
  std::vector<uint32_t> lengths(name_count);
  std::vector<char> namebuf;
  for (uint32_t n = 0; n < name_count; ++n) {
    const auto str = names.name(n + 1); // Add 1 since the first name is blank
    lengths[n] = str.length() + 1;      // Add 1 for the null terminator

    // Copy the string to the namebuf and add a terminator
    std::copy(str.c_str(), str.c_str() + str.length(), back_inserter(namebuf));
    namebuf.push_back(0);
  }

  // Write to file
  file.write(reinterpret_cast<const char*>(&name_count), sizeof(uint32_t));
  file.write(reinterpret_cast<const char*>(lengths.data()), lengths.size() * sizeof(uint32_t));
  uint32_t sz = namebuf.size();
  file.write(reinterpret_cast<const char*>(&sz), sizeof(uint32_t));
  file.write(reinterpret_cast<const char*>(namebuf.data()), namebuf.size());
  return true;
}

bool write_lane_connectivity(const std::string& filename,
                             const OSMLaneConnectivityMultiMap& lane_map) {
  // Open file and truncate
  std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
  if (!file.is_open()) {
    LOG_ERROR("write_lane_connectivity failed to open output file: " + filename);
    return false;
  }

  // Convert the multi map into a vector of TempLaneConnectivity
  std::vector<TempLaneConnectivity> lanes;
  for (auto it = lane_map.cbegin(); it != lane_map.cend(); ++it) {
    lanes.emplace_back(it->first, it->second);
  }

  // Write the count and then the via ids
  uint32_t sz = lanes.size();
  file.write(reinterpret_cast<const char*>(&sz), sizeof(uint32_t));
  file.write(reinterpret_cast<const char*>(lanes.data()),
             lanes.size() * sizeof(TempLaneConnectivity));
  file.close();
  return true;
  return true;
}

bool read_restrictions(const std::string& filename, RestrictionsMultiMap& res_map) {
  // Open file and truncate
  std::ifstream file(filename, std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    LOG_ERROR("read_restrictions failed to open input file: " + filename);
    return false;
  }

  // Read the count and then the temporary restriction list
  uint32_t count = 0;
  file.read(reinterpret_cast<char*>(&count), sizeof(uint32_t));
  std::vector<TempRestriction> access_res(count);
  file.read(reinterpret_cast<char*>(access_res.data()), count * sizeof(TempRestriction));
  file.close();

  // Iterate through the temporary restriction list and add to the restriction multi-map
  for (const auto& r : access_res) {
    res_map.insert({r.way_id, r.restriction});
  }
  return true;
}

bool read_viaset(const std::string& filename, ViaSet& via_set) {
  // Open file and truncate
  std::ifstream file(filename, std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    LOG_ERROR("read_viaset failed to open input file: " + filename);
    return false;
  }

  // Read the count and then the via ids
  uint32_t count = 0;
  file.read(reinterpret_cast<char*>(&count), sizeof(uint32_t));
  std::vector<uint32_t> via_vector(count);
  file.read(reinterpret_cast<char*>(via_vector.data()), count * sizeof(uint32_t));
  file.close();

  // Iterate through the vector of via Ids and add them to the via set
  for (const auto v : via_vector) {
    via_set.insert(v);
  }
  return true;
}

bool read_access_restrictions(const std::string& filename, AccessRestrictionsMultiMap& access_map) {
  // Open file and truncate
  std::ifstream file(filename, std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    LOG_ERROR("read_access_restrictions failed to open input file: " + filename);
    return false;
  }

  // Read the count and then the temporary access restriction list
  uint32_t count = 0;
  file.read(reinterpret_cast<char*>(&count), sizeof(uint32_t));
  std::vector<TempAccessRestriction> access_res(count);
  file.read(reinterpret_cast<char*>(access_res.data()), count * sizeof(TempAccessRestriction));
  file.close();

  // Iterate through the temporary access restriction list and add to the restriction multi-map
  for (const auto& r : access_res) {
    access_map.insert({r.way_id, r.restriction});
  }
  return true;
}

bool read_bike_relations(const std::string& filename, BikeMultiMap& bike_relations) {
  // Open file and truncate
  std::ifstream file(filename, std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    LOG_ERROR("read_bike_relations failed to open input file: " + filename);
    return false;
  }

  // Read the count and then the bike relations list
  uint32_t count = 0;
  file.read(reinterpret_cast<char*>(&count), sizeof(uint32_t));
  std::vector<BikeRelation> rel(count);
  file.read(reinterpret_cast<char*>(rel.data()), count * sizeof(BikeRelation));
  file.close();

  // Iterate through the temporary bike relations list and add to the bike relations multi-map
  for (const auto& r : rel) {
    bike_relations.insert({r.way_id, r.relation});
  }
  return true;
}

bool read_way_refs(const std::string& filename, OSMStringMap& way_refs) {
  // Open file and truncate
  std::ifstream file(filename, std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    LOG_ERROR("read_way_refs failed to open input file: " + filename);
    return false;
  }

  // Read the wayids (keys)
  uint32_t count;
  file.read(reinterpret_cast<char*>(&count), sizeof(uint32_t));
  std::vector<TempWayRef> temp_wayrefs(count);
  file.read(reinterpret_cast<char*>(temp_wayrefs.data()), sizeof(TempWayRef) * count);

  // Iterate through the temp wayrefs and form map
  for (const auto& r : temp_wayrefs) {
    way_refs[r.way_id] = r.name_index;
  }
  return true;
}

bool read_node_names(const std::string& filename, UniqueNames& names) {
  // Open file and truncate
  std::ifstream file(filename, std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    LOG_ERROR("read_node_names failed to open input file: " + filename);
    return false;
  }

  // Read from file
  uint32_t count = 0;
  file.read(reinterpret_cast<char*>(&count), sizeof(uint32_t));
  std::vector<uint32_t> lengths(count);
  file.read(reinterpret_cast<char*>(lengths.data()), count * sizeof(uint32_t));
  uint32_t bufsize = 0;
  file.read(reinterpret_cast<char*>(&bufsize), sizeof(uint32_t));
  std::vector<char> namebuf(bufsize);
  file.read(reinterpret_cast<char*>(namebuf.data()), bufsize);

  // Iterate through the temporary data and add the unique names
  uint32_t offset = 0;
  for (uint32_t n = 0; n < count; ++n) {
    std::string name(&namebuf[offset]);
    names.index(name);
    offset += lengths[n];

    if ((name.length() + 1) != lengths[n]) {
      LOG_ERROR("name " + name + " length should be " + std::to_string(lengths[n]));
    }
  }
  return true;
}

bool read_unique_names(const std::string& filename, UniqueNames& names) {
  // Open file and truncate
  std::ifstream file(filename, std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    LOG_ERROR("read_unique_names failed to open input file: " + filename);
    return false;
  }

  // Read from file
  uint32_t count = 0;
  file.read(reinterpret_cast<char*>(&count), sizeof(uint32_t));
  std::vector<uint32_t> lengths(count);
  file.read(reinterpret_cast<char*>(lengths.data()), count * sizeof(uint32_t));
  uint32_t bufsize = 0;
  file.read(reinterpret_cast<char*>(&bufsize), sizeof(uint32_t));
  std::vector<char> namebuf(bufsize);
  file.read(reinterpret_cast<char*>(namebuf.data()), bufsize);

  // Iterate through the temporary data and add the unique names
  uint32_t offset = 0;
  for (uint32_t n = 0; n < count; ++n) {
    std::string name(&namebuf[offset]);
    names.index(name);
    offset += lengths[n];

    if ((name.length() + 1) != lengths[n]) {
      LOG_ERROR("name " + name + " length should be " + std::to_string(lengths[n]));
    }
  }
  return true;
}

bool read_lane_connectivity(const std::string& filename, OSMLaneConnectivityMultiMap& lane_map) {
  // Open file and truncate
  std::ifstream file(filename, std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    LOG_ERROR("read_lane_connectivity failed to open input file: " + filename);
    return false;
  }

  // Read the count and then the temporary lane connectivity list
  uint32_t count = 0;
  file.read(reinterpret_cast<char*>(&count), sizeof(uint32_t));
  std::vector<TempLaneConnectivity> lanes(count);
  file.read(reinterpret_cast<char*>(lanes.data()), count * sizeof(TempLaneConnectivity));
  file.close();

  // Iterate through the temporary lane connectivity list and add to the lane connectivity multi-map
  for (const auto& l : lanes) {
    lane_map.insert({l.way_id, l.lane});
  }
  return true;
}

} // namespace

namespace valhalla {
namespace mjolnir {

// Write OSMData to temporary files
bool OSMData::write_to_temp_files(const std::string& tile_dir) {
  LOG_INFO("Write OSMData to temp files");

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
  bool status = write_restrictions(tile_dir + restrictions_file, restrictions) &&
                write_viaset(tile_dir + viaset_file, via_set) &&
                write_access_restrictions(tile_dir + access_restrictions_file, access_restrictions) &&
                write_bike_relations(tile_dir + bike_relations_file, bike_relations) &&
                write_way_refs(tile_dir + way_ref_file, way_ref) &&
                write_way_refs(tile_dir + way_ref_rev_file, way_ref_rev) &&
                write_node_names(tile_dir + node_names_file, node_names) &&
                write_unique_names(tile_dir + unique_names_file, name_offset_map) &&
                write_lane_connectivity(tile_dir + lane_connectivity_file, lane_connectivity_map);
  LOG_INFO("Done");
  return status;
}

// Read OSMData from temporary files
bool OSMData::read_from_temp_files(const std::string& tile_dir) {
  LOG_INFO("Read OSMData from temp files");

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
  bool status = read_restrictions(tile_dir + restrictions_file, restrictions) &&
                read_viaset(tile_dir + viaset_file, via_set) &&
                read_access_restrictions(tile_dir + access_restrictions_file, access_restrictions) &&
                read_bike_relations(tile_dir + bike_relations_file, bike_relations) &&
                read_way_refs(tile_dir + way_ref_file, way_ref) &&
                read_way_refs(tile_dir + way_ref_rev_file, way_ref_rev) &&
                read_node_names(tile_dir + node_names_file, node_names) &&
                read_unique_names(tile_dir + unique_names_file, name_offset_map) &&
                read_lane_connectivity(tile_dir + lane_connectivity_file, lane_connectivity_map);
  LOG_INFO("Done");
  return status;
}

// Read OSMData from temporary files
bool OSMData::read_from_unique_names_file(const std::string& tile_dir) {
  LOG_INFO("Read OSMData unique_names from temp file");

  // Read the other data
  bool status = read_unique_names(tile_dir + unique_names_file, name_offset_map);
  LOG_INFO("Done");
  return status;
}

// add the direction information to the forward or reverse map for relations.
void OSMData::add_to_name_map(const uint32_t member_id,
                              const std::string& direction,
                              const std::string& reference,
                              const bool forward) {

  std::string dir = direction;
  boost::algorithm::to_lower(dir);
  dir[0] = std::toupper(dir[0]);

  // TODO:  network=e-road with int_ref=E #
  if ((boost::starts_with(dir, "North (") || boost::starts_with(dir, "South (") ||
       boost::starts_with(dir, "East (") || boost::starts_with(dir, "West (")) ||
      dir == "North" || dir == "South" || dir == "East" || dir == "West") {

    if (forward) {
      auto iter = way_ref.find(member_id);
      if (iter != way_ref.end()) {
        std::string ref = name_offset_map.name(iter->second);
        way_ref[member_id] = name_offset_map.index(ref + ";" + reference + "|" + dir);
      } else {
        way_ref[member_id] = name_offset_map.index(reference + "|" + dir);
      }
    } else {
      auto iter = way_ref_rev.find(member_id);
      if (iter != way_ref_rev.end()) {
        std::string ref = name_offset_map.name(iter->second);
        way_ref_rev[member_id] = name_offset_map.index(ref + ";" + reference + "|" + dir);
      } else {
        way_ref_rev[member_id] = name_offset_map.index(reference + "|" + dir);
      }
    }
  }
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
  remove_temp_file(tile_dir + way_ref_file);
  remove_temp_file(tile_dir + way_ref_rev_file);
  remove_temp_file(tile_dir + node_names_file);
  remove_temp_file(tile_dir + unique_names_file);
  remove_temp_file(tile_dir + lane_connectivity_file);
}

} // namespace mjolnir
} // namespace valhalla
