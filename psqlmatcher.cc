// -*- mode: c++ -*-

#include <unordered_map>
#include <vector>
#include <cassert>
#include <iostream>

// boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

// psql
#include <postgresql/libpq-fe.h>
#include <pqxx/pqxx>

// geos
#include <geos/io/WKBReader.h>
#include <geos/geom/CoordinateSequence.h>
#include <geos/geom/Geometry.h>

// valhalla
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla;

#include "costings.h"
#include "map_matching.h"


namespace {
constexpr float kDefaultSigmaZ = 4.07;
constexpr float kDefaultBeta = 3;
constexpr float kDefaultSquaredSearchRadius = 25 * 25;  // 25 meters
}


using BoundingBox = midgard::AABB2<midgard::PointLL>;
using Sequence = std::vector<Measurement>;


std::string joinbbox(const BoundingBox& bbox)
{
  return std::to_string(bbox.minx())
      + " "
      + std::to_string(bbox.miny())
      + ", "
      + std::to_string(bbox.maxx())
      + " "
      + std::to_string(bbox.maxy());
}


// Convert a linestring's Well-Known Binary to a sequence (a vector of
// measurements)
Sequence to_sequence(std::istringstream& wkb)
{
  geos::geom::GeometryFactory factory;
  geos::io::WKBReader reader(factory);
  auto geometry = reader.read(wkb);
  Sequence sequence;

  auto coords = geometry->getCoordinates();
  for (decltype(coords->size()) idx = 0;
       idx < coords->size(); idx++) {
    const auto& coord = coords->getAt(idx);
    sequence.emplace_back(PointLL{coord.x, coord.y});
  }

  return sequence;
}


inline Sequence to_sequence(const std::string& wkb)
{
  std::istringstream is(wkb);
  return to_sequence(is);
}


using SequenceId = uint32_t;


// Query all sequences within the bounding box
std::unordered_map<SequenceId, Sequence>
query_sequences(pqxx::connection& conn, const BoundingBox& bbox)
{
  pqxx::work txn(conn);

  auto bbox_clause = txn.quote("BOX(" + joinbbox(bbox) + ")") + "::box2d";
  std::string statement = "SELECT id, ST_AsBinary(path_gm) AS geom FROM sequences WHERE "
                          + bbox_clause + " && path_gm"
                          + " LIMIT " + std::to_string(std::numeric_limits<uint32_t>::max());
  LOG_INFO("Querying: " + statement);

  // Send query
  auto tuples = txn.exec(statement);

  std::unordered_map<SequenceId, Sequence> sequences;
  for (auto it = tuples.begin(); it != tuples.end(); it++) {
    auto sid = it->at("id").as<SequenceId>();
    auto bs = pqxx::binarystring(it->at("geom"));
    sequences[sid] = to_sequence(bs.str());
  }

  return sequences;
}


// Collect all tile IDs
std::vector<uint32_t>
collect_local_tileids(const baldr::TileHierarchy& tile_hierarchy)
{
  auto local_level = tile_hierarchy.levels().rbegin()->second.level;
  const auto& tiles = tile_hierarchy.levels().rbegin()->second.tiles;

  std::vector<uint32_t> queue;

  for (uint32_t id = 0; id < tiles.TileCount(); id++) {
    // If tile exists add it to the queue
    GraphId tile_id(id, local_level, 0);
    if (baldr::GraphReader::DoesTileExist(tile_hierarchy, tile_id)) {
      queue.push_back(tile_id.tileid());
    }
  }

  return queue;
}


// Tell you which tile this sequence belongs to
inline uint32_t
which_tileid(const baldr::TileHierarchy& tile_hierarchy,
             const Sequence& sequence)
{
  auto local_level = tile_hierarchy.levels().rbegin()->second.level;
  auto graphid = tile_hierarchy.GetGraphId(sequence[0].lnglat(), local_level);
  if (graphid.Is_Valid()) {
    return graphid.tileid();
  }
  baldr::GraphId id;
  return id.tileid();
}


GraphId
which_graphid(const std::vector<const State*>& path,
              const std::vector<const State*>::const_iterator iterator)
{
  if (iterator != path.cend() && *iterator) {
    auto stateptr = *iterator;
    const auto prev_iterator = std::prev(iterator);
    const auto next_iterator = std::next(iterator);

    // If previous state is available, find the last edge/node id on
    // the route from previous state to current state
    if (iterator != path.cbegin()
        && prev_iterator != path.cend()
        && *prev_iterator
        && (*prev_iterator)->routed()) {
      const auto label_iterator = (*prev_iterator)->RouteBegin(*stateptr);
      if (label_iterator != (*prev_iterator)->RouteEnd()) {
        return label_iterator->nodeid.Is_Valid()?
            label_iterator->nodeid : label_iterator->edgeid;
      }
    }

    // Otherwise if next state is available, find the first edge/node
    // id on the route from current state to next state
    else if (next_iterator != path.cend()
             && *next_iterator
             && stateptr->routed()) {
      auto label_iterator = stateptr->RouteBegin(*(*next_iterator));
      GraphId last_valid_id;
      while (label_iterator != stateptr->RouteEnd()) {
        auto id = label_iterator->nodeid.Is_Valid()?
                  label_iterator->nodeid : label_iterator->edgeid;
        if (id.Is_Valid()) {
          last_valid_id = id;
        }
        label_iterator++;
      }
      return last_valid_id;
    }
  }

  // Bye!
  return GraphId();
}


// <sequence ID, coordinate index, edge ID>
using Result = std::tuple<SequenceId, uint32_t, baldr::GraphId>;


int main(int argc, char *argv[])
{
  //////////////////////////////////
  // Parse arguments
  if (argc < 4) {
    std::cerr << "usage: psqlmatcher CONF_FILE_PATH PSQL_URI SQLITE3_FILE_PATH" << std::endl;
    std::cerr << "example: psqlmatcher conf/valhalla.json \"dbname=sequence user=postgres password=secret host=localhost\"" << std::endl;
    return 1;
  }

  std::string config_file_path(argv[1]);
  std::string psql_uri(argv[2]);
  std::string sqlite3_file_path(argv[3]);

  /////////////////////////////////
  // Initialize
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(config_file_path.c_str(), pt);
  std::shared_ptr<sif::DynamicCost> mode_costing[4] = {
    nullptr, // CreateAutoCost(*config.get_child_optional("costing_options.auto")),
    nullptr, // CreateAutoShorterCost(*config.get_child_optional("costing_options.auto_shorter")),
    nullptr, // CreateBicycleCost(*config.get_child_optional("costing_options.bicycle")),
    CreatePedestrianCost(*pt.get_child_optional("costing_options.pedestrian"))
  };
  sif::TravelMode travel_mode = static_cast<sif::TravelMode>(3);
  baldr::GraphReader reader(pt.get_child("mjolnir.hierarchy"));
  // TODO read them from config
  auto sigma_z = kDefaultSigmaZ;
  auto beta = kDefaultBeta;
  MapMatching mm(sigma_z, beta, reader, mode_costing, travel_mode);

  const auto& tile_hierarchy = reader.GetTileHierarchy();
  const auto& tiles = tile_hierarchy.levels().rbegin()->second.tiles;
  auto tile_size = tiles.TileSize();
  const CandidateGridQuery grid(reader, tile_size/1000, tile_size/1000);
  LOG_INFO("Config: tile size = " + std::to_string(tile_size));
  LOG_INFO("Config: sigma_z = " + std::to_string(sigma_z));
  LOG_INFO("Config: beta = " + std::to_string(beta));

  ////////////////////////
  // Collect tiles
  const auto& tileids = collect_local_tileids(tile_hierarchy);
  LOG_INFO("The number of tiles collected: " + std::to_string(tileids.size()));

  //////////////////////
  // For each tile:
  //     1. Query all sequences within bounding box of this tile
  //     2. For each sequence:
  //            Offline match the sequence
  //     3. Insert rows [sequence's id, coordinate's index, edge's GraphId] into sqlite3

  uint64_t stat_matched_count_totally = 0;
  uint64_t stat_measurement_count_totally = 0;

  pqxx::connection conn(psql_uri.c_str());
  for (auto tileid : tileids) {
    std::vector<Result> results;

    uint32_t stat_matched_count_of_tile = 0;
    uint32_t stat_measurement_count_of_tile = 0;
    const auto& bbox = tiles.TileBounds(tileid);
    for (const auto& sequencepair : query_sequences(conn, bbox)) {
      auto sid = sequencepair.first;
      const auto& sequence = sequencepair.second;
      // Too verbose
      // LOG_INFO("Got sequence: id = " + std::to_string(sid) + " size = " + std::to_string(sequence.size()));

      if (sequence.empty()) {
        continue;
      }

      // Skip sequences that don't belong to this tile
      if (which_tileid(tile_hierarchy, sequence) != tileid) {
        continue;
      }

      const auto& path = OfflineMatch(mm, grid, sequence, kDefaultSquaredSearchRadius);
      assert (path.size() == sequence.size());

      uint32_t stat_matched_count_of_sequence = 0;
      uint32_t coord_idx = 0;
      for (auto state = path.cbegin(); state != path.cend(); state++, coord_idx++) {
        if (*state) {
          results.emplace_back(sid, coord_idx, which_graphid(path, state));
          stat_matched_count_of_sequence++;
        }
      }

      stat_matched_count_of_tile += stat_matched_count_of_sequence;
      stat_measurement_count_of_tile += sequence.size();
      // Too verbose
      // LOG_INFO("Matched " + std::to_string(stat_matched_count_of_sequence) + "/" + std::to_string(sequence.size()));
    }

    stat_matched_count_totally += stat_matched_count_of_tile;
    stat_measurement_count_totally += stat_measurement_count_of_tile;
    LOG_INFO("Summary of tile " + std::to_string(tileid) + ": matched " + std::to_string(stat_matched_count_of_tile) + "/" + std::to_string(stat_measurement_count_of_tile));
  }

  LOG_INFO("============= Summary ==================");
  LOG_INFO("Matched: " + std::to_string(stat_matched_count_totally) + "/" + std::to_string(stat_measurement_count_totally));
  return 0;
}
