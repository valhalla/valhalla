// -*- mode: c++ -*-

#include <sqlite3.h>
#include <unordered_map>

// boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/tilehierarchy.h>

using namespace valhalla;


// element (edge/node) id -> count
using CountMap = std::unordered_map<uint64_t, uint16_t>;

// tile id -> count map
using ScoreMap = std::unordered_map<uint32_t, CountMap>;


class DirectededgeAttacher: public baldr::DirectedEdge
{
 public:
  void set_photocount(uint16_t count)
  { attributes_.photocount = count; }
};


class GraphTileAttacher: public baldr::GraphTile
{
 public:
  GraphTileAttacher(const baldr::TileHierarchy& hierarchy,
                    const baldr::GraphId& graphid)
      : GraphTile(hierarchy, graphid),
        hierarchy_(hierarchy),
        graphid_(graphid) {}

  // Cast the internal directededge pointer to directededgeattacher
  // pointer that is able to set photo count
  DirectededgeAttacher* directededgeattachers()
  { return static_cast<DirectededgeAttacher*>(directededges_); }

  // Write only directededges back into the tile
  void UpdateDirectedEdges()
  {
    auto directededge_pos = sizeof(baldr::GraphTileHeader) + sizeof(baldr::NodeInfo) * header_->nodecount();
    auto file = OpenFile();
    file.seekp(directededge_pos);
    file.write(reinterpret_cast<const char*>(directededges_),
               (header_->directededgecount() * sizeof(baldr::DirectedEdge)) / sizeof(char));
    file.close();
  }

  void Verify() const
  {
    baldr::GraphTile newtile(hierarchy_, graphid_);
    // TODO compare graphtile_ byte by byte
  }

 private:
  baldr::TileHierarchy hierarchy_;
  baldr::GraphId graphid_;

  // Open the existing tile file for writing
  std::ofstream OpenFile() const
  {
    // Get the name of the file
    std::string filename = hierarchy_.tile_dir()
                           + '/'
                           + FileSuffix(graphid_.Tile_Base(), hierarchy_);
    // Open file and don't overwrite
    std::ofstream file(filename, std::ios::in | std::ios::out | std::ios::binary);

    if (!file.is_open()) {
      throw std::runtime_error("Failed to open file " + filename);
    }

    return file;
  }
};


bool aggregate_scores(sqlite3* db_handle, ScoreMap& score_map)
{
  sqlite3_stmt* stmt;
  std::string sql = "SELECT sequence_id, coordinate_index, graphid FROM scores";
  int ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, nullptr);
  if (SQLITE_OK != ret) {
    LOG_ERROR("Failed to prepare " + sql);
    return false;
  }
  do {
    ret = sqlite3_step(stmt);
    if (SQLITE_ROW == ret) {
      baldr::GraphId graphid(static_cast<uint64_t>(sqlite3_column_int64(stmt, 2)));
      if (graphid.Is_Valid()) {
        auto count = score_map[graphid.tileid()][graphid.id()];
        if (count < std::numeric_limits<decltype(count)>::max()) {
          score_map[graphid.tileid()][graphid.id()]++;
        } else {
          // Very likely to be an error
          LOG_ERROR("The photo count on " + std::to_string(graphid) + " exceeds the max limit");
        }
      } else {
        LOG_ERROR("Found invalid graphid " + std::to_string(graphid));
      }
    }
  } while (SQLITE_ROW == ret || SQLITE_BUSY == ret);

  if (SQLITE_DONE != ret) {
    LOG_ERROR("Expect SQLITE_DONE to return, but you got " + std::to_string(ret));
    sqlite3_finalize(stmt);
    return false;
  }

  sqlite3_finalize(stmt);
  return true;
}


void write_scores(const baldr::TileHierarchy& tile_hierarchy,
                  const ScoreMap& score_map)
{
  auto local_level = tile_hierarchy.levels().rbegin()->second.level;
  for (const auto scorepair : score_map) {
    auto tileid = scorepair.first;
    const baldr::GraphId graphid(tileid, local_level, 0);
    if (baldr::GraphReader::DoesTileExist(tile_hierarchy, graphid)) {
      GraphTileAttacher tileattacher(tile_hierarchy, graphid);
      auto directededgeattachers = tileattacher.directededgeattachers();
      auto directededgecount = tileattacher.header()->directededgecount();
      for (const auto countpair : scorepair.second) {
        auto idx = static_cast<size_t>(countpair.first);
        if (idx < directededgecount) {
          directededgeattachers[idx].set_photocount(countpair.second);
        } else {
          LOG_ERROR("Tile " + std::to_string(tileid) + ": DirectedEdge id " + std::to_string(idx) + "out of bounds which is " + std::to_string(directededgecount));
        }
      }
      // Write back into tiles
      tileattacher.UpdateDirectedEdges();
    }
  }
}


int main(int argc, char *argv[])
{
  if (argc < 3) {
    std::cerr << "usage: attacher CONFIG_FILENAME SQLITE3_FILENAME" << std::endl;
    return 1;
  }
  std::string config_filename = argv[1],
            sqlite3_filename  = argv[2];

  sqlite3* db_handle;
  int ret = sqlite3_open_v2(sqlite3_filename.c_str(), &db_handle, SQLITE_OPEN_READONLY, nullptr);
  if (SQLITE_OK != ret) {
    LOG_ERROR("Failed to open sqlite3 database at " + sqlite3_filename);
    sqlite3_close(db_handle);
    return 2;
  }

  ScoreMap score_map;
  bool ok = aggregate_scores(db_handle, score_map);
  sqlite3_close(db_handle);
  if (!ok) {
    return 2;
  }

  boost::property_tree::ptree pt;
  boost::property_tree::read_json(config_filename.c_str(), pt);
  baldr::GraphReader reader(pt.get_child("mjolnir.hierarchy"));
  const auto& tile_hierarchy = reader.GetTileHierarchy();

  write_scores(tile_hierarchy, score_map);

  return 0;
}
