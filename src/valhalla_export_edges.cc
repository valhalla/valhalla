#include <cstdint>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "baldr/graphconstants.h"
#include "baldr/graphreader.h"
#include "baldr/tilehierarchy.h"
#include "midgard/logging.h"
#include "midgard/encoded.h"

#include <unordered_map>
#include <algorithm>
#include <iostream>

#include "config.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace bpo = boost::program_options;

//global options instead of passing them around
std::string column_separator{'\0'};
std::string row_separator = "\n";
std::string config;
bool ferries;
bool unnamed;

namespace {

//a place we can mark what edges we've seen, even for the planet we should need < 100mb
struct bitset_t {
  bitset_t(size_t size) {bits.resize(std::ceil(size / 64.0));}
  void set(const uint64_t id) {
    if (id >= bits.size() * 64) throw std::runtime_error("id out of bounds");
    bits[id / 64] |= static_cast<uint64_t>(1) << (id % static_cast<uint64_t>(64));
  }
  bool get(const uint64_t id) const {
    if (id >= bits.size() * 64) throw std::runtime_error("id out of bounds");
    return bits[id / 64] & (static_cast<uint64_t>(1) << (id % static_cast<uint64_t>(64)));
  }
protected:
  std::vector<uint64_t> bits;
};

//often we need both the edge id and the directed edge, so lets have something to represent that
struct edge_t {
  GraphId i;
  const DirectedEdge* e;
  operator const GraphId&() const { return i; }
  operator const DirectedEdge*() const { return e; }
  operator bool() const { return i.Is_Valid() && e; }
};

// Get the opposing edge - if the opposing index is invalid return a nullptr
// for the directed edge. This should not occur but this can happen in
// GraphValidator if it fails to find an opposing edge.
edge_t opposing(GraphReader& reader, const GraphTile* tile, const DirectedEdge* edge) {
  const GraphTile* t = edge->leaves_tile() ? reader.GetGraphTile(edge->endnode()) : tile;
  auto id = edge->endnode();
  id.set_id(t->node(id)->edge_index() + edge->opp_index());

  // Check for invalid opposing index
  if (edge->opp_index() == kMaxEdgesPerNode) {
    PointLL ll = t->node(edge->endnode())->latlng();
    LOG_ERROR("Invalid edge opp index = " + std::to_string(edge->opp_index()) +
              " LL = " + std::to_string(ll.lat()) + "," + std::to_string(ll.lng()));
    return {id, nullptr};
  }
  return {id, t->directededge(id)};
}

edge_t next(const std::unordered_map<GraphId, uint64_t>& tile_set, const bitset_t& edge_set, GraphReader& reader,
            const GraphTile*& tile, const edge_t& edge, const std::vector<std::string>& names) {
  //get the right tile
  if(tile->id() != edge.e->endnode().Tile_Base())
    tile = reader.GetGraphTile(edge.e->endnode());

  //TODO: in the case of multiple candidates favor the ones with angles that are most straight

  //check all the edges here
  const auto* node = tile->node(edge.e->endnode());
  for(size_t i = 0; i < node->edge_count(); ++i) {
    //get the edge
    GraphId id = tile->id();
    id.set_id(node->edge_index() + i);
    //already used
    if(edge_set.get(tile_set.find(tile->id())->second + id.id()))
      continue;
    edge_t candidate{id, tile->directededge(id)};
    //dont need these
    if(!ferries && candidate.e->use() == Use::kFerry)
      continue;
    //TODO: dont skip transition edges but rather follow them to other levels
    //skip these
    if(candidate.e->trans_up() || candidate.e->use() == Use::kTransitConnection ||
       candidate.e->trans_down() || candidate.e->IsTransitLine()) //these should never happen
      continue;
    //names have to match
    auto candidate_names = tile->edgeinfo(candidate.e->edgeinfo_offset()).GetNames();
    if(names.size() == candidate_names.size() && std::equal(names.cbegin(), names.cend(), candidate_names.cbegin()))
      return candidate;
  }

  return {};
}

void extend(GraphReader& reader, const GraphTile*& tile, const edge_t& edge, std::list<PointLL>& shape) {
  //get the shape
  if(edge.i.Tile_Base() != tile->id())
    tile = reader.GetGraphTile(edge.i);
  //get the shape
  auto info = tile->edgeinfo(edge.e->edgeinfo_offset());
  auto more = valhalla::midgard::decode7<std::list<PointLL> >(info.encoded_shape());
  //this shape runs the other way
  if(!edge.e->forward())
    more.reverse();
  //connecting another shape we dont want dups where they meet
  if(shape.size())
    more.pop_front();
  shape.splice(shape.end(), more);
}

}

//program entry point
int main(int argc, char *argv[]) {
  bpo::options_description options("valhalla_export_edges " VERSION "\n"
  "\n"
  " Usage: valhalla_export_edges [options]\n"
  "\n"
  "valhalla_export_edges is a simple command line test tool which dumps information about each graph edge. "
  "\n"
  "\n");

  options.add_options()
      ("help,h", "Print this help message.")
      ("version,v", "Print the version of this software.")
      ("column,c", bpo::value<std::string>(&column_separator), "What separator to use between columns [default=\\0].")
      ("row,r", bpo::value<std::string>(&column_separator), "What separator to use between row [default=\\n].")
      ("ferries,f", "Export ferries as well [default=false]")
      ("unnamed,u", "Export unnamed edges as well [default=false]")
      // positional arguments
      ("config", bpo::value<std::string>(&config), "Valhalla configuration file [required]");


  bpo::positional_options_description pos_options;
  pos_options.add("config", 1);
  bpo::variables_map vm;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(options).positional(pos_options).run(), vm);
    bpo::notify(vm);
  }
  catch (std::exception &e) {
    std::cerr << "Unable to parse command line options because: " << e.what()
              << "\n" << "This is a bug, please report it at " PACKAGE_BUGREPORT
              << "\n";
    return EXIT_FAILURE;
  }

  if (vm.count("help") || !vm.count("config")) {
    std::cout << options << "\n";
    return EXIT_SUCCESS;
  }

  if (vm.count("version")) {
    std::cout << "valhalla_export_edges " << VERSION << "\n";
    return EXIT_SUCCESS;
  }

  bool ferries = vm.count("ferries");
  bool unnamed = vm.count("unnamed");

  //parse the config
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(config.c_str(), pt);

  //configure logging
  valhalla::midgard::logging::Configure({{"type","std_err"},{"color","true"}});

  //get something we can use to fetch tiles
  valhalla::baldr::GraphReader reader(pt.get_child("mjolnir"));

  //keep the global number of edges encountered at the point we encounter each tile
  //this allows an edge to have a sequential global id and makes storing it very small
  LOG_INFO("Enumerating edges...");
  std::unordered_map<GraphId, uint64_t> tile_set(kMaxGraphTileId * TileHierarchy::levels().size());
  uint64_t edge_count = 0;
  for(const auto& level : TileHierarchy::levels()) {
    for(uint32_t i = 0; i < level.second.tiles.TileCount(); ++i) {
      GraphId tile_id{i, level.first, 0};
      if(reader.DoesTileExist(tile_id)) {
        //TODO: just read the header, parsing the whole thing isnt worth it at this point
        tile_set.emplace(tile_id, edge_count);
        const auto* tile = reader.GetGraphTile(tile_id);
        edge_count += tile->header()->directededgecount();
        reader.Clear();
      }
    }
  }

  //this is how we know what i've touched and what we havent
  bitset_t edge_set(edge_count);

  //TODO: we could parallelize this and it might be quite a bit faster but if we really want to maximize
  //continuous edges we need to avoid the lady and the tramp scenario where two threads are consuming the same
  //stretch of road at the same time

  //for each tile
  LOG_INFO("Exporting " + std::to_string(edge_count) + " edges");
  int progress = -1;
  uint64_t set = 0;
  for(const auto& tile_count_pair : tile_set) {
    //for each edge in the tile
    reader.Clear();
    const auto* tile = reader.GetGraphTile(tile_count_pair.first);
    for(uint32_t i = 0; i < tile->header()->directededgecount(); ++i) {
      //we've seen this one already
      if(edge_set.get(tile_count_pair.second + i))
        continue;

      //TODO: dont mark transition edges since we may need to use them to change levels multiple times
      //maybe we should mark them though once every normal edge connected there has been marked

      //make sure we dont ever look at this again
      edge_t edge{tile_count_pair.first, tile->directededge(i)};
      edge.i.set_id(i);
      edge_set.set(tile_count_pair.second + i);
      ++set;

      //these wont have opposing edges that we care about
      if(edge.e->trans_up() || edge.e->use() == Use::kTransitConnection ||
         edge.e->trans_down() || edge.e->IsTransitLine()) //these 2 should never happen
        continue;

      //get the opposing edge as well (ensure a valid edge is returned)
      edge_t opposing_edge = opposing(reader, tile, edge);
      if (opposing_edge.e == nullptr) {
        continue;
      }
      edge_set.set(tile_set.find(opposing_edge.i.Tile_Base())->second + opposing_edge.i.id());
      ++set;

      //shortcuts arent real and maybe we dont want ferries
      if(edge.e->is_shortcut() || (!ferries && edge.e->use() == Use::kFerry))
        continue;

      //no name no thanks
      auto edge_info = tile->edgeinfo(edge.e->edgeinfo_offset());
      auto names = edge_info.GetNames();
      if(names.size() == 0 && !unnamed)
        continue;

      //TODO: at this point we need to traverse the graph from this edge to build a subgraph of like-named
      //connected edges. what we would like is that from that subgraph we extract linestrings which are of
      //the maximum length. this makes people's lives easier downstream. finding such segments is NP-Hard
      //and indeed even verifying a solution is NP-Complete. there are some tricks though.. you can do this
      //in linear time if your subgraph is a DAG. this can't be guaranteed in the overall graph, but we can
      //create the subgraphs in such a way that they are DAGs. this can produce suboptimal results however
      //and depends on the initial edge. so for now we'll just greedily export edges

      //keep some state about this section of road
      std::list<edge_t> edges {edge};

      //go forward
      const auto* t = tile;
      while((edge = next(tile_set, edge_set, reader, t, edge, names))) {
        //mark them to never be used again
        edge_set.set(tile_set.find(edge.i.Tile_Base())->second + edge.i.id());
        edge_t other = opposing(reader, t, edge);
        if (other.e == nullptr) {
          continue;
        }
        edge_set.set(tile_set.find(other.i.Tile_Base())->second + other.i.id());
        set += 2;
        //keep this
        edges.push_back(edge);
      }

      //go backward
      edge = opposing_edge;
      while((edge = next(tile_set, edge_set, reader, t, edge, names))) {
        //mark them to never be used again
        edge_set.set(tile_set.find(edge.i.Tile_Base())->second + edge.i.id());
        edge_t other = opposing(reader, t, edge);
        if (other.e == nullptr) {
          continue;
        }
        edge_set.set(tile_set.find(other.i.Tile_Base())->second + other.i.id());
        set += 2;
        //keep this
        edges.push_front(other);
      }

      //get the shape
      std::list<PointLL> shape;
      for(const auto& e : edges)
        extend(reader, t, e, shape);

      //output it as: shape,name,name,...
      auto encoded = encode(shape);
      std::cout << encoded << column_separator;
      for(const auto& name : names)
        std::cout << name << (&name == &names.back() ? "" : column_separator);
      std::cout << row_separator;
      std::cout.flush();

    }

    //check progress
    int procent = (100.f * set) / edge_count;
    if(procent > progress)
      LOG_INFO(std::to_string(progress = procent) + "%");
  }
  LOG_INFO("Done");

  for(uint64_t i = 0; i < edge_count; ++i) {
    if(!edge_set.get(i)) {
      LOG_INFO(std::to_string(i));
      break;
    }
  }

  return EXIT_SUCCESS;
}

