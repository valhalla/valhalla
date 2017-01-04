#include "baldr/graphtile.h"
#include "baldr/datetime.h"
#include <valhalla/midgard/tiles.h>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/logging.h>

#include <ctime>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <locale>
#include <iomanip>
#include <cmath>
#include <boost/algorithm/string.hpp>

namespace {
  struct dir_facet : public std::numpunct<char> {
   protected:
    virtual char do_thousands_sep() const {
        return '/';
    }

    virtual std::string do_grouping() const {
        return "\03";
    }
  };
  template <class numeric_t>
  size_t digits(numeric_t number) {
    size_t digits = (number < 0 ? 1 : 0);
    while (static_cast<long long int>(number)) {
        number /= 10;
        digits++;
    }
    return digits;
  }
  const std::locale dir_locale(std::locale("C"), new dir_facet());
  const AABB2<PointLL> world_box(PointLL(-180, -90), PointLL(180, 90));
}

namespace valhalla {
namespace baldr {

// Default constructor
GraphTile::GraphTile()
    : header_(nullptr),
      nodes_(nullptr),
      directededges_(nullptr),
      departures_(nullptr),
      transit_stops_(nullptr),
      transit_routes_(nullptr),
      transit_schedules_(nullptr),
      transit_transfers_(nullptr),
      access_restrictions_(nullptr),
      signs_(nullptr),
      admins_(nullptr),
      edge_bins_(nullptr),
      complex_restriction_forward_(nullptr),
      complex_restriction_reverse_(nullptr),
      edgeinfo_(nullptr),
      textlist_(nullptr),
      complex_restriction_forward_size_(0),
      complex_restriction_reverse_size_(0),
      edgeinfo_size_(0),
      textlist_size_(0),
      traffic_segments_(nullptr),
      traffic_chunks_(0),
      traffic_chunk_size_(0) {
}

// Constructor given a filename. Reads the graph data into memory.
GraphTile::GraphTile(const TileHierarchy& hierarchy, const GraphId& graphid): header_(nullptr) {

  // Don't bother with invalid ids
  if (!graphid.Is_Valid())
    return;

  // Open to the end of the file so we can immediately get size;
  std::string file_location = hierarchy.tile_dir() + "/" +
                FileSuffix(graphid.Tile_Base(), hierarchy);
  std::ifstream file(file_location, std::ios::in | std::ios::binary | std::ios::ate);
  if (file.is_open()) {
    // Read binary file into memory. TODO - protect against failure to
    // allocate memory
    size_t filesize = file.tellg();
    graphtile_.reset(new char[filesize]);
    file.seekg(0, std::ios::beg);
    file.read(graphtile_.get(), filesize);
    file.close();

    // Set pointers to internal data structures
    Initialize(graphid, graphtile_.get(), filesize);
  }
  else {
    LOG_DEBUG("Tile " + file_location + " was not found");
  }
}

GraphTile::GraphTile(const GraphId& graphid, char* ptr, size_t size): header_(nullptr) {
  // Initialize the internal tile data structures using a pointer to the
  // tile and the tile size
  Initialize(graphid, ptr, size);
}

GraphTile::~GraphTile() {
}

// Set pointers to internal tile data structures
void GraphTile::Initialize(const GraphId& graphid, char* tile_ptr,
                           const size_t tile_size) {
  char* ptr = tile_ptr;
  header_ = reinterpret_cast<GraphTileHeader*>(ptr);
  ptr += sizeof(GraphTileHeader);

  // TODO check version

  // Set a pointer to the node list
  nodes_ = reinterpret_cast<NodeInfo*>(ptr);
  ptr += header_->nodecount() * sizeof(NodeInfo);

  // Set a pointer to the directed edge list
  directededges_ = reinterpret_cast<DirectedEdge*>(ptr);
  ptr += header_->directededgecount() * sizeof(DirectedEdge);

  // Set a pointer access restriction list
  access_restrictions_ = reinterpret_cast<AccessRestriction*>(ptr);
  ptr += header_->access_restriction_count() * sizeof(AccessRestriction);

  // Set a pointer to the transit departure list
  departures_ = reinterpret_cast<TransitDeparture*>(ptr);
  ptr += header_->departurecount() * sizeof(TransitDeparture);

  // Set a pointer to the transit stop list
  transit_stops_ = reinterpret_cast<TransitStop*>(ptr);
  ptr += header_->stopcount() * sizeof(TransitStop);

  // Set a pointer to the transit route list
  transit_routes_ = reinterpret_cast<TransitRoute*>(ptr);
  ptr += header_->routecount() * sizeof(TransitRoute);

  // Set a pointer to the transit schedule list
  transit_schedules_ = reinterpret_cast<TransitSchedule*>(ptr);
  ptr += header_->schedulecount() * sizeof(TransitSchedule);

  // Set a pointer to the transit transfer list
  transit_transfers_ = reinterpret_cast<TransitTransfer*>(ptr);
  ptr += header_->transfercount() * sizeof(TransitTransfer);

  // Set a pointer to the sign list
  signs_ = reinterpret_cast<Sign*>(ptr);
  ptr += header_->signcount() * sizeof(Sign);

  // Set a pointer to the admininstrative information list
  admins_ = reinterpret_cast<Admin*>(ptr);
  ptr += header_->admincount() * sizeof(Admin);

  // Set a pointer to the edge bin list
  edge_bins_ = reinterpret_cast<GraphId*>(ptr);

  // Start of forward restriction information and its size
  complex_restriction_forward_ = tile_ptr + header_->complex_restriction_forward_offset();
  complex_restriction_forward_size_ =
      header_->complex_restriction_reverse_offset() - header_->complex_restriction_forward_offset();

  // Start of reverse restriction information and its size
  complex_restriction_reverse_ = tile_ptr + header_->complex_restriction_reverse_offset();
  complex_restriction_reverse_size_ =
      header_->edgeinfo_offset() - header_->complex_restriction_reverse_offset();

  // Start of edge information and its size
  edgeinfo_ = tile_ptr + header_->edgeinfo_offset();
  edgeinfo_size_ = header_->textlist_offset() - header_->edgeinfo_offset();

  // Start of text list and its size
  textlist_ = tile_ptr + header_->textlist_offset();
  textlist_size_ = header_->traffic_segmentid_offset() - header_->textlist_offset();

  // Start of the traffic segment association records
  traffic_segments_ = reinterpret_cast<TrafficAssociation*>(tile_ptr +
                          header_->traffic_segmentid_offset());

  // Start of traffic chunks and their size
  // TODO - update chunk definition...
  traffic_chunks_ = reinterpret_cast<uint64_t*>(tile_ptr + header_->traffic_chunk_offset());
  traffic_chunk_size_ = header_->end_offset() - header_->traffic_chunk_offset();

  // ANY NEW EXPANSION DATA GOES HERE

  // Associate one stop Ids for transit tiles
  if (graphid.level() == 3) {
    AssociateOneStopIds(graphid);
  }
}

// For transit tiles we need to save off the pair<tileid,lineid> lookup via
// onestop_ids.  This will be used for including or excluding transit lines
// for transit routes.  We save 2 maps because operators contain all of their
// route's tile_line pairs and it is used to include or exclude the operator
// as a whole. Also associates stops.
void GraphTile::AssociateOneStopIds(const GraphId& graphid) {
  // Associate stop Ids
  stop_one_stops.reserve(header_->stopcount());
  for (uint32_t i = 0; i < header_->stopcount(); i++) {
    const auto& stop = GetName(transit_stops_[i].one_stop_offset());
    stop_one_stops[stop] = tile_index_pair(graphid.tileid(), i);
  }

  // Associate route and operator Ids
  auto deps = GetTransitDepartures();
  for (auto const& dep: deps) {
    const auto* t = GetTransitRoute(dep.second->routeid());
    const auto& route_one_stop = GetName(t->one_stop_offset());
    auto stops = route_one_stops.find(route_one_stop);
    if (stops == route_one_stops.end()) {
      std::list<tile_index_pair> tile_line_ids;
      tile_line_ids.emplace_back(tile_index_pair(graphid.tileid(), dep.second->lineid()));
      route_one_stops[route_one_stop] = tile_line_ids;
    } else {
      route_one_stops[route_one_stop].emplace_back(tile_index_pair(graphid.tileid(), dep.second->lineid()));
    }

    // operators contain all of their route's tile_line pairs.
    const auto& op_one_stop = GetName(t->op_by_onestop_id_offset());
    stops = oper_one_stops.find(op_one_stop);
    if (stops == oper_one_stops.end()) {
      std::list<tile_index_pair> tile_line_ids;
      tile_line_ids.emplace_back(tile_index_pair(graphid.tileid(), dep.second->lineid()));
      oper_one_stops[op_one_stop] = tile_line_ids;
    } else {
      oper_one_stops[op_one_stop].emplace_back(tile_index_pair(graphid.tileid(), dep.second->lineid()));
    }
  }
}

std::string GraphTile::FileSuffix(const GraphId& graphid, const TileHierarchy& hierarchy) {
  /*
  if you have a graphid where level == 8 and tileid == 24134109851
  you should get: 8/024/134/109/851.gph
  since the number of levels is likely to be very small this limits
  the total number of objects in any one directory to 1000, which is an
  empirically derived good choice for mechanical harddrives
  this should be fine for s3 (even though it breaks the rule of most
  unique part of filename first) because there will be just so few
  objects in general in practice
  */

  //figure the largest id for this level
  auto level = hierarchy.levels().find(graphid.level());
  if(level == hierarchy.levels().end() &&
     graphid.level() == ((hierarchy.levels().rbegin())->second.level + 1))
    level = hierarchy.levels().begin();

  if(level == hierarchy.levels().end())
    throw std::runtime_error("Could not compute FileSuffix for non-existent level");

  const uint32_t max_id = Tiles<PointLL>::MaxTileId(world_box, level->second.tiles.TileSize());

  //figure out how many digits
  //TODO: dont convert it to a string to get the length there are faster ways..
  size_t max_length = digits<uint32_t>(max_id);
  const size_t remainder = max_length % 3;
  if(remainder)
    max_length += 3 - remainder;

  //make a locale to use as a formatter for numbers
  std::ostringstream stream;
  stream.imbue(dir_locale);

  //if it starts with a zero the pow trick doesn't work
  if(graphid.level() == 0) {
    stream << static_cast<uint32_t>(std::pow(10, max_length)) + graphid.tileid() << ".gph";
    std::string suffix = stream.str();
    suffix[0] = '0';
    return suffix;
  }
  //it was something else
  stream << graphid.level() * static_cast<uint32_t>(std::pow(10, max_length)) + graphid.tileid() << ".gph";
  return stream.str();
}

// Get the tile Id given the full path to the file.
GraphId GraphTile::GetTileId(const std::string& fname) {
  //from the front and back strip off anything that isnt a number or a slash, lose the junk
  auto name = fname;
  boost::algorithm::trim_if(name, [](char c){ return c == '/' || !std::isdigit(c); });

  //split on slash
  std::vector<std::string> tokens;
  boost::split(tokens, name, boost::is_any_of("/"));

  //need at least level and id
  if(tokens.size() < 2)
    throw std::runtime_error("Invalid tile path");

  // Compute the Id
  uint32_t id = 0;
  uint32_t multiplier = std::pow(1000, tokens.size() - 2);
  bool first = true;
  for(const auto& token : tokens) {
    if(first) {
      first = false;
      continue;
    }
    id += std::atoi(token.c_str()) * multiplier;
    multiplier /= 1000;
  }
  uint32_t level = std::atoi(tokens.front().c_str());
  return {id, level, 0};
}

// Get the bounding box of this graph tile.
AABB2<PointLL> GraphTile::BoundingBox(const TileHierarchy& hierarchy) const {

  //figure the largest id for this level
  auto level = hierarchy.levels().find(header_->graphid().level());
  if(level == hierarchy.levels().end() &&
      header_->graphid().level() == ((hierarchy.levels().rbegin())->second.level+1))
    level = hierarchy.levels().begin();

  auto tiles = level->second.tiles;
  return tiles.TileBounds(header_->graphid().tileid());
}

GraphId GraphTile::id() const {
  return header_->graphid();
}

const GraphTileHeader* GraphTile::header() const {
  return header_;
}

const NodeInfo* GraphTile::node(const GraphId& node) const {
  if (node.id() < header_->nodecount())
    return &nodes_[node.id()];
  throw std::runtime_error("GraphTile NodeInfo index out of bounds: " +
                             std::to_string(node.tileid()) + "," +
                             std::to_string(node.level()) + "," +
                             std::to_string(node.id()) + " nodecount= " +
                             std::to_string(header_->nodecount()));
}

const NodeInfo* GraphTile::node(const size_t idx) const {
  if (idx < header_->nodecount())
    return &nodes_[idx];
  throw std::runtime_error("GraphTile NodeInfo index out of bounds: " +
                           std::to_string(header_->graphid().tileid()) + "," +
                           std::to_string(header_->graphid().level()) + "," +
                           std::to_string(idx)  + " nodecount= " +
                           std::to_string(header_->nodecount()));
}

// Get the directed edge given a GraphId
const DirectedEdge* GraphTile::directededge(const GraphId& edge) const {
  if (edge.id() < header_->directededgecount())
    return &directededges_[edge.id()];
  throw std::runtime_error("GraphTile DirectedEdge index out of bounds: " +
                           std::to_string(header_->graphid().tileid()) + "," +
                           std::to_string(header_->graphid().level()) + "," +
                           std::to_string(edge.id())  + " directededgecount= " +
                           std::to_string(header_->directededgecount()));
}

// Get the directed edge at the specified index.
const DirectedEdge* GraphTile::directededge(const size_t idx) const {
  if (idx < header_->directededgecount())
    return &directededges_[idx];
  throw std::runtime_error("GraphTile DirectedEdge index out of bounds: " +
                           std::to_string(header_->graphid().tileid()) + "," +
                           std::to_string(header_->graphid().level()) + "," +
                           std::to_string(idx)  + " directededgecount= " +
                           std::to_string(header_->directededgecount()));
}

// Convenience method to get opposing edge Id given a directed edge.
// The end node of the directed edge must be in this tile.
GraphId GraphTile::GetOpposingEdgeId(const DirectedEdge* edge) const {
  GraphId endnode = edge->endnode();
  return { endnode.tileid(), endnode.level(),
           node(endnode.id())->edge_index() + edge->opp_index() };
}

// Get a pointer to edge info.
EdgeInfo GraphTile::edgeinfo(const size_t offset) const {
  return EdgeInfo(edgeinfo_ + offset, textlist_, textlist_size_);
}

// Get the complex restrictions in the forward or reverse order based on
// the id and modes.
std::vector<ComplexRestriction> GraphTile::GetRestrictions(const bool forward,
                                                           const GraphId id,
                                                           const uint64_t modes) const {
  std::vector<ComplexRestriction> cr_vector;
  size_t offset = 0;

  if (forward) {
    while (offset < complex_restriction_forward_size_) {

      ComplexRestriction cr(complex_restriction_forward_ + offset);
      offset += cr.SizeOf();
      if (cr.to_id() == id && (cr.modes() & modes))
        cr_vector.push_back(cr);
    }
  } else {
    while (offset < complex_restriction_reverse_size_) {

      ComplexRestriction cr(complex_restriction_reverse_ + offset);
      offset += cr.SizeOf();
      if (cr.from_id() == id && (cr.modes() & modes))
        cr_vector.push_back(cr);
    }
  }
  return cr_vector;
}

// Get the directed edges outbound from the specified node index.
const DirectedEdge* GraphTile::GetDirectedEdges(const uint32_t node_index,
                                                uint32_t& count,
                                                uint32_t& edge_index) const {
  const NodeInfo* nodeinfo = node(node_index);
  count = nodeinfo->edge_count();
  edge_index = nodeinfo->edge_index();
  return directededge(nodeinfo->edge_index());
}

// Convenience method to get the names for an edge given the offset to the
// edge info
std::vector<std::string> GraphTile::GetNames(const uint32_t edgeinfo_offset) const {
  return edgeinfo(edgeinfo_offset).GetNames();
}

// Get the admininfo at the specified index.
AdminInfo GraphTile::admininfo(const size_t idx) const {
  if (idx < header_->admincount()) {
    const Admin& admin = admins_[idx];
    return AdminInfo(textlist_ + admin.country_offset(),
                     textlist_ + admin.state_offset(),
                     admin.country_iso(), admin.state_iso());
  }
  throw std::runtime_error("GraphTile AdminInfo index out of bounds");
}

// Get the admin at the specified index.
const Admin* GraphTile::admin(const size_t idx) const {
  if (idx < header_->admincount()) {
    return &admins_[idx];
  }
  throw std::runtime_error("GraphTile Admin index out of bounds");
}

// Convenience method to get the text/name for a given offset to the textlist
std::string GraphTile::GetName(const uint32_t textlist_offset) const {

  if (textlist_offset < textlist_size_) {
    return textlist_ + textlist_offset;
  } else {
    throw std::runtime_error("GetName: offset exceeds size of text list");
  }
}

// Convenience method to get the signs for an edge given the
// directed edge index.
std::vector<SignInfo> GraphTile::GetSigns(const uint32_t idx) const {
  uint32_t count = header_->signcount();
  std::vector<SignInfo> signs;
  if (count == 0) {
    return signs;
  }

  // Signs are sorted by edge index.
  // Binary search to find a sign with matching edge index.
  int32_t low = 0;
  int32_t high = count-1;
  int32_t mid;
  int32_t found = count;
  while (low <= high) {
    mid = (low + high) / 2;
    const auto& sign = signs_[mid];
    //matching edge index
    if (idx == sign.edgeindex()) {
      found = mid;
      high = mid - 1;
    }//need a smaller index
    else if (idx < sign.edgeindex()) {
      high = mid - 1;
    }//need a bigger index
    else {
      low = mid + 1;
    }
  }

  // Add signs
  for(; found < count && signs_[found].edgeindex() == idx; ++found) {
    if (signs_[found].text_offset() < textlist_size_)
      signs.emplace_back(signs_[found].type(), (textlist_ + signs_[found].text_offset()));
    else
      throw std::runtime_error("GetSigns: offset exceeds size of text list");
  }
  if (signs.size() == 0)
    LOG_ERROR("No signs found for idx = " + std::to_string(idx));
  return signs;
}

// Get the next departure given the directed line Id and the current
// time (seconds from midnight).
const TransitDeparture* GraphTile::GetNextDeparture(const uint32_t lineid,
                 const uint32_t current_time, const uint32_t day,
                 const uint32_t dow, bool date_before_tile,
                 bool wheelchair, bool bicycle) const {
  uint32_t count = header_->departurecount();
  if (count == 0) {
    return nullptr;
  }

  // Departures are sorted by edge Id and then by departure time.
  // Binary search to find a departure with matching line Id.
  int32_t low = 0;
  int32_t high = count-1;
  int32_t mid;
  int32_t found = count;
  while (low <= high) {
    mid = (low + high) / 2;
    const auto& dep = departures_[mid];
    //matching lineid and a workable time
    if (lineid == dep.lineid() && current_time <= dep.departure_time()) {
      found = mid;
      high = mid - 1;
    }//need a smaller lineid
    else if (lineid < dep.lineid()) {
      high = mid - 1;
    }//either need a bigger lineid or a later time
    else {
      low = mid + 1;
    }
  }

  // Iterate through departures until one is found with valid date, dow or
  // calendar date, and does not have a calendar exception.
  for(; found < count && departures_[found].lineid() == lineid; ++found) {
    // Make sure valid departure time
    if (departures_[found].departure_time() >= current_time &&
      GetTransitSchedule(departures_[found].schedule_index())->IsValid(day, dow, date_before_tile) &&
      (!wheelchair || departures_[found].wheelchair_accessible()) &&
      (!bicycle || departures_[found].bicycle_accessible())) {
      return &departures_[found];
    }
  }

  // TODO - maybe wrap around, try next day?
  LOG_DEBUG("No more departures found for lineid = " + std::to_string(lineid) +
           " current_time = " + std::to_string(current_time));
  return nullptr;
}

// Get the departure given the line Id and tripid
const TransitDeparture* GraphTile::GetTransitDeparture(const uint32_t lineid,
                     const uint32_t tripid) const {
  uint32_t count = header_->departurecount();
  if (count == 0) {
    return nullptr;
  }

  // Departures are sorted by edge Id and then by departure time.
  // Binary search to find a departure with matching line Id.
  int32_t low = 0;
  int32_t high = count-1;
  int32_t mid;
  int32_t found = count;
  while (low <= high) {
    mid = (low + high) / 2;
    const auto& dep = departures_[mid];
    //find the first matching lineid in the list
    if (lineid == dep.lineid()) {
      found = mid;
      high = mid - 1;
    }//need a smaller lineid
    else if (lineid < dep.lineid()) {
      high = mid - 1;
    }//need a bigger lineid
    else {
      low = mid + 1;
    }
  }

  // Iterate through departures until one is found with matching trip id
  for(; found < count && departures_[found].lineid() == lineid; ++found)
    if (departures_[found].tripid() == tripid)
      return &departures_[found];

  LOG_INFO("No departures found for lineid = " + std::to_string(lineid) +
           " and tripid = " + std::to_string(tripid));
  return nullptr;
}

// Get a map of departures based on lineid.  No dups exist in the map.
std::unordered_map<uint32_t,TransitDeparture*> GraphTile::GetTransitDepartures() const {

  std::unordered_map<uint32_t,TransitDeparture*> deps;
  deps.reserve(header_->departurecount());

  for (uint32_t i = 0; i < header_->departurecount(); i++)
    deps.insert({departures_[i].lineid(),&departures_[i]});

  return deps;
}

// Get the stop onestops in this tile
std::unordered_map<std::string, tile_index_pair>
GraphTile::GetStopOneStops() const {
  return stop_one_stops;
}

// Get the route onestops in this tile.
std::unordered_map<std::string, std::list<tile_index_pair>>
GraphTile::GetRouteOneStops() const {
  return route_one_stops;
}

// Get the operator onestops in this tile.
std::unordered_map<std::string, std::list<tile_index_pair>>
GraphTile::GetOperatorOneStops() const {
  return oper_one_stops;
}

// Get the transit stop given its index within the tile.
const TransitStop* GraphTile::GetTransitStop(const uint32_t idx) const {
  uint32_t count = header_->stopcount();
  if (count == 0)
    return nullptr;

  if (idx < count)
    return &transit_stops_[idx];
  throw std::runtime_error("GraphTile Transit Stop index out of bounds");
}

// Get the transit route given its index within the tile.
const TransitRoute* GraphTile::GetTransitRoute(const uint32_t idx) const {
  uint32_t count = header_->routecount();
  if (count == 0)
    return nullptr;

  if (idx < count) {
    return &transit_routes_[idx];
  }
  throw std::runtime_error("GraphTile GetTransitRoute index out of bounds");
}

// Get the transit schedule given its schedule index.
const TransitSchedule* GraphTile::GetTransitSchedule(const uint32_t idx) const {
  uint32_t count = header_->schedulecount();
  if (count == 0)
    return nullptr;

  if (idx < count) {
    return &transit_schedules_[idx];
  }
  throw std::runtime_error("GraphTile GetTransitSchedule index out of bounds");
}

// Get the access restriction given its directed edge index
std::vector<AccessRestriction> GraphTile::GetAccessRestrictions(const uint32_t idx,
                                                                const uint32_t access) const {

  std::vector<AccessRestriction> restrictions;
  uint32_t count = header_->access_restriction_count();
  if (count == 0) {
    return restrictions;
  }

  // Access restriction are sorted by edge Id.
  // Binary search to find a access restriction with matching edge Id.
  int32_t low = 0;
  int32_t high = count-1;
  int32_t mid;
  int32_t found = count;
  while (low <= high) {
    mid = (low + high) / 2;
    const auto& res = access_restrictions_[mid];
    //find the first matching index in the list
    if (idx == res.edgeindex()) {
      found = mid;
      high = mid - 1;
    }//need a smaller index
    else if (idx < res.edgeindex()) {
      high = mid - 1;
    }//need a bigger index
    else {
      low = mid + 1;
    }
  }

  // Add restrictions for only the access that we are interested in
  for (; found < count && access_restrictions_[found].edgeindex() == idx; ++found)
    if (access_restrictions_[found].modes() & access)
      restrictions.emplace_back(access_restrictions_[found]);

  if (restrictions.size() == 0)
    LOG_ERROR("No restrictions found for edge index = " + std::to_string(idx));
  return restrictions;
}

// Get the array of graphids for this bin
midgard::iterable_t<GraphId> GraphTile::GetBin(size_t column, size_t row) const {
  auto offsets = header_->bin_offset(column, row);
  return iterable_t<GraphId>{edge_bins_ + offsets.first, edge_bins_ + offsets.second};
}

midgard::iterable_t<GraphId> GraphTile::GetBin(size_t index) const {
  auto offsets = header_->bin_offset(index);
  return iterable_t<GraphId>{edge_bins_ + offsets.first, edge_bins_ + offsets.second};
}

// Get traffic segment(s) associated to this edge.
std::vector<std::pair<TrafficAssociation, float>> GraphTile::GetTrafficSegments(const GraphId& edge) const {
  return GetTrafficSegments(edge.id());
}

// Get traffic segment(s) associated to this edge.
std::vector<std::pair<TrafficAssociation, float>> GraphTile::GetTrafficSegments(const size_t idx) const {
  if (idx < header_->traffic_id_count()) {
    const TrafficAssociation& t = traffic_segments_[idx];
    if (!t.chunk()) {
      // This edge associates to a single traffic segment
      return { std::make_pair(t, 1.0f) };
    } else {
      // This represents a traffic chunk - the offset into the chunk array and
      // the count are stored.
      // TODO!
      auto c = t.GetChunkCountAndIndex();
//    std::vector<std::pair<GraphId, float>> segments;
//     for (uint32_t i = 0; i < count; i++, chunk++) {
//     segments.emplace_back(std::make_pair(GraphId(t & kChunkIDMask),
//                           (t & kChunkWeightMask) / 255.0f));
//     }
//     return segments;
      return { };
    }
  } else if (header_->traffic_id_count() == 0) {
    return { };
  } else {
    throw std::runtime_error("GraphTile GetTrafficSegments index out of bounds: " +
                           std::to_string(header_->graphid().tileid()) + "," +
                           std::to_string(header_->graphid().level()) + "," +
                           std::to_string(idx)  + " traffic Id count= " +
                           std::to_string(header_->traffic_id_count()));
  }
}


}
}
