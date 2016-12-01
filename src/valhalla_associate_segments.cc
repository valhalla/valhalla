#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/merge.h>
#include <valhalla/loki/search.h>
#include <valhalla/thor/pathalgorithm.h>
#include <valhalla/thor/astar.h>

#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>

#include "config.h"
#include "segment.pb.h"
#include "tile.pb.h"

namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;
namespace vl = valhalla::loki;
namespace vs = valhalla::sif;
namespace vt = valhalla::thor;
namespace pbf = opentraffic::osmlr;

namespace bpo = boost::program_options;
namespace bpt = boost::property_tree;
namespace bfs = boost::filesystem;

namespace {

vm::PointLL interp(vm::PointLL a, vm::PointLL b, double frac) {
  return vm::PointLL(
    a.lng() + ((b.lng() - a.lng()) * frac),
    a.lat() + ((b.lat() - a.lat()) * frac));
}

void subsegment(std::vector<vm::PointLL> &seg, uint32_t dist) {
  const size_t len = seg.size();
  assert(len > 1);

  double d = 0.0;
  size_t i = 0;
  for (; i < len - 1; ++i) {
    auto segdist = seg[i].Distance(seg[i+1]);
    if ((d + segdist) >= dist) {
      double frac = (dist - d) / segdist;
      seg[i] = interp(seg[i], seg[i+1], frac);
      break;

    } else {
      d += segdist;
    }
  }

  seg.erase(seg.begin(), seg.begin() + i);
}

uint16_t bearing(const std::vector<vm::PointLL> &shape) {
  // OpenLR says to use 20m along the edge, but we could use the
  // GetOffsetForHeading function, which adapts it to the road class.
  float heading = vm::PointLL::HeadingAlongPolyline(shape, 20);
  assert(heading >= 0.0);
  assert(heading < 360.0);
  return uint16_t(std::round(heading));
}

uint16_t bearing(const vb::GraphTile *tile, vb::GraphId edge_id, float dist) {
  std::vector<vm::PointLL> shape;
  const auto *edge = tile->directededge(edge_id);
  uint32_t edgeinfo_offset = edge->edgeinfo_offset();
  auto edgeinfo = tile->edgeinfo(edgeinfo_offset);
  uint32_t edge_len = edge->length();

  shape = edgeinfo.shape();
  if (!edge->forward()) {
    std::reverse(shape.begin(), shape.end());
  }

  subsegment(shape, uint32_t(dist * edge_len));

  return bearing(shape);
}

struct segment_part {
  vb::GraphId segment_id;
  uint8_t fraction;
};

struct edge_association {
  explicit edge_association(vb::GraphReader &reader) : m_reader(reader) {}
  void add_tile(const std::string &file_name);

private:
  vb::GraphReader &m_reader;
  // map of edge ID to the segment parts which match it.
  std::unordered_map<vb::GraphId, std::list<segment_part> > m_edges;
};

struct edge_score {
  vb::GraphId id;
  int score;
};

bool edge_pred(const vb::DirectedEdge *edge) {
  return (edge->use() != vb::Use::kFerry &&
          edge->use() != vb::Use::kTransitConnection &&
          !edge->trans_up() &&
          !edge->trans_down());
}

bool check_access(const vb::DirectedEdge *edge) {
  uint32_t access = vb::kAllAccess;
  access &= edge->forwardaccess();

  // if any edge is a shortcut, then drop the whole path
  if (edge->shortcut()) {
    return false;
  }

  // if the edge predicate is false for any edge, then drop the whole
  // path.
  if (edge_pred(edge) == false) {
    return false;
  }

  // be permissive here, as we do want to collect traffic on most vehicular
  // routes.
  uint32_t vehicular = vb::kAutoAccess | vb::kTruckAccess |
    vb::kTaxiAccess | vb::kBusAccess | vb::kHOVAccess;
  return access & vehicular;
}

bool is_oneway(const vb::DirectedEdge *e) {
  // TODO: don't need to find opposite edge, as this info alread in the
  // reverseaccess mask?
  return e->reverseaccess() == 0;
}

enum class FormOfWay {
  kUndefined = 0,
  kMotorway = 1,
  kMultipleCarriageway = 2,
  kSingleCarriageway = 3,
  kRoundabout = 4,
  kTrafficSquare = 5,
  kSlipRoad = 6,
  kOther = 7
};

std::ostream &operator<<(std::ostream &out, FormOfWay fow) {
  switch (fow) {
  case FormOfWay::kUndefined:           out << "undefined";            break;
  case FormOfWay::kMotorway:            out << "motorway";             break;
  case FormOfWay::kMultipleCarriageway: out << "multiple_carriageway"; break;
  case FormOfWay::kSingleCarriageway:   out << "single_carriageway";   break;
  case FormOfWay::kRoundabout:          out << "roundabout";           break;
  case FormOfWay::kTrafficSquare:       out << "traffic_square";       break;
  case FormOfWay::kSlipRoad:            out << "sliproad";             break;
  default:
    out << "other";
  }
  return out;
}

FormOfWay form_of_way(const vb::DirectedEdge *e) {
  bool oneway = is_oneway(e);
  auto rclass = e->classification();

  // if it's a slip road, return that. TODO: am i doing this right?
  if (e->link()) {
    return FormOfWay::kSlipRoad;
  }
  // if it's a roundabout, return that
  else if (e->roundabout()) {
    return FormOfWay::kRoundabout;
  }
  // if it's a motorway and it's one-way, then it's likely to be grade separated
  else if (rclass == vb::RoadClass::kMotorway && oneway) {
    return FormOfWay::kMotorway;
  }
  // if it's a major road, and it's one-way then it might be a multiple
  // carriageway road.
  else if (rclass <= vb::RoadClass::kTertiary && oneway) {
    return FormOfWay::kMultipleCarriageway;
  }
  // not one-way, so perhaps it's a single carriageway
  else if (rclass <= vb::RoadClass::kTertiary) {
    return FormOfWay::kSingleCarriageway;
  }
  // everything else
  else {
    return FormOfWay::kOther;
  }
}

class DistanceOnlyCost : public vs::DynamicCost {
public:
  DistanceOnlyCost(vs::TravelMode travel_mode);
  virtual ~DistanceOnlyCost();
  uint32_t access_mode() const;
  bool Allowed(const vb::DirectedEdge* edge,
               const vs::EdgeLabel& pred,
               const vb::GraphTile*& tile,
               const vb::GraphId& edgeid) const;
  bool AllowedReverse(const vb::DirectedEdge* edge,
                      const vs::EdgeLabel& pred,
                      const vb::DirectedEdge* opp_edge,
                      const vb::GraphTile*& tile,
                      const vb::GraphId& edgeid) const;
  bool Allowed(const vb::NodeInfo* node) const;
  vs::Cost EdgeCost(const vb::DirectedEdge* edge) const;
  const vs::EdgeFilter GetEdgeFilter() const;
  const vs::NodeFilter GetNodeFilter() const;
  float AStarCostFactor() const;
};

DistanceOnlyCost::DistanceOnlyCost(vs::TravelMode travel_mode)
  : DynamicCost(bpt::ptree(), travel_mode) {
}

DistanceOnlyCost::~DistanceOnlyCost() {
}

uint32_t DistanceOnlyCost::access_mode() const {
  uint32_t vehicular = vb::kAutoAccess | vb::kTruckAccess |
    vb::kTaxiAccess | vb::kBusAccess | vb::kHOVAccess;
  return vehicular;
}

bool DistanceOnlyCost::Allowed(const vb::DirectedEdge* edge,
                               const vs::EdgeLabel&,
                               const vb::GraphTile*&,
                               const vb::GraphId&) const {
  return check_access(edge);
}

bool DistanceOnlyCost::AllowedReverse(const vb::DirectedEdge* edge,
                                      const vs::EdgeLabel& pred,
                                      const vb::DirectedEdge* opp_edge,
                                      const vb::GraphTile*& tile,
                                      const vb::GraphId& edgeid) const {
  return check_access(edge);
}

bool DistanceOnlyCost::Allowed(const vb::NodeInfo*) const {
  return true;
}

vs::Cost DistanceOnlyCost::EdgeCost(const vb::DirectedEdge* edge) const {
  float edge_len(edge->length());
  return {edge_len, edge_len};
}

const vs::EdgeFilter DistanceOnlyCost::GetEdgeFilter() const {
  return [](const vb::DirectedEdge *edge) -> float {
    return check_access(edge) ? 1.0f : 0.0f;
  };
}

const vs::NodeFilter DistanceOnlyCost::GetNodeFilter() const {
  return [](const vb::NodeInfo *) -> bool {
    return false;
  };
}

float DistanceOnlyCost::AStarCostFactor() const {
  return 1.0f;
}

vm::PointLL coord_for_lrp(const pbf::Segment::LocationReference &lrp) {
  int32_t lng = lrp.coord().lng();
  int32_t lat = lrp.coord().lat();
  vm::PointLL coord(double(lng) / 10000000, double(lat) / 10000000);
  return coord;
}

void edge_association::add_tile(const std::string &file_name) {
  pbf::Tile tile;
  {
    std::ifstream in(file_name);
    if (!tile.ParseFromIstream(&in)) {
      throw std::runtime_error("Unable to parse traffic segment file.");
    }
  }

  vs::TravelMode travel_mode = vs::TravelMode::kDrive;
  std::shared_ptr<vt::PathAlgorithm> path_algo(new vt::AStarPathAlgorithm());
  std::shared_ptr<vs::DynamicCost> costing(new DistanceOnlyCost(travel_mode));

  std::cout.precision(16);
  size_t entry_id = 0;
  for (auto &entry : tile.entries()) {
    if (!entry.has_marker()) {
      assert(entry.has_segment());
      auto &segment = entry.segment();

      const size_t size = segment.lrps_size();
      assert(size >= 2);

      std::vector<std::vector<edge_score> > locs;
      locs.resize(size - 1);

      std::cout << ">> " << entry_id << "\n";
      auto origin = vl::Search(vb::Location(coord_for_lrp(segment.lrps(0))), m_reader);
      for (size_t i = 0; i < size - 1; ++i) {
        auto &lrp = segment.lrps(i);
        auto coord = coord_for_lrp(lrp);
        auto next_coord = coord_for_lrp(segment.lrps(i+1));

        vb::RoadClass road_class = vb::RoadClass(lrp.start_frc());

        auto dest = vl::Search(vb::Location(next_coord), m_reader);

        std::cout << " >> FROM " << coord.lat() << "/" << coord.lng() << "\n";
        std::cout << " >>   TO " << next_coord.lat() << "/" << next_coord.lng() << "\n";

        auto path = path_algo->GetBestPath(
          origin, dest, m_reader, &costing, travel_mode);

        if (path.empty()) {
          // what to do if there's no path?
          throw std::runtime_error("Ooops");
        }

        int score = 0;
        uint32_t sum = 0;
        for (auto &p : path) {
          sum += p.elapsed_time;
        }
        score += std::abs(int(sum) - int(lrp.length())) / 10;

        auto edge_id = path.front().edgeid;
        auto *tile = m_reader.GetGraphTile(edge_id);
        auto *edge = tile->directededge(edge_id);

        if (!check_access(edge)) {
          continue;
        }

        score += std::abs(int(road_class) - int(edge->classification()));

        bool found = false;
        for (auto &e : origin.edges) {
          if (e.id == edge_id) {
            found = true;
            score += int(e.projected.Distance(coord));

            int bear1 = bearing(tile, edge_id, e.dist);
            int bear2 = lrp.bear();
            int bear_diff = std::abs(bear1 - bear2);
            if (bear_diff > 180) {
              bear_diff = 360 - bear_diff;
            }
            if (bear_diff < 0) {
              bear_diff += 360;
            }
            score += bear_diff / 10;

            break;
          }
        }
        assert(found);


        // form of way isn't really a metric space...
        FormOfWay fow1 = form_of_way(edge);
        FormOfWay fow2 = FormOfWay(lrp.start_fow());
        score += (fow1 == fow2) ? 0 : 5;

        std::cout << "  " << edge_id << ":\t" << score << "\n";
      }
    }

    entry_id += 1;
  }
}

} // anonymous namespace

int main(int argc, char** argv) {
  std::string config, tile_dir;

  bpo::options_description options("valhalla_associate_segments " VERSION "\n"
                                   "\n"
                                   " Usage: valhalla_associate_segments [options]\n"
                                   "\n"
                                   "osmlr associates traffic segment descriptors with a valhalla graph. "
                                   "\n"
                                   "\n");

  options.add_options()
    ("help,h", "Print this help message.")
    ("version,v", "Print the version of this software.")
    ("osmlr-tile-dir,t", bpo::value<std::string>(&tile_dir), "Location of traffic segment tiles.")
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
    std::cout << "valhalla_associate_segments " << VERSION << "\n";
    return EXIT_SUCCESS;
  }

  if (!vm.count("osmlr-tile-dir")) {
    std::cout << "You must provide a tile directory to read OSMLR tiles from.\n";
    return EXIT_FAILURE;
  }

  //parse the config
  bpt::ptree pt;
  bpt::read_json(config.c_str(), pt);

  //configure logging
  vm::logging::Configure({{"type","std_err"},{"color","true"}});

  //get something we can use to fetch tiles
  vb::GraphReader reader(pt.get_child("mjolnir"));

  // this holds the extra data before we serialize it to the extra section
  // of a tile.
  edge_association e(reader);

  for (auto dir_entry : bfs::recursive_directory_iterator(tile_dir)) {
    if (bfs::is_regular_file(dir_entry)) {
      auto ext = dir_entry.path().extension();
      if (ext == ".osmlr") {
        e.add_tile(dir_entry.path().string());
      }
    }
  }

  return EXIT_SUCCESS;
}
