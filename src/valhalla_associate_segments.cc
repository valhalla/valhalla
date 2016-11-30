#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/merge.h>
#include <valhalla/loki/search.h>

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
namespace pbf = opentraffic::osmlr;

namespace bpo = boost::program_options;
namespace bpt = boost::property_tree;
namespace bfs = boost::filesystem;

namespace {

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

void edge_association::add_tile(const std::string &file_name) {
  pbf::Tile tile;
  {
    std::ifstream in(file_name);
    if (!tile.ParseFromIstream(&in)) {
      throw std::runtime_error("Unable to parse traffic segment file.");
    }
  }

  for (auto &entry : tile.entries()) {
    if (!entry.has_marker()) {
      assert(entry.has_segment());
      auto &segment = entry.segment();

      assert(segment.lrps_size() >= 2);
      auto &lrp = segment.lrps(0);
      int32_t lng = lrp.coord().lng();
      int32_t lat = lrp.coord().lat();
      vm::PointLL coord(double(lng) / 10000000, double(lat) / 10000000);

      auto loc = vl::Search(vb::Location(coord), m_reader);
      auto pt = loc.ToPtree(0);
      std::cout << "====\n";
      bpt::json_parser::write_json(std::cout, pt);
      std::cout << "\n";
    }
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
