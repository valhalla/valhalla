#ifndef VALHALLA_MJOLNIR_PBFGRAPHPARSER_H
#define VALHALLA_MJOLNIR_PBFGRAPHPARSER_H

#include <boost/property_tree/ptree.hpp>
#include <cstdint>
#include <string>
#include <vector>

#include <valhalla/mjolnir/osmdata.h>

namespace valhalla {
namespace mjolnir {

/**
 * Class used to parse OSM protocol buffer extracts.
 */
class PBFGraphParser {
public:
  /**
   * Loads given input files
   * @param  pt                             properties file
   * @param  input_files                    the protobuf files to parse
   * @param  ways_file                      where to store the ways so they are not in memory
   * @param  way_nodes_file                 where to store the nodes so they are not in memory
   * @param  access_file                    where to store the access tags so they are not in memory
   * @param  pronunciation_file             where to store the pronunciations so they are not in
   * memory
   */
  static OSMData ParseWays(const boost::property_tree::ptree& pt,
                           const std::vector<std::string>& input_files,
                           const std::string& ways_file,
                           const std::string& way_nodes_file,
                           const std::string& access_file,
                           const std::string& pronunciation_file);

  /**
   * Loads given input files
   * @param  pt                             properties file
   * @param  input_files                    the protobuf files to parse
   * @param  complex_restriction_from_file  where to store the from complex restrictions so they are
   * not in memory
   * @param  complex_restriction_to_file    where to store the to complex restrictions so they are not
   * in memory
   * @param  osmdata                        OSM data
   *
   */
  static void ParseRelations(const boost::property_tree::ptree& pt,
                             const std::vector<std::string>& input_files,
                             const std::string& complex_restriction_from_file,
                             const std::string& complex_restriction_to_file,
                             OSMData& osmdata);

  /**
   * Loads given input files
   * @param  pt                             properties file
   * @param  input_files                    the protobuf files to parse
   * @param  way_nodes_file                 where to store the nodes so they are not in memory
   * @param  bss_nodes_file                 where to store the bss nodes so they are not in memory
   * @param  osmdata                        OSM data
   */
  static void ParseNodes(const boost::property_tree::ptree& pt,
                         const std::vector<std::string>& input_files,
                         const std::string& way_nodes_file,
                         const std::string& bss_nodes_file,
                         OSMData& osmdata);
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_PBFGRAPHPARSER_H
