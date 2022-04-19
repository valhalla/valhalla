#ifndef VALHALLA_MJOLNIR_GRAPHBUILDER_H
#define VALHALLA_MJOLNIR_GRAPHBUILDER_H

#include <boost/property_tree/ptree.hpp>
#include <cstdint>
#include <string>
#include <vector>

#include <valhalla/baldr/signinfo.h>

#include <valhalla/mjolnir/osmdata.h>
#include <valhalla/mjolnir/osmnode.h>
#include <valhalla/mjolnir/osmway.h>

namespace valhalla {
namespace mjolnir {

using boost::property_tree::ptree;

/**
 * Class used to construct temporary data used to build the initial graph.
 */
class GraphBuilder {
public:
  /**
   * Tell the builder to build the tiles from the provided datasource
   * and configs
   * @param  config                         properties file
   * @param  osmdata                        OSM data used to build the graph.
   * @param  ways_file                      where to store the ways so they are not in memory
   * @param  way_nodes_file                 where to store the nodes so they are not in memory
   * @param  nodes_file                     where to store node information so it isn't in memory
   * @param  edges_file                     where to store edge information so it isn't in memory
   * @param  complex_from_restriction_file  where to store the from complex restrictions so they are
   * not in memory
   * @param  complex_to_restriction_file    where to store the to complex restrictions so they are not
   * in memory
   * @param  pronunciation_file             where to store the to pronunciations so they are not
   * in memory
   */
  static void Build(const boost::property_tree::ptree& pt,
                    const OSMData& osmdata,
                    const std::string& ways_file,
                    const std::string& way_nodes_file,
                    const std::string& nodes_file,
                    const std::string& edges_file,
                    const std::string& complex_from_restriction_file,
                    const std::string& complex_to_restriction_file,
                    const std::string& pronunciation_file,
                    const std::map<baldr::GraphId, size_t>& tiles);

  static std::map<baldr::GraphId, size_t> BuildEdges(const ptree& conf,
                                                     const std::string& ways_file,
                                                     const std::string& way_nodes_file,
                                                     const std::string& nodes_file,
                                                     const std::string& edges_file);

  static std::string GetRef(const std::string& way_ref, const std::string& relation_ref);

  static void GetPronunciationTokens(const OSMData& osmdata,
                                     const uint32_t ipa_index,
                                     const uint32_t nt_sampa_index,
                                     const uint32_t katakana_index,
                                     const uint32_t jeita_index,
                                     std::vector<std::string>& ipa_tokens,
                                     std::vector<std::string>& nt_sampa_tokens,
                                     std::vector<std::string>& katakana_tokens,
                                     std::vector<std::string>& jeita_tokens,
                                     bool is_node_pronunciation = false);

  static void AddPronunciation(const baldr::PronunciationAlphabet alphabet,
                               const std::string& phoneme,
                               std::vector<std::string>& pronunciations,
                               uint32_t& count);

  static void BuildPronunciations(const std::vector<std::string>& ipa_tokens,
                                  const std::vector<std::string>& nt_sampa_tokens,
                                  const std::vector<std::string>& katakana_tokens,
                                  const std::vector<std::string>& jeita_tokens,
                                  const size_t index,
                                  std::vector<std::string>& pronunciations,
                                  bool add_ipa,
                                  bool add_nt_sampa,
                                  bool add_katakana,
                                  bool add_jeita,
                                  uint32_t& count);

  static bool CreateSignInfoList(const OSMNode& node,
                                 const OSMWay& way,
                                 const OSMPronunciation& pronunciation,
                                 const OSMData& osmdata,
                                 std::vector<baldr::SignInfo>& exits,
                                 std::vector<std::string>& pronunciations,
                                 bool fork,
                                 bool forward,
                                 bool ramp,
                                 bool tc);
};
} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_GRAPHBUILDER_H
