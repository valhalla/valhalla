#ifndef VALHALLA_MJOLNIR_GRAPHBUILDER_H
#define VALHALLA_MJOLNIR_GRAPHBUILDER_H

#include <boost/property_tree/ptree.hpp>
#include <cstdint>
#include <string>
#include <vector>

#include <valhalla/baldr/signinfo.h>
#include <valhalla/midgard/sequence.h>

#include <valhalla/mjolnir/osmdata.h>
#include <valhalla/mjolnir/osmnode.h>
#include <valhalla/mjolnir/osmnodelinguistic.h>
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
   * @param  linguistic_node_file           where to store the to linguistic info so they are not in
   * memory
   *
   */
  static void Build(const boost::property_tree::ptree& pt,
                    const OSMData& osmdata,
                    const std::string& ways_file,
                    const std::string& way_nodes_file,
                    const std::string& nodes_file,
                    const std::string& edges_file,
                    const std::string& complex_from_restriction_file,
                    const std::string& complex_to_restriction_file,
                    const std::string& linguistic_node_file,
                    const std::map<baldr::GraphId, size_t>& tiles);

  static std::map<baldr::GraphId, size_t> BuildEdges(const ptree& conf,
                                                     const std::string& ways_file,
                                                     const std::string& way_nodes_file,
                                                     const std::string& nodes_file,
                                                     const std::string& edges_file);

  static std::string GetRef(const std::string& way_ref, const std::string& relation_ref);

  static void
  GetPronunciationTokens(const UniqueNames& uniquenames,
                         const std::vector<std::pair<std::string, bool>>& default_languages,
                         const uint32_t ipa_index,
                         const uint32_t ipa_lang_index,
                         const uint32_t nt_sampa_index,
                         const uint32_t nt_sampa_lang_index,
                         const uint32_t katakana_index,
                         const uint32_t katakana_lang_index,
                         const uint32_t jeita_index,
                         const uint32_t jeita_lang_index,
                         std::vector<std::string>& ipa_tokens,
                         std::vector<baldr::Language>& ipa_langs,
                         std::vector<std::string>& nt_sampa_tokens,
                         std::vector<baldr::Language>& nt_sampa_langs,
                         std::vector<std::string>& katakana_tokens,
                         std::vector<baldr::Language>& katakana_langs,
                         std::vector<std::string>& jeita_tokens,
                         std::vector<baldr::Language>& jeita_langs);

  static void AddPronunciationsWithLang(std::vector<std::string>& pronunciations,
                                        std::map<size_t, baldr::Language>& lang_map,
                                        const baldr::PronunciationAlphabet verbal_type,
                                        const std::vector<std::string>& pronunciation_tokens,
                                        const std::vector<baldr::Language>& pronunciation_langs,
                                        const std::vector<baldr::Language>& token_langs,
                                        const size_t token_size,
                                        const size_t key);

  static void
  AddLanguage(const size_t index, const baldr::Language& lang, std::vector<std::string>& linguistics);

  static void BuildPronunciations(const std::vector<std::string>& ipa_tokens,
                                  const std::vector<baldr::Language>& ipa_langs,
                                  const std::vector<std::string>& nt_sampa_tokens,
                                  const std::vector<baldr::Language>& nt_sampa_langs,
                                  const std::vector<std::string>& katakana_tokens,
                                  const std::vector<baldr::Language>& katakana_langs,
                                  const std::vector<std::string>& jeita_tokens,
                                  const std::vector<baldr::Language>& jeita_langs,
                                  const std::vector<baldr::Language>& token_langs,
                                  const size_t token_size,
                                  const size_t key,
                                  std::vector<std::string>& pronunciations,
                                  std::map<size_t, baldr::Language>& lang_map,
                                  bool add_ipa,
                                  bool add_nt_sampa,
                                  bool add_katakana,
                                  bool add_jeita);

  static void GetShieldTokens(const OSMData& osmdata,
                              const uint32_t shield_text_color_index,
                              const uint32_t shield_name_index,
                              const uint32_t shield_display_ref_index,
                              std::vector<std::string>& shield_text_color_tokens,
                              std::vector<std::string>& shield_name_tokens,
                              std::vector<std::string>& display_ref_tokens);

  static void AddShields(const std::vector<std::string>& shield_text_color_tokens,
                         const std::vector<std::string>& shield_name_tokens,
                         const std::vector<std::string>& display_ref_tokens,
                         const size_t index,
                         uint32_t& count);

  static bool
  CreateSignInfoList(const OSMNode& node,
                     const OSMWay& way,
                     const std::map<std::pair<uint8_t, uint8_t>, uint32_t>& pronunciationMap,
                     const std::map<std::pair<uint8_t, uint8_t>, uint32_t>& langMap,
                     const OSMData& osmdata,
                     const std::vector<std::pair<std::string, bool>>& default_languages,
                     midgard::sequence<OSMNodeLinguistic>& linguistic_node,
                     std::vector<baldr::SignInfo>& exits,
                     std::vector<std::string>& linguistics,
                     bool fork,
                     bool forward,
                     bool ramp,
                     bool tc);
};
} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_GRAPHBUILDER_H
