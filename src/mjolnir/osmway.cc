#include "mjolnir/osmway.h"
#include "baldr/edgeinfo.h"
#include "mjolnir/util.h"
#include "regex"

#include "midgard/logging.h"
#include <boost/algorithm/string.hpp>

using namespace valhalla::baldr;

namespace {

constexpr uint32_t kMaxNodesPerWay = 65535;
constexpr uint8_t kUnlimitedOSMSpeed = std::numeric_limits<uint8_t>::max();
constexpr float kMaxOSMSpeed = 140.0f;

const std::regex kFloatRegex("\\d+\\.(\\d+)");

/**
 * For levels we use 7-bit varint encoding, with arbitrary precision stored separately.
 *
 * @param lvl       the level to encode
 * @param precision the precision with which to encode the level value
 *
 * @return a string containing the encoded value
 */
std::string encode_level(const float lvl, const int precision = 0) {
  std::string encoded;
  // most of the time precision will be zero and level values will be lower
  // than 4191 (even in the case of higher precision)
  encoded.reserve(2);

  int val = static_cast<int>(lvl * pow(10, precision));

  val = val < 0 ? ~(*reinterpret_cast<unsigned int*>(&val) << 1) : val << 1;
  // we take 7 bits of this at a time
  while (val > 0x7f) {
    // marking the most significant bit means there are more pieces to come
    int next_value = (0x80 | (val & 0x7f));
    encoded.push_back(static_cast<char>(next_value));
    val >>= 7;
  }
  // write the last chunk
  encoded.push_back(static_cast<char>(val & 0x7f));
  return encoded;
}

} // namespace

namespace valhalla {
namespace mjolnir {

// Set the number of nodes for this way.
void OSMWay::set_node_count(const uint32_t count) {
  if (count > kMaxNodesPerWay) {
    LOG_WARN("Exceeded max nodes per way: " + std::to_string(count));
    nodecount_ = static_cast<uint16_t>(kMaxNodesPerWay);
  } else {
    nodecount_ = static_cast<uint16_t>(count);
  }
}

// Sets the speed in KPH.
void OSMWay::set_speed(const float speed) {
  if (speed > kMaxOSMSpeed) {
    LOG_WARN("Exceeded max speed for way id: " + std::to_string(osmwayid_));
    speed_ = kMaxOSMSpeed;
  } else {
    speed_ = static_cast<unsigned char>(speed + 0.5f);
  }
}

// Sets the speed limit in KPH.
void OSMWay::set_speed_limit(const float speed_limit) {
  if (speed_limit == kUnlimitedOSMSpeed) {
    speed_limit_ = kUnlimitedOSMSpeed;
  } else if (speed_limit > kMaxOSMSpeed) {
    LOG_WARN("Exceeded max speed for way id: " + std::to_string(osmwayid_));
    speed_limit_ = kMaxOSMSpeed;
  } else {
    speed_limit_ = static_cast<unsigned char>(speed_limit + 0.5f);
  }
}

// Sets the backward speed in KPH.
void OSMWay::set_backward_speed(const float backward_speed) {
  if (backward_speed > kMaxOSMSpeed) {
    LOG_WARN("Exceeded max backward speed for way id: " + std::to_string(osmwayid_));
    backward_speed_ = kMaxOSMSpeed;
  } else {
    backward_speed_ = static_cast<unsigned char>(backward_speed + 0.5f);
  }
}

// Sets the backward speed in KPH.
void OSMWay::set_forward_speed(const float forward_speed) {
  if (forward_speed > kMaxOSMSpeed) {
    LOG_WARN("Exceeded max forward speed for way id: " + std::to_string(osmwayid_));
    forward_speed_ = kMaxOSMSpeed;
  } else {
    forward_speed_ = static_cast<unsigned char>(forward_speed + 0.5f);
  }
}

// Sets the truck speed in KPH.
void OSMWay::set_truck_speed(const float speed) {
  if (speed > kMaxOSMSpeed) {
    LOG_WARN("Exceeded max truck speed for way id: " + std::to_string(osmwayid_));
    truck_speed_ = kMaxOSMSpeed;
  } else {
    truck_speed_ = static_cast<unsigned char>(speed + 0.5f);
  }
}

void OSMWay::set_truck_speed_forward(const float truck_speed_forward) {
  if (truck_speed_forward > kMaxOSMSpeed) {
    LOG_WARN("Exceeded max forward truck speed for way id: " + std::to_string(osmwayid_));
    truck_speed_forward_ = kMaxOSMSpeed;
  } else {
    truck_speed_forward_ = static_cast<unsigned char>(truck_speed_forward + 0.5f);
  }
}

void OSMWay::set_truck_speed_backward(const float truck_speed_backward) {
  if (truck_speed_backward > kMaxOSMSpeed) {
    LOG_WARN("Exceeded max backward truck speed for way id: " + std::to_string(osmwayid_));
    truck_speed_backward_ = kMaxOSMSpeed;
  } else {
    truck_speed_backward_ = static_cast<unsigned char>(truck_speed_backward + 0.5f);
  }
}

// Sets the number of lanes
void OSMWay::set_lanes(const uint32_t lanes) {
  lanes_ = (lanes > kMaxLaneCount) ? kMaxLaneCount : lanes;
}

// Sets the number of backward lanes
void OSMWay::set_backward_lanes(const uint32_t backward_lanes) {
  backward_lanes_ = (backward_lanes > kMaxLaneCount) ? kMaxLaneCount : backward_lanes;
}

// Sets the number of forward lanes
void OSMWay::set_forward_lanes(const uint32_t forward_lanes) {
  forward_lanes_ = (forward_lanes > kMaxLaneCount) ? kMaxLaneCount : forward_lanes;
}

void OSMWay::set_layer(int8_t layer) {
  layer_ = layer;
}

void OSMWay::AddPronunciationsWithLang(std::vector<std::string>& pronunciations,
                                       std::map<size_t, baldr::Language>& lang_map,
                                       const baldr::PronunciationAlphabet verbal_type,
                                       const std::vector<std::string>& pronunciation_tokens,
                                       const std::vector<baldr::Language>& pronunciation_langs,
                                       const std::vector<baldr::Language>& token_langs,
                                       const size_t token_size,
                                       const size_t key) const {

  auto get_pronunciations = [](const std::vector<std::string>& pronunciation_tokens,
                               const std::vector<baldr::Language>& pronunciation_langs,
                               const std::map<size_t, size_t> indexMap, const size_t key,
                               const baldr::PronunciationAlphabet verbal_type) {
    linguistic_text_header_t header{static_cast<uint8_t>(baldr::Language::kNone),
                                    0,
                                    static_cast<uint8_t>(verbal_type),
                                    static_cast<uint8_t>(key),
                                    0,
                                    0};
    std::string pronunciation;
    for (size_t i = 0; i < pronunciation_tokens.size(); i++) {
      auto& t = pronunciation_tokens[i];

      if (indexMap.size() != 0) {
        auto index = indexMap.find(i);
        if (index != indexMap.end())
          header.name_index_ = index->second;
        else
          continue;
      }

      if (!t.size()) { // pronunciation is blank.  just add the lang

        if (!pronunciation_langs.size())
          continue;

        header.language_ = static_cast<uint8_t>(pronunciation_langs.at(i));
        header.length_ = 0;
        header.phonetic_alphabet_ = static_cast<uint8_t>(baldr::PronunciationAlphabet::kNone);

        pronunciation.append(
            std::string(reinterpret_cast<const char*>(&header), kLinguisticHeaderSize));
      } else {

        header.phonetic_alphabet_ = static_cast<uint8_t>(verbal_type);

        header.length_ = t.size();
        header.language_ =
            (pronunciation_langs.size() ? static_cast<uint8_t>(pronunciation_langs.at(i))
                                        : static_cast<uint8_t>(baldr::Language::kNone));
        pronunciation.append(
            std::string(reinterpret_cast<const char*>(&header), kLinguisticHeaderSize) + t);
      }
      if (indexMap.size() == 0)
        ++header.name_index_;
    }
    return pronunciation;
  };

  bool process = false, found = false;
  std::map<size_t, size_t> indexMap;
  size_t k = key;

  if ((pronunciation_langs.size() == 0 && token_langs.size() == 0) ||
      ((pronunciation_langs.size() != 0 && token_langs.size() == 0) &&
       (pronunciation_langs.size() <= token_size)))
    process = true;
  else {
    std::pair<std::map<size_t, size_t>::iterator, bool> ret;
    for (size_t i = 0; i < token_langs.size(); i++) {
      for (size_t j = 0; j < pronunciation_langs.size(); j++) {
        if (token_langs[i] == pronunciation_langs[j]) {
          ret = indexMap.insert(std::make_pair(j, k));
          if (ret.second == false) // already used
            continue;
          else {
            k++;
            process = true;
            found = true;
            break;
          }
        }
      }
      if (!found) {
        lang_map.emplace(k, token_langs[i]);
        k++;
      }
      found = false;
    }
  }

  if (lang_map.size() == 0 && pronunciation_langs.size() == 0 && token_langs.size() != 0) {
    for (size_t i = 0; i < token_langs.size(); i++) {
      lang_map.emplace(k, token_langs[i]);
      k++;
    }
  }

  if (process) {
    const auto& p =
        get_pronunciations(pronunciation_tokens, pronunciation_langs, indexMap, key, verbal_type);
    if (!p.empty())
      pronunciations.emplace_back(p);
  }
}

void OSMWay::AddPronunciations(std::vector<std::string>& pronunciations,
                               std::map<size_t, baldr::Language>& lang_map,
                               const UniqueNames& name_offset_map,
                               const std::vector<std::pair<std::string, bool>>& default_languages,
                               const std::vector<baldr::Language>& token_langs,
                               const uint32_t ipa_index,
                               const uint32_t ipa_lang_index,
                               const uint32_t nt_sampa_index,
                               const uint32_t nt_sampa_lang_index,
                               const uint32_t katakana_index,
                               const uint32_t katakana_lang_index,
                               const uint32_t jeita_index,
                               const uint32_t jeita_lang_index,
                               const size_t token_size,
                               const size_t key,
                               bool diff_names) const {

  if (ipa_index != 0) {
    std::vector<baldr::Language> pronunciation_langs;
    std::vector<std::string> pronunciation_tokens;

    ProcessNamesPronunciations(name_offset_map, default_languages, ipa_index, ipa_lang_index,
                               pronunciation_tokens, pronunciation_langs, diff_names, true);

    AddPronunciationsWithLang(pronunciations, lang_map, baldr::PronunciationAlphabet::kIpa,
                              pronunciation_tokens, pronunciation_langs, token_langs, token_size,
                              key);
  }

  if (nt_sampa_index != 0) {

    std::vector<baldr::Language> pronunciation_langs;
    std::vector<std::string> pronunciation_tokens;

    ProcessNamesPronunciations(name_offset_map, default_languages, nt_sampa_index,
                               nt_sampa_lang_index, pronunciation_tokens, pronunciation_langs,
                               diff_names, true);

    AddPronunciationsWithLang(pronunciations, lang_map, baldr::PronunciationAlphabet::kNtSampa,
                              pronunciation_tokens, pronunciation_langs, token_langs, token_size,
                              key);
  }

  if (katakana_index != 0) {
    std::vector<baldr::Language> pronunciation_langs;
    std::vector<std::string> pronunciation_tokens;

    ProcessNamesPronunciations(name_offset_map, default_languages, katakana_index,
                               katakana_lang_index, pronunciation_tokens, pronunciation_langs,
                               diff_names, true);

    AddPronunciationsWithLang(pronunciations, lang_map, baldr::PronunciationAlphabet::kKatakana,
                              pronunciation_tokens, pronunciation_langs, token_langs, token_size,
                              key);
  }

  if (jeita_index != 0) {
    std::vector<baldr::Language> pronunciation_langs;
    std::vector<std::string> pronunciation_tokens;

    ProcessNamesPronunciations(name_offset_map, default_languages, jeita_index, jeita_lang_index,
                               pronunciation_tokens, pronunciation_langs, diff_names, true);

    AddPronunciationsWithLang(pronunciations, lang_map, baldr::PronunciationAlphabet::kJeita,
                              pronunciation_tokens, pronunciation_langs, token_langs, token_size,
                              key);
  }
}

void OSMWay::AddLanguage(std::vector<std::string>& linguistics,
                         const size_t index,
                         const baldr::Language& lang) const {

  if (lang != baldr::Language::kNone) {
    linguistic_text_header_t header{static_cast<uint8_t>(lang),
                                    0,
                                    static_cast<uint8_t>(baldr::PronunciationAlphabet::kNone),
                                    static_cast<uint8_t>(index),
                                    0,
                                    0};
    linguistics.emplace_back(
        std::string(reinterpret_cast<const char*>(&header), kLinguisticHeaderSize));
  }
}

void OSMWay::ProcessNamesPronunciations(
    const UniqueNames& name_offset_map,
    const std::vector<std::pair<std::string, bool>>& default_languages,
    const uint32_t name_index,
    const uint32_t name_lang_index,
    std::vector<std::string>& tokens,
    std::vector<baldr::Language>& token_langs,
    bool diff_names,
    bool allow_empty_tokens) {

  std::vector<std::string> token_languages, found_languages, new_sort_order;
  std::vector<std::pair<std::string, std::string>> updated_token_languages, tokens_w_langs;
  tokens = GetTagTokens(name_offset_map.name(name_index));
  token_languages = GetTagTokens(name_offset_map.name(name_lang_index));

  if (default_languages.size() == 0)
    return;

  bool all_default = false, all_blank = true;

  // todo move this out to builder?
  if (default_languages.size() > 1) {
    if (std::find_if(default_languages.begin() + 1, default_languages.end(),
                     [](const std::pair<std::string, bool>& p) { return p.second == false; }) ==
        default_languages.end()) {
      all_default = true;
    }
  }

  if (name_index != 0 && name_lang_index == 0) {
    token_languages.resize(tokens.size());
    fill(token_languages.begin(), token_languages.end(), "");
  }

  // remove any entries that are not in our country language list
  // then sort our names based on the list.
  if (default_languages.size() &&
      (tokens.size() == token_languages.size())) { // should always be equal
    for (size_t i = 0; i < token_languages.size(); i++) {
      const auto& current_lang = token_languages[i];
      if (std::find_if(default_languages.begin(), default_languages.end(),
                       [&current_lang](const std::pair<std::string, bool>& p) {
                         return p.first == current_lang;
                       }) != default_languages.end()) {
        if (!token_languages[i].empty())
          all_blank = false;
        if (allow_empty_tokens || !tokens[i].empty()) {
          updated_token_languages.emplace_back(tokens[i], token_languages[i]);
          if (!token_languages[i].empty())
            tokens_w_langs.emplace_back(tokens[i], token_languages[i]);
        }
      }
    }

    std::unordered_map<std::string, uint32_t> lang_sort_order;
    new_sort_order.emplace_back("");

    for (size_t i = 0; i < default_languages.size(); i++) {
      if (i != 0)
        found_languages.emplace_back(default_languages[i].first);
      lang_sort_order[default_languages[i].first] = i;
    }

    auto cmp = [&lang_sort_order](const std::pair<std::string, std::string>& p1,
                                  const std::pair<std::string, std::string>& p2) {
      return lang_sort_order[p1.second] < lang_sort_order[p2.second];
    };

    std::stable_sort(updated_token_languages.begin(), updated_token_languages.end(), cmp);
    std::stable_sort(tokens_w_langs.begin(), tokens_w_langs.end(), cmp);

    std::vector<std::string> multilingual_names, multilingual_names_found, names_w_no_lang,
        supported_names;
    std::vector<baldr::Language> multilingual_langs_found, supported_langs;

    for (size_t i = 0; i < updated_token_languages.size(); ++i) {

      const auto& current_lang = updated_token_languages[i].second;

      if (current_lang.empty()) {
        // multilingual name
        // name = Place Saint-Pierre - Sint-Pietersplein
        std::vector<std::string> temp_names = GetTagTokens(updated_token_languages[i].first, " - ");
        if (temp_names.size() >= 2) {
          multilingual_names.insert(multilingual_names.end(), temp_names.begin(), temp_names.end());
          if (!diff_names)
            names_w_no_lang.insert(names_w_no_lang.end(), temp_names.begin(), temp_names.end());
        } else {
          temp_names = GetTagTokens(updated_token_languages[i].first, " / ");
          if (temp_names.size() >= 2) {
            multilingual_names.insert(multilingual_names.end(), temp_names.begin(), temp_names.end());
            if (!diff_names)
              names_w_no_lang.insert(names_w_no_lang.end(), temp_names.begin(), temp_names.end());
          } else {
            const auto& current_token = updated_token_languages[i].first;
            const auto& it =
                std::find_if(tokens_w_langs.begin(), tokens_w_langs.end(),
                             [&current_token](const std::pair<std::string, std::string>& p) {
                               return p.first == current_token;
                             });
            if (it != tokens_w_langs.end()) {
              new_sort_order.emplace_back(it->second);
            } else
              names_w_no_lang.emplace_back(updated_token_languages[i].first);
          }
        }
      } else if (std::find(multilingual_names.begin(), multilingual_names.end(),
                           updated_token_languages[i].first) != multilingual_names.end()) {
        multilingual_names_found.emplace_back(updated_token_languages[i].first);
        multilingual_langs_found.emplace_back(stringLanguage(current_lang));
        found_languages.erase(std::remove(found_languages.begin(), found_languages.end(),
                                          current_lang),
                              found_languages.end());
        if (!diff_names)
          names_w_no_lang.erase(std::remove(names_w_no_lang.begin(), names_w_no_lang.end(),
                                            updated_token_languages[i].first),
                                names_w_no_lang.end());

      } else if (std::find_if(default_languages.begin(), default_languages.end(),
                              [&current_lang](const std::pair<std::string, bool>& p) {
                                return p.first == current_lang;
                              }) != default_languages.end()) {

        supported_names.emplace_back(updated_token_languages[i].first);
        supported_langs.emplace_back(stringLanguage(current_lang));

        found_languages.erase(std::remove(found_languages.begin(), found_languages.end(),
                                          current_lang),
                              found_languages.end());
      }
    }
    bool multi_names = (multilingual_names_found.size());
    bool allowed_names = (supported_names.size() != 0);

    if (multi_names || allowed_names) {

      // did we find a name with a language in the name/destination key?  if so we need to redo the
      // sort order for the keys
      if (new_sort_order.size() != 1) {

        uint32_t count = 0;
        lang_sort_order.clear();

        for (const auto& lang : new_sort_order) {
          lang_sort_order[lang] = count++;
        }

        for (size_t i = 0; i < default_languages.size(); i++) {
          if (lang_sort_order.find(default_languages[i].first) == lang_sort_order.end())
            lang_sort_order[default_languages[i].first] = count++;
        }
      }

      tokens.clear();
      token_langs.clear();
      if (multi_names) {

        if (!diff_names && names_w_no_lang.size() >= 1 && found_languages.size() == 1) {

          std::vector<std::pair<std::string, std::string>> temp_token_languages;
          for (size_t i = 0; i < names_w_no_lang.size(); ++i) {
            temp_token_languages.emplace_back(names_w_no_lang[i], found_languages.at(0));
          }

          for (size_t i = 0; i < multilingual_names_found.size(); ++i) {
            temp_token_languages.emplace_back(multilingual_names_found[i],
                                              to_string(multilingual_langs_found[i]));
          }

          std::stable_sort(temp_token_languages.begin(), temp_token_languages.end(), cmp);

          for (size_t i = 0; i < temp_token_languages.size(); ++i) {
            tokens.emplace_back(temp_token_languages[i].first);
            token_langs.emplace_back(stringLanguage(temp_token_languages[i].second));
          }

        } else {
          tokens.insert(tokens.end(), multilingual_names_found.begin(),
                        multilingual_names_found.end());
          token_langs.insert(token_langs.end(), multilingual_langs_found.begin(),
                             multilingual_langs_found.end());
        }
      }
      if (allowed_names) {
        // assume the lang.
        if (names_w_no_lang.size() >= 1 && found_languages.size() == 1) {
          for (size_t i = 0; i < names_w_no_lang.size(); ++i) {
            tokens.emplace_back(names_w_no_lang.at(i));
            token_langs.emplace_back(stringLanguage(found_languages.at(0)));
          }

          for (size_t i = 0; i < supported_names.size(); ++i) {
            tokens.emplace_back(supported_names[i]);
            token_langs.emplace_back(supported_langs[i]);
          }
          // name key not found but all the langs were found.
        } else {
          std::vector<std::pair<std::string, std::string>> temp_token_languages;

          for (size_t i = 0; i < names_w_no_lang.size(); ++i) {
            temp_token_languages.emplace_back(names_w_no_lang[i], "");
          }

          for (size_t i = 0; i < supported_names.size(); ++i) {
            temp_token_languages.emplace_back(supported_names[i], to_string(supported_langs[i]));
          }

          std::stable_sort(temp_token_languages.begin(), temp_token_languages.end(), cmp);

          for (size_t i = 0; i < temp_token_languages.size(); ++i) {
            tokens.emplace_back(temp_token_languages[i].first);
            token_langs.emplace_back(stringLanguage(temp_token_languages[i].second));
          }
        }
      }
    } else { // bail
      tokens.clear();
      token_langs.clear();
      if ((updated_token_languages.size() > 1 && !all_blank && name_lang_index != 0) ||
          (default_languages.size() > 2 && all_blank && name_lang_index == 0))
        all_default = false;
      for (size_t i = 0; i < updated_token_languages.size(); ++i) {
        if (updated_token_languages[i].second.empty()) {
          tokens.emplace_back(updated_token_languages[i].first);
          if (all_default)
            token_langs.emplace_back(stringLanguage(default_languages.at(1).first));
        }
      }
    }
  }
}

// Get the names for the edge info based on the road class.
void OSMWay::GetNames(const std::string& ref,
                      const UniqueNames& name_offset_map,
                      const std::map<std::pair<uint8_t, uint8_t>, uint32_t>& pronunciationMap,
                      const std::map<std::pair<uint8_t, uint8_t>, uint32_t>& langMap,
                      const std::vector<std::pair<std::string, bool>>& default_languages,
                      const uint32_t ref_index,
                      const uint32_t ref_lang_index,
                      const uint32_t name_index,
                      const uint32_t name_lang_index,
                      const uint32_t official_name_index,
                      const uint32_t official_name_lang_index,
                      const uint32_t alt_name_index,
                      const uint32_t alt_name_lang_index,
                      uint16_t& types,
                      std::vector<std::string>& names,
                      std::vector<std::string>& linguistics,
                      OSMLinguistic::DiffType type,
                      bool diff_names) const {

  uint16_t location = 0;
  types = 0;
  const uint8_t ipa = static_cast<uint8_t>(PronunciationAlphabet::kIpa);
  const uint8_t nt_sampa = static_cast<uint8_t>(PronunciationAlphabet::kNtSampa);
  const uint8_t katakana = static_cast<uint8_t>(PronunciationAlphabet::kKatakana);
  const uint8_t jeita = static_cast<uint8_t>(PronunciationAlphabet::kJeita);

  auto get_pronunciation_index = [&pronunciationMap](const uint8_t type, const uint8_t alpha) {
    auto itr = pronunciationMap.find(std::make_pair(type, alpha));
    if (itr != pronunciationMap.end()) {
      return itr->second;
    }
    return static_cast<uint32_t>(0);
  };

  auto get_lang_index = [&langMap](const uint8_t type, const uint8_t alpha) {
    auto itr = langMap.find(std::make_pair(type, alpha));
    if (itr != langMap.end()) {
      return itr->second;
    }
    return static_cast<uint32_t>(0);
  };

  // Process motorway and trunk refs
  if ((ref_index != 0 || !ref.empty()) &&
      ((static_cast<RoadClass>(road_class_) == RoadClass::kMotorway) ||
       (static_cast<RoadClass>(road_class_) == RoadClass::kTrunk))) {
    std::vector<std::string> tokens;
    std::vector<baldr::Language> token_langs;
    std::vector<std::string> pronunciation_tokens;
    std::map<size_t, valhalla::baldr::Language> lang_map;

    if (!ref.empty()) {
      tokens = GetTagTokens(ref); // use updated refs from relations.
    } else {
      ProcessNamesPronunciations(name_offset_map, default_languages, ref_index, ref_lang_index,
                                 tokens, token_langs, diff_names);
    }

    for (size_t i = 0; i < tokens.size(); ++i) {
      types |= static_cast<uint64_t>(1) << location;
      location++;
    }

    names.insert(names.end(), tokens.begin(), tokens.end());

    size_t key = names.size() - tokens.size();
    size_t phoneme_start_index = linguistics.size();

    if (diff_names) {
      switch (type) {
        case OSMLinguistic::DiffType::kRight: {
          const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kRefRight);
          AddPronunciations(linguistics, lang_map, name_offset_map, default_languages, token_langs,
                            get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                            get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                            get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                            get_pronunciation_index(t, jeita), get_lang_index(t, jeita),
                            tokens.size(), key, diff_names);
          break;
        }
        case OSMLinguistic::DiffType::kLeft: {
          const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kRefLeft);
          AddPronunciations(linguistics, lang_map, name_offset_map, default_languages, token_langs,
                            get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                            get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                            get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                            get_pronunciation_index(t, jeita), get_lang_index(t, jeita),
                            tokens.size(), key, diff_names);
          break;
        }
        case OSMLinguistic::DiffType::kForward:
        case OSMLinguistic::DiffType::kBackward:
          break;
      }
    } else {
      const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kRef);
      AddPronunciations(linguistics, lang_map, name_offset_map, default_languages, token_langs,
                        get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                        get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                        get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                        get_pronunciation_index(t, jeita), get_lang_index(t, jeita), tokens.size(),
                        key, diff_names);
    }

    bool add_phoneme = (linguistics.size() - phoneme_start_index);

    for (size_t l = 0; l < token_langs.size(); ++l) {
      if (lang_map.size() != 0) {
        auto itr = lang_map.find(key);
        if (itr != lang_map.end()) {
          AddLanguage(linguistics, key, itr->second);
        }
      } else if (!add_phoneme)
        AddLanguage(linguistics, key, token_langs.at(l));
      key++;
    }
  }

  // TODO int_ref

  // Process name
  if (name_index != 0) {

    std::vector<std::string> tokens;
    std::vector<baldr::Language> token_langs;
    std::vector<std::string> pronunciation_tokens;
    std::map<size_t, valhalla::baldr::Language> lang_map;

    ProcessNamesPronunciations(name_offset_map, default_languages, name_index, name_lang_index,
                               tokens, token_langs, diff_names);

    location += tokens.size();
    names.insert(names.end(), tokens.begin(), tokens.end());

    size_t key = names.size() - tokens.size();
    size_t phoneme_start_index = linguistics.size();

    if (diff_names) {
      switch (type) {
        case OSMLinguistic::DiffType::kRight: {
          const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kNameRight);
          AddPronunciations(linguistics, lang_map, name_offset_map, default_languages, token_langs,
                            get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                            get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                            get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                            get_pronunciation_index(t, jeita), get_lang_index(t, jeita),
                            tokens.size(), key, diff_names);
          break;
        }
        case OSMLinguistic::DiffType::kLeft: {
          const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kNameLeft);
          AddPronunciations(linguistics, lang_map, name_offset_map, default_languages, token_langs,
                            get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                            get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                            get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                            get_pronunciation_index(t, jeita), get_lang_index(t, jeita),
                            tokens.size(), key, diff_names);
          break;
        }
        case OSMLinguistic::DiffType::kForward: {
          const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kNameForward);
          AddPronunciations(linguistics, lang_map, name_offset_map, default_languages, token_langs,
                            get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                            get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                            get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                            get_pronunciation_index(t, jeita), get_lang_index(t, jeita),
                            tokens.size(), key, diff_names);
          break;
        }
        case OSMLinguistic::DiffType::kBackward: {
          const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kNameBackward);
          AddPronunciations(linguistics, lang_map, name_offset_map, default_languages, token_langs,
                            get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                            get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                            get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                            get_pronunciation_index(t, jeita), get_lang_index(t, jeita),
                            tokens.size(), key, diff_names);
          break;
        }
      }
    } else {
      const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kName);
      AddPronunciations(linguistics, lang_map, name_offset_map, default_languages, token_langs,
                        get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                        get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                        get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                        get_pronunciation_index(t, jeita), get_lang_index(t, jeita), tokens.size(),
                        key, diff_names);
    }

    bool add_phoneme = (linguistics.size() - phoneme_start_index);

    for (size_t l = 0; l < token_langs.size(); ++l) {
      if (lang_map.size() != 0) {
        auto itr = lang_map.find(key);
        if (itr != lang_map.end()) {
          AddLanguage(linguistics, key, itr->second);
        }
      } else if (!add_phoneme)
        AddLanguage(linguistics, key, token_langs.at(l));
      key++;
    }
  }

  // create a set of keys for name list.  send to AddEdgeInfo?

  // Process non limited access refs
  if (ref_index != 0 && (static_cast<RoadClass>(road_class_) != RoadClass::kMotorway) &&
      (static_cast<RoadClass>(road_class_) != RoadClass::kTrunk)) {
    std::vector<std::string> tokens;
    std::vector<baldr::Language> token_langs;
    std::map<size_t, valhalla::baldr::Language> lang_map;

    if (!ref.empty()) {
      tokens = GetTagTokens(ref); // use updated refs from relations.
    } else {

      ProcessNamesPronunciations(name_offset_map, default_languages, ref_index, ref_lang_index,
                                 tokens, token_langs, diff_names);
    }

    for (size_t i = 0; i < tokens.size(); ++i) {
      types |= static_cast<uint64_t>(1) << location;
      location++;
    }

    names.insert(names.end(), tokens.begin(), tokens.end());

    size_t key = names.size() - tokens.size();
    size_t phoneme_start_index = linguistics.size();

    if (diff_names) {
      switch (type) {
        case OSMLinguistic::DiffType::kRight: {
          const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kRefRight);
          AddPronunciations(linguistics, lang_map, name_offset_map, default_languages, token_langs,
                            get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                            get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                            get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                            get_pronunciation_index(t, jeita), get_lang_index(t, jeita),
                            tokens.size(), key, diff_names);
          break;
        }
        case OSMLinguistic::DiffType::kLeft: {
          const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kRefLeft);
          AddPronunciations(linguistics, lang_map, name_offset_map, default_languages, token_langs,
                            get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                            get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                            get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                            get_pronunciation_index(t, jeita), get_lang_index(t, jeita),
                            tokens.size(), key, diff_names);
          break;
        }
        case OSMLinguistic::DiffType::kForward:
        case OSMLinguistic::DiffType::kBackward:
          break;
      }
    } else {
      const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kRef);
      AddPronunciations(linguistics, lang_map, name_offset_map, default_languages, token_langs,
                        get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                        get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                        get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                        get_pronunciation_index(t, jeita), get_lang_index(t, jeita), tokens.size(),
                        key, diff_names);
    }
    bool add_phoneme = (linguistics.size() - phoneme_start_index);

    for (size_t l = 0; l < token_langs.size(); ++l) {
      if (lang_map.size() != 0) {
        auto itr = lang_map.find(key);
        if (itr != lang_map.end()) {
          AddLanguage(linguistics, key, itr->second);
        }
      } else if (!add_phoneme)
        AddLanguage(linguistics, key, token_langs.at(l));
      key++;
    }
  }

  // Process alt_name
  if (alt_name_index != 0 && alt_name_index != name_index) {

    std::vector<std::string> tokens;
    std::vector<baldr::Language> token_langs;
    std::map<size_t, valhalla::baldr::Language> lang_map;

    ProcessNamesPronunciations(name_offset_map, default_languages, alt_name_index,
                               alt_name_lang_index, tokens, token_langs, diff_names);
    location += tokens.size();

    names.insert(names.end(), tokens.begin(), tokens.end());

    size_t key = names.size() - tokens.size();
    size_t phoneme_start_index = linguistics.size();

    if (diff_names) {
      switch (type) {
        case OSMLinguistic::DiffType::kRight: {
          const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kAltNameRight);
          AddPronunciations(linguistics, lang_map, name_offset_map, default_languages, token_langs,
                            get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                            get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                            get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                            get_pronunciation_index(t, jeita), get_lang_index(t, jeita),
                            tokens.size(), key, diff_names);
          break;
        }
        case OSMLinguistic::DiffType::kLeft: {
          const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kAltNameLeft);

          AddPronunciations(linguistics, lang_map, name_offset_map, default_languages, token_langs,
                            get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                            get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                            get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                            get_pronunciation_index(t, jeita), get_lang_index(t, jeita),
                            tokens.size(), key, diff_names);
          break;
        }
        case OSMLinguistic::DiffType::kForward:
        case OSMLinguistic::DiffType::kBackward:
          break;
      }
    } else {
      const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kAltName);

      AddPronunciations(linguistics, lang_map, name_offset_map, default_languages, token_langs,
                        get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                        get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                        get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                        get_pronunciation_index(t, jeita), get_lang_index(t, jeita), tokens.size(),
                        key, diff_names);
    }
    bool add_phoneme = (linguistics.size() - phoneme_start_index);

    for (size_t l = 0; l < token_langs.size(); ++l) {
      if (lang_map.size() != 0) {
        auto itr = lang_map.find(key);
        if (itr != lang_map.end()) {
          AddLanguage(linguistics, key, itr->second);
        }
      } else if (!add_phoneme)
        AddLanguage(linguistics, key, token_langs.at(l));
      key++;
    }
  }
  // Process official_name
  if (official_name_index != 0 && official_name_index != name_index &&
      official_name_index != alt_name_index) {

    std::vector<std::string> tokens;
    std::vector<baldr::Language> token_langs;
    std::map<size_t, valhalla::baldr::Language> lang_map;

    ProcessNamesPronunciations(name_offset_map, default_languages, official_name_index,
                               official_name_lang_index, tokens, token_langs, diff_names);
    location += tokens.size();

    names.insert(names.end(), tokens.begin(), tokens.end());

    size_t key = names.size() - tokens.size();
    size_t phoneme_start_index = linguistics.size();

    if (diff_names) {
      switch (type) {
        case OSMLinguistic::DiffType::kRight: {
          const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kOfficialNameRight);

          AddPronunciations(linguistics, lang_map, name_offset_map, default_languages, token_langs,
                            get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                            get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                            get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                            get_pronunciation_index(t, jeita), get_lang_index(t, jeita),
                            tokens.size(), key, diff_names);
          break;
        }
        case OSMLinguistic::DiffType::kLeft: {
          const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kOfficialNameLeft);

          AddPronunciations(linguistics, lang_map, name_offset_map, default_languages, token_langs,
                            get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                            get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                            get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                            get_pronunciation_index(t, jeita), get_lang_index(t, jeita),
                            tokens.size(), key, diff_names);
          break;
        }
        case OSMLinguistic::DiffType::kForward:
        case OSMLinguistic::DiffType::kBackward:
          break;
      }
    } else {
      const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kOfficialName);

      AddPronunciations(linguistics, lang_map, name_offset_map, default_languages, token_langs,
                        get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                        get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                        get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                        get_pronunciation_index(t, jeita), get_lang_index(t, jeita), tokens.size(),
                        key, diff_names);
    }
    bool add_phoneme = (linguistics.size() - phoneme_start_index);

    for (size_t l = 0; l < token_langs.size(); ++l) {
      if (lang_map.size() != 0) {
        auto itr = lang_map.find(key);
        if (itr != lang_map.end()) {
          AddLanguage(linguistics, key, itr->second);
        }
      } else if (!add_phoneme)
        AddLanguage(linguistics, key, token_langs.at(l));
      key++;
    }
  }
}

// Get the tagged names for an edge
void OSMWay::GetTaggedValues(const UniqueNames& name_offset_map,
                             const std::map<std::pair<uint8_t, uint8_t>, uint32_t>& pronunciationMap,
                             const std::map<std::pair<uint8_t, uint8_t>, uint32_t>& langMap,
                             const std::vector<std::pair<std::string, bool>>& default_languages,
                             const uint32_t tunnel_name_index,
                             const uint32_t tunnel_name_lang_index,
                             const size_t& names_size,
                             std::vector<std::string>& names,
                             std::vector<std::string>& linguistics,
                             OSMLinguistic::DiffType type,
                             bool diff_names) const {

  auto get_pronunciation_index = [&pronunciationMap](const uint8_t type, const uint8_t alpha) {
    auto itr = pronunciationMap.find(std::make_pair(type, alpha));
    if (itr != pronunciationMap.end()) {
      return itr->second;
    }
    return static_cast<uint32_t>(0);
  };

  auto get_lang_index = [&langMap](const uint8_t type, const uint8_t alpha) {
    auto itr = langMap.find(std::make_pair(type, alpha));
    if (itr != langMap.end()) {
      return itr->second;
    }
    return static_cast<uint32_t>(0);
  };
  auto encode_tag = [](TaggedValue tag) {
    return std::string(1, static_cast<std::string::value_type>(tag));
  };
  if (tunnel_name_index != 0) {
    // tunnel names

    std::vector<std::string> tokens;
    std::vector<baldr::Language> token_langs;
    std::map<size_t, valhalla::baldr::Language> lang_map;
    const uint8_t ipa = static_cast<uint8_t>(PronunciationAlphabet::kIpa);
    const uint8_t nt_sampa = static_cast<uint8_t>(PronunciationAlphabet::kNtSampa);
    const uint8_t katakana = static_cast<uint8_t>(PronunciationAlphabet::kKatakana);
    const uint8_t jeita = static_cast<uint8_t>(PronunciationAlphabet::kJeita);

    ProcessNamesPronunciations(name_offset_map, default_languages, tunnel_name_index,
                               tunnel_name_lang_index, tokens, token_langs, diff_names);

    for (const auto& t : tokens) {
      names.emplace_back(encode_tag(TaggedValue::kTunnel) + t);
    }

    size_t key = (names_size + names.size()) - tokens.size();
    size_t phoneme_start_index = linguistics.size();

    if (diff_names) {
      switch (type) {
        case OSMLinguistic::DiffType::kRight: {
          const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kTunnelNameRight);

          AddPronunciations(linguistics, lang_map, name_offset_map, default_languages, token_langs,
                            get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                            get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                            get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                            get_pronunciation_index(t, jeita), get_lang_index(t, jeita),
                            tokens.size(), key, diff_names);
          break;
        }
        case OSMLinguistic::DiffType::kLeft: {
          const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kTunnelNameLeft);

          AddPronunciations(linguistics, lang_map, name_offset_map, default_languages, token_langs,
                            get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                            get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                            get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                            get_pronunciation_index(t, jeita), get_lang_index(t, jeita),
                            tokens.size(), key, diff_names);
          break;
        }
        case OSMLinguistic::DiffType::kForward:
        case OSMLinguistic::DiffType::kBackward:
          break;
      }
    } else {
      const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kTunnelName);

      AddPronunciations(linguistics, lang_map, name_offset_map, default_languages, token_langs,
                        get_pronunciation_index(t, ipa), get_lang_index(t, ipa),
                        get_pronunciation_index(t, nt_sampa), get_lang_index(t, nt_sampa),
                        get_pronunciation_index(t, katakana), get_lang_index(t, katakana),
                        get_pronunciation_index(t, jeita), get_lang_index(t, jeita), tokens.size(),
                        key, diff_names);
    }
    bool add_phoneme = (linguistics.size() - phoneme_start_index);

    for (size_t l = 0; l < token_langs.size(); ++l) {
      if (lang_map.size() != 0) {
        auto itr = lang_map.find(key);
        if (itr != lang_map.end()) {
          AddLanguage(linguistics, key, itr->second);
        }
      } else if (!add_phoneme)
        AddLanguage(linguistics, key, token_langs.at(l));
      key++;
    }
  }

  if (layer_ != 0) {
    names.emplace_back(encode_tag(TaggedValue::kLayer) + static_cast<char>(layer_));
  }

  if (level_index_ != 0) {
    // level

    std::vector<std::pair<float, float>> values;
    const char dash = '-';
    auto tokens = GetTagTokens(name_offset_map.name(level_index_));

    // we store the precision once for all values
    // so we keep track of the max
    int precision = 0;
    for (size_t i = 0; i < tokens.size(); ++i) {
      const auto token = tokens[i];
      auto dash_pos = token.find(dash);
      std::pair<float, float> range;
      if (dash_pos != std::string::npos && dash_pos != 0) {
        // we're dealing with a range
        std::vector<std::string> nums;
        boost::algorithm::split(
            nums, token, [&](const char c) { return c == dash; },
            boost::algorithm::token_compress_on);

        try {
          std::smatch match;
          for (const auto& num : nums) {
            if (std::regex_search(num, match, kFloatRegex)) {
              precision = std::max(precision, static_cast<int>(match[1].str().size()));
            }
          }
          range.first = std::stof(nums[0]);
          range.second = std::stof(nums[1]);
        } catch (...) {
          LOG_WARN("Invalid level: " + token + "; way_id " + std::to_string(osmwayid_));
          continue;
        }

        if (range.first > range.second) {
          LOG_WARN("Invalid level range, " + std::to_string(range.first) + " - " +
                   std::to_string(range.second) + "; way_id " + std::to_string(osmwayid_));
          continue;
        }

      } else { // we have a number
        try {
          std::smatch match;
          if (std::regex_search(token, match, kFloatRegex)) {
            precision = std::max(precision, static_cast<int>(match[1].str().size()));
          }
          range.first = std::stof(token);
          range.second = range.first;
        } catch (...) {
          LOG_WARN("Invalid level: " + token + "; way_id " + std::to_string(osmwayid_));
          continue;
        }
      }
      values.emplace_back(range);
    }
    std::sort(values.begin(), values.end(),
              [](auto& left, auto& right) { return left.first < right.first; });

    std::string levels_encoded;
    // 2 bytes per value and 2 for each sentinel with some headroom should be good
    levels_encoded.reserve(values.size() * 4);

    // sentinel value that marks discontinuity
    std::string sep = encode_level(kLevelRangeSeparator);

    auto last_it = --values.end();
    auto prec_power = pow(10, precision);
    for (auto it = values.begin(); it != values.end(); ++it) {
      auto& range = *it;
      levels_encoded.append(encode_level(range.first * prec_power));
      if (range.first != range.second)
        levels_encoded.append(encode_level(range.second * prec_power));

      if (it != last_it)
        levels_encoded.append(sep);
    }

    std::string precision_enc = encode_level(static_cast<float>(precision));
    std::string encoded =
        encode_tag(TaggedValue::kLevels) +
        encode_level(static_cast<float>(levels_encoded.size() + precision_enc.size())) +
        precision_enc + levels_encoded;
    names.emplace_back(encoded);
  }
  if (level_ref_index_ != 0) {
    // level:ref
    auto tokens = GetTagTokens(name_offset_map.name(level_ref_index_));
    for (const auto& t : tokens) {
      names.emplace_back(encode_tag(TaggedValue::kLevelRef) + t);
    }
  }
}

} // namespace mjolnir
} // namespace valhalla
