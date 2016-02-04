#ifndef VALHALLA_ODIN_NARRATIVE_DICTIONARY_H_
#define VALHALLA_ODIN_NARRATIVE_DICTIONARY_H_

#include <vector>
#include <string>
#include <unordered_map>

#include <boost/property_tree/ptree.hpp>

namespace valhalla {
namespace odin {

struct PhraseSet {
  std::unordered_map<std::string, std::string> phrases;
};

struct StartSubset : PhraseSet {
  std::vector<std::string> cardinal_directions;
  std::vector<std::string> empty_street_name_labels;
  std::vector<std::string> empty_begin_street_name_labels;
};

struct StartVerbalSubset : StartSubset {
  std::vector<std::string> metric_lengths;
  std::vector<std::string> us_customary_lengths;
};

/**
 * TBD
 */
class NarrativeDictionary {
 public:
  NarrativeDictionary(const boost::property_tree::ptree& narrative_pt);

  StartSubset start_subset;
  StartVerbalSubset start_verbal_subset;

 protected:
  void Load(const boost::property_tree::ptree& narrative_pt);

  void Load(PhraseSet& phrase_handle, const boost::property_tree::ptree& pt);

  void Load(StartSubset& start_handle,
            const boost::property_tree::ptree& start_subset_pt);

  void Load(StartVerbalSubset& start_verbal_handle,
            const boost::property_tree::ptree& start_verbal_subset_pt);

};

}
}

#endif  // VALHALLA_ODIN_NARRATIVE_DICTIONARY_H_
