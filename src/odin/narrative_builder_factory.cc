#include "midgard/util.h"

#include "odin/enhancedtrippath.h"
#include "odin/narrative_builder_factory.h"
#include "odin/narrativebuilder.h"
#include "odin/util.h"

#include <valhalla/proto/options.pb.h>

namespace valhalla {
namespace odin {

std::unique_ptr<NarrativeBuilder> NarrativeBuilderFactory::Create(const Options& options,
                                                                  const EnhancedTripLeg* trip_path) {

  // Get the locale dictionary
  const auto phrase_dictionary = get_locales().find(options.language());

  // If language tag is not found then throw error
  if (phrase_dictionary == get_locales().end()) {
    throw std::runtime_error("Invalid language tag.");
  }

  // if a NarrativeBuilder is derived with specific code for a particular
  // language then add logic here and return derived NarrativeBuilder
  if (phrase_dictionary->second->GetLanguageTag() == "cs-CZ") {
    return std::make_unique<NarrativeBuilder_csCZ>(options, trip_path, *phrase_dictionary->second);
  } else if (phrase_dictionary->second->GetLanguageTag() == "hi-IN") {
    return std::make_unique<NarrativeBuilder_hiIN>(options, trip_path, *phrase_dictionary->second);
  } else if (phrase_dictionary->second->GetLanguageTag() == "it-IT") {
    return std::make_unique<NarrativeBuilder_itIT>(options, trip_path, *phrase_dictionary->second);
  } else if (phrase_dictionary->second->GetLanguageTag() == "ru-RU") {
    return std::make_unique<NarrativeBuilder_ruRU>(options, trip_path, *phrase_dictionary->second);
  }

  // otherwise just return pointer to NarrativeBuilder
  return std::make_unique<NarrativeBuilder>(options, trip_path, *phrase_dictionary->second);
}

} // namespace odin
} // namespace valhalla
