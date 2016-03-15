#include <boost/property_tree/ptree.hpp>

#include <valhalla/midgard/util.h>

#include "proto/directions_options.pb.h"
#include "odin/narrative_builder_factory.h"
#include "odin/narrativebuilder.h"
#include "odin/enhancedtrippath.h"
#include "odin/util.h"

namespace valhalla {
namespace odin {

std::unique_ptr<NarrativeBuilder> NarrativeBuilderFactory::Create(
    const DirectionsOptions& directions_options,
    const EnhancedTripPath* trip_path) {

  // Get the locale dictionary
  const auto phrase_dictionary = get_locales().find(
      directions_options.language());

  // If language tag is not found then throw error
  if (phrase_dictionary == get_locales().end()) {
    throw std::runtime_error("Invalid language tag.");
  }

  if (directions_options.language() == "en-US") {
    return midgard::make_unique<NarrativeBuilder>(directions_options,
        trip_path, phrase_dictionary->second);
  }
  // TODO: add in other locale specific builders

  // TODO: set default lang to "en-US"
  return midgard::make_unique<NarrativeBuilder>(directions_options, trip_path,
                                                phrase_dictionary->second);
}

}
}
