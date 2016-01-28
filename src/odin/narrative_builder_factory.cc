#include <valhalla/odin/narrative_builder_factory.h>
#include <valhalla/odin/narrativebuilder.h>
#include <proto/directions_options.pb.h>
#include <valhalla/odin/enhancedtrippath.h>
#include <valhalla/odin/util.h>
#include <valhalla/midgard/util.h>
#include <boost/property_tree/ptree.hpp>

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

  return midgard::make_unique<NarrativeBuilder>(directions_options, trip_path,
                                                phrase_dictionary->second);
}

}
}
