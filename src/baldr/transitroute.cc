#include "baldr/transitroute.h"
#include <boost/functional/hash.hpp>
#include <cmath>

namespace valhalla {
namespace baldr {

// Default constructor
TransitRoute::TransitRoute()
    : routeid_(0),
      agencyid_(0),
      tl_routeid_{},
      short_name_index_(0),
      long_name_index_(0),
      desc_index_(0) {
}

// Get the internal route Id.
uint32_t TransitRoute::routeid() const {
  return routeid_;
}

/**
* Get the internal agency Id for this route.
* @return  Returns the internal agency Id.
*/
uint32_t TransitRoute::agencyid() const {
  return agencyid_;
}

/**
* Get the TransitLand one stop Id for this route.
* @return  Returns the TransitLand one route Id.
*/
const char* TransitRoute::tl_routeid() const {
  return tl_routeid_;
}

/**
* Get the text/name index for the short route name.
* @return  Returns the short name index in the text/name list.
*/
uint32_t TransitRoute::short_name_index() const {
  return short_name_index_;
}

/**
* Get the text/name index for the long route name.
* @return  Returns the short name index in the text/name list.
*/
uint32_t TransitRoute::long_name_index() const {
  return long_name_index_;
}

/**
* Get the text/name index for the route description.
* @return  Returns the description index in the text/name list.
*/
uint32_t TransitRoute::desc_index() const {
  return desc_index_;
}

}
}
