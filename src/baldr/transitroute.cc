#include "baldr/transitroute.h"
#include <string.h>

namespace valhalla {
namespace baldr {

// Constructor with arguments
TransitRoute::TransitRoute(const TransitType route_type,
                           const uint32_t one_stop_offset,
                           const uint32_t op_by_onestop_id_offset,
                           const uint32_t op_by_name_offset,
                           const uint32_t op_by_website_offset,
                           const uint32_t route_color,
                           const uint32_t route_text_color,
                           const uint32_t short_name_offset,
                           const uint32_t long_name_offset,
                           const uint32_t desc_offset)
    : spare1_(0), spare2_(0), spare3_(0), spare4_(0) {

  route_type_ = static_cast<uint32_t>(route_type);
  route_color_ = route_color;
  route_text_color_ = route_text_color;

  if (one_stop_offset > kMaxNameOffset) {
    throw std::runtime_error("TransitRoute: Exceeded maximum name offset");
  }
  one_stop_offset_ = one_stop_offset;

  if (op_by_onestop_id_offset > kMaxNameOffset) {
    throw std::runtime_error("TransitRoute: Exceeded maximum name offset");
  }
  op_by_onestop_id_offset_ = op_by_onestop_id_offset;

  if (op_by_name_offset > kMaxNameOffset) {
    throw std::runtime_error("TransitRoute: Exceeded maximum name offset");
  }
  op_by_name_offset_ = op_by_name_offset;

  if (op_by_website_offset > kMaxNameOffset) {
    throw std::runtime_error("TransitRoute: Exceeded maximum name offset");
  }
  op_by_website_offset_ = op_by_website_offset;

  if (short_name_offset > kMaxNameOffset) {
    throw std::runtime_error("TransitRoute: Exceeded maximum name offset");
  }
  short_name_offset_ = short_name_offset;

  if (long_name_offset > kMaxNameOffset) {
    throw std::runtime_error("TransitRoute: Exceeded maximum name offset");
  }
  long_name_offset_ = long_name_offset;

  if (desc_offset > kMaxNameOffset) {
    throw std::runtime_error("TransitRoute: Exceeded maximum name offset");
  }
  desc_offset_ = desc_offset;
}

} // namespace baldr
} // namespace valhalla
