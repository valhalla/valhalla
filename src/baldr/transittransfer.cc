#include "baldr/transittransfer.h"

#include "midgard/logging.h"

namespace valhalla {
namespace baldr {

// Constructor with arguments
TransitTransfer::TransitTransfer(const uint32_t from_stopid,
                                 const uint32_t to_stopid,
                                 const TransferType type,
                                 const uint32_t mintime)
    : from_stopid_(from_stopid), to_stopid_(to_stopid), spare_(0) {
  type_ = static_cast<uint32_t>(type);

  if (mintime > kMaxTransferTime) {
    LOG_ERROR("TransitTransfer: Exceeded maximum transfer time");
    mintime_ = kMaxTransferTime;
  } else {
    mintime_ = mintime;
  }
}

// Get the from stop Id.
uint32_t TransitTransfer::from_stopid() const {
  return from_stopid_;
}

// Get the to stop Id.
uint32_t TransitTransfer::to_stopid() const {
  return to_stopid_;
}

// Gets the transfer type.
TransferType TransitTransfer::type() const {
  return static_cast<TransferType>(type_);
}

// Get the minimum time (seconds) to make the transfer.
uint32_t TransitTransfer::mintime() const {
  return mintime_;
}

// operator < - for sorting. Sort by from stop Id and to stop Id.
bool TransitTransfer::operator<(const TransitTransfer& other) const {
  if (from_stopid() == other.from_stopid()) {
    return to_stopid() < other.to_stopid();
  } else {
    return from_stopid() < other.from_stopid();
  }
}

} // namespace baldr
} // namespace valhalla
