#include "baldr/transittransfer.h"

namespace valhalla {
namespace baldr {

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
  return static_cast<TransferType>(transfer_.type);
}

// Get the minimum time (seconds) to make the transfer.
uint32_t TransitTransfer::mintime() const {
  return transfer_.mintime;
}

}
}
