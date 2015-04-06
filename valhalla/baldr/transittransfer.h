#ifndef VALHALLA_BALDR_TRANSITTRANSFER_H_
#define VALHALLA_BALDR_TRANSITTRANSFER_H_

#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace baldr {

/**
 * Transit transfer information between stops.
 */
class TransitTransfer {
 public:
  /**
   * Get the from stop Id.
   * @return  Returns the from stop Id.
   */
  uint32_t from_stopid() const;

  /**
   * Get the to stop Id.
   * @return  Returns the to stop Id.
   */
  uint32_t to_stopid() const;

  /**
   * Gets the transfer type.
   * @return  Returns the transfer type.
   */
  TransferType type() const;

  /**
   * Get the minimum time (seconds) to make the transfer.
   * @return  Returns the minimum transfer time (seconds).
   */
  uint32_t mintime() const;

 protected:
  // From stop Id (internal)
  uint32_t from_stopid_;

  // To stop Id (internal)
  uint32_t to_stopid_;

  struct Transfer {
    uint32_t type     : 4;   // Transfer type
    uint32_t mintime : 28;   // Minimum transfer time (seconds)
  };
  Transfer transfer_;
};

}
}

#endif  // VALHALLA_BALDR_TRANSITTRANSFER_H_
