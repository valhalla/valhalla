#ifndef VALHALLA_BALDR_TRANSITTRANSFER_H_
#define VALHALLA_BALDR_TRANSITTRANSFER_H_

#include <cstdint>
#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace baldr {

/**
 * Transit transfer information between stops.
 */
class TransitTransfer {
public:
  // Constructor with arguments
  TransitTransfer(const uint32_t from_stopid,
                  const uint32_t to_stopid,
                  const TransferType type,
                  const uint32_t mintime);

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

  /**
   * operator < - for sorting. Sort by from stop Id and to stop Id.
   * @param  other  Other transit transfer to compare to.
   * @return  Returns true if from stop Id < other from stop Id or
   *          from stop Ids are equal and to stop Id < other to stop Id.
   */
  bool operator<(const TransitTransfer& other) const;

protected:
  uint32_t from_stopid_; // From stop Id (internal)

  uint32_t to_stopid_; // To stop Id (internal)

  uint32_t type_ : 4;     // Transfer type
  uint32_t mintime_ : 16; // Minimum transfer time (seconds)
  uint32_t spare_ : 12;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_TRANSITTRANSFER_H_
