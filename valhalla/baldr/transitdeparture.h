#ifndef VALHALLA_BALDR_TRANSITDEPARTURE_H_
#define VALHALLA_BALDR_TRANSITDEPARTURE_H_

#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace baldr {

/**
 * Information held for each departure from a transit stop. Departures within
 * a tile are ordered by the stop Id of the departure stop, followed by the
 * time of the departure.
 */
class TransitDeparture {
 public:
  /**
   * Constructor with arguments.
   * @param  lineid   Unique line Id within the tile
   * @param  tripid   Unique trip Id (spans tiles)
   * @param  routeid  Route index within the tile.
   * @param  blockid  Block Id.
   * @param  headsign_offset  Offset to headsign within the text/name table.
   * @param  departure_time   Departure time (seconds from midnight)
   * @param  elapsed_time     Elapsed time to next stop
   * @param  schedule_index   Index into schedule validity table for this tile
   */
  TransitDeparture(const uint32_t lineid, const uint32_t tripid,
                   const uint32_t routeid, const uint32_t blockid,
                   const uint32_t headsign_offset,
                   const uint32_t departure_time,
                   const uint32_t elapsed_time,
                   const uint32_t schedule_index);

  /**
   * Get the line Id - for lookup of all departures along this edge. Each
   * line Id represents a unique departure/arrival stop pair and route Id.
   * @return  Returns the departure line Id.
   */
  uint32_t lineid() const;

  /**
   * Get the internal trip Id for this departure.
   * @return  Returns the internal trip Id.
   */
  uint32_t tripid() const;

  /**
   * Get the route index for this departure.
   * @return  Returns the internal route Id.
   */
  uint32_t routeid() const;

  /**
   * Get the block Id of this trip.
   * @return  Returns the block Id.
   */
  uint32_t blockid() const;

  /**
   * Get the headsign offset into the names/text list.
   * @return  Returns the offset into the names/text list.
   */
  uint32_t headsign_offset() const;

  /**
   * Get the departure time.
   * @return  Returns the departure time in seconds from midnight.
   */
  uint32_t departure_time() const;

  /**
   * Get the elapsed time until arrival at the next stop.
   * @return  Returns the time in seconds.
   */
  uint32_t elapsed_time() const;

  /**
   * Get the schedule validity index.
   * @return  Returns the index into the transit schedules.
   */
  uint32_t schedule_index() const;

  /**
   * operator < - for sorting. Sort by line Id and departure time.
   * @param  other  Other transit departure to compare to.
   * @return  Returns true if line Id < other line Id or if line Ids are
   *          equal and departure < other departure.
   */
  bool operator < (const TransitDeparture& other) const;

 protected:
  uint32_t lineid_          : 20; // Line Id - lookup departures by unique line
                                  // Id (which indicates a unique departure /
                                  // arrival stop pair.
  uint32_t routeid_         : 12; // Route index.

  uint32_t tripid_;               // TripId (internal).
  uint32_t headsign_offset_;      // Headsign offset into the names/text list.

  uint32_t blockid_         : 20;  // Block Id
  uint32_t schedule_index_  : 12; // Schedule validity index


  uint32_t departure_time_  : 17; // Departure time (seconds from midnight)
                                  // (86400 secs per day)
  uint32_t elapsed_time_    : 15; // Time (secs) until arrival at next stop
};

}
}

#endif  // VALHALLA_BALDR_TRANSITDEPARTURE_H_
