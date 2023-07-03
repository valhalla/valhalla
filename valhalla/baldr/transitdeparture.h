#ifndef VALHALLA_BALDR_TRANSITDEPARTURE_H_
#define VALHALLA_BALDR_TRANSITDEPARTURE_H_

#include <cstdint>
#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace baldr {

constexpr uint32_t kFixedSchedule = 0;
constexpr uint32_t kFrequencySchedule = 1;

struct FixedDeparture {
  uint64_t departure_time_ : 17; // Departure time (seconds from midnight)
                                 // (86400 secs per day)
  uint64_t elapsed_time_ : 17;   // Time (secs) until arrival at next stop
  uint64_t spare_ : 30;
};

struct FrequencyDeparture {
  uint64_t departure_time_ : 17; // Departure time (seconds from midnight)
  uint64_t end_time_ : 17;       // End time of departures (seconds from midnight)
  uint64_t frequency_ : 13;      // Interval between departures (seconds)
  uint64_t elapsed_time_ : 17;   // Time (secs) until arrival at next stop
};

/**
 * Information held for each departure from a transit stop. Departures within
 * a tile are ordered by the stop Id of the departure stop, followed by the
 * time of the departure.
 */
class TransitDeparture {
public:
  /**
   * Constructor for a fixed departure time.
   * @param  lineid   Unique line Id within the tile
   * @param  tripid   Unique trip Id (spans tiles)
   * @param  routeindex  Route index within the tile.
   * @param  blockid  Block Id.
   * @param  headsign_offset  Offset to headsign within the text/name table.
   * @param  departure_time   Departure time (seconds from midnight)
   * @param  elapsed_time     Elapsed time to next stop
   * @param  schedule_index   Index into the schedule validity table
   * @param  wheelchair_accessible  Is this a wheelchair accessible departure
   * @param  bicycle_accessible   Is this a bicycle accessible departure
   */
  TransitDeparture(const uint32_t lineid,
                   const uint32_t tripid,
                   const uint32_t routeindex,
                   const uint32_t blockid,
                   const uint32_t headsign_offset,
                   const uint32_t departure_time,
                   const uint32_t elapsed_time,
                   const uint32_t schedule_index,
                   const bool wheelchair_accessible,
                   const bool bicycle_accessible);

  /**
   * Constructor for a frequency based departure.
   * @param  lineid   Unique line Id within the tile
   * @param  tripid   Unique trip Id (spans tiles)
   * @param  routeindex  Route index within the tile.
   * @param  blockid  Block Id.
   * @param  headsign_offset  Offset to headsign within the text/name table.
   * @param  departure_time   Start time for the frequency (seconds from midnight)
   * @param  end_time     End time for departures (seconds from midnight)
   * @param  frequency    Seconds between successive departures.
   * @param  elapsed_time     Elapsed time to next stop
   * @param  schedule_index   Index into the schedule validity table
   * @param  wheelchair_accessible  Is this a wheelchair accessible departure
   * @param  bicycle_accessible   Is this a bicycle accessible departure
   */
  TransitDeparture(const uint32_t lineid,
                   const uint32_t tripid,
                   const uint32_t routeindex,
                   const uint32_t blockid,
                   const uint32_t headsign_offset,
                   const uint32_t departure_time,
                   const uint32_t end_time,
                   const uint32_t frequency,
                   const uint32_t elapsed_time,
                   const uint32_t schedule_index,
                   const bool wheelchair_accessible,
                   const bool bicycle_accessible);

  /**
   * Get the type of departure.
   * @return  Returns the departure type.
   */
  uint32_t type() const {
    return type_;
  }

  /**
   * Get the line Id - for lookup of all departures along this edge. Each
   * line Id represents a unique departure/arrival stop pair and route Id.
   * @return  Returns the departure line Id.
   */
  uint32_t lineid() const {
    return lineid_;
  }

  /**
   * Get the internal trip Id for this departure.
   * @return  Returns the internal trip Id.
   */
  uint32_t tripid() const {
    return tripid_;
  }

  /**
   * Get the route index for this departure.
   * @return  Returns the internal route Index.
   */
  uint32_t routeindex() const {
    return routeindex_;
  }

  /**
   * Get the block Id of this trip.
   * @return  Returns the block Id.
   */
  uint32_t blockid() const {
    return blockid_;
  }

  /**
   * Get the headsign offset into the names/text list.
   * @return  Returns the offset into the names/text list.
   */
  uint32_t headsign_offset() const {
    return headsign_offset_;
  }

  /**
   * Get the departure time.
   * @return  Returns the departure time in seconds from midnight.
   */
  uint32_t departure_time() const {
    return (type_ == kFixedSchedule) ? departure_times_.fixed_.departure_time_
                                     : departure_times_.frequency_.departure_time_;
  }

  /**
   * Get the elapsed time until arrival at the next stop.
   * @return  Returns the time in seconds.
   */
  uint32_t elapsed_time() const {
    return (type_ == kFixedSchedule) ? departure_times_.fixed_.elapsed_time_
                                     : departure_times_.frequency_.elapsed_time_;
  }

  /**
   * Get the end time of frequency based departures.
   * @return  Returns the end time in seconds from midnight.
   */
  uint32_t end_time() const {
    return departure_times_.frequency_.end_time_;
  }

  /**
   * Get the interval for frequency based departures.
   * @return  Returns the interval in seconds.
   */
  uint32_t frequency() const {
    return departure_times_.frequency_.frequency_;
  }

  /**
   * Get the schedule validity index.
   * @return  Returns the index into the transit schedules.
   */
  uint32_t schedule_index() const {
    return schedule_index_;
  }

  /**
   * Get the wheelchair accessible flag
   * @return  Returns the wheelchair accessible flag
   */
  bool wheelchair_accessible() const {
    return wheelchair_accessible_;
  }

  /**
   * Get the bicycle accessible flag
   * @return  Returns the bicycle accessible flag
   */
  bool bicycle_accessible() const {
    return bicycle_accessible_;
  }

  /**
   * operator < - for sorting. Sort by line Id and departure time.
   * @param  other  Other transit departure to compare to.
   * @return  Returns true if line Id < other line Id or if line Ids are
   *          equal and departure < other departure.
   */
  bool operator<(const TransitDeparture& other) const;

protected:
  uint64_t lineid_ : 20;     // Line Id - lookup departures by unique line
                             // Id (which indicates a unique departure /
                             // arrival stop pair.
  uint64_t routeindex_ : 12; // Route index.
  uint64_t tripid_ : 32;     // TripId (internal).

  uint64_t blockid_ : 20;         // Block Id
  uint64_t schedule_index_ : 12;  // Schedule validity index
  uint64_t headsign_offset_ : 24; // Headsign offset into the names/text list.
  uint64_t type_ : 2;             // Departure type (fixed, frequency)
  uint64_t wheelchair_accessible_ : 1;
  uint64_t bicycle_accessible_ : 1;
  uint64_t spare_ : 4;

  // Departure times
  union DepartureTimes {
    FixedDeparture fixed_;
    FrequencyDeparture frequency_;
  };
  DepartureTimes departure_times_;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_TRANSITDEPARTURE_H_
