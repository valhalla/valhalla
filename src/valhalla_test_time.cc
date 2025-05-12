#include "date/tz.h"
#include <iostream>
#include <valhalla/baldr/datetime.h>

int main() {
  namespace dt = valhalla::baldr::DateTime;
  using namespace date;
  using namespace std::chrono;
  using namespace std::chrono_literals;

  auto tz = get_tzdb().locate_zone("Europe/Berlin");

  std::string dti = "2024-03-26T00:00";
  auto parsed_dt = dt::get_local_time(dti, true);

  const auto then_date = make_zoned(tz, parsed_dt, date::choose::latest);

  const auto now_utc = time_point_cast<seconds>(date::utc_clock::now());
  const auto now_sys = time_point_cast<seconds>(system_clock::now());
  const auto now_tz_utc = make_zoned(tz, clock_cast<system_clock>(now_utc));
  const auto now_tz_sys = make_zoned(tz, now_sys);

  std::chrono::seconds dur(now_utc.time_since_epoch().count());
  std::chrono::time_point<std::chrono::system_clock> utp(dur);
  const auto tp = date::make_zoned(tz, utp).get_local_time();

  std::cout << tp << '\n';
  std::cout << now_utc << '\n';
  std::cout << date::utc_clock::from_local(tp) << '\n';
}
