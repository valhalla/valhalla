#pragma once

#include <algorithm>
#include <boost/optional.hpp>
#include <cassert>
#include <cstdint>
#include <exception>
#include <fstream>
#include <functional>
#include <iomanip>
#include <istream>
#include <iterator>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace gtfs {
// File names and other entities defined in GTFS----------------------------------------------------
const std::string file_agency = "agency.txt";
const std::string file_stops = "stops.txt";
const std::string file_routes = "routes.txt";
const std::string file_trips = "trips.txt";
const std::string file_stop_times = "stop_times.txt";
const std::string file_calendar = "calendar.txt";
const std::string file_calendar_dates = "calendar_dates.txt";
const std::string file_fare_attributes = "fare_attributes.txt";
const std::string file_fare_rules = "fare_rules.txt";
const std::string file_shapes = "shapes.txt";
const std::string file_frequencies = "frequencies.txt";
const std::string file_transfers = "transfers.txt";
const std::string file_pathways = "pathways.txt";
const std::string file_levels = "levels.txt";
const std::string file_feed_info = "feed_info.txt";
const std::string file_translations = "translations.txt";
const std::string file_attributions = "attributions.txt";

constexpr char csv_separator = ',';
constexpr char quote = '"';

// Helper classes and functions---------------------------------------------------------------------
struct InvalidFieldFormat : public std::exception {
public:
  explicit InvalidFieldFormat(const std::string& msg) : message(prefix + msg) {
  }

  const char* what() const noexcept {
    return message.c_str();
  }

private:
  const std::string prefix = "Invalid GTFS field format. ";
  std::string message;
};

enum ResultCode {
  OK,
  END_OF_FILE,
  ERROR_INVALID_GTFS_PATH,
  ERROR_FILE_ABSENT,
  ERROR_REQUIRED_FIELD_ABSENT,
  ERROR_INVALID_FIELD_FORMAT
};

using Message = std::string;

struct Result {
  Result() = default;
  Result(ResultCode&& in_code) : code(in_code) {
  }
  Result(const ResultCode& in_code, const Message& msg) : code(in_code), message(msg) {
  }
  bool operator==(ResultCode result_code) const {
    return code == result_code;
  }
  bool operator!=(ResultCode result_code) const {
    return !(*this == result_code);
  }

  ResultCode code = OK;
  Message message;
};

inline std::string add_trailing_slash(const std::string& path) {
  auto extended_path = path;
  if (!extended_path.empty() && extended_path.back() != '/')
    extended_path += "/";
  return extended_path;
}

inline void write_joined(std::ofstream& out, std::vector<std::string>&& elements) {
  for (size_t i = 0; i < elements.size(); ++i) {
    out << elements[i];
    if (i != elements.size() - 1)
      out << csv_separator;
  }
  out << std::endl;
}

inline std::string quote_text(const std::string& text) {
  std::stringstream stream;
  stream << std::quoted(text, quote, quote);
  return stream.str();
}

inline std::string unquote_text(const std::string& text) {
  std::string res;
  bool prev_is_quote = false;
  bool prev_is_skipped = false;

  size_t start_index = 0;
  size_t end_index = text.size();

  // Field values that contain quotation marks or commas must be enclosed within quotation marks.
  if (text.size() > 1 && text.front() == quote && text.back() == quote) {
    ++start_index;
    --end_index;
  }

  // In addition, each quotation mark in the field value must be preceded with a quotation mark.
  for (size_t i = start_index; i < end_index; ++i) {
    if (text[i] != quote) {
      res += text[i];
      prev_is_quote = false;
      prev_is_skipped = false;
      continue;
    }

    if (prev_is_quote) {
      if (prev_is_skipped)
        res += text[i];

      prev_is_skipped = !prev_is_skipped;
    } else {
      prev_is_quote = true;
      res += text[i];
    }
  }

  return res;
}

// Csv field values that contain quotation marks or commas must be enclosed within quotation marks.
inline std::string wrap(const std::string& text) {
  static const std::string symbols = std::string(1, quote) + std::string(1, csv_separator);

  if (text.find_first_of(symbols) == std::string::npos)
    return text;

  return quote_text(text);
}

// Save to csv enum value as unsigned integer.
template <typename T>
std::enable_if_t<std::is_integral<T>::value || std::is_enum<T>::value, std::string>
wrap(const T& val) {
  return std::to_string(static_cast<int>(val));
}

// Save to csv coordinates with custom precision.
inline std::string wrap(double val) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6);
  stream << val;
  return stream.str();
}

inline void write_agency_header(std::ofstream& out) {
  std::vector<std::string> fields = {"agency_id",       "agency_name", "agency_url",
                                     "agency_timezone", "agency_lang", "agency_phone",
                                     "agency_fare_url", "agency_email"};
  write_joined(out, std::move(fields));
}

inline void write_routes_header(std::ofstream& out) {
  std::vector<std::string> fields = {"route_id",         "agency_id",         "route_short_name",
                                     "route_long_name",  "route_desc",        "route_type",
                                     "route_url",        "route_color",       "route_text_color",
                                     "route_sort_order", "continuous_pickup", "continuous_drop_off"};
  write_joined(out, std::move(fields));
}

inline void write_shapes_header(std::ofstream& out) {
  std::vector<std::string> fields = {"shape_id", "shape_pt_lat", "shape_pt_lon", "shape_pt_sequence"};
  write_joined(out, std::move(fields));
}

inline void write_trips_header(std::ofstream& out) {
  std::vector<std::string> fields = {"route_id",      "service_id",      "trip_id",
                                     "trip_headsign", "trip_short_name", "direction_id",
                                     "block_id",      "shape_id",        "wheelchair_accessible",
                                     "bikes_allowed"};
  write_joined(out, std::move(fields));
}

inline void write_stops_header(std::ofstream& out) {
  std::vector<std::string> fields = {"stop_id",        "stop_code",     "stop_name",
                                     "stop_desc",      "stop_lat",      "stop_lon",
                                     "zone_id",        "stop_url",      "location_type",
                                     "parent_station", "stop_timezone", "wheelchair_boarding",
                                     "level_id",       "platform_code"};
  write_joined(out, std::move(fields));
}

inline void write_stop_times_header(std::ofstream& out) {
  std::vector<std::string> fields =
      {"trip_id",           "arrival_time",        "departure_time",      "stop_id",
       "stop_sequence",     "stop_headsign",       "pickup_type",         "drop_off_type",
       "continuous_pickup", "continuous_drop_off", "shape_dist_traveled", "timepoint"};
  write_joined(out, std::move(fields));
}

inline void write_calendar_header(std::ofstream& out) {
  std::vector<std::string> fields = {"service_id", "monday",   "tuesday", "wednesday",  "thursday",
                                     "friday",     "saturday", "sunday",  "start_date", "end_date"};
  write_joined(out, std::move(fields));
}

inline void write_calendar_dates_header(std::ofstream& out) {
  std::vector<std::string> fields = {"service_id", "date", "exception_type"};
  write_joined(out, std::move(fields));
}

inline void write_transfers_header(std::ofstream& out) {
  std::vector<std::string> fields = {"from_stop_id", "to_stop_id", "transfer_type",
                                     "min_transfer_time"};
  write_joined(out, std::move(fields));
}

inline void write_frequencies_header(std::ofstream& out) {
  std::vector<std::string> fields = {"trip_id", "start_time", "end_time", "headway_secs",
                                     "exact_times"};
  write_joined(out, std::move(fields));
}

inline void write_fare_attributes_header(std::ofstream& out) {
  std::vector<std::string> fields = {"fare_id",   "price",     "currency_type",    "payment_method",
                                     "transfers", "agency_id", "transfer_duration"};
  write_joined(out, std::move(fields));
}

inline void write_fare_rules_header(std::ofstream& out) {
  std::vector<std::string> fields = {"fare_id", "route_id", "origin_id", "destination_id",
                                     "contains_id"};
  write_joined(out, std::move(fields));
}

inline void write_pathways_header(std::ofstream& out) {
  std::vector<std::string> fields = {"pathway_id",     "from_stop_id",     "to_stop_id",
                                     "pathway_mode",   "is_bidirectional", "length",
                                     "traversal_time", "stair_count",      "max_slope",
                                     "min_width",      "signposted_as",    "reversed_signposted_as"};
  write_joined(out, std::move(fields));
}

inline void write_levels_header(std::ofstream& out) {
  std::vector<std::string> fields = {"level_id", "level_index", "level_name"};
  write_joined(out, std::move(fields));
}

inline void write_feed_info_header(std::ofstream& out) {
  std::vector<std::string> fields = {"feed_publisher_name", "feed_publisher_url", "feed_lang",
                                     "default_lang",        "feed_start_date",    "feed_end_date",
                                     "feed_version",        "feed_contact_email", "feed_contact_url"};
  write_joined(out, std::move(fields));
}

inline void write_translations_header(std::ofstream& out) {
  std::vector<std::string> fields = {"table_name", "field_name",    "language",   "translation",
                                     "record_id",  "record_sub_id", "field_value"};
  write_joined(out, std::move(fields));
}

inline void write_attributions_header(std::ofstream& out) {
  std::vector<std::string> fields = {"attribution_id",    "agency_id",         "route_id",
                                     "trip_id",           "organization_name", "is_producer",
                                     "is_operator",       "is_authority",      "attribution_url",
                                     "attribution_email", "attribution_phone"};
  write_joined(out, std::move(fields));
}

// Csv parser  -------------------------------------------------------------------------------------
class CsvParser {
public:
  CsvParser() = default;
  inline explicit CsvParser(const std::string& gtfs_directory);

  inline Result read_header(const std::string& csv_filename);
  inline Result read_row(std::map<std::string, std::string>& obj);

  inline static std::vector<std::string> split_record(const std::string& record,
                                                      bool is_header = false);

private:
  std::vector<std::string> field_sequence;
  std::string gtfs_path;
  std::ifstream csv_stream;
};

inline CsvParser::CsvParser(const std::string& gtfs_directory) : gtfs_path(gtfs_directory) {
}

inline std::string trim_spaces(const std::string& token) {
  static const std::string delimiters = " \t";
  std::string res = token;
  res.erase(0, res.find_first_not_of(delimiters));
  res.erase(res.find_last_not_of(delimiters) + 1);
  return res;
}

inline std::string normalize(std::string& token, bool has_quotes) {
  std::string res = trim_spaces(token);
  if (has_quotes)
    return unquote_text(res);
  return res;
}

inline std::vector<std::string> CsvParser::split_record(const std::string& record, bool is_header) {
  size_t start_index = 0;
  if (is_header) {
    // ignore UTF-8 BOM prefix:
    if (record.size() > 2 && record[0] == '\xef' && record[1] == '\xbb' && record[2] == '\xbf')
      start_index = 3;
  }

  std::vector<std::string> fields;
  fields.reserve(20);

  std::string token;
  token.reserve(record.size());

  bool is_inside_quotes = false;
  bool quotes_in_token = false;

  for (size_t i = start_index; i < record.size(); ++i) {
    if (record[i] == quote) {
      is_inside_quotes = !is_inside_quotes;
      quotes_in_token = true;
      token += record[i];
      continue;
    }

    if (record[i] == csv_separator) {
      if (is_inside_quotes) {
        token += record[i];
        continue;
      }

      fields.emplace_back(normalize(token, quotes_in_token));
      token.clear();
      quotes_in_token = false;
      continue;
    }

    // Skip delimiters:
    if (record[i] != '\t' && record[i] != '\r')
      token += record[i];
  }

  fields.emplace_back(normalize(token, quotes_in_token));
  return fields;
}

inline Result CsvParser::read_header(const std::string& csv_filename) {
  if (csv_stream.is_open())
    csv_stream.close();

  csv_stream.open(gtfs_path + csv_filename);
  if (!csv_stream.is_open())
    return {ResultCode::ERROR_FILE_ABSENT, "File " + csv_filename + " could not be opened"};

  std::string header;
  if (!getline(csv_stream, header) || header.empty())
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, "Empty header in file " + csv_filename};

  field_sequence = split_record(header, true);
  return ResultCode::OK;
}

inline Result CsvParser::read_row(std::map<std::string, std::string>& obj) {
  obj = {};
  std::string row;
  if (!getline(csv_stream, row))
    return {ResultCode::END_OF_FILE, {}};

  if (row == "\r")
    return ResultCode::OK;

  const std::vector<std::string> fields_values = split_record(row);

  // Different count of fields in the row and in the header of csv.
  // Typical approach is to skip not required fields.
  const size_t fields_count = std::min(field_sequence.size(), fields_values.size());

  for (size_t i = 0; i < fields_count; ++i)
    obj[field_sequence[i]] = fields_values[i];

  return ResultCode::OK;
}

// Custom types for GTFS fields --------------------------------------------------------------------
// Id of GTFS entity, a sequence of any UTF-8 characters. Used as type for ID GTFS fields.
using Id = std::string;
// A string of UTF-8 characters. Used as type for Text GTFS fields.
using Text = std::string;

// Time in GTFS is in the HH:MM:SS format (H:MM:SS is also accepted)
// Time within a service day can be above 24:00:00, e.g. 28:41:30
class Time {
public:
  inline Time() = default;
  inline explicit Time(const std::string& raw_time_str);
  inline Time(uint16_t hours, uint16_t minutes, uint16_t seconds);
  inline bool is_provided() const;
  inline size_t get_total_seconds() const;
  inline std::tuple<uint16_t, uint16_t, uint16_t> get_hh_mm_ss() const;
  inline std::string get_raw_time() const;
  inline bool limit_hours_to_24max();

private:
  inline void set_total_seconds();
  inline void set_raw_time();
  bool time_is_provided = false;
  std::string raw_time;
  size_t total_seconds = 0;
  uint16_t hh = 0;
  uint16_t mm = 0;
  uint16_t ss = 0;
};

inline bool operator==(const Time& lhs, const Time& rhs) {
  return lhs.get_hh_mm_ss() == rhs.get_hh_mm_ss() && lhs.is_provided() == rhs.is_provided();
}

inline bool Time::limit_hours_to_24max() {
  if (hh < 24)
    return false;

  hh = hh % 24;
  set_total_seconds();
  set_raw_time();
  return true;
}

inline void Time::set_total_seconds() {
  total_seconds = hh * 60 * 60 + mm * 60 + ss;
}

inline std::string append_leading_zero(const std::string& s, bool check = true) {
  if (check && s.size() > 2)
    throw InvalidFieldFormat("The string for appending zero is too long: " + s);

  if (s.size() == 2)
    return s;
  return "0" + s;
}

inline void Time::set_raw_time() {
  const std::string hh_str = append_leading_zero(std::to_string(hh), false);
  const std::string mm_str = append_leading_zero(std::to_string(mm));
  const std::string ss_str = append_leading_zero(std::to_string(ss));

  raw_time = hh_str + ":" + mm_str + ":" + ss_str;
}

// Time in the HH:MM:SS format (H:MM:SS is also accepted). Used as type for Time GTFS fields.
inline Time::Time(const std::string& raw_time_str) : raw_time(raw_time_str) {
  if (raw_time_str.empty())
    return;

  const size_t len = raw_time.size();
  if (!(len >= 7 && len <= 9) || raw_time[len - 3] != ':' || raw_time[len - 6] != ':')
    throw InvalidFieldFormat("Time is not in [[H]H]H:MM:SS format: " + raw_time_str);

  hh = static_cast<uint16_t>(std::stoi(raw_time.substr(0, len - 6)));
  mm = static_cast<uint16_t>(std::stoi(raw_time.substr(len - 5, 2)));
  ss = static_cast<uint16_t>(std::stoi(raw_time.substr(len - 2)));

  if (mm > 60 || ss > 60)
    throw InvalidFieldFormat("Time minutes/seconds wrong value: " + std::to_string(mm) +
                             " minutes, " + std::to_string(ss) + " seconds");

  set_total_seconds();
  time_is_provided = true;
}

inline Time::Time(uint16_t hours, uint16_t minutes, uint16_t seconds)
    : hh(hours), mm(minutes), ss(seconds) {
  if (mm > 60 || ss > 60)
    throw InvalidFieldFormat("Time is out of range: " + std::to_string(mm) + "minutes " +
                             std::to_string(ss) + "seconds");

  set_total_seconds();
  set_raw_time();
  time_is_provided = true;
}

inline bool Time::is_provided() const {
  return time_is_provided;
}

inline size_t Time::get_total_seconds() const {
  return total_seconds;
}

inline std::tuple<uint16_t, uint16_t, uint16_t> Time::get_hh_mm_ss() const {
  return {hh, mm, ss};
}

inline std::string Time::get_raw_time() const {
  return raw_time;
}

// Service day in the YYYYMMDD format.
class Date {
public:
  inline Date() = default;
  inline Date(uint16_t year, uint16_t month, uint16_t day);
  inline explicit Date(const std::string& raw_date_str);
  inline bool is_provided() const;
  inline std::tuple<uint16_t, uint16_t, uint16_t> get_yyyy_mm_dd() const;
  inline std::string get_raw_date() const;

private:
  inline void check_valid() const;

  std::string raw_date;
  uint16_t yyyy = 0;
  uint16_t mm = 0;
  uint16_t dd = 0;
  bool date_is_provided = false;
};

inline bool operator==(const Date& lhs, const Date& rhs) {
  return lhs.get_yyyy_mm_dd() == rhs.get_yyyy_mm_dd() && lhs.is_provided() == rhs.is_provided();
}

inline void Date::check_valid() const {
  if (yyyy < 1000 || yyyy > 9999 || mm < 1 || mm > 12 || dd < 1 || dd > 31)
    throw InvalidFieldFormat("Date check failed: out of range. " + std::to_string(yyyy) + " year, " +
                             std::to_string(mm) + " month, " + std::to_string(dd) + " day");

  if (mm == 2 && dd > 28) {
    // The year is not leap. Days count should be 28.
    if (yyyy % 4 != 0 || (yyyy % 100 == 0 && yyyy % 400 != 0))
      throw InvalidFieldFormat("Invalid days count in February of non-leap year: " +
                               std::to_string(dd) + " year" + std::to_string(yyyy));

    // The year is leap. Days count should be 29.
    if (dd > 29)
      throw InvalidFieldFormat("Invalid days count in February of leap year: " + std::to_string(dd) +
                               " year" + std::to_string(yyyy));
  }

  if (dd > 30 && (mm == 4 || mm == 6 || mm == 9 || mm == 11))
    throw InvalidFieldFormat("Invalid days count in month: " + std::to_string(dd) + " days in " +
                             std::to_string(mm));
}

inline Date::Date(uint16_t year, uint16_t month, uint16_t day) : yyyy(year), mm(month), dd(day) {
  check_valid();
  const std::string mm_str = append_leading_zero(std::to_string(mm));
  const std::string dd_str = append_leading_zero(std::to_string(dd));

  raw_date = std::to_string(yyyy) + mm_str + dd_str;
  date_is_provided = true;
}

inline Date::Date(const std::string& raw_date_str) : raw_date(raw_date_str) {
  if (raw_date.empty())
    return;

  if (raw_date.size() != 8)
    throw InvalidFieldFormat("Date is not in YYYY:MM::DD format: " + raw_date_str);

  yyyy = static_cast<uint16_t>(std::stoi(raw_date.substr(0, 4)));
  mm = static_cast<uint16_t>(std::stoi(raw_date.substr(4, 2)));
  dd = static_cast<uint16_t>(std::stoi(raw_date.substr(6, 2)));

  check_valid();

  date_is_provided = true;
}

inline bool Date::is_provided() const {
  return date_is_provided;
}

inline std::tuple<uint16_t, uint16_t, uint16_t> Date::get_yyyy_mm_dd() const {
  return {yyyy, mm, dd};
}

inline std::string Date::get_raw_date() const {
  return raw_date;
}

// An ISO 4217 alphabetical currency code. Used as type for Currency Code GTFS fields.
using CurrencyCode = std::string;
// An IETF BCP 47 language code. Used as type for Language Code GTFS fields.
using LanguageCode = std::string;

// Helper enums for some GTFS fields ---------------------------------------------------------------
enum class StopLocationType {
  StopOrPlatform = 0,
  Station = 1,
  EntranceExit = 2,
  GenericNode = 3,
  BoardingArea = 4
};

// The type of transportation used on a route.
enum class RouteType {
  // GTFS route types
  Tram = 0,        // Tram, Streetcar, Light rail
  Subway = 1,      // Any underground rail system within a metropolitan area
  Rail = 2,        // Intercity or long-distance travel
  Bus = 3,         // Short- and long-distance bus routes
  Ferry = 4,       // Boat service
  CableTram = 5,   // Street-level rail cars where the cable runs beneath the vehicle
  AerialLift = 6,  // Aerial lift, suspended cable car (gondola lift, aerial tramway)
  Funicular = 7,   // Any rail system designed for steep inclines
  Trolleybus = 11, // Electric buses that draw power from overhead wires using poles
  Monorail = 12,   // Railway in which the track consists of a single rail or a beam

  // Extended route types
  // https://developers.google.com/transit/gtfs/reference/extended-route-types
  RailwayService = 100,
  HighSpeedRailService = 101,
  LongDistanceTrains = 102,
  InterRegionalRailService = 103,
  CarTransportRailService = 104,
  SleeperRailService = 105,
  RegionalRailService = 106,
  TouristRailwayService = 107,
  RailShuttleWithinComplex = 108,
  SuburbanRailway = 109,
  ReplacementRailService = 110,
  SpecialRailService = 111,
  LorryTransportRailService = 112,
  AllRailServices = 113,
  CrossCountryRailService = 114,
  VehicleTransportRailService = 115,
  RackAndPinionRailway = 116,
  AdditionalRailService = 117,

  CoachService = 200,
  InternationalCoachService = 201,
  NationalCoachService = 202,
  ShuttleCoachService = 203,
  RegionalCoachService = 204,
  SpecialCoachService = 205,
  SightseeingCoachService = 206,
  TouristCoachService = 207,
  CommuterCoachService = 208,
  AllCoachServices = 209,

  UrbanRailwayService400 = 400,
  MetroService = 401,
  UndergroundService = 402,
  UrbanRailwayService403 = 403,
  AllUrbanRailwayServices = 404,
  Monorail405 = 405,

  BusService = 700,
  RegionalBusService = 701,
  ExpressBusService = 702,
  StoppingBusService = 703,
  LocalBusService = 704,
  NightBusService = 705,
  PostBusService = 706,
  SpecialNeedsBus = 707,
  MobilityBusService = 708,
  MobilityBusForRegisteredDisabled = 709,
  SightseeingBus = 710,
  ShuttleBus = 711,
  SchoolBus = 712,
  SchoolAndPublicServiceBus = 713,
  RailReplacementBusService = 714,
  DemandAndResponseBusService = 715,
  AllBusServices = 716,

  TrolleybusService = 800,

  TramService = 900,
  CityTramService = 901,
  LocalTramService = 902,
  RegionalTramService = 903,
  SightseeingTramService = 904,
  ShuttleTramService = 905,
  AllTramServices = 906,

  WaterTransportService = 1000,
  AirService = 1100,
  FerryService = 1200,
  AerialLiftService = 1300,
  FunicularService = 1400,
  TaxiService = 1500,
  CommunalTaxiService = 1501,
  WaterTaxiService = 1502,
  RailTaxiService = 1503,
  BikeTaxiService = 1504,
  LicensedTaxiService = 1505,
  PrivateHireServiceVehicle = 1506,
  AllTaxiServices = 1507,
  MiscellaneousService = 1700,
  HorseDrawnCarriage = 1702
};

enum class TripDirectionId {
  DefaultDirection = 0, // e.g. outbound
  OppositeDirection = 1 // e.g. inbound
};

enum class TripAccess { NoInfo = 0, Yes = 1, No = 2 };

enum class StopTimeBoarding {
  RegularlyScheduled = 0,
  No = 1,                  // Not available
  Phone = 2,               // Must phone agency to arrange
  CoordinateWithDriver = 3 // Must coordinate with driver to arrange
};

enum class StopTimePoint { Approximate = 0, Exact = 1 };

enum class CalendarAvailability { NotAvailable = 0, Available = 1 };

enum class CalendarDateException {
  Added = 1, // Service has been added for the specified date
  Removed = 2
};

enum class FarePayment {
  OnBoard = 0,
  BeforeBoarding = 1 // Fare must be paid before boarding
};

enum class FareTransfers {
  No = 0, // No transfers permitted on this fare
  Once = 1,
  Twice = 2,
  Unlimited = 3
};

enum class FrequencyTripService {
  FrequencyBased = 0, // Frequency-based trips
  ScheduleBased = 1   // Schedule-based trips with the exact same headway throughout the day
};

enum class TransferType { Recommended = 0, Timed = 1, MinimumTime = 2, NotPossible = 3 };

enum class PathwayMode {
  Walkway = 1,
  Stairs = 2,
  MovingSidewalk = 3, // Moving sidewalk/travelator
  Escalator = 4,
  Elevator = 5,
  FareGate = 6, // Payment gate
  ExitGate = 7
};

enum class PathwayDirection { Unidirectional = 0, Bidirectional = 1 };

enum class AttributionRole {
  No = 0, // Organization doesn’t have this role
  Yes = 1 // Organization does have this role
};

// Structures representing GTFS entities -----------------------------------------------------------
// Required dataset file
struct Agency {
  // Conditionally optional:
  Id agency_id;

  // Required:
  Text agency_name;
  Text agency_url;
  Text agency_timezone;

  // Optional:
  Text agency_lang;
  Text agency_phone;
  Text agency_fare_url;
  Text agency_email;
};

inline bool operator==(const Agency& lhs, const Agency& rhs) {
  return std::tie(lhs.agency_id, lhs.agency_name, lhs.agency_url, lhs.agency_timezone,
                  lhs.agency_lang, lhs.agency_phone, lhs.agency_fare_url, lhs.agency_email) ==
         std::tie(rhs.agency_id, rhs.agency_name, rhs.agency_url, rhs.agency_timezone,
                  rhs.agency_lang, rhs.agency_phone, rhs.agency_fare_url, rhs.agency_email);
}

// Required dataset file
struct Stop {
  // Required:
  Id stop_id;

  // Conditionally required:
  Text stop_name;

  bool coordinates_present = true;
  double stop_lat = 0.0;
  double stop_lon = 0.0;
  Id zone_id;
  Id parent_station;

  // Optional:
  Text stop_code;
  Text stop_desc;
  Text stop_url;
  StopLocationType location_type = StopLocationType::GenericNode;
  Text stop_timezone;
  Text wheelchair_boarding;
  Id level_id;
  Text platform_code;
};

// Required dataset file
struct Route {
  // Required:
  Id route_id;
  RouteType route_type = RouteType::Tram;

  // Conditionally required:
  Id agency_id;
  Text route_short_name;
  Text route_long_name;

  // Optional
  Text route_desc;
  Text route_url;
  Text route_color;
  Text route_text_color;
  size_t route_sort_order = 0; // Routes with smaller value values should be displayed first
};

// Required dataset file
struct Trip {
  // Required:
  Id route_id;
  Id service_id;
  Id trip_id;

  // Optional:
  Text trip_headsign;
  Text trip_short_name;
  TripDirectionId direction_id = TripDirectionId::DefaultDirection;
  Id block_id;
  Id shape_id;
  TripAccess wheelchair_accessible = TripAccess::NoInfo;
  TripAccess bikes_allowed = TripAccess::NoInfo;
};

// Required dataset file
struct StopTime {
  // Required:
  Id trip_id;
  Id stop_id;
  size_t stop_sequence = 0;

  // Conditionally required:
  Time arrival_time;

  Time departure_time;

  // Optional:
  Text stop_headsign;
  StopTimeBoarding pickup_type = StopTimeBoarding::RegularlyScheduled;
  StopTimeBoarding drop_off_type = StopTimeBoarding::RegularlyScheduled;

  double shape_dist_traveled = 0.0;
  StopTimePoint timepoint = StopTimePoint::Exact;
};

// Conditionally required dataset file:
struct CalendarItem {
  // Required:
  Id service_id;

  CalendarAvailability monday = CalendarAvailability::NotAvailable;
  CalendarAvailability tuesday = CalendarAvailability::NotAvailable;
  CalendarAvailability wednesday = CalendarAvailability::NotAvailable;
  CalendarAvailability thursday = CalendarAvailability::NotAvailable;
  CalendarAvailability friday = CalendarAvailability::NotAvailable;
  CalendarAvailability saturday = CalendarAvailability::NotAvailable;
  CalendarAvailability sunday = CalendarAvailability::NotAvailable;

  Date start_date;
  Date end_date;
};

// Conditionally required dataset file
struct CalendarDate {
  // Required:
  Id service_id;
  Date date;
  CalendarDateException exception_type = CalendarDateException::Added;
};

// Optional dataset file
struct FareAttributesItem {
  // Required:
  Id fare_id;
  double price = 0.0;
  CurrencyCode currency_type;
  FarePayment payment_method = FarePayment::BeforeBoarding;
  FareTransfers transfers = FareTransfers::Unlimited;

  // Conditionally required:
  Id agency_id;

  // Optional:
  size_t transfer_duration = 0; // Length of time in seconds before a transfer expires
};

// Optional dataset file
struct FareRule {
  // Required:
  Id fare_id;

  // Optional:
  Id route_id;
  Id origin_id;
  Id destination_id;
  Id contains_id;
};

// Optional dataset file
struct ShapePoint {
  // Required:
  Id shape_id;
  double shape_pt_lat = 0.0;
  double shape_pt_lon = 0.0;
  size_t shape_pt_sequence = 0;

  // Optional:
  double shape_dist_traveled = 0;
};

// Optional dataset file
struct Frequency {
  // Required:
  Id trip_id;
  Time start_time;
  Time end_time;
  size_t headway_secs = 0;

  // Optional:
  FrequencyTripService exact_times = FrequencyTripService::FrequencyBased;
};

// Optional dataset file
struct Transfer {
  // Required:
  Id from_stop_id;
  Id to_stop_id;
  TransferType transfer_type = TransferType::Recommended;

  // Optional:
  size_t min_transfer_time = 0;
};

// Optional dataset file for the GTFS-Pathways extension
struct Pathway {
  // Required:
  Id pathway_id;
  Id from_stop_id;
  Id to_stop_id;
  PathwayMode pathway_mode = PathwayMode::Walkway;
  PathwayDirection is_bidirectional = PathwayDirection::Unidirectional;

  // Optional fields:
  // Horizontal length in meters of the pathway from the origin location
  double length = 0.0;
  // Average time in seconds needed to walk through the pathway from the origin location
  size_t traversal_time = 0;
  // Number of stairs of the pathway
  size_t stair_count = 0;
  // Maximum slope ratio of the pathway
  double max_slope = 0.0;
  // Minimum width of the pathway in meters
  double min_width = 0.0;
  // Text from physical signage visible to transit riders
  Text signposted_as;
  // Same as signposted_as, but when the pathways is used backward
  Text reversed_signposted_as;
};

// Optional dataset file
struct Level {
  // Required:
  Id level_id;

  // Numeric index of the level that indicates relative position of this level in relation to other
  // levels (levels with higher indices are assumed to be located above levels with lower indices).
  // Ground level should have index 0, with levels above ground indicated by positive indices and
  // levels below ground by negative indices
  double level_index = 0.0;

  // Optional:
  Text level_name;
};

// Optional dataset file
struct FeedInfo {
  // Required:
  Text feed_publisher_name;
  Text feed_publisher_url;
  LanguageCode feed_lang;

  // Optional:
  Date feed_start_date;
  Date feed_end_date;
  Text feed_version;
  Text feed_contact_email;
  Text feed_contact_url;
};

// Optional dataset file
struct Translation {
  // Required:
  Text table_name;
  Text field_name;
  LanguageCode language;
  Text translation;

  // Conditionally required:
  Id record_id;
  Id record_sub_id;
  Text field_value;
};

// Optional dataset file
struct Attribution {
  // Required:
  Text organization_name;

  // Optional:
  Id attribution_id; // Useful for translations
  Id agency_id;
  Id route_id;
  Id trip_id;

  AttributionRole is_producer = AttributionRole::No;
  AttributionRole is_operator = AttributionRole::No;
  AttributionRole is_authority = AttributionRole::No;

  Text attribution_url;
  Text attribution_email;
  Text attribution_phone;
};

// Main classes for working with GTFS feeds
using Agencies = std::vector<Agency>;
using Stops = std::vector<Stop>;
using Routes = std::vector<Route>;
using Trips = std::vector<Trip>;
using StopTimes = std::vector<StopTime>;
using Calendar = std::vector<CalendarItem>;
using CalendarDates = std::vector<CalendarDate>;

using FareRules = std::vector<FareRule>;
using FareAttributes = std::vector<FareAttributesItem>;
using Shapes = std::vector<ShapePoint>;
using Shape = std::vector<ShapePoint>;
using Frequencies = std::vector<Frequency>;
using Transfers = std::vector<Transfer>;
using Pathways = std::vector<Pathway>;
using Levels = std::vector<Level>;
// FeedInfo is a unique object and doesn't need a container.
using Translations = std::vector<Translation>;
using Attributions = std::vector<Attribution>;

using ParsedCsvRow = std::map<std::string, std::string>;

class Feed {
public:
  inline Feed() = default;
  inline explicit Feed(const std::string& gtfs_path);

  inline Result read_feed();
  inline Result write_feed(const std::string& gtfs_path) const;

  inline Result read_agencies();
  inline Result write_agencies(const std::string& gtfs_path) const;

  inline const Agencies& get_agencies() const;
  inline boost::optional<Agency> get_agency(const Id& agency_id) const;
  inline void add_agency(const Agency& agency);

  inline Result read_stops();
  inline Result write_stops(const std::string& gtfs_path) const;

  inline const Stops& get_stops() const;
  inline boost::optional<Stop> get_stop(const Id& stop_id) const;
  inline void add_stop(const Stop& stop);

  inline Result read_routes();
  inline Result write_routes(const std::string& gtfs_path) const;

  inline const Routes& get_routes() const;
  inline boost::optional<Route> get_route(const Id& route_id) const;
  inline void add_route(const Route& route);

  inline Result read_trips();
  inline Result write_trips(const std::string& gtfs_path) const;

  inline const Trips& get_trips() const;
  inline boost::optional<Trip> get_trip(const Id& trip_id) const;
  inline void add_trip(const Trip& trip);

  inline Result read_stop_times();
  inline Result write_stop_times(const std::string& gtfs_path) const;

  inline const StopTimes& get_stop_times() const;
  inline StopTimes get_stop_times_for_stop(const Id& stop_id) const;
  inline StopTimes get_stop_times_for_trip(const Id& trip_id, bool sort_by_sequence = true) const;
  inline void add_stop_time(const StopTime& stop_time);

  inline Result read_calendar();
  inline Result write_calendar(const std::string& gtfs_path) const;

  inline const Calendar& get_calendar() const;
  inline boost::optional<CalendarItem> get_calendar(const Id& service_id) const;
  inline void add_calendar_item(const CalendarItem& calendar_item);

  inline Result read_calendar_dates();
  inline Result write_calendar_dates(const std::string& gtfs_path) const;

  inline const CalendarDates& get_calendar_dates() const;
  inline CalendarDates get_calendar_dates(const Id& service_id, bool sort_by_date = true) const;
  inline void add_calendar_date(const CalendarDate& calendar_date);

  inline Result read_fare_rules();
  inline Result write_fare_rules(const std::string& gtfs_path) const;

  inline const FareRules& get_fare_rules() const;
  inline FareRules get_fare_rules(const Id& fare_id) const;
  inline void add_fare_rule(const FareRule& fare_rule);

  inline Result read_fare_attributes();
  inline Result write_fare_attributes(const std::string& gtfs_path) const;

  inline const FareAttributes& get_fare_attributes() const;
  inline FareAttributes get_fare_attributes(const Id& fare_id) const;
  inline void add_fare_attributes(const FareAttributesItem& fare_attributes_item);

  inline Result read_shapes();
  inline Result write_shapes(const std::string& gtfs_path) const;

  inline const Shapes& get_shapes() const;
  inline Shape get_shape(const Id& shape_id, bool sort_by_sequence = true) const;
  inline void add_shape(const ShapePoint& shape);

  inline Result read_frequencies();
  inline Result write_frequencies(const std::string& gtfs_path) const;

  inline const Frequencies& get_frequencies() const;
  inline Frequencies get_frequencies(const Id& trip_id) const;
  inline void add_frequency(const Frequency& frequency);

  inline Result read_transfers();
  inline Result write_transfers(const std::string& gtfs_path) const;

  inline const Transfers& get_transfers() const;
  inline boost::optional<Transfer> get_transfer(const Id& from_stop_id, const Id& to_stop_id) const;
  inline void add_transfer(const Transfer& transfer);

  inline Result read_pathways();
  inline Result write_pathways(const std::string& gtfs_path) const;

  inline const Pathways& get_pathways() const;
  inline Pathways get_pathways(const Id& pathway_id) const;
  inline Pathways get_pathways(const Id& from_stop_id, const Id& to_stop_id) const;
  inline void add_pathway(const Pathway& pathway);

  inline Result read_levels();
  inline Result write_levels(const std::string& gtfs_path) const;

  inline const Levels& get_levels() const;
  inline boost::optional<Level> get_level(const Id& level_id) const;
  inline void add_level(const Level& level);

  inline Result read_feed_info();
  inline Result write_feed_info(const std::string& gtfs_path) const;

  inline FeedInfo get_feed_info() const;
  inline void set_feed_info(const FeedInfo& feed_info);

  inline Result read_translations();
  inline Result write_translations(const std::string& gtfs_path) const;

  inline const Translations& get_translations() const;
  inline Translations get_translations(const Text& table_name) const;
  inline void add_translation(const Translation& translation);

  inline Result read_attributions();
  inline Result write_attributions(const std::string& gtfs_path) const;

  inline const Attributions& get_attributions() const;
  inline void add_attribution(const Attribution& attribution);

private:
  inline Result parse_csv(const std::string& filename,
                          const std::function<Result(const ParsedCsvRow& record)>& add_entity);

  inline Result write_csv(const std::string& path,
                          const std::string& file,
                          const std::function<void(std::ofstream& out)>& write_header,
                          const std::function<void(std::ofstream& out)>& write_entities) const;

  inline Result add_agency(const ParsedCsvRow& row);
  inline Result add_route(const ParsedCsvRow& row);
  inline Result add_shape(const ParsedCsvRow& row);
  inline Result add_trip(const ParsedCsvRow& row);
  inline Result add_stop(const ParsedCsvRow& row);
  inline Result add_stop_time(const ParsedCsvRow& row);
  inline Result add_calendar_item(const ParsedCsvRow& row);
  inline Result add_calendar_date(const ParsedCsvRow& row);
  inline Result add_transfer(const ParsedCsvRow& row);
  inline Result add_frequency(const ParsedCsvRow& row);
  inline Result add_fare_attributes(const ParsedCsvRow& row);
  inline Result add_fare_rule(const ParsedCsvRow& row);
  inline Result add_pathway(const ParsedCsvRow& row);
  inline Result add_level(const ParsedCsvRow& row);
  inline Result add_feed_info(const ParsedCsvRow& row);
  inline Result add_translation(const ParsedCsvRow& row);
  inline Result add_attribution(const ParsedCsvRow& row);

  inline void write_agencies(std::ofstream& out) const;
  inline void write_routes(std::ofstream& out) const;
  inline void write_shapes(std::ofstream& out) const;
  inline void write_trips(std::ofstream& out) const;
  inline void write_stops(std::ofstream& out) const;
  inline void write_stop_times(std::ofstream& out) const;
  inline void write_calendar(std::ofstream& out) const;
  inline void write_calendar_dates(std::ofstream& out) const;
  inline void write_transfers(std::ofstream& out) const;
  inline void write_frequencies(std::ofstream& out) const;
  inline void write_fare_attributes(std::ofstream& out) const;
  inline void write_fare_rules(std::ofstream& out) const;
  inline void write_pathways(std::ofstream& out) const;
  inline void write_levels(std::ofstream& out) const;
  inline void write_feed_info(std::ofstream& out) const;
  inline void write_translations(std::ofstream& out) const;
  inline void write_attributions(std::ofstream& out) const;

  std::string gtfs_directory;

  Agencies agencies;
  Stops stops;
  Routes routes;
  Trips trips;
  StopTimes stop_times;

  Calendar calendar;
  CalendarDates calendar_dates;
  FareRules fare_rules;
  FareAttributes fare_attributes;
  Shape shapes;
  Frequencies frequencies;
  Transfers transfers;
  Pathways pathways;
  Levels levels;
  Translations translations;
  Attributions attributions;
  FeedInfo feed_info;
};

inline Feed::Feed(const std::string& gtfs_path) : gtfs_directory(add_trailing_slash(gtfs_path)) {
}

inline bool ErrorParsingOptionalFile(const Result& res) {
  return res != ResultCode::OK && res != ResultCode::ERROR_FILE_ABSENT;
}

inline Result Feed::read_feed() {
  // Read required files:
  auto res = read_agencies();
  if (res != ResultCode::OK)
    return res;

  res = read_stops();
  if (res != ResultCode::OK)
    return res;

  res = read_routes();
  if (res != ResultCode::OK)
    return res;

  res = read_trips();
  if (res != ResultCode::OK)
    return res;

  res = read_stop_times();
  if (res != ResultCode::OK)
    return res;

  // Read conditionally required files:
  res = read_calendar();
  if (ErrorParsingOptionalFile(res))
    return res;

  res = read_calendar_dates();
  if (ErrorParsingOptionalFile(res))
    return res;

  // Read optional files:
  res = read_shapes();
  if (ErrorParsingOptionalFile(res))
    return res;

  res = read_transfers();
  if (ErrorParsingOptionalFile(res))
    return res;

  res = read_frequencies();
  if (ErrorParsingOptionalFile(res))
    return res;

  res = read_fare_attributes();
  if (ErrorParsingOptionalFile(res))
    return res;

  res = read_fare_rules();
  if (ErrorParsingOptionalFile(res))
    return res;

  res = read_pathways();
  if (ErrorParsingOptionalFile(res))
    return res;

  res = read_levels();
  if (ErrorParsingOptionalFile(res))
    return res;

  res = read_attributions();
  if (ErrorParsingOptionalFile(res))
    return res;

  res = read_feed_info();
  if (ErrorParsingOptionalFile(res))
    return res;

  res = read_translations();
  if (ErrorParsingOptionalFile(res))
    return res;

  return ResultCode::OK;
}

inline Result Feed::write_feed(const std::string& gtfs_path) const {
  if (gtfs_path.empty())
    return {ResultCode::ERROR_INVALID_GTFS_PATH, "Empty output path for writing feed"};
  // TODO Write feed to csv files
  return {};
}

inline std::string get_value_or_default(const ParsedCsvRow& container,
                                        const std::string& key,
                                        const std::string& default_value = "") {
  const auto it = container.find(key);
  if (it == container.end())
    return default_value;

  return it->second;
}

template <class T>
inline void
set_field(T& field, const ParsedCsvRow& container, const std::string& key, bool is_optional = true) {
  const std::string key_str = get_value_or_default(container, key);
  if (!key_str.empty() || !is_optional)
    field = static_cast<T>(std::stoi(key_str));
}

inline bool set_fractional(double& field,
                           const ParsedCsvRow& container,
                           const std::string& key,
                           bool is_optional = true) {
  const std::string key_str = get_value_or_default(container, key);
  if (!key_str.empty() || !is_optional) {
    field = std::stod(key_str);
    return true;
  }
  return false;
}

// Throw if not valid WGS84 decimal degrees.
inline void check_coordinates(double latitude, double longitude) {
  if (latitude < -90.0 || latitude > 90.0)
    throw std::out_of_range("Latitude");

  if (longitude < -180.0 || longitude > 180.0)
    throw std::out_of_range("Longitude");
}

inline Result Feed::add_agency(const ParsedCsvRow& row) {
  Agency agency;

  // Conditionally required id:
  agency.agency_id = get_value_or_default(row, "agency_id");

  // Required fields:
  try {
    agency.agency_name = row.at("agency_name");
    agency.agency_url = row.at("agency_url");
    agency.agency_timezone = row.at("agency_timezone");
  } catch (const std::out_of_range& ex) {
    return {ResultCode::ERROR_REQUIRED_FIELD_ABSENT, ex.what()};
  }

  // Optional fields:
  agency.agency_lang = get_value_or_default(row, "agency_lang");
  agency.agency_phone = get_value_or_default(row, "agency_phone");
  agency.agency_fare_url = get_value_or_default(row, "agency_fare_url");
  agency.agency_email = get_value_or_default(row, "agency_email");

  agencies.emplace_back(agency);
  return ResultCode::OK;
}

inline Result Feed::add_route(const ParsedCsvRow& row) {
  Route route;

  try {
    // Required fields:
    route.route_id = row.at("route_id");
    set_field(route.route_type, row, "route_type", false);

    // Optional:
    set_field(route.route_sort_order, row, "route_sort_order");
  } catch (const std::out_of_range& ex) {
    return {ResultCode::ERROR_REQUIRED_FIELD_ABSENT, ex.what()};
  } catch (const std::invalid_argument& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  }

  // Conditionally required:
  route.agency_id = get_value_or_default(row, "agency_id");

  route.route_short_name = get_value_or_default(row, "route_short_name");
  route.route_long_name = get_value_or_default(row, "route_long_name");

  if (route.route_short_name.empty() && route.route_long_name.empty()) {
    return {ResultCode::ERROR_REQUIRED_FIELD_ABSENT,
            "'route_short_name' or 'route_long_name' must be specified"};
  }

  route.route_color = get_value_or_default(row, "route_color");
  route.route_text_color = get_value_or_default(row, "route_text_color");
  route.route_desc = get_value_or_default(row, "route_desc");
  route.route_url = get_value_or_default(row, "route_url");

  routes.emplace_back(route);

  return ResultCode::OK;
}

inline Result Feed::add_shape(const ParsedCsvRow& row) {
  ShapePoint point;
  try {
    // Required:
    point.shape_id = row.at("shape_id");
    point.shape_pt_sequence = std::stoi(row.at("shape_pt_sequence"));

    point.shape_pt_lon = std::stod(row.at("shape_pt_lon"));
    point.shape_pt_lat = std::stod(row.at("shape_pt_lat"));
    check_coordinates(point.shape_pt_lat, point.shape_pt_lon);

    // Optional:
    set_fractional(point.shape_dist_traveled, row, "shape_dist_traveled");
    if (point.shape_dist_traveled < 0.0)
      throw std::invalid_argument("Invalid shape_dist_traveled");
  } catch (const std::out_of_range& ex) {
    return {ResultCode::ERROR_REQUIRED_FIELD_ABSENT, ex.what()};
  } catch (const std::invalid_argument& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  }

  shapes.emplace_back(point);
  return ResultCode::OK;
}

inline Result Feed::add_trip(const ParsedCsvRow& row) {
  Trip trip;
  try {
    // Required:
    trip.route_id = row.at("route_id");
    trip.service_id = row.at("service_id");
    trip.trip_id = row.at("trip_id");

    // Optional:
    set_field(trip.direction_id, row, "direction_id");
    set_field(trip.wheelchair_accessible, row, "wheelchair_accessible");
    set_field(trip.bikes_allowed, row, "bikes_allowed");
  } catch (const std::out_of_range& ex) {
    return {ResultCode::ERROR_REQUIRED_FIELD_ABSENT, ex.what()};
  } catch (const std::invalid_argument& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  }

  // Optional:
  trip.shape_id = get_value_or_default(row, "shape_id");
  trip.trip_headsign = get_value_or_default(row, "trip_headsign");
  trip.trip_short_name = get_value_or_default(row, "trip_short_name");
  trip.block_id = get_value_or_default(row, "block_id");

  trips.emplace_back(trip);
  return ResultCode::OK;
}

inline Result Feed::add_stop(const ParsedCsvRow& row) {
  Stop stop;

  try {
    stop.stop_id = row.at("stop_id");

    // Optional:
    bool const set_lon = set_fractional(stop.stop_lon, row, "stop_lon");
    bool const set_lat = set_fractional(stop.stop_lat, row, "stop_lat");

    if (!set_lon || !set_lat)
      stop.coordinates_present = false;
  } catch (const std::out_of_range& ex) {
    return {ResultCode::ERROR_REQUIRED_FIELD_ABSENT, ex.what()};
  } catch (const std::invalid_argument& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  }

  // Conditionally required:
  stop.stop_name = get_value_or_default(row, "stop_name");
  stop.parent_station = get_value_or_default(row, "parent_station");
  stop.zone_id = get_value_or_default(row, "zone_id");

  // Optional:
  stop.stop_code = get_value_or_default(row, "stop_code");
  stop.stop_desc = get_value_or_default(row, "stop_desc");
  stop.stop_url = get_value_or_default(row, "stop_url");
  set_field(stop.location_type, row, "location_type");
  stop.stop_timezone = get_value_or_default(row, "stop_timezone");
  stop.wheelchair_boarding = get_value_or_default(row, "wheelchair_boarding");
  stop.level_id = get_value_or_default(row, "level_id");
  stop.platform_code = get_value_or_default(row, "platform_code");

  stops.emplace_back(stop);

  return ResultCode::OK;
}

inline Result Feed::add_stop_time(const ParsedCsvRow& row) {
  StopTime stop_time;

  try {
    // Required:
    stop_time.trip_id = row.at("trip_id");
    stop_time.stop_id = row.at("stop_id");
    stop_time.stop_sequence = std::stoi(row.at("stop_sequence"));

    // Conditionally required:
    stop_time.departure_time = Time(row.at("departure_time"));
    stop_time.arrival_time = Time(row.at("arrival_time"));

    // Optional:
    set_field(stop_time.pickup_type, row, "pickup_type");
    set_field(stop_time.drop_off_type, row, "drop_off_type");

    set_fractional(stop_time.shape_dist_traveled, row, "shape_dist_traveled");
    if (stop_time.shape_dist_traveled < 0.0)
      throw std::invalid_argument("Invalid shape_dist_traveled");

    set_field(stop_time.timepoint, row, "timepoint");
  } catch (const std::out_of_range& ex) {
    return {ResultCode::ERROR_REQUIRED_FIELD_ABSENT, ex.what()};
  } catch (const std::invalid_argument& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  } catch (const InvalidFieldFormat& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  }

  // Optional fields:
  stop_time.stop_headsign = get_value_or_default(row, "stop_headsign");

  stop_times.emplace_back(stop_time);
  return ResultCode::OK;
}

inline Result Feed::add_calendar_item(const ParsedCsvRow& row) {
  CalendarItem calendar_item;
  try {
    // Required fields:
    calendar_item.service_id = row.at("service_id");

    set_field(calendar_item.monday, row, "monday", false);
    set_field(calendar_item.tuesday, row, "tuesday", false);
    set_field(calendar_item.wednesday, row, "wednesday", false);
    set_field(calendar_item.thursday, row, "thursday", false);
    set_field(calendar_item.friday, row, "friday", false);
    set_field(calendar_item.saturday, row, "saturday", false);
    set_field(calendar_item.sunday, row, "sunday", false);

    calendar_item.start_date = Date(row.at("start_date"));
    calendar_item.end_date = Date(row.at("end_date"));
  } catch (const std::out_of_range& ex) {
    return {ResultCode::ERROR_REQUIRED_FIELD_ABSENT, ex.what()};
  } catch (const std::invalid_argument& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  } catch (const InvalidFieldFormat& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  }

  calendar.emplace_back(calendar_item);
  return ResultCode::OK;
}

inline Result Feed::add_calendar_date(const ParsedCsvRow& row) {
  CalendarDate calendar_date;
  try {
    // Required fields:
    calendar_date.service_id = row.at("service_id");

    set_field(calendar_date.exception_type, row, "exception_type", false);
    calendar_date.date = Date(row.at("date"));
  } catch (const std::out_of_range& ex) {
    return {ResultCode::ERROR_REQUIRED_FIELD_ABSENT, ex.what()};
  } catch (const std::invalid_argument& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  } catch (const InvalidFieldFormat& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  }

  calendar_dates.emplace_back(calendar_date);
  return ResultCode::OK;
}

inline Result Feed::add_transfer(const ParsedCsvRow& row) {
  Transfer transfer;
  try {
    // Required fields:
    transfer.from_stop_id = row.at("from_stop_id");
    transfer.to_stop_id = row.at("to_stop_id");
    set_field(transfer.transfer_type, row, "transfer_type", false);

    // Optional:
    set_field(transfer.min_transfer_time, row, "min_transfer_time");
  } catch (const std::out_of_range& ex) {
    return {ResultCode::ERROR_REQUIRED_FIELD_ABSENT, ex.what()};
  } catch (const std::invalid_argument& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  } catch (const InvalidFieldFormat& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  }

  transfers.emplace_back(transfer);
  return ResultCode::OK;
}

inline Result Feed::add_frequency(const ParsedCsvRow& row) {
  Frequency frequency;
  try {
    // Required fields:
    frequency.trip_id = row.at("trip_id");
    frequency.start_time = Time(row.at("start_time"));
    frequency.end_time = Time(row.at("end_time"));
    set_field(frequency.headway_secs, row, "headway_secs", false);

    // Optional:
    set_field(frequency.exact_times, row, "exact_times");
  } catch (const std::out_of_range& ex) {
    return {ResultCode::ERROR_REQUIRED_FIELD_ABSENT, ex.what()};
  } catch (const std::invalid_argument& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  } catch (const InvalidFieldFormat& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  }

  frequencies.emplace_back(frequency);
  return ResultCode::OK;
}

inline Result Feed::add_fare_attributes(const ParsedCsvRow& row) {
  FareAttributesItem item;
  try {
    // Required fields:
    item.fare_id = row.at("fare_id");
    set_fractional(item.price, row, "price", false);

    item.currency_type = row.at("currency_type");
    set_field(item.payment_method, row, "payment_method", false);
    set_field(item.transfers, row, "transfers", false);

    // Conditionally optional:
    item.agency_id = get_value_or_default(row, "agency_id");
    set_field(item.transfer_duration, row, "transfer_duration");
  } catch (const std::out_of_range& ex) {
    return {ResultCode::ERROR_REQUIRED_FIELD_ABSENT, ex.what()};
  } catch (const std::invalid_argument& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  } catch (const InvalidFieldFormat& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  }

  fare_attributes.emplace_back(item);
  return ResultCode::OK;
}

inline Result Feed::add_fare_rule(const ParsedCsvRow& row) {
  FareRule fare_rule;
  try {
    // Required fields:
    fare_rule.fare_id = row.at("fare_id");
  } catch (const std::out_of_range& ex) {
    return {ResultCode::ERROR_REQUIRED_FIELD_ABSENT, ex.what()};
  } catch (const std::invalid_argument& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  } catch (const InvalidFieldFormat& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  }

  // Optional fields:
  fare_rule.route_id = get_value_or_default(row, "route_id");
  fare_rule.origin_id = get_value_or_default(row, "origin_id");
  fare_rule.destination_id = get_value_or_default(row, "destination_id");
  fare_rule.contains_id = get_value_or_default(row, "contains_id");

  fare_rules.emplace_back(fare_rule);

  return ResultCode::OK;
}

inline Result Feed::add_pathway(const ParsedCsvRow& row) {
  Pathway path;
  try {
    // Required fields:
    path.pathway_id = row.at("pathway_id");
    path.from_stop_id = row.at("from_stop_id");
    path.to_stop_id = row.at("to_stop_id");
    set_field(path.pathway_mode, row, "pathway_mode", false);
    set_field(path.is_bidirectional, row, "is_bidirectional", false);

    // Optional fields:
    set_fractional(path.length, row, "length");
    set_field(path.traversal_time, row, "traversal_time");
    set_field(path.stair_count, row, "stair_count");
    set_fractional(path.max_slope, row, "max_slope");
    set_fractional(path.min_width, row, "min_width");
  } catch (const std::out_of_range& ex) {
    return {ResultCode::ERROR_REQUIRED_FIELD_ABSENT, ex.what()};
  } catch (const std::invalid_argument& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  } catch (const InvalidFieldFormat& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  }

  path.signposted_as = get_value_or_default(row, "signposted_as");
  path.reversed_signposted_as = get_value_or_default(row, "reversed_signposted_as");

  pathways.emplace_back(path);
  return ResultCode::OK;
}

inline Result Feed::add_level(const ParsedCsvRow& row) {
  Level level;
  try {
    // Required fields:
    level.level_id = row.at("level_id");

    set_fractional(level.level_index, row, "level_index", false);
  } catch (const std::out_of_range& ex) {
    return {ResultCode::ERROR_REQUIRED_FIELD_ABSENT, ex.what()};
  } catch (const std::invalid_argument& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  } catch (const InvalidFieldFormat& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  }

  // Optional field:
  level.level_name = get_value_or_default(row, "level_name");

  levels.emplace_back(level);

  return ResultCode::OK;
}

inline Result Feed::add_feed_info(const ParsedCsvRow& row) {
  try {
    // Required fields:
    feed_info.feed_publisher_name = row.at("feed_publisher_name");
    feed_info.feed_publisher_url = row.at("feed_publisher_url");
    feed_info.feed_lang = row.at("feed_lang");

    // Optional fields:
    feed_info.feed_start_date = Date(get_value_or_default(row, "feed_start_date"));
    feed_info.feed_end_date = Date(get_value_or_default(row, "feed_end_date"));
  } catch (const std::out_of_range& ex) {
    return {ResultCode::ERROR_REQUIRED_FIELD_ABSENT, ex.what()};
  } catch (const std::invalid_argument& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  } catch (const InvalidFieldFormat& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  }

  // Optional fields:
  feed_info.feed_version = get_value_or_default(row, "feed_version");
  feed_info.feed_contact_email = get_value_or_default(row, "feed_contact_email");
  feed_info.feed_contact_url = get_value_or_default(row, "feed_contact_url");

  return ResultCode::OK;
}

inline Result Feed::add_translation(const ParsedCsvRow& row) {
  static const std::vector<Text> available_tables{"agency",     "stops",    "routes", "trips",
                                                  "stop_times", "pathways", "levels"};
  Translation translation;

  try {
    // Required fields:
    translation.table_name = row.at("table_name");
    if (std::find(available_tables.begin(), available_tables.end(), translation.table_name) ==
        available_tables.end()) {
      throw InvalidFieldFormat("Field table_name of translations doesn't have required value");
    }

    translation.field_name = row.at("field_name");
    translation.language = row.at("language");
    translation.translation = row.at("translation");

    // Conditionally required:
    translation.record_id = get_value_or_default(row, "record_id");
    translation.record_sub_id = get_value_or_default(row, "record_sub_id");
  } catch (const std::out_of_range& ex) {
    return {ResultCode::ERROR_REQUIRED_FIELD_ABSENT, ex.what()};
  } catch (const std::invalid_argument& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  } catch (const InvalidFieldFormat& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  }

  // Conditionally required:
  translation.field_value = get_value_or_default(row, "field_value");

  translations.emplace_back(translation);

  return ResultCode::OK;
}

inline Result Feed::add_attribution(const ParsedCsvRow& row) {
  Attribution attribution;

  try {
    // Required fields:
    attribution.organization_name = row.at("organization_name");

    // Optional fields:
    attribution.attribution_id = get_value_or_default(row, "attribution_id");
    attribution.agency_id = get_value_or_default(row, "agency_id");
    attribution.route_id = get_value_or_default(row, "route_id");
    attribution.trip_id = get_value_or_default(row, "trip_id");

    set_field(attribution.is_producer, row, "is_producer");
    set_field(attribution.is_operator, row, "is_operator");
    set_field(attribution.is_authority, row, "is_authority");

    attribution.attribution_url = get_value_or_default(row, "attribution_url");
    attribution.attribution_email = get_value_or_default(row, "attribution_email");
    attribution.trip_id = get_value_or_default(row, "attribution_phone");
  } catch (const std::out_of_range& ex) {
    return {ResultCode::ERROR_REQUIRED_FIELD_ABSENT, ex.what()};
  } catch (const std::invalid_argument& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  } catch (const InvalidFieldFormat& ex) {
    return {ResultCode::ERROR_INVALID_FIELD_FORMAT, ex.what()};
  }

  attributions.emplace_back(attribution);

  return ResultCode::OK;
}

inline Result Feed::write_csv(const std::string& path,
                              const std::string& file,
                              const std::function<void(std::ofstream& out)>& write_header,
                              const std::function<void(std::ofstream& out)>& write_entities) const {
  const std::string filepath = add_trailing_slash(path) + file;
  std::ofstream out(filepath);
  if (!out.is_open())
    return {ResultCode::ERROR_INVALID_GTFS_PATH, "Could not open path for writing " + filepath};

  write_header(out);
  write_entities(out);
  return ResultCode::OK;
}

inline Result Feed::parse_csv(const std::string& filename,
                              const std::function<Result(const ParsedCsvRow& record)>& add_entity) {
  CsvParser parser(gtfs_directory);
  auto res_header = parser.read_header(filename);
  if (res_header.code != ResultCode::OK)
    return res_header;

  ParsedCsvRow record;
  Result res_row;
  while ((res_row = parser.read_row(record)) != ResultCode::END_OF_FILE) {
    if (res_row != ResultCode::OK)
      return res_row;

    if (record.empty())
      continue;

    Result res = add_entity(record);
    if (res != ResultCode::OK) {
      res.message += " while adding item from " + filename;
      return res;
    }
  }

  return {ResultCode::OK, {"Parsed " + filename}};
}

inline Result Feed::read_agencies() {
  auto handler = [this](const ParsedCsvRow& record) { return this->add_agency(record); };
  return parse_csv(file_agency, handler);
}

inline Result Feed::write_agencies(const std::string& gtfs_path) const {
  auto container_writer = [this](std::ofstream& out) { return this->write_agencies(out); };
  return write_csv(gtfs_path, file_agency, write_agency_header, container_writer);
}

inline const Agencies& Feed::get_agencies() const {
  return agencies;
}

inline boost::optional<Agency> Feed::get_agency(const Id& agency_id) const {
  // agency id is required when the dataset contains data for multiple agencies,
  // otherwise it is optional:
  if (agency_id.empty() && agencies.size() == 1)
    return agencies[0];

  const auto it = std::find_if(agencies.begin(), agencies.end(), [&agency_id](const Agency& agency) {
    return agency.agency_id == agency_id;
  });

  if (it == agencies.end())
    return boost::none;

  return *it;
}

inline void Feed::add_agency(const Agency& agency) {
  agencies.emplace_back(agency);
}

inline Result Feed::read_stops() {
  auto handler = [this](const ParsedCsvRow& record) { return this->add_stop(record); };
  return parse_csv(file_stops, handler);
}

inline Result Feed::write_stops(const std::string& gtfs_path) const {
  auto container_writer = [this](std::ofstream& out) { return this->write_stops(out); };
  return write_csv(gtfs_path, file_stops, write_stops_header, container_writer);
}

inline const Stops& Feed::get_stops() const {
  return stops;
}

inline boost::optional<Stop> Feed::get_stop(const Id& stop_id) const {
  const auto it = std::find_if(stops.begin(), stops.end(),
                               [&stop_id](const Stop& stop) { return stop.stop_id == stop_id; });

  if (it == stops.end())
    return boost::none;

  return *it;
}

inline void Feed::add_stop(const Stop& stop) {
  stops.emplace_back(stop);
}

inline Result Feed::read_routes() {
  auto handler = [this](const ParsedCsvRow& record) { return this->add_route(record); };
  return parse_csv(file_routes, handler);
}

inline Result Feed::write_routes(const std::string& gtfs_path) const {
  auto container_writer = [this](std::ofstream& out) { return this->write_routes(out); };
  return write_csv(gtfs_path, file_routes, write_routes_header, container_writer);
}

inline const Routes& Feed::get_routes() const {
  return routes;
}

inline boost::optional<Route> Feed::get_route(const Id& route_id) const {
  const auto it = std::find_if(routes.begin(), routes.end(), [&route_id](const Route& route) {
    return route.route_id == route_id;
  });

  if (it == routes.end())
    return boost::none;

  return *it;
}

inline void Feed::add_route(const Route& route) {
  routes.emplace_back(route);
}

inline Result Feed::read_trips() {
  auto handler = [this](const ParsedCsvRow& record) { return this->add_trip(record); };
  return parse_csv(file_trips, handler);
}

inline Result Feed::write_trips(const std::string& gtfs_path) const {
  auto container_writer = [this](std::ofstream& out) { return this->write_trips(out); };
  return write_csv(gtfs_path, file_trips, write_trips_header, container_writer);
}

inline const Trips& Feed::get_trips() const {
  return trips;
}

inline boost::optional<Trip> Feed::get_trip(const Id& trip_id) const {
  const auto it = std::find_if(trips.begin(), trips.end(),
                               [&trip_id](const Trip& trip) { return trip.trip_id == trip_id; });

  if (it == trips.end())
    return boost::none;

  return *it;
}

inline void Feed::add_trip(const Trip& trip) {
  trips.emplace_back(trip);
}

inline Result Feed::read_stop_times() {
  auto handler = [this](const ParsedCsvRow& record) { return this->add_stop_time(record); };
  return parse_csv(file_stop_times, handler);
}

inline Result Feed::write_stop_times(const std::string& gtfs_path) const {
  auto container_writer = [this](std::ofstream& out) { return this->write_stop_times(out); };
  return write_csv(gtfs_path, file_stop_times, write_stop_times_header, container_writer);
}

inline const StopTimes& Feed::get_stop_times() const {
  return stop_times;
}

inline StopTimes Feed::get_stop_times_for_stop(const Id& stop_id) const {
  StopTimes res;
  for (const auto& stop_time : stop_times) {
    if (stop_time.stop_id == stop_id)
      res.emplace_back(stop_time);
  }
  return res;
}

inline StopTimes Feed::get_stop_times_for_trip(const Id& trip_id, bool sort_by_sequence) const {
  StopTimes res;
  for (const auto& stop_time : stop_times) {
    if (stop_time.trip_id == trip_id)
      res.emplace_back(stop_time);
  }
  if (sort_by_sequence) {
    std::sort(res.begin(), res.end(), [](const StopTime& t1, const StopTime& t2) {
      return t1.stop_sequence < t2.stop_sequence;
    });
  }
  return res;
}

inline void Feed::add_stop_time(const StopTime& stop_time) {
  stop_times.emplace_back(stop_time);
}

inline Result Feed::read_calendar() {
  auto handler = [this](const ParsedCsvRow& record) { return this->add_calendar_item(record); };
  return parse_csv(file_calendar, handler);
}

inline Result Feed::write_calendar(const std::string& gtfs_path) const {
  auto container_writer = [this](std::ofstream& out) { return this->write_calendar(out); };
  return write_csv(gtfs_path, file_calendar, write_calendar_header, container_writer);
}

inline const Calendar& Feed::get_calendar() const {
  return calendar;
}

inline boost::optional<CalendarItem> Feed::get_calendar(const Id& service_id) const {
  const auto it = std::find_if(calendar.begin(), calendar.end(),
                               [&service_id](const CalendarItem& calendar_item) {
                                 return calendar_item.service_id == service_id;
                               });

  if (it == calendar.end())
    return boost::none;

  return *it;
}

inline void Feed::add_calendar_item(const CalendarItem& calendar_item) {
  calendar.emplace_back(calendar_item);
}

inline Result Feed::read_calendar_dates() {
  auto handler = [this](const ParsedCsvRow& record) { return this->add_calendar_date(record); };
  return parse_csv(file_calendar_dates, handler);
}

inline Result Feed::write_calendar_dates(const std::string& gtfs_path) const {
  auto container_writer = [this](std::ofstream& out) { return this->write_calendar_dates(out); };
  return write_csv(gtfs_path, file_calendar_dates, write_calendar_dates_header, container_writer);
}

inline const CalendarDates& Feed::get_calendar_dates() const {
  return calendar_dates;
}

inline CalendarDates Feed::get_calendar_dates(const Id& service_id, bool sort_by_date) const {
  CalendarDates res;
  for (const auto& calendar_date : calendar_dates) {
    if (calendar_date.service_id == service_id)
      res.emplace_back(calendar_date);
  }

  if (sort_by_date) {
    std::sort(res.begin(), res.end(), [](const CalendarDate& d1, const CalendarDate& d2) {
      return d1.date.get_raw_date() < d2.date.get_raw_date();
    });
  }

  return res;
}

inline void Feed::add_calendar_date(const CalendarDate& calendar_date) {
  calendar_dates.emplace_back(calendar_date);
}

inline Result Feed::read_fare_rules() {
  auto handler = [this](const ParsedCsvRow& record) { return this->add_fare_rule(record); };
  return parse_csv(file_fare_rules, handler);
}

inline Result Feed::write_fare_rules(const std::string& gtfs_path) const {
  auto container_writer = [this](std::ofstream& out) { return this->write_fare_rules(out); };
  return write_csv(gtfs_path, file_fare_rules, write_fare_rules_header, container_writer);
}

inline const FareRules& Feed::get_fare_rules() const {
  return fare_rules;
}

inline FareRules Feed::get_fare_rules(const Id& fare_id) const {
  FareRules res;
  for (const auto& fare_rule : fare_rules) {
    if (fare_rule.fare_id == fare_id)
      res.emplace_back(fare_rule);
  }

  return res;
}

inline void Feed::add_fare_rule(const FareRule& fare_rule) {
  fare_rules.emplace_back(fare_rule);
}

inline Result Feed::read_fare_attributes() {
  auto handler = [this](const ParsedCsvRow& record) { return this->add_fare_attributes(record); };
  return parse_csv(file_fare_attributes, handler);
}

inline Result Feed::write_fare_attributes(const std::string& gtfs_path) const {
  auto container_writer = [this](std::ofstream& out) { return this->write_fare_attributes(out); };
  return write_csv(gtfs_path, file_fare_attributes, write_fare_attributes_header, container_writer);
}

inline const FareAttributes& Feed::get_fare_attributes() const {
  return fare_attributes;
}

FareAttributes Feed::get_fare_attributes(const Id& fare_id) const {
  FareAttributes res;
  for (const auto& attributes : fare_attributes) {
    if (attributes.fare_id == fare_id)
      res.emplace_back(attributes);
  }

  return res;
}

inline void Feed::add_fare_attributes(const FareAttributesItem& fare_attributes_item) {
  fare_attributes.emplace_back(fare_attributes_item);
}

inline Result Feed::read_shapes() {
  auto handler = [this](const ParsedCsvRow& record) { return this->add_shape(record); };
  return parse_csv(file_shapes, handler);
}

inline Result Feed::write_shapes(const std::string& gtfs_path) const {
  auto container_writer = [this](std::ofstream& out) { return this->write_shapes(out); };
  return write_csv(gtfs_path, file_shapes, write_shapes_header, container_writer);
}

inline const Shapes& Feed::get_shapes() const {
  return shapes;
}

inline Shape Feed::get_shape(const Id& shape_id, bool sort_by_sequence) const {
  Shape res;
  for (const auto& shape : shapes) {
    if (shape.shape_id == shape_id)
      res.emplace_back(shape);
  }
  if (sort_by_sequence) {
    std::sort(res.begin(), res.end(), [](const ShapePoint& s1, const ShapePoint& s2) {
      return s1.shape_pt_sequence < s2.shape_pt_sequence;
    });
  }
  return res;
}

inline void Feed::add_shape(const ShapePoint& shape) {
  shapes.emplace_back(shape);
}

inline Result Feed::read_frequencies() {
  auto handler = [this](const ParsedCsvRow& record) { return this->add_frequency(record); };
  return parse_csv(file_frequencies, handler);
}

inline Result Feed::write_frequencies(const std::string& gtfs_path) const {
  auto container_writer = [this](std::ofstream& out) { return this->write_frequencies(out); };
  return write_csv(gtfs_path, file_frequencies, write_frequencies_header, container_writer);
}

inline const Frequencies& Feed::get_frequencies() const {
  return frequencies;
}

inline Frequencies Feed::get_frequencies(const Id& trip_id) const {
  Frequencies res;
  for (const auto& frequency : frequencies) {
    if (frequency.trip_id == trip_id)
      res.emplace_back(frequency);
  }
  return res;
}

inline void Feed::add_frequency(const Frequency& frequency) {
  frequencies.emplace_back(frequency);
}

inline Result Feed::read_transfers() {
  auto handler = [this](const ParsedCsvRow& record) { return this->add_transfer(record); };
  return parse_csv(file_transfers, handler);
}

inline Result Feed::write_transfers(const std::string& gtfs_path) const {
  auto container_writer = [this](std::ofstream& out) { return this->write_transfers(out); };
  return write_csv(gtfs_path, file_transfers, write_transfers_header, container_writer);
}

inline const Transfers& Feed::get_transfers() const {
  return transfers;
}

inline boost::optional<Transfer> Feed::get_transfer(const Id& from_stop_id,
                                                    const Id& to_stop_id) const {
  const auto it = std::find_if(transfers.begin(), transfers.end(),
                               [&from_stop_id, &to_stop_id](const Transfer& transfer) {
                                 return transfer.from_stop_id == from_stop_id &&
                                        transfer.to_stop_id == to_stop_id;
                               });

  if (it == transfers.end())
    return boost::none;

  return *it;
}

inline void Feed::add_transfer(const Transfer& transfer) {
  transfers.emplace_back(transfer);
}

inline Result Feed::read_pathways() {
  auto handler = [this](const ParsedCsvRow& record) { return this->add_pathway(record); };
  return parse_csv(file_pathways, handler);
}

inline Result Feed::write_pathways(const std::string& gtfs_path) const {
  auto container_writer = [this](std::ofstream& out) { return this->write_pathways(out); };
  return write_csv(gtfs_path, file_pathways, write_pathways_header, container_writer);
}

inline const Pathways& Feed::get_pathways() const {
  return pathways;
}

inline Pathways Feed::get_pathways(const Id& pathway_id) const {
  Pathways res;
  for (const auto& path : pathways) {
    if (path.pathway_id == pathway_id)
      res.emplace_back(path);
  }
  return res;
}

inline Pathways Feed::get_pathways(const Id& from_stop_id, const Id& to_stop_id) const {
  Pathways res;
  for (const auto& path : pathways) {
    if (path.from_stop_id == from_stop_id && path.to_stop_id == to_stop_id)
      res.emplace_back(path);
  }
  return res;
}

inline void Feed::add_pathway(const Pathway& pathway) {
  pathways.emplace_back(pathway);
}

inline Result Feed::read_levels() {
  auto handler = [this](const ParsedCsvRow& record) { return this->add_level(record); };
  return parse_csv(file_levels, handler);
}

inline Result Feed::write_levels(const std::string& gtfs_path) const {
  auto container_writer = [this](std::ofstream& out) { return this->write_levels(out); };
  return write_csv(gtfs_path, file_levels, write_levels_header, container_writer);
}

inline const Levels& Feed::get_levels() const {
  return levels;
}

inline boost::optional<Level> Feed::get_level(const Id& level_id) const {
  const auto it = std::find_if(levels.begin(), levels.end(), [&level_id](const Level& level) {
    return level.level_id == level_id;
  });

  if (it == levels.end())
    return boost::none;

  return *it;
}

inline void Feed::add_level(const Level& level) {
  levels.emplace_back(level);
}

inline Result Feed::read_feed_info() {
  auto handler = [this](const ParsedCsvRow& record) { return this->add_feed_info(record); };
  return parse_csv(file_feed_info, handler);
}

inline Result Feed::write_feed_info(const std::string& gtfs_path) const {
  auto container_writer = [this](std::ofstream& out) { return this->write_feed_info(out); };
  return write_csv(gtfs_path, file_feed_info, write_feed_info_header, container_writer);
}

inline FeedInfo Feed::get_feed_info() const {
  return feed_info;
}

inline void Feed::set_feed_info(const FeedInfo& info) {
  feed_info = info;
}

inline Result Feed::read_translations() {
  auto handler = [this](const ParsedCsvRow& record) { return this->add_translation(record); };
  return parse_csv(file_translations, handler);
}

inline Result Feed::write_translations(const std::string& gtfs_path) const {
  auto container_writer = [this](std::ofstream& out) { return this->write_translations(out); };
  return write_csv(gtfs_path, file_translations, write_translations_header, container_writer);
}

inline const Translations& Feed::get_translations() const {
  return translations;
}

inline Translations Feed::get_translations(const Text& table_name) const {
  Translations res;
  for (const auto& translation : translations) {
    if (translation.table_name == table_name)
      res.emplace_back(translation);
  }
  return res;
}

inline void Feed::add_translation(const Translation& translation) {
  translations.emplace_back(translation);
}

inline Result Feed::read_attributions() {
  auto handler = [this](const ParsedCsvRow& record) { return this->add_attribution(record); };
  return parse_csv(file_attributions, handler);
}

inline Result Feed::write_attributions(const std::string& gtfs_path) const {
  auto container_writer = [this](std::ofstream& out) { return this->write_attributions(out); };
  return write_csv(gtfs_path, file_attributions, write_attributions_header, container_writer);
}

inline const Attributions& Feed::get_attributions() const {
  return attributions;
}

inline void Feed::add_attribution(const Attribution& attribution) {
  attributions.emplace_back(attribution);
}

inline void Feed::write_agencies(std::ofstream& out) const {
  for (const auto& agency : agencies) {
    std::vector<std::string> fields{wrap(agency.agency_id),  wrap(agency.agency_name),
                                    wrap(agency.agency_url), agency.agency_timezone,
                                    agency.agency_lang,      wrap(agency.agency_phone),
                                    agency.agency_fare_url,  agency.agency_email};
    write_joined(out, std::move(fields));
  }
}

inline void Feed::write_routes(std::ofstream& out) const {
  for (const auto& route : routes) {
    std::vector<std::string> fields{wrap(route.route_id),
                                    wrap(route.agency_id),
                                    wrap(route.route_short_name),
                                    wrap(route.route_long_name),
                                    wrap(route.route_desc),
                                    wrap(route.route_type),
                                    route.route_url,
                                    route.route_color,
                                    route.route_text_color,
                                    wrap(route.route_sort_order),
                                    "" /* continuous_pickup */,
                                    "" /* continuous_drop_off */};
    // TODO: handle new route fields.
    write_joined(out, std::move(fields));
  }
}

inline void Feed::write_shapes(std::ofstream& out) const {
  for (const auto& shape : shapes) {
    std::vector<std::string> fields{wrap(shape.shape_id), wrap(shape.shape_pt_lat),
                                    wrap(shape.shape_pt_lon), wrap(shape.shape_pt_sequence),
                                    wrap(shape.shape_dist_traveled)};
    write_joined(out, std::move(fields));
  }
}

inline void Feed::write_trips(std::ofstream& out) const {
  for (const auto& trip : trips) {
    std::vector<std::string>
        fields{wrap(trip.route_id),      wrap(trip.service_id),      wrap(trip.trip_id),
               wrap(trip.trip_headsign), wrap(trip.trip_short_name), wrap(trip.direction_id),
               wrap(trip.block_id),      wrap(trip.shape_id),        wrap(trip.wheelchair_accessible),
               wrap(trip.bikes_allowed)};
    write_joined(out, std::move(fields));
  }
}

inline void Feed::write_stops(std::ofstream& out) const {
  for (const auto& stop : stops) {
    std::vector<std::string> fields{wrap(stop.stop_id),       wrap(stop.stop_code),
                                    wrap(stop.stop_name),     wrap(stop.stop_desc),
                                    wrap(stop.stop_lat),      wrap(stop.stop_lon),
                                    wrap(stop.zone_id),       stop.stop_url,
                                    wrap(stop.location_type), wrap(stop.parent_station),
                                    stop.stop_timezone,       wrap(stop.wheelchair_boarding),
                                    wrap(stop.level_id),      wrap(stop.platform_code)};
    write_joined(out, std::move(fields));
  }
}

inline void Feed::write_stop_times(std::ofstream& out) const {
  for (const auto& stop_time : stop_times) {
    std::vector<std::string> fields{wrap(stop_time.trip_id),
                                    stop_time.arrival_time.get_raw_time(),
                                    stop_time.departure_time.get_raw_time(),
                                    wrap(stop_time.stop_id),
                                    wrap(stop_time.stop_sequence),
                                    wrap(stop_time.stop_headsign),
                                    wrap(stop_time.pickup_type),
                                    wrap(stop_time.drop_off_type),
                                    "" /* continuous_pickup */,
                                    "" /* continuous_drop_off */,
                                    wrap(stop_time.shape_dist_traveled),
                                    wrap(stop_time.timepoint)};
    // TODO: handle new stop_times fields.
    write_joined(out, std::move(fields));
  }
}

inline void Feed::write_calendar(std::ofstream& out) const {
  for (const auto& item : calendar) {
    std::vector<std::string>
        fields{wrap(item.service_id),       wrap(item.monday),   wrap(item.tuesday),
               wrap(item.wednesday),        wrap(item.thursday), wrap(item.friday),
               wrap(item.saturday),         wrap(item.sunday),   item.start_date.get_raw_date(),
               item.end_date.get_raw_date()};
    write_joined(out, std::move(fields));
  }
}

inline void Feed::write_calendar_dates(std::ofstream& out) const {
  for (const auto& date : calendar_dates) {
    std::vector<std::string> fields{wrap(date.service_id), date.date.get_raw_date(),
                                    wrap(date.exception_type)};
    write_joined(out, std::move(fields));
  }
}

inline void Feed::write_transfers(std::ofstream& out) const {
  for (const auto& transfer : transfers) {
    std::vector<std::string> fields{wrap(transfer.from_stop_id), wrap(transfer.to_stop_id),
                                    wrap(transfer.transfer_type), wrap(transfer.min_transfer_time)};
    write_joined(out, std::move(fields));
  }
}

inline void Feed::write_frequencies(std::ofstream& out) const {
  for (const auto& frequency : frequencies) {
    std::vector<std::string> fields{wrap(frequency.trip_id), frequency.start_time.get_raw_time(),
                                    frequency.end_time.get_raw_time(), wrap(frequency.headway_secs),
                                    wrap(frequency.exact_times)};
    write_joined(out, std::move(fields));
  }
}

inline void Feed::write_fare_attributes(std::ofstream& out) const {
  for (const auto& attribute : fare_attributes) {
    std::vector<std::string> fields{wrap(attribute.fare_id),          wrap(attribute.price),
                                    attribute.currency_type,          wrap(attribute.payment_method),
                                    wrap(attribute.transfers),        wrap(attribute.agency_id),
                                    wrap(attribute.transfer_duration)};
    write_joined(out, std::move(fields));
  }
}

inline void Feed::write_fare_rules(std::ofstream& out) const {
  for (const auto& rule : fare_rules) {
    std::vector<std::string> fields{wrap(rule.fare_id), wrap(rule.route_id), wrap(rule.origin_id),
                                    wrap(rule.destination_id), wrap(rule.contains_id)};
    write_joined(out, std::move(fields));
  }
}

inline void Feed::write_pathways(std::ofstream& out) const {
  for (const auto& path : pathways) {
    std::vector<std::string> fields{wrap(path.pathway_id),       wrap(path.from_stop_id),
                                    wrap(path.to_stop_id),       wrap(path.pathway_mode),
                                    wrap(path.is_bidirectional), wrap(path.length),
                                    wrap(path.traversal_time),   wrap(path.stair_count),
                                    wrap(path.max_slope),        wrap(path.min_width),
                                    wrap(path.signposted_as),    wrap(path.reversed_signposted_as)};
    write_joined(out, std::move(fields));
  }
}

inline void Feed::write_levels(std::ofstream& out) const {
  for (const auto& level : levels) {
    std::vector<std::string> fields{wrap(level.level_id), wrap(level.level_index),
                                    wrap(level.level_name)};
    write_joined(out, std::move(fields));
  }
}

inline void Feed::write_feed_info(std::ofstream& out) const {
  std::vector<std::string> fields{wrap(feed_info.feed_publisher_name),
                                  feed_info.feed_publisher_url,
                                  feed_info.feed_lang,
                                  "" /* default_lang */,
                                  feed_info.feed_start_date.get_raw_date(),
                                  feed_info.feed_end_date.get_raw_date(),
                                  wrap(feed_info.feed_version),
                                  feed_info.feed_contact_email,
                                  feed_info.feed_contact_url};
  // TODO: handle new field_info field.
  write_joined(out, std::move(fields));
}

inline void Feed::write_translations(std::ofstream& out) const {
  for (const auto& translation : translations) {
    std::vector<std::string> fields{translation.table_name,       translation.field_name,
                                    translation.language,         wrap(translation.translation),
                                    wrap(translation.record_id),  wrap(translation.record_sub_id),
                                    wrap(translation.field_value)};
    write_joined(out, std::move(fields));
  }
}

inline void Feed::write_attributions(std::ofstream& out) const {
  for (const auto& attr : attributions) {
    std::vector<std::string> fields{wrap(attr.attribution_id),    wrap(attr.agency_id),
                                    wrap(attr.route_id),          wrap(attr.trip_id),
                                    wrap(attr.organization_name), wrap(attr.is_producer),
                                    wrap(attr.is_operator),       wrap(attr.is_authority),
                                    attr.attribution_url,         attr.attribution_email,
                                    attr.attribution_phone};
    write_joined(out, std::move(fields));
  }
}
} // namespace gtfs
