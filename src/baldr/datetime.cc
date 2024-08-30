#include <algorithm>
#include <sstream>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/join.hpp>

#include "baldr/datetime.h"
#include "baldr/graphconstants.h"
#include "baldr/timedomain.h"
#include "midgard/logging.h"
#include "midgard/util.h"

namespace { // NOTE, the below timezone maps are indexed for compatibility reasons. We put the value
            // (index)
const valhalla::baldr::DateTime::dt_info_t INVALID_DT = {"", "", ""};
// into the tiles, so it needs to be stable. We have 2 timezone maps here:
//  - tz_name_to_id:
//    - before 2023c: currently and past official timezone names
//    - after 2023c: we point to the old timezone using a bit shift which is resolved in NodeInfo
//      to keep forward-compatibility
//  - tz_new_to_old_id:
//    - before 2023c: deprecated timezone names to their current names
//    - after 2023c: new name of renamed/merged timezone(s) to their previous/deprecated name to keep
//      forward-compatibility
// When updating the timezone release and new timezones were added, add them to the end and keep
// incrementing the index. NEVER remove any entry in either map.
const std::unordered_map<std::string, size_t> tz_name_to_id = {
    {"Africa/Abidjan", 1}, // start timezones release 2018d;
    {"Africa/Accra", 2},
    {"Africa/Algiers", 3},
    {"Africa/Bissau", 4},
    {"Africa/Cairo", 5},
    {"Africa/Casablanca", 6},
    {"Africa/Ceuta", 7},
    {"Africa/El_Aaiun", 8},
    {"Africa/Johannesburg", 9},
    {"Africa/Juba", 10},
    {"Africa/Khartoum", 11},
    {"Africa/Lagos", 12},
    {"Africa/Maputo", 13},
    {"Africa/Monrovia", 14},
    {"Africa/Nairobi", 15},
    {"Africa/Ndjamena", 16},
    {"Africa/Sao_Tome", 17},
    {"Africa/Tripoli", 18},
    {"Africa/Tunis", 19},
    {"Africa/Windhoek", 20},
    {"America/Adak", 21},
    {"America/Anchorage", 22},
    {"America/Araguaina", 23},
    {"America/Argentina/Buenos_Aires", 24},
    {"America/Argentina/Catamarca", 25},
    {"America/Argentina/Cordoba", 26},
    {"America/Argentina/Jujuy", 27},
    {"America/Argentina/La_Rioja", 28},
    {"America/Argentina/Mendoza", 29},
    {"America/Argentina/Rio_Gallegos", 30},
    {"America/Argentina/Salta", 31},
    {"America/Argentina/San_Juan", 32},
    {"America/Argentina/San_Luis", 33},
    {"America/Argentina/Tucuman", 34},
    {"America/Argentina/Ushuaia", 35},
    {"America/Asuncion", 36},
    {"America/Atikokan", 37},
    {"America/Bahia", 38},
    {"America/Bahia_Banderas", 39},
    {"America/Barbados", 40},
    {"America/Belem", 41},
    {"America/Belize", 42},
    {"America/Blanc-Sablon", 43},
    {"America/Boa_Vista", 44},
    {"America/Bogota", 45},
    {"America/Boise", 46},
    {"America/Cambridge_Bay", 47},
    {"America/Campo_Grande", 48},
    {"America/Cancun", 49},
    {"America/Caracas", 50},
    {"America/Cayenne", 51},
    {"America/Chicago", 52},
    {"America/Chihuahua", 53},
    {"America/Costa_Rica", 54},
    {"America/Creston", 55},
    {"America/Cuiaba", 56},
    {"America/Curacao", 57},
    {"America/Danmarkshavn", 58},
    {"America/Dawson", 59},
    {"America/Dawson_Creek", 60},
    {"America/Denver", 61},
    {"America/Detroit", 62},
    {"America/Edmonton", 63},
    {"America/Eirunepe", 64},
    {"America/El_Salvador", 65},
    {"America/Fort_Nelson", 66},
    {"America/Fortaleza", 67},
    {"America/Glace_Bay", 68},
    {"America/Godthab", 69},
    {"America/Goose_Bay", 70},
    {"America/Grand_Turk", 71},
    {"America/Guatemala", 72},
    {"America/Guayaquil", 73},
    {"America/Guyana", 74},
    {"America/Halifax", 75},
    {"America/Havana", 76},
    {"America/Hermosillo", 77},
    {"America/Indiana/Indianapolis", 78},
    {"America/Indiana/Knox", 79},
    {"America/Indiana/Marengo", 80},
    {"America/Indiana/Petersburg", 81},
    {"America/Indiana/Tell_City", 82},
    {"America/Indiana/Vevay", 83},
    {"America/Indiana/Vincennes", 84},
    {"America/Indiana/Winamac", 85},
    {"America/Inuvik", 86},
    {"America/Iqaluit", 87},
    {"America/Jamaica", 88},
    {"America/Juneau", 89},
    {"America/Kentucky/Louisville", 90},
    {"America/Kentucky/Monticello", 91},
    {"America/La_Paz", 92},
    {"America/Lima", 93},
    {"America/Los_Angeles", 94},
    {"America/Maceio", 95},
    {"America/Managua", 96},
    {"America/Manaus", 97},
    {"America/Martinique", 98},
    {"America/Matamoros", 99},
    {"America/Mazatlan", 100},
    {"America/Menominee", 101},
    {"America/Merida", 102},
    {"America/Metlakatla", 103},
    {"America/Mexico_City", 104},
    {"America/Miquelon", 105},
    {"America/Moncton", 106},
    {"America/Monterrey", 107},
    {"America/Montevideo", 108},
    {"America/Nassau", 109},
    {"America/New_York", 110},
    {"America/Nipigon", 111},
    {"America/Nome", 112},
    {"America/Noronha", 113},
    {"America/North_Dakota/Beulah", 114},
    {"America/North_Dakota/Center", 115},
    {"America/North_Dakota/New_Salem", 116},
    {"America/Ojinaga", 117},
    {"America/Panama", 118},
    {"America/Pangnirtung", 119},
    {"America/Paramaribo", 120},
    {"America/Phoenix", 121},
    {"America/Port-au-Prince", 122},
    {"America/Port_of_Spain", 123},
    {"America/Porto_Velho", 124},
    {"America/Puerto_Rico", 125},
    {"America/Punta_Arenas", 126},
    {"America/Rainy_River", 127},
    {"America/Rankin_Inlet", 128},
    {"America/Recife", 129},
    {"America/Regina", 130},
    {"America/Resolute", 131},
    {"America/Rio_Branco", 132},
    {"America/Santarem", 133},
    {"America/Santiago", 134},
    {"America/Santo_Domingo", 135},
    {"America/Sao_Paulo", 136},
    {"America/Scoresbysund", 137},
    {"America/Sitka", 138},
    {"America/St_Johns", 139},
    {"America/Swift_Current", 140},
    {"America/Tegucigalpa", 141},
    {"America/Thule", 142},
    {"America/Thunder_Bay", 143},
    {"America/Tijuana", 144},
    {"America/Toronto", 145},
    {"America/Vancouver", 146},
    {"America/Whitehorse", 147},
    {"America/Winnipeg", 148},
    {"America/Yakutat", 149},
    {"America/Yellowknife", 150},
    {"Antarctica/Casey", 151},
    {"Antarctica/Davis", 152},
    {"Antarctica/DumontDUrville", 153},
    {"Antarctica/Macquarie", 154},
    {"Antarctica/Mawson", 155},
    {"Antarctica/Palmer", 156},
    {"Antarctica/Rothera", 157},
    {"Antarctica/Syowa", 158},
    {"Antarctica/Troll", 159},
    {"Antarctica/Vostok", 160},
    {"Asia/Almaty", 161},
    {"Asia/Amman", 162},
    {"Asia/Anadyr", 163},
    {"Asia/Aqtau", 164},
    {"Asia/Aqtobe", 165},
    {"Asia/Ashgabat", 166},
    {"Asia/Atyrau", 167},
    {"Asia/Baghdad", 168},
    {"Asia/Baku", 169},
    {"Asia/Bangkok", 170},
    {"Asia/Barnaul", 171},
    {"Asia/Beirut", 172},
    {"Asia/Bishkek", 173},
    {"Asia/Brunei", 174},
    {"Asia/Chita", 175},
    {"Asia/Choibalsan", 176},
    {"Asia/Colombo", 177},
    {"Asia/Damascus", 178},
    {"Asia/Dhaka", 179},
    {"Asia/Dili", 180},
    {"Asia/Dubai", 181},
    {"Asia/Dushanbe", 182},
    {"Asia/Famagusta", 183},
    {"Asia/Gaza", 184},
    {"Asia/Hebron", 185},
    {"Asia/Ho_Chi_Minh", 186},
    {"Asia/Hong_Kong", 187},
    {"Asia/Hovd", 188},
    {"Asia/Irkutsk", 189},
    {"Asia/Jakarta", 190},
    {"Asia/Jayapura", 191},
    {"Asia/Jerusalem", 192},
    {"Asia/Kabul", 193},
    {"Asia/Kamchatka", 194},
    {"Asia/Karachi", 195},
    {"Asia/Kathmandu", 196},
    {"Asia/Khandyga", 197},
    {"Asia/Kolkata", 198},
    {"Asia/Krasnoyarsk", 199},
    {"Asia/Kuala_Lumpur", 200},
    {"Asia/Kuching", 201},
    {"Asia/Macau", 202},
    {"Asia/Magadan", 203},
    {"Asia/Makassar", 204},
    {"Asia/Manila", 205},
    {"Asia/Nicosia", 206},
    {"Asia/Novokuznetsk", 207},
    {"Asia/Novosibirsk", 208},
    {"Asia/Omsk", 209},
    {"Asia/Oral", 210},
    {"Asia/Pontianak", 211},
    {"Asia/Pyongyang", 212},
    {"Asia/Qatar", 213},
    {"Asia/Qyzylorda", 214},
    {"Asia/Riyadh", 215},
    {"Asia/Sakhalin", 216},
    {"Asia/Samarkand", 217},
    {"Asia/Seoul", 218},
    {"Asia/Shanghai", 219},
    {"Asia/Singapore", 220},
    {"Asia/Srednekolymsk", 221},
    {"Asia/Taipei", 222},
    {"Asia/Tashkent", 223},
    {"Asia/Tbilisi", 224},
    {"Asia/Tehran", 225},
    {"Asia/Thimphu", 226},
    {"Asia/Tokyo", 227},
    {"Asia/Tomsk", 228},
    {"Asia/Ulaanbaatar", 229},
    {"Asia/Urumqi", 230},
    {"Asia/Ust-Nera", 231},
    {"Asia/Vladivostok", 232},
    {"Asia/Yakutsk", 233},
    {"Asia/Yangon", 234},
    {"Asia/Yekaterinburg", 235},
    {"Asia/Yerevan", 236},
    {"Atlantic/Azores", 237},
    {"Atlantic/Bermuda", 238},
    {"Atlantic/Canary", 239},
    {"Atlantic/Cape_Verde", 240},
    {"Atlantic/Faroe", 241},
    {"Atlantic/Madeira", 242},
    {"Atlantic/Reykjavik", 243},
    {"Atlantic/South_Georgia", 244},
    {"Atlantic/Stanley", 245},
    {"Australia/Adelaide", 246},
    {"Australia/Brisbane", 247},
    {"Australia/Broken_Hill", 248},
    {"Australia/Currie", 249},
    {"Australia/Darwin", 250},
    {"Australia/Eucla", 251},
    {"Australia/Hobart", 252},
    {"Australia/Lindeman", 253},
    {"Australia/Lord_Howe", 254},
    {"Australia/Melbourne", 255},
    {"Australia/Perth", 256},
    {"Australia/Sydney", 257},
    {"CET", 258},
    {"CST6CDT", 259},
    {"EET", 260},
    {"EST", 261},
    {"EST5EDT", 262},
    {"Etc/GMT", 263},
    {"Etc/GMT+1", 264},
    {"Etc/GMT+10", 265},
    {"Etc/GMT+11", 266},
    {"Etc/GMT+12", 267},
    {"Etc/GMT+2", 268},
    {"Etc/GMT+3", 269},
    {"Etc/GMT+4", 270},
    {"Etc/GMT+5", 271},
    {"Etc/GMT+6", 272},
    {"Etc/GMT+7", 273},
    {"Etc/GMT+8", 274},
    {"Etc/GMT+9", 275},
    {"Etc/GMT-1", 276},
    {"Etc/GMT-10", 277},
    {"Etc/GMT-11", 278},
    {"Etc/GMT-12", 279},
    {"Etc/GMT-13", 280},
    {"Etc/GMT-14", 281},
    {"Etc/GMT-2", 282},
    {"Etc/GMT-3", 283},
    {"Etc/GMT-4", 284},
    {"Etc/GMT-5", 285},
    {"Etc/GMT-6", 286},
    {"Etc/GMT-7", 287},
    {"Etc/GMT-8", 288},
    {"Etc/GMT-9", 289},
    {"Etc/UCT", 290},
    {"Etc/UTC", 291},
    {"Europe/Amsterdam", 292},
    {"Europe/Andorra", 293},
    {"Europe/Astrakhan", 294},
    {"Europe/Athens", 295},
    {"Europe/Belgrade", 296},
    {"Europe/Berlin", 297},
    {"Europe/Brussels", 298},
    {"Europe/Bucharest", 299},
    {"Europe/Budapest", 300},
    {"Europe/Chisinau", 301},
    {"Europe/Copenhagen", 302},
    {"Europe/Dublin", 303},
    {"Europe/Gibraltar", 304},
    {"Europe/Helsinki", 305},
    {"Europe/Istanbul", 306},
    {"Europe/Kaliningrad", 307},
    {"Europe/Kiev", 308},
    {"Europe/Kirov", 309},
    {"Europe/Lisbon", 310},
    {"Europe/London", 311},
    {"Europe/Luxembourg", 312},
    {"Europe/Madrid", 313},
    {"Europe/Malta", 314},
    {"Europe/Minsk", 315},
    {"Europe/Monaco", 316},
    {"Europe/Moscow", 317},
    {"Europe/Oslo", 318},
    {"Europe/Paris", 319},
    {"Europe/Prague", 320},
    {"Europe/Riga", 321},
    {"Europe/Rome", 322},
    {"Europe/Samara", 323},
    {"Europe/Saratov", 324},
    {"Europe/Simferopol", 325},
    {"Europe/Sofia", 326},
    {"Europe/Stockholm", 327},
    {"Europe/Tallinn", 328},
    {"Europe/Tirane", 329},
    {"Europe/Ulyanovsk", 330},
    {"Europe/Uzhgorod", 331},
    {"Europe/Vienna", 332},
    {"Europe/Vilnius", 333},
    {"Europe/Volgograd", 334},
    {"Europe/Warsaw", 335},
    {"Europe/Zaporozhye", 336},
    {"Europe/Zurich", 337},
    {"HST", 338},
    {"Indian/Chagos", 339},
    {"Indian/Christmas", 340},
    {"Indian/Cocos", 341},
    {"Indian/Kerguelen", 342},
    {"Indian/Mahe", 343},
    {"Indian/Maldives", 344},
    {"Indian/Mauritius", 345},
    {"Indian/Reunion", 346},
    {"MET", 347},
    {"MST", 348},
    {"MST7MDT", 349},
    {"PST8PDT", 350},
    {"Pacific/Apia", 351},
    {"Pacific/Auckland", 352},
    {"Pacific/Bougainville", 353},
    {"Pacific/Chatham", 354},
    {"Pacific/Chuuk", 355},
    {"Pacific/Easter", 356},
    {"Pacific/Efate", 357},
    {"Pacific/Enderbury", 358},
    {"Pacific/Fakaofo", 359},
    {"Pacific/Fiji", 360},
    {"Pacific/Funafuti", 361},
    {"Pacific/Galapagos", 362},
    {"Pacific/Gambier", 363},
    {"Pacific/Guadalcanal", 364},
    {"Pacific/Guam", 365},
    {"Pacific/Honolulu", 366},
    {"Pacific/Kiritimati", 367},
    {"Pacific/Kosrae", 368},
    {"Pacific/Kwajalein", 369},
    {"Pacific/Majuro", 370},
    {"Pacific/Marquesas", 371},
    {"Pacific/Nauru", 372},
    {"Pacific/Niue", 373},
    {"Pacific/Norfolk", 374},
    {"Pacific/Noumea", 375},
    {"Pacific/Pago_Pago", 376},
    {"Pacific/Palau", 377},
    {"Pacific/Pitcairn", 378},
    {"Pacific/Pohnpei", 379},
    {"Pacific/Port_Moresby", 380},
    {"Pacific/Rarotonga", 381},
    {"Pacific/Tahiti", 382},
    {"Pacific/Tarawa", 383},
    {"Pacific/Tongatapu", 384},
    {"Pacific/Wake", 385},
    {"Pacific/Wallis", 386},
    {"WET", 387},                              // end timezones release 2018d;
    {"America/Ciudad_Juarez", 117 | (1 << 9)}, // start timezones release 2023c; was America/Ojinaga
    {"Asia/Qostanay", 214 | (1 << 9)},         // end timezones release 2023c; was Asia/Qyzylorda
};

// this holds:
//   - before 2023c: deprecated timezone names to their current names
//   - after  2023c: new name of renamed/merged timezone(s) to their previous/deprecated name to keep
//      forward-compatibility
const std::unordered_map<std::string, size_t> tz_new_to_old_id = {
    {"Africa/Addis_Ababa", 15}, // start deprecated timezones 2018d
    {"Africa/Asmara", 15},
    {"Africa/Asmera", 15},
    {"Africa/Bamako", 1},
    {"Africa/Bangui", 12},
    {"Africa/Banjul", 1},
    {"Africa/Blantyre", 13},
    {"Africa/Brazzaville", 12},
    {"Africa/Bujumbura", 13},
    {"Africa/Conakry", 1},
    {"Africa/Dakar", 1},
    {"Africa/Dar_es_Salaam", 15},
    {"Africa/Djibouti", 15},
    {"Africa/Douala", 12},
    {"Africa/Freetown", 1},
    {"Africa/Gaborone", 13},
    {"Africa/Harare", 13},
    {"Africa/Kampala", 15},
    {"Africa/Kigali", 13},
    {"Africa/Kinshasa", 12},
    {"Africa/Libreville", 12},
    {"Africa/Lome", 1},
    {"Africa/Luanda", 12},
    {"Africa/Lubumbashi", 13},
    {"Africa/Lusaka", 13},
    {"Africa/Malabo", 12},
    {"Africa/Maseru", 9},
    {"Africa/Mbabane", 9},
    {"Africa/Mogadishu", 15},
    {"Africa/Niamey", 12},
    {"Africa/Nouakchott", 1},
    {"Africa/Ouagadougou", 1},
    {"Africa/Porto-Novo", 12},
    {"Africa/Timbuktu", 1},
    {"America/Anguilla", 123},
    {"America/Antigua", 123},
    {"America/Argentina/ComodRivadavia", 25},
    {"America/Aruba", 57},
    {"America/Atka", 21},
    {"America/Buenos_Aires", 24},
    {"America/Catamarca", 25},
    {"America/Cayman", 118},
    {"America/Coral_Harbour", 37},
    {"America/Cordoba", 26},
    {"America/Dominica", 123},
    {"America/Ensenada", 144},
    {"America/Fort_Wayne", 78},
    {"America/Grenada", 123},
    {"America/Guadeloupe", 123},
    {"America/Indianapolis", 78},
    {"America/Jujuy", 27},
    {"America/Knox_IN", 79},
    {"America/Kralendijk", 57},
    {"America/Louisville", 90},
    {"America/Lower_Princes", 57},
    {"America/Marigot", 123},
    {"America/Mendoza", 29},
    {"America/Montreal", 145},
    {"America/Montserrat", 123},
    {"America/Porto_Acre", 132},
    {"America/Rosario", 26},
    {"America/Santa_Isabel", 144},
    {"America/Shiprock", 61},
    {"America/St_Barthelemy", 123},
    {"America/St_Kitts", 123},
    {"America/St_Lucia", 123},
    {"America/St_Thomas", 123},
    {"America/St_Vincent", 123},
    {"America/Tortola", 123},
    {"America/Virgin", 123},
    {"Antarctica/McMurdo", 352},
    {"Antarctica/South_Pole", 352},
    {"Arctic/Longyearbyen", 318},
    {"Asia/Aden", 215},
    {"Asia/Ashkhabad", 166},
    {"Asia/Bahrain", 213},
    {"Asia/Calcutta", 198},
    {"Asia/Chongqing", 219},
    {"Asia/Chungking", 219},
    {"Asia/Dacca", 179},
    {"Asia/Harbin", 219},
    {"Asia/Istanbul", 306},
    {"Asia/Kashgar", 230},
    {"Asia/Katmandu", 196},
    {"Asia/Kuwait", 215},
    {"Asia/Macao", 202},
    {"Asia/Muscat", 181},
    {"Asia/Phnom_Penh", 170},
    {"Asia/Rangoon", 234},
    {"Asia/Saigon", 186},
    {"Asia/Tel_Aviv", 192},
    {"Asia/Thimbu", 226},
    {"Asia/Ujung_Pandang", 204},
    {"Asia/Ulan_Bator", 229},
    {"Asia/Vientiane", 170},
    {"Atlantic/Faeroe", 241},
    {"Atlantic/Jan_Mayen", 318},
    {"Atlantic/St_Helena", 1},
    {"Australia/ACT", 257},
    {"Australia/Canberra", 257},
    {"Australia/LHI", 254},
    {"Australia/NSW", 257},
    {"Australia/North", 250},
    {"Australia/Queensland", 247},
    {"Australia/South", 246},
    {"Australia/Tasmania", 252},
    {"Australia/Victoria", 255},
    {"Australia/West", 256},
    {"Australia/Yancowinna", 248},
    {"Brazil/Acre", 132},
    {"Brazil/DeNoronha", 113},
    {"Brazil/East", 136},
    {"Brazil/West", 97},
    {"Canada/Atlantic", 75},
    {"Canada/Central", 148},
    {"Canada/Eastern", 145},
    {"Canada/Mountain", 63},
    {"Canada/Newfoundland", 139},
    {"Canada/Pacific", 146},
    {"Canada/Saskatchewan", 130},
    {"Canada/Yukon", 147},
    {"Chile/Continental", 134},
    {"Chile/EasterIsland", 356},
    {"Cuba", 76},
    {"Egypt", 5},
    {"Eire", 303},
    {"Etc/GMT+0", 263},
    {"Etc/GMT-0", 263},
    {"Etc/GMT0", 263},
    {"Etc/Greenwich", 263},
    {"Etc/Universal", 291},
    {"Etc/Zulu", 291},
    {"Europe/Belfast", 311},
    {"Europe/Bratislava", 320},
    {"Europe/Busingen", 337},
    {"Europe/Guernsey", 311},
    {"Europe/Isle_of_Man", 311},
    {"Europe/Jersey", 311},
    {"Europe/Ljubljana", 296},
    {"Europe/Mariehamn", 305},
    {"Europe/Nicosia", 206},
    {"Europe/Podgorica", 296},
    {"Europe/San_Marino", 322},
    {"Europe/Sarajevo", 296},
    {"Europe/Skopje", 296},
    {"Europe/Tiraspol", 301},
    {"Europe/Vaduz", 337},
    {"Europe/Vatican", 322},
    {"Europe/Zagreb", 296},
    {"GB", 311},
    {"GB-Eire", 311},
    {"GMT", 263},
    {"GMT+0", 263},
    {"GMT-0", 263},
    {"GMT0", 263},
    {"Greenwich", 263},
    {"Hongkong", 187},
    {"Iceland", 243},
    {"Indian/Antananarivo", 15},
    {"Indian/Comoro", 15},
    {"Indian/Mayotte", 15},
    {"Iran", 225},
    {"Israel", 192},
    {"Jamaica", 88},
    {"Japan", 227},
    {"Kwajalein", 369},
    {"Libya", 18},
    {"Mexico/BajaNorte", 144},
    {"Mexico/BajaSur", 100},
    {"Mexico/General", 104},
    {"NZ", 352},
    {"NZ-CHAT", 354},
    {"Navajo", 61},
    {"PRC", 219},
    {"Pacific/Johnston", 366},
    {"Pacific/Midway", 376},
    {"Pacific/Ponape", 379},
    {"Pacific/Saipan", 365},
    {"Pacific/Samoa", 376},
    {"Pacific/Truk", 355},
    {"Pacific/Yap", 355},
    {"Poland", 335},
    {"Portugal", 310},
    {"ROC", 222},
    {"ROK", 218},
    {"Singapore", 220},
    {"Turkey", 306},
    {"UCT", 290},
    {"US/Alaska", 22},
    {"US/Aleutian", 21},
    {"US/Arizona", 121},
    {"US/Central", 52},
    {"US/East-Indiana", 78},
    {"US/Eastern", 110},
    {"US/Hawaii", 366},
    {"US/Indiana-Starke", 79},
    {"US/Michigan", 62},
    {"US/Mountain", 61},
    {"US/Pacific", 94},
    {"US/Pacific-New", 94},
    {"US/Samoa", 376},
    {"UTC", 291},
    {"Universal", 291},
    {"W-SU", 317},
    {"Zulu", 291},        // end deprecated timezones 2018d
    {"America/Nuuk", 69}, // start renamed timezones 2023c
    {"Europe/Kyiv", 308},
    {"Pacific/Kanton", 358}, // end renamed timezones 2023c
};

// checks the integrity of the static tz maps, which will fail in case of
// tzdb updates. this function pretty-prints missing tzs for convenience
const std::string check_tz_map(const date::tzdb& db) {
  std::vector<std::string> new_zones_msg, new_links_msg;
  std::unordered_set<std::string> new_zones_names;

  // first find the link's targets, then the new timezones, to not get dups
  for (const auto& link : db.links) {
    // if we already know both link & target tz, we can skip; most likely
    // means that existing timezones were merged which is fine for us
    if (tz_name_to_id.find(link.target()) != tz_name_to_id.end() ||
        tz_new_to_old_id.find(link.target()) != tz_new_to_old_id.end() ||
        !new_zones_names.insert(link.target()).second) {
      continue;
    }

    // correlate the new timezone with the old tz ID for compat
    auto old_tz = tz_name_to_id.find(link.name());
    new_links_msg.emplace_back("{\"" + link.target() + "\", " + std::to_string(old_tz->second) +
                               "},");
  }

  for (const auto& tz : db.zones) {
    // only add entirely new zones if we didn't see them as link target yet
    if (tz_name_to_id.find(tz.name()) == tz_name_to_id.end() &&
        tz_new_to_old_id.find(tz.name()) == tz_new_to_old_id.end() &&
        std::find(new_zones_names.begin(), new_zones_names.end(), tz.name()) ==
            new_zones_names.end()) {
      new_zones_msg.emplace_back(tz.name());
    }
  }

  std::string result;
  if (new_links_msg.size()) {
    result += "\nNew links: \n" + boost::algorithm::join(new_links_msg, "\n");
  }
  if (new_zones_msg.size()) {
    result +=
        "\nNew timezones to be manually resolved: \n" + boost::algorithm::join(new_zones_msg, "\n");
  }

  return result;
}
// use a cache to store already constructed sys_info's since they aren't cheap
template <typename TP>
const date::sys_info&
from_cache(const TP& tp,
           const date::time_zone* tz,
           std::unordered_map<const date::time_zone*, std::vector<date::sys_info>>& cache) {
  // check if we have anything for this timezone in the cache
  auto tz_it = cache.find(tz);
  if (tz_it != cache.cend()) {
    // we have something in the cache, see if one of the infos has the particular time point in range
    auto st = date::floor<std::chrono::seconds>(tp.get_sys_time());
    auto info_it =
        std::find_if(tz_it->second.begin(), tz_it->second.end(),
                     [&st](const date::sys_info& info) { return info.begin <= st && st < info.end; });

    // if it was in the cache we return it
    if (info_it != tz_it->second.cend()) {
      return *info_it;
    }
  }

  // either this timezone is new or the right info for the range was missing so lets get it
  auto& infos = tz_it == cache.cend() ? cache.emplace(tz, std::vector<date::sys_info>{}).first->second
                                      : tz_it->second;
  infos.emplace_back(tp.get_info());
  return infos.back();
}
} // namespace

using namespace valhalla::baldr;
namespace valhalla {
namespace baldr {
namespace DateTime {

tz_db_t::tz_db_t() {
  const auto& db = date::get_tzdb();

  // update timezones & run the tests will fail here if new timezones were added
  if (const std::string& msg = check_tz_map(db); msg.size()) {
    throw std::runtime_error("Update timezone map at " + std::string(__FILE__) + ": " + msg);
  }

  zones.reserve(tz_name_to_id.size());
  for (const auto& tz_pair : tz_name_to_id) {
    // we find either the official timezone or we get the target timezone of a link
    auto* tz = db.locate_zone(tz_pair.first);
    zones[tz_pair.second] = &*tz;
  }
}

size_t tz_db_t::to_index(const std::string& zone) const {
  auto it = tz_name_to_id.find(zone);
  if (it != tz_name_to_id.cend() || (it = tz_new_to_old_id.find(zone)) != tz_new_to_old_id.end()) {
    return it->second;
  }
  return 0;
}

const date::time_zone* tz_db_t::from_index(size_t index) const {
  auto it = zones.find(index);
  return it != zones.end() ? it->second : nullptr;
}

const tz_db_t& get_tz_db() {
  static const tz_db_t tz_db;
  return tz_db;
}

// get a formatted date.  date in the format of 2016-11-06T01:00 or 2016-11-06
date::local_seconds get_formatted_date(const std::string& date, bool can_throw) {
  std::istringstream in{date};
  date::local_seconds tp;

  if (date.find('T') != std::string::npos)
    in >> date::parse("%FT%R", tp);
  else if (date.find('-') != std::string::npos)
    in >> date::parse("%F", tp);
  else
    in.setstate(std::ios::failbit);

  // we weren't able to use this string as a date and you'd like to know about it
  if (can_throw && in.fail())
    throw std::invalid_argument("Date string is invalid: " + date);

  return tp;
}

// get a local_date_time with support for dst.  Assumes that we are moving
// forward in time.  e.g. depart at
// 2016-11-06T02:00 ---> 2016-11-06T01:00
date::zoned_seconds get_ldt(const date::local_seconds& d, const date::time_zone* time_zone) {
  if (!time_zone)
    return date::zoned_seconds(0);
  date::zoned_time<std::chrono::seconds> zt = date::make_zoned(time_zone, d, date::choose::latest);
  return zt;
}

// Get the number of days that have elapsed from the pivot date for the input date.
// date_time is in the format of 2015-05-06T08:00
uint32_t days_from_pivot_date(const date::local_seconds& date_time) {
  if (date_time <= pivot_date_) {
    return 0;
  }
  return static_cast<uint32_t>(date::floor<date::days>(date_time - pivot_date_).count());
}

// Get the current iso date and time.
std::string iso_date_time(const date::time_zone* time_zone) {
  if (!time_zone)
    return "";
  std::ostringstream iso_date_time;
  const auto date = date::make_zoned(time_zone, std::chrono::system_clock::now());
  iso_date_time << date::format("%FT%R", date);
  return iso_date_time.str();
}

// Get the seconds since epoch time is already adjusted based on TZ
uint64_t seconds_since_epoch(const std::string& date_time, const date::time_zone* time_zone) {
  if (date_time.empty() || !time_zone) {
    return 0;
  }
  const auto d = get_formatted_date(date_time);
  const auto utc = date::to_utc_time(get_ldt(d, time_zone).get_sys_time()); // supports leap sec.
  return static_cast<uint64_t>(utc.time_since_epoch().count());
}

// Get the difference between two timezones using the current time (seconds from epoch
// so that DST can be take into account). Returns the difference in seconds.
int timezone_diff(const uint64_t seconds,
                  const date::time_zone* origin_tz,
                  const date::time_zone* dest_tz,
                  tz_sys_info_cache_t* cache) {

  if (!origin_tz || !dest_tz || origin_tz == dest_tz) {
    return 0;
  }
  std::chrono::seconds dur(seconds);
  std::chrono::time_point<std::chrono::system_clock> tp(dur);

  const auto origin = date::make_zoned(origin_tz, tp);
  const auto dest = date::make_zoned(dest_tz, tp);

  // if we have a cache use it
  const auto& origin_info = cache ? from_cache(origin, origin_tz, *cache) : origin.get_info();
  const auto& dest_info = cache ? from_cache(dest, dest_tz, *cache) : dest.get_info();
  return static_cast<int>(
      std::chrono::duration_cast<std::chrono::seconds>(dest_info.offset - origin_info.offset)
          .count());
}

std::string
seconds_to_date(const uint64_t seconds, const date::time_zone* time_zone, bool tz_format) {

  std::string iso_date;
  if (seconds == 0 || !time_zone) {
    return iso_date;
  }

  std::chrono::seconds dur(seconds);
  std::chrono::time_point<std::chrono::system_clock> tp(dur);
  const auto date = date::make_zoned(time_zone, tp);

  std::ostringstream iso_date_time;
  if (tz_format)
    iso_date_time << date::format("%FT%R%z", date);
  else
    iso_date_time << date::format("%FT%R", date);
  iso_date = iso_date_time.str();
  if (tz_format)
    iso_date.insert(19, 1, ':');
  return iso_date;
}

// Get the date from seconds and timezone.
void seconds_to_date(const uint64_t origin_seconds,
                     const uint64_t dest_seconds,
                     const date::time_zone* origin_tz,
                     const date::time_zone* dest_tz,
                     std::string& iso_origin,
                     std::string& iso_dest) {

  if (!origin_tz || !dest_tz)
    return;

  iso_origin = seconds_to_date(origin_seconds, origin_tz);
  iso_dest = seconds_to_date(dest_seconds, dest_tz);
}

// Get the dow mask
// date_time is in the format of 2015-05-06T08:00
uint32_t day_of_week_mask(const std::string& date_time) {
  date::local_seconds date;
  date = get_formatted_date(date_time);
  if (date < pivot_date_) {
    return kDOWNone;
  }
  auto ld = date::floor<date::days>(date);
  uint8_t wd = (date::weekday(ld) - date::Sunday).count();

  switch (wd) {
    case 0:
      return kSunday;
      break;
    case 1:
      return kMonday;
      break;
    case 2:
      return kTuesday;
      break;
    case 3:
      return kWednesday;
      break;
    case 4:
      return kThursday;
      break;
    case 5:
      return kFriday;
      break;
    case 6:
      return kSaturday;
      break;
  }
  return kDOWNone;
}

// add x seconds to a date_time and return a ISO date_time string.
// date_time is in the format of 2015-05-06T08:00
std::string
get_duration(const std::string& date_time, const uint32_t seconds, const date::time_zone* time_zone) {

  date::local_seconds date;
  date = get_formatted_date(date_time);
  if (date < pivot_date_)
    return "";

  std::chrono::seconds dur(seconds_since_epoch(date_time, time_zone) + seconds);
  std::chrono::time_point<std::chrono::system_clock> tp(dur);
  std::ostringstream iso_date_time;

  const auto origin = date::make_zoned(time_zone, tp);
  iso_date_time << date::format("%FT%R%z %Z", origin);
  std::string iso_date = iso_date_time.str();
  iso_date.insert(19, 1, ':');
  return iso_date;
}

// does this date fall in the begin and end date range?
bool is_conditional_active(const bool type,
                           const uint8_t begin_hrs,
                           const uint8_t begin_mins,
                           const uint8_t end_hrs,
                           const uint8_t end_mins,
                           const uint8_t dow,
                           const uint8_t begin_week,
                           const uint8_t begin_month,
                           const uint8_t begin_day_dow,
                           const uint8_t end_week,
                           const uint8_t end_month,
                           const uint8_t end_day_dow,
                           const uint64_t current_time,
                           const date::time_zone* time_zone) {

  if (!time_zone)
    return false;

  bool dow_in_range = true;
  bool dt_in_range = true;

  // date::time_of_day()
  std::chrono::minutes b_td = std::chrono::hours(0);
  std::chrono::minutes e_td = std::chrono::hours(23) + std::chrono::minutes(59);

  std::chrono::seconds dur(current_time);
  std::chrono::time_point<std::chrono::system_clock> tp(dur);

  uint32_t e_year = 0, b_year = 0;
  const auto in_local_time = date::make_zoned(time_zone, tp);
  auto date = date::floor<date::days>(in_local_time.get_local_time());
  auto d = date::year_month_day(date);
  auto t = date::make_time(in_local_time.get_local_time() - date); // Yields time_of_day type
  std::chrono::minutes td = t.hours() + t.minutes();               // Yields time_of_day type

  try {
    date::year_month_day begin_date, end_date;

    // we have dow
    if (dow) {

      uint8_t wd = (date::weekday{date} - date::Sunday).count();
      uint8_t local_dow = 0;
      switch (wd) {
        case 0:
          local_dow = kSunday;
          break;
        case 1:
          local_dow = kMonday;
          break;
        case 2:
          local_dow = kTuesday;
          break;
        case 3:
          local_dow = kWednesday;
          break;
        case 4:
          local_dow = kThursday;
          break;
        case 5:
          local_dow = kFriday;
          break;
        case 6:
          local_dow = kSaturday;
          break;
        default:
          return false; // should never happen
          break;
      }
      dow_in_range = (dow & local_dow);
    }

    uint8_t b_month = begin_month;
    uint8_t e_month = end_month;
    uint8_t b_day_dow = begin_day_dow;
    uint8_t e_day_dow = end_day_dow;
    uint8_t b_week = begin_week;
    uint8_t e_week = end_week;

    if (type == kNthDow && begin_week && !begin_day_dow && !begin_month) { // Su[-1]
      b_month = unsigned(d.month());
    }
    if (type == kNthDow && end_week && !end_day_dow && !end_month) { // Su[-1]
      e_month = unsigned(d.month());
    }

    if (type == kNthDow && begin_week && !begin_day_dow && !begin_month && !end_week &&
        !end_day_dow && !end_month) { // only Su[-1] set in begin.
      // First Sunday of every month only.
      e_month = b_month;
      b_day_dow = e_day_dow = dow;
      e_week = b_week;
    } else if (type == kYMD && (b_month && e_month) &&
               (!b_day_dow && !e_day_dow)) { // Sep-Jun We 08:15-08:45

      b_day_dow = 1;
      date::year_month_day e_d = date::year_month_day(d.year(), date::month(e_month), date::day(1));
      e_day_dow = unsigned((date::year_month(e_d.year(), e_d.month()) / date::last).day());
    }

    bool edge_case = false; // Jan 04 to Jan 01
    // month only
    if (type == kYMD && (b_month && e_month) && (!b_day_dow && !e_day_dow && !b_week && !e_week) &&
        b_month == e_month) {

      dt_in_range = (b_month <= unsigned(d.month()) && unsigned(d.month()) <= e_month);

      if (begin_hrs || begin_mins || end_hrs || end_mins) {
        b_td = std::chrono::hours(begin_hrs) + std::chrono::minutes(begin_mins);
        e_td = std::chrono::hours(end_hrs) + std::chrono::minutes(end_mins);
      }

      dt_in_range = (dt_in_range && (b_td <= td && td <= e_td));
      return (dow_in_range && dt_in_range);
    } else if (type == kYMD && b_month && b_day_dow) {

      e_year = int(d.year()), b_year = int(d.year());
      if (b_month == e_month) {
        if (b_day_dow > e_day_dow) { // Mar 15 - Mar 1
          edge_case = true;
        }
      } else if (b_month > e_month) { // Oct 10 - Mar 3
        if (b_month > unsigned(d.month())) {
          b_year = int(d.year()) - 1;
        } else {
          e_year = int(d.year()) + 1;
        }
      }

      begin_date =
          date::year_month_day(date::year(b_year), date::month(b_month), date::day(b_day_dow));
      end_date = date::year_month_day(date::year(e_year), date::month(e_month), date::day(e_day_dow));

    } else if (type == kNthDow && b_month && b_day_dow && e_month &&
               e_day_dow) { // kNthDow types can have a mix of ymd and nthdow. (e.g. Dec Su[-1]-Mar
                            // 3 Sat 15:00-17:00)

      e_year = int(d.year()), b_year = int(d.year());
      if (b_month == e_month) {
        if (b_day_dow > e_day_dow) { // Mar 15 - Mar 1
          edge_case = true;
        }
      } else if (b_month > e_month) { // Oct 10 - Mar 3
        if (b_month > unsigned(d.month())) {
          b_year = int(d.year()) - 1;
        } else {
          e_year = int(d.year()) + 1;
        }
      }

      if (b_week && b_week <= 5) { // kNthDow
        auto ymwd =
            date::year_month_weekday(date::year(b_year), date::month(b_month),
                                     date::weekday_indexed(date::weekday(b_day_dow - 1), b_week));

        if (b_week == 5 && !ymwd.ok()) { // we tried to get the 5th x(e.g., Friday) of some month and
                                         // there are only 4
          b_week--;
          ymwd =
              date::year_month_weekday(date::year(b_year), date::month(b_month),
                                       date::weekday_indexed(date::weekday(b_day_dow - 1), b_week));
        }

        begin_date = date::year_month_day(ymwd);
      } else { // YMD
        begin_date =
            date::year_month_day(date::year(b_year), date::month(b_month), date::day(b_day_dow));
      }

      if (e_week && e_week <= 5) { // kNthDow
        auto ymwd =
            date::year_month_weekday(date::year(e_year), date::month(e_month),
                                     date::weekday_indexed(date::weekday(e_day_dow - 1), e_week));
        if (e_week == 5 && !ymwd.ok()) { // we tried to get the 5th x(e.g., Friday) of some month and
                                         // there are only 4
          e_week--;
          date::year_month_weekday(date::year(e_year), date::month(e_month),
                                   date::weekday_indexed(date::weekday(e_day_dow - 1), e_week));
        }

        end_date = date::year_month_day(ymwd);
      } else { // YMD
        end_date = date::year_month_day(date::year(e_year), date::month(e_month),
                                        date::day(e_day_dow)); // Dec 5 to Mar 3
      }
    } else { // just time or dow with or without time

      if (begin_hrs || begin_mins || end_hrs || end_mins) {
        b_td = std::chrono::hours(begin_hrs) + std::chrono::minutes(begin_mins);
        e_td = std::chrono::hours(end_hrs) + std::chrono::minutes(end_mins);

        if (begin_hrs > end_hrs) { // 19:00 - 06:00
          dt_in_range = !(e_td <= td && td <= b_td);
        } else {
          dt_in_range = (b_td <= td && td <= e_td);
        }
      }
      return (dow_in_range && dt_in_range);
    }

    if (begin_hrs || begin_mins || end_hrs || end_mins) {
      b_td = std::chrono::hours(begin_hrs) + std::chrono::minutes(begin_mins);
      e_td = std::chrono::hours(end_hrs) + std::chrono::minutes(end_mins);
    }

    // Time does not matter here; we are only dealing with dates.
    auto b_in_local_time =
        date::make_zoned(time_zone, date::local_days(begin_date), date::choose::latest);
    auto local_dt = date::make_zoned(time_zone, date::local_days(d), date::choose::latest);
    auto e_in_local_time =
        date::make_zoned(time_zone, date::local_days(end_date), date::choose::latest);

    if (edge_case) {

      // Jan 04 to Jan 02.  We need to test to end of the year and then from the first of the
      // year to the end date.

      // begin date = Jan 04, 2021
      // end date = Jan 02, 2021
      date::year_month_day new_ed =
          date::year_month_day(date::year(b_year), date::month(12), date::day(31));
      auto new_e_in_local_time =
          date::make_zoned(time_zone, date::local_days(new_ed), date::choose::latest);

      date::year_month_day new_bd =
          date::year_month_day(date::year(b_year), date::month(1), date::day(1));
      auto new_b_in_local_time =
          date::make_zoned(time_zone, date::local_days(new_bd), date::choose::latest);

      // we need to check Jan 04, 2021 to Dec 31, 2021 and Jan 01, 2021 to Jan 02, 2021
      dt_in_range = (((b_in_local_time.get_local_time() <= local_dt.get_local_time() &&
                       local_dt.get_local_time() <= new_e_in_local_time.get_local_time())) ||
                     ((new_b_in_local_time.get_local_time() <= local_dt.get_local_time() &&
                       local_dt.get_local_time() <= e_in_local_time.get_local_time())));
    } else {
      dt_in_range = (b_in_local_time.get_local_time() <= local_dt.get_local_time() &&
                     local_dt.get_local_time() <= e_in_local_time.get_local_time());
    }

    bool time_in_range = false;

    if (begin_hrs > end_hrs) { // 19:00 - 06:00
      time_in_range = !(e_td <= td && td <= b_td);
    } else {
      time_in_range = (b_td <= td && td <= e_td);
    }

    dt_in_range = (dt_in_range && time_in_range);
  } catch (std::exception& e) {}
  return (dow_in_range && dt_in_range);
}

uint32_t second_of_week(uint32_t epoch_time, const date::time_zone* time_zone) {
  // get the date time in this timezone
  std::chrono::seconds dur(epoch_time);
  std::chrono::time_point<std::chrono::system_clock> utp(dur);
  const auto tp = date::make_zoned(time_zone, utp).get_local_time();
  // floor to midnight of that day
  auto days = date::floor<date::days>(tp);
  // get the ordinal day of the week
  uint32_t day = (date::year_month_weekday(days).weekday() - date::Sunday).count();
  // subtract midnight from the time to get just the time since midnight
  auto since_midnight =
      std::chrono::duration_cast<std::chrono::seconds>(date::make_time(tp - days).to_duration());
  // get the seconds of the week
  return day * midgard::kSecondsPerDay + since_midnight.count();
}

dt_info_t
offset_date(const std::string& in_dt, const uint32_t in_tz, const uint32_t out_tz, float offset) {
  if (in_dt.empty()) {
    return INVALID_DT;
  } // get the input UTC time, add the offset and translate to the out timezone

  auto iepoch = DateTime::seconds_since_epoch(in_dt, DateTime::get_tz_db().from_index(in_tz));

  auto oepoch =
      static_cast<uint64_t>(static_cast<double>(iepoch) + static_cast<double>(offset + .5f));
  auto tz = DateTime::get_tz_db().from_index(out_tz);
  auto dt = DateTime::seconds_to_date(oepoch, tz, true);

  // dt can be empty if time zones are invalid
  if (dt.empty()) {
    return INVALID_DT;
  };
  return {dt.substr(0, 16), dt.substr(16), tz->name()};
}
} // namespace DateTime
} // namespace baldr
} // namespace valhalla
