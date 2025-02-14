using System;
using System.Collections.Generic;

namespace valhalla.baldr
{
    public class Admin
    {
        private uint country_offset_;
        private uint state_offset_;
        private string country_iso_;
        private string state_iso_;

        private static readonly Dictionary<string, string> iso2_to_iso3 = new Dictionary<string, string>
        {
            {"AD", "AND"}, {"AE", "ARE"}, {"AF", "AFG"}, {"AG", "ATG"}, {"AI", "AIA"}, {"AL", "ALB"},
            {"AM", "ARM"}, {"AO", "AGO"}, {"AQ", "ATA"}, {"AR", "ARG"}, {"AS", "ASM"}, {"AT", "AUT"},
            {"AU", "AUS"}, {"AW", "ABW"}, {"AX", "ALA"}, {"AZ", "AZE"}, {"BA", "BIH"}, {"BB", "BRB"},
            {"BD", "BGD"}, {"BE", "BEL"}, {"BF", "BFA"}, {"BG", "BGR"}, {"BH", "BHR"}, {"BI", "BDI"},
            {"BJ", "BEN"}, {"BL", "BLM"}, {"BM", "BMU"}, {"BN", "BRN"}, {"BO", "BOL"}, {"BQ", "BES"},
            {"BR", "BRA"}, {"BS", "BHS"}, {"BT", "BTN"}, {"BV", "BVT"}, {"BW", "BWA"}, {"BY", "BLR"},
            {"BZ", "BLZ"}, {"CA", "CAN"}, {"CC", "CCK"}, {"CD", "COD"}, {"CF", "CAF"}, {"CG", "COG"},
            {"CH", "CHE"}, {"CI", "CIV"}, {"CK", "COK"}, {"CL", "CHL"}, {"CM", "CMR"}, {"CN", "CHN"},
            {"CO", "COL"}, {"CR", "CRI"}, {"CU", "CUB"}, {"CV", "CPV"}, {"CW", "CUW"}, {"CX", "CXR"},
            {"CY", "CYP"}, {"CZ", "CZE"}, {"DE", "DEU"}, {"DJ", "DJI"}, {"DK", "DNK"}, {"DM", "DMA"},
            {"DO", "DOM"}, {"DZ", "DZA"}, {"EC", "ECU"}, {"EE", "EST"}, {"EG", "EGY"}, {"EH", "ESH"},
            {"ER", "ERI"}, {"ES", "ESP"}, {"ET", "ETH"}, {"FI", "FIN"}, {"FJ", "FJI"}, {"FK", "FLK"},
            {"FM", "FSM"}, {"FO", "FRO"}, {"FR", "FRA"}, {"GA", "GAB"}, {"GB", "GBR"}, {"GD", "GRD"},
            {"GE", "GEO"}, {"GF", "GUF"}, {"GG", "GGY"}, {"GH", "GHA"}, {"GI", "GIB"}, {"GL", "GRL"},
            {"GM", "GMB"}, {"GN", "GIN"}, {"GP", "GLP"}, {"GQ", "GNQ"}, {"GR", "GRC"}, {"GS", "SGS"},
            {"GT", "GTM"}, {"GU", "GUM"}, {"GW", "GNB"}, {"GY", "GUY"}, {"HK", "HKG"}, {"HM", "HMD"},
            {"HN", "HND"}, {"HR", "HRV"}, {"HT", "HTI"}, {"HU", "HUN"}, {"ID", "IDN"}, {"IE", "IRL"},
            {"IL", "ISR"}, {"IM", "IMN"}, {"IN", "IND"}, {"IO", "IOT"}, {"IQ", "IRQ"}, {"IR", "IRN"},
            {"IS", "ISL"}, {"IT", "ITA"}, {"JE", "JEY"}, {"JM", "JAM"}, {"JO", "JOR"}, {"JP", "JPN"},
            {"KE", "KEN"}, {"KG", "KGZ"}, {"KH", "KHM"}, {"KI", "KIR"}, {"KM", "COM"}, {"KN", "KNA"},
            {"KP", "PRK"}, {"KR", "KOR"}, {"XK", "XKX"}, {"KW", "KWT"}, {"KY", "CYM"}, {"KZ", "KAZ"},
            {"LA", "LAO"}, {"LB", "LBN"}, {"LC", "LCA"}, {"LI", "LIE"}, {"LK", "LKA"}, {"LR", "LBR"},
            {"LS", "LSO"}, {"LT", "LTU"}, {"LU", "LUX"}, {"LV", "LVA"}, {"LY", "LBY"}, {"MA", "MAR"},
            {"MC", "MCO"}, {"MD", "MDA"}, {"ME", "MNE"}, {"MF", "MAF"}, {"MG", "MDG"}, {"MH", "MHL"},
            {"MK", "MKD"}, {"ML", "MLI"}, {"MM", "MMR"}, {"MN", "MNG"}, {"MO", "MAC"}, {"MP", "MNP"},
            {"MQ", "MTQ"}, {"MR", "MRT"}, {"MS", "MSR"}, {"MT", "MLT"}, {"MU", "MUS"}, {"MV", "MDV"},
            {"MW", "MWI"}, {"MX", "MEX"}, {"MY", "MYS"}, {"MZ", "MOZ"}, {"NA", "NAM"}, {"NC", "NCL"},
            {"NE", "NER"}, {"NF", "NFK"}, {"NG", "NGA"}, {"NI", "NIC"}, {"NL", "NLD"}, {"NO", "NOR"},
            {"NP", "NPL"}, {"NR", "NRU"}, {"NU", "NIU"}, {"NZ", "NZL"}, {"OM", "OMN"}, {"PA", "PAN"},
            {"PE", "PER"}, {"PF", "PYF"}, {"PG", "PNG"}, {"PH", "PHL"}, {"PK", "PAK"}, {"PL", "POL"},
            {"PM", "SPM"}, {"PN", "PCN"}, {"PR", "PRI"}, {"PS", "PSE"}, {"PT", "PRT"}, {"PW", "PLW"},
            {"PY", "PRY"}, {"QA", "QAT"}, {"RE", "REU"}, {"RO", "ROU"}, {"RS", "SRB"}, {"RU", "RUS"},
            {"RW", "RWA"}, {"SA", "SAU"}, {"SB", "SLB"}, {"SC", "SYC"}, {"SD", "SDN"}, {"SS", "SSD"},
            {"SE", "SWE"}, {"SG", "SGP"}, {"SH", "SHN"}, {"SI", "SVN"}, {"SJ", "SJM"}, {"SK", "SVK"},
            {"SL", "SLE"}, {"SM", "SMR"}, {"SN", "SEN"}, {"SO", "SOM"}, {"SR", "SUR"}, {"ST", "STP"},
            {"SV", "SLV"}, {"SX", "SXM"}, {"SY", "SYR"}, {"SZ", "SWZ"}, {"TC", "TCA"}, {"TD", "TCD"},
            {"TF", "ATF"}, {"TG", "TGO"}, {"TH", "THA"}, {"TJ", "TJK"}, {"TK", "TKL"}, {"TL", "TLS"},
            {"TM", "TKM"}, {"TN", "TUN"}, {"TO", "TON"}, {"TR", "TUR"}, {"TT", "TTO"}, {"TV", "TUV"},
            {"TW", "TWN"}, {"TZ", "TZA"}, {"UA", "UKR"}, {"UG", "UGA"}, {"UM", "UMI"}, {"US", "USA"},
            {"UY", "URY"}, {"UZ", "UZB"}, {"VA", "VAT"}, {"VC", "VCT"}, {"VE", "VEN"}, {"VG", "VGB"},
            {"VI", "VIR"}, {"VN", "VNM"}, {"VU", "VUT"}, {"WF", "WLF"}, {"WS", "WSM"}, {"YE", "YEM"},
            {"YT", "MYT"}, {"ZA", "ZAF"}, {"ZM", "ZMB"}, {"ZW", "ZWE"}, {"CS", "SCG"}, {"AN", "ANT"}
        };

        public Admin(uint country_offset, uint state_offset, string country_iso, string state_iso)
        {
            country_offset_ = country_offset;
            state_offset_ = state_offset;

            if (country_iso.Length == 2)
            {
                country_iso_ = country_iso;
            }
            else
            {
                country_iso_ = string.Empty;
            }

            if (state_iso.Length == 2)
            {
                state_iso_ = state_iso + "\0";
            }
            else if (state_iso.Length == 3)
            {
                state_iso_ = state_iso;
            }
            else
            {
                state_iso_ = string.Empty;
            }
        }

        public uint state_offset()
        {
            return state_offset_;
        }

        public uint country_offset()
        {
            return country_offset_;
        }

        public string country_iso()
        {
            return country_iso_;
        }

        public string state_iso()
        {
            return state_iso_;
        }

        public static string get_iso_3166_1_alpha3(string iso_3166_1_alpha2)
        {
            if (iso2_to_iso3.TryGetValue(iso_3166_1_alpha2, out string iso_3166_1_alpha3))
            {
                return iso_3166_1_alpha3;
            }
            return string.Empty;
        }
    }
}
