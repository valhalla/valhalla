using System;
using System.Text.Json;

namespace valhalla.baldr
{
    public class GraphTileHeader
    {
        private uint graphid_;
        private uint density_;
        private uint name_quality_;
        private uint speed_quality_;
        private uint exit_quality_;
        private uint has_elevation_;
        private uint has_ext_directededge_;
        private uint nodecount_;
        private uint directededgecount_;
        private uint predictedspeeds_count_;
        private uint spare1_;
        private uint transitioncount_;
        private uint spare3_;
        private uint turnlane_count_;
        private uint spare4_;
        private uint transfercount_;
        private uint spare2_;
        private uint departurecount_;
        private uint stopcount_;
        private uint spare5_;
        private uint routecount_;
        private uint schedulecount_;
        private uint signcount_;
        private uint spare6_;
        private uint access_restriction_count_;
        private uint admincount_;
        private uint spare7_;
        private string version_;

        public GraphTileHeader()
        {
            graphid_ = 0;
            density_ = 0;
            name_quality_ = 0;
            speed_quality_ = 0;
            exit_quality_ = 0;
            has_elevation_ = 0;
            has_ext_directededge_ = 0;
            nodecount_ = 0;
            directededgecount_ = 0;
            predictedspeeds_count_ = 0;
            spare1_ = 0;
            transitioncount_ = 0;
            spare3_ = 0;
            turnlane_count_ = 0;
            spare4_ = 0;
            transfercount_ = 0;
            spare2_ = 0;
            departurecount_ = 0;
            stopcount_ = 0;
            spare5_ = 0;
            routecount_ = 0;
            schedulecount_ = 0;
            signcount_ = 0;
            spare6_ = 0;
            access_restriction_count_ = 0;
            admincount_ = 0;
            spare7_ = 0;
            version_ = string.Empty;
        }

        public void set_version(string version)
        {
            version_ = version.Substring(0, Math.Min(16, version.Length));
        }

        public void set_departurecount(uint departures)
        {
            if (departures > 65535)
            {
                throw new ArgumentException("Exceeding maximum number of transit departures per tile");
            }
            departurecount_ = departures;
        }

        public void set_stopcount(uint stops)
        {
            if (stops > 65535)
            {
                throw new ArgumentException("Exceeding maximum number of transit stops per tile");
            }
            stopcount_ = stops;
        }

        public void set_routecount(uint routes)
        {
            if (routes > 65535)
            {
                throw new ArgumentException("Exceeding maximum number of transit routes per tile");
            }
            routecount_ = routes;
        }

        public void set_schedulecount(uint schedules)
        {
            if (schedules > 65535)
            {
                throw new ArgumentException("Exceeding maximum number of transit schedule entries per tile");
            }
            schedulecount_ = schedules;
        }

        public void set_transfercount(uint transfers)
        {
            if (transfers > 65535)
            {
                throw new ArgumentException("Exceeding maximum number of transit transfer entries per tile");
            }
            transfercount_ = transfers;
        }

        public JsonElement json()
        {
            var headerMap = new
            {
                graphid = graphid_,
                density = density_,
                name_quality = name_quality_,
                speed_quality = speed_quality_,
                exit_quality = exit_quality_,
                has_elevation = has_elevation_,
                has_ext_directededge = has_ext_directededge_,
                nodecount = nodecount_,
                directededgecount = directededgecount_,
                predictedspeeds_count = predictedspeeds_count_,
                spare1 = spare1_,
                transitioncount = transitioncount_,
                spare3 = spare3_,
                turnlane_count = turnlane_count_,
                spare4 = spare4_,
                transfercount = transfercount_,
                spare2 = spare2_,
                departurecount = departurecount_,
                stopcount = stopcount_,
                spare5 = spare5_,
                routecount = routecount_,
                schedulecount = schedulecount_,
                signcount = signcount_,
                spare6 = spare6_,
                access_restriction_count = access_restriction_count_,
                admincount = admincount_,
                spare7 = spare7_,
                version = version_
            };
            return JsonDocument.Parse(JsonSerializer.Serialize(headerMap)).RootElement;
        }
    }
}
