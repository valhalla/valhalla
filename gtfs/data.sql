copy shapes_tmp(shape_id,shape_pt_lat,shape_pt_lon,shape_pt_sequence,shape_dist_traveled) from '/data/valhalla/mjolnir/gtfs/shapes.txt' with delimiter '|' csv header;
copy agency_tmp(agency_id,agency_name,agency_url,agency_timezone,agency_lang,agency_phone) from '/data/valhalla/mjolnir/gtfs/agency.txt' with delimiter '|' csv header;
copy stops_tmp(stop_id,stop_code,stop_name,stop_desc,stop_lat,stop_lon,zone_id,stop_url,location_type,parent_station) from '/data/valhalla/mjolnir/gtfs/stops.txt' with delimiter '|' csv header;
copy routes_tmp(route_id,agency_id,route_short_name,route_long_name,route_desc,route_type,route_url,route_color,route_text_color) from '/data/valhalla/mjolnir/gtfs/routes.txt' with delimiter '|' csv header;
copy trips_tmp(route_id,service_id,trip_id,trip_headsign,direction_id,block_id,shape_id) from '/data/valhalla/mjolnir/gtfs/trips.txt' with delimiter '|' csv header;
copy stop_times_tmp(trip_id,arrival_time,departure_time,stop_id,stop_sequence,pickup_type,drop_off_type) from '/data/valhalla/mjolnir/gtfs/stop_times.txt' with delimiter '|' csv header;
copy calendar_tmp(service_id,monday,tuesday,wednesday,thursday,friday,saturday,sunday,start_date,end_date) from '/data/valhalla/mjolnir/gtfs/calendar.txt' with delimiter '|' csv header;
copy calendar_dates_tmp(service_id,date,exception_type) from '/data/valhalla/mjolnir/gtfs/calendar_dates.txt' with delimiter '|' csv header;
