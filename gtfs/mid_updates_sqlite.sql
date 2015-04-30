UPDATE routes_tmp set agency_key = (select agency_key from agency_tmp a where routes_tmp.agency_id = a.agency_id);

update shapes_tmp set geom = SetSRID(MakePoint(shape_pt_lon, shape_pt_lat),4326);

insert into shape_tmp(shape_id) select distinct shape_id from shapes_tmp;
update shape_tmp set geom = (select SetSRID(MakeLine(geom),4326) FROM shapes_tmp where shapes_tmp.shape_id = shape_tmp.shape_id GROUP BY shape_id);

update stop_times_tmp set stop_key = (select stop_key from stops_tmp b where b.stop_id = stop_times_tmp.stop_id);
update stops_tmp set parent_station_key = (select stop_key from stops_tmp b where b.location_type=1 and b.stop_id = stops_tmp.parent_station);

update trips_tmp set route_key = (select route_key from routes_tmp where trips_tmp.route_id = routes_tmp.route_id);

update stop_times_tmp set trip_key = (select trip_key from trips_tmp where stop_times_tmp.trip_id = trips_tmp.trip_id);

update transfers_tmp set from_stop_key = (select stop_key from stops_tmp where stop_id = from_stop_id);
update transfers_tmp set to_stop_key = (select stop_key from stops_tmp where stop_id = to_stop_id);

create table s_tmp as select trips_tmp.trip_key, trips_tmp.trip_id, trips_tmp.service_id, route_key, stop_id, stop_key, stop_sequence, departure_time, arrival_time,service_key,sunday,monday,tuesday,wednesday,thursday,friday,saturday,start_date,end_date, CASE WHEN stop_times_tmp.stop_headsign IS NOT NULL THEN stop_times_tmp.stop_headsign ELSE trips_tmp.trip_headsign END AS headsign from trips_tmp, stop_times_tmp, calendar_tmp where trips_tmp.trip_id = stop_times_tmp.trip_id and calendar_tmp.service_id = trips_tmp.service_id order by stop_times_tmp.trip_id, stop_sequence;

VACUUM ANALYZE;
