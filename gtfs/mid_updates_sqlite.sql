
UPDATE calendar_dates_tmp set service_key = (select service_key from calendar_tmp where calendar_dates_tmp.service_id = calendar_tmp.service_id);

update sqlite_sequence set seq = (select seq from sqlite_sequence where name = 'calendar_tmp') where name = 'cal_dates_tmp';
insert into cal_dates_tmp(service_id) select distinct service_id from calendar_dates_tmp where service_key is NULL;
update calendar_dates_tmp set service_key = (select c.service_key from cal_dates_tmp c where calendar_dates_tmp.service_id = c.service_id) where exists (select c.service_key from cal_dates_tmp c where calendar_dates_tmp.service_id = c.service_id);

UPDATE routes_tmp set agency_key = (select agency_key from agency_tmp a where routes_tmp.agency_id = a.agency_id);

update shapes_tmp set geom = SetSRID(MakePoint(shape_pt_lon, shape_pt_lat),4326);

insert into shape_tmp(shape_id) select distinct shape_id from shapes_tmp;
update shape_tmp set geom = (select SetSRID(MakeLine(geom),4326) FROM shapes_tmp where shapes_tmp.shape_id = shape_tmp.shape_id GROUP BY shape_id);

update shapes_tmp set shape_key = (select shape_key from shape_tmp where shapes_tmp.shape_id = shape_tmp.shape_id);
update trips_tmp set shape_key = (select shape_key from shape_tmp where trips_tmp.shape_id = shape_tmp.shape_id);
update trips_tmp set shape_key = 0 where shape_key = '' or shape_key is null;

update stop_times_tmp set stop_key = (select stop_key from stops_tmp b where b.stop_id = stop_times_tmp.stop_id);
update stops_tmp set parent_station_key = (select stop_key from stops_tmp b where b.location_type=1 and b.stop_id = stops_tmp.parent_station);

update stops_tmp set wheelchair_boarding = 0 where wheelchair_boarding = '' or wheelchair_boarding is null;
update trips_tmp set wheelchair_accessible = 0 where wheelchair_accessible = '' or wheelchair_accessible is null;
update trips_tmp set bikes_allowed = 0 where bikes_allowed = '' or bikes_allowed is null;

update trips_tmp set route_key = (select route_key from routes_tmp where trips_tmp.route_id = routes_tmp.route_id);

update stop_times_tmp set trip_key = (select trip_key from trips_tmp where stop_times_tmp.trip_id = trips_tmp.trip_id);

update transfers_tmp set from_stop_key = (select stop_key from stops_tmp where stop_id = from_stop_id);
update transfers_tmp set to_stop_key = (select stop_key from stops_tmp where stop_id = to_stop_id);

update stop_times_tmp set stop_key = (select b.stop_key from stops_tmp a, stops b where a.onestop_id = b.onestop_id and a.stop_key = stop_times_tmp.stop_key) where exists (select b.stop_key from stops_tmp a, stops b where a.onestop_id = b.onestop_id and a.stop_key = stop_times_tmp.stop_key);
update stop_times_tmp set stop_id = (select b.stop_id from stops_tmp a, stops b where a.onestop_id = b.onestop_id and a.stop_id = stop_times_tmp.stop_id) where exists (select b.stop_id from stops_tmp a, stops b where a.onestop_id = b.onestop_id and a.stop_id = stop_times_tmp.stop_id);

update transfers_tmp set from_stop_key = (select b.stop_key from stops_tmp a, stops b where a.onestop_id = b.onestop_id and a.stop_key = transfers_tmp.from_stop_key) where exists (select b.stop_key from stops_tmp a, stops b where a.onestop_id = b.onestop_id and a.stop_key = transfers_tmp.from_stop_key);
update transfers_tmp set to_stop_key = (select b.stop_key from stops_tmp a, stops b where a.onestop_id = b.onestop_id and a.stop_key = transfers_tmp.to_stop_key) where exists (select b.stop_key from stops_tmp a, stops b where a.onestop_id = b.onestop_id and a.stop_key = transfers_tmp.to_stop_key);
update transfers_tmp set from_stop_id = (select b.stop_id from stops_tmp a, stops b where a.onestop_id = b.onestop_id and a.stop_id = transfers_tmp.from_stop_id) where exists (select b.stop_id from stops_tmp a, stops b where a.onestop_id = b.onestop_id and a.stop_id = transfers_tmp.from_stop_id);
update transfers_tmp set to_stop_id = (select b.stop_id from stops_tmp a, stops b where a.onestop_id = b.onestop_id and a.stop_id = transfers_tmp.to_stop_id) where exists (select b.stop_id from stops_tmp a, stops b where a.onestop_id = b.onestop_id and a.stop_id = transfers_tmp.to_stop_id);

delete from stops_tmp where stops_tmp.stop_key in (select a.stop_key from stops_tmp a, stops b where a.onestop_id = b.onestop_id);

create table s_tmp as select trips_tmp.trip_key, trips_tmp.trip_id, trips_tmp.service_id, route_key, stop_id, stop_key, stop_sequence,arrival_time,departure_time,service_key,trips_tmp.shape_key,sunday,monday,tuesday,wednesday,thursday,friday,saturday,start_date,end_date, 0 as has_subtractions, block_id, CASE WHEN (stop_times_tmp.stop_headsign IS NOT NULL and stop_times_tmp.stop_headsign <> "") THEN stop_times_tmp.stop_headsign ELSE trips_tmp.trip_headsign END AS headsign,wheelchair_accessible,bikes_allowed from trips_tmp, stop_times_tmp, calendar_tmp where trips_tmp.trip_id = stop_times_tmp.trip_id and calendar_tmp.service_id = trips_tmp.service_id order by stop_times_tmp.trip_id, stop_sequence;

create table s_dates_tmp as select trips_tmp.trip_key, trips_tmp.trip_id, trips_tmp.service_id, route_key, stop_id, stop_key, stop_sequence,arrival_time,departure_time,service_key,trips_tmp.shape_key,date as start_date,date as end_date, 0 as has_subtractions, block_id, CASE WHEN (stop_times_tmp.stop_headsign IS NOT NULL and stop_times_tmp.stop_headsign <> "") THEN stop_times_tmp.stop_headsign ELSE trips_tmp.trip_headsign END AS headsign,wheelchair_accessible,bikes_allowed from trips_tmp, stop_times_tmp, calendar_dates_tmp where trips_tmp.trip_id = stop_times_tmp.trip_id and calendar_dates_tmp.service_id = trips_tmp.service_id and exception_type = 1 order by stop_times_tmp.trip_id,start_date, stop_sequence;

VACUUM ANALYZE;
