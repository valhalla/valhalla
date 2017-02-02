DROP TABLE IF EXISTS "agency" CASCADE;
CREATE TABLE "agency"
(
  agency_key integer primary key,
  agency_id text,
  agency_name text,
  agency_url text,
  agency_timezone text,
  agency_lang text,
  agency_phone text
);

DROP TABLE IF EXISTS "stops" CASCADE;
CREATE TABLE "stops"
(
  stop_key integer primary key,
  stop_id text,
  onestop_id text,
  osm_way_id bigint,
  stop_code text,
  stop_name text,
  stop_desc text,
  stop_lat double precision,
  stop_lon double precision,
  zone_id text,
  stop_url text,
  location_type integer,
  parent_station text,
  parent_station_key integer,
  wheelchair_boarding integer
);

DROP TABLE IF EXISTS "routes" CASCADE;
CREATE TABLE "routes"
(
  route_key integer primary key,
  route_id text,
  agency_id text,
  agency_key integer,
  route_short_name text,
  route_long_name text,
  route_desc text,
  route_type integer,
  route_url text,
  route_color text,
  route_text_color text
);

DROP TABLE IF EXISTS "trips" CASCADE;
CREATE TABLE "trips"
(
  trip_key integer primary key,
  route_key integer,
  route_id text,
  service_id text,
  trip_id text,
  trip_headsign text,
  trip_short_name text,
  direction_id text,
  block_id text,
  shape_id text,
  shape_key integer,
  wheelchair_accessible integer,
  bikes_allowed integer
);

DROP TABLE IF EXISTS "stop_times" CASCADE;
CREATE TABLE "stop_times"
(
  stop_times_key integer primary key,
  stop_key integer,
  trip_key integer,
  trip_id text,
  arrival_time text,
  departure_time text,
  stop_id text,
  stop_sequence integer,
  stop_headsign text,
  pickup_type text,
  drop_off_type text,
  shape_dist_traveled double precision
);

DROP TABLE IF EXISTS "calendar" CASCADE;
CREATE TABLE "calendar"
(
  service_key integer primary key,
  service_id text,
  monday integer,
  tuesday integer,
  wednesday integer,
  thursday integer,
  friday integer,
  saturday integer,
  sunday integer,
  start_date text,
  end_date text
);

DROP TABLE IF EXISTS "calendar_dates" CASCADE;
CREATE TABLE "calendar_dates"
(
  service_key integer,
  service_id text,
  date text,
  exception_type integer
);

DROP TABLE IF EXISTS "shapes" CASCADE;
CREATE TABLE "shapes"
(
  shape_key integer,
  shape_id text,
  shape_pt_lat double precision,
  shape_pt_lon double precision,
  shape_pt_sequence integer,
  shape_dist_traveled double precision
);

DROP TABLE IF EXISTS "shape" CASCADE;
CREATE TABLE "shape"
(
  shape_key integer primary key,
  shape_id text
);

DROP TABLE IF EXISTS "transfers" CASCADE;
CREATE TABLE "transfers"
(
  transfer_key integer primary key,
  from_stop_id text,
  from_stop_key integer,
  to_stop_id text,
  to_stop_key integer,
  transfer_type integer,
  min_transfer_time text
);

DROP TABLE IF EXISTS "schedule" CASCADE;
CREATE TABLE "schedule"
(
  origin_stop_key integer,
  dest_stop_key integer,
  trip_key integer,
  route_key integer,
  service_key integer,
  shape_key integer,
  departure_time text,
  arrival_time text,
  start_date text,
  end_date text,
  dow_mask integer,
  has_subtractions integer,
  block_id text,
  headsign text,
  wheelchair_accessible integer,
  bikes_allowed integer
);

DROP TABLE IF EXISTS "agency_tmp" CASCADE;
CREATE TABLE "agency_tmp"
(
  agency_key serial primary key,
  agency_id text,
  agency_name text,
  agency_url text,
  agency_timezone text,
  agency_lang text,
  agency_phone text
);

DROP TABLE IF EXISTS "stops_tmp" CASCADE;
CREATE TABLE "stops_tmp"
(
  stop_key serial primary key,
  stop_id text,
  onestop_id text,
  osm_way_id bigint,
  stop_code text,
  stop_name text,
  stop_desc text,
  stop_lat double precision,
  stop_lon double precision,
  zone_id text,
  stop_url text,
  location_type integer,
  parent_station text,
  parent_station_key integer,
  wheelchair_boarding integer
);

DROP TABLE IF EXISTS "routes_tmp" CASCADE;
CREATE TABLE "routes_tmp"
(
  route_key serial primary key,
  route_id text,
  agency_id text,
  agency_key integer,
  route_short_name text,
  route_long_name text,
  route_desc text,
  route_type integer,
  route_url text,
  route_color text,
  route_text_color text
);

DROP TABLE IF EXISTS "trips_tmp" CASCADE;
CREATE TABLE "trips_tmp"
(
  trip_key serial primary key,
  route_key integer,
  route_id text,
  service_id text,
  trip_id text,
  trip_headsign text,
  trip_short_name text,
  direction_id text,
  block_id text,
  shape_id text,
  shape_key integer,
  wheelchair_accessible integer,
  bikes_allowed integer
);

DROP TABLE IF EXISTS "stop_times_tmp" CASCADE;
CREATE TABLE "stop_times_tmp"
(
  stop_times_key serial primary key,
  stop_key integer,
  trip_key integer,
  trip_id text,
  arrival_time text,
  departure_time text,
  stop_id text,
  stop_sequence integer,
  stop_headsign text,
  pickup_type text,
  drop_off_type text,
  shape_dist_traveled double precision
);

DROP TABLE IF EXISTS "calendar_tmp" CASCADE;
CREATE TABLE "calendar_tmp"
(
  service_key serial primary key,
  service_id text,
  monday integer,
  tuesday integer,
  wednesday integer,
  thursday integer,
  friday integer,
  saturday integer,
  sunday integer,
  start_date text,
  end_date text
);

DROP TABLE IF EXISTS "calendar_dates_tmp" CASCADE;
CREATE TABLE "calendar_dates_tmp"
(
  service_key integer,
  service_id text,
  date text,
  exception_type integer
);

DROP TABLE IF EXISTS "cal_dates_tmp" CASCADE;
CREATE TABLE "cal_dates_tmp"
(
  service_key serial primary key,
  service_id text
);

DROP TABLE IF EXISTS "shapes_tmp" CASCADE;
CREATE TABLE "shapes_tmp"
(
  shape_key integer,
  shape_id text,
  shape_pt_lat double precision,
  shape_pt_lon double precision,
  shape_pt_sequence integer,
  shape_dist_traveled double precision
);

DROP TABLE IF EXISTS "shape_tmp" CASCADE;
CREATE TABLE "shape_tmp"
(
  shape_key serial primary key,
  shape_id text
);

DROP TABLE IF EXISTS "transfers_tmp" CASCADE;
CREATE TABLE "transfers_tmp"
(
  transfer_key serial primary key,
  from_stop_id text,
  from_stop_key integer,
  to_stop_id text,
  to_stop_key integer,
  transfer_type integer,
  min_transfer_time text
);

DROP TABLE IF EXISTS "schedule_tmp" CASCADE;
CREATE TABLE "schedule_tmp"
(
  origin_stop_key integer,
  dest_stop_key integer,
  trip_key integer,
  route_key integer,
  service_key integer,
  shape_key integer,
  departure_time text,
  arrival_time text,
  start_date text,
  end_date text,
  dow_mask integer,
  has_subtractions integer,
  block_id text,
  headsign text,
  wheelchair_accessible integer,
  bikes_allowed integer
);

CREATE INDEX t_trip_id_index ON trips USING btree (trip_id);
CREATE INDEX s_trip_id_index ON stop_times USING btree (trip_id);
CREATE INDEX c_service_id_index ON calendar USING btree (service_id);
CREATE INDEX t_service_id_index ON trips USING btree (service_id);
CREATE INDEX cd_service_id_index ON calendar_dates USING btree (service_id);

SELECT AddGeometryColumn('shapes', 'geom', 4326, 'POINT', 2);
SELECT AddGeometryColumn('shape', 'geom', 4326, 'LINESTRING', 2);

SELECT AddGeometryColumn('shapes_tmp', 'geom', 4326, 'POINT', 2);
SELECT AddGeometryColumn('shape_tmp', 'geom', 4326, 'LINESTRING', 2);

SELECT AddGeometryColumn('stops', 'geom', 4326, 'POINT', 2);
SELECT AddGeometryColumn('stops_tmp', 'geom', 4326, 'POINT', 2);

