CREATE TABLE "shape_seq"
(
  seq integer
);
insert into shape_seq(seq) select seq from sqlite_sequence where name = 'shape_tmp';

DROP TABLE IF EXISTS "shapes_tmp";
CREATE TABLE "shapes_tmp"
(
  shape_key integer,
  shape_id text,
  shape_pt_lat double precision,
  shape_pt_lon double precision,
  shape_pt_sequence integer,
  shape_dist_traveled double precision
);

DROP TABLE IF EXISTS "shape_tmp";
CREATE TABLE "shape_tmp"
(
  shape_key integer primary key autoincrement,
  shape_id text
);

insert into shape_tmp(shape_id) values('temp');
update sqlite_sequence set seq = (select seq from shape_seq) where name = 'shape_tmp';
DELETE from shape_tmp;
DROP TABLE "shape_seq";

delete from geometry_columns where f_table_name = "shapes_tmp" or f_table_name = "shape_tmp";
