DROP TABLE IF EXISTS "shapes_tmp";
CREATE TABLE "shapes_tmp"
(
  shape_id text,
  shape_pt_lat double precision,
  shape_pt_lon double precision,
  shape_pt_sequence integer,
  shape_dist_traveled double precision
);

DROP TABLE IF EXISTS "shape_tmp";
CREATE TABLE "shape_tmp"
(
  shape_id text
);
delete from geometry_columns where f_table_name = "shapes_tmp" or f_table_name = "shape_tmp";
