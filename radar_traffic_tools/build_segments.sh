#!/bin/bash

# Remove old database
rm -f ./xds_segments.db*

# Create a temporary directory to store headerless CSVs
TEMP_DIR=./data/headerless_tmp
mkdir -p "$TEMP_DIR"

# Strip headers from all CSVs in ./data/inrix-edges-csv and save them in TEMP_DIR
for file in ./data/inrix-edges-csv/*.csv; do
  base_name=$(basename "$file")
  tail -n +2 "$file" > "$TEMP_DIR/$base_name"
done

# Create a temporary SQL script with all imports
IMPORT_SQL=$(mktemp)
cat <<'EOSQL' > "$IMPORT_SQL"
.echo on
.output stdout
PRAGMA journal_mode = WAL;
PRAGMA synchronous = OFF;
PRAGMA cache_size = 8000000;
PRAGMA temp_store = MEMORY;

DROP TABLE IF EXISTS segments;
CREATE TABLE segments (
  seg_id INTEGER,
  OSMWayIDs TEXT,
  OSMWayDirections TEXT,
  WayStartOffset_m REAL,
  WayEndOffset_m REAL,
  WayStartOffset_percent REAL,
  WayEndOffset_percent REAL
);

.timer on
.mode csv
.separator ","
EOSQL

# Append each import to the SQL script
for file in "$TEMP_DIR"/*.csv; do
  echo ".import '$file' segments" >> "$IMPORT_SQL"
done

# Final optimizations
cat <<'EOSQL' >> "$IMPORT_SQL"
CREATE INDEX idx_seg_id ON segments(seg_id);
ANALYZE;
VACUUM;
SELECT COUNT(*) AS total_rows FROM segments;
EOSQL

# Run everything in one SQLite session
sqlite3 ./xds_segments.db < "$IMPORT_SQL"

# Cleanup
rm "$IMPORT_SQL"
