#!/bin/bash

# remove data from previous runs
rm ./ways_to_edges.db*

# separate data by | instead of comma to handle variable column widths

sqlite3 ./ways_to_edges.db <<EOF
-- Turn on echo mode to see commands as they execute
.echo on

-- Enable output mode to show results of queries
.output stdout

-- Enable WAL mode and performance optimizations
PRAGMA journal_mode = WAL;
PRAGMA synchronous = OFF;
PRAGMA cache_size = 8000000;
PRAGMA temp_store = MEMORY;

-- Create the table
CREATE TABLE edges (
  way_id TEXT PRIMARY KEY,
  data TEXT
);

-- Show execution times
.timer on

-- Import the CSV file
.mode csv
.separator ","
.import ./data/ways_to_edges.csv edges

-- Create the index after import for better performance
CREATE INDEX idx_way_id ON edges(way_id);

-- Optimize the database
ANALYZE;
VACUUM;

-- Show row count
SELECT COUNT(*) AS total_rows FROM edges;

EOF
