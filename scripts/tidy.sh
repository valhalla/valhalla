#!/bin/sh

set -e

# Backup compile_commands and filter out protobuf generated .pb.cc files
mkdir -p tidytmp
cat compile_commands.json | jq '[ .[] | select(.file | endswith(".pb.cc") | not) ]' > tidytmp/compile_commands.json

# Run the tidy tool
run-clang-tidy-5.0.py -p tidytmp -header-filter "^$(pwd)/((src|test|valhalla/(baldr|midgard|sif|odin|thor|skadi|tyr|loki|mjolnir))/.*|(src|test|valhalla)/[^/]+)$" -quiet -fix -format -j 4
