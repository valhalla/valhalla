#!/bin/sh

# mm.json: input configuration required to run the service or
# instantiate map matcher factory. It is generated from mm.partial.json
# and Valhalla configuration.
#
# mm.partial.json: it holds map-matching specific configuration.
#
# To generate mm.json:
#
# 1. Clone Valhalla configuration:
#
#     git clone --depth=1 https://github.com/valhalla/conf.git
#
# 2. Install jq at https://stedolan.github.io/jq/
#
# 3. Add mjolnir.hierarchy and costing_options from Valhalla
#    configuration to mm.partial.json to generate mm.json:
#
#     jq -s '.[1].mjolnir.hierarchy = .[0].mjolnir.hierarchy | .[1].costing_options = .[0].costing_options | .[1]' \
#        conf/valhalla.json mm.partial.json > mm.json

git clone --depth=1 https://github.com/valhalla/conf.git
jq -s '.[1].mjolnir.hierarchy = .[0].mjolnir.hierarchy | .[1].costing_options = .[0].costing_options | .[1]' \
   conf/valhalla.json mm.partial.json > mm.json
