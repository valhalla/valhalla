-- Perform HTTP-based benchmarking of the trace_route endpoint

local common = require("common")

done = function(summary, latency, requests)
   common.write_result_csv(summary, latency, requests)
end

great_highway_body = '{"shape": [{"lon": -122.5075381, "lat": 37.7417249, "type": "break", "radius": 15, "preferred_side": "either", "search_cutoff": 21000000}, {"lon": -122.507566, "lat": 37.7419624, "type": "break", "radius": 15, "preferred_side": "either", "search_cutoff": 21000000}, {"lon": -122.5076124, "lat": 37.7422244, "type": "break", "radius": 15, "preferred_side": "either", "search_cutoff": 21000000}, {"lon": -122.5076527, "lat": 37.742501, "type": "break", "radius": 15, "preferred_side": "either", "search_cutoff": 21000000}], "costing": "auto", "shape_match": "map_snap", "format": "osrm", "shape_format": "polyline6", "filters": {}, "trace_options": {"interpolation_distance": 0}}'

wrk.headers["Content-Type"] = "application/x-www-form-urlencoded"

request = function()
   return wrk.format('POST', '/trace_route', nil, great_highway_body)
end
