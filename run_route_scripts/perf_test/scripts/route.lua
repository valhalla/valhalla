-- Perform HTTP-based benchmarking of the route endpoint

local common = require("common")

done = function(summary, latency, requests)
   common.write_result_csv(summary, latency, requests)
end

-- Lancaster, PA to San Diego, CA
lancaster2sd_body = '{"locations":[{"lat":40.0381300,"lon":-76.3056686,"name":"Lancaster"},{"lat":32.7174209,"lon":-117.1627714,"name":"San Diego"}],"costing":"auto","units":"miles"}'

-- Paris to Dover, UK coast (avoids ferries)
paris2london_body = '{"locations":[{"lat":48.841663,"lon":2.286931},{"lat":51.094425,"lon":1.152167}],"costing":"auto","costing_options":{"auto":{"use_ferry":0.0}},"units":"kilometers"}'

wrk.headers["Content-Type"] = "application/x-www-form-urlencoded"

request = function()
   return wrk.format('POST', '/route', nil, lancaster2sd_body)
end
