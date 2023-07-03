-- Shared library utilities

local common = {}

-- Shared table output for benchmarking results. Writes to stderr so report
-- output can be easily separated out for analysis and plotting
-- NOTE(mookerji): Edit with care, otherwise columns will become misaligned.
function common.write_result_csv (summary, latency, requests)
   -- Write header row
   io.stderr:write("latency.min(usec),")
   io.stderr:write("latency.max(usec),")
   io.stderr:write("latency.mean(usec),")
   io.stderr:write("latency.stdev(usec),")
   io.stderr:write("latency:percentile(50.0)(usec),")
   io.stderr:write("latency:percentile(90.0)(usec),")
   io.stderr:write("latency:percentile(99.0)(usec),")
   io.stderr:write("latency:percentile(99.999)(usec),")
   io.stderr:write("summary.duration(usec),")
   io.stderr:write("summary.requests,")
   io.stderr:write("summary.errors.connect,")
   io.stderr:write("summary.errors.read,")
   io.stderr:write("summary.errors.write,")
   io.stderr:write("summary.errors.status,")
   io.stderr:write("summary.errors.timeout,")
   io.stderr:write("requests.min,")
   io.stderr:write("requests.max,")
   io.stderr:write("requests.mean,")
   io.stderr:write("requests.stdev,")
   io.stderr:write("requests:percentile(99.0)\n")
   -- Write row
   io.stderr:write(string.format("%d,",latency.min))
   io.stderr:write(string.format("%d,",latency.max))
   io.stderr:write(string.format("%d,",latency.mean))
   io.stderr:write(string.format("%d,",latency.stdev))
   io.stderr:write(string.format("%d,",latency:percentile(50.0)))
   io.stderr:write(string.format("%d,",latency:percentile(90.0)))
   io.stderr:write(string.format("%d,",latency:percentile(99.0)))
   io.stderr:write(string.format("%d,",latency:percentile(99.999)))
   io.stderr:write(string.format("%d,",summary.duration))
   io.stderr:write(string.format("%d,",summary.requests))
   io.stderr:write(string.format("%d,",summary.errors.connect))
   io.stderr:write(string.format("%d,",summary.errors.read))
   io.stderr:write(string.format("%d,",summary.errors.write))
   io.stderr:write(string.format("%d,",summary.errors.status))
   io.stderr:write(string.format("%d,",summary.errors.timeout))
   io.stderr:write(string.format("%d,",requests.min))
   io.stderr:write(string.format("%d,",requests.max))
   io.stderr:write(string.format("%d,",requests.mean))
   io.stderr:write(string.format("%d,",requests.stdev))
   io.stderr:write(string.format("%d\n",requests:percentile(99.0)))
end

function common.print_table(tab)
   for k, v in pairs(tab) do
      print(k, v)
   end
end

-- Parse argument list to argument map
function common.parse_args (args)
   local arguments = {}
   local token = ''
   for i, line in ipairs(args) do
      if i % 2 == 1 then
         token = line
      else
         arguments[token] = line
      end
   end
   return arguments
end

return common
