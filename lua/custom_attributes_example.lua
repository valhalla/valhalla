-- Example for valhalla_add_custom_attributes
-- Return one table for both directions of the way, or a forward and a backward table relative to
-- the direction of the OSM way
function custom_attributes(tags)
  local score_fwd = tonumber(tags["score:fwd"])
  local score_bwd = tonumber(tags["score:bwd"])
  return { walkability_score = score_fwd }, { walkability_score = score_fwd }
end
