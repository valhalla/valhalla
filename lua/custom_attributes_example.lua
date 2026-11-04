-- Example for valhalla_add_custom_attributes --tag-keys score
function custom_attributes(tags)
  local score = tonumber(tags["score"])
  if score == nil then
    return nil
  end
  return { riser_score = score }
end
