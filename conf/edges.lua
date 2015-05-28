--TODO: check if you can use lua type boolean instead of strings and pass that back to osm2pgsql
--with the hopes that they will become strings once they get back to c++ and then just work in
--postgres

highway = {
["motorway"] =          {["auto_forward"] = "true",  ["bus_forward"] = "true",  ["pedestrian"] = "false",  ["bike_forward"] = "false"},
["motorway_link"] =     {["auto_forward"] = "true",  ["bus_forward"] = "true",  ["pedestrian"] = "false",  ["bike_forward"] = "false"},
["trunk"] =             {["auto_forward"] = "true",  ["bus_forward"] = "true",  ["pedestrian"] = "true",   ["bike_forward"] = "true"},
["trunk_link"] =        {["auto_forward"] = "true",  ["bus_forward"] = "true",  ["pedestrian"] = "true",   ["bike_forward"] = "true"},
["primary"] =           {["auto_forward"] = "true",  ["bus_forward"] = "true",  ["pedestrian"] = "true",   ["bike_forward"] = "true"},
["primary_link"] =      {["auto_forward"] = "true",  ["bus_forward"] = "true",  ["pedestrian"] = "true",   ["bike_forward"] = "true"},
["secondary"] =         {["auto_forward"] = "true",  ["bus_forward"] = "true",  ["pedestrian"] = "true",   ["bike_forward"] = "true"},
["secondary_link"] =    {["auto_forward"] = "true",  ["bus_forward"] = "true",  ["pedestrian"] = "true",   ["bike_forward"] = "true"},
["residential"] =       {["auto_forward"] = "true",  ["bus_forward"] = "true",  ["pedestrian"] = "true",   ["bike_forward"] = "true"},
["residential_link"] =  {["auto_forward"] = "true",  ["bus_forward"] = "true",  ["pedestrian"] = "true",   ["bike_forward"] = "true"},
["service"] =           {["auto_forward"] = "true",  ["bus_forward"] = "true",  ["pedestrian"] = "true",   ["bike_forward"] = "true"},
["tertiary"] =          {["auto_forward"] = "true",  ["bus_forward"] = "true",  ["pedestrian"] = "true",   ["bike_forward"] = "true"},
["tertiary_link"] =     {["auto_forward"] = "true",  ["bus_forward"] = "true",  ["pedestrian"] = "true",   ["bike_forward"] = "true"},
["road"] =              {["auto_forward"] = "true",  ["bus_forward"] = "true",  ["pedestrian"] = "true",   ["bike_forward"] = "true"},
["track"] =             {["auto_forward"] = "true",  ["bus_forward"] = "true",  ["pedestrian"] = "true",   ["bike_forward"] = "true"},
["unclassified"] =      {["auto_forward"] = "true",  ["bus_forward"] = "true",  ["pedestrian"] = "true",   ["bike_forward"] = "true"},
["undefined"] =         {["auto_forward"] = "false", ["bus_forward"] = "false", ["pedestrian"] = "false",  ["bike_forward"] = "false"},
["unknown"] =           {["auto_forward"] = "false", ["bus_forward"] = "false", ["pedestrian"] = "false",  ["bike_forward"] = "false"},
["living_street"] =     {["auto_forward"] = "true",  ["bus_forward"] = "true",  ["pedestrian"] = "true",   ["bike_forward"] = "true"},
["footway"] =           {["auto_forward"] = "false", ["bus_forward"] = "false", ["pedestrian"] = "true",   ["bike_forward"] = "false"},
["pedestrian"] =        {["auto_forward"] = "false", ["bus_forward"] = "false", ["pedestrian"] = "true",   ["bike_forward"] = "false"},
["steps"] =             {["auto_forward"] = "false", ["bus_forward"] = "false", ["pedestrian"] = "true",   ["bike_forward"] = "true"},
["bridleway"] =         {["auto_forward"] = "false", ["bus_forward"] = "false", ["pedestrian"] = "false",  ["bike_forward"] = "false"},
["construction"] =      {["auto_forward"] = "false", ["bus_forward"] = "false", ["pedestrian"] = "false",  ["bike_forward"] = "false"},
["cycleway"] =          {["auto_forward"] = "false", ["bus_forward"] = "false", ["pedestrian"] = "false",  ["bike_forward"] = "true"},
["path"] =              {["auto_forward"] = "false", ["bus_forward"] = "false", ["pedestrian"] = "true",   ["bike_forward"] = "true"},
["bus_guideway"] =      {["auto_forward"] = "false", ["bus_forward"] = "true", 	["pedestrian"] = "false",  ["bike_forward"] = "false"}
}

road_class = {
["motorway"] = 0, 
["motorway_link"] = 0,
["trunk"] = 1,
["trunk_link"] = 1,
["primary"] = 2,
["primary_link"] = 2,
["secondary"] = 3,
["secondary_link"] = 3,
["tertiary"] = 4, 
["tertiary_link"] = 4, 
["unclassified"] = 5, 
["residential"] = 6, 
["residential_link"] = 6
}

restriction = {
["no_left_turn"] = 0,
["no_right_turn"] = 1,
["no_straight_on"] = 2,
["no_u_turn"] = 3,
["only_right_turn"] = 4,
["only_left_turn"] = 5,
["only_straight_on"] = 6
}

dow = {
["Sunday"] = 1,
["sunday"] = 1,
["Sun"] = 1,
["sun"] = 1,
["Su"] = 1,
["su"] = 1,
["Monday"] = 2,
["monday"] = 2,
["Mon"] = 2,
["mon"] = 2,
["Mo"] = 2,
["mo"] = 2,
["Tuesday"] = 3,
["tuesday"] = 3,
["Tues"] = 3,
["tues"] = 3,
["Tue"] = 3,
["tue"] = 3,
["Tu"] = 3,
["tu"] = 3,
["Wednesday"] = 4,
["wednesday"] = 4,
["Weds"] = 4,
["weds"] = 4,
["Wed"] = 4,
["wed"] = 4,
["We"] = 4,
["we"] = 4,
["Thursday"] = 5,
["thursday"] = 5,
["Thurs"] = 5,
["thurs"] = 5,
["Thur"] = 5,
["thur"] = 5,
["Th"] = 5,
["th"] = 5,
["Friday"] = 6,
["friday"] = 6,
["Fri"] = 6,
["fri"] = 6,
["Fr"] = 6,
["fr"] = 6,
["Saturday"] = 7,
["saturday"] = 7,
["Sat"] = 7,
["sat"] = 7,
["Sa"] = 7,
["sa"] = 7
}

default_speed = {
[0] = 105,
[1] = 90,
[2] = 75,
[3] = 60,
[4] = 50,
[5] = 40,
[6] = 30,
[7] = 20
}

access = {
["no"] = "false",
["official"] = "false",
["private"] = "false",
["destination"] = "false",
["yes"] = "true",
["permissive"] = "true",
["agricultural"] = "false",
["customers"] = "true"
}

private = {
["private"] = "true"
}

no_thru_traffic = {
["destination"] = "true",
["customers"] = "true"
}

use = {
["driveway"] = 4,
["alley"] = 5,
["parking_aisle"] = 6,
["emergency_access"] = 7,
["drive-through"] = 8
}

motor_vehicle = {
["no"] = "false",
["yes"] = "true",
["agricultural"] = "false",
["destination"] = "false",
["private"] = "false",
["forestry"] = "false",
["designated"] = "true",
["permissive"] = "true"
}

foot = {
["no"] = "false",
["yes"] = "true",
["designated"] = "true",
["permissive"] = "true",
["crossing"] = "true"
}

bus = {
["no"] = "false",
["yes"] = "true",
["designated"] = "true",
["permissive"] = "true",
["restricted"] = "true",
["destination"] = "false",
["delivery"] = "false"
}

psv = {
["bus"] = "true",
["no"] = "false",
["yes"] = "true",
["1"] = "true",
["2"] = "true"
}

bicycle = {
["yes"] = "true",
["designated"] = "true",
["dismount"] = "true",
["no"] = "false",
["lane"] = "true",
["track"] = "true",
["shared"] = "true",
["shared_lane"] = "true",
["sidepath"] = "true",
["share_busway"] = "true",
["none"] = "false"
}

bike_reverse = {
["opposite"] = "true",
["opposite_lane"] = "true",
["opposite_track"] = "true"
}

bus_reverse = {
["opposite"] = "true",
["opposite_lane"] = "true"
}

shared = {
["shared_lane"] = 1,
["share_busway"] = 1,
["shared"] = 1
}

dedicated = {
["opposite_track"] = 2,
["track"] = 2
}

separated = {
["opposite_lane"] = 3,
["lane"] = 3
}

oneway = {
["no"] = "false",
["-1"] = "true",
["yes"] = "true",
["true"] = "true",
["1"] = "true"
}

bridge = {
["yes"] = "true",
["no"] = "false",
["1"] = "true"
}

--TODO: building_passage is for ped only
tunnel = {
["yes"] = "true",
["no"] = "false",
["1"] = "true",
["building_passage"] = "true"
}

--TODO: snowmobile might not really be passable for much other than ped..
toll = {
["yes"] = "true",
["no"] = "false",
["true"] = "true",
["false"] = "false",
["1"] = "true",
["interval"] = "true",
["snowmobile"] = "true"
}

--convert the numeric (non negative) number portion at the beginning of the string
function numeric_prefix(num_str)
  --not a string
  if num_str == nil then
    return nil
  end

  --find where the numbers stop
  local index = 0
  for c in num_str:gmatch"." do
    if tonumber(c) == nil then
      break
    end
    index = index + 1
  end

  --there weren't any numbers
  if index == 0 then
    return nil
  end

  --convert number portion of string to actual numeric type
  return tonumber(num_str:sub(0, index))
end

--normalize a speed value
function normalize_speed(speed)
  --grab the number prefix
  local num = numeric_prefix(speed)

  --check if the rest of the string ends in "mph" convert to kph
  if num and speed:sub(-3) == "mph" then
    num = num * 1.609344
  end

  return num
end

--returns 1 if you should filter this way 0 otherwise
function filter_tags_generic(kv)
  --figure out what basic type of road it is
  local forward = highway[kv["highway"]]
  local ferry = kv["route"] == "ferry"
  local access = access[kv["access"]]
  if forward then
    for k,v in pairs(forward) do
      kv[k] = v
    end

    if access == "false" then
      kv["auto_forward"] = "false"
      kv["bus_forward"] = "false"
      kv["pedestrian"] = "false"
      kv["bike_forward"] = "false"

      kv["auto_backward"] = "false"
      kv["bus_backward"] = "false"
      kv["bike_backward"] = "false"
    end  

    --check for auto_forward overrides
    kv["auto_forward"] = motor_vehicle[kv["motorcar"]] or motor_vehicle[kv["motor_vehicle"]] or kv["auto_forward"]

    --check for bus_forward overrides
    kv["bus_forward"] = bus[kv["bus"]] or psv[kv["psv"]] or psv[kv["lanes:psv:forward"]] or kv["bus_forward"]

    --check for ped overrides
    kv["pedestrian"] = foot[kv["foot"]] or foot[kv["pedestrian"]] or kv["pedestrian"] 

    --check for bike_forward overrides
    kv["bike_forward"] = bicycle[kv["bicycle"]] or bicycle[kv["cycleway"]] or kv["bike_forward"]

  else
    --if its a ferry and these tags dont show up we want to set them to true 
    local default_val = tostring(ferry)

    if access == "false" then
      kv["auto_forward"] = "false"
      kv["bus_forward"] = "false"
      kv["pedestrian"] = "false"
      kv["bike_forward"] = "false"

      kv["auto_backward"] = "false"
      kv["bus_backward"] = "false"
      kv["bike_backward"] = "false"
    end      

    --check for auto_forward overrides
    kv["auto_forward"] = motor_vehicle[kv["motorcar"]] or motor_vehicle[kv["motor_vehicle"]] or default_val

    --check for bus_forward overrides
    kv["bus_forward"] = bus[kv["bus"]] or psv[kv["psv"]] or psv[kv["lanes:psv:forward"]] or default_val

    --check for ped overrides
    kv["pedestrian"] = foot[kv["foot"]] or foot[kv["pedestrian"]] or default_val

    --check for bike_forward overrides
    kv["bike_forward"] = bicycle[kv["bicycle"]] or bicycle[kv["cycleway"]] or default_val
  end

  --TODO: handle Time conditional restrictions if available for HOVs with oneway = reversible
  if ((kv["access"] == "permissive" or kv["access"] == "hov") and kv["oneway"] == "reversible") then
    return 1
  end

  --service=driveway means all are routable
  if kv["service"] == "driveway" and kv["access"] == nil then
    kv["auto_forward"] = "true"
    kv["bus_forward"] = "true"
    kv["pedestrian"] = "true"
    kv["bike_forward"] = "true"
  end

  --check the oneway-ness and traversability against the direction of the geom
  kv["bike_backward"] = bike_reverse[kv["cycleway"]] or bike_reverse[kv["cycleway:left"]] or bike_reverse[kv["cycleway:right"]] or "false"

  if kv["bike_backward"] == "true" then
    oneway_bike = oneway[kv["oneway:bicycle"]]
  end

  kv["bus_backward"] = bus_reverse[kv["busway"]] or psv[kv["lanes:psv:backward"]] or "false"

  if kv["bus_backward"] == "true" then
    oneway_bus = oneway[kv["oneway:bus"]]
  end

  local oneway_reverse = kv["oneway"]
  local oneway_norm = oneway[kv["oneway"]]
  if kv["junction"] == "roundabout" then
    oneway_norm = "true"
    kv["roundabout"] = "true"
  else
    kv["roundabout"] = "false"
  end
  kv["oneway"] = oneway_norm
  if oneway_norm == "true" then
    kv["auto_backward"] = "false"
    
    if kv["bike_backward"] == "true" then 
      if (oneway_bike == nil or oneway_bike == "true") then --bike only in reverse on a bike path.
        kv["bike_forward"] = "false"
      elseif oneway_bike == "false" then --bike in both directions on a bike path.
        kv["bike_forward"] = "true"
      end
    end
    if kv["bus_backward"] == "true" then
      if (oneway_bus == nil or oneway_bus == "true") then --bus only in reverse on a bus path.
        kv["bus_forward"] = "false"
      elseif oneway_bus == "false" then --bus in both directions on a bus path.
        kv["bus_forward"] = "true"
      end
    end

  elseif oneway_norm == nil or oneway_norm == "false" then
    kv["auto_backward"] = kv["auto_forward"]
    if kv["bike_backward"] == "false" then
      kv["bike_backward"] = kv["bike_forward"]
    end

    if kv["bus_backward"] == "false" then
      kv["bus_backward"] = kv["bus_forward"]
    end
  end

  --Bike forward / backward overrides.
  if ((shared[kv["cycleway:both"]] or separated[kv["cycleway:both"]] or dedicated[kv["cycleway:both"]]) or
      ((shared[kv["cycleway:right"]] or separated[kv["cycleway:right"]] or dedicated[kv["cycleway:right"]]) and 
       (shared[kv["cycleway:left"]] or separated[kv["cycleway:left"]] or dedicated[kv["cycleway:left"]]))) then
    kv["bike_forward"] = "true"
    kv["bike_backward"] = "true"
  end

  if (kv["busway"] == "lane" or (kv["busway:left"] == "lane" and kv["busway:right"] == "lane")) then
    kv["bus_forward"] = "true"
    kv["bus_backward"] = "true"
  end

  --flip the onewayness 
  if oneway_reverse == "-1" then
    local forwards = kv["auto_forward"]
    kv["auto_forward"] = kv["auto_backward"]
    kv["auto_backward"] = forwards

    forwards = kv["bus_forward"]
    kv["bus_forward"] = kv["bus_backward"]
    kv["bus_backward"] = forwards

    forwards = kv["bike_forward"]
    kv["bike_forward"] = kv["bike_backward"]
    kv["bike_backward"] = forwards
  end

  -- bus only logic
  if kv["lanes:bus"] == "1" then
    kv["auto_forward"] = "false"
    kv["bus_forward"] = "true"
    kv["pedestrian"] = "false"
    kv["bike_forward"] = "false"

    kv["auto_backward"] = "false"
    kv["bus_backward"] = "false"
    kv["bike_backward"] = "false"
  elseif kv["lanes:bus"] == "2" then  
    kv["auto_forward"] = "false"
    kv["bus_forward"] = "true"
    kv["pedestrian"] = "false"
    kv["bike_forward"] = "false"

    kv["auto_backward"] = "false"
    kv["bus_backward"] = "true"
    kv["bike_backward"] = "false"
  end 


  --if none of the modes were set we are done looking at this junker
  if kv["auto_forward"] == "false" and kv["bus_forward"] == "false" and kv["bike_forward"] == "false" and kv["auto_backward"] == "false" and kv["bus_backward"] == "false" and kv["bike_backward"] == "false" and kv["pedestrian"] == "false" then
    return 1
  end

   delete_tags = { 'FIXME', 'note', 'source' }

   for i,k in ipairs(delete_tags) do
      kv[k] = nil
   end

  --set a few flags
  local road_class = road_class[kv["highway"]]

  if kv["highway"] == nil and ferry then
    road_class = 2 --TODO:  can we weight based on ferry types?
  elseif kv["highway"] == nil and kv["railway"] then
    road_class = 2 --TODO:  can we weight based on rail types?    
  elseif road_class == nil then --service and other = 7
    road_class = 7
  end 
  
  kv["road_class"] = road_class

  kv["default_speed"] = default_speed[kv["road_class"]]

  local use = use[kv["service"]]

  if kv["highway"] == "steps" then
    use = 26 --steps/stairs
  elseif kv["highway"] == "track" then
    use = 3 
  elseif kv["highway"] == nil then 
    use = 0
  elseif kv["highway"] then
    --favor bicycles
    if kv["highway"] == "cycleway" then
        use = 20
    elseif kv["pedestrian"] == "false" and kv["auto_forward"] == "false" and kv["auto_backward"] == "false" and (kv["bike_forward"] == "true" or kv["bike_backward"] == "true") then
       use = 20
    --favor pedestrians
    elseif kv["highway"] == "footway" or kv["highway"] == "pedestrian" then 
       use = 25
    elseif kv["pedestrian"] == "true" and kv["auto_forward"] == "false" and kv["auto_backward"] == "false" and kv["bike_forward"] == "false" and kv["bike_backward"] == "false" then
       use = 25
    end
  elseif use == nil and kv["service"] then
    use = 40 --other
  else 
    use = 0 --general road, no special use
  end

  if kv["access"] == "emergency" or kv["emergency"] == "yes" then
    use = 7
  end

  kv["use"] = use

  local cycle_lane = shared[kv["cycleway"]] or separated[kv["cycleway"]] or dedicated[kv["cycleway"]] or 0

  if cycle_lane == 0 then
    cycle_lane = shared[kv["cycleway:right"]] or separated[kv["cycleway:right"]] or dedicated[kv["cycleway:right"]] or 0

    if cycle_lane == 0 then
      cycle_lane = shared[kv["cycleway:left"]] or separated[kv["cycleway:left"]] or dedicated[kv["cycleway:left"]] or 0
    end
  end

  kv["cycle_lane"] = cycle_lane

  if kv["highway"] and string.find(kv["highway"], "_link") then --*_link 
     kv["link"] = "true"  --do we need to add more?  turnlane?
  end

  kv["private"] = private[kv["access"]] or "false"
  kv["no_thru_traffic"] = no_thru_traffic[kv["access"]] or "false"
  kv["ferry"] = tostring(ferry)
  kv["rail"] = tostring(kv["auto_forward"] == "true" and kv["railway"] == "rail")
  kv["name"] = kv["name"]
  kv["name:en"] = kv["name:en"]
  kv["alt_name"] = kv["alt_name"]
  kv["official_name"] = kv["official_name"]
  kv["speed"] = normalize_speed(kv["maxspeed"])
  kv["int"] = kv["int"]
  kv["int_ref"] = kv["int_ref"]
  kv["surface"] = kv["surface"]

  --use unsigned_ref if all the conditions are met.
  if ((kv["name"] == nil and kv["name:en"] == nil and kv["alt_name"] == nil and kv["official_name"] == nil and kv["ref"] == nil and kv["int_ref"] == nil) and
      (kv["highway"] == "motorway" or kv["highway"] == "trunk" or kv["highway"] == "primary") and kv["unsigned_ref"] ~= nil) then
        kv["ref"] = kv["unsigned_ref"]
  end

  lane_count = numeric_prefix(kv["lanes"])
  if lane_count and lane_count > 10 then
    lane_count = 10
  end
  kv["lanes"] = lane_count
  kv["bridge"] = bridge[kv["bridge"]] or "false"
  
  -- TODO access:conditional
  if kv["seasonal"] and kv["seasonal"] ~= "no" then
    kv["seasonal"] = "true"
  end

  -- TODO access
  if ((kv["hov"] and kv["hov"] ~= "no") or kv["hov:lanes"] or kv["hov:minimum"]) then
    kv["hov"] = "true"
  end

  kv["tunnel"] = tunnel[kv["tunnel"]] or "false"
  kv["toll"] = toll[kv["toll"]] or "false"
  kv["destination"] = kv["destination"]
  kv["destination:ref"] = kv["destination:ref"]
  kv["destination:ref:to"] = kv["destination:ref:to"]
  kv["destination:street"] = kv["destination:street"]
  kv["destination:street:to"] = kv["destination:street:to"]
  kv["junction:ref"] = kv["junction:ref"]

  local nref = kv["ncn_ref"]
  local rref = kv["rcn_ref"]
  local lref = kv["lcn_ref"]
  local bike_mask = 0
  if nref or kv["ncn"] == "yes" then
    bike_mask = 1
  end
  if rref or kv["rcn"] == "yes" then
    bike_mask = bit32.bor(bike_mask, 2)
  end
  if lref or kv["lcn"] == "yes" then
    bike_mask = bit32.bor(bike_mask, 4)
  end
  if kv["mtb"] == "yes" then
    bike_mask = bit32.bor(bike_mask, 8)
  end

  kv["bike_national_ref"] = nref
  kv["bike_regional_ref"] = rref
  kv["bike_local_ref"] = lref
  kv["bike_network_mask"] = bike_mask

  return 0
end

function nodes_proc (keyvalues, nokeys)
  --we dont care about nodes at all so filter all of them
  return 1, keyvalues
end

function ways_proc (kv, nokeys)
  --if there were no tags passed in, ie keyvalues is empty
  if nokeys == 0 then
    return 1, kv, 0, 0
  end

  --does it at least have some interesting tags
  filter = filter_tags_generic(kv)

  --let the caller know if its a keeper or not and give back the  modified tags
  --also tell it whether or not its a polygon or road
  return filter, kv, 0, 0
end

function rels_proc (kv, nokeys)
  if (kv["type"] == "route" or kv["type"] == "restriction") then

     local restrict = restriction[kv["restriction"]]

     if kv["type"] == "restriction" then

       if restrict ~= nil then
         kv["restriction"] = restrict

         if kv["day_on"] or kv["day_off"] then

           local day_on = dow[kv["day_on"]]
           if day_on == nil then
             kv["day_on"] = 0
           else
             kv["day_on"] = day_on
           end

           local day_off = dow[kv["day_off"]]
           if day_off == nil then
             kv["day_off"] = 0
           else
             kv["day_off"] = day_off
           end
         end
       else
         return 1, kv
       end
       return 0, kv
  --has a restiction but type is not restriction...ignore
     elseif restrict ~= nil then
       return 1, kv
     else
       kv["day_on"] = nil
       kv["day_off"] = nil
       kv["restriction"] = nil
       return 0, kv
     end
  end

  return 1, kv
end

function rel_members_proc (keyvalues, keyvaluemembers, roles, membercount)
  --because we filter all rels we never call this function
  --because we do rel processing later we simply say that no ways are used
  --in the given relation, what would be nice is if we could push tags
  --back to the ways via keyvaluemembers, we could then avoid doing
  --post processing to get the shielding and directional highway info
  membersuperseeded = {}
  for i = 1, membercount do
    membersuperseeded[i] = 0
  end

  return 1, keyvalues, membersuperseeded, 0, 0, 0
end
