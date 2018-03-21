--TODO: check if you can use lua type boolean instead of strings and pass that back to osm2pgsql
--with the hopes that they will become strings once they get back to c++ and then just work in
--postgres

highway = {
["motorway"] =          {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["moped_forward"] = "false", ["pedestrian"] = "false", ["bike_forward"] = "false"},
["motorway_link"] =     {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["moped_forward"] = "false", ["pedestrian"] = "false", ["bike_forward"] = "false"},
["trunk"] =             {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["moped_forward"] = "true",  ["pedestrian"] = "true",  ["bike_forward"] = "true"},
["trunk_link"] =        {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["moped_forward"] = "true",  ["pedestrian"] = "true",  ["bike_forward"] = "true"},
["primary"] =           {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["moped_forward"] = "true",  ["pedestrian"] = "true",  ["bike_forward"] = "true"},
["primary_link"] =      {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["moped_forward"] = "true",  ["pedestrian"] = "true",  ["bike_forward"] = "true"},
["secondary"] =         {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["moped_forward"] = "true",  ["pedestrian"] = "true",  ["bike_forward"] = "true"},
["secondary_link"] =    {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["moped_forward"] = "true",  ["pedestrian"] = "true",  ["bike_forward"] = "true"},
["residential"] =       {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["moped_forward"] = "true",  ["pedestrian"] = "true",  ["bike_forward"] = "true"},
["residential_link"] =  {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["moped_forward"] = "true",  ["pedestrian"] = "true",  ["bike_forward"] = "true"},
["service"] =           {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["moped_forward"] = "true",  ["pedestrian"] = "true",  ["bike_forward"] = "true"},
["tertiary"] =          {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["moped_forward"] = "true",  ["pedestrian"] = "true",  ["bike_forward"] = "true"},
["tertiary_link"] =     {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["moped_forward"] = "true",  ["pedestrian"] = "true",  ["bike_forward"] = "true"},
["road"] =              {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["moped_forward"] = "true",  ["pedestrian"] = "true",  ["bike_forward"] = "true"},
["track"] =             {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["moped_forward"] = "true",  ["pedestrian"] = "true",  ["bike_forward"] = "true"},
["unclassified"] =      {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["moped_forward"] = "true",  ["pedestrian"] = "true",  ["bike_forward"] = "true"},
["undefined"] =         {["auto_forward"] = "false", ["truck_forward"] = "false", ["bus_forward"] = "false", ["moped_forward"] = "false", ["pedestrian"] = "false", ["bike_forward"] = "false"},
["unknown"] =           {["auto_forward"] = "false", ["truck_forward"] = "false", ["bus_forward"] = "false", ["moped_forward"] = "false", ["pedestrian"] = "false", ["bike_forward"] = "false"},
["living_street"] =     {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["moped_forward"] = "true",  ["pedestrian"] = "true",  ["bike_forward"] = "true"},
["footway"] =           {["auto_forward"] = "false", ["truck_forward"] = "false", ["bus_forward"] = "false", ["moped_forward"] = "false", ["pedestrian"] = "true",  ["bike_forward"] = "false"},
["pedestrian"] =        {["auto_forward"] = "false", ["truck_forward"] = "false", ["bus_forward"] = "false", ["moped_forward"] = "false", ["pedestrian"] = "true",  ["bike_forward"] = "false"},
["steps"] =             {["auto_forward"] = "false", ["truck_forward"] = "false", ["bus_forward"] = "false", ["moped_forward"] = "false", ["pedestrian"] = "true",  ["bike_forward"] = "true"},
["bridleway"] =         {["auto_forward"] = "false", ["truck_forward"] = "false", ["bus_forward"] = "false", ["moped_forward"] = "false", ["pedestrian"] = "false", ["bike_forward"] = "false"},
["construction"] =      {["auto_forward"] = "false", ["truck_forward"] = "false", ["bus_forward"] = "false", ["moped_forward"] = "false", ["pedestrian"] = "false", ["bike_forward"] = "false"},
["cycleway"] =          {["auto_forward"] = "false", ["truck_forward"] = "false", ["bus_forward"] = "false", ["moped_forward"] = "false", ["pedestrian"] = "false", ["bike_forward"] = "true"},
["path"] =              {["auto_forward"] = "false", ["truck_forward"] = "false", ["bus_forward"] = "false", ["moped_forward"] = "false", ["pedestrian"] = "true",  ["bike_forward"] = "true"},
["bus_guideway"] =      {["auto_forward"] = "false", ["truck_forward"] = "false", ["bus_forward"] = "true",  ["moped_forward"] = "false", ["pedestrian"] = "false", ["bike_forward"] = "false"}
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
["only_straight_on"] = 6,
["no_entry"] = 7,
["no_exit"] = 8,
["no_turn"] = 9
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

--the default speed for tracks is lowered after 
--the call to default_speed
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
["yes"] = "true",
["private"] = "true",
["no"] = "false",
["permissive"] = "true",
["agricultural"] = "false",
["use_sidepath"] = "true",
["delivery"] = "true",
["designated"] = "true",
["dismount"] = "true",
["discouraged"] = "false",
["forestry"] = "false",
["destination"] = "true",
["customers"] = "true",
["official"] = "false",
["public"] = "true",
["restricted"] = "true",
["allowed"] = "true",
["emergency"] = "false"
}

private = {
["private"] = "true",
["delivery"] = "true"
}

no_thru_traffic = {
["destination"] = "true",
["customers"] = "true",
["delivery"] = "true"
}

sidewalk = {
["both"] = "true",
["none"] = "false",
["no"] = "false",
["right"] = "true",
["left"] = "true",
["separate"] = "false",
["yes"] = "true",
["shared"] = "true",
["this"] = "true",
["detached"] = "false",
["raised"] = "true",
["separate_double"] = "false",
["sidepath"] = "false",
["explicit"] = "true"
}

use = {
["driveway"] = 4,
["alley"] = 5,
["parking_aisle"] = 6,
["emergency_access"] = 7,
["drive-through"] = 8
}

motor_vehicle = {
["yes"] = "true",
["private"] = "true",
["no"] = "false",
["permissive"] = "true",
["agricultural"] = "false",
["delivery"] = "true",
["designated"] = "true",
["discouraged"] = "false",
["forestry"] = "false",
["destination"] = "true",
["customers"] = "true",
["official"] = "false",
["public"] = "true",
["restricted"] = "true",
["allowed"] = "true"
}

moped = {
["yes"] = "true",
["designated"] = "true",
["private"] = "true",
["permissive"] = "true",
["destination"] = "true",
["delivery"] = "true",
["dismount"] = "true",
["no"] = "false",
["unknown"] = "false",
["agricultural"] = "false"
}

foot = {
["yes"] = "true",
["private"] = "true",
["no"] = "false",
["permissive"] = "true",
["agricultural"] = "false",
["use_sidepath"] = "true",
["delivery"] = "true",
["designated"] = "true",
["discouraged"] = "false",
["forestry"] = "false",
["destination"] = "true",
["customers"] = "true",
["official"] = "true",
["public"] = "true",
["restricted"] = "true",
["crossing"] = "true",
["sidewalk"] = "true",
["allowed"] = "true",
["passable"] = "true",
["footway"] = "true"
}

wheelchair = {
["no"] = "false",
["yes"] = "true",
["designated"] = "true",
["limited"] = "true",
["official"] = "true",
["destination"] = "true",
["public"] = "true",
["permissive"] = "true",
["only"] = "true",
["private"] = "true",
["impassable"] = "false",
["partial"] = "false",
["bad"] = "false",
["half"] = "false",
["assisted"] = "true"
}

bus = {
["no"] = "false",
["yes"] = "true",
["designated"] = "true",
["urban"] = "true",
["permissive"] = "true",
["restricted"] = "true",
["destination"] = "true",
["delivery"] = "false",
["official"] = "false"
}

psv = {
["bus"] = "true",
["no"] = "false",
["yes"] = "true",
["designated"] = "true",
["permissive"] = "true",
["1"] = "true",
["2"] = "true"
}

truck = {
["designated"] = "true",
["yes"] = "true",
["no"] = "false",
["destination"] = "true",
["delivery"] = "true",
["local"] = "true",
["agricultural"] = "false",
["private"] = "true",
["discouraged"] = "false",
["permissive"] = "false",
["unsuitable"] = "false",
["agricultural;forestry"] = "false",
["official"] = "false",
["forestry"] = "false",
["destination;delivery"] = "true"
}

hazmat = {
["designated"] = "true",
["yes"] = "true",
["no"] = "false",
["destination"] = "true",
["delivery"] = "true"
}

shoulder = {
["yes"] = "true",
["both"] = "true",
["no"] = "false"
}

shoulder_right = {
["right"] = "true"
}

shoulder_left = {
["left"] = "true"
}

bicycle = {
["yes"] = "true",
["designated"] = "true",
["use_sidepath"] = "true",
["no"] = "false",
["permissive"] = "true",
["destination"] = "true",
["dismount"] = "true",
["lane"] = "true",
["track"] = "true",
["shared"] = "true",
["shared_lane"] = "true",
["sidepath"] = "true",
["share_busway"] = "true",
["none"] = "false",
["allowed"] = "true",
["private"] = "true",
["official"] = "true"
}

cycleway = {
["yes"] = "true",
["designated"] = "true",
["use_sidepath"] = "true",
["permissive"] = "true",
["destination"] = "true",
["dismount"] = "true",
["lane"] = "true",
["track"] = "true",
["shared"] = "true",
["shared_lane"] = "true",
["sidepath"] = "true",
["share_busway"] = "true",
["allowed"] = "true",
["private"] = "true",
["cyclestreet"] = "true"
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

buffer = {
["yes"] = 2
}

dedicated = {
["opposite_lane"] = 2,
["lane"] = 2,
["buffered_lane"] = 2
}

separated = {
["opposite_track"] = 3,
["track"] = 3
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

--node proc needs the same info as above but in the form of a mask so duplicate..
motor_vehicle_node = {
["yes"] = 1,
["private"] = 1,
["no"] = 0,
["permissive"] = 1,
["agricultural"] = 0,
["delivery"] = 1,
["designated"] = 1,
["discouraged"] = 0,
["forestry"] = 0,
["destination"] = 1,
["customers"] = 1,
["official"] = 0,
["public"] = 1,
["restricted"] = 1,
["allowed"] = 1
}

bicycle_node = {
["yes"] = 4,
["designated"] = 4,
["use_sidepath"] = 4,
["no"] = 0,
["permissive"] = 4,
["destination"] = 4,
["dismount"] = 4,
["lane"] = 4,
["track"] = 4,
["shared"] = 4,
["shared_lane"] = 4,
["sidepath"] = 4,
["share_busway"] = 4,
["none"] = 0,
["allowed"] = 4,
["private"] = 4,
["official"] = 4
}

foot_node = {
["yes"] = 2,
["private"] = 2,
["no"] = 0,
["permissive"] = 2,
["agricultural"] = 0,
["use_sidepath"] = 2,
["delivery"] = 2,
["designated"] = 2,
["discouraged"] = 0,
["forestry"] = 0,
["destination"] = 2,
["customers"] = 2,
["official"] = 2,
["public"] = 2,
["restricted"] = 2,
["crossing"] = 2,
["sidewalk"] = 2,
["allowed"] = 2,
["passable"] = 2,
["footway"] = 2
}

wheelchair_node = {
["no"] = 0,
["yes"] = 256,
["designated"] = 256,
["limited"] = 256,
["official"] = 256,
["destination"] = 256,
["public"] = 256,
["permissive"] = 256,
["only"] = 256,
["private"] = 256,
["impassable"] = 0,
["partial"] = 0,
["bad"] = 0,
["half"] = 0,
["assisted"] = 256
}

moped_node = {
["yes"] = 512,
["designated"] = 512,
["private"] = 512,
["permissive"] = 512,
["destination"] = 512,
["delivery"] = 512,
["dismount"] = 512,
["no"] = 0,
["unknown"] = 0,
["agricultural"] = 0
}

bus_node = {
["no"] = 0,
["yes"] = 64,
["designated"] = 64,
["urban"] = 64,
["permissive"] = 64,
["restricted"] = 64,
["destination"] = 64,
["delivery"] = 0,
["official"] = 0,
}

truck_node = {
["designated"] = 8,
["yes"] = 8,
["no"] = 0,
["destination"] = 8,
["delivery"] = 8,
["local"] = 8,
["agricultural"] = 0,
["private"] = 8,
["discouraged"] = 0,
["permissive"] = 0,
["unsuitable"] = 0,
["agricultural;forestry"] = 0,
["official"] = 0,
["forestry"] = 0,
["destination;delivery"] = 8
}

psv_node = {
["bus"] = 64,
["no"] = 0,
["yes"] = 64,
["designated"] = 64,
["permissive"] = 64,
["1"] = 64,
["2"] = 64
}

function round(val, n)
  if (n) then
    return math.floor( (val * 10^n) + 0.5) / (10^n)
  else
    return math.floor(val+0.5)
  end
end

--convert the numeric (non negative) number portion at the beginning of the string
function numeric_prefix(num_str, allow_decimals)
  --not a string
  if num_str == nil then
    return nil
  end

  --find where the numbers stop
  local index = 0
  for c in num_str:gmatch"." do
    if tonumber(c) == nil then
      if c == "." then 
        if allow_decimals == false then
           break
        end
      else 
        break
      end
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
  local num = numeric_prefix(speed, false)

  --check if the rest of the string ends in "mph" convert to kph
  if num then 
    if speed:sub(-3) == "mph" then
      num = round(num * 1.609344)
    end

    --if num > 150kph or num < 10kph....toss    
    if num > 150 or num < 10 then
      return nil
    end
  end

  return num
end

function normalize_weight(weight)

  if weight then
    local w = weight:gsub("%s+", "")
    --grab the number prefix
    local num = numeric_prefix(w, true)

    if num then
      if w:sub(-1) == "t" or w:sub(-5) == "tonne" or w:sub(-6) == "tonnes" then 
        if (num .. "t" == w) or (num .. "tonne" == w) or (num .. "tonnes" == w) then
          return round(num,2)
        end
      end 

      if w:sub(-3) == "ton" or w:sub(-4) == "tons" then
         if (num .. "ton" == w) or (num .. "tons" == w) then
           return round(num,2)
         end
      end

      if w:sub(-2) == "lb" or w:sub(-3) == "lbs" then
        if (num .. "lb" == w) or (num .. "lbs" == w) then
          return round((num/2000),2) -- convert to tons
        end
      end

      if w:sub(-2) == "kg" then
        if (num .. "kg" == w) then
          return round((num/1000),2)
        end
      end
      return round(num,2) --3.5
    end
  end
  return nil
end

function normalize_measurement(measurement)

  if measurement then
     
    local m = measurement:gsub("%s+", "")
    --7'6" or 7ft6in
    --7m
    --7
    --grab the number prefix
    local num = numeric_prefix(m, true)
    if num then
      if m:sub(-1) == "m" or m:sub(-5) == "meter" or m:sub(-6) == "meters" then 
        if (num .. "m" == m) or (num .. "meter" == m) or (num .. "meters" == m) then
          return round(num,2)
        end
      end

      local feet
      local inches

      if m:sub(-2) == "in" or m:sub(-1) == "\"" or m:sub(-6) == "inches" or m:sub(-4) == "inch" then

--have to check for inches only
        if (num .. "in" == m) or (num .. "\"" == m) or (num .. "inches" == m) or (num .. "inch" == m) then
          return round((num * 0.0254),2)
        end

        feet = num
        m = string.sub(measurement, string.len(tostring(feet))+1)

        local index = 0
        for c in m:gmatch"." do
          if tonumber(c) ~= nil then
            break
          end
          index = index + 1
        end

        m = m:sub(index+1)
        inches = numeric_prefix(m, true)
        num = round((feet * 0.3048) + (inches * 0.0254),2)
      elseif m:sub(-2) == "ft" or m:sub(-1) == "\'" or m:sub(-4) == "feet" then
        feet = num
        num = round((feet * 0.3048),2)
      else 
        feet = num
        m = string.sub(measurement, string.len(tostring(feet))+1)

        local index = 0
        for c in m:gmatch"." do
          if tonumber(c) ~= nil then
            break
          end
          index = index + 1
        end

        if index ~= 0 then
--crappy data case.  7'6 or 7ft6
          m = m:sub(index+1)
          inches = numeric_prefix(m, true)
          if inches then 
            num = round((feet * 0.3048) + (inches * 0.0254),2)
          else 
            num = round((feet * 0.3048),2)
          end 
        end
      end
      return round(num,2)
    end
  end
  return nil
end

--returns 1 if you should filter this way 0 otherwise
function filter_tags_generic(kv)

  if (kv["highway"] == "construction" or kv["highway"] == "proposed") then
    return 1
  end

  --figure out what basic type of road it is
  local forward = highway[kv["highway"]]
  local ferry = kv["route"] == "ferry"
  local rail = kv["route"] == "shuttle_train"
  local access = access[kv["access"]]

  kv["emergency_forward"] = "false"
  kv["emergency_backward"] = "false"

  if (ferry == true or rail == true or kv["highway"]) then
    
    if (kv["access"] == "emergency" or kv["emergency"] == "yes" or kv["service"] == "emergency_access") then
      kv["emergency_forward"] = "true"
      kv["emergency_tag"] = "true"
    end
    
    if kv["emergency"] == "no" then
      kv["emergency_tag"] = "false"
    end
  end

  if forward then
    for k,v in pairs(forward) do
      kv[k] = v
    end

    if kv["impassable"] == "yes" or access == "false" or (kv["access"] == "private" and (kv["emergency"] == "yes" or kv["service"] == "emergency_access")) then

      kv["auto_forward"] = "false"
      kv["truck_forward"] = "false"
      kv["bus_forward"] = "false"
      kv["moped_forward"] = "false"
      kv["pedestrian"] = "false"
      kv["bike_forward"] = "false"

      kv["auto_backward"] = "false"
      kv["truck_backward"] = "false"
      kv["bus_backward"] = "false"
      kv["moped_backward"] = "false"
      kv["bike_backward"] = "false"
    elseif kv["vehicle"] == "no" then --don't change ped access.
      kv["auto_forward"] = "false"
      kv["truck_forward"] = "false"
      kv["bus_forward"] = "false"
      kv["moped_forward"] = "false"
      kv["bike_forward"] = "false"

      kv["auto_backward"] = "false"
      kv["truck_backward"] = "false"
      kv["bus_backward"] = "false"
      kv["moped_backward"] = "false"
      kv["bike_backward"] = "false"
    end

    --check for auto_forward overrides
    kv["auto_forward"] = motor_vehicle[kv["motorcar"]] or motor_vehicle[kv["motor_vehicle"]] or kv["auto_forward"]
    kv["auto_tag"] = motor_vehicle[kv["motorcar"]] or motor_vehicle[kv["motor_vehicle"]] or nil

    --check for truck_forward override
    kv["truck_forward"] = truck[kv["hgv"]] or motor_vehicle[kv["motor_vehicle"]] or kv["truck_forward"]
    kv["truck_tag"] = truck[kv["hgv"]] or motor_vehicle[kv["motor_vehicle"]] or nil

    --check for bus_forward overrides
    kv["bus_forward"] = bus[kv["bus"]] or psv[kv["psv"]] or psv[kv["lanes:psv:forward"]] or motor_vehicle[kv["motor_vehicle"]] or kv["bus_forward"]
    kv["bus_tag"] = bus[kv["bus"]] or psv[kv["psv"]] or psv[kv["lanes:psv:forward"]] or motor_vehicle[kv["motor_vehicle"]] or nil

    --check for ped overrides
    kv["pedestrian"] = foot[kv["foot"]] or foot[kv["pedestrian"]] or kv["pedestrian"] 
    kv["foot_tag"] = foot[kv["foot"]] or foot[kv["pedestrian"]] or nil

    --check for bike_forward overrides
    kv["bike_forward"] = bicycle[kv["bicycle"]] or cycleway[kv["cycleway"]] or bicycle[kv["bicycle_road"]] or bicycle[kv["cyclestreet"]] or kv["bike_forward"]
    kv["bike_tag"] = bicycle[kv["bicycle"]] or cycleway[kv["cycleway"]] or bicycle[kv["bicycle_road"]] or bicycle[kv["cyclestreet"]] or nil

    --check for moped forward overrides
    kv["moped_forward"] = moped[kv["moped"]] or moped[kv["mofa"]] or motor_vehicle[kv["motor_vehicle"]] or kv["moped_forward"]
    kv["moped_tag"] = moped[kv["moped"]] or moped[kv["mofa"]] or motor_vehicle[kv["motor_vehicle"]] or nil

    if kv["bike_tag"] == nil then
      if kv["sac_scale"] == "hiking" then
        kv["bike_forward"] = "true"
        kv["bike_tag"] = "true"
      elseif kv["sac_scale"] then
        kv["bike_forward"] = "false"
      end
    end

    if kv["motorroad"] == "yes" then
      kv["motorroad_tag"] = "true"
    end

  else
    --if its a ferry and these tags dont show up we want to set them to true 
    local default_val = tostring(ferry)

    if ferry == false and rail == true then
      default_val = tostring(rail)
    end

    if ((ferry == false and rail == false) or kv["impassable"] == "yes" or access == "false" or (kv["access"] == "private" and (kv["emergency"] == "yes" or kv["service"] == "emergency_access"))) then

      kv["auto_forward"] = "false"
      kv["truck_forward"] = "false"
      kv["bus_forward"] = "false"
      kv["moped_forward"] = "false"
      kv["pedestrian"] = "false"
      kv["bike_forward"] = "false"

      kv["auto_backward"] = "false"
      kv["truck_backward"] = "false"
      kv["bus_backward"] = "false"
      kv["moped_backward"] = "false"
      kv["bike_backward"] = "false"

    else
      local ped_val = default_val
      if kv["vehicle"] == "no" then --don't change ped access.
        default_val = "false"
      end
      --check for auto_forward overrides
      kv["auto_forward"] = motor_vehicle[kv["motorcar"]] or motor_vehicle[kv["motor_vehicle"]] or default_val
      kv["auto_tag"] = motor_vehicle[kv["motorcar"]] or motor_vehicle[kv["motor_vehicle"]] or nil

      --check for truck_forward override
      kv["truck_forward"] = truck[kv["hgv"]] or kv["truck_forward"] or motor_vehicle[kv["motor_vehicle"]] or default_val
      kv["truck_tag"] = truck[kv["hgv"]] or motor_vehicle[kv["motor_vehicle"]] or nil

      --check for bus_forward overrides
      kv["bus_forward"] = bus[kv["bus"]] or psv[kv["psv"]] or psv[kv["lanes:psv:forward"]] or motor_vehicle[kv["motor_vehicle"]] or default_val
      kv["bus_tag"] = bus[kv["bus"]] or psv[kv["psv"]] or psv[kv["lanes:psv:forward"]] or motor_vehicle[kv["motor_vehicle"]] or nil

      --check for ped overrides
      kv["pedestrian"] = foot[kv["foot"]] or foot[kv["pedestrian"]] or ped_val
      kv["foot_tag"] = foot[kv["foot"]] or foot[kv["pedestrian"]] or nil

      --check for bike_forward overrides
      kv["bike_forward"] = bicycle[kv["bicycle"]] or cycleway[kv["cycleway"]] or bicycle[kv["bicycle_road"]] or bicycle[kv["cyclestreet"]] or default_val
      kv["bike_tag"] = bicycle[kv["bicycle"]] or cycleway[kv["cycleway"]] or bicycle[kv["bicycle_road"]] or bicycle[kv["cyclestreet"]] or nil

      --check for moped forward overrides
      kv["moped_forward"] = moped[kv["moped"]] or moped[kv["mofa"]] or motor_vehicle[kv["motor_vehicle"]] or default_val
      kv["moped_tag"] = moped[kv["moped"]] or moped[kv["mofa"]] or motor_vehicle[kv["motor_vehicle"]] or nil
      
      if kv["bike_tag"] == nil then
        if kv["sac_scale"] == "hiking" then
          kv["bike_forward"] = "true"
          kv["bike_tag"] = "true"
        elseif kv["sac_scale"] then
          kv["bike_forward"] = "false"
        end
      end

      if kv["motorroad"] == "yes" then
        kv["motorroad_tag"] = "true"
      end
    end
  end

  --TODO: handle Time conditional restrictions if available for HOVs with oneway = reversible
  if ((kv["access"] == "permissive" or kv["access"] == "hov") and kv["oneway"] == "reversible") then
   
    -- for now enable only for buses if the tag exists and they are allowed.
    if (kv["bus_forward"] == "true") then
      kv["auto_forward"] = "false"
      kv["truck_forward"] = "false"
      kv["pedestrian"] = "false"
      kv["bike_forward"] = "false"
      kv["moped_forward"] = "false"
    else
      return 1
    end
  end

  --service=driveway means all are routable
  if kv["service"] == "driveway" and kv["access"] == nil then
    kv["auto_forward"] = "true"
    kv["truck_forward"] = "true"
    kv["bus_forward"] = "true"
    kv["pedestrian"] = "true"
    kv["bike_forward"] = "true"
    kv["moped_forward"] = "true"
  end

  --check the oneway-ness and traversability against the direction of the geom
  if ((kv["oneway"] == "yes" and kv["oneway:bicycle"] == "no") or kv["bicycle:backward"] == "yes" or kv["bicycle:backward"] == "no") then
    kv["bike_backward"] = "true"
  end

  if kv["bike_backward"] == nil or kv["bike_backward"] == "false" then
    kv["bike_backward"] = bike_reverse[kv["cycleway"]] or bike_reverse[kv["cycleway:left"]] or bike_reverse[kv["cycleway:right"]] or "false"
  end

  if kv["bike_backward"] == "true" then
    oneway_bike = oneway[kv["oneway:bicycle"]]
  end

  if kv["oneway:bus"] == nil and kv["oneway:psv"] ~= nil then
    kv["oneway:bus"] = kv["oneway:psv"]
  end

  if ((kv["oneway"] == "yes" and kv["oneway:bus"] == "no") or kv["bus:backward"] == "yes" or kv["bus:backward"] == "designated") then
    kv["bus_backward"] = "true"
  end

  if kv["bus_backward"] == nil or kv["bus_backward"] == "false" then
    kv["bus_backward"] = bus_reverse[kv["busway"]] or bus_reverse[kv["busway:left"]] or bus_reverse[kv["busway:right"]] or psv[kv["lanes:psv:backward"]] or "false"
  end

  if kv["bus_backward"] == "true" then
    oneway_bus = oneway[kv["oneway:bus"]]
    if (oneway_bus == "false" and kv["bus:backward"] == "yes") then
      oneway_bus = "true"
    end
  end

  if kv["moped_backward"] == nil then
    kv["moped_backward"] = "false"
  end

  if ((kv["oneway"] == "yes" and (kv["oneway:moped"] == "no" or kv["oneway:mofa"] == "no")) or kv["moped:backward"] == "yes" or kv["mofa:backward"] == "yes") then
    kv["moped_backward"] = "true"
  end

  if kv["moped_backward"] == "true" then
    oneway_moped = oneway[kv["oneway:moped"]] or oneway[kv["oneway:mofa"]]
  end

  local oneway_reverse = kv["oneway"]
  local oneway_norm = oneway[kv["oneway"]]
  if kv["junction"] == "roundabout" or kv["junction"] == "circular" then
    oneway_norm = "true"
    kv["roundabout"] = "true"
  else
    kv["roundabout"] = "false"
  end
  kv["oneway"] = oneway_norm
  if oneway_norm == "true" then
    kv["auto_backward"] = "false"
    kv["truck_backward"] = "false"
    kv["emergency_backward"] = "false"
 
    if kv["bike_backward"] == "true" then 
      if (oneway_bike == "true") then --bike only in reverse on a bike path.
        kv["bike_forward"] = "false"
      elseif oneway_bike == "false" then --bike in both directions on a bike path.
        kv["bike_forward"] = "true"
      end
    end
    if kv["bus_backward"] == "true" then
      if (oneway_bus == "true") then --bus only in reverse on a bus path.
        kv["bus_forward"] = "false"
      elseif oneway_bus == "false" then --bus in both directions on a bus path.
        kv["bus_forward"] = "true"
      end
    end
    if kv["moped_backward"] == "true" then
      if (oneway_moped == "true") then --moped only in reverse direction on street
        kv["moped_forward"] = "false"
      elseif oneway_moped == "false" then
        kv["moped_forward"] = "true"
      end
    end

  elseif oneway_norm == nil or oneway_norm == "false" then
    kv["auto_backward"] = kv["auto_forward"]
    kv["truck_backward"] = kv["truck_forward"]
    kv["emergency_backward"] = kv["emergency_forward"]

    if (kv["bike_backward"] == "false" and kv["oneway:bicycle"] ~= "-1" and 
       (kv["oneway:bicycle"] == nil or oneway[kv["oneway:bicycle"]] == false or kv["oneway:bicycle"] == "no")) then
      kv["bike_backward"] = kv["bike_forward"]
    end

    if (kv["bus_backward"] == "false" and kv["oneway:bus"] ~= "-1" and 
       (kv["oneway:bus"] == nil or oneway[kv["oneway:bus"]] == false)) then
      kv["bus_backward"] = kv["bus_forward"]
    end

    if (kv["moped_backward"] == "false" and (kv["oneway:moped"] == nil or oneway[kv["oneway:moped"]] == false or kv["oneway:moped"] == "no") and
       (kv["oneway:mofa"] == nil or oneway[kv["oneway:mofa"]] == false or kv["oneway:mofa"] == "no")) then
      kv["moped_backward"] = kv["moped_forward"]
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

  kv["oneway_reverse"] = "false"

  --flip the onewayness 
  if oneway_reverse == "-1" then
    kv["oneway_reverse"] = "true"
    local forwards = kv["auto_forward"]
    kv["auto_forward"] = kv["auto_backward"]
    kv["auto_backward"] = forwards

    forwards = kv["truck_forward"]
    kv["truck_forward"] = kv["truck_backward"]
    kv["truck_backward"] = forwards

    forwards = kv["emergency_forward"]
    kv["emergency_forward"] = kv["emergency_backward"]
    kv["emergency_backward"] = forwards

    forwards = kv["bus_forward"]
    kv["bus_forward"] = kv["bus_backward"]
    kv["bus_backward"] = forwards

    forwards = kv["bike_forward"]
    kv["bike_forward"] = kv["bike_backward"]
    kv["bike_backward"] = forwards

    forwards = kv["moped_forward"]
    kv["moped_forward"] = kv["moped_backward"]
    kv["moped_backward"] = forwards
  end

  if kv["oneway:bicycle"] == "-1" then
    local forwards = kv["bike_forward"]
    kv["bike_forward"] = kv["bike_backward"]
    kv["bike_backward"] = forwards
  end

  if kv["oneway:bus"] == "-1" then
    local forwards = kv["bus_forward"]
    kv["bus_forward"] = kv["bus_backward"]
    kv["bus_backward"] = forwards
  end

  -- bus only logic
  if kv["lanes:bus"] == "1" then
    kv["bus_forward"] = "true"
    kv["bus_backward"] = "false"
  elseif kv["lanes:bus"] == "2" then  
    kv["bus_forward"] = "true"
    kv["bus_backward"] = "true"
  end 

  --if none of the modes were set we are done looking at this  
  if kv["auto_forward"] == "false" and kv["truck_forward"] == "false" and kv["bus_forward"] == "false" and
     kv["bike_forward"] == "false" and kv["emergency_forward"] == "false" and kv["moped_forward"] == "false" and
     kv["auto_backward"] == "false" and kv["truck_backward"] == "false" and kv["bus_backward"] == "false" and
     kv["bike_backward"] == "false" and kv["emergency_backward"] == "false" and kv["moped_backward"] == "false" and
     kv["pedestrian"] == "false" then
       if kv["highway"] ~= "bridleway" then --save bridleways for country access logic.
         return 1
       end
  end

   --toss actual areas
   if kv["area"] == "yes" then
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
  elseif kv["highway"] == nil and (kv["railway"] or kv["route"] == "shuttle_train") then
    road_class = 2 --TODO:  can we weight based on rail types?    
  elseif road_class == nil then --service and other = 7
    road_class = 7
  end 
  
  kv["road_class"] = road_class

  kv["default_speed"] = default_speed[kv["road_class"]]

  --lower the default speed for driveways
  if kv["service"] == "driveway" then
     kv["default_speed"] = math.floor(tonumber(kv["default_speed"]) * 0.5)
  end

  local use = use[kv["service"]]

  if kv["highway"] then
     if kv["highway"] == "track" then
        use = 3
     elseif kv["highway"] == "living_street" then
        use = 10
     elseif kv["highway"] == "cycleway" then
        use = 20
     elseif kv["pedestrian"] == "false" and kv["auto_forward"] == "false" and kv["auto_backward"] == "false" and (kv["bike_forward"] == "true" or kv["bike_backward"] == "true") then
        use = 20
     elseif (kv["highway"] == "footway" and kv["footway"] == "sidewalk") then
        use = 24
     elseif kv["highway"] == "footway" then
        use = 25
     elseif kv["highway"] == "steps" then
        use = 26 --steps/stairs     
     elseif kv["highway"] == "path" then
        use = 27
     elseif kv["highway"] == "pedestrian" then
        use = 28
     elseif kv["pedestrian"] == "true" and
            kv["auto_forward"] == "false" and kv["auto_backward"] == "false" and
            kv["truck_forward"] == "false" and kv["truck_backward"] == "false" and
            kv["bus_forward"] == "false" and kv["bus_backward"] == "false" and
            kv["bike_forward"] == "false" and kv["bike_backward"] == "false" and
            kv["moped_forward"] == "false" and kv["moped_backward"] == "false" then
        use = 28
     elseif kv["highway"] == "bridleway" then
        use = 29
     end
  end

  if use == nil and kv["service"] then
    use = 40 --other
  elseif use == nil then
    use = 0 --general road, no special use
  end

  if kv["access"] == "emergency" or kv["emergency"] == "yes" then
    use = 7
  end

  kv["use"] = use

  local r_shoulder = shoulder[kv["shoulder"]] or shoulder[kv["shoulder:both"]] or nil
  local l_shoulder = r_shoulder

  if r_shoulder == nil then
    r_shoulder = shoulder[kv["shoulder:right"]] or shoulder_right[kv["shoulder"]] or "false"
    l_shoulder = shoulder[kv["shoulder:left"]] or shoulder_left[kv["shoulder"]] or "false"

    --If the road is oneway and one shoulder is tagged but not the other, we set both to true so that when setting the
    --shoulder in graphbuilder, driving on the right side vs the left side doesn't cause the edge to miss the shoulder tag
    if oneway_norm == "true" and r_shoulder == "true" and l_shoulder == "false" then
      l_shoulder = "true"
    elseif oneway_norm == "true" and r_shoulder == "false" and l_shoulder == "true" then
      r_shoulder = "true"
    end
  end

  kv["shoulder_right"] = r_shoulder
  kv["shoulder_left"] = l_shoulder


  local cycle_lane_right_opposite = "false"
  local cycle_lane_left_opposite = "false"

  local cycle_lane_right = 0
  local cycle_lane_left = 0

  --We have special use cases for cycle lanes when on a cycleway, footway, or path
  if (use == 20 or use == 25 or use == 27) and
     (kv["bike_forward"] == "true" or kv["bike_backward"] == "true") then
    if kv["pedestrian"] == "false" then
      cycle_lane_right = 3 --separated
    elseif kv["segregated"] == "yes" then
      cycle_lane_right = 2 --dedicated
    elseif kv["segregated"] == "no" then
      cycle_lane_right = 1 --shared
    elseif use == 20 then
      cycle_lane_right = 2 --If no segregated tag but it is tagged as a cycleway then we assume separated lanes
    else
      cycle_lane_right = 1 --If no segregated tag and it's tagged as a footway or path then we assume shared lanes
    end
    cycle_lane_left = cycle_lane_right
  else
    --Set flags if any of the lanes are marked "opposite" (contraflow)
    cycle_lane_right_opposite = bike_reverse[kv["cycleway"]] or "false"
    cycle_lane_left_opposite = cycle_lane_right_opposite

    if cycle_lane_right_opposite == "false" then
      cycle_lane_right_opposite = bike_reverse[kv["cycleway:right"]] or "false"
      cycle_lane_left_opposite = bike_reverse[kv["cycleway:left"]] or "false"
    end

    --Figure out which side of the road has what cyclelane
    cycle_lane_right = shared[kv["cycleway"]] or separated[kv["cycleway"]] or dedicated[kv["cycleway"]] or buffer[kv["cycleway:both:buffer"]] or 0
    cycle_lane_left = cycle_lane_right

    if cycle_lane_right == 0 then
      cycle_lane_right = shared[kv["cycleway:right"]] or separated[kv["cycleway:right"]] or dedicated[kv["cycleway:right"]] or buffer[kv["cycleway:right:buffer"]] or 0
      cycle_lane_left = shared[kv["cycleway:left"]] or separated[kv["cycleway:left"]] or dedicated[kv["cycleway:left"]] or buffer[kv["cycleway:left:buffer"]] or 0
    end

    --If we have the oneway:bicycle=no tag and there are not "opposite_lane/opposite_track" tags then there are certain situations where
    --the cyclelane is considered a two-way. (Based off of some examples on wiki.openstreetmap.org/wiki/Bicycle)
    if kv["oneway:bicycle"] == "no" and cycle_lane_right_opposite == "false" and cycle_lane_left_opposite == "false" then
      if cycle_lane_right == 2 or cycle_lane_right == 3 then
        --Example M1 or M2d but on the right side
        if oneway_norm == "true" then
          cycle_lane_left = cycle_lane_right
          cycle_lane_left_opposite = "true"

        --Example L1b
        elseif cycle_lane_left == 0 then
          cycle_lane_left = cycle_lane_right
        end

      elseif cycle_lane_left == 2 or cycle_lane_left == 3 then
        --Example M2d
        if oneway_norm == "true" then
          cycle_lane_right = cycle_lane_left
          cycle_lane_right_opposite = "true"

        --Example L1b but on the left side
        elseif cycle_lane_right == 0 then
          cycle_lane_right = cycle_lane_left
        end
      end
    end
  end

  kv["cycle_lane_right"] = cycle_lane_right
  kv["cycle_lane_left"] = cycle_lane_left

  kv["cycle_lane_right_opposite"] = cycle_lane_right_opposite
  kv["cycle_lane_left_opposite"] = cycle_lane_left_opposite


  if kv["highway"] and string.find(kv["highway"], "_link") then --*_link 
     kv["link"] = "true"  --do we need to add more?  turnlane?
     kv["link_type"] = kv["link_type"]
  end

  kv["private"] = private[kv["access"]] or private[kv["motor_vehicle"]] or "false"
  kv["no_thru_traffic"] = no_thru_traffic[kv["access"]] or "false"
  kv["ferry"] = tostring(ferry)
  kv["rail"] = tostring(kv["auto_forward"] == "true" and (kv["railway"] == "rail" or kv["route"] == "shuttle_train"))
  kv["name"] = kv["name"]
  kv["name:en"] = kv["name:en"]
  kv["alt_name"] = kv["alt_name"]
  kv["official_name"] = kv["official_name"]
  kv["max_speed"] = normalize_speed(kv["maxspeed"])
  kv["advisory_speed"] = normalize_speed(kv["maxspeed:advisory"])
  kv["average_speed"] = normalize_speed(kv["maxspeed:practical"])
  kv["backward_speed"] = normalize_speed(kv["maxspeed:backward"])
  kv["forward_speed"] = normalize_speed(kv["maxspeed:forward"])
  kv["int"] = kv["int"]
  kv["int_ref"] = kv["int_ref"]
  kv["surface"] = kv["surface"]
  kv["wheelchair"] = wheelchair[kv["wheelchair"]]

  --lower the default speed for tracks
  if kv["highway"] == "track" then
     kv["default_speed"] = 5
     if kv["tracktype"] then
       if kv["tracktype"] == "grade1" then
         kv["default_speed"] = 20
       elseif kv["tracktype"] == "grade2" then
         kv["default_speed"] = 15
       elseif kv["tracktype"] == "grade3" then
         kv["default_speed"] = 12
       elseif kv["tracktype"] == "grade4" then
         kv["default_speed"] = 10
       end
     end
  end

  --use unsigned_ref if all the conditions are met.
  if ((kv["name"] == nil and kv["name:en"] == nil and kv["alt_name"] == nil and kv["official_name"] == nil and kv["ref"] == nil and kv["int_ref"] == nil) and
      (kv["highway"] == "motorway" or kv["highway"] == "trunk" or kv["highway"] == "primary") and kv["unsigned_ref"] ~= nil) then
        kv["ref"] = kv["unsigned_ref"]
  end

  lane_count = numeric_prefix(kv["lanes"],false)
  if lane_count and lane_count > 10 then
    lane_count = 10
  end
  kv["lanes"] = lane_count

  lane_count = numeric_prefix(kv["lanes:forward"],false)
  if lane_count and lane_count > 10 then
    lane_count = 10
  end
  kv["forward_lanes"] = lane_count

  lane_count = numeric_prefix(kv["lanes:backward"],false)
  if lane_count and lane_count > 10 then
    lane_count = 10
  end
  kv["backward_lanes"] = lane_count

  kv["bridge"] = bridge[kv["bridge"]] or "false"
  
  -- TODO access:conditional
  if kv["seasonal"] and kv["seasonal"] ~= "no" then
    kv["seasonal"] = "true"
  end

  if (kv["hov"] and kv["hov"] == "no") then
    kv["hov_tag"] = "false"
    kv["hov_forward"] = "false"
    kv["hov_backward"] = "false"
  else
    kv["hov_forward"] = kv["auto_forward"]
    kv["hov_backward"] = kv["auto_backward"]

  end

  if ((kv["hov"] and kv["hov"] ~= "no") or kv["hov:lanes"] or kv["hov:minimum"]) then

    kv["hov_tag"] = "true"

    if (kv["hov"] == "designated") then
      if (kv["auto_tag"] == nil) then
        kv["auto_forward"] = "false"
        kv["auto_backward"] = "false"
      end

      if (kv["truck_tag"] == nil) then
        kv["truck_forward"] = "false"
        kv["truck_backward"] = "false"
      end

      if (kv["bus_tag"] == nil) then
        kv["bus_forward"] = "false"
        kv["bus_backward"] = "false"
      end

      if (kv["foot_tag"] == nil) then
        kv["pedestrian"] = "false"
      end

      if (kv["bike_tag"] == nil) then
        kv["bike_forward"] = "false"
        kv["bike_backward"] = "false"
      end

      if (kv["moped_tag"] == nil) then
        kv["moped_forward"] = "false"
        kv["moped_backward"] = "false"
      end
    end
  end

  kv["tunnel"] = tunnel[kv["tunnel"]] or "false"
  kv["toll"] = toll[kv["toll"]] or "false"
  kv["destination"] = kv["destination"]
  kv["destination:forward"] = kv["destination:forward"]
  kv["destination:backward"] = kv["destination:backward"]
  kv["destination:ref"] = kv["destination:ref"]
  kv["destination:ref:to"] = kv["destination:ref:to"]
  kv["destination:street"] = kv["destination:street"]
  kv["destination:street:to"] = kv["destination:street:to"]
  kv["junction:ref"] = kv["junction:ref"]

  --truck goodies
  kv["maxheight"] = normalize_measurement(kv["maxheight"]) or normalize_measurement(kv["maxheight:physical"])
  kv["maxwidth"] = normalize_measurement(kv["maxwidth"]) or normalize_measurement(kv["maxwidth:physical"])
  kv["maxlength"] = normalize_measurement(kv["maxlength"])
  
  kv["maxweight"] = normalize_weight(kv["maxweight"])
  kv["maxaxleload"] = normalize_weight(kv["maxaxleload"])

  --TODO: hazmat really should have subcategories
  kv["hazmat"] = hazmat[kv["hazmat"]] or hazmat[kv["hazmat:water"]] or hazmat[kv["hazmat:A"]] or hazmat[kv["hazmat:B"]] or 
                 hazmat[kv["hazmat:C"]] or hazmat[kv["hazmat:D"]] or hazmat[kv["hazmat:E"]] 
  kv["maxspeed:hgv"] = normalize_speed(kv["maxspeed:hgv"])
  
  if (kv["hgv:national_network"] or kv["hgv:state_network"] or kv["hgv"] == "local" or kv["hgv"] == "designated") then
    kv["truck_route"] = "true"
  end

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

function nodes_proc (kv, nokeys)
  --normalize a few tags that we care about
  local access = access[kv["access"]] or "true"

  if (kv["impassable"] == "yes" or (kv["access"] == "private" and (kv["emergency"] == "yes" or kv["service"] == "emergency_access"))) then
    access = "false"
  end 

  local hov_tag = nil
  if ((kv["hov"] and kv["hov"] ~= "no") or kv["hov:lanes"] or kv["hov:minimum"]) then
    hov_tag = 128
  end

  local foot_tag = foot_node[kv["foot"]]
  local wheelchair_tag = wheelchair_node[kv["wheelchair"]]
  local bike_tag = bicycle_node[kv["bicycle"]]
  local truck_tag = truck_node[kv["hgv"]]
  local auto_tag = motor_vehicle_node[kv["motorcar"]]
  local motor_vehicle_tag = motor_vehicle_node[kv["motor_vehicle"]]
  local moped_tag = moped_node[kv["moped"]] or moped_node[kv["mofa"]]
  if auto_tag == nil then
    auto_tag = motor_vehicle_tag
  end
  local bus_tag = bus_node[kv["bus"]]
  if bus_tag == nil then
    bus_tag = psv_node[kv["psv"]]
  end
  --if bus was not set and car is
  if bus_tag == nil and auto_tag == 1 then
    bus_tag = 64
  end

  --if wheelchair was not set and foot is
  if wheelchair_tag == nil and foot_tag == 2 then
    wheelchair_tag = 256
  end

  --if hov was not set and car is
  if hov_tag == nil and auto_tag == 1 then
    hov_tag = 128
  end

  --if truck was not set and car is
  if truck_tag == nil and auto_tag == 1 then
    truck_tag = 8
  end

  --must shut these off if motor_vehicle = 0
  if motor_vehicle_tag == 0 then
    bus_tag = 0
    truck_tag = 0
    moped_tag = 0
  end

  local emergency_tag --implies nil 
  if kv["access"] == "emergency" or kv["emergency"] == "yes" or kv["service"] == "emergency_access" then
     emergency_tag = 16
  end

  --do not shut off bike access if there is a highway crossing.
  if bike_tag == 0 and kv["highway"] == "crossing" then
    bike_tag = 4
  end

  --if tag exists use it, otherwise access allowed for all modes unless access = false or kv["hov"] == "designated" or kv["vehicle"] == "no")
  local auto = auto_tag or 1
  local truck = truck_tag or 8 
  local bus = bus_tag or 64
  local foot = foot_tag or 2
  local wheelchair = wheelchair_tag or 256
  local bike = bike_tag or 4
  local emergency = emergency_tag or 16
  local hov = hov_tag or 128
  local moped = moped_tag or 512

  --if access = false use tag if exists, otherwise no access for that mode.
  if (access == "false" or kv["vehicle"] == "no" or kv["hov"] == "designated") then
    auto = auto_tag or 0
    truck = truck_tag or 0
    bus = bus_tag or 0

    --don't change ped if kv["vehicle"] == "no"
    if access == "false" or kv["hov"] == "designated" then
      foot = foot_tag or 0
    end

    wheelchair = wheelchair_tag or 0
    bike = bike_tag or 0
    moped = moped_tag or 0
    emergency = emergency_tag or 0
    hov = hov_tag or 0
  end 

  --check for gates and bollards
  local gate = kv["barrier"] == "gate" or kv["barrier"] == "lift_gate" 
  local bollard = false
  if gate == false then
    --if there was a bollard cars can't get through it
    bollard = kv["barrier"] == "bollard" or kv["barrier"] == "block" or kv["bollard"] == "removable" or false

    --save the following as gates.
    if (bollard and (kv["bollard"] == "rising")) then
      gate = true
      bollard = false
    end
   
    --bollard = true shuts off access unless the tag exists.
    if bollard == true then
      auto = auto_tag or 0
      truck = truck_tag or 0
      bus = bus_tag or 0
      foot = foot_tag or 2
      wheelchair = wheelchair_tag or 256
      bike = bike_tag or 4
      moped = moped_tag or 0
      emergency = emergency_tag or 0
      hov = hov_tag or 0
    end
  end

  --if nothing blocks access at this node assume access is allowed.
  if gate == false and bollard == false and access == "true" then    
    if kv["highway"] == "crossing" or kv["railway"] == "crossing" or 
       kv["footway"] == "crossing" or kv["cycleway"] == "crossing" or
       kv["foot"] == "crossing" or kv["bicycle"] == "crossing" or
       kv["pedestrian"] == "crossing" or kv["crossing"] then
         auto = auto_tag or 1
         truck = truck_tag or 8
         bus = bus_tag or 64
         foot = foot_tag or 2
         wheelchair = wheelchair_tag or 256
         bike = bike_tag or 4
         moped = moped_tag or 512
         emergency = emergency_tag or 16
         hov = hov_tag or 128
    end
  end

  --store the gate and bollard info
  kv["gate"] = tostring(gate)
  kv["bollard"] = tostring(bollard)

  if kv["barrier"] == "border_control" then
    kv["border_control"] = "true"
  elseif kv["barrier"] == "toll_booth" then
    kv["toll_booth"] = "true"
  end

  local coins = toll[kv["payment:coins"]] or "false"
  local notes = toll[kv["payment:notes"]] or "false"

  --assume cash for toll, toll:*, and fee
  local cash =  toll[kv["toll"]] or toll[kv["toll:hgv"]] or toll[kv["toll:bicycle"]] or toll[kv["toll:hov"]] or
                toll[kv["toll:motorcar"]] or toll[kv["toll:motor_vehicle"]] or toll[kv["toll:bus"]] or 
                toll[kv["toll:motorcycle"]] or toll[kv["payment:cash"]] or toll[kv["fee"]] or "false"
  
  local etc = toll[kv["payment:e_zpass"]] or toll[kv["payment:e_zpass:name"]] or
              toll[kv["payment:pikepass"]] or toll[kv["payment:via_verde"]] or "false"
  
  local cash_payment = 0

  if (cash == "true" or (coins == "true" and notes == "true")) then
    cash_payment = 3
  elseif coins == "true" then
    cash_payment = 1
  elseif notes == "true" then
    cash_payment = 2
  end

  local etc_payment = 0

  if etc == "true" then 
    etc_payment = 4
  end

  --store a mask denoting payment type 
  kv["payment_mask"] = bit32.bor(cash_payment, etc_payment)

  if kv["amenity"] == "bicycle_rental" or (kv["shop"] == "bicycle" and kv["service:bicycle:rental"] == "yes") then
    kv["bicycle_rental"] = "true"
  end

  if kv["traffic_signals:direction"] == "forward" then
    kv["forward_signal"] = "true"
  end

  if kv["traffic_signals:direction"] == "backward" then
    kv["backward_signal"] = "true"
  end
 
  --store a mask denoting access
  kv["access_mask"] = bit32.bor(auto, emergency, truck, bike, foot, wheelchair, bus, hov, moped)

  return 0, kv
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
  if (kv["type"] == "connectivity") then
    return 0, kv
  end

  if (kv["type"] == "route" or kv["type"] == "restriction") then

     local restrict = restriction[kv["restriction"]]

     local restrict_type = restriction[kv["restriction:hgv"]] or restriction[kv["restriction:emergency"]] or
                           restriction[kv["restriction:taxi"]] or restriction[kv["restriction:motorcar"]] or
                           restriction[kv["restriction:bus"]] or restriction[kv["restriction:bicycle"]] or
                           restriction[kv["restriction:hazmat"]]

     --restrictions with type win over just restriction key.  people enter both.
     if restrict_type ~= nil then
       restrict = restrict_type
     end

     if kv["type"] == "restriction" then

       if restrict ~= nil then

         kv["restriction:hgv"] = restriction[kv["restriction:hgv"]]
         kv["restriction:emergency"] = restriction[kv["restriction:emergency"]]
         kv["restriction:taxi"] = restriction[kv["restriction:taxi"]]
         kv["restriction:motorcar"] = restriction[kv["restriction:motorcar"]]
         kv["restriction:bus"] = restriction[kv["restriction:bus"]]
         kv["restriction:bicycle"] = restriction[kv["restriction:bicycle"]]
         kv["restriction:hazmat"] = restriction[kv["restriction:hazmat"]]

         if restrict_type == nil then
           kv["restriction"] = restrict
         else
           kv["restriction"] = nil
         end

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
     elseif kv["route"] == "bicycle" or kv["route"] == "mtb" then
       
       local bike_mask = 0

       if kv["network"] == "mtb" or kv["route"] == "mtb" then
         bike_mask = 8
       end

       if kv["network"] == "ncn" then
         bike_mask = bit32.bor(bike_mask, 1)
       elseif kv["network"] == "rcn" then
         bike_mask = bit32.bor(bike_mask, 2)
       elseif kv["network"] == "lcn" then
         bike_mask = bit32.bor(bike_mask, 4)
       end

       kv["bike_network_mask"] = bike_mask

       kv["day_on"] = nil
       kv["day_off"] = nil
       kv["restriction"] = nil       
  
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
