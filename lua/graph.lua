--TODO: check if you can use lua type boolean instead of strings and pass that back to osm2pgsql
--with the hopes that they will become strings once they get back to c++ and then just work in
--postgres

highway = {
["motorway"] =          {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["taxi_forward"] = "true",  ["moped_forward"] = "false", ["motorcycle_forward"] = "true",  ["pedestrian_forward"] = "false", ["bike_forward"] = "false"},
["motorway_link"] =     {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["taxi_forward"] = "true",  ["moped_forward"] = "false", ["motorcycle_forward"] = "true",  ["pedestrian_forward"] = "false", ["bike_forward"] = "false"},
["trunk"] =             {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["taxi_forward"] = "true",  ["moped_forward"] = "true",  ["motorcycle_forward"] = "true",  ["pedestrian_forward"] = "true",  ["bike_forward"] = "true"},
["trunk_link"] =        {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["taxi_forward"] = "true",  ["moped_forward"] = "true",  ["motorcycle_forward"] = "true",  ["pedestrian_forward"] = "true",  ["bike_forward"] = "true"},
["primary"] =           {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["taxi_forward"] = "true",  ["moped_forward"] = "true",  ["motorcycle_forward"] = "true",  ["pedestrian_forward"] = "true",  ["bike_forward"] = "true"},
["primary_link"] =      {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["taxi_forward"] = "true",  ["moped_forward"] = "true",  ["motorcycle_forward"] = "true",  ["pedestrian_forward"] = "true",  ["bike_forward"] = "true"},
["secondary"] =         {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["taxi_forward"] = "true",  ["moped_forward"] = "true",  ["motorcycle_forward"] = "true",  ["pedestrian_forward"] = "true",  ["bike_forward"] = "true"},
["secondary_link"] =    {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["taxi_forward"] = "true",  ["moped_forward"] = "true",  ["motorcycle_forward"] = "true",  ["pedestrian_forward"] = "true",  ["bike_forward"] = "true"},
["residential"] =       {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["taxi_forward"] = "true",  ["moped_forward"] = "true",  ["motorcycle_forward"] = "true",  ["pedestrian_forward"] = "true",  ["bike_forward"] = "true"},
["residential_link"] =  {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["taxi_forward"] = "true",  ["moped_forward"] = "true",  ["motorcycle_forward"] = "true",  ["pedestrian_forward"] = "true",  ["bike_forward"] = "true"},
["service"] =           {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["taxi_forward"] = "true",  ["moped_forward"] = "true",  ["motorcycle_forward"] = "true",  ["pedestrian_forward"] = "true",  ["bike_forward"] = "true"},
["tertiary"] =          {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["taxi_forward"] = "true",  ["moped_forward"] = "true",  ["motorcycle_forward"] = "true",  ["pedestrian_forward"] = "true",  ["bike_forward"] = "true"},
["tertiary_link"] =     {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["taxi_forward"] = "true",  ["moped_forward"] = "true",  ["motorcycle_forward"] = "true",  ["pedestrian_forward"] = "true",  ["bike_forward"] = "true"},
["road"] =              {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["taxi_forward"] = "true",  ["moped_forward"] = "true",  ["motorcycle_forward"] = "true",  ["pedestrian_forward"] = "true",  ["bike_forward"] = "true"},
["track"] =             {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["taxi_forward"] = "true",  ["moped_forward"] = "true",  ["motorcycle_forward"] = "true",  ["pedestrian_forward"] = "true",  ["bike_forward"] = "true"},
["unclassified"] =      {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["taxi_forward"] = "true",  ["moped_forward"] = "true",  ["motorcycle_forward"] = "true",  ["pedestrian_forward"] = "true",  ["bike_forward"] = "true"},
["undefined"] =         {["auto_forward"] = "false", ["truck_forward"] = "false", ["bus_forward"] = "false", ["taxi_forward"] = "false", ["moped_forward"] = "false", ["motorcycle_forward"] = "false", ["pedestrian_forward"] = "false", ["bike_forward"] = "false"},
["unknown"] =           {["auto_forward"] = "false", ["truck_forward"] = "false", ["bus_forward"] = "false", ["taxi_forward"] = "false", ["moped_forward"] = "false", ["motorcycle_forward"] = "false", ["pedestrian_forward"] = "false", ["bike_forward"] = "false"},
["living_street"] =     {["auto_forward"] = "true",  ["truck_forward"] = "true",  ["bus_forward"] = "true",  ["taxi_forward"] = "true",  ["moped_forward"] = "true",  ["motorcycle_forward"] = "true",  ["pedestrian_forward"] = "true",  ["bike_forward"] = "true"},
["footway"] =           {["auto_forward"] = "false", ["truck_forward"] = "false", ["bus_forward"] = "false", ["taxi_forward"] = "false", ["moped_forward"] = "false", ["motorcycle_forward"] = "false", ["pedestrian_forward"] = "true",  ["bike_forward"] = "false"},
["pedestrian"] =        {["auto_forward"] = "false", ["truck_forward"] = "false", ["bus_forward"] = "false", ["taxi_forward"] = "false", ["moped_forward"] = "false", ["motorcycle_forward"] = "false", ["pedestrian_forward"] = "true",  ["bike_forward"] = "false"},
["steps"] =             {["auto_forward"] = "false", ["truck_forward"] = "false", ["bus_forward"] = "false", ["taxi_forward"] = "false", ["moped_forward"] = "false", ["motorcycle_forward"] = "false", ["pedestrian_forward"] = "true",  ["bike_forward"] = "true"},
["bridleway"] =         {["auto_forward"] = "false", ["truck_forward"] = "false", ["bus_forward"] = "false", ["taxi_forward"] = "false", ["moped_forward"] = "false", ["motorcycle_forward"] = "false", ["pedestrian_forward"] = "false", ["bike_forward"] = "false"},
["construction"] =      {["auto_forward"] = "false", ["truck_forward"] = "false", ["bus_forward"] = "false", ["taxi_forward"] = "false", ["moped_forward"] = "false", ["motorcycle_forward"] = "false", ["pedestrian_forward"] = "false", ["bike_forward"] = "false"},
["cycleway"] =          {["auto_forward"] = "false", ["truck_forward"] = "false", ["bus_forward"] = "false", ["taxi_forward"] = "false", ["moped_forward"] = "false", ["motorcycle_forward"] = "false", ["pedestrian_forward"] = "false", ["bike_forward"] = "true"},
["path"] =              {["auto_forward"] = "false", ["truck_forward"] = "false", ["bus_forward"] = "false", ["taxi_forward"] = "false", ["moped_forward"] = "false", ["motorcycle_forward"] = "false", ["pedestrian_forward"] = "true",  ["bike_forward"] = "true"},
["bus_guideway"] =      {["auto_forward"] = "false", ["truck_forward"] = "false", ["bus_forward"] = "true",  ["taxi_forward"] = "false", ["moped_forward"] = "false", ["motorcycle_forward"] = "false", ["pedestrian_forward"] = "false", ["bike_forward"] = "false"}
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

--the default speed for tracks is lowered after
--the call to default_speed
default_speed = {
[0] = 105,
[1] = 90,
[2] = 75,
[3] = 60,
[4] = 50,
[5] = 40,
[6] = 35,
[7] = 25
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
["emergency"] = "false",
["psv"] = "false",
["permit"] = "true",
["residents"] = "true"
}

private = {
["private"] = "true",
["destination"] = "true",
["customers"] = "true",
["delivery"] = "true",
["permit"] = "true",
["residents"] = "true"
}

no_thru_traffic = {
["destination"] = "true",
["customers"] = "true",
["delivery"] = "true",
["permit"] = "true",
["residents"] = "true"
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
["allowed"] = "true",
["permit"] = "true",
["residents"] = "true"
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
["agricultural"] = "false",
["permit"] = "true",
["residents"] = "true"
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
["footway"] = "true",
["permit"] = "true",
["residents"] = "true"
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
["assisted"] = "true",
["permit"] = "true",
["residents"] = "true"
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

taxi = {
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
["taxi"] = "true",
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
["destination;delivery"] = "true",
["permit"] = "true",
["residents"] = "true"
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
["official"] = "true",
["permit"] = "true",
["residents"] = "true"
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
["cyclestreet"] = "true",
["crossing"] = "true"
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
["false"] = "false",
["-1"] = "true",
["yes"] = "true",
["true"] = "true",
["1"] = "true",
["reversible"] = "false",
["alternating"] = "false"
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
["allowed"] = 1,
["permit"] = 1,
["residents"] = 1
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
["official"] = 4,
["permit"] = 4,
["residents"] = 4
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
["footway"] = 2,
["permit"] = 2,
["residents"] = 2
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
["assisted"] = 256,
["permit"] = 256,
["residents"] = 256
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
["agricultural"] = 0,
["permit"] = 512,
["residents"] = 512
}

motor_cycle_node = {
["yes"] = 1024,
["private"] = 1024,
["no"] = 0,
["permissive"] = 1024,
["agricultural"] = 0,
["delivery"] = 1024,
["designated"] = 1024,
["discouraged"] = 0,
["forestry"] = 0,
["destination"] = 1024,
["customers"] = 1024,
["official"] = 0,
["public"] = 1024,
["restricted"] = 1024,
["allowed"] = 1024
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

taxi_node = {
["no"] = 0,
["yes"] = 32,
["designated"] = 32,
["urban"] = 32,
["permissive"] = 32,
["restricted"] = 32,
["destination"] = 32,
["delivery"] = 0,
["official"] = 0
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
["destination;delivery"] = 8,
["permit"] = 8,
["residents"] = 8
}

psv_bus_node = {
["bus"] = 64,
["no"] = 0,
["yes"] = 64,
["designated"] = 64,
["permissive"] = 64,
["1"] = 64,
["2"] = 64
}

psv_taxi_node = {
["taxi"] = 32,
["no"] = 0,
["yes"] = 32,
["designated"] = 32,
["permissive"] = 32,
["1"] = 32,
["2"] = 32
}

function round(val, n)
  if (n) then
    return math.floor( (val * 10^n) + 0.5) / (10^n)
  else
    return math.floor(val+0.5)
  end
end

function restriction_prefix(restriction_str)
  --not a string
  if restriction_str == nil then
    return nil
  end

  --find where the restriction type ends.
  --format looks like
  --restriction:conditional=no_left_turn @ (07:00-09:00,15:30-17:30)
  local index = 0
  local found = false
  for c in restriction_str:gmatch"." do
    if c == "@" then
      found = true
      break
    end
    if c ~= " " then
      index = index + 1
    end
  end

  --@ symbol did not exist
  if found == false then
    return nil
  end

  --return the type of restriction
  return restriction_str:sub(0, index)
end

function restriction_suffix(restriction_str)
  --not a string
  if restriction_str == nil then
    return nil
  end

  --find where the restriction type ends.
  --format looks like
  --restriction:conditional=no_left_turn @ (07:00-09:00,15:30-17:30)
  local index = 0
  local found = false
  for c in restriction_str:gmatch"." do

    if found == true then
      if c ~= " " then
        index = index + 1
        break
      end
    elseif c == "@" then
      found = true
    end
    index = index + 1
  end

  --@ symbol did not exist
  if found == false then
    return nil
  end

  --return the date and time information for the restriction
  return restriction_str:sub(index, string.len(restriction_str))
end


--convert the numeric (non negative) number portion at the beginning of the string
function numeric_prefix(num_str, allow_decimals)
  --not a string
  if num_str == nil then
    return nil
  end

  --find where the numbers stop
  local index = 0
  -- flag to say if we've seen a decimal dot already. we shouldn't allow two,
  -- otherwise the call to tonumber() might fail.
  local seen_dot = false
  for c in num_str:gmatch"." do
    if tonumber(c) == nil then
      if c == "." then
        if allow_decimals == false or seen_dot then
           break
        end
        seen_dot = true
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

  --Returns the substring that's numeric at the start of num_str
  return num_str:sub(0, index)
end

--normalize a speed value
function normalize_speed(speed)
  --grab the number prefix
  local num = tonumber(numeric_prefix(speed, false))

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
          return round(tonumber(num),2)
        end
      end

      if w:sub(-3) == "ton" or w:sub(-4) == "tons" then
         if (num .. "ton" == w) or (num .. "tons" == w) then
           return round(tonumber(num),2)
         end
      end

      if w:sub(-2) == "lb" or w:sub(-3) == "lbs" then
        if (num .. "lb" == w) or (num .. "lbs" == w) then
          return round((tonumber(num)/2000),2) -- convert to tons
        end
      end

      if w:sub(-2) == "kg" then
        if (num .. "kg" == w) then
          return round((tonumber(num)/1000),2)
        end
      end
      return round(tonumber(num),2) --3.5
    end
  end
  return nil
end

function normalize_measurement(measurement)

  if measurement then
    -- turn commas into dots to handle European-style decimal separators
    measurement = measurement:gsub(",", ".")

    -- handle the simple case: it's just a plain number
    local num = tonumber(measurement)
    if num then
      return round(num, 2)
    end

    -- more complicated case, try some Lua patterns. they're almost like regular
    -- expressions, so there'll probably be some unintended consequences!
    --
    -- because we want to parse compount expressions such as 3ft6in, then we use
    -- an accumulator to sum up each term in meters. this has the unintended
    -- side-effect that 10m6ft would also be valid... but whatever.
    local sum = 0
    local count = 0
    for item, unit in measurement:gmatch("(%d+[.,]?%d*) *([a-zA-Z\"\']*)") do

      -- just in case the pattern above matches some things that tonumber()
      -- disagrees are valid numbers, we'll bail here if we don't get a valid
      -- number.
      item_num = tonumber(item)
      if not item_num then
        return nil
      end

      -- case shouldn't matter for units, so just lower case everything before
      -- comparing.
      unit = unit:lower()

      if unit == "m" or unit == "meter" or unit == "meters" then
        sum = sum + item_num
      elseif unit == "cm" then
        sum = sum + item_num * 0.01
      elseif (unit == "ft" or unit == "feet" or unit == "foot" or
              unit == "'") then
        sum = sum + item_num * 0.3048
      elseif (unit == "in" or unit == "inches" or unit == "inch" or
              unit == "\"" or unit == "''") then
        sum = sum + item_num * 0.0254
      else
        -- unknown unit! bail!
        return nil
      end

      -- increment counter, so we can check how many parts we parsed (mainly to
      -- make sure it's not none at all).
      count = count + 1
    end

    if count > 0 then
      -- if we got here, then the units seem valid. yay! now we round to two
      -- decimal digits, because precision of less than a centimeter seems
      -- unnecessary.
      return round(sum, 2)
    end
  end
  return nil
end

-- Returns true if the only payment types present are cash. Example payment kv's look like:
-- payment:cash=yes
-- payment:credit_cards=no
-- There can be multiple payment types on a given way/node. This routine determines
-- if the payment types on the way/node are all cash types. There are (at the moment) 60
-- types of payments, but only three are cash: cash, notes, coins.
-- Examining the types of values you might find for 'payment:coins' the predominant
-- usages are 'yes' and 'no'. However, there are also some values like '$0.35' and 'euro'.
-- Hence, this routine considers ~'NO' an affirmative value.
function is_cash_only_payment(kv)
  local allows_cash_payment = false
  local allows_noncash_payment = false
  for key, value in pairs(kv) do
    if string.sub(key, 1, 8) == "payment:" then
      local payment_type = string.sub(key, 9, -1)
      local is_cash_payment_type = payment_type == "cash" or payment_type == "notes" or payment_type == "coins"
      if (is_cash_payment_type == true and allows_cash_payment == false) then
        allows_cash_payment = string.upper(value) ~= "NO"
      end
      if (is_cash_payment_type == false and allows_noncash_payment == false) then
        allows_noncash_payment = string.upper(value) ~= "NO"
      end
    end
  end

  return allows_cash_payment == true and allows_noncash_payment == false
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
      kv["taxi_forward"] = "false"
      kv["moped_forward"] = "false"
      kv["motorcycle_forward"] = "false"
      kv["pedestrian_forward"] = "false"
      kv["bike_forward"] = "false"

      kv["auto_backward"] = "false"
      kv["truck_backward"] = "false"
      kv["bus_backward"] = "false"
      kv["taxi_backward"] = "false"
      kv["moped_backward"] = "false"
      kv["motorcycle_backward"] = "false"
      kv["pedestrian_backward"] = "false"
      kv["bike_backward"] = "false"
    elseif kv["vehicle"] == "no" then --don't change ped access.
      kv["auto_forward"] = "false"
      kv["truck_forward"] = "false"
      kv["bus_forward"] = "false"
      kv["taxi_forward"] = "false"
      kv["moped_forward"] = "false"
      kv["motorcycle_forward"] = "false"
      kv["bike_forward"] = "false"

      kv["auto_backward"] = "false"
      kv["truck_backward"] = "false"
      kv["bus_backward"] = "false"
      kv["taxi_backward"] = "false"
      kv["moped_backward"] = "false"
      kv["motorcycle_backward"] = "false"
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

    --check for taxi_forward overrides
    kv["taxi_forward"] = taxi[kv["taxi"]] or psv[kv["psv"]] or psv[kv["lanes:psv:forward"]] or motor_vehicle[kv["motor_vehicle"]] or kv["taxi_forward"]
    kv["taxi_tag"] = taxi[kv["taxi"]] or psv[kv["psv"]] or psv[kv["lanes:psv:forward"]] or motor_vehicle[kv["motor_vehicle"]] or nil

    --check for ped overrides
    kv["pedestrian_forward"] = foot[kv["foot"]] or foot[kv["pedestrian"]] or kv["pedestrian_forward"]
    kv["foot_tag"] = foot[kv["foot"]] or foot[kv["pedestrian"]] or nil

    --check for bike_forward overrides
    kv["bike_forward"] = bicycle[kv["bicycle"]] or cycleway[kv["cycleway"]] or bicycle[kv["bicycle_road"]] or bicycle[kv["cyclestreet"]] or kv["bike_forward"]
    kv["bike_tag"] = bicycle[kv["bicycle"]] or cycleway[kv["cycleway"]] or bicycle[kv["bicycle_road"]] or bicycle[kv["cyclestreet"]] or nil

    --check for moped forward overrides
    kv["moped_forward"] = moped[kv["moped"]] or moped[kv["mofa"]] or motor_vehicle[kv["motor_vehicle"]] or kv["moped_forward"]
    kv["moped_tag"] = moped[kv["moped"]] or moped[kv["mofa"]] or motor_vehicle[kv["motor_vehicle"]] or nil

    --check for motorcycle forward overrides
    kv["motorcycle_forward"] = motor_vehicle[kv["motorcycle"]] or motor_vehicle[kv["motor_vehicle"]] or kv["motorcycle_forward"]
    kv["motorcycle_tag"] = motor_vehicle[kv["motorcycle"]] or motor_vehicle[kv["motor_vehicle"]] or nil

    if kv["bike_tag"] == nil then
      if kv["sac_scale"] == "hiking" then
        kv["bike_forward"] = "true"
        kv["bike_tag"] = "true"
      elseif kv["sac_scale"] then
        kv["bike_forward"] = "false"
      end
    end

    if kv["access"] == "psv" then
      kv["taxi_forward"] = "true"
      kv["taxi_tag"] = "true"

      kv["bus_forward"] = "true"
      kv["bus_tag"] = "true"
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
      kv["taxi_forward"] = "false"
      kv["moped_forward"] = "false"
      kv["motorcycle_forward"] = "false"
      kv["pedestrian_forward"] = "false"
      kv["bike_forward"] = "false"

      kv["auto_backward"] = "false"
      kv["truck_backward"] = "false"
      kv["bus_backward"] = "false"
      kv["taxi_backward"] = "false"
      kv["moped_backward"] = "false"
      kv["motorcycle_backward"] = "false"
      kv["pedestrian_backward"] = "false"
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

      --check for taxi_forward overrides
      kv["taxi_forward"] = taxi[kv["taxi"]] or psv[kv["psv"]] or psv[kv["lanes:psv:forward"]] or motor_vehicle[kv["motor_vehicle"]] or default_val
      kv["taxi_tag"] = taxi[kv["taxi"]] or psv[kv["psv"]] or psv[kv["lanes:psv:forward"]] or motor_vehicle[kv["motor_vehicle"]] or nil

      --check for ped overrides
      kv["pedestrian_forward"] = foot[kv["foot"]] or foot[kv["pedestrian"]] or ped_val
      kv["foot_tag"] = foot[kv["foot"]] or foot[kv["pedestrian"]] or nil

      --check for bike_forward overrides
      kv["bike_forward"] = bicycle[kv["bicycle"]] or cycleway[kv["cycleway"]] or bicycle[kv["bicycle_road"]] or bicycle[kv["cyclestreet"]] or default_val
      kv["bike_tag"] = bicycle[kv["bicycle"]] or cycleway[kv["cycleway"]] or bicycle[kv["bicycle_road"]] or bicycle[kv["cyclestreet"]] or nil

      --check for moped forward overrides
      kv["moped_forward"] = moped[kv["moped"]] or moped[kv["mofa"]] or motor_vehicle[kv["motor_vehicle"]] or default_val
      kv["moped_tag"] = moped[kv["moped"]] or moped[kv["mofa"]] or motor_vehicle[kv["motor_vehicle"]] or nil

      --check for motorcycle forward overrides
      kv["motorcycle_forward"] = motor_vehicle[kv["motorcycle"]] or motor_vehicle[kv["motor_vehicle"]] or default_val
      kv["motorcycle_tag"] = motor_vehicle[kv["motorcycle"]] or motor_vehicle[kv["motor_vehicle"]] or nil

      if kv["bike_tag"] == nil then
        if kv["sac_scale"] == "hiking" then
          kv["bike_forward"] = "true"
          kv["bike_tag"] = "true"
        elseif kv["sac_scale"] then
          kv["bike_forward"] = "false"
        end
      end

      if kv["access"] == "psv" then
        kv["taxi_forward"] = "true"
        kv["taxi_tag"] = "true"

        kv["bus_forward"] = "true"
        kv["bus_tag"] = "true"
      end

      if kv["motorroad"] == "yes" then
        kv["motorroad_tag"] = "true"
      end
    end
  end

  --TODO: handle Time conditional restrictions if available for HOVs with oneway = reversible
  if ((kv["access"] == "permissive" or kv["access"] == "hov" or kv["access"] == "taxi") and kv["oneway"] == "reversible") then

    -- for now enable only for buses if the tag exists and they are allowed.
    if (kv["bus_forward"] == "true") then
      kv["auto_forward"] = "false"
      kv["truck_forward"] = "false"
      kv["pedestrian_forward"] = "false"
      kv["bike_forward"] = "false"
      kv["moped_forward"] = "false"
      kv["motorcycle_forward"] = "false"
    else
      -- by returning 1 we will toss this way
      return 1
    end
  end

  --service=driveway means all are routable
  if kv["service"] == "driveway" and kv["access"] == nil then
    kv["auto_forward"] = "true"
    kv["truck_forward"] = "true"
    kv["bus_forward"] = "true"
    kv["taxi_forward"] = "true"
    kv["pedestrian_forward"] = "true"
    kv["bike_forward"] = "true"
    kv["moped_forward"] = "true"
    kv["motorcycle_forward"] = "true"
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

  if kv["oneway:taxi"] == nil and kv["oneway:psv"] ~= nil then
    kv["oneway:taxi"] = kv["oneway:psv"]
  end

  if ((kv["oneway"] == "yes" and kv["oneway:taxi"] == "no") or kv["taxi:backward"] == "yes" or kv["taxi:backward"] == "designated") then
    kv["taxi_backward"] = "true"
  end

  if kv["taxi_backward"] == nil or kv["taxi_backward"] == "false" then
    kv["taxi_backward"] = psv[kv["lanes:psv:backward"]] or "false"
  end

  if kv["taxi_backward"] == "true" then
    oneway_taxi = oneway[kv["oneway:taxi"]]
    if (oneway_taxi == "false" and kv["taxi:backward"] == "yes") then
      oneway_taxi = "true"
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

  if kv["motorcycle_backward"] == nil then
    kv["motorcycle_backward"] = "false"
  end

  if ((kv["oneway"] == "yes" and kv["oneway:motorcycle"] == "no") or kv["motorcycle:backward"] == "yes") then
    kv["motorcycle_backward"] = "true"
  end

  if kv["motorcycle_backward"] == "true" then
    oneway_motorcycle = oneway[kv["oneway:motorcycle"]]
  end

  if kv["pedestrian_backward"] == nil then
      kv["pedestrian_backward"] = "false"
  end

  if ((kv["oneway"] == "yes" and kv["oneway:foot"] == "no") or kv["foot:backward"] == "yes") then
    kv["pedestrian_backward"] = "true"
  end

  if kv["pedestrian_backward"] == "true" then
    oneway_foot = oneway[kv["oneway:foot"]]
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
    if kv["taxi_backward"] == "true" then
      if (oneway_taxi == "true") then --taxi only in reverse on a taxi path.
        kv["taxi_forward"] = "false"
      elseif oneway_taxi == "false" then --taxi in both directions on a taxi path.
        kv["taxi_forward"] = "true"
      end
    end
    if kv["moped_backward"] == "true" then
      if (oneway_moped == "true") then --moped only in reverse direction on street
        kv["moped_forward"] = "false"
      elseif oneway_moped == "false" then
        kv["moped_forward"] = "true"
      end
    end
    if kv["motorcycle_backward"] == "true" then
      if (oneway_motorcycle == "true") then --motorcycle only in reverse direction on street
        kv["motorcycle_forward"] = "false"
      elseif oneway_motorcycle == "false" then
        kv["motorcycle_forward"] = "true"
      end
    end
    if (kv["highway"] == "footway" or kv["highway"] == "pedestrian" or kv["highway"] == "steps" or kv["highway"] == "path" or kv["oneway:foot"]) then --don't apply oneway tag unless oneway:foot or pedestrian only way
      if kv["pedestrian_backward"] == "true" then
        if (oneway_foot == "true") then --pedestrian only in reverse direction on street
          kv["pedestrian_forward"] = "false"
        elseif oneway_foot == "false" then
          kv["pedestrian_forward"] = "true"
        end
      end
    else
      kv["pedestrian_backward"] = kv["pedestrian_forward"]
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

    if (kv["taxi_backward"] == "false" and kv["oneway:taxi"] ~= "-1" and
       (kv["oneway:taxi"] == nil or oneway[kv["oneway:taxi"]] == false)) then
      kv["taxi_backward"] = kv["taxi_forward"]
    end

    if (kv["moped_backward"] == "false" and (kv["oneway:moped"] == nil or oneway[kv["oneway:moped"]] == false or kv["oneway:moped"] == "no") and
       (kv["oneway:mofa"] == nil or oneway[kv["oneway:mofa"]] == false or kv["oneway:mofa"] == "no")) then
      kv["moped_backward"] = kv["moped_forward"]
    end

    if (kv["motorcycle_backward"] == "false" and kv["oneway:motorcycle"] ~= "-1" and
       (kv["oneway:motorcycle"] == nil or oneway[kv["oneway:motorcycle"]] == false or kv["oneway:motorcycle"] == "no")) then
      kv["motorcycle_backward"] = kv["motorcycle_forward"]
    end

    if (kv["pedestrian_backward"] == "false" and (kv["oneway:foot"] == nil or oneway[kv["oneway:foot"]] == false or kv["oneway:foot"] == "no")) then
      kv["pedestrian_backward"] = kv["pedestrian_forward"]
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

    forwards = kv["taxi_forward"]
    kv["taxi_forward"] = kv["taxi_backward"]
    kv["taxi_backward"] = forwards

    forwards = kv["bike_forward"]
    kv["bike_forward"] = kv["bike_backward"]
    kv["bike_backward"] = forwards

    forwards = kv["moped_forward"]
    kv["moped_forward"] = kv["moped_backward"]
    kv["moped_backward"] = forwards

    forwards = kv["motorcycle_forward"]
    kv["motorcycle_forward"] = kv["motorcycle_backward"]
    kv["motorcycle_backward"] = forwards

    forwards = kv["pedestrian_forward"]
    kv["pedestrian_forward"] = kv["pedestrian_backward"]
    kv["pedestrian_backward"] = forwards

  end

  if kv["oneway:bicycle"] == "-1" then
    local forwards = kv["bike_forward"]
    kv["bike_forward"] = kv["bike_backward"]
    kv["bike_backward"] = forwards
  end

  if (kv["oneway:moped"] == "-1" or kv["oneway:mofa"] == "-1") then
    local forwards = kv["moped_forward"]
    kv["moped_forward"] = kv["moped_backward"]
    kv["moped_backward"] = forwards
  end

  if kv["oneway:motorcycle"] == "-1" then
    local forwards = kv["motorcycle_forward"]
    kv["motorcycle_forward"] = kv["motorcycle_backward"]
    kv["motorcycle_backward"] = forwards
  end

  if kv["oneway:foot"] == "-1" then
    local forwards = kv["pedestrian_forward"]
    kv["pedestrian_forward"] = kv["pedestrian_backward"]
    kv["pedestrian_backward"] = forwards
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

  if kv["oneway:taxi"] == "-1" then
    local forwards = kv["taxi_forward"]
    kv["taxi_forward"] = kv["taxi_backward"]
    kv["taxi_backward"] = forwards
  end

  if kv["lanes:psv"] == "1" then
    kv["taxi_forward"] = "true"
    kv["taxi_backward"] = "false"
  elseif kv["lanes:psv"] == "2" then
    kv["taxi_forward"] = "true"
    kv["taxi_backward"] = "true"
  end

  --if none of the modes were set we are done looking at this
  if kv["auto_forward"] == "false" and kv["truck_forward"] == "false" and kv["bus_forward"] == "false" and
     kv["bike_forward"] == "false" and kv["emergency_forward"] == "false" and kv["moped_forward"] == "false" and
     kv["motorcycle_forward"] == "false" and kv["pedestrian_forward"] == "false" and
     kv["auto_backward"] == "false" and kv["truck_backward"] == "false" and kv["bus_backward"] == "false" and
     kv["bike_backward"] == "false" and kv["emergency_backward"] == "false" and kv["moped_backward"] == "false" and
     kv["motorcycle_backward"] == "false" and kv["pedestrian_backward"] == "false" then
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
     elseif use == nil and kv["highway"] == "service" then
        use = 11
     elseif kv["highway"] == "cycleway" then
        use = 20
     elseif kv["pedestrian_forward"] == "false" and kv["auto_forward"] == "false" and kv["auto_backward"] == "false" and (kv["bike_forward"] == "true" or kv["bike_backward"] == "true") then
        use = 20
     elseif (kv["highway"] == "footway" and kv["footway"] == "sidewalk") then
        use = 24
     elseif (kv["highway"] == "footway" and kv["footway"] == "crossing") then
        use = 32
     elseif kv["highway"] == "footway" then
        use = 25
     elseif kv["highway"] == "steps" then
        use = 26 --steps/stairs
     elseif kv["highway"] == "path" then
        use = 27
     elseif kv["highway"] == "pedestrian" then
        use = 28
     elseif kv["pedestrian_forward"] == "true" and
            kv["auto_forward"] == "false" and kv["auto_backward"] == "false" and
            kv["truck_forward"] == "false" and kv["truck_backward"] == "false" and
            kv["bus_forward"] == "false" and kv["bus_backward"] == "false" and
            kv["bike_forward"] == "false" and kv["bike_backward"] == "false" and
            kv["moped_forward"] == "false" and kv["moped_backward"] == "false" and
            kv["motorcycle_forward"] == "false" and kv["motorcycle_backward"] == "false" then
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

  if (kv["access"] == "emergency" or kv["emergency"] == "yes") and
      kv["auto_forward"] == "false" and kv["auto_backward"] == "false" and
      kv["truck_forward"] == "false" and kv["truck_backward"] == "false" and
      kv["bus_forward"] == "false" and kv["bus_backward"] == "false" and
      kv["bike_forward"] == "false" and kv["bike_backward"] == "false" and
      kv["moped_forward"] == "false" and kv["moped_backward"] == "false" and
      kv["motorcycle_forward"] == "false" and kv["motorcycle_backward"] == "false" then
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
    if kv["pedestrian_forward"] == "false" then
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

  if kv["maxspeed"] == "none" then
    --- special case unlimited speed limit (german autobahn)
    kv["max_speed"] = "unlimited"
  else
    kv["max_speed"] = normalize_speed(kv["maxspeed"])
  end

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

  lane_count = tonumber(numeric_prefix(kv["lanes"],false))
  if lane_count and lane_count > 15 then
    lane_count = nil
  end
  kv["lanes"] = lane_count

  lane_count = tonumber(numeric_prefix(kv["lanes:forward"],false))
  if lane_count and lane_count > 15 then
    lane_count = nil
  end
  kv["forward_lanes"] = lane_count

  lane_count = tonumber(numeric_prefix(kv["lanes:backward"],false))
  if lane_count and lane_count > 15 then
    lane_count = nil
  end
  kv["backward_lanes"] = lane_count

  kv["bridge"] = bridge[kv["bridge"]] or "false"

  -- TODO access:conditional
  if kv["seasonal"] and kv["seasonal"] ~= "no" then
    kv["seasonal"] = "true"
  end

  kv["hov_tag"] = "true"
  if (kv["hov"] and kv["hov"] == "no") then
    kv["hov_forward"] = "false"
    kv["hov_backward"] = "false"
  else
    kv["hov_forward"] = kv["auto_forward"]
    kv["hov_backward"] = kv["auto_backward"]
  end

  -- hov restrictions
  if ((kv["hov"] and kv["hov"] ~= "no") or kv["hov:lanes"] or kv["hov:minimum"]) then

    local only_hov_allowed = kv["hov"] == "designated"
    if only_hov_allowed then
      if kv["hov:lanes"] then
        for lane in (kv["hov:lanes"] .. '|'):gmatch("([^|]*)|") do
          if lane and lane ~= "designated" then
            only_hov_allowed = false
          end
        end
      end
    end

    if only_hov_allowed then
      -- If we get here we know the way is a true hov-lane (not mixed).
      -- As a result, none of the following costings can use it.
      -- (Okay, that's not exactly true, we do some wizardry in some of the
      -- costings to allow hov under certain conditions.)
      if (kv["auto_tag"] == nil) then
        kv["auto_forward"] = "false"
        kv["auto_backward"] = "false"
      end

      if (kv["truck_tag"] == nil) then
        kv["truck_forward"] = "false"
        kv["truck_backward"] = "false"
      end

      if (kv["foot_tag"] == nil) then
        kv["pedestrian_forward"] = "false"
        kv["pedestrian_backward"] = "false"
      end

      if (kv["bike_tag"] == nil) then
        kv["bike_forward"] = "false"
        kv["bike_backward"] = "false"
      end

      -- I want to be strict with the "hov:minimum" tag: I will only accept the
      -- values 2 or 3. We want to be strict because routing onto an HOV lane
      -- without the correct number of occupants is illegal.
      if (kv["hov:minimum"] == "2") then
        kv["hov_type"] = "HOV2";
      elseif (kv["hov:minimum"] == "3") then
        kv["hov_type"] = "HOV3";
      end

      -- HOV lanes are sometimes time-conditional and can change direction. We avoid
      -- these. Also, we expect "hov_type" to be set.
      local avoid_these_hovs = kv["oneway"] == "alternating" or kv["oneway"] == "reversible" or kv["oneway"] == "false" or
            kv["oneway:conditional"] ~= nil or kv["access:conditional"] ~= nil or kv["hov_type"] == nil

      if avoid_these_hovs then
        kv["hov_forward"] = "false"
        kv["hov_backward"] = "false"
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
  kv["turn:lanes"] = kv["turn:lanes"]
  kv["turn:lanes:forward"] = kv["turn:lanes:forward"]
  kv["turn:lanes:backward"] = kv["turn:lanes:backward"]

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
    bike_mask = bit.bor(bike_mask, 2)
  end
  if lref or kv["lcn"] == "yes" then
    bike_mask = bit.bor(bike_mask, 4)
  end
  if kv["mtb"] == "yes" then
    bike_mask = bit.bor(bike_mask, 8)
  end

  kv["bike_national_ref"] = nref
  kv["bike_regional_ref"] = rref
  kv["bike_local_ref"] = lref
  kv["bike_network_mask"] = bike_mask

  return 0
end

function nodes_proc (kv, nokeys)

  if kv["iso:3166_2"] then
    i, j = string.find(kv["iso:3166_2"], '-', 1, true)
    if i == 3 then
      if string.len(kv["iso:3166_2"]) == 6 or string.len(kv["iso:3166_2"]) == 5 then
        kv["state_iso_code"] = string.sub(kv["iso:3166_2"], 4)
      end
    elseif string.find(kv["iso:3166_2"], '-', 1, true) == nil then
      if string.len(kv["iso:3166_2"]) == 2 or  string.len(kv["iso:3166_2"]) == 3 then
        kv["state_iso_code"] = kv["iso:3166_2"]
      elseif string.len(kv["iso:3166_2"]) == 4 or  string.len(kv["iso:3166_2"]) == 5 then
        kv["state_iso_code"] = string.sub(kv["iso:3166_2"], 3)
      end
    end
  end

  --normalize a few tags that we care about
  local initial_access = access[kv["access"]]
  local access = initial_access or "true"

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
  local motorcycle_tag = motor_cycle_node[kv["motorcycle"]]

  if auto_tag == nil then
    auto_tag = motor_vehicle_tag
  end
  local bus_tag
  local taxi_tag

  if (kv["access"] == "psv") then
    bus_tag = 64
    taxi_tag = 32
  else
    bus_tag = bus_node[kv["bus"]]
    taxi_tag = taxi_node[kv["taxi"]]
  end

  if bus_tag == nil then
    bus_tag = psv_bus_node[kv["psv"]]
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

  if taxi_tag == nil then
    taxi_tag = psv_taxi_node[kv["psv"]]
  end
  --if taxi was not set and car is
  if taxi_tag == nil and auto_tag == 1 then
    taxi_tag = 32
  end

  --if truck was not set and car is
  if truck_tag == nil and auto_tag == 1 then
    truck_tag = 8
  end

  --must shut these off if motor_vehicle = 0
  if motor_vehicle_tag == 0 then
    hov_tag = hov_tag or 0
    bus_tag = bus_tag or 0
    taxi_tag = taxi_tag or 0
    truck_tag = truck_tag or 0
    moped_tag = moped_tag or 0
    motorcycle_tag = motorcycle_tag or 0
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
  --if access=private use allowed modes, but consider private_access tag as true.
  local auto = auto_tag or 1
  local truck = truck_tag or 8
  local bus = bus_tag or 64
  local taxi = taxi_tag or auto_tag or 32
  local foot = foot_tag or 2
  local wheelchair = wheelchair_tag or 256
  local bike = bike_tag or 4
  local emergency = emergency_tag or 16
  local hov = hov_tag or auto_tag or 128
  local moped = moped_tag or 512
  local motorcycle = motorcycle_tag or 1024

  --if access = false use tag if exists, otherwise no access for that mode.
  if (access == "false" or kv["vehicle"] == "no" or kv["hov"] == "designated") then
    auto = auto_tag or 0
    truck = truck_tag or 0
    bus = bus_tag or 0
    taxi = taxi_tag or 0

    --don't change ped if kv["vehicle"] == "no"
    if access == "false" or kv["hov"] == "designated" then
      foot = foot_tag or 0
    end

    wheelchair = wheelchair_tag or 0
    bike = bike_tag or 0
    moped = moped_tag or 0
    motorcycle = motorcycle_tag or 0
    emergency = emergency_tag or 0
    hov = hov_tag or 0
  end

  --check for gates, bollards, and sump_busters
  local gate = kv["barrier"] == "gate" or kv["barrier"] == "yes" or
    kv["barrier"] == "lift_gate" or kv["barrier"] == "swing_gate"
  local bollard = false
  local sump_buster = false

  if gate == false then
    --if there was a bollard cars can't get through it
    bollard = kv["barrier"] == "bollard" or kv["barrier"] == "block" or
      kv["barrier"] == "jersey_barrier" or kv["bollard"] == "removable" or false

    --if sump_buster then no access for auto, hov, and taxi unless a tag exists.
    sump_buster = kv["barrier"] == "sump_buster" or false

    --save the following as gates.
    if (bollard and (kv["bollard"] == "rising")) then
      gate = true
      bollard = false
    end

    --bollard = true shuts off access when access is not originally specified.
    if bollard == true and initial_access == nil then
      auto = auto_tag or 0
      truck = truck_tag or 0
      bus = bus_tag or 0
      taxi = taxi_tag or 0
      foot = foot_tag or 2
      wheelchair = wheelchair_tag or 256
      bike = bike_tag or 4
      moped = moped_tag or 0
      motorcycle = motorcycle_tag or 0
      emergency = emergency_tag or 0
      hov = hov_tag or 0
    --sump_buster = true shuts off access unless the tag exists.
    elseif sump_buster == true then
      auto = auto_tag or 0
      truck = truck_tag or 8
      bus = bus_tag or 64
      taxi = taxi_tag or 0
      foot = foot_tag or 2
      wheelchair = wheelchair_tag or 256
      bike = bike_tag or 4
      moped = moped_tag or 512
      motorcycle = motorcycle_tag or 1024
      emergency = emergency_tag or 16
      hov = hov_tag or 0
    end
  end

  --if nothing blocks access at this node assume access is allowed.
  if gate == false and bollard == false and sump_buster == false and access == "true" then
    if kv["highway"] == "crossing" or kv["railway"] == "crossing" or
       kv["footway"] == "crossing" or kv["cycleway"] == "crossing" or
       kv["foot"] == "crossing" or kv["bicycle"] == "crossing" or
       kv["pedestrian"] == "crossing" or kv["crossing"] then
         auto = auto_tag or 1
         truck = truck_tag or 8
         bus = bus_tag or 64
         taxi = taxi_tag or 32
         foot = foot_tag or 2
         wheelchair = wheelchair_tag or 256
         bike = bike_tag or 4
         moped = moped_tag or 512
         motorcycle = motorcycle_tag or 1024
         emergency = emergency_tag or 16
         hov = hov_tag or 128
    end
  end

  --store the gate and bollard info
  kv["gate"] = tostring(gate)
  kv["bollard"] = tostring(bollard)
  kv["sump_buster"] = tostring(sump_buster)

  if kv["barrier"] == "border_control" then
    kv["border_control"] = "true"
  elseif kv["barrier"] == "toll_booth" then
    kv["toll_booth"] = "true"
    if is_cash_only_payment(kv) then
      kv["cash_only_toll"] = "true"
    end
  elseif kv["highway"] == "toll_gantry" then
    kv["toll_gantry"] = "true"
  end

  if kv["amenity"] == "bicycle_rental" or (kv["shop"] == "bicycle" and kv["service:bicycle:rental"] == "yes") then
    kv["bicycle_rental"] = "true"
  end

  if kv["traffic_signals:direction"] == "forward" then
    kv["forward_signal"] = "true"

    if kv["public_transport"] == nil and kv["name"] then
       kv["junction"] = "named"
    end
  end

  if kv["traffic_signals:direction"] == "backward" then
    kv["backward_signal"] = "true"

    if kv["public_transport"] == nil and kv["name"] then
       kv["junction"] = "named"
    end
  end

  if kv["highway"] == "stop" then
    if kv["direction"] == "both" then
      kv["forward_stop"] = "true"
      kv["backward_stop"] = "true"
    elseif kv["direction"] == "forward" then
      kv["forward_stop"] = "true"
    elseif kv["direction"] == "backward" or kv["direction"] == "reverse" then
      kv["backward_stop"] = "true"
    elseif kv["direction"] ~= nil and kv["stop"] == nil then
      kv["highway"] = nil
    end
  end

  if kv["highway"] == "give_way" then
    if kv["direction"] == "both" then
      kv["forward_yield"] = "true"
      kv["backward_yield"] = "true"
    elseif kv["direction"] == "forward" then
      kv["forward_yield"] = "true"
    elseif kv["direction"] == "backward" or kv["direction"] == "reverse" then
      kv["backward_yield"] = "true"
    elseif kv["direction"] ~= nil and kv["give_way"] == nil then
      kv["highway"] = nil
    end
  end

  if kv["public_transport"] == nil and kv["name"] then
    if kv["highway"] == "traffic_signals" then
       if kv["junction"] ~= "yes" then
          kv["junction"] = "named"
       end
    elseif kv["junction"] == "yes" or kv["reference_point"] == "yes" then
       kv["junction"] = "named"
    end
  end

  kv["private"] = private[kv["access"]] or private[kv["motor_vehicle"]] or "false"

  --store a mask denoting access
  kv["access_mask"] = bit.bor(auto, emergency, truck, bike, foot, wheelchair, bus, hov, moped, motorcycle, taxi)

  --if no information about access is given.
  if initial_access == nil and auto_tag == nil and truck_tag == nil and bus_tag == nil and taxi_tag == nil and
   foot_tag == nil and wheelchair_tag == nil and bike_tag == nil and moped_tag == nil and
   motorcycle_tag == nil and emergency_tag == nil and hov_tag == nil then
    kv["tagged_access"] = 0
  else
    kv["tagged_access"] = 1
  end

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

     local restrict = restriction[kv["restriction"]] or restriction[restriction_prefix(kv["restriction:conditional"])]

     local restrict_type = restriction[kv["restriction:hgv"]] or restriction[kv["restriction:emergency"]] or
                           restriction[kv["restriction:taxi"]] or restriction[kv["restriction:motorcar"]] or
                           restriction[kv["restriction:bus"]] or restriction[kv["restriction:bicycle"]] or
                           restriction[kv["restriction:hazmat"]] or restriction[kv["restriction:motorcycle"]] or
			   restriction[kv["restriction:foot"]]

     --restrictions with type win over just restriction key.  people enter both.
     if restrict_type ~= nil then
       restrict = restrict_type
     end

     if kv["type"] == "restriction" or kv["restriction:conditional"] then

       if restrict ~= nil then

         kv["restriction:conditional"] = restriction_suffix(kv["restriction:conditional"])
         kv["restriction:hgv"] = restriction[kv["restriction:hgv"]]
         kv["restriction:emergency"] = restriction[kv["restriction:emergency"]]
         kv["restriction:taxi"] = restriction[kv["restriction:taxi"]]
         kv["restriction:motorcar"] = restriction[kv["restriction:motorcar"]]
         kv["restriction:motorcycle"] = restriction[kv["restriction:motorcycle"]]
         kv["restriction:bus"] = restriction[kv["restriction:bus"]]
         kv["restriction:bicycle"] = restriction[kv["restriction:bicycle"]]
         kv["restriction:hazmat"] = restriction[kv["restriction:hazmat"]]
         kv["restriction:foot"] = restriction[kv["restriction:foot"]]

         if restrict_type == nil then
           kv["restriction"] = restrict
         else
           kv["restriction"] = nil
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
         bike_mask = bit.bor(bike_mask, 1)
       elseif kv["network"] == "rcn" then
         bike_mask = bit.bor(bike_mask, 2)
       elseif kv["network"] == "lcn" then
         bike_mask = bit.bor(bike_mask, 4)
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
