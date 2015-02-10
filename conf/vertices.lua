access = {
["no"] = 0,
["official"] = 0,
["private"] = 0,
["destination"] = 0,
["yes"] = 7,
["permissive"] = 7,
["agricultural"] = 0,
["customers"] = 7
}

motor_vehicle = {
["no"] = 0,
["yes"] = 1,
["agricultural"] = 0,
["destination"] = 0,
["private"] = 0,
["forestry"] = 0,
["designated"] = 1,
["permissive"] = 1
}

bicycle = {
["yes"] = 2,
["designated"] = 2,
["dismount"] = 2,
["no"] = 0,
["lane"] = 2,
["track"] = 2,
["shared"] = 2,
["shared_lane"] = 2,
["sidepath"] = 2,
["share_busway"] = 2,
["none"] = 0
}

foot = {
["no"] = 0,
["yes"] = 4,
["designated"] = 4,
["permissive"] = 4,
["crossing"] = 4
}

restriction = {
["no_left_turn"] = 1,
["no_right_turn"] = 2,
["no_straight_on"] = 3,
["no_u_turn"] = 4,
["only_right_turn"] = 5,
["only_left_turn"] = 6,
["only_straight_on"] = 7,
["no_entry"] = 8,
["no_exit"]  = 9
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

function nodes_proc (kv, nokeys)
  --normalize a few tags that we care about
  local access = access[kv["access"]] or 7
  local foot = foot[kv["foot"]] or bit32.band(access, 4)
  local bike = bicycle[kv["bicycle"]] or bit32.band(access, 2)
  local auto = motor_vehicle[kv["motor_vehicle"]]
  if auto == nil then
    auto = motor_vehicle[kv["motorcar"]]
  end
  auto = auto or bit32.band(access, 1)

  --check for gates and bollards
  local gate = kv["barrier"] == "gate" or kv["barrier"] == "lift_gate"
  local bollard = false
  if gate == false then
    --if there was a bollard cars can't get through it
    bollard = kv["barrier"] == "bollard" or kv["barrier"] == "block"

    --save the following as gates.
    if (bollard and (kv["bollard"] == "rising" or kv["bollard"] == "removable")) then
      gate = true
      bollard = false
    end

    auto = bollard and 0 or 1

  end

  if kv["amenity"] == "bicycle_rental" or (kv["shop"] == "bicycle" and kv["service:bicycle:rental"] == "yes")then
     kv["bicycle_rental"] = true
  end

  --store the gate and bollard info
  kv["gate"] = tostring(gate)
  kv["bollard"] = tostring(bollard)

  --store a mask denoting which modes of transport are allowed
  kv["modes_mask"] = bit32.bor(auto, bike, foot)

  return 0, kv
end

function ways_proc (keyvalues, nokeys)
  --we dont care about ways at all so filter all of them
  return 1, keyvalues, 0, 0
end

function rels_proc (kv, nokeys)
  if (kv["type"] == "route" or kv["type"] == "restriction" or
     (kv["type"] == "boundary" and kv["boundary"] == "administrative" and
     (kv["admin_level"] == "2" or kv["admin_level"] == "4"))) then

     if kv["type"] == "restriction" then

       local restrict = restriction[kv["restriction"]]
       if restrict == nil then
         kv["restriction"] = 0
       else
         kv["restriction"] = restrict
       end

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

     return 0, kv
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

