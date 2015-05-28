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
["yes"] = 4,
["designated"] = 4,
["dismount"] = 4,
["no"] = 0,
["lane"] = 4,
["track"] = 4,
["shared"] = 4,
["shared_lane"] = 4,
["sidepath"] = 4,
["share_busway"] = 4,
["none"] = 0
}

foot = {
["no"] = 0,
["yes"] = 2,
["designated"] = 2,
["permissive"] = 2,
["crossing"] = 2
}

bus = {
["no"] = 0,
["yes"] = 64,
["designated"] = 64,
["permissive"] = 64,
["restricted"] = 64,
["destination"] = 0,
["delivery"] = 0
}

psv = {
["bus"] = 64,
["no"] = 0,
["yes"] = 64,
["1"] = 64,
["2"] = 64
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

function nodes_proc (kv, nokeys)
  --normalize a few tags that we care about
  local access = access[kv["access"]] or "true"
  local foot = foot[kv["foot"]] or 0
  local bike = bicycle[kv["bicycle"]] or 0
  local auto = motor_vehicle[kv["motorcar"]]
  if auto == nil then
    auto = motor_vehicle[kv["motor_vehicle"]]
  end
  local bus = bus[kv["bus"]]
  if bus == nil then
    bus = psv[kv["psv"]]
  end
  auto = auto or 0
  bus = bus or 0
  --access was set, but foot, bus, bike, and auto tags were not.
  if access == "true" and bit32.bor(auto, bike, foot, bus) == 0 then
    bus  = 64
    bike = 4
    foot = 2
    auto = 1
  end 

  --check for gates and bollards
  local gate = kv["barrier"] == "gate" or kv["barrier"] == "lift_gate"
  local bollard = false
  if gate == false then
    --if there was a bollard cars can't get through it
    bollard = kv["barrier"] == "bollard" or kv["barrier"] == "block" or kv["bollard"] == "removable"

    --save the following as gates.
    if (bollard and (kv["bollard"] == "rising")) then
      gate = true
      bollard = false
    end

    auto = (bollard and 0) or 1

  end

  --store the gate and bollard info
  kv["gate"] = tostring(gate)
  kv["bollard"] = tostring(bollard)

  if kv["barrier"] == "toll_booth" then
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
  kv["access_mask"] = bit32.bor(auto, bike, foot, bus)

  return 0, kv
end

function ways_proc (keyvalues, nokeys)
  --we dont care about ways at all so filter all of them
  return 1, keyvalues, 0, 0
end

function rels_proc (keyvalues, nokeys)
  --we dont care about rels at all so filter all of them
  return 1, keyvalues
end

function rel_members_proc (keyvalues, keyvaluemembers, roles, membercount)
  --because we filter all rels we never call this function
  membersuperseeded = {}
  for i = 1, membercount do
    membersuperseeded[i] = 0
  end

  return 1, keyvalues, membersuperseeded, 0, 0, 0
end


