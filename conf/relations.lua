restriction = {
["no_left_turn"] = 0,
["no_right_turn"] = 1, 
["no_straight_on"] = 2, 
["no_u_turn"] = 3, 
["only_right_turn"] = 4,
["only_left_turn"] = 5, 
["only_straight_on"] = 6,
["no_entry"] = 7,
["no_exit"]  = 8
}


function nodes_proc (keyvalues, nokeys)
  --we dont care about nodes at all so filter all of them
  return 1, keyvalues
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
        kv["restriction"] = restriction[kv["restriction"]]
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
