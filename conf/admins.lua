--TODO: check if you can use lua type boolean instead of strings and pass that back to osm2pgsql
--with the hopes that they will become strings once they get back to c++ and then just work in
--postgres

--returns 1 if you should filter this way 0 otherwise
function filter_tags_generic(kv)
--  if (kv["boundary"] == "administrative" and
--     (kv["admin_level"] == "2" or kv["admin_level"] == "4")) then

     delete_tags = { 'FIXME', 'note', 'source' }

     for i,k in ipairs(delete_tags) do
        kv[k] = nil
     end

     return 0
--  end

--  return 1
end

function nodes_proc (kv, nokeys)
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
  if (kv["type"] == "boundary" and kv["boundary"] == "administrative" and
     (kv["admin_level"] == "2" or kv["admin_level"] == "4") or kv["admin_level"] == "6") then

     --save only states/prov for USA, MX, and CA.
     if (kv["name"] == "United States of America" or 
         kv["name"] == "Estados Unidos Mexicanos" or kv["name"] == "Canada") then
       return 1, kv
     end

     if kv["admin_level"] == "6" and kv["name"] ~= "District of Columbia" then
       return 1, kv
     end

     delete_tags = { 'FIXME', 'note', 'source' }

     for i,k in ipairs(delete_tags) do
        kv[k] = nil
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


