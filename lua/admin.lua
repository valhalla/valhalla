--TODO: check if you can use lua type boolean instead of strings and pass that back to osm2pgsql
--with the hopes that they will become strings once they get back to c++ and then just work in
--postgres

drive_on_right = {
["Anguilla"] = "false",
["Antigua and Barbuda"] = "false",
["Australia"] = "false",
["Bangladesh"] = "false",
["Barbados"] = "false",
["Bermuda"] = "false",
["Bhutan"] = "false",
["Botswana"] = "false",
["British Virgin Islands"] = "false",
["Brunei Darussalam"] = "false",
["Cayman Islands"] = "false",
["Cook Islands"] = "false",
["Cyprus"] = "false",
["Dominica"] = "false",
["East Timor"] = "false",
["England"] = "false",
["Falkland Islands"] = "false",
["Grenada"] = "false",
["Guernsey"] = "false",
["Guyana"] = "false",
["Hong Kong"] = "false",
["India"] = "false",
["Indonesia"] = "false",
["Ireland"] = "false",
["Isle of Man"] = "false",
["Jamaica"] = "false",
["Japan"] = "false",
["Jersey"] = "false",
["Kenya"] = "false",
["Kiribati"] = "false",
["Lesotho"] = "false",
["Macao"] = "false",
["Malawi"] = "false",
["Malaysia"] = "false",
["Maldives"] = "false",
["Malta"] = "false",
["Mauritius"] = "false",
["Moçambique"] = "false",
["Montserrat"] = "false",
["Namibia"] = "false",
["Naoero"] = "false",
["Nepal"] = "false",
["New Zealand"] = "false",
["Niue"] = "false",
["Northern Ireland"] = "false",
["Pakistan"] = "false",
["Papua Niugini"] = "false",
["Pitcairn Islands"] = "false",
["Republic of Ireland"] = "false",
["Saint Helena, Ascension and Tristan da Cunha"] = "false",
["Saint Kitts and Nevis"] = "false",
["Saint Lucia"] = "false",
["Saint Vincent and the Grenadines"] = "false",
["Samoa"] = "false",
["Sesel"] = "false",
["Singapore"] = "false",
["Solomon Islands"] = "false",
["Soomaaliya"] = "false",
["South Africa"] = "false",
["Sri Lanka"] = "false",
["Suriname"] = "false",
["Alba / Scotland"] = "false",
["Swatini"] = "false",
["Tanzania"] = "false",
["Thailand"] = "false",
["The Bahamas"] = "false",
["Tokelau"] = "false",
["Tonga"] = "false",
["Trinidad and Tobago"] = "false",
["Turks and Caicos Islands"] = "false",
["Tuvalu"] = "false",
["Uganda"] = "false",
["United States Virgin Islands"] = "false",
["Viti"] = "false",
["Cymru / Wales"] = "false",
["Zambia"] = "false",
["Zimbabwe"] = "false"
}

allow_intersection_names = {
["Japan"] = "true",
["North Korea"] = "true",
["South Korea"] = "true",
["Nicaragua"] = "true"
}

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

  --we save admins as 2(country) or 4(state/prov).  
function rels_proc (kv, nokeys)
  if (kv["type"] == "boundary" and (kv["boundary"] == "administrative" or kv["boundary"] == "territorial") and
     (kv["admin_level"] == "2" or kv["admin_level"] == "3" or kv["admin_level"] == "4" or kv["admin_level"] == "6")) then


     if (kv["admin_level"] == "3" and kv["name"] ~= "Guyane" and kv["name"] ~= "Guadeloupe" and  kv["name"] ~= "La Réunion" and  
         kv["name"] ~= "Martinique" and kv["name"] ~= "Mayotte" and kv["name"] ~= "Saint-Pierre-et-Miquelon" and
         kv["name"] ~= "Saint-Barthélemy" and  kv["name"] ~= "Saint-Martin (France)" and kv["name"] ~= "Polynésie Française" and 
         kv["name"] ~= "Wallis-et-Futuna" and kv["name"] ~= "Nouvelle-Calédonie" and kv["name"] ~= "Île de Clipperton" and 
         kv["name"] ~= "Terres australes et antarctiques françaises" and kv["name:en"] ~= "Metropolitan France" and
         kv["name:en"] ~= "Hong Kong" and kv["name"] ~= "Metro Manila") then
        return 1, kv
     end

     if kv["admin_level"] == "6" and kv["name"] ~= "District of Columbia" then
       return 1, kv
     end

     if kv["admin_level"] == "2" then 
        if kv["name"] ==  "France" or kv["name"] == "United Kingdom" then
          return 1, kv
        elseif kv["name:en"] == "Abkhazia" or kv["name:en"] == "South Ossetia" then
          kv["admin_level"] = "4"
        end
     end

     if kv["name"] == "Metro Manila" then
        kv["admin_level"] = "4"
     end

     if kv["admin_level"] == "3" then
       kv["admin_level"] = "2"
       if kv["name:en"] == "Metropolitan France" then
         kv["name"] = "France"
         kv["iso_code"] = "FR"
       end
     end

     if kv["admin_level"] == "6" then
       kv["admin_level"] = "4"
     end

     if kv["admin_level"] == "2" then
       if kv["ISO3166-1:alpha2"] then
         kv["iso_code"] = kv["ISO3166-1:alpha2"]
       elseif kv["ISO3166-1"] then
         kv["iso_code"] = kv["ISO3166-1"]
       end
       if kv["name"] == "British Sovereign Base Areas" and kv["iso_code"] == nil then
         kv["iso_code"] = "GB"
       end
     elseif kv["admin_level"] == "4" then
       if kv["ISO3166-2"] then
         i, j = string.find(kv["ISO3166-2"], '-', 1, true)
         if i == 3 then
           if string.len(kv["ISO3166-2"]) == 6 or string.len(kv["ISO3166-2"]) == 5 then
             kv["iso_code"] = string.sub(kv["ISO3166-2"], 4)
           end
         elseif string.find(kv["ISO3166-2"], '-', 1, true) == nil then
           if string.len(kv["ISO3166-2"]) == 2 or  string.len(kv["ISO3166-2"]) == 3 then 
             kv["iso_code"] = kv["ISO3166-2"]
           elseif string.len(kv["ISO3166-2"]) == 4 or  string.len(kv["ISO3166-2"]) == 5 then
             kv["iso_code"] = string.sub(kv["ISO3166-2"], 3)
           end
         end
       end
       if kv["name"] == "England" or kv["name"] == "Alba / Scotland" or kv["name"] == "Cymru / Wales" or kv["name"] == "Northern Ireland" then
         kv["admin_level"] = 2
       end
     end

     kv["drive_on_right"] = drive_on_right[kv["name"]] or drive_on_right[kv["name:en"]] or "true"
     kv["allow_intersection_names"] = allow_intersection_names[kv["name"]] or allow_intersection_names[kv["name:en"]] or "false"

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


