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
["Christmas Island"] = "false",
["Cocos (Keeling) Islands	"] = "false",
["Cook Islands"] = "false",
["Cyprus"] = "false",
["Dominica"] = "false",
["East Timor"] = "false",
["Falkland Islands"] = "false",
["Grenada"] = "false",
["Guernsey"] = "false",
["Guyana"] = "false",
["Hong Kong"] = "false",
["India"] = "false",
["Indonesia"] = "false",
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
["Norfolk Island"] = "false",
["Pakistan"] = "false",
["Papua Niugini"] = "false",
["Pitcairn Islands"] = "false",
["Republic of Ireland"] = "false",
["Saint Helena"] = "false",
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
["United Kingdom"] = "false",
["United States Virgin Islands"] = "false",
["Viti"] = "false",
["Zambia"] = "false",
["Zimbabwe"] = "false"
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

function rels_proc (kv, nokeys)
  if (kv["type"] == "boundary" and kv["boundary"] == "administrative" and
     (kv["admin_level"] == "2" or kv["admin_level"] == "3" or kv["admin_level"] == "4" or kv["admin_level"] == "6")) then

     --save only states/prov for USA, MX, and CA.
     -- TODO Save state/prov for everywhere
     if (kv["name"] == "United States of America" or kv["name"] == "Canada") then
       return 1, kv
     end

     if (kv["admin_level"] == "3" and kv["name"] ~= "Guyane" and kv["name"] ~= "Guadeloupe" and  kv["name"] ~= "La Réunion" and  
         kv["name"] ~= "Martinique" and kv["name"] ~= "Mayotte" and kv["name"] ~= "Saint-Pierre-et-Miquelon" and
         kv["name"] ~= "Saint-Barthélemy" and  kv["name"] ~= "Saint-Martin" and kv["name"] ~= "Polynésie Française" and 
         kv["name"] ~= "Wallis-et-Futuna" and kv["name"] ~= "Nouvelle-Calédonie" and kv["name"] ~= "Île de Clipperton" and 
         kv["name"] ~= "Terres australes et antarctiques françaises" and kv["name"] ~= "Saint Helena") then
        return 1, kv
     end

     if (kv["admin_level"] == "4" and 
        (kv["name"] ~= "Alabama" and kv["name"] ~= "Alaska" and kv["name"] ~= "Alberta" and
         kv["name"] ~= "Arizona" and kv["name"] ~= "Arkansas" and kv["name"] ~= "British Columbia" and
         kv["name"] ~= "California" and kv["name"] ~= "Colorado" and kv["name"] ~= "Connecticut" and
         kv["name"] ~= "Delaware" and kv["name"] ~= "District of Columbia" and kv["name"] ~= "Florida" and 
         kv["name"] ~= "Georgia" and kv["name"] ~= "Idaho" and kv["name"] ~= "Illinois" and 
         kv["name"] ~= "Indiana" and kv["name"] ~= "Iowa" and kv["name"] ~= "Kansas" and 
         kv["name"] ~= "Kentucky" and kv["name"] ~= "Louisiana" and kv["name"] ~= "Maine" and 
         kv["name"] ~= "Manitoba" and kv["name"] ~= "Maryland" and kv["name"] ~= "Massachusetts" and 
         kv["name"] ~= "Michigan" and kv["name"] ~= "Minnesota" and kv["name"] ~= "Mississippi" and 
         kv["name"] ~= "Missouri" and kv["name"] ~= "Montana" and kv["name"] ~= "Nebraska" and
         kv["name"] ~= "Nevada" and kv["name"] ~= "New Brunswick" and kv["name"] ~= "Newfoundland and Labrador" and
         kv["name"] ~= "New Hampshire" and kv["name"] ~= "New Jersey" and kv["name"] ~= "New Mexico" and
         kv["name"] ~= "New York" and kv["name"] ~= "North Carolina" and kv["name"] ~= "North Dakota" and
         kv["name"] ~= "Northwest Territories" and kv["name"] ~= "Nova Scotia" and kv["name"] ~= "Nunavut" and
         kv["name"] ~= "Ohio" and kv["name"] ~= "Oklahoma" and kv["name"] ~= "Ontario" and
         kv["name"] ~= "Oregon" and kv["name"] ~= "Pennsylvania" and kv["name"] ~= "Prince Edward Island" and 
         kv["name"] ~= "Québec" and kv["name"] ~= "Rhode Island" and kv["name"] ~= "Saskatchewan" and 
         kv["name"] ~= "South Carolina" and kv["name"] ~= "South Dakota" and kv["name"] ~= "Tennessee" and
         kv["name"] ~= "Texas" and kv["name"] ~= "Utah" and kv["name"] ~= "Vermont" and 
         kv["name"] ~= "Virginia" and kv["name"] ~= "Washington" and kv["name"] ~= "West Virginia" and
         kv["name"] ~= "Wisconsin" and kv["name"] ~= "Wyoming" and kv["name"] ~= "Yukon" and
         kv["name"] ~= "Hawaii" and kv["name"] ~= "American Samoa" and 
         kv["name"] ~= "United States Minor Outlying Islands" and kv["name"] ~= "Northern Mariana Islands" and 
         kv["name"] ~= "Puerto Rico" and kv["name"] ~= "United States Virgin Islands") and 
         kv["name:en"] ~= "Hong Kong" and kv["name:en"] ~= "Macao" and kv["name"] ~="Christmas Island" and 
         kv["name"] ~= "Cocos (Keeling) Islands" and kv["name"] ~= "Norfolk Island") then
       return 1, kv
     end

     if kv["admin_level"] == "6" and kv["name"] ~= "District of Columbia" then
       return 1, kv
     end

     kv["drive_on_right"] = "true"

     local drive_on_right = drive_on_right[kv["name"]]

     if drive_on_right then
       kv["drive_on_right"] = tostring(drive_on_right)
     else 
       drive_on_right = drive_on_right[kv["name:en"]]
       if drive_on_right then
         kv["drive_on_right"] = tostring(drive_on_right)
       end
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


