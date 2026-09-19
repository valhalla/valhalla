# street name hint

goal: have a "street_name" hint in the waypoint description, such that matching roads/edges are the ones we correlate to the waypoint.

thoughts:
- it's a soft filter: if there's no half-matching road/edge name anywhere nearby, then use the usual logic
- it's ok to have a 80% solution, i.e. only latin scripts with diacritics (ascii + diacritics?)
- it's obviously very close to geocoding, but we don't want the full geocoding solution, rather the one that matches our level of accuracy
- don't yet go look at pelias or nominatim, pls only go from your own memory/training
- we need to resolve things like "Primus-Truber-Str" or "Primus Truber Str" or "Primus Truber Straße" or "Primus-Truber Straße", or "Hermannstraße" vs "Hermann Straße"; the input to the "street_name" hint is human
- there might be languages where "Straße" is before "Hermann"?!
- we obviously need some sort of scoring, since there might be quite a few roads/edges which partially match the input

now for you, the AI:
- I _only_ want to discuss the conceptual process for now (i.e. Levenstein or other algos)
- do _not_ go through the valhalla code for now!
