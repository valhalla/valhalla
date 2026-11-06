# `street_name_hint` **BETA**

A common problem is that geocoders lack the data to return "navigable" points, i.e. how an address should be approached by car. Often we get a building's center coordinate as waypoint. And sometimes that's closer to another road than the building's address, see the example below:

![A demo route where the closest road isn't the right road](images/street-name-hint.png)

Valhalla supports a `street_name_hint` (see [API docs](https://valhalla.github.io/valhalla/api/route/api-reference/#locations)), with which it'll score the surrounding edges names within `street_name_radius` against and use the highest scored edge (& its opposing) above `street_name_threshold` to initialize the expansion.

## Processing

1. **Collect candidates**: Collect the pool of edge candidates within `street_name_radius` if `street_name_hint` is not empty. Note, the other hard filters apply for this pool too.
2. **Normalize names**: Decompose each name using `utf8proc` plus our extra table for additional decompositions; strip non-alphanumeric characters
3. **Convert to Trigrams**: Turn hint & each road name into a collection of trigrams (single `uint64_t` with 3 x 21 bit), e.g. `Weserstr` -> `{"wes", "ese", "ser", "ers", "rst", "str"}`
4. **Calculate weights for hint trigrams**: For each hint's trigram, determine its score, which is proportional to its "inverse document frequency" in the whole pool. Rare trigrams have a higher weight than common ones. The idea is that trigrams of generics like "straße", "avenue" etc are then scored much lower and hardly contribute to the overall score, which makes it automatically adapt to each location.
5. **Score candidate names**: Each trigram in the pool gets scored against the hint, by adding up the weights of the shared hint-candidate trigrams and dividing by the total weight of all hint trigrams. Hint trigrams which aren't found in the pool lower the score significantly.
5. **Keep the best scored candidate**: We only keep the candidate with the highest score and only . If there's a tie of multiple edge candidates with the same score, distance breaks it. If the best score is at least `street_name_threshold`, we use _only_ that edge and its opposing if no other hard or soft location filter applies and they're reachable. All other loki edge candidates move to the filtered edges.

## Recommendations

- use a result of a geocoder as input, ideally on the same dataset as the graph
- don't set other location filters

## Known limitations

In the current state, the implementation all but expects the `street_name_hint` from a geocoder with the same data source for road names.

Current limitations are:

- very little tolerance towards typos: typos are most likely "rare trigrams" which get scored highly on the wrong road names
- we ignore unreachable candidates, i.e. island locations with less minimum reachability than specified for the location will ignore the hint
- refs like A1 are ignored
- not sure on script/locale support, only Latin, Greek & Cyrillic is known to work; others probably work, however, not Chinese & Japanese and cross-script (no transliteration applied, e.g. Cyrillic in hint, Latin in data)
