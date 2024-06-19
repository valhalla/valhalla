# Data sources in Valhalla
Valhalla routing tiles are created from several different open data sets.

## OpenStreetMap

[OpenStreetMap](https://www.openstreetmap.org/) is a community-driven, editable map of the world. It prioritizes local knowledge and individual contributions over bulk imports, which often means it has excellent coverage even in remote areas where no large-scale mapping efforts have been attempted. OpenStreetMap contains information on landmarks, buildings, roads, and natural features. Valhalla primarily uses the road information from OSM to create its routing network. Valhalla also uses the relations information to provide extra attribution to roads and to create restrictions. The higher-level administrative polygon information is also used to identify country and state/province information for roads.

You can download extracts from [Geofabrik](http://download.geofabrik.de/).

## Elevation data

Valhalla uses terrain data for building elevation-influenced routes, such as for bicycles.

## Boost Timezone Data

Valhalla attaches timezone information to every node/intersection in the road network. The timezone data comes from [tz_world](http://efele.net/maps/tz/world/). This data contains polygon definitions of the various timezones throughout the world. Timezone specifications within each region and conversions of time between timezones are derived using [Boost](http://www.boost.org/users/license.html).
