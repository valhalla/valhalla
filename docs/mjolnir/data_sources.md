# Data sources in Valhalla
Valhalla routing tiles are created from several different open data sets. We owe a tremendous debt of gratitude to the individuals and communities which produced them. This document identifies the data sources and licensing of these data sources.

**Attribution is required** for many of our data providers. See the [Attribution](./attribution.md) document for more information.

## OpenStreetMap

[OpenStreetMap](https://www.openstreetmap.org/) is a community-driven, editable map of the world. It prioritizes local knowledge and individual contributions over bulk imports, which often means it has excellent coverage even in remote areas where no large-scale mapping efforts have been attempted. OpenStreetMap contains information on landmarks, buildings, roads, and natural features. Valhalla primarily uses the road information from OSM to create its routing network. Valhalla also uses the relations information to provide extra attribution to roads and to create restrictions. The higher-level administrative polygon information is also used to identify country and state/province information for roads.

All OpenStreetMap data is licensed under the [ODbL](http://opendatacommons.org/licenses/odbl/), a [share-alike](https://en.wikipedia.org/wiki/Share-alike) license, which also requires [attribution](https://wiki.osmfoundation.org/wiki/Licence/Attribution_Guidelines).

Please consider donating to the OSM Foundation to help cover the administration and server costs.

## Elevation data

Valhalla uses terrain data for building elevation-influenced routes, such as for bicycles. You can find more about the license and attribution requirements for elevation data [here](https://github.com/tilezen/joerd/blob/master/docs/attribution.md).

## Boost Timezone Data

Valhalla attaches timezone information to every node/intersection in the road network. The timezone data comes from [tz_world](http://efele.net/maps/tz/world/). This data contains polygon definitions of the various timezones throughout the world. Timezone specifications within each region and conversions of time between timezones are derived using [Boost](http://www.boost.org/users/license.html).

Please notify us if you believe that an open data project has not been properly noted.
