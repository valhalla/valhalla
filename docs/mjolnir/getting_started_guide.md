# Mjolnir: Getting started guide

The mjolnir library is essentially a set of applications, data structures and alogrithms which deal with things like: parsing OpenStreetMap data extracts, cutting routable "graph" tiles, generating tile hierarchies and testing for data deficiencies.

If you would like to create your own routing tiles, this guilde will help you get started.  

### Data

You can either download city-sized extracts from [Mapzen](https://mapzen.com/data/metro-extracts/).  However, if you are looking for larger extracts, they can be downloaded from [Geofabrik GmbH](http://http://download.geofabrik.de/).  If you wish to convert the entire planet, we have successfully run conversions on quad-cores(CPU @ 2.70GHz) machines with 16 gigs of RAM utilizing a SSD.  Conversion with administrative areas and timezones, but without elevation data will take around 15 hours.  This route graph will include motor vehicle, pedestrian, and bicycle route information.

```
./autogen.sh
./configure CPPFLAGS="-DBOOST_SPIRIT_THREADSAFE -DBOOST_NO_CXX11_SCOPED_ENUMS" --enable-python-bindings
make test -j$(nproc)
```

### Creating Data

Run `valhalla_build_tiles` under the valhalla directory.  If needed, update the values under valhalla in your `valhalla.json` config.

./valhalla_build_tiles --config  /path_to_your_config/valhalla.json /data/osm_data/your_osm_extract.pbf

## Optional Prerequisites

### Administrative Areas

An administrative database is created via `valhalla_build_admins` and is used to flag country crossings on edges during the building of the graph data.  Moreover, we also use admins to determine if we drive on the right or left (default: right).  In the future, we will use admins to set the default access restrictions per country.  

We recommend running the `valhalla_build_admins` on the planet; otherwise, parent admin information maybe lost or not all admins will get saved to the database.  This usually happens when a way is missing from the extract, but is part of the admin relation.  Most likely the extract polygon does not cover the entire admin relation.

If you would like administrative information within the route graph, please follow the following steps:

1. Download your osm data.
2. If needed, update the admin value under mjolnir in your valhalla.json config.  Default filename and directory is /data/valhalla/admin.sqlite.
3. Run the valhalla_build_admins under the valhalla directory.  ./valhalla_build_admins --config  /path_to_your_config/valhalla.json /data/osm_data/your_osm_extract.pbf
4. The next time you run valhalla_build_admins, admin information will be added to the route graph.  

### Timezones

Timezones are used if you want to set your departure or arrival date and time. 

If you would like timezone information within the route graph, please follow the following steps:

1. If needed, update the timezone value under mjolnir in your valhalla.json config.  Default filename and directory is /data/valhalla/tz_world.sqlite.
2. Go to your_valhalla_directory/scripts.
3. Run valhalla_build_timezones /path_to_your_config/valhalla.json
4. The next time you run valhalla_build_tiles, timezone information will be added to the route graph.  
