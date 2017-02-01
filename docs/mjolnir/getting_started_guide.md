# Mjolnir: Getting started guide

The mjolnir library is essentially a set of applications, data structures and alogrithms which deal with things like: parsing OpenStreetMap data extracts, cutting routable "graph" tiles, generating tile hierarchies and testing for data deficiencies.

If you would like to create your own routing tiles, this guilde will help you get started.  

## Prerequisites

### Repositories

At a minimum, Mjolnir depends on the following repositories:

- [Midgard](https://github.com/valhalla/midgard) - Basic geographic and geometric algorithms for use in the various other projects.
- [Baldr](https://github.com/valhalla/baldr) - The base data structures for accessing and caching tiled route data.
- [Sif](https://github.com/valhalla/sif) - Library used in costing of graph nodes and edges. This can be used as input to `loki` and `thor`.
- [Skadi](https://github.com/valhalla/skadi) - Library and service for accessing elevation data. This can be used as input to `mjolnir` or as a standalone service.
- [conf](https://github.com/valhalla/conf) - Runtime configuration files.

However, if you would like to test out the graph with our API, you will also need to obtain the following additional repositories.

- [Loki](https://github.com/valhalla/loki) - Library used to search graph tiles and correlate input locations to an entity within a tile. This correlated entity (edge or vertex) can be used as input to `thor`.
- [Thor](https://github.com/valhalla/thor) - Library used to generate a path through the graph tile hierarchy.  This path and attribution along the path can be used as input to `odin`.
- [Odin](https://github.com/valhalla/odin) - Library used to generate maneuvers and narrative based on a path. This set of directions information can be used as input to `tyr`.
- [Tyr](https://github.com/valhalla/tyr) - Service used to handle http requests for a route communicating with all of the other valhalla APIs. The service will format output from `odin` and support json (and eventually protocol buffer) output.
- [Tools](https://github.com/valhalla/tools) - A set command line tools that exercise bits of functionality from the libraries above and provide the basis for quality testing and performance benchmarking.

### Data

You can either download city-sized extracts from [Mapzen](https://mapzen.com/data/metro-extracts/).  However, if you are looking for larger extracts, they can be downloaded from [Geofabrik GmbH](http://http://download.geofabrik.de/).  If you wish to convert the entire planet, we are running conversions on quad-cores(CPU @ 2.70GHz) machines with 16 gigs of RAM utilizing a SSD.  Conversion with administrative areas, timezones, and transit data, but without elevation data will take around 12 hours.  This route graph will include motor vehicle, pedestrian, transit, and bicycle route information.

### Build the Required Repositories.
Build the repositories in this order: Midgard, Baldr, Sif, Skadi, Mjolnir

```
./autogen.sh
./configure CPPFLAGS="-DBOOST_SPIRIT_THREADSAFE -DBOOST_NO_CXX11_SCOPED_ENUMS"
make test -j($nproc)
sudo make install
```

### Creating Data

Run `valhalla_build_tiles` under the mjolnir directory.  If needed, update the values under mjolnir in your `valhalla.json` config.

./valhalla_build_tiles --config  /path_to_your_config/valhalla.json /data/osm_data/your_osm_extract.pbf

## Optional Prerequisites

### Administrative Areas

An administrative database is created via `valhalla_build_admins` and is used to flag country crossings on edges during the building of the graph data.  Moreover, we also use admins to determine if we drive on the right or left (default: right).  In the future, we will use admins to set the default access restrictions per country.  

We recommend running the `valhalla_build_admins` on the planet; otherwise, parent admin information maybe lost or not all admins will get saved to the database.  This usually happens when a way is missing from the extract, but is part of the admin relation.  Most likely the extract polygon does not cover the entire admin relation.

If you would like administrative information within the route graph, please follow the following steps:

1. Download your osm data.
2. If needed, update the admin value under mjolnir in your valhalla.json config.  Default filename and directory is /data/valhalla/admin.sqlite.
3. Run the valhalla_build_admins under the mjolnir directory.  ./valhalla_build_admins --config  /path_to_your_config/valhalla.json /data/osm_data/your_osm_extract.pbf
4. The next time you run valhalla_build_admins, admin information will be added to the route graph.  

### Timezones

Timezones are used if you want to set your departure or arrival date and time. 

If you would like timezone information within the route graph, please follow the following steps:

1. If needed, update the timezone value under mjolnir in your valhalla.json config.  Default filename and directory is /data/valhalla/tz_world.sqlite.
2. Go to your_mjolnir_directory/scripts.
3. Run valhalla_build_timezones /path_to_your_config/valhalla.json
4. The next time you run valhalla_build_tiles, timezone information will be added to the route graph.  
