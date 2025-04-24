# Mjolnir: Getting started guide

The mjolnir library is essentially a set of applications, data structures and algorithms which deal with things like: parsing OpenStreetMap data extracts, cutting routable "graph" tiles, generating tile hierarchies and testing for data deficiencies.

If you would like to create your own routing tiles, this guide will help you get started.  

### Data

You can download extracts from [Geofabrik GmbH](http://download.geofabrik.de/).  If you wish to convert the entire planet, we have successfully run conversions on quad-cores(CPU @ 2.70GHz) machines with 16 gigs of RAM utilizing a SSD.  Conversions with administrative areas and timezones, but without elevation data will take around 15 hours.  This route graph will include motor vehicle, pedestrian, and bicycle route information.

For more information on what tags and values we use from OSM please consult our [tag parsing guide](tag_parsing.md).

### Creating Data

Build valhalla and its various tools:

```bash
mkdir build
cd build
cmake ..
make all check -j$(nproc)
```

You'll see `valhalla_build_tiles` under the build directory. One level up you'll find `scripts/valhalla_build_config` which you can use to generate the config file needed to build routing tiles. Check the `--help` text for all the various configuration options in the config file.

Generate your config:

    ../scripts/valhalla_build_config > valhalla.json

Then build some tiles from an extract:

    ./valhalla_build_tiles --config  /path_to_your_config/valhalla.json /data/osm_data/your_osm_extract.pbf

## Optional Prerequisites

### Administrative Areas

An administrative database is created via `valhalla_build_admins` and is used to flag country crossings on edges during the building of the graph data.  Moreover, we also use admins to determine if we drive on the right or left (default: right).  In the future, we will use admins to set the default access restrictions per country.  

We recommend running the `valhalla_build_admins` on the planet; otherwise, parent admin information maybe lost or not all admins will get saved to the database.  This usually happens when a way is missing from the extract, but is part of the admin relation.  Most likely the extract polygon does not cover the entire admin relation.

If you would like administrative information within the route graph, please follow the following steps:

1. Download your osm data.
2. If needed, update the admin value under mjolnir in your valhalla.json config.  Default filename and directory is `/data/valhalla/admin.sqlite`.
3. Run `valhalla_build_admins` under the valhalla directory. `./valhalla_build_admins --config  /path_to_your_config/valhalla.json /data/osm_data/your_osm_extract.pbf`
4. The next time you run `valhalla_build_tiles`, admin information will be added to the route graph.

### Timezones

Timezones are used if you want to set your departure or arrival date and time. 

If you would like timezone information within the route graph, please follow the following steps:

1. If needed, update the timezone value under mjolnir in your valhalla.json config.  Default filename and directory is `/data/valhalla/tz_world.sqlite`.
2. Go to `your_valhalla_directory/scripts`.
3. Run `valhalla_build_timezones > /path_to_your/tz_world.sqlite`
4. The next time you run `valhalla_build_tiles`, timezone information will be added to the route graph.

### Elevation

If you want to add elevation information to your route tiles you can do so using SRTMv3 tiles as the input. 

1. If needed run `valhalla_build_elevation`. I will create an elevation dataset which is about 1.6TB for the whole world
2. Point your `valhalla.json` configuration to this directory so it can be referenced when building the graph tiles. The proper configuration value for this can be sent when running `valhalla_build_config` using the `--additional-data-elevation` argument.
3. The next time you run `valhalla_build_tiles`, elevation information will be added to the route graph.

### Transit Data

If you want to add transit data to your route tiles you can follow these steps:

1. Use `valhalla_build_transit` to create an initial set of transit tiles for your region.
2. Configure `valhalla.json` using `valhalla_build_config` and the `--mjolnir-transit-dir` argument.
3. The next time you run `valhalla_build_tiles`, transit graph will be connected to the route graph.
