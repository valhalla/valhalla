     ██▒   █▓ ▄▄▄       ██▓     ██░ ██  ▄▄▄       ██▓     ██▓    ▄▄▄      
    ▓██░   █▒▒████▄    ▓██▒    ▓██░ ██▒▒████▄    ▓██▒    ▓██▒   ▒████▄    
     ▓██  █▒░▒██  ▀█▄  ▒██░    ▒██▀▀██░▒██  ▀█▄  ▒██░    ▒██░   ▒██  ▀█▄  
      ▒██ █░░░██▄▄▄▄██ ▒██░    ░▓█ ░██ ░██▄▄▄▄██ ▒██░    ▒██░   ░██▄▄▄▄██ 
       ▒▀█░   ▓█   ▓██▒░██████▒░▓█▒░██▓ ▓█   ▓██▒░██████▒░██████▒▓█   ▓██▒
       ░ ▐░   ▒▒   ▓▒█░░ ▒░▓  ░ ▒ ░░▒░▒ ▒▒   ▓▒█░░ ▒░▓  ░░ ▒░▓  ░▒▒   ▓▒█░
       ░ ░░    ▒   ▒▒ ░░ ░ ▒  ░ ▒ ░▒░ ░  ▒   ▒▒ ░░ ░ ▒  ░░ ░ ▒  ░ ▒   ▒▒ ░
         ░░    ░   ▒     ░ ░    ░  ░░ ░  ░   ▒     ░ ░     ░ ░    ░   ▒   
          ░        ░  ░    ░  ░ ░  ░  ░      ░  ░    ░  ░    ░  ░     ░  ░
         ░                                                                    

Valhalla is an open source routing engine and accompanying libraries for use with OpenStreetMap data. Valhalla also includes tools like time+distance matrix computation and tour optimization (Traveling Salesman).

Overview
--------

The are several key features that we hope can differentiate the Valhalla project from other routing and network analysis engines. They are:

- Open source software, on open source data with a very liberal license. Should allow for transparency in development, encourage contribution and community input, and foster use in other projects.
- Tiled hierarchical data structure. Should allow users to have a small memory footprint on memory constrained devices, enable offline routing, provide a means for regional extracts and partial updates.
- Dynamic, runtime costing of edges and vertices within the graph via a plugin architecture. Should allow for customization and alternate route generation.
- C++ based API. Should allow for cross compilation of the various pieces to enable routing on offline portable devices.
- A plugin based narrative and maneuver generation architecture. Should allow for generation that is customized either to the administrative area or to the target locale.
- Multi-modal and time-based routes. Should allow for mixing auto, pedestrian, bike and public transportation in the same route or setting a time by which one must arrive at a location.

Organization
--------

The Valhalla organization is comprised of several repositories each responsible for a different function. The layout of the various repositories is as follows:

- [Midgard](https://github.com/valhalla/midgard) - Basic geographic and geometric algorithms for use in the various other projects.
- [Baldr](https://github.com/valhalla/baldr) - The base data structures for accessing and caching tiled route data.
- [Sif](https://github.com/valhalla/sif) - Library used in costing of graph nodes and edges. This can be used as input to `loki` and `thor`.
- [Skadi](https://github.com/valhalla/skadi) - Library and service for accessing elevation data. This can be used as input to `mjolnir` or as a standalone service.
- [Mjolnir](https://github.com/valhalla/mjolnir) - Tools for turning open data into Valhalla graph tiles.
- [Loki](https://github.com/valhalla/loki) - Library used to search graph tiles and correlate input locations to an entity within a tile. This correlated entity (edge or vertex) can be used as input to `thor`.
- [Thor](https://github.com/valhalla/thor) - Library used to generate a path through the graph tile hierarchy.  This path and attribution along the path can be used as input to `odin`.
- [Odin](https://github.com/valhalla/odin) - Library used to generate maneuvers and narrative based on a path. This set of directions information can be used as input to `tyr`.
- [Tyr](https://github.com/valhalla/tyr) - Service used to handle http requests for a route communicating with all of the other valhalla APIs. The service will format output from `odin` and support json (and eventually protocol buffer) output.
- [conf](https://github.com/valhalla/conf) - Runtime configuration files.
- [Tools](https://github.com/valhalla/tools) - A set command line tools that exercise bits of functionality from the libraries above and provide the basis for quality testing and performance benchmarking.
- [Demos](https://github.com/valhalla/demos) - A set of demos which allows interacting with the service and APIs.
- [Chef](https://github.com/valhalla/chef-valhalla) - A chef cookbook demonstrating how to deploy the valhalla stack to a virtual machine (sample vagrant file included).

Documentation
--------

Documentation for the Valhalla service provided through tyr as well as links to a variety of technical descriptions are provided within the [valhalla-docs](https://github.com/valhalla/valhalla-docs) repository.
