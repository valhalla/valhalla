# OSM Tag Usage

## Table of Contents

1. [OSM Data Model Overview](#osm-data-model-overview)
2. [OSM Processing Overview](#osm-processing-overview)
3. [Lua Tag Processing](#lua-tag-processing)
4. [C++ Tag Processing](#c++-tag-processing)
5. [Common Pitfalls and Debugging](#common-pitfalls-and-debugging)

## OSM Data Model Overview

OSM's data is mostly schemaless. It currently has a top level structure which consists of 3 element types. Those are nodes, ways and relations. Nodes are point features, ways are linear features (comprised of nodes) and relations are multi features (comprised of nodes and/or ways). See [here](https://wiki.openstreetmap.org/wiki/Relation) for more on the types of elements within the OSM dataset.

Each of these objects can have any number of key value pairs, called [tags](https://wiki.openstreetmap.org/wiki/Tags) associated with it. The tags are used to delineate physical, political, temporal, etc attribution of the element to which they are attached.

Valhalla creates a routable graph directly from OSM elements and their tags. The connectivity of the graph is a direct result of how the nodes, ways and relations are correlated in the data model. The attribution on edges and nodes in the resulting graph come directly from the tags on those OSM elements. Note that many OSM elements, because of their tags (or lack thereof), will not be useful in creating a routable graph and are simply ignored.

## OSM Processing Overview

The basic flow valhalla follows when creating routing tiles is as follows:

* Parse all the ways and their tags
* Parse all the relations and their tags
* Parse all the nodes and their tags

Each of the step uses a combination of `lua` and `c++` to transform the tags into a structured set of values. From `c++` we call into `lua` passing it a single element. What comes out is a `map` of `keys` to `values` where both have been massaged to fit into a small (compared to the original data) set of permutations. Then the `c++` side of things will turn an element's key value pair strings into well defined structures for storage. The result of this is a vector of fixed-size (static number of bytes) structures (think of a `c++` `struct`) for each element type. Because we turn the tags into fixed-size structures we can store these in a file and use memory mapping to complete this process on very large datasets (the planet) with very modest hardward. Note that we don't store relations in a file simply because they require very little space. 

Once we've gotten all of the basic structures parsed out of the OSM data model into a well-defined set of structures we simply iterate over those structures to create the graph. We take care to sort the structures so that we can iterate over ways and then, within a given way, over the nodes that comprise it (in the right order).

## Lua Tag Processing

The concept of using `lua` to transform OSM tags into a discrete set of keys and values is inspired by `osm2pgsql`. This allows those who want to change the way tags are interpreted to do so without having to recompile valhalla. Additionally, `lua` is quite simplistic so one could argue that its barrier to entry is pretty low. Valhalla comes with default `lua` transformation functions which can be found [here](https://github.com/valhalla/valhalla/tree/master/lua). If you'd like to override these you may do so by changing arguments in the `valhalla.json` configuration used with `valhalla_build_tiles`.

The process of boiling down all the different permutations of OSM values into a discrete set is quite formidable (one could argue that it's never done). An interesting resource for inspecting what type of tags on what types of elements exist in wild can be found at [TagInfo](https://taginfo.openstreetmap.org/). This is great when you want to figure out what tags your parser should target to get as much of the desired attribution as possible. TagInfo also publishes a list of different projects that use OSM data and what tag permutations lead to what attribution in those respective projects. Valhalla publishes a file called [taginfo.json](https://github.com/valhalla/valhalla/blob/master/taginfo.json) which allows the TagInfo website to list the tags that valhalla parses. For more see [here](https://taginfo.openstreetmap.org/projects/valhalla#tags).

## C++ Tag Processing

The basic principle of this part of the process is that we take the string outputs from the `lua` calls and marshal them into what are essentially integral types. There may be some other interesting things that happen at this point in time such as marking which OSM nodes which will become graph nodes (because more than one way references them), and marking loops in ways (because the same node happens more than once) or storing turn restriction information.

## Common Pitfalls and Debugging

When you find a problem with the quality of a route or with route tile creation, one of the first places to look will be at the data. There can be a few places you want to check.

* Missing or wrong tags on OSM elements
* Missing lua code to parse particular tags
* Incorrect error handling of tag values

### Failure to Cut Tiles

Take a look at the log that comes out of `valhalla_build_tiles`. There are many phases of this process. If the process failed at the beginning (in the parsing phase) there is a good chance we have a bug within the `c++` marshalling of strings to integers. You'll want to get a copy of the `pbf` OSM data that was being used when the program crashed (it usually crashes in this case). If you can get a backtrace or even a core file then you can localize this without running it yourself but otherwise simply run `valhalla_build_tiles` and find where it's crashing. Chances are there is an unprotected `std::stoi` or something similar.

### Route Quality Issue

If you are having a route quality issue where the route is using a particular road it shouldn't or not using a particular road that it should you'll want to see what valhalla interpreted the particular street to have. We have a tool for this. The first step here is to get the tile set with the quality issue and run a server with it. You can then crack open [this debug tool](http://valhalla.github.io/demos/locate/) which is pointed at `localhost` (or you can point it at another server). Simply click the problem road. You'll get a green result (or multiple). Click which ever green result is nearest to the problem you are experiencing. The side panel will show you detailed attribution that valhalla stores for this graph edge (or node if you clicked one of those). Carefully inspect the information. Do any of the access flags tell you anything you wouldn't expect? Once you notice what attribution looks incorrect you can then sift through the details to find the way id. Take the way id and plug it into osm.org, like [http://osm.org/way/way_id_here](http://osm.org/way/). From there you can see the tags table which should show you the attributes valhalla sees when its parsing. At this point there is either wrong data in OSM or wrong interpretation in valhalla.