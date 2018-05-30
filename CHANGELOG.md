## Release Date: UNRELEASED Valhalla 2.7.0

## Release Date: 2018-05-28 Valhalla 2.6.0
* **Infrastructure**:
   * CHANGED: Update cmake buildsystem to replace autoconf [#1272](https://github.com/valhalla/valhalla/pull/1272)
* **API**:
   * CHANGED: Move `trace_options` parsing to map matcher factory [#1260](https://github.com/valhalla/valhalla/pull/1260)
   * ADDED: New costing method for AutoDataFix [#1283](https://github.com/valhalla/valhalla/pull/1283)

## Release Date: 2018-05-21 Valhalla 2.5.0
* **Infrastructure**
   * ADDED: Add code formatting and linting.
* **API**
   * ADDED: Added new motorcycle costing, motorcycle access flag in data and use_trails option.
* **Routing**
   * ADDED: Add time dependnet forward and reverse A* methods.
   * FIXED: Increase minimum threshold for driving routes in bidirectional A* (fixes some instances of bad paths).
* **Data Producer Update**
   * CHANGED: Updates to properly handle cycleway crossings.
   * CHANGED: Conditionally include driveways that are private.
   * ADDED: Added logic to set motorcycle access.  This includes lua, country access, and user access flags for motorcycles.

## Release Date: 2018-04-11 Valhalla 2.4.9
* **Enhancement**
   * Added European Portuguese localization for Valhalla
   * Updates to EdgeStatus to improve performance. Use an unordered_map of tile Id and allocate an array for each edge in the tile. This allows using pointers to access status for sequential edges. This improves performance by 50% or so.
   * A couple of bicycle costing updates to improve route quality: avoid roads marked as part of a truck network, to remove the density penalty for transition costs.
   * When optimal matrix type is selected, now use CostMatrix for source to target pedestrian and bicycle matrix calls when both counts are above some threshold. This improves performance in general and lessens some long running requests.
*  **Data Producer Update**
   * Added logic to protect against setting a speed of 0 for ferries.

## Release Date: 2018-03-27 Valhalla 2.4.8
* **Enhancement**
   * Updates for Italian verbal translations
   * Optionally remove driveways at graph creation time
   * Optionally disable candidate edge penalty in path finding
   * OSRM compatible route, matrix and map matching response generation
   * Minimal Windows build compatibility
   * Refactoring to use PBF as the IPC mechanism for all objects
   * Improvements to internal intersection marking to reduce false positives
* **Bug Fix**
   * Cap candidate edge penalty in path finding to reduce excessive expansion
   * Fix trivial paths at deadends

## Release Date: 2018-02-08 Valhalla 2.4.7
* **Enhancement**
   * Speed up building tiles from small OSM imports by using boost directory iterator rather than going through all possible tiles and testing each if the file exists.
* **Bug Fix**
   * Protect against overflow in string to float conversion inside OSM parsing.

## Release Date: 2018-01-26 Valhalla 2.4.6
* **Enhancement**
   * Elevation library will lazy load RAW formatted sources

## Release Date: 2018-01-24 Valhalla 2.4.5
* **Enhancement**
   * Elevation packing utility can unpack lz4hc now
* **Bug Fix**
   * Fixed broken darwin builds

## Release Date: 2018-01-23 Valhalla 2.4.4
* **Enhancement**
   * Elevation service speed improvments and the ability to serve lz4hc compressed data
   * Basic support for downloading routing tiles on demand
   * Deprecated `valhalla_route_service`, now all services (including elevation) are found under `valhalla_service`

## Release Date: 2017-12-11 Valhalla 2.4.3
* **Enhancement**
   * Remove union from GraphId speeds up some platforms
   * Use SAC scale in pedestrian costing
   * Expanded python bindings to include all actions (route, matrix, isochrone, etc)
* **Bug Fix**
   * French translation typo fixes
*  **Data Producer Update**
   * Handling shapes that intersect the poles when binning
   * Handling when transit shapes are less than 2 points

## Release Date: 2017-11-09 Valhalla 2.4.1
*  **Data Producer Update**
   * Added kMopedAccess to modes for complex restrictions.  Remove the kMopedAccess when auto access is removed.  Also, add the kMopedAccess when an auto restriction is found.

## Release Date: 2017-11-08 Valhalla 2.4.0
*  **Data Producer Update**
   * Added logic to support restriction = x with a the except tag.  We apply the restriction to everything except for modes in the except tag.
   * Added logic to support railway_service and coach_service in transit.
* **Bug Fix**
  * Return proper edge_walk path for requested shape_match=walk_or_snap
  * Skip invalid stateid for Top-K requests

## Release Date: 2017-11-07 Valhalla 2.3.9
* **Enhancement**
  * Top-K map matched path generation now only returns unique paths and does so with fewer iterations
  * Navigator call outs for both imperial and metric units
  * The surface types allowed for a given bike route can now be controlled via a request parameter `avoid_bad_surfaces`
  * Improved support for motorscooter costing via surface types, road classification and vehicle specific tagging
* **Bug Fix**
  * Connectivity maps now include information about transit tiles
  * Lane counts for singly digitized roads are now correct for a given directed edge
  * Edge merging code for assigning osmlr segments is now robust to partial tile sets
  * Fix matrix path finding to allow transitioning down to lower levels when appropriate. In particular, do not supersede shortcut edges until no longer expanding on the next level.
  * Fix optimizer rotate location method. This fixes a bug where optimal ordering was bad for large location sets.
*  **Data Producer Update**
   * Duration tags are now used to properly set the speed of travel for a ferry routes

## Release Date: 2017-10-17 Valhalla 2.3.8
* **Bug Fix**
  * Fixed the roundabout exit count for bicycles when the roundabout is a road and not a cycleway
  * Enable a pedestrian path to remain on roundabout instead of getting off and back on
  * Fixed the penalization of candidate locations in the uni-directional A* algorithm (used for trivial paths)
*  **Data Producer Update**
   * Added logic to set bike forward and tag to true where kv["sac_scale"] == "hiking". All other values for sac_scale turn off bicycle access.  If sac_scale or mtb keys are found and a surface tag is not set we default to kPath.
   * Fixed a bug where surface=unpaved was being assigned Surface::kPavedSmooth.

## Release Date: 2017-9-11 Valhalla 2.3.7
* **Bug Fix**
  * Update bidirectional connections to handle cases where the connecting edge is one of the origin (or destination) edges and the cost is high. Fixes some pedestrian route issues that were reported.
*  **Data Producer Update**
   * Added support for motorroad tag (default and per country).
   * Update OSMLR segment association logic to fix issue where chunks wrote over leftover segments. Fix search along edges to include a radius so any nearby edges are also considered.

## Release Date: 2017-08-29 Valhalla 2.3.6
* **Bug Fix**
  * Pedestrian paths including ferries no longer cause circuitous routes
  * Fix a crash in map matching route finding where heading from shape was using a `nullptr` tile
  * Spanish language narrative corrections
  * Fix traffic segment matcher to always set the start time of a segment when its known
* **Enhancement**
  * Location correlation scoring improvements to avoid situations where less likely start or ending locations are selected

## Release Date: 2017-08-22 Valhalla 2.3.5
* **Bug Fix**
  * Clamp the edge score in thor. Extreme values were causing bad alloc crashes.
  * Fix multimodal isochrones. EdgeLabel refactor caused issues.
* **Data Producer Update**
  * Update lua logic to properly handle vehicle=no tags.

## Release Date: 2017-08-14 Valhalla 2.3.4
* **Bug Fix**
  * Enforce limits on maximum per point accuracy to avoid long running map matching computations

## Release Date: 2017-08-14 Valhalla 2.3.3
* **Bug Fix**
  * Maximum osm node reached now causes bitset to resize to accomodate when building tiles
  * Fix wrong side of street information and remove redundant node snapping
  * Fix path differences between services and `valhalla_run_route`
  * Fix map matching crash when interpolating duplicate input points
  * Fix unhandled exception when trace_route or trace_attributes when there are no continuous matches
* **Enhancement**
  * Folded Low-Stress Biking Code into the regular Bicycle code and removed the LowStressBicycleCost class. Now when making a query for bicycle routing, a value of 0 for use_hills and use_roads produces low-stress biking routes, while a value of 1 for both provides more intense professional bike routes.
  * Bike costing default values changed. use_roads and use_hills are now 0.25 by default instead of 0.5 and the default bike is now a hybrid bike instead of a road bike.
  * Added logic to use station hierarchy from transitland.  Osm and egress nodes are connected by transitconnections.  Egress and stations are connected by egressconnections.  Stations and platforms are connected by platformconnections.  This includes narrative updates for Odin as well.

## Release Date: 2017-07-31 Valhalla 2.3.2
* **Bug Fix**
  * Update to use oneway:psv if oneway:bus does not exist.
  * Fix out of bounds memory issue in DoubleBucketQueue.
  * Many things are now taken into consideration to determine which sides of the road have what cyclelanes, because they were not being parsed correctly before
  * Fixed issue where sometimes a "oneway:bicycle=no" tag on a two-way street would cause the road to become a oneway for bicycles
  * Fixed trace_attributes edge_walk cases where the start or end points in the shape are close to graph nodes (intersections)
  * Fixed 32bit architecture crashing for certain routes with non-deterministic placement of edges labels in bucketized queue datastructure
* **Enhancement**
  * Improve multi-modal routes by adjusting the pedestrian mode factor (routes use less walking in favor of public transit).
  * Added interface framework to support "top-k" paths within map-matching.
  * Created a base EdgeLabel class that contains all data needed within costing methods and supports the basic path algorithms (forward direction, A*, with accumulated path distance). Derive class for bidirectional algorithms (BDEdgeLabel) and for multimodal algorithms. Lowers memory use by combining some fields (using spare bits from GraphId).
  * Added elapsed time estimates to map-matching labels in preparation for using timestamps in map-matching.
  * Added parsing of various OSM tags: "bicycle=use_sidepath", "bicycle=dismount", "segregated=*", "shoulder=*", "cycleway:buffer=*", and several variations of these.
  * Both trace_route and trace_attributes will parse `time` and `accuracy` parameters when the shape is provided as unencoded
  * Map-matching will now use the time (in seconds) of each gps reading (if provided) to narrow the search space and avoid finding matches that are impossibly fast

## Release Date: 2017-07-10 Valhalla 2.3.0
* **Bug Fix**
  * Fixed a bug in traffic segment matcher where length was populated but had invalid times
* **Embedded Compilation**
  * Decoupled the service components from the rest of the worker objects so that the worker objects could be used in non http service contexts
   * Added an actor class which encapsulates the various worker objects and allows the various end points to be called /route /height etc. without needing to run a service
* **Low-Stress Bicycle**
  * Worked on creating a new low-stress biking option that focuses more on taking safer roads like cycle ways or residential roads than the standard bike costing option does.

## Release Date: 2017-06-26 Valhalla 2.2.9
* **Bug Fix**
  * Fix a bug introduced in 2.2.8 where map matching search extent was incorrect in longitude axis.

## Release Date: 2017-06-23 Valhalla 2.2.8
* **Bug Fix**
  * Traffic segment matcher (exposed through Python bindings) - fix cases where partial (or no) results could be returned when breaking out of loop in form_segments early.
* **Traffic Matching Update**
  * Traffic segment matcher - handle special cases when entering and exiting turn channels.
* **Guidance Improvements**
  * Added Swedish (se-SV) narrative file.

## Release Date: 2017-06-20 Valhalla 2.2.7
* **Bug Fixes**
  * Traffic segment matcher (exposed through Python bindings) makes use of accuracy per point in the input
  * Traffic segment matcher is robust to consecutive transition edges in matched path
* **Isochrone Changes**
  * Set up isochrone to be able to handle multi-location queries in the future
* **Data Producer Updates**
  * Fixes to valhalla_associate_segments to address threading issue.
  * Added support for restrictions that refers only to appropriate type of vehicle.
* **Navigator**
  * Added pre-alpha implementation that will perform guidance for mobile devices.
* **Map Matching Updates**
  * Added capability to customize match_options

## Release Date: 2017-06-12 Valhalla 2.2.6
* **Bug Fixes**
  * Fixed the begin shape index where an end_route_discontinuity exists
* **Guidance Improvements**
  * Updated Slovenian (sl-SI) narrative file.
* **Data Producer Updates**
  * Added support for per mode restrictions (e.g., restriction:&lt;type&gt;)  Saved these restrictions as "complex" restrictions which currently support per mode lookup (unlike simple restrictions which are assumed to apply to all driving modes).
* **Matrix Updates**
  * Increased max distance threshold for auto costing and other similar costings to 400 km instead of 200 km

## Release Date: 2017-06-05 Valhalla 2.2.5
* **Bug Fixes**
  * Fixed matched point edge_index by skipping transition edges.
  * Use double precision in meili grid traversal to fix some incorrect grid cases.
  * Update meili to use DoubleBucketQueue and GraphReader methods rather than internal methods.

## Release Date: 2017-05-17 Valhalla 2.2.4
* **Bug Fixes**
  * Fix isochrone bug where the default access mode was used - this rejected edges that should not have been rejected for cases than automobile.
  * Fix A* handling of edge costs for trivial routes. This fixed an issue with disconnected regions that projected to a single edge.
  * Fix TripPathBuilder crash if first edge is a transition edge (was occurring with map-matching in rare occasions).

## Release Date: 2017-05-15 Valhalla 2.2.3
* **Map Matching Improvement**
  * Return begin and end route discontinuities. Also, returns partial shape of edge at route discontinuity.
* **Isochrone Improvements**
  * Add logic to make sure the center location remains fixed at the center of a tile/grid in the isotile.
  * Add a default generalization factor that is based on the grid size. Users can still override this factor but the default behavior is improved.
  * Add ExpandForward and ExpandReverse methods as is done in bidirectional A*. This improves handling of transitions between hierarchy levels.
* **Graph Correlation Improvements**
  * Add options to control both radius and reachability per input location (with defaults) to control correlation of input locations to the graph in such a way as to avoid routing between disconnected regions and favor more likely paths.

## Release Date: 2017-05-08 Valhalla 2.2.0
* **Guidance Improvements**
  * Added Russian (ru-RU) narrative file.
  * Updated Slovenian (sl-SI) narrative file.
* **Data Producer Updates**
  * Assign destination sign info on bidirectional ramps.
  * Update ReclassifyLinks. Use a "link-tree" which is formed from the exit node and terminates at entrance nodes. Exit nodes are sorted by classification so motorway exits are done before trunks, etc. Updated the turn channel logic - now more consistently applies turn channel use.
  * Updated traffic segment associations to properly work with elevation and lane connectivity information (which is stored after the traffic association).

## Release Date: 2017-04-24 Valhalla 2.1.9
* **Elevation Update**
  * Created a new EdgeElevation structure which includes max upward and downward slope (moved from DirectedEdge) and mean elevation.
* **Routing Improvements**
  * Destination only fix when "nested" destination only areas cause a route failure. Allow destination only edges (with penalty) on 2nd pass.
  * Fix heading to properly use the partial edge shape rather than entire edge shape to determine heading at the begin and end locations.
  * Some cleanup and simplification of the bidirectional A* algorithm.
  * Some cleanup and simplification of TripPathBuilder.
  * Make TileHierarchy data and methods static and remove tile_dir from the tile hierarchy.
* **Map Matching Improvement**
  * Return matched points with trace attributes when using map_snap.
* **Data Producer Updates**
  * lua updates so that the chunnel will work again.

## Release Date: 2017-04-04 Valhalla 2.1.8
* **Map Matching Release**
  * Added max trace limits and out-of-bounds checks for customizable trace options

## Release Date: 2017-03-29 Valhalla 2.1.7
* **Map Matching Release**
  * Increased service limits for trace
* **Data Producer Updates**
  * Transit: Remove the dependency on using level 2 tiles for transit builder
* **Traffic Updates**
  * Segment matcher completely re-written to handle many complex issues when matching traces to OTSs
* **Service Improvement**
  * Bug Fix - relaxed rapidjson parsing to allow numeric type coercion
* **Routing Improvements**
  * Level the forward and reverse paths in bidirectional A * to account for distance approximation differences.
  * Add logic for Use==kPath to bicycle costing so that paths are favored (as are footways).

## Release Date: 2017-03-10 Valhalla 2.1.3
* **Guidance Improvement**
  * Corrections to Slovenian narrative language file
  **Routing Improvements**
  * Increased the pedestrian search radius from 25 to 50 within the meili configuration to reduce U-turns with map-matching
  * Added a max avoid location limit

## Release Date: 2017-02-22 Valhalla 2.1.0
* **Guidance Improvement**
  * Added ca-ES (Catalan) and sl-SI (Slovenian) narrative language files
* **Routing  Improvement**
  * Fix through location reverse ordering bug (introduced in 2.0.9) in output of route responses for depart_at routes
  * Fix edge_walking method to handle cases where more than 1 initial edge is found
* **Data Producer Updates**
  * Improved transit by processing frequency based schedules.
  * Updated graph validation to more aggressively check graph consistency on level 0 and level 1
  * Fix the EdgeInfo hash to not create duplicate edge info records when creating hierarchies

## Release Date: 2017-02-21 Valhalla 2.0.9
* **Guidance Improvement**
  * Improved Italian narrative by handling articulated prepositions
  * Properly calling out turn channel maneuver
* **Routing Improvement**
  * Improved path determination by increasing stop impact for link to link transitions at intersections
  * Fixed through location handling, now includes cost at throughs and properly uses heading
  * Added ability to adjust location heading tolerance
* **Traffic Updates**
  * Fixed segment matching json to properly return non-string values where apropriate
* **Data Producer Updates**
  * Process node:ref and way:junction_ref as a semicolon separated list for exit numbers
  * Removed duplicated interchange sign information when ways are split into edges
  * Use a sequence within HierarchyBuilder to lower memory requirements for planet / large data imports.
  * Add connecting OSM wayId to a transit stop within NodeInfo.
  * Lua update:  removed ways that were being added to the routing graph.
  * Transit:  Fixed an issue where add_service_day and remove_service_day was not using the tile creation date, but the service start date for transit.
  * Transit:  Added acceptance test logic.
  * Transit:  Added fallback option if the associated wayid is not found.  Use distance approximator to find the closest edge.
  * Transit:  Added URL encoding for one stop ids that contain diacriticals.  Also, added include_geometry=false for route requests.
* **Optimized Routing Update**
  * Added an original index to the location object in the optimized route response
* **Trace Route Improvement**
  * Updated find_start_node to fix "GraphTile NodeInfo index out of bounds" error

## Release Date: 2017-01-30 Valhalla 2.0.6
* **Guidance Improvement**
  * Italian phrases were updated
* **Routing Improvement**
  * Fixed an issue where date and time was returning an invalid ISO8601 time format for date_time values in positive UTC. + sign was missing.
  * Fixed an encoding issue that was discovered for tranist_fetcher.  We were not encoding onestop_ids or route_ids.  Also, added exclude_geometry=true for route API calls.
* **Data Producer Updates**
  * Added logic to grab a single feed in valhalla_build_transit.

## Release Date: 2017-01-04 Valhalla 2.0.3
* **Service Improvement**
  * Added support for interrupting requests. If the connection is closed, route computation and map-matching can be interrupted prior to completion.
* **Routing Improvement**
  * Ignore name inconsistency when entering a link to avoid double penalizing.
* **Data Producer Updates**
  * Fixed consistent name assignment for ramps and turn lanes which improved guidance.
  * Added a flag to directed edges indicating if the edge has names. This can potentially be used in costing methods.
  * Allow future use of spare GraphId bits within DirectedEdge.

## Release Date: 2016-12-13 Valhalla 2.0.2
* **Routing Improvement**
  * Added support for multi-way restrictions to matrix and isochrones.
  * Added HOV costing model.
  * Speed limit updates.   Added logic to save average speed separately from speed limits.
  * Added transit include and exclude logic to multimodal isochrone.
  * Fix some edge cases for trivial (single edge) paths.
  * Better treatment of destination access only when using bidirectional A*.
* **Performance Improvement**
  * Improved performance of the path algorithms by making many access methods inline.

## Release Date: 2016-11-28 Valhalla 2.0.1
* **Routing Improvement**
  * Preliminary support for multi-way restrictions
* **Issues Fixed**
  * Fixed tile incompatiblity between 64 and 32bit architectures
  * Fixed missing edges within tile edge search indexes
  * Fixed an issue where transit isochrone was cut off if we took transit that was greater than the max_seconds and other transit lines or buses were then not considered.

## Release Date: 2016-11-15 Valhalla 2.0

* **Tile Redesign**
  * Updated the graph tiles to store edges only on the hierarchy level they belong to. Prior to this, the highways were stored on all levels, they now exist only on the highway hierarchy. Similar changes were made for arterial level roads. This leads to about a 20% reduction in tile size.
  * The tile redesign required changes to the path generation algorithms. They must now transition freely beteeen levels, even for pedestrian and bicycle routes. To offset the extra transitions, the main algorithms were changed to expand nodes at each level that has directed edges, rather than adding the transition edges to the priority queue/adjacency list. This change helps performance. The hierarchy limits that are used to speed the computation of driving routes by utilizing the highway hierarchy were adjusted to work with the new path algorithms.
  * Some changes to costing were also required, for example pedestrian and bicycle routes skip shortcut edges.
  * Many tile data structures were altered to explicitly size different fields and make room for "spare" fields that will allow future growth. In addition, the tile itself has extra "spare" records that can be appended to the end of the tile and referenced from the tile header. This also will allow future growth without breaking backward compatibility.
* **Guidance Improvement**
  * Refactored trip path to use an enumerated `Use` for edge and an enumerated `NodeType` for node
  * Fixed some wording in the Hindi narrative file
  * Fixed missing turn maneuver by updating the forward intersecting edge logic
* **Issues Fixed**
  * Fixed an issue with pedestrian routes where a short u-turn was taken to avoid the "crossing" penalty.
  * Fixed bicycle routing due to high penalty to enter an access=destination area. Changed to a smaller, length based factor to try to avoid long regions where access = destination. Added a driveway penalty to avoid taking driveways (which are often marked as access=destination).
  * Fixed regression where service did not adhere to the list of allowed actions in the Loki configuration
* **Graph Correlation**
  * External contributions from Navitia have lead to greatly reduced per-location graph correlation. Average correlation time is now less than 1ms down from 4-9ms.

## Release Date: 2016-10-17

* **Guidance Improvement**
  * Added the Hindi (hi-IN) narrative language
* **Service Additions**
  * Added internal valhalla error codes utility in baldr and modified all services to make use of and return as JSON response
  * See documentation https://github.com/valhalla/valhalla-docs/blob/master/api-reference.md#internal-error-codes-and-conditions
* **Time-Distance Matrix Improvement**
  * Added a costmatrix performance fix for one_to_many matrix requests
* **Memory Mapped Tar Archive - Tile Extract Support**
  * Added the ability to load a tar archive of the routing graph tiles. This improves performance under heavy load and reduces the memory requirement while allowing multiple processes to share cache resources.

## Release Date: 2016-09-19

* **Guidance Improvement**
  * Added pirate narrative language
* **Routing Improvement**
  * Added the ability to include or exclude stops, routes, and operators in multimodal routing.
* **Service Improvement**
  * JSONify Error Response

## Release Date: 2016-08-30

* **Pedestrian Routing Improvement**
  * Fixes for trivial pedestrian routes

## Release Date: 2016-08-22

* **Guidance Improvements**
  * Added Spanish narrative
  * Updated the start and end edge heading calculation to be based on road class and edge use
* **Bicycle Routing Improvements**
  * Prevent getting off a higher class road for a small detour only to get back onto the road immediately.
  * Redo the speed penalties and road class factors - they were doubly penalizing many roads with very high values.
  * Simplify the computation of weighting factor for roads that do not have cycle lanes. Apply speed penalty to slightly reduce favoring
of non-separated bicycle lanes on high speed roads.
* **Routing Improvements**
  * Remove avoidance of U-turn for pedestrian routes. This improves use with map-matching since pedestrian routes can make U-turns.
  * Allow U-turns at dead-ends for driving (and bicycling) routes.
* **Service Additions**
  * Add support for multi-modal isochrones.
  * Added base code to allow reverse isochrones (path from anywhere to a single destination).
* **New Sources to Targets**
  * Added a new Matrix Service action that allows you to request any of the 3 types of time-distance matrices by calling 1 action.  This action takes a sources and targets parameter instead of the locations parameter.  Please see the updated Time-Distance Matrix Service API reference for more details.

## Release Date: 2016-08-08

 * **Service additions**
  * Latitude, longitude bounding boxes of the route and each leg have been added to the route results.
  * Added an initial isochrone capability. This includes methods to create an "isotile" - a 2-D gridded data set with time to reach each lat,lon grid from an origin location. This isoltile is then used to create contours at specified times. Interior contours are optionally removed and the remaining outer contours are generalized and converted to GeoJSON polygons. An initial version supporting multimodal route types has also been added.
 * **Data Producer Updates**
  * Fixed tranist scheduling issue where false schedules were getting added.
 * **Tools Additionas**
  * Added `valhalla_export_edges` tool to allow shape and names to be dumped from the routing tiles

## Release Date: 2016-07-19

 * **Guidance Improvements**
  * Added French narrative
  * Added capability to have narrative language aliases - For example: German `de-DE` has an alias of `de`
 * **Transit Stop Update** - Return latitude and longitude for each transit stop
 * **Data Producer Updates**
  * Added logic to use lanes:forward, lanes:backward, speed:forward, and speed:backward based on direction of the directed edge.
  * Added support for no_entry, no_exit, and no_turn restrictions.
  * Added logic to support country specific access. Based on country tables found here: http://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Access-Restrictions

## Release Date: 2016-06-08

 * **Bug Fix** - Fixed a bug where edge indexing created many small tiles where no edges actually intersected. This allowed impossible routes to be considered for path finding instead of rejecting them earlier.
 * **Guidance Improvements**
  * Fixed invalid u-turn direction
  * Updated to properly call out jughandle routes
  * Enhanced signless interchange maneuvers to help guide users
 * **Data Producer Updates**
  * Updated the speed assignment for ramp to be a percentage of the original road class speed assignment
  * Updated stop impact logic for turn channel onto ramp

## Release Date: 2016-05-19

 * **Bug Fix** - Fixed a bug where routes fail within small, disconnected "islands" due to the threshold logic in prior release. Also better logic for not-thru roads.

## Release Date: 2016-05-18

 * **Bidirectional A* Improvements** - Fixed an issue where if both origin and destination locations where on not-thru roads that meet at a common node the path ended up taking a long detour. Not all cases were fixed though - next release should fix. Trying to address the termination criteria for when the best connection point of the 2 paths is optimal. Turns out that the initial case where both opposing edges are settled is not guaranteed to be the least cost path. For now we are setting a threshold and extending the search while still tracking best connections. Fixed the opposing edge when a hierarchy transition occurs.
 * **Guidance Globalization** -  Fixed decimal distance to be locale based.
 * **Guidance Improvements**
  * Fixed roundabout spoke count issue by fixing the drive_on_right attribute.
  * Simplified narative by combining unnamed straight maneuvers
  * Added logic to confirm maneuver type assignment to avoid invalid guidance
  * Fixed turn maneuvers by improving logic for the following:
    * Internal intersection edges
    * 'T' intersections
    * Intersecting forward edges
 * **Data Producer Updates** - Fix the restrictions on a shortcut edge to be the same as the last directed edge of the shortcut (rather than the first one).

## Release Date: 2016-04-28

 * **Tile Format Updates** - Separated the transit graph from the "road only" graph into different tiles but retained their interconnectivity. Transit tiles are now hierarchy level 3.
 * **Tile Format Updates** - Reduced the size of graph edge shape data by 5% through the use of varint encoding (LEB128)
 * **Tile Format Updates** - Aligned `EdgeInfo` structures to proper byte boundaries so as to maintain compatibility for systems who don't support reading from unaligned addresses.
 * **Guidance Globalization** -  Added the it-IT(Italian) language file. Added support for CLDR plural rules. The cs-CZ(Czech), de-DE(German), and en-US(US English) language files have been updated.
 * **Travel mode based instructions** -  Updated the start, post ferry, and post transit insructions to be based on the travel mode, for example:
  * `Drive east on Main Street.`
  * `Walk northeast on Broadway.`
  * `Bike south on the cycleway.`

## Release Date: 2016-04-12

 * **Guidance Globalization** -  Added logic to use tagged language files that contain the guidance phrases. The initial versions of en-US, de-DE, and cs-CZ have been deployed.
 * **Updated ferry defaults** -  Bumped up use_ferry to 0.65 so that we don't penalize ferries as much.

## Release Date: 2016-03-31
 * **Data producer updates** - Do not generate shortcuts across a node which is a fork. This caused missing fork maneuvers on longer routes.  GetNames update ("Broadway fix").  Fixed an issue with looking up a name in the ref map and not the name map.  Also, removed duplicate names.  Private = false was unsetting destination only flags for parking aisles.

## Release Date: 2016-03-30
 * **TripPathBuilder Bug Fix** - Fixed an exception that was being thrown when trying to read directed edges past the end of the list within a tile. This was due to errors in setting walkability and cyclability on upper hierarchies.

## Release Date: 2016-03-28

 * **Improved Graph Correlation** -  Correlating input to the routing graph is carried out via closest first traversal of the graph's, now indexed, geometry. This results in faster correlation and gaurantees the absolute closest edge is found.

## Release Date: 2016-03-16

 * **Transit type returned** -  The transit type (e.g. tram, metro, rail, bus, ferry, cable car, gondola, funicular) is now returned with each transit maneuver.
 * **Guidance language** -  If the language option is not supplied or is unsupported then the language will be set to the default (en-US). Also, the service will return the language in the trip results.
 * **Update multimodal path algorithm** - Applied some fixes to multimodal path algorithm. In particular fixed a bug where the wrong sortcost was added to the adjacency list. Also separated "in-station" transfer costs from transfers between stops.
 * **Data producer updates** - Do not combine shortcut edges at gates or toll booths. Fixes avoid toll issues on routes that included shortcut edges.

## Release Date: 2016-03-07

 * **Updated all APIs to honor the optional DNT (Do not track) http header** -  This will avoid logging locations.
 * **Reduce 'Merge maneuver' verbal alert instructions** -  Only create a verbal alert instruction for a 'Merge maneuver' if the previous maneuver is > 1.5 km.
 * **Updated transit defaults.  Tweaked transit costing logic to obtain better routes.** -  use_rail = 0.6, use_transfers = 0.3, transfer_cost = 15.0 and transfer_penalty = 300.0.  Updated the TransferCostFactor to use the transfer_factor correctly.  TransitionCost for pedestrian costing bumped up from 20.0f to 30.0f when predecessor edge is a transit connection.
 * **Initial Guidance Globalization** -  Partial framework for Guidance Globalization. Started reading some guidance phrases from en-US.json file.

## Release Date: 2016-02-22

 * **Use bidirectional A* for automobile routes** - Switch to bidirectional A* for all but bus routes and short routes (where origin and destination are less than 10km apart). This improves performance and has less failure cases for longer routes. Some data import adjustments were made (02-19) to fix some issues encountered with arterial and highway hierarchies. Also only use a maximum of 2 passes for bidirecdtional A* to reduce "long time to fail" cases.
 * **Added verbal multi-cue guidance** - This combines verbal instructions when 2 successive maneuvers occur in a short amount of time (e.g., Turn right onto MainStreet. Then Turn left onto 1st Avenue).

## Release Date: 2016-02-19

 * **Data producer updates** - Reduce stop impact when all edges are links (ramps or turn channels). Update opposing edge logic to reject edges that do no have proper access (forward access == reverse access on opposing edge and vice-versa). Update ReclassifyLinks for cases where a single edge (often a service road) intersects a ramp improperly causing the ramp to reclassified when it should not be. Updated maximum OSM node Id (now exceeds 4000000000). Move lua from conf repository into mjolnir.

## Release Date: 2016-02-01

 * **Data producer updates** - Reduce speed on unpaved/rough roads. Add statistics for hgv (truck) restrictions.

## Release Date: 2016-01-26

 * **Added capability to disable narrative production** - Added the `narrative` boolean option to allow users to disable narrative production. Locations, shape, length, and time are still returned. The narrative production is enabled by default. The possible values for the `narrative` option are: false and true
 * **Added capability to mark a request with an id** - The `id` is returned with the response so a user could match to the corresponding request.
 * **Added some logging enhancements, specifically [ANALYTICS] logging** - We want to focus more on what our data is telling us by logging specific stats in Logstash.

## Release Date: 2016-01-18

 * **Data producer updates** - Data importer configuration (lua) updates to fix a bug where buses were not allowed on restricted lanes.  Fixed surface issue (change the default surface to be "compacted" for footways).

## Release Date: 2016-01-04

 * **Fixed Wrong Costing Options Applied** - Fixed a bug in which a previous requests costing options would be used as defaults for all subsequent requests.

## Release Date: 2015-12-18

 * **Fix for bus access** - Data importer configuration (lua) updates to fix a bug where bus lanes were turning off access for other modes.
 * **Fix for extra emergency data** - Data importer configuration (lua) updates to fix a bug where we were saving hospitals in the data.
 * **Bicycle costing update** - Updated kTCSlight and kTCFavorable so that cycleways are favored by default vs roads.

## Release Date: 2015-12-17

 * **Graph Tile Data Structure update** - Updated structures within graph tiles to support transit efforts and truck routing. Removed TransitTrip, changed TransitRoute and TransitStop to indexes (rather than binary search). Added access restrictions (like height and weight restrictions) and the mode which they impact to reduce need to look-up.
 * **Data producer updates** - Updated graph tile structures and import processes.

## Release Date: 2015-11-23

 * **Fixed Open App for OSRM functionality** - Added OSRM functionality back to Loki to support Open App.

## Release Date: 2015-11-13

 * **Improved narrative for unnamed walkway, cycleway, and mountain bike trail** - A generic description will be used for the street name when a walkway, cycleway, or mountain bike trail maneuver is unnamed. For example, a turn right onto a unnamed walkway maneuver will now be: "Turn right onto walkway."
 * **Fix costing bug** - Fix a bug introduced in EdgeLabel refactor (impacted time distance matrix only).

## Release Date: 2015-11-3

 * **Enhance bi-directional A* logic** - Updates to bidirectional A* algorithm to fix the route completion logic to handle cases where a long "connection" edge could lead to a sub-optimal path. Add hierarchy and shortcut logic so we can test and use bidirectional A* for driving routes. Fix the destination logic to properly handle oneways as the destination edge. Also fix U-turn detection for reverse search when hierarchy transitions occur.
 * **Change "Go" to "Head" for some instructions** - Start, exit ferry.
 * **Update to roundabout instructions** - Call out roundabouts for edges marked as links (ramps, turn channels).
 * **Update bicycle costing** - Fix the road factor (for applying weights based on road classification) and lower turn cost values.

## Data Producer Release Date: 2015-11-2

 * **Updated logic to not create shortcut edges on roundabouts** - This fixes some roundabout exit counts.

## Release Date: 2015-10-20

 * **Bug Fix for Pedestrian and Bicycle Routes** - Fixed a bug with setting the destination in the bi-directional Astar algorithm. Locations that snapped to a dead-end node would have failed the route and caused a timeout while searching for a valid path. Also fixed the elapsed time computation on the reverse path of bi-directional algorithm.

## Release Date: 2015-10-16

 * **Through Location Types** - Improved support for locations with type = "through". Routes now combine paths that meet at each through location to create a single "leg" between locations with type = "break". Paths that continue at a through location will not create a U-turn unless the path enters a "dead-end" region (neighborhood with no outbound access).
 * **Update shortcut edge logic** - Now skips long shortcut edges when close to the destination. This can lead to missing the proper connection if the shortcut is too long. Fixes #245 (thor).
 * **Per mode service limits** - Update configuration to allow setting different maximum number of locations and distance per mode.
 * **Fix shape index for trivial path** - Fix a bug where when building the the trip path for a "trivial" route (includes just one edge) where the shape index exceeded that size of the shape.

## Release Date: 2015-09-28

 * **Elevation Influenced Bicycle Routing** - Enabled elevation influenced bicycle routing. A "use-hills" option was added to the bicycle costing profile that can tune routes to avoid hills based on grade and amount of elevation change.
 * **"Loop Edge" Fix** - Fixed a bug with edges that form a loop. Split them into 2 edges during data import.
 * **Additional information returned from 'locate' method** - Added information that can be useful when debugging routes and data. Adds information about nodes and edges at a location.
 * **Guidance/Narrative Updates** - Added side of street to destination narrative. Updated verbal instructions.
