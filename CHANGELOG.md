## Release Date: 2022-??-?? Valhalla 3.4.1
* **Removed**
* **Bug Fix**
* **Enhancement**

## Release Date: 2023-05-11 Valhalla 3.4.0
* **Removed**
   * REMOVED: Docker image pushes to Dockerhub [#4033](https://github.com/valhalla/valhalla/pull/4033)
   * REMOVED: transitland references and scripts and replace with info for raw GTFS feeds [#4033](https://github.com/valhalla/valhalla/pull/3906)
* **Bug Fix**
   * FIXED: underflow of uint64_t cast for matrix time results [#3906](https://github.com/valhalla/valhalla/pull/3906)
   * FIXED: update vcpkg commit for Azure pipelines to fix libtool mirrors [#3915](https://github.com/valhalla/valhalla/pull/3915)
   * FIXED: fix CHANGELOG release year (2022->2023) [#3927](https://github.com/valhalla/valhalla/pull/3927)
   * FIXED: avoid segfault on invalid exclude_polygons input [#3907](https://github.com/valhalla/valhalla/pull/3907)
   * FIXED: allow \_WIN32_WINNT to be defined by build system [#3933](https://github.com/valhalla/valhalla/issues/3933)
   * FIXED: disconnected stop pairs in gtfs import [#3943](https://github.com/valhalla/valhalla/pull/3943)
   * FIXED: in/egress traversability in gtfs ingestion is now defaulted to kBoth to enable pedestrian access on transit connect edges and through the in/egress node [#3948](https://github.com/valhalla/valhalla/pull/3948)
   * FIXED: parsing logic needed implicit order of stations/egresses/platforms in the GTFS feeds [#3949](https://github.com/valhalla/valhalla/pull/3949)
   * FIXED: segfault in TimeDistanceMatrix [#3964](https://github.com/valhalla/valhalla/pull/3949)
   * FIXED: write multiple PBFs if the protobuf object gets too big [#3954](https://github.com/valhalla/valhalla/pull/3954)
   * FIXED: pin conan version to latest 1.x for now [#3990](https://github.com/valhalla/valhalla/pull/3990)
   * FIXED: Fix matrix_locations when used in pbf request [#3997](https://github.com/valhalla/valhalla/pull/3997)
   * FIXED: got to the point where the basic transit routing test works [#3988](https://github.com/valhalla/valhalla/pull/3988)
   * FIXED: fix build with LOGGING_LEVEL=ALL [#3992](https://github.com/valhalla/valhalla/pull/3992)
   * FIXED: transit stitching when determining whether a platform was generated [#4020](https://github.com/valhalla/valhalla/pull/4020)
   * FIXED: multimodal isochrones [#4030](https://github.com/valhalla/valhalla/pull/4030)
   * FIXED: duplicated recosting names should throw [#4042](https://github.com/valhalla/valhalla/pull/4042)
   * FIXED: Remove arch specificity from strip command of Python bindings to make it more compatible with other archs [#4040](https://github.com/valhalla/valhalla/pull/4040)
   * FIXED: GraphReader::GetShortcut no longer returns false positives or false negatives [#4019](https://github.com/valhalla/valhalla/pull/4019)
   * FIXED: Tagging with bus=permit or taxi=permit did not override access=no [#4045](https://github.com/valhalla/valhalla/pull/4045)
   * FIXED: Upgrade RapidJSON to address undefined behavior [#4051](https://github.com/valhalla/valhalla/pull/4051)
   * FIXED: time handling for transit service [#4052](https://github.com/valhalla/valhalla/pull/4052)
   * FIXED: multiple smaller bugs while testing more multimodal /route & /isochrones [#4055](https://github.com/valhalla/valhalla/pull/4055)
   * FIXED: `FindLuaJit.cmake` to include Windows paths/library names [#4067](https://github.com/valhalla/valhalla/pull/4067)
   * FIXED: Move complex turn restriction check out of can_form_shortcut() [#4047](https://github.com/valhalla/valhalla/pull/4047)
   * FIXED: fix `clear` methods on matrix algorithms and reserve some space for labels with a new config [#4075](https://github.com/valhalla/valhalla/pull/4075)
   * FIXED: fix `valhalla_build_admins` & `valhalla_ways_to_edges` argument parsing [#4097](https://github.com/valhalla/valhalla/pull/4097)
   * FIXED: fail early in `valhalla_build_admins` if parent directory can't be created, also exit with failure [#4099](https://github.com/valhalla/valhalla/pull/4099)
* **Enhancement**
   * CHANGED: replace boost::optional with C++17's std::optional where possible [#3890](https://github.com/valhalla/valhalla/pull/3890)
   * ADDED: parse `lit` tag on ways and add it to graph [#3893](https://github.com/valhalla/valhalla/pull/3893)
   * ADDED: log lat/lon of node where children link edges exceed the configured maximum [#3911](https://github.com/valhalla/valhalla/pull/3911)
   * ADDED: log matrix algorithm which was used [#3916](https://github.com/valhalla/valhalla/pull/3916)
   * UPDATED: docker base image to Ubuntu 22.04 [#3912](https://github.com/valhalla/valhalla/pull/3912)
   * CHANGED: Unify handling of single-file -Werror in all modules [#3910](https://github.com/valhalla/valhalla/pull/3910)
   * CHANGED: Build skadi with -Werror [#3935](https://github.com/valhalla/valhalla/pull/3935)
   * ADDED: Connect transit tiles to the graph [#3700](https://github.com/valhalla/valhalla/pull/3700)
   * CHANGED: switch to C++17 master branch of `just_gtfs` [#3947](https://github.com/valhalla/valhalla/pull/3947)
   * ADDED: Support for configuring a universal request timeout [#3966](https://github.com/valhalla/valhalla/pull/3966)
   * ADDED: optionally include highway=platform edges for pedestrian access [#3971](https://github.com/valhalla/valhalla/pull/3971)
   * ADDED: `use_lit` costing option for pedestrian costing [#3957](https://github.com/valhalla/valhalla/pull/3957)
   * CHANGED: Removed stray NULL values in log output[#3974](https://github.com/valhalla/valhalla/pull/3974)
   * CHANGED: More conservative estimates for cost of walking slopes [#3982](https://github.com/valhalla/valhalla/pull/3982)
   * ADDED: An option to slim down matrix response [#3987](https://github.com/valhalla/valhalla/pull/3987)
   * CHANGED: Updated url for just_gtfs library [#3994](https://github.com/valhalla/valhalla/pull/3995)
   * ADDED: Docker image pushes to Github's docker registry [#4033](https://github.com/valhalla/valhalla/pull/4033)
   * ADDED: `disable_hierarchy_pruning` costing option to find the actual optimal route for motorized costing modes, i.e `auto`, `motorcycle`, `motor_scooter`, `bus`, `truck` & `taxi`. [#4000](https://github.com/valhalla/valhalla/pull/4000)
   * CHANGED: baldr directory: remove warnings and C++17 adjustments [#4011](https://github.com/valhalla/valhalla/pull/4011)
   * UPDATED: `vcpkg` to latest master, iconv wasn't building anymore [#4066](https://github.com/valhalla/valhalla/pull/4066)
   * CHANGED: pybind11 upgrade for python 3.11 [#4067](https://github.com/valhalla/valhalla/pull/4067)
   * CHANGED: added transit level to connectivity map [#4082](https://github.com/valhalla/valhalla/pull/4082)
   * ADDED: "has_transit_tiles" & "osm_changeset" to verbose status response [#4062](https://github.com/valhalla/valhalla/pull/4062)
   * ADDED: time awareness to CostMatrix for e.g. traffic support [#4071](https://github.com/valhalla/valhalla/pull/4071)
   * UPDATED: transifex translations [#4102](https://github.com/valhalla/valhalla/pull/4102)

## Release Date: 2023-01-03 Valhalla 3.3.0
* **Removed**
* **Bug Fix**
* **Enhancement**
  * CHANGED: Upgraded from C++14 to C++17. [#3878](https://github.com/valhalla/valhalla/pull/3878)

## Release Date: 2023-01-03 Valhalla 3.2.1
* **Removed**
* **Bug Fix**
   * FIXED: valhalla_run_route was missing config logic.[#3824](https://github.com/valhalla/valhalla/pull/3824)
   * FIXED: Added missing ferry tag if manoeuver uses a ferry. It's supposed to be there according to the docs. [#3815](https://github.com/valhalla/valhalla/issues/3815)
   * FIXED: Handle hexlifying strings with unsigned chars [#3842](https://github.com/valhalla/valhalla/pull/3842)
   * FIXED: Newer clang warns on `sprintf` which becomes a compilation error (due to `Werror`) so we use `snprintf` instead [#3846](https://github.com/valhalla/valhalla/issues/3846)
   * FIXED: Build all of Mjolnir with -Werror [#3845](https://github.com/valhalla/valhalla/pull/3845)
   * FIXED: Only set most destination information once for all origins in timedistancematrix [#3830](https://github.com/valhalla/valhalla/pull/3830)
   * FIXED: Integers to expansion JSON output were cast wrongly [#3857](https://github.com/valhalla/valhalla/pull/3857)
   * FIXED: hazmat=destination should be hazmat=false and fix the truckcost usage of hazmat [#3865](https://github.com/valhalla/valhalla/pull/3865)
   * FIXED: Make sure there is at least one path which is accessible for all vehicular modes when reclassifying ferry edges [#3860](https://github.com/valhalla/valhalla/pull/3860)
   * FIXED: valhalla_build_extract was failing to determine the tile ID to include in the extract [#3864](https://github.com/valhalla/valhalla/pull/3864)
   * FIXED: valhalla_ways_to_edges missed trimming the cache when overcommitted [#3872](https://github.com/valhalla/valhalla/pull/3864)
   * FIXED: Strange detours with multi-origin/destination unidirectional A* [#3585](https://github.com/valhalla/valhalla/pull/3585)
* **Enhancement**
   * ADDED: Added has_toll, has_higway, has_ferry tags to summary field of a leg and route and a highway tag to a maneuver if it includes a highway. [#3815](https://github.com/valhalla/valhalla/issues/3815)
   * ADDED: Add time info to sources_to_targets [#3795](https://github.com/valhalla/valhalla/pull/3795)
   * ADDED: "available_actions" to the /status response [#3836](https://github.com/valhalla/valhalla/pull/3836)
   * ADDED: "waiting" field on input/output intermediate break(_through) locations to respect services times [#3849](https://github.com/valhalla/valhalla/pull/3849)
   * ADDED: --bbox & --geojson-dir options to valhalla_build_extract to only archive a subset of tiles [#3856](https://github.com/valhalla/valhalla/pull/3856)
   * CHANGED: Replace unstable c++ geos API with a mix of geos' c api and boost::geometry for admin building [#3683](https://github.com/valhalla/valhalla/pull/3683)
   * ADDED: optional write-access to traffic extract from GraphReader [#3876](https://github.com/valhalla/valhalla/pull/3876)
   * UPDATED: locales from Transifex [#3879](https://github.com/valhalla/valhalla/pull/3879)
   * CHANGED: Build most of Baldr with -Werror [#3885](https://github.com/valhalla/valhalla/pull/3885)
   * UPDATED: some documentation overhaul to slim down root's README [#3881](https://github.com/valhalla/valhalla/pull/3881)
   * CHANGED: move documentation hosting to Github Pages from readthedocs.io [#3884](https://github.com/valhalla/valhalla/pull/3884)
   * ADDED: inline config arguments to some more executables [#3873](https://github.com/valhalla/valhalla/pull/3873)

## Release Date: 2022-10-26 Valhalla 3.2.0
* **Removed**
   * REMOVED: "build-\*" docker image to decrease complexity [#3689](https://github.com/valhalla/valhalla/pull/3541)

* **Bug Fix**
   * FIXED: Fix precision losses while encoding-decoding distance parameter in openlr [#3374](https://github.com/valhalla/valhalla/pull/3374)
   * FIXED: Fix bearing calculation for openlr records [#3379](https://github.com/valhalla/valhalla/pull/3379)
   * FIXED: Some refactoring that was proposed for the PR 3379 [3381](https://github.com/valhalla/valhalla/pull/3381)
   * FIXED: Avoid calling out "keep left/right" when passing an exit [3349](https://github.com/valhalla/valhalla/pull/3349)
   * FIXED: Fix iterator decrement beyond begin() in GeoPoint::HeadingAtEndOfPolyline() method [#3393](https://github.com/valhalla/valhalla/pull/3393)
   * FIXED: Add string for Use:kPedestrianCrossing to fix null output in to_string(Use). [#3416](https://github.com/valhalla/valhalla/pull/3416)
   * FIXED: Remove simple restrictions check for pedestrian cost calculation. [#3423](https://github.com/valhalla/valhalla/pull/3423)
   * FIXED: Parse "highway=busway" OSM tag: https://wiki.openstreetmap.org/wiki/Tag:highway%3Dbusway [#3413](https://github.com/valhalla/valhalla/pull/3413)
   * FIXED: Process int_ref irrespective of `use_directions_on_ways_` [#3446](https://github.com/valhalla/valhalla/pull/3446)
   * FIXED: workaround python's ArgumentParser bug to not accept negative numbers as arguments [#3443](https://github.com/valhalla/valhalla/pull/3443)
   * FIXED: Undefined behaviour on some platforms due to unaligned reads [#3447](https://github.com/valhalla/valhalla/pull/3447)
   * FIXED: Fixed undefined behavior due to invalid shift exponent when getting edge's heading [#3450](https://github.com/valhalla/valhalla/pull/3450)
   * FIXED: Use midgard::unaligned_read in GraphTileBuilder::AddSigns [#3456](https://github.com/valhalla/valhalla/pull/3456)
   * FIXED: Relax test margin for time dependent traffic test [#3467](https://github.com/valhalla/valhalla/pull/3467)
   * FIXED: Fixed missed intersection heading [#3463](https://github.com/valhalla/valhalla/pull/3463)
   * FIXED: Stopped putting binary bytes into a string field of the protobuf TaggedValue since proto3 protects against that for cross language support [#3468](https://github.com/valhalla/valhalla/pull/3468)
   * FIXED: valhalla_service uses now loki logging config instead of deprecated tyr logging [#3481](https://github.com/valhalla/valhalla/pull/3481)
   * FIXED: Docker image `valhalla/valhalla:run-latest`: conan error + python integration [#3485](https://github.com/valhalla/valhalla/pull/3485)
   * FIXED: fix more protobuf unstable 3.x API [#3494](https://github.com/valhalla/valhalla/pull/3494)
   * FIXED: fix one more protobuf unstable 3.x API [#3501](https://github.com/valhalla/valhalla/pull/3501)
   * FIXED: Fix valhalla_build_tiles imports only bss from last osm file [#3503](https://github.com/valhalla/valhalla/pull/3503)
   * FIXED: Fix total_run_stat.sh script. [#3511](https://github.com/valhalla/valhalla/pull/3511)
   * FIXED: Both `hov:designated` and `hov:minimum` have to be correctly set for the way to be considered hov-only [#3526](https://github.com/valhalla/valhalla/pull/3526)
   * FIXED: Wrong out index in route intersections [#3541](https://github.com/valhalla/valhalla/pull/3541)
   * FIXED: fix valhalla_export_edges: missing null columns separator [#3543](https://github.com/valhalla/valhalla/pull/3543)
   * FIXED: Removed/updated narrative language aliases that are not IETF BCP47 compliant [#3546](https://github.com/valhalla/valhalla/pull/3546)
   * FIXED: Wrong predecessor opposing edge in dijkstra's expansion [#3528](https://github.com/valhalla/valhalla/pull/3528)
   * FIXED: exit and exit_verbal in Russian locale should be same [#3545](https://github.com/valhalla/valhalla/pull/3545)
   * FIXED: Skip transit tiles in hierarchy builder [#3559](https://github.com/valhalla/valhalla/pull/3559)
   * FIXED: Fix some country overrides in adminconstants and add a couple new countries. [#3578](https://github.com/valhalla/valhalla/pull/3578)
   * FIXED: Improve build errors reporting [#3579](https://github.com/valhalla/valhalla/pull/3579)
   * FIXED: Fix "no elevation" values and /locate elevation response [#3571](https://github.com/valhalla/valhalla/pull/3571)
   * FIXED: Build tiles with admin/timezone support on Windows [#3580](https://github.com/valhalla/valhalla/pull/3580)
   * FIXED: admin "Saint-Martin" changed name to "Saint-Martin (France)" [#3619](https://github.com/valhalla/valhalla/pull/3619)
   * FIXED: openstreetmapspeeds global config with `null`s now supported [#3621](https://github.com/valhalla/valhalla/pull/3621)
   * FIXED: valhalla_run_matrix was failing (could not find proper max_matrix_distance) [#3635](https://github.com/valhalla/valhalla/pull/3635)
   * FIXED: Removed duplicate degrees/radians constants [#3642](https://github.com/valhalla/valhalla/pull/3642)
   * FIXED: Forgot to adapt driving side and country access rules in [#3619](https://github.com/valhalla/valhalla/pull/3619) [#3652](https://github.com/valhalla/valhalla/pull/3652)
   * FIXED: DateTime::is_conditional_active(...) incorrect end week handling [#3655](https://github.com/valhalla/valhalla/pull/3655)
   * FIXED: TimeDistanceBSSMatrix: incorrect initialization for destinations[#3659](https://github.com/valhalla/valhalla/pull/3659)
   * FIXED: Some interpolated points had invalid edge_index in trace_attributes response [#3646](https://github.com/valhalla/valhalla/pull/3670)
   * FIXED: Use a small node snap distance in map-matching. FIxes issue with incorrect turn followed by Uturn. [#3677](https://github.com/valhalla/valhalla/pull/3677)
   * FIXED: Conan error when building Docker image. [#3689](https://github.com/valhalla/valhalla/pull/3689)
   * FIXED: Allow country overrides for sidewalk [#3711](https://github.com/valhalla/valhalla/pull/3711)
   * FIXED: CostMatrix incorrect tile usage with oppedge. [#3719](https://github.com/valhalla/valhalla/pull/3719)
   * FIXED: Fix elevation serializing [#3735](https://github.com/valhalla/valhalla/pull/3735)
   * FIXED: Fix returning a potentially uninitialized value in PointXY::ClosestPoint [#3737](https://github.com/valhalla/valhalla/pull/3737)
   * FIXED: Wales and Scotland name change. [#3746](https://github.com/valhalla/valhalla/pull/3746)
   * FIXED: Pedestrian crossings are allowed for bikes [#3751](https://github.com/valhalla/valhalla/pull/3751)
   * FIXED: Fix for Mac OSx.  Small update for the workdir for the admin_sidewalk_override test.  [#3757](https://github.com/valhalla/valhalla/pull/3757)
   * FIXED: Add missing service road case from GetTripLegUse method. [#3763](https://github.com/valhalla/valhalla/pull/3763)
   * FIXED: Fix TimeDistanceMatrix results sequence [#3738](https://github.com/valhalla/valhalla/pull/3738)
   * FIXED: Fix status endpoint not reporting that the service is shutting down [#3785](https://github.com/valhalla/valhalla/pull/3785)
   * FIXED: Fix TimdDistanceMatrix SetSources and SetTargets [#3792](https://github.com/valhalla/valhalla/pull/3792)
   * FIXED: Added highway and surface factor in truckcost [#3590](https://github.com/valhalla/valhalla/pull/3590)
   * FIXED: Potential integer underflow in file suffix generation [#3783](https://github.com/valhalla/valhalla/pull/3783)
   * FIXED: Building Valhalla as a submodule [#3781](https://github.com/valhalla/valhalla/issues/3781)
   * FIXED: Fixed invalid time detection in GetSpeed [#3800](https://github.com/valhalla/valhalla/pull/3800)
   * FIXED: Osmway struct update: added up to 33 and not 32 [#3808](https://github.com/valhalla/valhalla/pull/3808)

* **Enhancement**
   * CHANGED: Pronunciation for names and destinations [#3132](https://github.com/valhalla/valhalla/pull/3132)
   * CHANGED: Requested code clean up for phonemes PR [#3356](https://github.com/valhalla/valhalla/pull/3356)
   * CHANGED: Refactor Pronunciation class to struct [#3359](https://github.com/valhalla/valhalla/pull/3359)
   * ADDED: Added support for probabale restrictions [#3361](https://github.com/valhalla/valhalla/pull/3361)
   * CHANGED: Refactored the verbal text formatter to handle logic for street name and sign [#3369](https://github.com/valhalla/valhalla/pull/3369)
   * CHANGED: return "version" and "tileset_age" on parameterless /status call [#3367](https://github.com/valhalla/valhalla/pull/3367)
   * CHANGED: de-singleton tile_extract by introducing an optional index.bin file created by valhalla_build_extract [#3281](https://github.com/valhalla/valhalla/pull/3281)
   * CHANGED: implement valhalla_build_elevation in python and add more --from-geojson & --from-graph options [#3318](https://github.com/valhalla/valhalla/pull/3318)
   * ADDED: Add boolean parameter to clear memory for edge labels from thor. [#2789](https://github.com/valhalla/valhalla/pull/2789)
   * CHANGED: Do not create statsd client in workers if it is not configured [#3394](https://github.com/valhalla/valhalla/pull/3394)
   * ADDED: Import of Bike Share Stations information in BSS Connection edges [#3411](https://github.com/valhalla/valhalla/pull/3411)
   * ADDED: Add heading to PathEdge to be able to return it on /locate [#3399](https://github.com/valhalla/valhalla/pull/3399)
   * ADDED: Add `prioritize_bidirectional` option for fast work and correct ETA calculation for `depart_at` date_time type. Smoothly stop using live-traffic [#3398](https://github.com/valhalla/valhalla/pull/3398)
   * CHANGED: Minor fix for headers  [#3436](https://github.com/valhalla/valhalla/pull/3436)
   * CHANGED: Use std::multimap for polygons returned for admin and timezone queries. Improves performance when building tiles. [#3427](https://github.com/valhalla/valhalla/pull/3427)
   * CHANGED: Refactored GraphBuilder::CreateSignInfoList [#3438](https://github.com/valhalla/valhalla/pull/3438)
   * ADDED: Add support for LZ4 compressed elevation tiles [#3401](https://github.com/valhalla/valhalla/pull/3401)
   * CHANGED: Rearranged some of the protobufs to remove redundancy [#3452](https://github.com/valhalla/valhalla/pull/3452)
   * CHANGED: overhaul python bindings [#3380](https://github.com/valhalla/valhalla/pull/3380)
   * CHANGED: Removed all protobuf defaults either by doing them in code or by relying on 0 initialization. Also deprecated best_paths and do_not_track [#3454](https://github.com/valhalla/valhalla/pull/3454)
   * ADDED: isochrone action for /expansion endpoint to track dijkstra expansion [#3215](https://github.com/valhalla/valhalla/pull/3215)
   * CHANGED: remove boost from dependencies and add conan as prep for #3346 [#3459](https://github.com/valhalla/valhalla/pull/3459)
   * CHANGED: Remove boost.program_options in favor of cxxopts header-only lib and use conan to install header-only boost. [#3346](https://github.com/valhalla/valhalla/pull/3346)
   * CHANGED: Moved all protos to proto3 for internal request/response handling [#3457](https://github.com/valhalla/valhalla/pull/3457)
   * CHANGED: Allow up to 32 outgoing link edges on a node when reclassifying links [#3483](https://github.com/valhalla/valhalla/pull/3483)
   * CHANGED: Reuse sample::get implementation [#3471](https://github.com/valhalla/valhalla/pull/3471)
   * ADDED: Beta support for interacting with the http/bindings/library via serialized and pbf objects respectively [#3464](https://github.com/valhalla/valhalla/pull/3464)
   * CHANGED: Update xcode to 12.4.0 [#3492](https://github.com/valhalla/valhalla/pull/3492)
   * ADDED: Add JSON generator to conan [#3493](https://github.com/valhalla/valhalla/pull/3493)
   * CHANGED: top_speed option: ignore live speed for speed based penalties [#3460](https://github.com/valhalla/valhalla/pull/3460)
   * ADDED: Add `include_construction` option into the config to include/exclude roads under construction from the graph [#3455](https://github.com/valhalla/valhalla/pull/3455)
   * CHANGED: Refactor options protobuf for Location and Costing objects [#3506](https://github.com/valhalla/valhalla/pull/3506)
   * CHANGED: valhalla.h and config.h don't need cmake configuration [#3502](https://github.com/valhalla/valhalla/pull/3502)
   * ADDED: New options to control what fields of the pbf are returned when pbf format responses are requested [#3207](https://github.com/valhalla/valhalla/pull/3507)
   * CHANGED: Rename tripcommon to common [#3516](https://github.com/valhalla/valhalla/pull/3516)
   * ADDED: Indoor routing - data model, data processing. [#3509](https://github.com/valhalla/valhalla/pull/3509)
   * ADDED: On-demand elevation tile fetching [#3391](https://github.com/valhalla/valhalla/pull/3391)
   * CHANGED: Remove many oneof uses from the protobuf api where the semantics of optional vs required isnt necessary [#3527](https://github.com/valhalla/valhalla/pull/3527)
   * ADDED: Indoor routing maneuvers [#3519](https://github.com/valhalla/valhalla/pull/3519)
   * ADDED: Expose reverse isochrone parameter for reverse expansion [#3528](https://github.com/valhalla/valhalla/pull/3528)
   * CHANGED: Add matrix classes to thor worker so they persist between requests. [#3560](https://github.com/valhalla/valhalla/pull/3560)
   * CHANGED: Remove `max_matrix_locations` and introduce `max_matrix_location_pairs` to configure the allowed number of total routes for the matrix action for more flexible asymmetric matrices [#3569](https://github.com/valhalla/valhalla/pull/3569)
   * CHANGED: modernized spatialite syntax [#3580](https://github.com/valhalla/valhalla/pull/3580)
   * ADDED: Options to generate partial results for time distance matrix when there is one source (one to many) or one target (many to one). [#3181](https://github.com/valhalla/valhalla/pull/3181)
   * ADDED: Enhance valhalla_build_elevation with LZ4 recompression support [#3607](https://github.com/valhalla/valhalla/pull/3607)
   * CHANGED: removed UK admin and upgraded its constituents to countries [#3619](https://github.com/valhalla/valhalla/pull/3619)
   * CHANGED: expansion service: only track requested max time/distance [#3532](https://github.com/valhalla/valhalla/pull/3509)
   * ADDED: Shorten down the request delay, when some sources/targets searches are early aborted [#3611](https://github.com/valhalla/valhalla/pull/3611)
   * ADDED: add `pre-commit` hook for running the `format.sh` script [#3637](https://github.com/valhalla/valhalla/pull/3637)
   * CHANGED: upgrade pybind11 to v2.9.2 to remove cmake warning [#3658](https://github.com/valhalla/valhalla/pull/3658)
   * ADDED: tests for just_gtfs reading and writing feeds [#3665](https://github.com/valhalla/valhalla/pull/3665)
   * CHANGED: Precise definition of types of edges on which BSS could be projected [#3658](https://github.com/valhalla/valhalla/pull/3663)
   * CHANGED: Remove duplicate implementation of `adjust_scores` [#3673](https://github.com/valhalla/valhalla/pull/3673)
   * ADDED: convert GTFS data into protobuf tiles [#3629](https://github.com/valhalla/valhalla/issues/3629)
   * CHANGED: Use `starts_with()` instead of `substr(0, N)` getting and comparing to prefix [#3702](https://github.com/valhalla/valhalla/pull/3702)
   * ADDED: Ferry support for HGV [#3710](https://github.com/valhalla/valhalla/issues/3710)
   * ADDED: Linting & formatting checks for Python code [#3713](https://github.com/valhalla/valhalla/pull/3713)
   * CHANGED: rename Turkey admin to Türkiye [#3720](https://github.com/valhalla/valhalla/pull/3713)
   * CHANGED: bumped vcpkg version to "2022.08.15" [#3754](https://github.com/valhalla/valhalla/pull/3754)
   * CHANGED: chore: Updates to clang-format 11.0.0 [#3533](https://github.com/valhalla/valhalla/pull/3533)
   * CHANGED: Ported trace_attributes serialization to RapidJSON. [#3333](https://github.com/valhalla/valhalla/pull/3333)
   * ADDED: Add helpers for DirectedEdgeExt and save them to file in GraphTileBuilder [#3562](https://github.com/valhalla/valhalla/pull/3562)
   * ADDED: Fixed Speed costing option [#3576](https://github.com/valhalla/valhalla/pull/3576)
   * ADDED: axle_count costing option for hgv [#3648](https://github.com/valhalla/valhalla/pull/3648)
   * ADDED: Matrix action for gurka [#3793](https://github.com/valhalla/valhalla/pull/3793)
   * ADDED: Add warnings array to response. [#3588](https://github.com/valhalla/valhalla/pull/3588)
   * CHANGED: Templatized TimeDistanceMatrix for forward/reverse search [#3773](https://github.com/valhalla/valhalla/pull/3773)
   * CHANGED: Templatized TimeDistanceBSSMatrix for forward/reverse search [#3778](https://github.com/valhalla/valhalla/pull/3778)
   * CHANGED: error code 154 shows distance limit in error message [#3779](https://github.com/valhalla/valhalla/pull/3779)

## Release Date: 2021-10-07 Valhalla 3.1.4
* **Removed**
* **Bug Fix**
   * FIXED: Revert default speed boost for turn channels [#3232](https://github.com/valhalla/valhalla/pull/3232)
   * FIXED: Use the right tile to get country for incident [#3235](https://github.com/valhalla/valhalla/pull/3235)
   * FIXED: Fix factors passed to `RelaxHierarchyLimits` [#3253](https://github.com/valhalla/valhalla/pull/3253)
   * FIXED: Fix TransitionCostReverse usage [#3260](https://github.com/valhalla/valhalla/pull/3260)
   * FIXED: Fix Tagged Value Support in EdgeInfo [#3262](https://github.com/valhalla/valhalla/issues/3262)
   * FIXED: TransitionCostReverse fix: revert internal_turn change [#3271](https://github.com/valhalla/valhalla/issues/3271)
   * FIXED: Optimize tiles usage in reach-based pruning [#3294](https://github.com/valhalla/valhalla/pull/3294)
   * FIXED: Slip lane detection: track visited nodes to avoid infinite loops [#3297](https://github.com/valhalla/valhalla/pull/3297)
   * FIXED: Fix distance value in a 0-length road [#3185](https://github.com/valhalla/valhalla/pull/3185)
   * FIXED: Trivial routes were broken when origin was node snapped and destnation was not and vice-versa for reverse astar [#3299](https://github.com/valhalla/valhalla/pull/3299)
   * FIXED: Tweaked TestAvoids map to get TestAvoidShortcutsTruck working [#3301](https://github.com/valhalla/valhalla/pull/3301)
   * FIXED: Overflow in sequence sort [#3303](https://github.com/valhalla/valhalla/pull/3303)
   * FIXED: Setting statsd tags in config via valhalla_build_config [#3225](https://github.com/valhalla/valhalla/pull/3225)
   * FIXED: Cache for gzipped elevation tiles [#3120](https://github.com/valhalla/valhalla/pull/3120)
   * FIXED: Current time conversion regression introduced in unidirectional algorithm refractor [#3278](https://github.com/valhalla/valhalla/issues/3278)
   * FIXED: Make combine_route_stats.py properly quote CSV output (best practice improvement) [#3328](https://github.com/valhalla/valhalla/pull/3328)
   * FIXED: Merge edge segment records in map matching properly so that resulting edge indices in trace_attributes are valid [#3280](https://github.com/valhalla/valhalla/pull/3280)
   * FIXED: Shape walking map matcher now sets correct edge candidates used in the match for origin and destination location [#3329](https://github.com/valhalla/valhalla/pull/3329)
   * FIXED: Better hash function of GraphId [#3332](https://github.com/valhalla/valhalla/pull/3332)

* **Enhancement**
   * CHANGED: Favor turn channels more [#3222](https://github.com/valhalla/valhalla/pull/3222)
   * CHANGED: Rename `valhalla::midgard::logging::LogLevel` enumerators to avoid clash with common macros [#3237](https://github.com/valhalla/valhalla/pull/3237)
   * CHANGED: Move pre-defined algorithm-based factors inside `RelaxHierarchyLimits` [#3253](https://github.com/valhalla/valhalla/pull/3253)
   * ADDED: Reject alternatives with too long detours [#3238](https://github.com/valhalla/valhalla/pull/3238)
   * ADDED: Added info to /status endpoint [#3008](https://github.com/valhalla/valhalla/pull/3008)
   * ADDED: Added stop and give_way/yield signs to the data and traffic signal fixes [#3251](https://github.com/valhalla/valhalla/pull/3251)
   * ADDED: use_hills for pedestrian costing, which also affects the walking speed [#3234](https://github.com/valhalla/valhalla/pull/3234)
   * CHANGED: Fixed cost threshold fot bidirectional astar. Implemented reach-based pruning for suboptimal branches [#3257](https://github.com/valhalla/valhalla/pull/3257)
   * ADDED: Added `exclude_unpaved` request parameter [#3240](https://github.com/valhalla/valhalla/pull/3240)
   * ADDED: Added support for routing onto HOV/HOT lanes via request parameters `include_hot`, `include_hov2`, and `include_hov3` [#3273](https://github.com/valhalla/valhalla/pull/3273)
   * ADDED: Add Z-level field to `EdgeInfo`. [#3261](https://github.com/valhalla/valhalla/pull/3261)
   * CHANGED: Calculate stretch threshold for alternatives based on the optimal route cost [#3276](https://github.com/valhalla/valhalla/pull/3276)
   * ADDED: Add `preferred_z_level` as a parameter of loki requests. [#3270](https://github.com/valhalla/valhalla/pull/3270)
   * ADDED: Add `preferred_layer` as a parameter of loki requests. [#3270](https://github.com/valhalla/valhalla/pull/3270)
   * ADDED: Exposing service area names in passive maneuvers. [#3277](https://github.com/valhalla/valhalla/pull/3277)
   * ADDED: Added traffic signal and stop sign check for stop impact. These traffic signals and stop sign are located on edges. [#3279](https://github.com/valhalla/valhalla/pull/3279)
   * CHANGED: Improved sharing criterion to obtain more reasonable alternatives; extended alternatives search [#3302](https://github.com/valhalla/valhalla/pull/3302)
   * ADDED: pull ubuntu:20.04 base image before building [#3233](https://github.com/valhalla/valhalla/pull/3223)
   * CHANGED: Improve Loki nearest-neighbour performance for large radius searches in open space [#3233](https://github.com/valhalla/valhalla/pull/3324)
   * ADDED: testing infrastructure for scripts and valhalla_build_config tests [#3308](https://github.com/valhalla/valhalla/pull/3308)
   * ADDED: Shape points and information about where intermediate locations are placed along the legs of a route [#3274](https://github.com/valhalla/valhalla/pull/3274)
   * CHANGED: Improved existing hov lane transition test case to make more realistic [#3330](https://github.com/valhalla/valhalla/pull/3330)
   * CHANGED: Update python usage in all scripts to python3 [#3337](https://github.com/valhalla/valhalla/pull/3337)
   * ADDED: Added `exclude_cash_only_tolls` request parameter [#3341](https://github.com/valhalla/valhalla/pull/3341)
   * CHANGED: Update api-reference for street_names [#3342](https://github.com/valhalla/valhalla/pull/3342)
   * ADDED: Disable msse2 flags when building on Apple Silicon chip [#3327](https://github.com/valhalla/valhalla/pull/3327)

## Release Date: 2021-07-20 Valhalla 3.1.3
* **Removed**
   * REMOVED: Unused overloads of `to_response` function [#3167](https://github.com/valhalla/valhalla/pull/3167)

* **Bug Fix**
   * FIXED: Fix heading on small edge [#3114](https://github.com/valhalla/valhalla/pull/3114)
   * FIXED: Added support for `access=psv`, which disables routing on these nodes and edges unless the mode is taxi or bus [#3107](https://github.com/valhalla/valhalla/pull/3107)
   * FIXED: Disables logging in CI to catch issues [#3121](https://github.com/valhalla/valhalla/pull/3121)
   * FIXED: Fixed U-turns through service roads [#3082](https://github.com/valhalla/valhalla/pull/3082)
   * FIXED: Added forgotten penalties for kLivingStreet and kTrack for pedestrian costing model [#3116](https://github.com/valhalla/valhalla/pull/3116)
   * FIXED: Updated the reverse turn bounds [#3122](https://github.com/valhalla/valhalla/pull/3122)
   * FIXED: Missing fork maneuver [#3134](https://github.com/valhalla/valhalla/pull/3134)
   * FIXED: Update turn channel logic to call out specific turn at the end of the turn channel if needed [#3140](https://github.com/valhalla/valhalla/pull/3140)
   * FIXED: Fixed cost thresholds for TimeDistanceMatrix. [#3131](https://github.com/valhalla/valhalla/pull/3131)
   * FIXED: Use distance threshold in hierarchy limits for bidirectional astar to expand more important lower level roads [#3156](https://github.com/valhalla/valhalla/pull/3156)
   * FIXED: Fixed incorrect dead-end roundabout labels. [#3129](https://github.com/valhalla/valhalla/pull/3129)
   * FIXED: googletest wasn't really updated in #3166 [#3187](https://github.com/valhalla/valhalla/pull/3187)
   * FIXED: Minor fix of benchmark code [#3190](https://github.com/valhalla/valhalla/pull/3190)
   * FIXED: avoid_polygons intersected edges as polygons instead of linestrings [#3194]((https://github.com/valhalla/valhalla/pull/3194)
   * FIXED: when binning horizontal edge shapes using single precision floats (converted from not double precision floats) allowed for the possiblity of marking many many tiles no where near the shape [#3204](https://github.com/valhalla/valhalla/pull/3204)
   * FIXED: Fix improper iterator usage in ManeuversBuilder [#3205](https://github.com/valhalla/valhalla/pull/3205)
   * FIXED: Modified approach for retrieving signs from a directed edge #3166 [#3208](https://github.com/valhalla/valhalla/pull/3208)
   * FIXED: Improve turn channel classification: detect slip lanes [#3196](https://github.com/valhalla/valhalla/pull/3196)
   * FIXED: Compatibility with older boost::optional versions [#3219](https://github.com/valhalla/valhalla/pull/3219)
   * FIXED: Older boost.geometry versions don't have correct() for geographic rings [#3218](https://github.com/valhalla/valhalla/pull/3218)
   * FIXED: Use default road speed for bicycle costing so traffic does not reduce penalty on high speed roads. [#3143](https://github.com/valhalla/valhalla/pull/3143)

* **Enhancement**
   * CHANGED: Refactor base costing options parsing to handle more common stuff in a one place [#3125](https://github.com/valhalla/valhalla/pull/3125)
   * CHANGED: Unified Sign/SignElement into sign.proto [#3146](https://github.com/valhalla/valhalla/pull/3146)
   * ADDED: New verbal succinct transition instruction to maneuver & narrativebuilder. Currently this instruction will be used in place of a very long street name to avoid repetition of long names [#2844](https://github.com/valhalla/valhalla/pull/2844)
   * ADDED: Added oneway support for pedestrian access and foot restrictions [#3123](https://github.com/valhalla/valhalla/pull/3123)
   * ADDED: Exposing rest-area names in passive maneuvers [#3172](https://github.com/valhalla/valhalla/pull/3172)
   * CHORE: Updates robin-hood-hashing third-party library
   * ADDED: Support `barrier=yes|swing_gate|jersey_barrier` tags [#3154](https://github.com/valhalla/valhalla/pull/3154)
   * ADDED: Maintain `access=permit|residents` tags as private [#3149](https://github.com/valhalla/valhalla/pull/3149)
   * CHANGED: Replace `avoid_*` API parameters with more accurate `exclude_*` [#3093](https://github.com/valhalla/valhalla/pull/3093)
   * ADDED: Penalize private gates [#3144](https://github.com/valhalla/valhalla/pull/3144)
   * CHANGED: Renamed protobuf Sign/SignElement to TripSign/TripSignElement [#3168](https://github.com/valhalla/valhalla/pull/3168)
   * CHORE: Updates googletest to release-1.11.0 [#3166](https://github.com/valhalla/valhalla/pull/3166)
   * CHORE: Enables -Wall on sif sources [#3178](https://github.com/valhalla/valhalla/pull/3178)
   * ADDED: Allow going through accessible `barrier=bollard` and penalize routing through it, when the access is private [#3175](https://github.com/valhalla/valhalla/pull/3175)
   * ADDED: Add country code to incident metadata [#3169](https://github.com/valhalla/valhalla/pull/3169)
   * CHANGED: Use distance instead of time to check limited sharing criteria [#3183](https://github.com/valhalla/valhalla/pull/3183)
   * ADDED: Introduced a new via_waypoints array on the leg in the osrm route serializer that describes where a particular waypoint from the root-level array matches to the route. [#3189](https://github.com/valhalla/valhalla/pull/3189)
   * ADDED: Added vehicle width and height as an option for auto (and derived: taxi, bus, hov) profile (https://github.com/valhalla/valhalla/pull/3179)
   * ADDED: Support for statsd integration for basic error and requests metrics [#3191](https://github.com/valhalla/valhalla/pull/3191)
   * CHANGED: Get rid of typeid in statistics-related code. [#3227](https://github.com/valhalla/valhalla/pull/3227)

## Release Date: 2021-05-26 Valhalla 3.1.2
* **Removed**
* **Bug Fix**
   * FIXED: Change unnamed road intersections from being treated as penil point u-turns [#3084](https://github.com/valhalla/valhalla/pull/3084)
   * FIXED: Fix TimeDepReverse termination and path cost calculation (for arrive_by routing) [#2987](https://github.com/valhalla/valhalla/pull/2987)
   * FIXED: Isochrone (::Generalize()) fix to avoid generating self-intersecting polygons [#3026](https://github.com/valhalla/valhalla/pull/3026)
   * FIXED: Handle day_on/day_off/hour_on/hour_off restrictions [#3029](https://github.com/valhalla/valhalla/pull/3029)
   * FIXED: Apply conditional restrictions with dow only to the edges when routing [#3039](https://github.com/valhalla/valhalla/pull/3039)
   * FIXED: Missing locking in incident handler needed to hang out to scop lock rather than let the temporary disolve [#3046](https://github.com/valhalla/valhalla/pull/3046)
   * FIXED: Continuous lane guidance fix [#3054](https://github.com/valhalla/valhalla/pull/3054)
   * FIXED: Fix reclassification for "shorter" ferries and rail ferries (for Chunnel routing issues) [#3038](https://github.com/valhalla/valhalla/pull/3038)
   * FIXED: Incorrect routing through motor_vehicle:conditional=destination. [#3041](https://github.com/valhalla/valhalla/pull/3041)
   * FIXED: Allow destination-only routing on the first-pass for non bidirectional Astar algorithms. [#3085](https://github.com/valhalla/valhalla/pull/3085)
   * FIXED: Highway/ramp lane bifurcation [#3088](https://github.com/valhalla/valhalla/pull/3088)
   * FIXED: out of bound access of tile hierarchy in base_ll function in graphheader [#3089](https://github.com/valhalla/valhalla/pull/3089)
   * FIXED: include shortcuts in avoid edge set for avoid_polygons [#3090](https://github.com/valhalla/valhalla/pull/3090)

* **Enhancement**
   * CHANGED: Refactor timedep forward/reverse to reduce code repetition [#2987](https://github.com/valhalla/valhalla/pull/2987)
   * CHANGED: Sync translation files with Transifex command line tool [#3030](https://github.com/valhalla/valhalla/pull/3030)
   * CHANGED: Use osm tags in links reclassification algorithm in order to reduce false positive downgrades [#3042](https://github.com/valhalla/valhalla/pull/3042)
   * CHANGED: Use CircleCI XL instances for linux based builds [#3043](https://github.com/valhalla/valhalla/pull/3043)
   * ADDED: ci: Enable undefined sanitizer [#2999](https://github.com/valhalla/valhalla/pull/2999)
   * ADDED: Optionally pass preconstructed graphreader to connectivity map [#3046](https://github.com/valhalla/valhalla/pull/3046)
   * CHANGED: ci: Skip Win CI runs for irrelevant files [#3014](https://github.com/valhalla/valhalla/pull/3014)
   * ADDED: Allow configuration-driven default speed assignment based on edge properties [#3055](https://github.com/valhalla/valhalla/pull/3055)
   * CHANGED: Use std::shared_ptr in case if ENABLE_THREAD_SAFE_TILE_REF_COUNT is ON. [#3067](https://github.com/valhalla/valhalla/pull/3067)
   * CHANGED: Reduce stop impact when driving in parking lots [#3051](https://github.com/valhalla/valhalla/pull/3051)
   * ADDED: Added another through route test [#3074](https://github.com/valhalla/valhalla/pull/3074)
   * ADDED: Adds incident-length to metadata proto [#3083](https://github.com/valhalla/valhalla/pull/3083)
   * ADDED: Do not penalize gates that have allowed access [#3078](https://github.com/valhalla/valhalla/pull/3078)
   * ADDED: Added missing k/v pairs to taginfo.json.  Updated PR template. [#3101](https://github.com/valhalla/valhalla/pull/3101)
   * CHANGED: Serialize isochrone 'contour' properties as floating point so they match user supplied value [#3078](https://github.com/valhalla/valhalla/pull/3095)
   * NIT: Enables compiler warnings as errors in midgard module [#3104](https://github.com/valhalla/valhalla/pull/3104)
   * CHANGED: Check all tiles for nullptr that reads from graphreader to avoid fails in case tiles might be missing. [#3065](https://github.com/valhalla/valhalla/pull/3065)

## Release Date: 2021-04-21 Valhalla 3.1.1
* **Removed**
   * REMOVED: The tossing of private roads in [#1960](https://github.com/valhalla/valhalla/pull/1960) was too aggressive and resulted in a lot of no routes.  Reverted this logic.  [#2934](https://github.com/valhalla/valhalla/pull/2934)
   * REMOVED: stray references to node bindings [#3012](https://github.com/valhalla/valhalla/pull/3012)

* **Bug Fix**
   * FIXED: Fix compression_utils.cc::inflate(...) throw - make it catchable [#2839](https://github.com/valhalla/valhalla/pull/2839)
   * FIXED: Fix compiler errors if HAVE_HTTP not enabled [#2807](https://github.com/valhalla/valhalla/pull/2807)
   * FIXED: Fix alternate route serialization [#2811](https://github.com/valhalla/valhalla/pull/2811)
   * FIXED: Store restrictions in the right tile [#2781](https://github.com/valhalla/valhalla/pull/2781)
   * FIXED: Failing to write tiles because of racing directory creation [#2810](https://github.com/valhalla/valhalla/pull/2810)
   * FIXED: Regression in stopping expansion on transitions down in time-dependent routes [#2815](https://github.com/valhalla/valhalla/pull/2815)
   * FIXED: Fix crash in loki when trace_route is called with 2 locations.[#2817](https://github.com/valhalla/valhalla/pull/2817)
   * FIXED: Mark the restriction start and end as via ways to fix IsBridgingEdge function in Bidirectional Astar [#2796](https://github.com/valhalla/valhalla/pull/2796)
   * FIXED: Dont add predictive traffic to the tile if it's empty [#2826](https://github.com/valhalla/valhalla/pull/2826)
   * FIXED: Fix logic bidirectional astar to avoid double u-turns and extra detours [#2802](https://github.com/valhalla/valhalla/pull/2802)
   * FIXED: Re-enable transition cost for motorcycle profile [#2837](https://github.com/valhalla/valhalla/pull/2837)
   * FIXED: Increase limits for timedep_* algorithms. Split track_factor into edge factor and transition penalty [#2845](https://github.com/valhalla/valhalla/pull/2845)
   * FIXED: Loki was looking up the wrong costing enum for avoids [#2856](https://github.com/valhalla/valhalla/pull/2856)
   * FIXED: Fix way_ids -> graph_ids conversion for complex restrictions: handle cases when a way is split into multiple edges [#2848](https://github.com/valhalla/valhalla/pull/2848)
   * FIXED: Honor access mode while matching OSMRestriction with the graph [#2849](https://github.com/valhalla/valhalla/pull/2849)
   * FIXED: Ensure route summaries are unique among all returned route/legs [#2874](https://github.com/valhalla/valhalla/pull/2874)
   * FIXED: Fix compilation errors when boost < 1.68 and libprotobuf < 3.6  [#2878](https://github.com/valhalla/valhalla/pull/2878)
   * FIXED: Allow u-turns at no-access barriers when forced by heading [#2875](https://github.com/valhalla/valhalla/pull/2875)
   * FIXED: Fixed "No route found" error in case of multipoint request with locations near low reachability edges [#2914](https://github.com/valhalla/valhalla/pull/2914)
   * FIXED: Python bindings installation [#2751](https://github.com/valhalla/valhalla/issues/2751)
   * FIXED: Skip bindings if there's no Python development version [#2893](https://github.com/valhalla/valhalla/pull/2893)
   * FIXED: Use CMakes built-in Python variables to configure installation [#2931](https://github.com/valhalla/valhalla/pull/2931)
   * FIXED: Sometimes emitting zero-length route geometry when traffic splits edge twice [#2943](https://github.com/valhalla/valhalla/pull/2943)
   * FIXED: Fix map-match segfault when gps-points project very near a node [#2946](https://github.com/valhalla/valhalla/pull/2946)
   * FIXED: Use kServiceRoad edges while searching for ferry connection [#2933](https://github.com/valhalla/valhalla/pull/2933)
   * FIXED: Enhanced logic for IsTurnChannelManeuverCombinable [#2952](https://github.com/valhalla/valhalla/pull/2952)
   * FIXED: Restore compatibility with gcc 6.3.0, libprotobuf 3.0.0, boost v1.62.0 [#2953](https://github.com/valhalla/valhalla/pull/2953)
   * FIXED: Dont abort bidirectional a-star search if only one direction is exhausted [#2936](https://github.com/valhalla/valhalla/pull/2936)
   * FIXED: Fixed missing comma in the scripts/valhalla_build_config [#2963](https://github.com/valhalla/valhalla/pull/2963)
   * FIXED: Reverse and Multimodal Isochrones were returning forward results [#2967](https://github.com/valhalla/valhalla/pull/2967)
   * FIXED: Map-match fix for first gps-point being exactly equal to street shape-point [#2977](https://github.com/valhalla/valhalla/pull/2977)
   * FIXED: Add missing GEOS:GEOS dep to mjolnir target [#2901](https://github.com/valhalla/valhalla/pull/2901)
   * FIXED: Allow expansion into a region when not_thru_pruning is false on 2nd pass [#2978](https://github.com/valhalla/valhalla/pull/2978)
   * FIXED: Fix polygon area calculation: use Shoelace formula [#2927](https://github.com/valhalla/valhalla/pull/2927)
   * FIXED: Isochrone: orient segments/rings acoording to the right-hand rule [#2932](https://github.com/valhalla/valhalla/pull/2932)
   * FIXED: Parsenodes fix: check if index is out-of-bound first [#2984](https://github.com/valhalla/valhalla/pull/2984)
   * FIXED: Fix for unique-summary logic [#2996](https://github.com/valhalla/valhalla/pull/2996)
   * FIXED: Isochrone: handle origin edges properly [#2990](https://github.com/valhalla/valhalla/pull/2990)
   * FIXED: Annotations fail with returning NaN speed when the same point is duplicated in route geometry [#2992](https://github.com/valhalla/valhalla/pull/2992)
   * FIXED: Fix run_with_server.py to work on macOS [#3003](https://github.com/valhalla/valhalla/pull/3003)
   * FIXED: Removed unexpected maneuvers at sharp bends [#2968](https://github.com/valhalla/valhalla/pull/2968)
   * FIXED: Remove large number formatting for non-US countries [#3015](https://github.com/valhalla/valhalla/pull/3015)
   * FIXED: Odin undefined behaviour: handle case when xedgeuse is not initialized [#3020](https://github.com/valhalla/valhalla/pull/3020)

* **Enhancement**
   * Pedestrian crossing should be a separate TripLeg_Use [#2950](https://github.com/valhalla/valhalla/pull/2950)
   * CHANGED: Azure uses ninja as generator [#2779](https://github.com/valhalla/valhalla/pull/2779)
   * ADDED: Support for date_time type invariant for map matching [#2712](https://github.com/valhalla/valhalla/pull/2712)
   * ADDED: Add Bulgarian locale [#2825](https://github.com/valhalla/valhalla/pull/2825)
   * FIXED: No need for write permissions on tarball indices [#2822](https://github.com/valhalla/valhalla/pull/2822)
   * ADDED: nit: Links debug build with lld [#2813](https://github.com/valhalla/valhalla/pull/2813)
   * ADDED: Add costing option `use_living_streets` to avoid or favor living streets in route. [#2788](https://github.com/valhalla/valhalla/pull/2788)
   * CHANGED: Do not allocate mapped_cache vector in skadi when no elevation source is provided. [#2841](https://github.com/valhalla/valhalla/pull/2841)
   * ADDED: avoid_polygons logic [#2750](https://github.com/valhalla/valhalla/pull/2750)
   * ADDED: Added support for destination for conditional access restrictions [#2857](https://github.com/valhalla/valhalla/pull/2857)
   * CHANGED: Large sequences are now merge sorted which can be dramatically faster with certain hardware configurations. This is especially useful in speeding up the earlier stages (parsing, graph construction) of tile building [#2850](https://github.com/valhalla/valhalla/pull/2850)
   * CHANGED: When creating the intial graph edges by setting at which nodes they start and end, first mark the indices of those nodes in another sequence and then sort them by edgeid so that we can do the setting of start and end node sequentially in the edges file. This is much more efficient on certain hardware configurations [#2851](https://github.com/valhalla/valhalla/pull/2851)
   * CHANGED: Use relative cost threshold to extend search in bidirectional astar in order to find more alternates [#2868](https://github.com/valhalla/valhalla/pull/2868)
   * CHANGED: Throw an exception if directory does not exist when building traffic extract [#2871](https://github.com/valhalla/valhalla/pull/2871)
   * CHANGED: Support for ignoring multiple consecutive closures at start/end locations [#2846](https://github.com/valhalla/valhalla/pull/2846)
   * ADDED: Added sac_scale to trace_attributes output and locate edge output [#2818](https://github.com/valhalla/valhalla/pull/2818)
   * ADDED: Ukrainian language translations [#2882](https://github.com/valhalla/valhalla/pull/2882)
   * ADDED: Add support for closure annotations [#2816](https://github.com/valhalla/valhalla/pull/2816)
   * ADDED: Add costing option `service_factor`. Implement possibility to avoid or favor generic service roads in route for all costing options. [#2870](https://github.com/valhalla/valhalla/pull/2870)
   * CHANGED: Reduce stop impact cost when flow data is present [#2891](https://github.com/valhalla/valhalla/pull/2891)
   * CHANGED: Update visual compare script [#2803](https://github.com/valhalla/valhalla/pull/2803)
   * CHANGED: Service roads are not penalized for `pedestrian` costing by default. [#2898](https://github.com/valhalla/valhalla/pull/2898)
   * ADDED: Add complex mandatory restrictions support [#2766](https://github.com/valhalla/valhalla/pull/2766)
   * ADDED: Status endpoint for future status info and health checking of running service [#2907](https://github.com/valhalla/valhalla/pull/2907)
   * ADDED: Add min_level argument to valhalla_ways_to_edges [#2918](https://github.com/valhalla/valhalla/pull/2918)
   * ADDED: Adding ability to store the roundabout_exit_turn_degree to the maneuver [#2941](https://github.com/valhalla/valhalla/pull/2941)
   * ADDED: Penalize pencil point uturns and uturns at short internal edges. Note: `motorcycle` and `motor_scooter` models do not penalize on short internal edges. No new uturn penalty logic has been added to the pedestrian and bicycle costing models. [#2944](https://github.com/valhalla/valhalla/pull/2944)
   * CHANGED: Allow config object to be passed-in to path algorithms [#2949](https://github.com/valhalla/valhalla/pull/2949)
   * CHANGED: Allow disabling Werror
   * ADDED: Add ability to build Valhalla modules as STATIC libraries. [#2957](https://github.com/valhalla/valhalla/pull/2957)
   * NIT: Enables compiler warnings in part of mjolnir module [#2922](https://github.com/valhalla/valhalla/pull/2922)
   * CHANGED: Refactor isochrone/reachability forward/reverse search to reduce code repetition [#2969](https://github.com/valhalla/valhalla/pull/2969)
   * ADDED: Set the roundabout exit shape index when we are collapsing the roundabout maneuvers. [#2975](https://github.com/valhalla/valhalla/pull/2975)
   * CHANGED: Penalized closed edges if using them at start/end locations [#2964](https://github.com/valhalla/valhalla/pull/2964)
   * ADDED: Add shoulder to trace_attributes output. [#2980](https://github.com/valhalla/valhalla/pull/2980)
   * CHANGED: Refactor bidirectional astar forward/reverse search to reduce code repetition [#2970](https://github.com/valhalla/valhalla/pull/2970)
   * CHANGED: Factor for service roads is 1.0 by default. [#2988](https://github.com/valhalla/valhalla/pull/2988)
   * ADDED: Support for conditionally skipping CI runs [#2986](https://github.com/valhalla/valhalla/pull/2986)
   * ADDED: Add instructions for building valhalla on `arm64` macbook [#2997](https://github.com/valhalla/valhalla/pull/2997)
   * NIT: Enables compiler warnings in part of mjolnir module [#2995](https://github.com/valhalla/valhalla/pull/2995)
   * CHANGED: nit(rename): Renames the encoded live speed properties [#2998](https://github.com/valhalla/valhalla/pull/2998)
   * ADDED: ci: Vendors the codecov script [#3002](https://github.com/valhalla/valhalla/pull/3002)
   * CHANGED: Allow None build type [#3005](https://github.com/valhalla/valhalla/pull/3005)
   * CHANGED: ci: Build Python bindings for Mac OS [#3013](https://github.com/valhalla/valhalla/pull/3013)

## Release Date: 2021-01-25 Valhalla 3.1.0
* **Removed**
   * REMOVED: Remove Node bindings. [#2502](https://github.com/valhalla/valhalla/pull/2502)
   * REMOVED: appveyor builds. [#2550](https://github.com/valhalla/valhalla/pull/2550)
   * REMOVED: Removed x86 CI builds. [#2792](https://github.com/valhalla/valhalla/pull/2792)

* **Bug Fix**
   * FIXED: Crazy ETAs.  If a way has forward speed with no backward speed and it is not oneway, then we must set the default speed.  The reverse logic applies as well.  If a way has no backward speed but has a forward speed and it is not a oneway, then set the default speed. [#2102](https://github.com/valhalla/valhalla/pull/2102)
   * FIXED: Map matching elapsed times spliced amongst different legs and discontinuities are now correct [#2104](https://github.com/valhalla/valhalla/pull/2104)
   * FIXED: Date time information is now propogated amongst different legs and discontinuities [#2107](https://github.com/valhalla/valhalla/pull/2107)
   * FIXED: Adds support for geos-3.8 c++ api [#2021](https://github.com/valhalla/valhalla/issues/2021)
   * FIXED: Updated the osrm serializer to not set junction name for osrm origin/start maneuver - this is not helpful since we are not transitioning through the intersection.  [#2121](https://github.com/valhalla/valhalla/pull/2121)
   * FIXED: Removes precomputing of edge-costs which lead to wrong results [#2120](https://github.com/valhalla/valhalla/pull/2120)
   * FIXED: Complex turn-restriction invalidates edge marked as kPermanent [#2103](https://github.com/valhalla/valhalla/issues/2103)
   * FIXED: Fixes bug with inverted time-restriction parsing [#2167](https://github.com/valhalla/valhalla/pull/2167)
   * FIXED: Fixed several bugs with numeric underflow in map-matching trip durations. These may
     occur when serializing match results where adjacent trace points appear out-of-sequence on the
     same edge [#2178](https://github.com/valhalla/valhalla/pull/2178)
     - `MapMatcher::FormPath` now catches route discontinuities on the same edge when the distance
       percentage along don't agree. The trip leg builder builds disconnected legs on a single edge
       to avoid duration underflow.
     - Correctly populate edge groups when matching results contain loops. When a loop occurs,
       the leg builder now starts at the correct edge where the loop ends, and correctly accounts
       for any contained edges.
     - Duration over-trimming at the terminating edge of a match.
   * FIXED: Increased internal precision of time tracking per edge and maneuver so that maneuver times sum to the same time represented in the leg summary [#2195](https://github.com/valhalla/valhalla/pull/2195)
   * FIXED: Tagged speeds were not properly marked. We were not using forward and backward speeds to flag if a speed is tagged or not.  Should not update turn channel speeds if we are not inferring them.  Added additional logic to handle PH in the conditional restrictions. Do not update stop impact for ramps if they are marked as internal. [#2198](https://github.com/valhalla/valhalla/pull/2198)
   * FIXED: Fixed the sharp turn phrase [#2226](https://github.com/valhalla/valhalla/pull/2226)
   * FIXED: Protect against duplicate points in the input or points that snap to the same location resulting in `nan` times for the legs of the map match (of a 0 distance route) [#2229](https://github.com/valhalla/valhalla/pull/2229)
   * FIXED: Improves restriction check on briding edge in Bidirectional Astar [#2228](https://github.com/valhalla/valhalla/pull/2242)
   * FIXED: Allow nodes at location 0,0 [#2245](https://github.com/valhalla/valhalla/pull/2245)
   * FIXED: Fix RapidJSON compiler warnings and naming conflict [#2249](https://github.com/valhalla/valhalla/pull/2249)
   * FIXED: Fixed bug in resample_spherical_polyline where duplicate successive lat,lng locations in the polyline resulting in `nan` for the distance computation which shortcuts further sampling [#2239](https://github.com/valhalla/valhalla/pull/2239)
   * FIXED: Update exit logic for non-motorways [#2252](https://github.com/valhalla/valhalla/pull/2252)
   * FIXED: Transition point map-matching. When match results are on a transition point, we search for the sibling nodes at that transition and snap it to the corresponding edges in the route. [#2258](https://github.com/valhalla/valhalla/pull/2258)
   * FIXED: Fixed verbal multi-cue logic [#2270](https://github.com/valhalla/valhalla/pull/2270)
   * FIXED: Fixed Uturn cases when a not_thru edge is connected to the origin edge. [#2272](https://github.com/valhalla/valhalla/pull/2272)
   * FIXED: Update intersection classes in osrm response to not label all ramps as motorway [#2279](https://github.com/valhalla/valhalla/pull/2279)
   * FIXED: Fixed bug in mapmatcher when interpolation point goes before the first valid match or after the last valid match. Such behavior usually leads to discontinuity in matching. [#2275](https://github.com/valhalla/valhalla/pull/2275)
   * FIXED: Fixed an issue for time_allowed logic.  Previously we returned false on the first time allowed restriction and did not check them all. Added conditional restriction gurka test and datetime optional argument to gurka header file. [#2286](https://github.com/valhalla/valhalla/pull/2286)
   * FIXED: Fixed an issue for date ranges.  For example, for the range Jan 04 to Jan 02 we need to test to end of the year and then from the first of the year to the end date.  Also, fixed an emergency tag issue.  We should only set the use to emergency if all other access is off. [#2290](https://github.com/valhalla/valhalla/pull/2290)
   * FIXED: Found a few issues with the initial ref and direction logic for ways.  We were overwriting the refs with directionals to the name_offset_map instead of concatenating them together.  Also, we did not allow for blank entries for GetTagTokens. [#2298](https://github.com/valhalla/valhalla/pull/2298)
   * FIXED: Fixed an issue where MatchGuidanceViewJunctions is only looking at the first edge. Set the data_id for guidance views to the changeset id as it is already being populated. Also added test for guidance views. [#2303](https://github.com/valhalla/valhalla/pull/2303)
   * FIXED: Fixed a problem with live speeds where live speeds were being used to determine access, even when a live
   speed (current time) route wasn't what was requested. [#2311](https://github.com/valhalla/valhalla/pull/2311)
   * FIXED: Fix break/continue typo in search filtering [#2317](https://github.com/valhalla/valhalla/pull/2317)
   * FIXED: Fix a crash in trace_route due to iterating past the end of a vector. [#2322](https://github.com/valhalla/valhalla/pull/2322)
   * FIXED: Don't allow timezone information in the local date time string attached at each location. [#2312](https://github.com/valhalla/valhalla/pull/2312)
   * FIXED: Fix short route trimming in bidirectional astar [#2323](https://github.com/valhalla/valhalla/pull/2323)
   * FIXED: Fix shape trimming in leg building for snap candidates that lie within the margin of rounding error [#2326](https://github.com/valhalla/valhalla/pull/2326)
   * FIXED: Fixes route duration underflow with traffic data [#2325](https://github.com/valhalla/valhalla/pull/2325)
   * FIXED: Parse mtb:scale tags and set bicycle access if present [#2117](https://github.com/valhalla/valhalla/pull/2117)
   * FIXED: Fixed segfault.  Shape was missing from options for valhalla_path_comparison and valhalla_run_route.  Also, costing options was missing in valhalla_path_comparison. [#2343](https://github.com/valhalla/valhalla/pull/2343)
   * FIXED: Handle decimal numbers with zero-value mantissa properly in Lua [#2355](https://github.com/valhalla/valhalla/pull/2355)
   * FIXED: Many issues that resulted in discontinuities, failed matches or incorrect time/duration for map matching requests. [#2292](https://github.com/valhalla/valhalla/pull/2292)
   * FIXED: Seeing segfault when loading large osmdata data files before loading LuaJit. LuaJit fails to create luaL_newstate() Ref: [#2158](https://github.com/ntop/ntopng/issues/2158) Resolution is to load LuaJit before loading the data files. [#2383](https://github.com/valhalla/valhalla/pull/2383)
   * FIXED: Store positive/negative OpenLR offsets in bucketed form [#2405](https://github.com/valhalla/valhalla/2405)
   * FIXED: Fix on map-matching return code when breakage distance limitation exceeds. Instead of letting the request goes into meili and fails in finding a route, we check the distance in loki and early return with exception code 172. [#2406](https://github.com/valhalla/valhalla/pull/2406)
   * FIXED: Don't create edges for portions of ways that are doubled back on themselves as this confuses opposing edge index computations [#2385](https://github.com/valhalla/valhalla/pull/2385)
   * FIXED: Protect against nan in uniform_resample_spherical_polyline. [#2431](https://github.com/valhalla/valhalla/pull/2431)
   * FIXED: Obvious maneuvers. [#2436](https://github.com/valhalla/valhalla/pull/2436)
   * FIXED: Base64 encoding/decoding [#2452](https://github.com/valhalla/valhalla/pull/2452)
   * FIXED: Added post roundabout instruction when enter/exit roundabout maneuvers are combined [#2454](https://github.com/valhalla/valhalla/pull/2454)
   * FIXED: openlr: Explicitly check for linear reference option for Valhalla serialization. [#2458](https://github.com/valhalla/valhalla/pull/2458)
   * FIXED: Fix segfault: Do not combine last turn channel maneuver. [#2463](https://github.com/valhalla/valhalla/pull/2463)
   * FIXED: Remove extraneous whitespaces from ja-JP.json. [#2471](https://github.com/valhalla/valhalla/pull/2471)
   * FIXED: Checks protobuf serialization/parsing success [#2477](https://github.com/valhalla/valhalla/pull/2477)
   * FIXED: Fix dereferencing of end for std::lower_bound in sequence and possible UB [#2488](https://github.com/valhalla/valhalla/pull/2488)
   * FIXED: Make tile building reproducible: fix UB-s [#2480](https://github.com/valhalla/valhalla/pull/2480)
   * FIXED: Zero initialize EdgeInfoInner.spare0_. Uninitialized spare0_ field produced UB which causes gurka_reproduce_tile_build to fail intermittently. [2499](https://github.com/valhalla/valhalla/pull/2499)
   * FIXED: Drop unused CHANGELOG validation script, straggling NodeJS references [#2506](https://github.com/valhalla/valhalla/pull/2506)
   * FIXED: Fix missing nullptr checks in graphreader and loki::Reach (causing segfault during routing with not all levels of tiles availble) [#2504](https://github.com/valhalla/valhalla/pull/2504)
   * FIXED: Fix mismatch of triplegedge roadclass and directededge roadclass [#2507](https://github.com/valhalla/valhalla/pull/2507)
   * FIXED: Improve german destination_verbal_alert phrases [#2509](https://github.com/valhalla/valhalla/pull/2509)
   * FIXED: Undefined behavior cases discovered with undefined behavior sanitizer tool. [2498](https://github.com/valhalla/valhalla/pull/2498)
   * FIXED: Fixed logic so verbal keep instructions use branch exit sign info for ramps [#2520](https://github.com/valhalla/valhalla/pull/2520)
   * FIXED: Fix bug in trace_route for uturns causing garbage coordinates [#2517](https://github.com/valhalla/valhalla/pull/2517)
   * FIXED: Simplify heading calculation for turn type. Remove undefined behavior case. [#2513](https://github.com/valhalla/valhalla/pull/2513)
   * FIXED: Always set costing name even if one is not provided for osrm serializer weight_name. [#2528](https://github.com/valhalla/valhalla/pull/2528)
   * FIXED: Make single-thread tile building reproducible: fix seed for shuffle, use concurrency configuration from the mjolnir section. [#2515](https://github.com/valhalla/valhalla/pull/2515)
   * FIXED: More Windows compatibility: build tiles and some run actions work now (including CI tests) [#2300](https://github.com/valhalla/valhalla/issues/2300)
   * FIXED: Transcoding of c++ location to pbf location used path edges in the place of filtered edges. [#2542](https://github.com/valhalla/valhalla/pull/2542)
   * FIXED: Add back whitelisting action types. [#2545](https://github.com/valhalla/valhalla/pull/2545)
   * FIXED: Allow uturns for truck costing now that we have derived deadends marked in the edge label [#2559](https://github.com/valhalla/valhalla/pull/2559)
   * FIXED: Map matching uturn trimming at the end of an edge where it wasn't needed. [#2558](https://github.com/valhalla/valhalla/pull/2558)
   * FIXED: Multicue enter roundabout [#2556](https://github.com/valhalla/valhalla/pull/2556)
   * FIXED: Changed reachability computation to take into account live speed [#2597](https://github.com/valhalla/valhalla/pull/2597)
   * FIXED: Fixed a bug where the temp files were not getting read in if you started with the construct edges or build phase for valhalla_build_tiles. [#2601](https://github.com/valhalla/valhalla/pull/2601)
   * FIXED: Updated fr-FR.json with partial translations. [#2605](https://github.com/valhalla/valhalla/pull/2605)
   * FIXED: Removed superfluous const qualifier from odin/signs [#2609](https://github.com/valhalla/valhalla/pull/2609)
   * FIXED: Internal maneuver placement [#2600](https://github.com/valhalla/valhalla/pull/2600)
   * FIXED: Complete fr-FR.json locale. [#2614](https://github.com/valhalla/valhalla/pull/2614)
   * FIXED: Don't truncate precision in polyline encoding [#2632](https://github.com/valhalla/valhalla/pull/2632)
   * FIXED: Fix all compiler warnings in sif and set to -Werror [#2642](https://github.com/valhalla/valhalla/pull/2642)
   * FIXED: Remove unnecessary maneuvers to continue straight [#2647](https://github.com/valhalla/valhalla/pull/2647)
   * FIXED: Linear reference support in route/mapmatch apis (FOW, FRC, bearing, and number of references) [#2645](https://github.com/valhalla/valhalla/pull/2645)
   * FIXED: Ambiguous local to global (with timezone information) date time conversions now all choose to use the later time instead of throwing unhandled exceptions [#2665](https://github.com/valhalla/valhalla/pull/2665)
   * FIXED: Overestimated reach caused be reenquing transition nodes without checking that they had been already expanded [#2670](https://github.com/valhalla/valhalla/pull/2670)
   * FIXED: Build with C++17 standard. Deprecated function calls are substituted with new ones. [#2669](https://github.com/valhalla/valhalla/pull/2669)
   * FIXED: Improve German post_transition_verbal instruction [#2677](https://github.com/valhalla/valhalla/pull/2677)
   * FIXED: Lane updates.  Add the turn lanes to all edges of the way.  Do not "enhance" turn lanes if they are part of a complex restriction.  Moved ProcessTurnLanes after UpdateManeuverPlacementForInternalIntersectionTurns.  Fix for a missing "uturn" indication for intersections on the previous maneuver, we were serializing an empty list. [#2679](https://github.com/valhalla/valhalla/pull/2679)
   * FIXED: Fixes OpenLr serialization [#2688](https://github.com/valhalla/valhalla/pull/2688)
   * FIXED: Internal edges can't be also a ramp or a turn channel.  Also, if an edge is marked as ramp and turn channel mark it as a ramp.  [2689](https://github.com/valhalla/valhalla/pull/2689)
   * FIXED: Check that speeds are equal for the edges going in the same direction while buildig shortcuts [#2691](https://github.com/valhalla/valhalla/pull/2691)
   * FIXED: Missing fork or bear instruction [#2683](https://github.com/valhalla/valhalla/pull/2683)
   * FIXED: Eliminate null pointer dereference in GraphReader::AreEdgesConnected [#2695](https://github.com/valhalla/valhalla/issues/2695)
   * FIXED: Fix polyline simplification float/double comparison [#2698](https://github.com/valhalla/valhalla/issues/2698)
   * FIXED: Weights were sometimes negative due to incorrect updates to elapsed_cost [#2702](https://github.com/valhalla/valhalla/pull/2702)
   * FIXED: Fix bidirectional route failures at deadends [#2705](https://github.com/valhalla/valhalla/pull/2705)
   * FIXED: Updated logic to call out a non-obvious turn [#2708](https://github.com/valhalla/valhalla/pull/2708)
   * FIXED: valhalla_build_statistics multithreaded mode fixed [#2707](https://github.com/valhalla/valhalla/pull/2707)
   * FIXED: If infer_internal_intersections is true then allow internals that are also ramps or TCs. Without this we produce an extra continue manuever.  [#2710](https://github.com/valhalla/valhalla/pull/2710)
   * FIXED: We were routing down roads that should be destination only. Now we mark roads with motor_vehicle=destination and motor_vehicle=customers or access=destination and access=customers as destination only. [#2722](https://github.com/valhalla/valhalla/pull/2722)
   * FIXED: Replace all Python2 print statements with Python3 syntax [#2716](https://github.com/valhalla/valhalla/issues/2716)
   * FIXED: Some HGT files not found [#2723](https://github.com/valhalla/valhalla/issues/2723)
   * FIXED: Fix PencilPointUturn detection by removing short-edge check and updating angle threshold [#2725](https://github.com/valhalla/valhalla/issues/2725)
   * FIXED: Fix invalid continue/bear maneuvers [#2729](https://github.com/valhalla/valhalla/issues/2729)
   * FIXED: Fixes an issue that lead to double turns within a very short distance, when instead, it should be a u-turn. We now collapse double L turns or double R turns in short non-internal intersections to u-turns. [#2740](https://github.com/valhalla/valhalla/pull/2740)
   * FIXED: fixes an issue that lead to adding an extra maneuver. We now combine a current maneuver short length non-internal edges (left or right) with the next maneuver that is a kRampStraight. [#2741](https://github.com/valhalla/valhalla/pull/2741)
   * FIXED: Reduce verbose instructions by collapsing small end ramp forks [#2762](https://github.com/valhalla/valhalla/issues/2762)
   * FIXED: Remove redundant return statements [#2776](https://github.com/valhalla/valhalla/pull/2776)
   * FIXED: Added unit test for BuildAdminFromPBF() to test GEOS 3.9 update. [#2787](https://github.com/valhalla/valhalla/pull/2787)
   * FIXED: Add support for geos-3.9 c++ api [#2739](https://github.com/valhalla/valhalla/issues/2739)
   * FIXED: Fix check for live speed validness [#2797](https://github.com/valhalla/valhalla/pull/2797)

* **Enhancement**
   * ADDED: Matrix of Bike Share [#2590](https://github.com/valhalla/valhalla/pull/2590)
   * ADDED: Add ability to provide custom implementation for candidate collection in CandidateQuery. [#2328](https://github.com/valhalla/valhalla/pull/2328)
   * ADDED: Cancellation of tile downloading. [#2319](https://github.com/valhalla/valhalla/pull/2319)
   * ADDED: Return the coordinates of the nodes isochrone input locations snapped to [#2111](https://github.com/valhalla/valhalla/pull/2111)
   * ADDED: Allows more complicated routes in timedependent a-star before timing out [#2068](https://github.com/valhalla/valhalla/pull/2068)
   * ADDED: Guide signs and junction names [#2096](https://github.com/valhalla/valhalla/pull/2096)
   * ADDED: Added a bool to the config indicating whether to use commercially set attributes.  Added logic to not call IsIntersectionInternal if this is a commercial data set.  [#2132](https://github.com/valhalla/valhalla/pull/2132)
   * ADDED: Removed commerical data set bool to the config and added more knobs for data.  Added infer_internal_intersections, infer_turn_channels, apply_country_overrides, and use_admin_db.  [#2173](https://github.com/valhalla/valhalla/pull/2173)
   * ADDED: Allow using googletest in unit tests and convert all tests to it (old test.cc is completely removed). [#2128](https://github.com/valhalla/valhalla/pull/2128)
   * ADDED: Add guidance view capability. [#2209](https://github.com/valhalla/valhalla/pull/2209)
   * ADDED: Collect turn cost information as path is formed so that it can be seralized out for trace attributes or osrm flavored intersections. Also add shape_index to osrm intersections. [#2207](https://github.com/valhalla/valhalla/pull/2207)
   * ADDED: Added alley factor to autocost.  Factor is defaulted at 1.0f or do not avoid alleys. [#2246](https://github.com/valhalla/valhalla/pull/2246)
   * ADDED: Support unlimited speed limits where maxspeed=none. [#2251](https://github.com/valhalla/valhalla/pull/2251)
   * ADDED: Implement improved Reachability check using base class Dijkstra. [#2243](https://github.com/valhalla/valhalla/pull/2243)
   * ADDED: Gurka integration test framework with ascii-art maps [#2244](https://github.com/valhalla/valhalla/pull/2244)
   * ADDED: Add to the stop impact when transitioning from higher to lower class road and we are not on a turn channel or ramp. Also, penalize lefts when driving on the right and vice versa. [#2282](https://github.com/valhalla/valhalla/pull/2282)
   * ADDED: Added reclassify_links, use_direction_on_ways, and allow_alt_name as config options.  If `use_direction_on_ways = true` then use `direction` and `int_direction` on the way to update the directional for the `ref` and `int_ref`.  Also, copy int_efs to the refs. [#2285](https://github.com/valhalla/valhalla/pull/2285)
   * ADDED: Add support for live traffic. [#2268](https://github.com/valhalla/valhalla/pull/2268)
   * ADDED: Implement per-location search filters for functional road class and forms of way. [#2289](https://github.com/valhalla/valhalla/pull/2289)
   * ADDED: Approach, multi-cue, and length updates [#2313](https://github.com/valhalla/valhalla/pull/2313)
   * ADDED: Speed up timezone differencing calculation if cache is provided. [#2316](https://github.com/valhalla/valhalla/pull/2316)
   * ADDED: Added rapidjson/schema.h to baldr/rapidjson_util.h to make it available for use within valhalla. [#2330](https://github.com/valhalla/valhalla/issues/2330)
   * ADDED: Support decimal precision for height values in elevation service. Also support polyline5 for encoded polylines input and output to elevation service. [#2324](https://github.com/valhalla/valhalla/pull/2324)
   * ADDED: Use both imminent and distant verbal multi-cue phrases. [#2353](https://github.com/valhalla/valhalla/pull/2353)
   * ADDED: Split parsing stage into 3 separate stages. [#2339](https://github.com/valhalla/valhalla/pull/2339)
   * CHANGED: Speed up graph enhancing by avoiding continuous unordered_set rebuilding [#2349](https://github.com/valhalla/valhalla/pull/2349)
   * CHANGED: Skip calling out to Lua for nodes/ways/relations with not tags - speeds up parsing. [#2351](https://github.com/valhalla/valhalla/pull/2351)
   * CHANGED: Switch to LuaJIT for lua scripting - speeds up file parsing [#2352](https://github.com/valhalla/valhalla/pull/2352)
   * ADDED: Ability to create OpenLR records from raw data. [#2356](https://github.com/valhalla/valhalla/pull/2356)
   * ADDED: Revamp length phrases [#2359](https://github.com/valhalla/valhalla/pull/2359)
   * CHANGED: Do not allocate memory in skadi if we don't need it. [#2373](https://github.com/valhalla/valhalla/pull/2373)
   * CHANGED: Map matching: throw error (443/NoSegment) when no candidate edges are available. [#2370](https://github.com/valhalla/valhalla/pull/2370/)
   * ADDED: Add sk-SK.json (slovak) localization file. [#2376](https://github.com/valhalla/valhalla/pull/2376)
   * ADDED: Extend roundabout phrases. [#2378](https://github.com/valhalla/valhalla/pull/2378)
   * ADDED: More roundabout phrase tests. [#2382](https://github.com/valhalla/valhalla/pull/2382)
   * ADDED: Update the turn and continue phrases to include junction names and guide signs. [#2386](https://github.com/valhalla/valhalla/pull/2386)
   * ADDED: Add the remaining guide sign toward phrases [#2389](https://github.com/valhalla/valhalla/pull/2389)
   * ADDED: The ability to allow immediate uturns at trace points in a map matching request [#2380](https://github.com/valhalla/valhalla/pull/2380)
   * ADDED: Add utility functions to Signs. [#2390](https://github.com/valhalla/valhalla/pull/2390)
   * ADDED: Unified time tracking for all algorithms that support time-based graph expansion. [#2278](https://github.com/valhalla/valhalla/pull/2278)
   * ADDED: Add rail_ferry use and costing. [#2408](https://github.com/valhalla/valhalla/pull/2408)
   * ADDED: `street_side_max_distance`, `display_lat` and `display_lon` to `locations` in input for better control of routing side of street [#1769](https://github.com/valhalla/valhalla/pull/1769)
   * ADDED: Add addtional exit phrases. [#2421](https://github.com/valhalla/valhalla/pull/2421)
   * ADDED: Add Japanese locale, update German. [#2432](https://github.com/valhalla/valhalla/pull/2432)
   * ADDED: Gurka expect_route refactor [#2435](https://github.com/valhalla/valhalla/pull/2435)
   * ADDED: Add option to suppress roundabout exits [#2437](https://github.com/valhalla/valhalla/pull/2437)
   * ADDED: Add Greek locale. [#2438](https://github.com/valhalla/valhalla/pull/2438)
   * ADDED (back): Support for 64bit wide way ids in the edgeinfo structure with no impact to size for data sources with ids 32bits wide. [#2422](https://github.com/valhalla/valhalla/pull/2422)
   * ADDED: Support for 64bit osm node ids in parsing stage of tile building [#2422](https://github.com/valhalla/valhalla/pull/2422)
   * CHANGED: Point2/PointLL are now templated to allow for higher precision coordinate math when desired [#2429](https://github.com/valhalla/valhalla/pull/2429)
   * ADDED: Optional OpenLR Encoded Path Edges in API Response [#2424](https://github.com/valhalla/valhalla/pull/2424)
   * ADDED: Add explicit include for sstream to be compatible with msvc_x64 toolset. [#2449](https://github.com/valhalla/valhalla/pull/2449)
   * ADDED: Properly split returned path if traffic conditions change partway along edges [#2451](https://github.com/valhalla/valhalla/pull/2451/files)
   * ADDED: Add Dutch locale. [#2464](https://github.com/valhalla/valhalla/pull/2464)
   * ADDED: Check with address sanititizer in CI. Add support for undefined behavior sanitizer. [#2487](https://github.com/valhalla/valhalla/pull/2487)
   * ADDED: Ability to recost a path and increased cost/time details along the trippath and json output [#2425](https://github.com/valhalla/valhalla/pull/2425)
   * ADDED: Add the ability to do bikeshare based (ped/bike) multimodal routing [#2031](https://github.com/valhalla/valhalla/pull/2031)
   * ADDED: Route through restrictions enabled by introducing a costing option. [#2469](https://github.com/valhalla/valhalla/pull/2469)
   * ADDED: Migrated to Ubuntu 20.04 base-image [#2508](https://github.com/valhalla/valhalla/pull/2508)
   * CHANGED: Speed up parseways stage by avoiding multiple string comparisons [#2518](https://github.com/valhalla/valhalla/pull/2518)
   * CHANGED: Speed up enhance stage by avoiding GraphTileBuilder copying [#2468](https://github.com/valhalla/valhalla/pull/2468)
   * ADDED: Costing options now includes shortest flag which favors shortest path routes [#2555](https://github.com/valhalla/valhalla/pull/2555)
   * ADDED: Incidents in intersections [#2547](https://github.com/valhalla/valhalla/pull/2547)
   * CHANGED: Refactor mapmatching configuration to use a struct (instead of `boost::property_tree::ptree`). [#2485](https://github.com/valhalla/valhalla/pull/2485)
   * ADDED: Save exit maneuver's begin heading when combining enter & exit roundabout maneuvers. [#2554](https://github.com/valhalla/valhalla/pull/2554)
   * ADDED: Added new urban flag that can be set if edge is within city boundaries to data processing; new use_urban_tag config option; added to osrm response within intersections. [#2522](https://github.com/valhalla/valhalla/pull/2522)
   * ADDED: Parses OpenLr of type PointAlongLine [#2565](https://github.com/valhalla/valhalla/pull/2565)
   * ADDED: Use edge.is_urban is set for serializing is_urban. [#2568](https://github.com/valhalla/valhalla/pull/2568)
   * ADDED: Added new rest/service area uses on the edge. [#2533](https://github.com/valhalla/valhalla/pull/2533)
   * ADDED: Dependency cache for Azure [#2567](https://github.com/valhalla/valhalla/pull/2567)
   * ADDED: Added flexibility to remove the use of the admindb and to use the country and state iso from the tiles; [#2579](https://github.com/valhalla/valhalla/pull/2579)
   * ADDED: Added toll gates and collection points (gantry) to the node;  [#2532](https://github.com/valhalla/valhalla/pull/2532)
   * ADDED: Added osrm serialization for rest/service areas and admins. [#2594](https://github.com/valhalla/valhalla/pull/2594)
   * CHANGED: Improved Russian localization; [#2593](https://github.com/valhalla/valhalla/pull/2593)
   * ADDED: Support restricted class in intersection annotations [#2589](https://github.com/valhalla/valhalla/pull/2589)
   * ADDED: Added trail type trace [#2606](https://github.com/valhalla/valhalla/pull/2606)
   * ADDED: Added tunnel names to the edges as a tagged name.  [#2608](https://github.com/valhalla/valhalla/pull/2608)
   * CHANGED: Moved incidents to the trip leg and cut the shape of the leg at that location [#2610](https://github.com/valhalla/valhalla/pull/2610)
   * ADDED: Costing option to ignore_closures when routing with current flow [#2615](https://github.com/valhalla/valhalla/pull/2615)
   * ADDED: Cross-compilation ability with MinGW64 [#2619](https://github.com/valhalla/valhalla/pull/2619)
   * ADDED: Defines the incident tile schema and incident metadata [#2620](https://github.com/valhalla/valhalla/pull/2620)
   * ADDED: Moves incident serializer logic into a generic serializer [#2621](https://github.com/valhalla/valhalla/pull/2621)
   * ADDED: Incident loading singleton for continually refreshing incident tiles[#2573](https://github.com/valhalla/valhalla/pull/2573)
   * ADDED: One shot mode to valhalla_service so you can run a single request of any type without starting a server [#2624](https://github.com/valhalla/valhalla/pull/2624)
   * ADDED: Adds text instructions to OSRM output [#2625](https://github.com/valhalla/valhalla/pull/2625)
   * ADDED: Adds support for alternate routes [#2626](https://github.com/valhalla/valhalla/pull/2626)
   * CHANGED: Switch Python bindings generator from boost.python to header-only pybind11[#2644](https://github.com/valhalla/valhalla/pull/2644)
   * ADDED: Add support of input file for one-shot mode of valhalla_service [#2648](https://github.com/valhalla/valhalla/pull/2648)
   * ADDED: Linear reference support to locate api [#2645](https://github.com/valhalla/valhalla/pull/2645)
   * ADDED: Implemented OSRM-like turn duration calculation for car. Uses it now in auto costing. [#2651](https://github.com/valhalla/valhalla/pull/2651)
   * ADDED: Enhanced turn lane information in guidance [#2653](https://github.com/valhalla/valhalla/pull/2653)
   * ADDED: `top_speed` option for all motorized vehicles [#2667](https://github.com/valhalla/valhalla/issues/2667)
   * CHANGED: Move turn_lane_direction helper to odin/util [#2675](https://github.com/valhalla/valhalla/pull/2675)
   * ADDED: Add annotations to osrm response including speed limits, unit and sign conventions [#2668](https://github.com/valhalla/valhalla/pull/2668)
   * ADDED: Added functions for predicted speeds encoding-decoding [#2674](https://github.com/valhalla/valhalla/pull/2674)
   * ADDED: Time invariant routing via the bidirectional algorithm. This has the effect that when time dependent routes (arrive_by and depart_at) fall back to bidirectional due to length restrictions they will actually use the correct time of day for one of the search directions [#2660](https://github.com/valhalla/valhalla/pull/2660)
   * ADDED: If the length of the edge is greater than kMaxEdgeLength, then consider this a catastrophic error if the should_error bool is true in the set_length function. [2678](https://github.com/valhalla/valhalla/pull/2678)
   * ADDED: Moved lat,lon coordinates structures from single to double precision. Improves geometry accuracy noticibly at zooms above 17 as well as coordinate snapping and any other geometric operations. Addes about a 2% performance pentalty for standard routes. Graph nodes now have 7 digits of precision.  [#2693](https://github.com/valhalla/valhalla/pull/2693)
   * ADDED: Added signboards to guidance views.  [#2687](https://github.com/valhalla/valhalla/pull/2687)
   * ADDED: Regular speed on shortcut edges is calculated with turn durations taken into account. Truck, motorcycle and motorscooter profiles use OSRM-like turn duration. [#2662](https://github.com/valhalla/valhalla/pull/2662)
   * CHANGED: Remove astar algorithm and replace its use with timedep_forward as its redundant [#2706](https://github.com/valhalla/valhalla/pull/2706)
   * ADDED: Recover and recost all shortcuts in final path for bidirectional astar algorithm [#2711](https://github.com/valhalla/valhalla/pull/2711)
   * ADDED: An option for shortcut recovery to be cached at start up to reduce the time it takes to do so on the fly [#2714](https://github.com/valhalla/valhalla/pull/2714)
   * ADDED: If width <= 1.9 then no access for auto, truck, bus, taxi, emergency and hov. [#2713](https://github.com/valhalla/valhalla/pull/2713)
   * ADDED: Centroid/Converge/Rendezvous/Meet API which allows input locations to find a least cost convergence point from all locations [#2734](https://github.com/valhalla/valhalla/pull/2734)
   * ADDED: Added support to process the sump_buster tag.  Also, fixed a few small access bugs for nodes. [#2731](https://github.com/valhalla/valhalla/pull/2731)
   * ADDED: Log message if failed to create tiles directory. [#2738](https://github.com/valhalla/valhalla/pull/2738)
   * CHANGED: Tile memory is only owned by the GraphTile rather than shared amongst copies of the graph tile (in GraphReader and TileCaches). [#2340](https://github.com/valhalla/valhalla/pull/2340)
   * ADDED: Add Estonian locale. [#2748](https://github.com/valhalla/valhalla/pull/2748)
   * CHANGED: Handle GraphTile objects as smart pointers [#2703](https://github.com/valhalla/valhalla/pull/2703)
   * CHANGED: Improve stability with no RTTI build [#2759](https://github.com/valhalla/valhalla/pull/2759) and [#2760](https://github.com/valhalla/valhalla/pull/2760)
   * CHANGED: Change generic service roads to a new Use=kServiceRoad. This is for highway=service without other service= tags (such as driveway, alley, parking aisle) [#2419](https://github.com/valhalla/valhalla/pull/2419)
   * ADDED: Isochrones support isodistance lines as well [#2699](https://github.com/valhalla/valhalla/pull/2699)
   * ADDED: Add support for ignoring live traffic closures for waypoints [#2685](https://github.com/valhalla/valhalla/pull/2685)
   * ADDED: Add use_distance to auto cost to allow choosing between two primary cost components, time or distance [#2771](https://github.com/valhalla/valhalla/pull/2771)
   * CHANGED: nit: Enables compiler warnings in part of loki module [#2767](https://github.com/valhalla/valhalla/pull/2767)
   * CHANGED: Reducing the number of uturns by increasing the cost to for them to 9.5f. Note: Did not increase the cost for motorcycles or motorscooters. [#2770](https://github.com/valhalla/valhalla/pull/2770)
   * ADDED: Add option to use thread-safe GraphTile's reference counter. [#2772](https://github.com/valhalla/valhalla/pull/2772)
   * CHANGED: nit: Enables compiler warnings in part of thor module [#2768](https://github.com/valhalla/valhalla/pull/2768)
   * ADDED: Add costing option `use_tracks` to avoid or favor tracks in route. [#2769](https://github.com/valhalla/valhalla/pull/2769)
   * CHANGED: chore: Updates libosmium [#2786](https://github.com/valhalla/valhalla/pull/2786)
   * CHANGED: Optimize double bucket queue to reduce memory reallocations. [#2719](https://github.com/valhalla/valhalla/pull/2719)
   * CHANGED: Collapse merge maneuvers [#2773](https://github.com/valhalla/valhalla/pull/2773)
   * CHANGED: Add shortcuts to the tiles' bins so we can find them when doing spatial lookups. [#2744](https://github.com/valhalla/valhalla/pull/2744)

## Release Date: 2019-11-21 Valhalla 3.0.9
* **Bug Fix**
   * FIXED: Changed reachability computation to consider both directions of travel wrt candidate edges [#1965](https://github.com/valhalla/valhalla/pull/1965)
   * FIXED: toss ways where access=private and highway=service and service != driveway. [#1960](https://github.com/valhalla/valhalla/pull/1960)
   * FIXED: Fix search_cutoff check in loki correlate_node. [#2023](https://github.com/valhalla/valhalla/pull/2023)
   * FIXED: Computes notion of a deadend at runtime in bidirectional a-star which fixes no-route with a complicated u-turn. [#1982](https://github.com/valhalla/valhalla/issues/1982)
   * FIXED: Fix a bug with heading filter at nodes. [#2058](https://github.com/valhalla/valhalla/pull/2058)
   * FIXED: Bug in map matching continuity checking such that continuity must only be in the forward direction. [#2029](https://github.com/valhalla/valhalla/pull/2029)
   * FIXED: Allow setting the time for map matching paths such that the time is used for speed lookup. [#2030](https://github.com/valhalla/valhalla/pull/2030)
   * FIXED: Don't use density factor for transition cost when user specified flag disables flow speeds. [#2048](https://github.com/valhalla/valhalla/pull/2048)
   * FIXED: Map matching trace_route output now allows for discontinuities in the match though multi match is not supported in valhalla route output. [#2049](https://github.com/valhalla/valhalla/pull/2049)
   * FIXED: Allows routes with no time specified to use time conditional edges and restrictions with a flag denoting as much [#2055](https://github.com/valhalla/valhalla/pull/2055)
   * FIXED: Fixed a bug with 'current' time type map matches. [#2060](https://github.com/valhalla/valhalla/pull/2060)
   * FIXED: Fixed a bug with time dependent expansion in which the expansion distance heuristic was not being used. [#2064](https://github.com/valhalla/valhalla/pull/2064)

* **Enhancement**
   * ADDED: Establish pinpoint test pattern [#1969](https://github.com/valhalla/valhalla/pull/1969)
   * ADDED: Suppress relative direction in ramp/exit instructions if it matches driving side of street [#1990](https://github.com/valhalla/valhalla/pull/1990)
   * ADDED: Added relative direction to the merge maneuver [#1989](https://github.com/valhalla/valhalla/pull/1989)
   * ADDED: Refactor costing to better handle multiple speed datasources [#2026](https://github.com/valhalla/valhalla/pull/2026)
   * ADDED: Better usability of curl for fetching tiles on the fly [#2026](https://github.com/valhalla/valhalla/pull/2026)
   * ADDED: LRU cache scheme for tile storage [#2026](https://github.com/valhalla/valhalla/pull/2026)
   * ADDED: GraphTile size check [#2026](https://github.com/valhalla/valhalla/pull/2026)
   * ADDED: Pick more sane values for highway and toll avoidance [#2026](https://github.com/valhalla/valhalla/pull/2026)
   * ADDED: Refactor adding predicted speed info to speed up process [#2026](https://github.com/valhalla/valhalla/pull/2026)
   * ADDED: Allow selecting speed data sources at request time [#2026](https://github.com/valhalla/valhalla/pull/2026)
   * ADDED: Allow disabling certain neighbors in connectivity map [#2026](https://github.com/valhalla/valhalla/pull/2026)
   * ADDED: Allows routes with time-restricted edges if no time specified and notes restriction in response [#1992](https://github.com/valhalla/valhalla/issues/1992)
   * ADDED: Runtime deadend detection to timedependent a-star. [#2059](https://github.com/valhalla/valhalla/pull/2059)

## Release Date: 2019-09-06 Valhalla 3.0.8
* **Bug Fix**
   * FIXED: Added logic to detect if user is to merge to the left or right [#1892](https://github.com/valhalla/valhalla/pull/1892)
   * FIXED: Overriding the destination_only flag when reclassifying ferries; Also penalizing ferries with a 5 min. penalty in the cost to allow us to avoid destination_only the majority of the time except when it is necessary. [#1895](https://github.com/valhalla/valhalla/pull/1905)
   * FIXED: Suppress forks at motorway junctions and intersecting service roads [#1909](https://github.com/valhalla/valhalla/pull/1909)
   * FIXED: Enhanced fork assignment logic [#1912](https://github.com/valhalla/valhalla/pull/1912)
   * FIXED: Added logic to fall back to return country poly if no state and updated lua for Metro Manila and Ireland [#1910](https://github.com/valhalla/valhalla/pull/1910)
   * FIXED: Added missing motorway fork instruction [#1914](https://github.com/valhalla/valhalla/pull/1914)
   * FIXED: Use begin street name for osrm compat mode [#1916](https://github.com/valhalla/valhalla/pull/1916)
   * FIXED: Added logic to fix missing highway cardinal directions in the US [#1917](https://github.com/valhalla/valhalla/pull/1917)
   * FIXED: Handle forward traversable significant road class intersecting edges [#1928](https://github.com/valhalla/valhalla/pull/1928)
   * FIXED: Fixed bug with shape trimming that impacted Uturns at Via locations. [#1935](https://github.com/valhalla/valhalla/pull/1935)
   * FIXED: Dive bomb updates.  Updated default speeds for urban areas based on roadclass for the enhancer.  Also, updated default speeds based on roadclass in lua.  Fixed an issue where we were subtracting 1 from uint32_t when 0 for stop impact.  Updated reclassify link logic to allow residential roads to be added to the tree, but we only downgrade the links to tertiary.  Updated TransitionCost functions to add 1.5 to the turncost when transitioning from a ramp to a non ramp and vice versa.  Also, added 0.5f to the turncost if the edge is a roundabout. [#1931](https://github.com/valhalla/valhalla/pull/1931)

* **Enhancement**
   * ADDED: Caching url fetched tiles to disk [#1887](https://github.com/valhalla/valhalla/pull/1887)
   * ADDED: filesystem::remove_all [#1887](https://github.com/valhalla/valhalla/pull/1887)
   * ADDED: Minimum enclosing bounding box tool [#1887](https://github.com/valhalla/valhalla/pull/1887)
   * ADDED: Use constrained flow speeds in bidirectional_astar.cc [#1907](https://github.com/valhalla/valhalla/pull/1907)
   * ADDED: Bike Share Stations are now in the graph which should set us up to do multimodal walk/bike scenarios [#1852](https://github.com/valhalla/valhalla/pull/1852)

## Release Date: 2019-7-18 Valhalla 3.0.7
* **Bug Fix**
   * FIXED: Fix pedestrian fork [#1886](https://github.com/valhalla/valhalla/pull/1886)

## Release Date: 2019-7-15 Valhalla 3.0.6
* **Bug Fix**
   * FIXED: Admin name changes. [#1853](https://github.com/valhalla/valhalla/pull/1853) Ref: [#1854](https://github.com/valhalla/valhalla/issues/1854)
   * FIXED: valhalla_add_predicted_traffic was overcommitted while gathering stats. Added a clear. [#1857](https://github.com/valhalla/valhalla/pull/1857)
   * FIXED: regression in map matching when moving to valhalla v3.0.0 [#1863](https://github.com/valhalla/valhalla/pull/1863)
   * FIXED: last step shape in osrm serializer should be 2 of the same point [#1867](https://github.com/valhalla/valhalla/pull/1867)
   * FIXED: Shape trimming at the beginning and ending of the route to not be degenerate [#1876](https://github.com/valhalla/valhalla/pull/1876)
   * FIXED: Duplicate waypoints in osrm serializer [#1880](https://github.com/valhalla/valhalla/pull/1880)
   * FIXED: Updates for heading precision [#1881](https://github.com/valhalla/valhalla/pull/1881)
   * FIXED: Map matching allowed untraversable edges at start of route [#1884](https://github.com/valhalla/valhalla/pull/1884)

* **Enhancement**
   * ADDED: Use the same protobuf object the entire way through the request process [#1837](https://github.com/valhalla/valhalla/pull/1837)
   * ADDED: Enhanced turn lane processing [#1859](https://github.com/valhalla/valhalla/pull/1859)
   * ADDED: Add global_synchronized_cache in valhalla_build_config [#1851](https://github.com/valhalla/valhalla/pull/1851)

## Release Date: 2019-06-04 Valhalla 3.0.5
* **Bug Fix**
   * FIXED: Protect against unnamed rotaries and routes that end in roundabouts not turning off rotary logic [#1840](https://github.com/valhalla/valhalla/pull/1840)

* **Enhancement**
   * ADDED: Add turn lane info at maneuver point [#1830](https://github.com/valhalla/valhalla/pull/1830)

## Release Date: 2019-05-31 Valhalla 3.0.4
* **Bug Fix**
   * FIXED: Improved logic to decide between bear vs. continue [#1798](https://github.com/valhalla/valhalla/pull/1798)
   * FIXED: Bicycle costing allows use of roads with all surface values, but with a penalty based on bicycle type. However, the edge filter totally disallows bad surfaces for some bicycle types, creating situations where reroutes fail if a rider uses a road with a poor surface. [#1800](https://github.com/valhalla/valhalla/pull/1800)
   * FIXED: Moved complex restrictions building to before validate. [#1805](https://github.com/valhalla/valhalla/pull/1805)
   * FIXED: Fix bicycle edge filter whan avoid_bad_surfaces = 1.0 [#1806](https://github.com/valhalla/valhalla/pull/1806)
   * FIXED: Replace the EnhancedTripPath class inheritance with aggregation [#1807](https://github.com/valhalla/valhalla/pull/1807)
   * FIXED: Replace the old timezone shape zip file every time valhalla_build_timezones is ran [#1817](https://github.com/valhalla/valhalla/pull/1817)
   * FIXED: Don't use island snapped edge candidates (from disconnected components or low reach edges) when we rejected other high reachability edges that were closer [#1835](https://github.com/valhalla/valhalla/pull/1835)

## Release Date: 2019-05-08 Valhalla 3.0.3
* **Bug Fix**
   * FIXED: Fixed a rare loop condition in route matcher (edge walking to match a trace).
   * FIXED: Fixed VACUUM ANALYZE syntax issue.  [#1704](https://github.com/valhalla/valhalla/pull/1704)
   * FIXED: Fixed the osrm maneuver type when a maneuver has the to_stay_on attribute set.  [#1714](https://github.com/valhalla/valhalla/pull/1714)
   * FIXED: Fixed osrm compatibility mode attributes.  [#1716](https://github.com/valhalla/valhalla/pull/1716)
   * FIXED: Fixed rotary/roundabout issues in Valhalla OSRM compatibility.  [#1727](https://github.com/valhalla/valhalla/pull/1727)
   * FIXED: Fixed the destinations assignment for exit names in OSRM compatibility mode. [#1732](https://github.com/valhalla/valhalla/pull/1732)
   * FIXED: Enhance merge maneuver type assignment. [#1735](https://github.com/valhalla/valhalla/pull/1735)
   * FIXED: Fixed fork assignments and on ramps for OSRM compatibility mode. [#1738](https://github.com/valhalla/valhalla/pull/1738)
   * FIXED: Fixed cardinal direction on reference names when forward/backward tag is present on relations. Fixes singly digitized roads with opposing directional modifiers. [#1741](https://github.com/valhalla/valhalla/pull/1741)
   * FIXED: Fixed fork assignment and narrative logic when a highway ends and splits into multiple ramps. [#1742](https://github.com/valhalla/valhalla/pull/1742)
   * FIXED: Do not use any avoid edges as origin or destination of a route, matrix, or isochrone. [#1745](https://github.com/valhalla/valhalla/pull/1745)
   * FIXED: Add leg summary and remove unused hint attribute for OSRM compatibility mode. [#1753](https://github.com/valhalla/valhalla/pull/1753)
   * FIXED: Improvements for pedestrian forks, pedestrian roundabouts, and continue maneuvers. [#1768](https://github.com/valhalla/valhalla/pull/1768)
   * FIXED: Added simplified overview for OSRM response and added use_toll logic back to truck costing. [#1765](https://github.com/valhalla/valhalla/pull/1765)
   * FIXED: temp fix for location distance bug [#1774](https://github.com/valhalla/valhalla/pull/1774)
   * FIXED: Fix pedestrian routes using walkway_factor [#1780](https://github.com/valhalla/valhalla/pull/1780)
   * FIXED: Update the begin and end heading of short edges based on use [#1783](https://github.com/valhalla/valhalla/pull/1783)
   * FIXED: GraphReader::AreEdgesConnected update.  If transition count == 0 return false and do not call transition function. [#1786](https://github.com/valhalla/valhalla/pull/1786)
   * FIXED: Only edge candidates that were used in the path are send to serializer: [1788](https://github.com/valhalla/valhalla/pull/1788)
   * FIXED: Added logic to prevent the removal of a destination maneuver when ending on an internal edge [#1792](https://github.com/valhalla/valhalla/pull/1792)
   * FIXED: Fixed instructions when starting on an internal edge [#1796](https://github.com/valhalla/valhalla/pull/1796)

* **Enhancement**
   * Add the ability to run valhalla_build_tiles in stages. Specify the begin_stage and end_stage as command line options. Also cleans up temporary files as the last stage in the pipeline.
   * Add `remove` to `filesystem` namespace. [#1752](https://github.com/valhalla/valhalla/pull/1752)
   * Add TaxiCost into auto costing options.
   * Add `preferred_side` to allow per-location filtering of edges based on the side of the road the location is on and the driving side for that locale.
   * Slightly decreased the internal side-walk factor to .90f to favor roads with attached sidewalks. This impacts roads that have added sidewalk:left, sidewalk:right or sidewalk:both OSM tags (these become attributes on each directedEdge). The user can then avoid/penalize dedicated sidewalks and walkways, when they increase the walkway_factor. Since we slightly decreased the sidewalk_factor internally and only favor sidewalks if use is tagged as sidewalk_left or sidewalk_right, we should tend to route on roads with attached sidewalks rather than separate/dedicated sidewalks, allowing for more road names to be called out since these are labeled more.
   * Add `via` and `break_through` location types [#1737](https://github.com/valhalla/valhalla/pull/1737)
   * Add `street_side_tolerance` and `search_cutoff` to input `location` [#1777](https://github.com/valhalla/valhalla/pull/1777)
   * Return the Valhalla error `Path distance exceeds the max distance limit` for OSRM responses when the route is greater than the service limits. [#1781](https://github.com/valhalla/valhalla/pull/1781)

## Release Date: 2019-01-14 Valhalla 3.0.2
* **Bug Fix**
   * FIXED: Transit update - fix dow and exception when after midnight trips are normalized [#1682](https://github.com/valhalla/valhalla/pull/1682)
   * FIXED: valhalla_convert_transit segfault - GraphTileBuilder has null GraphTileHeader [#1683](https://github.com/valhalla/valhalla/issues/1683)
   * FIXED: Fix crash for trace_route with osrm serialization. Was passing shape rather than locations to the waypoint method.
   * FIXED: Properly set driving_side based on data set in TripPath.
   * FIXED: A bad bicycle route exposed an issue with bidirectional A* when the origin and destination edges are connected. Use A* in these cases to avoid requiring a high cost threshold in BD A*.
   * FIXED: x86 and x64 data compatibility was fixed as the structures weren't aligned.
   * FIXED: x86 tests were failing due mostly to floating point issues and the aforementioned structure misalignment.
* **Enhancement**
   * Add a durations list (delta time between each pair of trace points), a begin_time and a use_timestamp flag to trace_route requests. This allows using the input trace timestamps or durations plus the begin_time to compute elapsed time at each edge in the matched path (rather than using costing methods).
   * Add support for polyline5 encoding for OSRM formatted output.
* **Note**
   * Isochrones and openlr are both noted as not working with release builds for x86 (32bit) platforms. We'll look at getting this fixed in a future release

## Release Date: 2018-11-21 Valhalla 3.0.1
* **Bug Fix**
   * FIXED: Fixed a rare, but serious bug with bicycle costing. ferry_factor_ in bicycle costing shadowed the data member in the base dynamic cost class, leading to an unitialized variable. Occasionally, this would lead to negative costs which caused failures. [#1663](https://github.com/valhalla/valhalla/pull/1663)
   * FIXED: Fixed use of units in OSRM compatibility mode. [#1662](https://github.com/valhalla/valhalla/pull/1662)

## Release Date: 2018-11-21 Valhalla 3.0.0
* **NOTE**
   * This release changes the Valhalla graph tile formats to make the tile data more efficient and flexible. Tile data is incompatible with Valhalla 2.x builds, and code for 3.x is incompatible with data built for Valahalla 2.x versions. Valhalla tile sizes are slightly smaller (for datasets using elevation information the size savings is over 10%). In addition, there is increased flexibility for creating different variants of tiles to support different applications (e.g. bicycle only, or driving only).
* **Enhancement**
   * Remove the use of DirectedEdge for transitions between nodes on different hierarchy levels. A new structure, NodeTransition, is now used to transition to nodes on different hierarchy level. This saves space since only the end node GraphId is needed for the transitions (and DirectedEdge is a large data structure).
   * Change the NodeInfo lat,lon to use an offset from the tile base lat,lon. This potentially allows higher precision than using float, but more importantly saves space and allows support for NodeTransitions as well as spare for future growth.
   * Remove the EdgeElevation structure and max grade information into DirectedEdge and mean elevation into EdgeInfo. This saves space.
   * Reduce wayid to 32 bits. This allows sufficient growth when using OpenStreetMap data and frees space in EdgeInfo (allows moving speed limit and mean elevation from other structures).
   * Move name consistency from NodeInfo to DirectedEdge. This allows a more efficient lookup of name consistency.
   * Update all path algorithms to use NodeTransition logic rather than special DirectedEdge transition types. This simplifies PathAlgorithms slightly and removes some conditional logic.
   * Add an optional GraphFilter stage to tile building pipeline. This allows removal of edges and nodes based on access. This allows bicycle only, pedestrian only, or driving only datasets (or combinations) to be created - allowing smaller datasets for special purpose applications.
* **Deprecate**
   * Valhalla 3.0 removes support for OSMLR.

## Release Date: 2018-11-20 Valhalla 2.7.2
* **Enhancement**
   * UPDATED: Added a configuration variable for max_timedep_distance. This is used in selecting the path algorithm and provides the maximum distance between locations when choosing a time dependent path algorithm (other than multi modal). Above this distance, bidirectional A* is used with no time dependencies.
   * UPDATED: Remove transition edges from priority queue in Multimodal methods.
   * UPDATED: Fully implement street names and exit signs with ability to identify route numbers. [#1635](https://github.com/valhalla/valhalla/pull/1635)
* **Bug Fix**
   * FIXED: A timed-turned restriction should not be applied when a non-timed route is executed.  [#1615](https://github.com/valhalla/valhalla/pull/1615)
   * FIXED: Changed unordered_map to unordered_multimap for polys. Poly map can contain the same key but different multi-polygons. For example, islands for a country or timezone polygons for a country.
   * FIXED: Fixed timezone db issue where TZIDs did not exist in the Howard Hinnant date time db that is used in the date_time class for tz indexes.  Added logic to create aliases for TZIDs based on https://en.wikipedia.org/wiki/List_of_tz_database_time_zones
   * FIXED: Fixed the ramp turn modifiers for osrm compat [#1569](https://github.com/valhalla/valhalla/pull/1569)
   * FIXED: Fixed the step geometry when using the osrm compat mode [#1571](https://github.com/valhalla/valhalla/pull/1571)
   * FIXED: Fixed a data creation bug causing issues with A* routes ending on loops. [#1576](https://github.com/valhalla/valhalla/pull/1576)
   * FIXED: Fixed an issue with a bad route where destination only was present. Was due to thresholds in bidirectional A*. Changed threshold to be cost based rather than number of iterations). [#1586](https://github.com/valhalla/valhalla/pull/1586)
   * FIXED: Fixed an issue with destination only (private) roads being used in bicycle routes. Centralized some "base" transition cost logic in the base DynamicCost class. [#1587](https://github.com/valhalla/valhalla/pull/1587)
   * FIXED: Remove extraneous ramp maneuvers [#1657](https://github.com/valhalla/valhalla/pull/1657)

## Release Date: 2018-10-02 Valhalla 2.7.1
* **Enhancement**
   * UPDATED: Added date time support to forward and reverse isochrones. Add speed lookup (predicted speeds and/or free-flow or constrained flow speed) if date_time is present.
   * UPDATED: Add timezone checks to multimodal routes and isochrones (updates localtime if the path crosses into a timezone different than the start location).
* **Data Producer Update**
   * UPDATED: Removed boost date time support from transit.  Now using the Howard Hinnant date library.
* **Bug Fix**
   * FIXED: Fixed a bug with shortcuts that leads to inconsistent routes depending on whether shortcuts are taken, different origins can lead to different paths near the destination. This fix also improves performance on long routes and matrices.
   * FIXED: We were getting inconsistent results between departing at current date/time vs entering the current date/time.  This issue is due to the fact that the iso_date_time function returns the full iso date_time with the timezone offset (e.g., 2018-09-27T10:23-07:00 vs 2018-09-27T10:23). When we refactored the date_time code to use the new Howard Hinnant date library, we introduced this bug.
   * FIXED: Increased the threshold in CostMatrix to address null time and distance values occuring for truck costing with locations near the max distance.

## Release Date: 2018-09-13 Valhalla 2.7.0
* **Enhancement**
   * UPDATED: Refactor to use the pbf options instead of the ptree config [#1428](https://github.com/valhalla/valhalla/pull/1428) This completes [1357](https://github.com/valhalla/valhalla/issues/1357)
   * UPDATED: Removed the boost/date_time dependency from baldr and odin. We added the Howard Hinnant date and time library as a submodule. [#1494](https://github.com/valhalla/valhalla/pull/1494)
   * UPDATED: Fixed 'Drvie' typo [#1505](https://github.com/valhalla/valhalla/pull/1505) This completes [1504](https://github.com/valhalla/valhalla/issues/1504)
   * UPDATED: Optimizations of GetSpeed for predicted speeds [1490](https://github.com/valhalla/valhalla/issues/1490)
   * UPDATED: Isotile optimizations
   * UPDATED: Added stats to predictive traffic logging
   * UPDATED: resample_polyline - Breaks the polyline into equal length segments at a sample distance near the resolution. Break out of the loop through polyline points once we reach the specified number of samplesthen append the last
polyline point.
   * UPDATED: added android logging and uses a shared graph reader
   * UPDATED: Do not run a second pass on long pedestrian routes that include a ferry (but succeed on first pass). This is a performance fix. Long pedestrian routes with A star factor based on ferry speed end up being very inefficient.
* **Bug Fix**
   * FIXED: A* destination only
   * FIXED: Fixed through locations weren't honored [#1449](https://github.com/valhalla/valhalla/pull/1449)


## Release Date: 2018-08-02 Valhalla 3.0.0-rc.4
* **Node Bindings**
   * UPDATED: add some worker pool handling
   [#1467](https://github.com/valhalla/valhalla/pull/1467)

## Release Date: 2018-08-02 Valhalla 3.0.0-rc.3
* **Node Bindings**
   * UPDATED: replaced N-API with node-addon-api wrapper and made the actor
   functions asynchronous
   [#1457](https://github.com/valhalla/valhalla/pull/1457)

## Release Date: 2018-07-24 Valhalla 3.0.0-rc.2
* **Node Bindings**
   * FIXED: turn on the autocleanup functionality for the actor object.
   [#1439](https://github.com/valhalla/valhalla/pull/1439)

## Release Date: 2018-07-16 Valhalla 3.0.0-rc.1
* **Enhancement**
   * ADDED: exposed the rest of the actions to the node bindings and added tests. [#1415](https://github.com/valhalla/valhalla/pull/1415)

## Release Date: 2018-07-12 Valhalla 3.0.0-alpha.1
**NOTE**: There was already a small package named `valhalla` on the npm registry, only published up to version 0.0.3. The team at npm has transferred the package to us, but would like us to publish something to it ASAP to prove our stake in it. Though the bindings do not have all of the actor functionality exposed yet (just route), we are going to publish an alpha release of 3.0.0 to get something up on npm.
* **Infrastructure**:
   * ADDED: add in time dependent algorithms if the distance between locations is less than 500km.
   * ADDED: TurnLanes to indicate turning lanes at the end of a directed edge.
   * ADDED: Added PredictedSpeeds to Valhalla tiles and logic to compute speed based on predictive speed profiles.
* **Data Producer Update**
   * ADDED: is_route_num flag was added to Sign records. Set this to true if the exit sign comes from a route number/ref.
   * CHANGED: Lower speeds on driveways, drive-thru, and parking aisle. Set destination only flag for drive thru use.
   * ADDED: Initial implementation of turn lanes.
  **Bug Fix**
   * CHANGED: Fix destination only penalty for A* and time dependent cases.
   * CHANGED: Use the distance from GetOffsetForHeading, based on road classification and road use (e.g. ramp, turn channel, etc.), within tangent_angle function.
* **Map Matching**
   * FIXED: Fixed trace_route edge_walk server abort [#1365](https://github.com/valhalla/valhalla/pull/1365)
* **Enhancement**
   * ADDED: Added post process for updating free and constrained speeds in the directed edges.
   * UPDATED: Parse the json request once and store in a protocol buffer to pass along the pipeline. This completed the first portion of [1357](https://github.com/valhalla/valhalla/issues/1357)
   * UPDATED: Changed the shape_match attribute from a string to an enum. Fixes [1376](https://github.com/valhalla/valhalla/issues/1376)
   * ADDED: Node bindings for route [#1341](https://github.com/valhalla/valhalla/pull/1341)
   * UPDATED: Use a non-linear use_highways factor (to more heavily penalize highways as use_highways approaches 0).

## Release Date: 2018-07-15 Valhalla 2.6.3
* **API**:
   * FIXED: Use a non-linear use_highways factor (to more heavily penalize highways as use_highways approaches 0).
   * FIXED: Fixed the highway_factor when use_highways < 0.5.
   * ENHANCEMENT: Added logic to modulate the surface factor based on use_trails.
   * ADDED: New customer test requests for motorcycle costing.

## Release Date: 2018-06-28 Valhalla 2.6.2
* **Data Producer Update**
   * FIXED: Complex restriction sorting bug.  Check of has_dt in ComplexRestrictionBuilder::operator==.
* **API**:
   * FIXED: Fixed CostFactory convenience method that registers costing models
   * ADDED: Added use_tolls into motorcycle costing options

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
