ACLOCAL_AMFLAGS = -Im4
AM_LDFLAGS = @BOOST_LDFLAGS@ @COVERAGE_LDFLAGS@ @LUA_LIB@ 
AM_CPPFLAGS = @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@ -I@abs_srcdir@/valhalla -I@abs_srcdir@/valhalla/proto -Igenfiles
AM_CXXFLAGS = @COVERAGE_CXXFLAGS@ -I@abs_srcdir@/valhalla -I@abs_srcdir@/valhalla/proto -Igenfiles @LUA_INCLUDE@
BOOST_LIBS = $(BOOST_DATE_TIME_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_REGEX_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) $(BOOST_IOSTREAMS_LIB)
LIBTOOL_DEPS = @LIBTOOL_DEPS@
libtool: $(LIBTOOL_DEPS)
	$(SHELL) ./config.status libtool

# things for versioning
pkgconfig_DATA = libvalhalla.pc

# conditional test coverage
if ENABLE_COVERAGE
.PHONY: clean-coverage
clean-coverage:
	-find -name '*.gcda' -exec rm -rf {} \;
	-$(LCOV) --directory $(top_builddir) -z
	-rm -rf coverage.info coverage/

.PHONY: coverage-report
coverage-report: clean-coverage
	-$(MAKE) $(AM_MAKEFLAGS) -k check
	$(MAKE) $(AM_MAKEFLAGS) coverage/index.html

coverage.info:
	$(LCOV) --directory $(top_builddir) --base-directory $(top_builddir) --no-external --capture --output-file $@ --no-checksum --compat-libtool

coverage/index.html: coverage.info
	$(GENHTML) --prefix $(top_builddir) --output-directory $(@D) --title "Test Coverage" --legend --show-details $<


.PHONY: clean-gcno
clean-gcno:
	-find -name '*.gcno' -exec rm -rf {} \;

clean-local: clean-coverage clean-gcno clean-genfiles clean-test
else
clean-local: clean-genfiles clean-test
endif

clean-genfiles:
	-rm -rf @abs_builddir@/genfiles

TEST_CLEANFILES =
clean-test:
	-rm -rf $(TEST_CLEANFILES)

genfiles/date_time_zonespec.h:
	-mkdir -p @abs_builddir@/genfiles && cd @abs_srcdir@ && xxd -i -s +$$(head -n 1 date_time/zonespec.csv | wc -c | xargs) date_time/zonespec.csv > @abs_builddir@/genfiles/date_time_zonespec.h
genfiles/graph_lua_proc.h: lua/graph.lua
	-mkdir -p @abs_builddir@/genfiles && cd @abs_srcdir@ && xxd -i lua/graph.lua > @abs_builddir@/genfiles/graph_lua_proc.h
genfiles/admin_lua_proc.h: lua/admin.lua
	-mkdir -p @abs_builddir@/genfiles && cd @abs_srcdir@ && xxd -i lua/admin.lua > @abs_builddir@/genfiles/admin_lua_proc.h
genfiles/locales.h: locales/*.json
	-mkdir -p @abs_builddir@/genfiles && cd @abs_srcdir@/locales && ./make_locales.sh *.json > @abs_builddir@/genfiles/locales.h

#non optional protobuffs
valhalla/proto/tripcommon.pb.h: src/proto/tripcommon.pb.cc
	-mkdir -p valhalla/proto && mv src/proto/tripcommon.pb.h valhalla/proto
src/proto/tripcommon.pb.cc:
	@echo " PROTOC tripcommon.proto" && mkdir -p src/proto && @PROTOC_BIN@ -Iproto --cpp_out=src/proto proto/tripcommon.proto
valhalla/proto/trippath.pb.h: src/proto/trippath.pb.cc
	-mkdir -p valhalla/proto && mv src/proto/trippath.pb.h valhalla/proto
src/proto/trippath.pb.cc:
	@echo " PROTOC trippath.proto" && mkdir -p src/proto && @PROTOC_BIN@ -Iproto --cpp_out=src/proto proto/trippath.proto
valhalla/proto/directions_options.pb.h: src/proto/directions_options.pb.cc
	-mkdir -p valhalla/proto && mv src/proto/directions_options.pb.h valhalla/proto
src/proto/directions_options.pb.cc:
	@echo " PROTOC directions_options.proto" && mkdir -p src/proto && @PROTOC_BIN@ -Iproto --cpp_out=src/proto proto/directions_options.proto
valhalla/proto/tripdirections.pb.h: src/proto/tripdirections.pb.cc
	-mkdir -p valhalla/proto && mv src/proto/tripdirections.pb.h valhalla/proto
src/proto/tripdirections.pb.cc:
	@echo " PROTOC tripdirections.proto" && mkdir -p src/proto && @PROTOC_BIN@ -Iproto --cpp_out=src/proto proto/tripdirections.proto
valhalla/proto/route.pb.h: src/proto/route.pb.cc
	-mkdir -p valhalla/proto && mv src/proto/route.pb.h valhalla/proto
src/proto/route.pb.cc:
	@echo " PROTOC route.proto" && mkdir -p src/proto && @PROTOC_BIN@ -Iproto --cpp_out=src/proto proto/route.proto
valhalla/proto/navigator.pb.h: src/proto/navigator.pb.cc
	-mkdir -p valhalla/proto && mv src/proto/navigator.pb.h valhalla/proto
src/proto/navigator.pb.cc:
	@echo " PROTOC navigator.proto" && mkdir -p src/proto && @PROTOC_BIN@ -Iproto --cpp_out=src/proto proto/navigator.proto

BUILT_SOURCES = genfiles/date_time_zonespec.h genfiles/graph_lua_proc.h genfiles/admin_lua_proc.h genfiles/locales.h valhalla/proto/tripcommon.pb.h valhalla/proto/trippath.pb.h valhalla/proto/directions_options.pb.h valhalla/proto/tripdirections.pb.h valhalla/proto/route.pb.h valhalla/proto/navigator.pb.h src/proto/tripcommon.pb.cc src/proto/trippath.pb.cc src/proto/directions_options.pb.cc src/proto/tripdirections.pb.cc src/proto/route.pb.cc src/proto/navigator.pb.cc
nodist_libvalhalla_la_SOURCES = genfiles/date_time_zonespec.h genfiles/graph_lua_proc.h genfiles/admin_lua_proc.h genfiles/locales.h
CLEANFILES = $(EXTRA_PROGRAMS) genfiles/date_time_zonespec.h genfiles/graph_lua_proc.h genfiles/admin_lua_proc.h genfiles/locales.h valhalla/proto/tripcommon.pb.h valhalla/proto/trippath.pb.h valhalla/proto/directions_options.pb.h valhalla/proto/tripdirections.pb.h valhalla/proto/route.pb.h valhalla/proto/navigator.pb.h src/proto/tripcommon.pb.cc src/proto/trippath.pb.cc src/proto/directions_options.pb.cc src/proto/tripdirections.pb.cc src/proto/route.pb.cc src/proto/navigator.pb.cc

#data tools protobuffs
if DATA_TOOLS
valhalla/proto/osmformat.pb.h: src/proto/osmformat.pb.cc
	-mkdir -p valhalla/proto && mv src/proto/osmformat.pb.h valhalla/proto
src/proto/osmformat.pb.cc:
	@echo " PROTOC osmformat.proto" && mkdir -p src/proto && @PROTOC_BIN@ -Iproto --cpp_out=src/proto proto/osmformat.proto
valhalla/proto/fileformat.pb.h: src/proto/fileformat.pb.cc
	-mkdir -p valhalla/proto && mv src/proto/fileformat.pb.h valhalla/proto
src/proto/fileformat.pb.cc:
	@echo " PROTOC fileformat.proto" && mkdir -p src/proto && @PROTOC_BIN@ -Iproto --cpp_out=src/proto proto/fileformat.proto
valhalla/proto/transit.pb.h: src/proto/transit.pb.cc
	-mkdir -p valhalla/proto && mv src/proto/transit.pb.h valhalla/proto
src/proto/transit.pb.cc:
	@echo " PROTOC transit.proto" && mkdir -p src/proto && @PROTOC_BIN@ -Iproto --cpp_out=src/proto proto/transit.proto
valhalla/proto/transit_fetch.pb.h: src/proto/transit_fetch.pb.cc
	-mkdir -p valhalla/proto && mv src/proto/transit_fetch.pb.h valhalla/proto
src/proto/transit_fetch.pb.cc:
	@echo " PROTOC transit_fetch.proto" && mkdir -p src/proto && @PROTOC_BIN@ -Iproto --cpp_out=src/proto proto/transit_fetch.proto
valhalla/proto/tile.pb.h: src/proto/tile.pb.cc
	-mkdir -p valhalla/proto && mv src/proto/tile.pb.h valhalla/proto
src/proto/tile.pb.cc:
	@echo " PROTOC tile.proto" && mkdir -p src/proto && @PROTOC_BIN@ -Iproto --cpp_out=src/proto proto/tile.proto
valhalla/proto/segment.pb.h: src/proto/segment.pb.cc
	-mkdir -p valhalla/proto && mv src/proto/segment.pb.h valhalla/proto
src/proto/segment.pb.cc:
	@echo " PROTOC segment.proto" && mkdir -p src/proto && @PROTOC_BIN@ -Iproto --cpp_out=src/proto proto/segment.proto
BUILT_SOURCES += valhalla/proto/osmformat.pb.h valhalla/proto/fileformat.pb.h valhalla/proto/transit.pb.h valhalla/proto/transit_fetch.pb.h valhalla/proto/tile.pb.h valhalla/proto/segment.pb.h
nodist_libvalhalla_la_SOURCES += valhalla/proto/osmformat.pb.h valhalla/proto/fileformat.pb.h valhalla/proto/transit.pb.h valhalla/proto/transit_fetch.pb.h valhalla/proto/tile.pb.h valhalla/proto/segment.pb.h
CLEANFILES += valhalla/proto/osmformat.pb.h valhalla/proto/fileformat.pb.h valhalla/proto/transit.pb.h valhalla/proto/transit_fetch.pb.h valhalla/proto/tile.pb.h valhalla/proto/segment.pb.h src/proto/tripcommon.pb.cc src/proto/trippath.pb.cc src/proto/directions_options.pb.cc src/proto/tripdirections.pb.cc src/proto/osmformat.pb.cc src/proto/fileformat.pb.cc src/proto/transit.pb.cc src/proto/transit_fetch.pb.cc src/proto/tile.pb.cc src/proto/segment.pb.cc
endif

lib_LTLIBRARIES = libvalhalla.la
nobase_include_HEADERS = \
	valhalla/valhalla.h \
	valhalla/exception.h \
	valhalla/worker.h \
	valhalla/midgard/linesegment2.h \
	valhalla/midgard/tiles.h \
	valhalla/midgard/gridded_data.h \
	valhalla/midgard/polyline2.h \
	valhalla/midgard/obb2.h \
	valhalla/midgard/pointll.h \
	valhalla/midgard/vector2.h \
	valhalla/midgard/constants.h \
	valhalla/midgard/aabb2.h \
	valhalla/midgard/point2.h \
	valhalla/midgard/util.h \
	valhalla/midgard/distanceapproximator.h \
	valhalla/midgard/ellipse.h \
	valhalla/midgard/sequence.h \
	valhalla/midgard/shape_decoder.h \
	valhalla/midgard/encoded.h \
	valhalla/midgard/logging.h \
	valhalla/baldr/accessrestriction.h \
	valhalla/baldr/admin.h \
	valhalla/baldr/admininfo.h \
	valhalla/baldr/complexrestriction.h \
	valhalla/baldr/connectivity_map.h \
	valhalla/baldr/datetime.h \
	valhalla/baldr/directededge.h \
	valhalla/baldr/double_bucket_queue.h \
	valhalla/baldr/edge_elevation.h \
	valhalla/baldr/edgeinfo.h \
	valhalla/baldr/geojson.h \
	valhalla/baldr/graphconstants.h \
	valhalla/baldr/graphid.h \
	valhalla/baldr/graphreader.h \
	valhalla/baldr/graphtile.h \
	valhalla/baldr/graphtileheader.h \
	valhalla/baldr/json.h \
	valhalla/baldr/nodeinfo.h \
	valhalla/baldr/location.h \
	valhalla/baldr/pathlocation.h \
	valhalla/baldr/rapidjson_utils.h \
	valhalla/baldr/sign.h \
	valhalla/baldr/signinfo.h \
	valhalla/baldr/tilehierarchy.h \
	valhalla/baldr/turn.h \
	valhalla/baldr/streetname.h \
	valhalla/baldr/streetnames.h \
	valhalla/baldr/streetnames_factory.h \
	valhalla/baldr/streetname_us.h \
	valhalla/baldr/streetnames_us.h \
	valhalla/baldr/trafficassociation.h \
	valhalla/baldr/transitdeparture.h \
	valhalla/baldr/transitroute.h \
	valhalla/baldr/transitschedule.h \
	valhalla/baldr/transitstop.h \
	valhalla/baldr/transittransfer.h \
	valhalla/baldr/laneconnectivity.h \
	valhalla/baldr/verbal_text_formatter.h \
	valhalla/baldr/verbal_text_formatter_us.h \
	valhalla/baldr/verbal_text_formatter_us_co.h \
	valhalla/baldr/verbal_text_formatter_us_tx.h \
	valhalla/baldr/verbal_text_formatter_factory.h \
	valhalla/baldr/reutil.h \
	valhalla/baldr/edgetracker.h \
	valhalla/baldr/merge.h \
	valhalla/sif/costconstants.h \
	valhalla/sif/costfactory.h \
	valhalla/sif/autocost.h \
	valhalla/sif/bicyclecost.h \
	valhalla/sif/motorscootercost.h \
	valhalla/sif/pedestriancost.h \
	valhalla/sif/transitcost.h \
	valhalla/sif/truckcost.h \
	valhalla/sif/dynamiccost.h \
	valhalla/sif/hierarchylimits.h \
	valhalla/sif/edgelabel.h \
	valhalla/meili/universal_cost.h \
	valhalla/meili/candidate_search.h \
	valhalla/meili/geometry_helpers.h \
	valhalla/meili/grid_range_query.h \
	valhalla/meili/emission_cost_model.h \
	valhalla/meili/transition_cost_model.h \
	valhalla/meili/priority_queue.h \
	valhalla/meili/routing.h \
	valhalla/meili/stateid.h \
	valhalla/meili/viterbi_search.h \
	valhalla/meili/topk_search.h \
	valhalla/meili/measurement.h \
	valhalla/meili/match_result.h \
	valhalla/meili/grid_traversal.h \
	valhalla/meili/state.h \
	valhalla/meili/map_matcher.h \
	valhalla/meili/map_matcher_factory.h \
	valhalla/meili/traffic_segment_matcher.h \
	valhalla/skadi/worker.h \
	valhalla/skadi/sample.h \
	valhalla/skadi/util.h \
	valhalla/loki/worker.h \
	valhalla/loki/search.h \
	valhalla/loki/node_search.h \
	valhalla/proto/tripcommon.pb.h \
	valhalla/proto/trippath.pb.h \
	valhalla/proto/tripdirections.pb.h \
	valhalla/proto/route.pb.h \
	valhalla/proto/navigator.pb.h \
	valhalla/proto/directions_options.pb.h \
	valhalla/odin/worker.h \
	valhalla/odin/directionsbuilder.h \
	valhalla/odin/maneuversbuilder.h \
	valhalla/odin/narrative_dictionary.h \
	valhalla/odin/narrative_builder_factory.h \
	valhalla/odin/narrativebuilder.h \
	valhalla/odin/enhancedtrippath.h \
	valhalla/odin/maneuver.h \
	valhalla/odin/sign.h \
	valhalla/odin/signs.h \
	valhalla/odin/util.h \
	valhalla/odin/transitrouteinfo.h \
	valhalla/thor/worker.h \
	valhalla/thor/astar.h \
	valhalla/thor/astarheuristic.h \
	valhalla/thor/bidirectional_astar.h \
	valhalla/thor/costmatrix.h \
	valhalla/thor/edgestatus.h \
	valhalla/thor/isochrone.h \
	valhalla/thor/optimizer.h \
	valhalla/thor/map_matcher.h \
	valhalla/thor/match_result.h \
	valhalla/thor/multimodal.h \
	valhalla/thor/pathalgorithm.h \
	valhalla/thor/pathinfo.h \
	valhalla/thor/route_matcher.h \
	valhalla/thor/trippathbuilder.h \
	valhalla/thor/attributes_controller.h \
	valhalla/thor/trafficalgorithm.h \
	valhalla/thor/timedistancematrix.h \
	valhalla/tyr/serializers.h \
	valhalla/tyr/navigator.h \
	valhalla/tyr/actor.h
libvalhalla_la_SOURCES = \
	src/worker.cc \
	src/midgard/linesegment2.cc \
	src/midgard/tiles.cc \
	src/midgard/gridded_data.cc \
	src/midgard/polyline2.cc \
	src/midgard/obb2.cc \
	src/midgard/pointll.cc \
	src/midgard/vector2.cc \
	src/midgard/aabb2.cc \
	src/midgard/point2.cc \
	src/midgard/util.cc \
	src/midgard/ellipse.cc \
	src/midgard/logging.cc \
	src/baldr/accessrestriction.cc \
	src/baldr/admin.cc \
	src/baldr/admininfo.cc \
	src/baldr/complexrestriction.cc \
	src/baldr/connectivity_map.cc \
	src/baldr/datetime.cc \
	src/baldr/directededge.cc \
	src/baldr/double_bucket_queue.cc \
	src/baldr/edge_elevation.cc \
	src/baldr/edgeinfo.cc \
	src/baldr/geojson.cc \
	src/baldr/graphid.cc \
	src/baldr/graphreader.cc \
	src/baldr/graphtile.cc \
	src/baldr/graphtileheader.cc \
	src/baldr/edgetracker.cc \
	src/baldr/merge.cc \
	src/baldr/nodeinfo.cc \
	src/baldr/location.cc \
	src/baldr/pathlocation.cc \
	src/baldr/sign.cc \
	src/baldr/signinfo.cc \
	src/baldr/tilehierarchy.cc \
	src/baldr/turn.cc \
	src/baldr/streetname.cc \
	src/baldr/streetnames.cc \
	src/baldr/streetnames_factory.cc \
	src/baldr/streetname_us.cc \
	src/baldr/streetnames_us.cc \
	src/baldr/transitdeparture.cc \
	src/baldr/transitroute.cc \
	src/baldr/transitschedule.cc \
	src/baldr/transitstop.cc \
	src/baldr/transittransfer.cc \
	src/baldr/laneconnectivity.cc \
	src/baldr/verbal_text_formatter.cc \
	src/baldr/verbal_text_formatter_us.cc \
	src/baldr/verbal_text_formatter_us_co.cc \
	src/baldr/verbal_text_formatter_us_tx.cc \
	src/baldr/verbal_text_formatter_factory.cc \
	src/baldr/date_time_zonespec.h \
	src/sif/autocost.cc \
	src/sif/bicyclecost.cc \
	src/sif/motorscootercost.cc \
	src/sif/pedestriancost.cc \
	src/sif/transitcost.cc \
	src/sif/truckcost.cc \
	src/sif/dynamiccost.cc \
	src/meili/universal_cost.cc \
	src/meili/viterbi_search.cc \
	src/meili/topk_search.cc \
	src/meili/routing.cc \
	src/meili/candidate_search.cc \
	src/meili/transition_cost_model.cc \
	src/meili/map_matcher.cc \
	src/meili/map_matcher_factory.cc \
	src/meili/match_route.cc \
	src/meili/traffic_segment_matcher.cc \
	src/skadi/sample.cc \
	src/skadi/worker.cc \
	src/skadi/util.cc \
	src/loki/search.cc \
	src/loki/worker.cc \
	src/loki/locate_action.cc \
	src/loki/route_action.cc \
	src/loki/matrix_action.cc \
	src/loki/isochrone_action.cc \
	src/loki/trace_route_action.cc \
	src/loki/transit_available_action.cc \
	src/loki/node_search.cc \
	src/proto/tripcommon.pb.cc \
	src/proto/trippath.pb.cc \
	src/proto/tripdirections.pb.cc \
	src/proto/route.pb.cc \
	src/proto/navigator.pb.cc \
	src/proto/directions_options.pb.cc \
	src/odin/directionsbuilder.cc \
	src/odin/maneuversbuilder.cc \
	src/odin/narrative_dictionary.cc \
	src/odin/narrative_builder_factory.cc \
	src/odin/narrativebuilder.cc \
	src/odin/enhancedtrippath.cc \
	src/odin/maneuver.cc \
	src/odin/sign.cc \
	src/odin/signs.cc \
	src/odin/util.cc \
	src/odin/transitrouteinfo.cc \
	src/odin/locales.h \
	src/odin/worker.cc \
	src/thor/astar.cc \
	src/thor/bidirectional_astar.cc \
	src/thor/costmatrix.cc \
	src/thor/isochrone.cc \
	src/thor/map_matcher.cc \
	src/thor/multimodal.cc \
	src/thor/optimizer.cc \
	src/thor/trippathbuilder.cc \
	src/thor/attributes_controller.cc \
	src/thor/route_matcher.cc \
	src/thor/trafficalgorithm.cc \
	src/thor/timedistancematrix.cc \
	src/thor/worker.cc \
	src/thor/isochrone_action.cc \
	src/thor/matrix_action.cc \
	src/thor/optimized_route_action.cc \
	src/thor/route_action.cc \
	src/thor/trace_attributes_action.cc \
	src/thor/trace_route_action.cc \
	src/tyr/serializers.cc \
	src/tyr/navigator.cc \
	src/tyr/actor.cc
libvalhalla_la_CPPFLAGS = @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@ $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
libvalhalla_la_LIBADD = @BOOST_LDFLAGS@ @PROTOC_LIBS@ $(BOOST_LIBS) $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS)

if DATA_TOOLS
nobase_include_HEADERS += \
	valhalla/mjolnir/admin.h \
	valhalla/mjolnir/countryaccess.h \
	valhalla/mjolnir/complexrestrictionbuilder.h \
	valhalla/mjolnir/dataquality.h \
	valhalla/mjolnir/directededgebuilder.h \
	valhalla/mjolnir/graphtilebuilder.h \
	valhalla/mjolnir/edgeinfobuilder.h \
	valhalla/mjolnir/uniquenames.h \
	valhalla/mjolnir/ferry_connections.h \
	valhalla/mjolnir/graphbuilder.h \
	valhalla/mjolnir/graphenhancer.h \
	valhalla/mjolnir/graphvalidator.h \
	valhalla/mjolnir/hierarchybuilder.h \
	valhalla/mjolnir/idtable.h \
	valhalla/mjolnir/linkclassification.h \
	valhalla/mjolnir/luatagtransform.h \
	valhalla/mjolnir/node_expander.h \
	valhalla/mjolnir/osmaccess.h \
	valhalla/mjolnir/osmadmin.h \
	valhalla/mjolnir/osmdata.h \
	valhalla/mjolnir/osmnode.h \
	valhalla/mjolnir/osmpbfparser.h \
	valhalla/mjolnir/osmaccessrestriction.h \
	valhalla/mjolnir/osmrestriction.h \
	valhalla/mjolnir/osmway.h \
	valhalla/mjolnir/pbfadminparser.h \
	valhalla/mjolnir/pbfgraphparser.h \
	valhalla/mjolnir/restrictionbuilder.h \
	valhalla/mjolnir/shortcutbuilder.h \
	valhalla/mjolnir/transitbuilder.h \
	valhalla/mjolnir/util.h \
	valhalla/mjolnir/validatetransit.h
libvalhalla_la_SOURCES += \
	src/mjolnir/admin.cc \
	src/mjolnir/complexrestrictionbuilder.cc \
	src/mjolnir/countryaccess.cc \
	src/mjolnir/dataquality.cc \
	src/mjolnir/directededgebuilder.cc \
	src/mjolnir/graphtilebuilder.cc \
	src/mjolnir/edgeinfobuilder.cc \
	src/mjolnir/uniquenames.cc \
	src/proto/fileformat.pb.cc \
	src/proto/osmformat.pb.cc \
	src/mjolnir/ferry_connections.cc \
	src/mjolnir/graphbuilder.cc \
	src/mjolnir/graphenhancer.cc \
	src/mjolnir/graphvalidator.cc \
	src/mjolnir/hierarchybuilder.cc \
	src/mjolnir/linkclassification.cc \
	src/mjolnir/luatagtransform.cc \
	src/mjolnir/node_expander.cc \
	src/mjolnir/osmaccess.cc \
	src/mjolnir/osmadmin.cc \
	src/mjolnir/osmnode.cc \
	src/mjolnir/osmpbfparser.cc \
	src/mjolnir/osmaccessrestriction.cc \
	src/mjolnir/osmrestriction.cc \
	src/mjolnir/osmway.cc \
	src/mjolnir/pbfadminparser.cc \
	src/mjolnir/pbfgraphparser.cc \
	src/mjolnir/restrictionbuilder.cc \
	src/mjolnir/shortcutbuilder.cc \
	src/mjolnir/transitbuilder.cc \
	src/mjolnir/util.cc \
	src/mjolnir/validatetransit.cc \
	src/mjolnir/graph_lua_proc.h \
	src/mjolnir/admin_lua_proc.h
libvalhalla_la_LIBADD += -lspatialite -lsqlite3
endif


if PYTHON_BINDINGS
pyexec_LTLIBRARIES = \
	valhalla.la
valhalla_la_SOURCES = src/bindings/python/python.cc
valhalla_la_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@ @PYTHON_CPPFLAGS@
valhalla_la_LIBADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ @PYTHON_LIBS@ -l$(BOOST_PYTHON_LIB) libvalhalla.la
valhalla_la_LDFLAGS = @COVERAGE_LDFLAGS@ -avoid-version -module -shared
endif

bin_PROGRAMS = \
	valhalla_run_map_match \
	valhalla_benchmark_loki \
	valhalla_benchmark_skadi \
	valhalla_run_isochrone \
	valhalla_run_route \
	valhalla_benchmark_adjacency_list \
	valhalla_run_matrix \
	valhalla_path_comparison \
	valhalla_export_edges
valhalla_run_map_match_SOURCES = src/meili/valhalla_run_map_match.cc
valhalla_run_map_match_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_run_map_match_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
valhalla_benchmark_loki_SOURCES = src/valhalla_benchmark_loki.cc
valhalla_benchmark_loki_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_benchmark_loki_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
valhalla_benchmark_skadi_SOURCES = src/valhalla_benchmark_skadi.cc
valhalla_benchmark_skadi_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_benchmark_skadi_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
valhalla_run_isochrone_SOURCES = src/valhalla_run_isochrone.cc
valhalla_run_isochrone_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_run_isochrone_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
valhalla_run_route_SOURCES = src/valhalla_run_route.cc
valhalla_run_route_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_run_route_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
valhalla_benchmark_adjacency_list_SOURCES = src/valhalla_benchmark_adjacency_list.cc
valhalla_benchmark_adjacency_list_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_benchmark_adjacency_list_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
valhalla_run_matrix_SOURCES = src/valhalla_run_matrix.cc
valhalla_run_matrix_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_run_matrix_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
valhalla_path_comparison_SOURCES = src/valhalla_path_comparison.cc
valhalla_path_comparison_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_path_comparison_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
valhalla_export_edges_SOURCES = src/valhalla_export_edges.cc
valhalla_export_edges_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_export_edges_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la

if SERVICES
bin_PROGRAMS += \
	valhalla_route_service \
	valhalla_skadi_worker \
	valhalla_loki_worker \
	valhalla_odin_worker \
	valhalla_thor_worker \
	valhalla_elevation_service 
valhalla_skadi_worker_SOURCES = src/valhalla_skadi_worker.cc
valhalla_skadi_worker_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_skadi_worker_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
valhalla_loki_worker_SOURCES = src/valhalla_loki_worker.cc
valhalla_loki_worker_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_loki_worker_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
valhalla_odin_worker_SOURCES = src/valhalla_odin_worker.cc
valhalla_odin_worker_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_odin_worker_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
valhalla_thor_worker_SOURCES = src/valhalla_thor_worker.cc
valhalla_thor_worker_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_thor_worker_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
valhalla_elevation_service_SOURCES = src/valhalla_elevation_service.cc
valhalla_elevation_service_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_elevation_service_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
valhalla_route_service_SOURCES = src/valhalla_route_service.cc
valhalla_route_service_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_route_service_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
endif

if DATA_TOOLS
bin_PROGRAMS += \
	valhalla_benchmark_admins \
	valhalla_build_connectivity \
	valhalla_build_tiles \
	valhalla_build_admins \
	valhalla_build_transit \
	valhalla_fetch_transit \
	valhalla_query_transit \
	valhalla_validate_transit \
	valhalla_ways_to_edges \
	valhalla_build_speeds \
	valhalla_build_statistics \
	valhalla_associate_segments
valhalla_benchmark_admins_SOURCES = src/mjolnir/valhalla_benchmark_admins.cc
valhalla_benchmark_admins_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_benchmark_admins_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ @PROTOC_LIBS@ -lz -lgeos -lsqlite3 -lspatialite $(BOOST_LIBS) libvalhalla.la
valhalla_build_connectivity_SOURCES = src/mjolnir/valhalla_build_connectivity.cc
valhalla_build_connectivity_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_build_connectivity_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
valhalla_build_tiles_SOURCES = src/mjolnir/valhalla_build_tiles.cc
valhalla_build_tiles_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_build_tiles_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ @PROTOC_LIBS@ -lz -lsqlite3 -lspatialite $(BOOST_LIBS) libvalhalla.la
valhalla_build_admins_SOURCES = src/mjolnir/valhalla_build_admins.cc
valhalla_build_admins_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_build_admins_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ @PROTOC_LIBS@ -lz -lgeos -lsqlite3 -lspatialite $(BOOST_LIBS) libvalhalla.la
valhalla_build_transit_SOURCES = src/mjolnir/valhalla_build_transit.cc src/proto/transit.pb.cc
valhalla_build_transit_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_build_transit_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ @PROTOC_LIBS@ -lz $(BOOST_LIBS) libvalhalla.la
valhalla_fetch_transit_SOURCES = src/mjolnir/valhalla_fetch_transit.cc  src/proto/transit_fetch.pb.cc
valhalla_fetch_transit_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_fetch_transit_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ @PROTOC_LIBS@ -lz $(BOOST_LIBS) libvalhalla.la
valhalla_query_transit_SOURCES = src/mjolnir/valhalla_query_transit.cc src/proto/transit.pb.cc
valhalla_query_transit_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_query_transit_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ @PROTOC_LIBS@ -lz $(BOOST_LIBS) libvalhalla.la
valhalla_validate_transit_SOURCES = src/mjolnir/valhalla_validate_transit.cc
valhalla_validate_transit_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_validate_transit_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ @PROTOC_LIBS@ -lz $(BOOST_LIBS) libvalhalla.la
valhalla_ways_to_edges_SOURCES = src/mjolnir/valhalla_ways_to_edges.cc
valhalla_ways_to_edges_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_ways_to_edges_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ @PROTOC_LIBS@ -lz $(BOOST_LIBS) libvalhalla.la
valhalla_build_speeds_SOURCES = src/mjolnir/valhalla_build_speeds.cc
valhalla_build_speeds_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_build_speeds_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ @PROTOC_LIBS@ -lsqlite3 $(BOOST_LIBS) libvalhalla.la
valhalla_build_statistics_SOURCES = src/mjolnir/valhalla_build_statistics.cc src/mjolnir/statistics.cc src/mjolnir/statistics_database.cc src/mjolnir/statistics.h
valhalla_build_statistics_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_build_statistics_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ @PROTOC_LIBS@ -lz $(BOOST_LIBS) libvalhalla.la
valhalla_associate_segments_SOURCES = src/valhalla_associate_segments.cc src/proto/segment.pb.cc src/proto/tile.pb.cc
valhalla_associate_segments_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
valhalla_associate_segments_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
endif

bin_SCRIPTS = \
	scripts/valhalla_build_elevation \
	scripts/valhalla_build_config
if DATA_TOOLS
bin_SCRIPTS += scripts/valhalla_build_timezones
endif

TESTS_ENVIRONMENT = LOCPATH=locales
check_PROGRAMS = \
	test/logging \
	test/point2 \
	test/distanceapproximator \
	test/aabb2 \
	test/obb2 \
	test/linesegment2 \
	test/vector2 \
	test/polyline2 \
	test/pointll \
	test/ellipse \
	test/encode \
	test/tiles \
	test/sequence \
	test/util_midgard \
	test/gridded_data \
	test/location \
	test/admin \
	test/datetime \
	test/directededge \
	test/double_bucket_queue \
	test/edge_elevation \
	test/edgecollapser \
	test/laneconnectivity \
	test/graphid \
	test/tilehierarchy \
	test/graphtile \
	test/nodeinfo \
	test/turn \
	test/graphreader \
	test/streetname \
	test/streetname_us \
	test/streetnames \
	test/streetnames_us \
	test/streetnames_factory \
	test/json \
	test/verbal_text_formatter \
	test/verbal_text_formatter_us \
	test/verbal_text_formatter_us_co \
	test/verbal_text_formatter_us_tx \
	test/factory \
	test/grid_range_query \
	test/grid_traversal \
	test/map_matcher_factory \
	test/queue \
	test/routing \
	test/viterbi_search \
	test/util_skadi \
	test/sample \
	test/maneuversbuilder \
	test/narrativebuilder \
	test/navigator \
	test/enhancedtrippath \
	test/sign \
	test/signs \
	test/util_odin \
	test/narrative_dictionary \
	test/edgestatus \
	test/optimizer \
	test/attributes_controller \
	test/astar \
	test/serializers \
	test/traffic_matcher \
	test/autocost \
	test/motorscootercost \
	test/bicyclecost \
	test/pedestriancost \
	test/transitcost \
	test/truckcost \
	test/actor
test_logging_SOURCES = test/logging.cc test/test.cc
test_logging_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) -DLOGGING_LEVEL_ALL
test_logging_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_point2_SOURCES = test/point2.cc test/test.cc
test_point2_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_point2_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_distanceapproximator_SOURCES = test/distanceapproximator.cc test/test.cc
test_distanceapproximator_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_distanceapproximator_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_aabb2_SOURCES = test/aabb2.cc test/test.cc
test_aabb2_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_aabb2_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_obb2_SOURCES = test/obb2.cc test/test.cc
test_obb2_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_obb2_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_linesegment2_SOURCES = test/linesegment2.cc test/test.cc
test_linesegment2_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_linesegment2_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_vector2_SOURCES = test/vector2.cc test/test.cc
test_vector2_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_vector2_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_polyline2_SOURCES = test/polyline2.cc test/test.cc
test_polyline2_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_polyline2_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_pointll_SOURCES = test/pointll.cc test/test.cc
test_pointll_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_pointll_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_ellipse_SOURCES = test/ellipse.cc test/test.cc
test_ellipse_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_ellipse_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_encode_SOURCES = test/encode.cc test/test.cc
test_encode_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_encode_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_tiles_SOURCES = test/tiles.cc test/test.cc
test_tiles_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_tiles_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_sequence_SOURCES = test/sequence.cc test/test.cc
test_sequence_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_sequence_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_util_midgard_SOURCES = test/util_midgard.cc test/test.cc
test_util_midgard_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_util_midgard_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_gridded_data_SOURCES = test/gridded_data.cc test/test.cc
test_gridded_data_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) 
test_gridded_data_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_location_SOURCES = test/location.cc test/test.cc
test_location_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) 
test_location_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_admin_SOURCES = test/admin.cc test/test.cc
test_admin_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_admin_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_datetime_SOURCES = test/datetime.cc test/test.cc
test_datetime_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_datetime_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_directededge_SOURCES = test/directededge.cc test/test.cc
test_directededge_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_directededge_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_edge_elevation_SOURCES = test/edge_elevation.cc test/test.cc
test_edge_elevation_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_edge_elevation_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_double_bucket_queue_SOURCES = test/double_bucket_queue.cc test/test.cc
test_double_bucket_queue_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_double_bucket_queue_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_edgecollapser_SOURCES = test/edgecollapser.cc test/test.cc
test_edgecollapser_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_edgecollapser_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_laneconnectivity_SOURCES = test/laneconnectivity.cc test/test.cc
test_laneconnectivity_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_laneconnectivity_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_graphid_SOURCES = test/graphid.cc test/test.cc
test_graphid_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_graphid_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_tilehierarchy_SOURCES = test/tilehierarchy.cc test/test.cc
test_tilehierarchy_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_tilehierarchy_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_graphtile_SOURCES = test/graphtile.cc test/test.cc
test_graphtile_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_graphtile_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_nodeinfo_SOURCES = test/nodeinfo.cc test/test.cc
test_nodeinfo_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_nodeinfo_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_turn_SOURCES = test/turn.cc test/test.cc
test_turn_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_turn_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_graphreader_SOURCES = test/graphreader.cc test/test.cc
test_graphreader_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_graphreader_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_streetname_SOURCES = test/streetname.cc test/test.cc
test_streetname_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_streetname_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_streetname_us_SOURCES = test/streetname_us.cc test/test.cc
test_streetname_us_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_streetname_us_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_streetnames_SOURCES = test/streetnames.cc test/test.cc
test_streetnames_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_streetnames_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_streetnames_us_SOURCES = test/streetnames_us.cc test/test.cc
test_streetnames_us_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_streetnames_us_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_streetnames_factory_SOURCES = test/streetnames_factory.cc test/test.cc
test_streetnames_factory_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS)
test_streetnames_factory_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) $(BOOST_LIBS) libvalhalla.la
test_json_SOURCES = test/json.cc test/test.cc
test_json_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_json_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_verbal_text_formatter_SOURCES = test/verbal_text_formatter.cc test/test.cc
test_verbal_text_formatter_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_verbal_text_formatter_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_verbal_text_formatter_us_SOURCES = test/verbal_text_formatter_us.cc test/test.cc
test_verbal_text_formatter_us_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_verbal_text_formatter_us_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_verbal_text_formatter_us_co_SOURCES = test/verbal_text_formatter_us_co.cc test/test.cc
test_verbal_text_formatter_us_co_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_verbal_text_formatter_us_co_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_verbal_text_formatter_us_tx_SOURCES = test/verbal_text_formatter_us_tx.cc test/test.cc
test_verbal_text_formatter_us_tx_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_verbal_text_formatter_us_tx_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_factory_SOURCES = test/factory.cc test/test.cc
test_factory_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_factory_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_grid_range_query_SOURCES = test/grid_range_query.cc test/test.cc
test_grid_range_query_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_grid_range_query_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_grid_traversal_SOURCES = test/grid_traversal.cc test/test.cc
test_grid_traversal_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_grid_traversal_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_map_matcher_factory_SOURCES = test/map_matcher_factory.cc test/test.cc
test_map_matcher_factory_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_map_matcher_factory_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_queue_SOURCES = test/queue.cc test/test.cc
test_queue_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_queue_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_routing_SOURCES = test/routing.cc test/test.cc
test_routing_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_routing_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_viterbi_search_SOURCES = test/viterbi_search.cc test/test.cc
test_viterbi_search_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_viterbi_search_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_util_skadi_SOURCES = test/util_skadi.cc test/test.cc
test_util_skadi_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_util_skadi_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_sample_SOURCES = test/sample.cc test/test.cc
test_sample_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_sample_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_maneuversbuilder_SOURCES = test/maneuversbuilder.cc test/test.cc
test_maneuversbuilder_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_maneuversbuilder_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_narrativebuilder_SOURCES = test/narrativebuilder.cc test/test.cc
test_narrativebuilder_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_narrativebuilder_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_navigator_SOURCES = test/navigator.cc test/test.cc
test_navigator_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_navigator_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_enhancedtrippath_SOURCES = test/enhancedtrippath.cc test/test.cc
test_enhancedtrippath_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_enhancedtrippath_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_sign_SOURCES = test/sign.cc test/test.cc
test_sign_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_sign_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_signs_SOURCES = test/signs.cc test/test.cc
test_signs_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_signs_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_util_odin_SOURCES = test/util_odin.cc test/test.cc
test_util_odin_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_util_odin_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_narrative_dictionary_SOURCES = test/narrative_dictionary.cc test/test.cc
test_narrative_dictionary_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_narrative_dictionary_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_edgestatus_SOURCES = test/edgestatus.cc test/test.cc
test_edgestatus_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_edgestatus_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_optimizer_SOURCES = test/optimizer.cc test/test.cc
test_optimizer_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_optimizer_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_attributes_controller_SOURCES = test/attributes_controller.cc test/test.cc
test_attributes_controller_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_attributes_controller_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_astar_SOURCES = test/astar.cc test/test.cc
test_astar_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_astar_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_serializers_SOURCES = test/serializers.cc test/test.cc
test_serializers_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_serializers_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_traffic_matcher_SOURCES = test/traffic_matcher.cc test/test.cc
test_traffic_matcher_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_traffic_matcher_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_autocost_SOURCES = src/sif/autocost.cc test/test.cc
test_autocost_CPPFLAGS = -DINLINE_TEST $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_autocost_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_motorscootercost_SOURCES = src/sif/motorscootercost.cc test/test.cc
test_motorscootercost_CPPFLAGS = -DINLINE_TEST $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_motorscootercost_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_bicyclecost_SOURCES = src/sif/bicyclecost.cc test/test.cc
test_bicyclecost_CPPFLAGS = -DINLINE_TEST $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_bicyclecost_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_pedestriancost_SOURCES = src/sif/pedestriancost.cc test/test.cc
test_pedestriancost_CPPFLAGS = -DINLINE_TEST $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_pedestriancost_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_transitcost_SOURCES = src/sif/transitcost.cc test/test.cc
test_transitcost_CPPFLAGS = -DINLINE_TEST $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_transitcost_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_truckcost_SOURCES = src/sif/truckcost.cc test/test.cc
test_truckcost_CPPFLAGS = -DINLINE_TEST $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_truckcost_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_actor_SOURCES = test/actor.cc test/test.cc
test_actor_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_actor_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la

if SERVICES
check_PROGRAMS += \
	test/skadi_service \
	test/loki_service \
	test/thor_service
test_skadi_service_SOURCES = test/skadi_service.cc test/test.cc
test_skadi_service_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_skadi_service_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_loki_service_SOURCES = test/loki_service.cc test/test.cc
test_loki_service_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_loki_service_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_thor_service_SOURCES = test/thor_service.cc test/test.cc
test_thor_service_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_thor_service_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
endif

if DATA_TOOLS
#some of the tests want tiles of a certain location so lets build those
test/data/utrecht_tiles: valhalla_build_tiles
	@echo "Building Utrecht Tiles..." && rm -rf test/data/*utrecht_tiles && ./valhalla_build_tiles --inline-config '{"mjolnir":{"tile_dir":"test/data/tmp_utrecht_tiles","concurrency":1,"logging":{"type":""}}}' test/data/utrecht_netherlands.osm.pbf && mv test/data/tmp_utrecht_tiles test/data/utrecht_tiles
TEST_CLEANFILES += test/data/utrecht_tiles

check_PROGRAMS += \
	test/mapmatch \
	test/utrecht \
	test/edgeinfobuilder \
	test/uniquenames \
	test/idtable \
	test/graphbuilder \
	test/graphparser \
	test/names \
	test/refs \
	test/signinfo \
	test/countryaccess \
	test/graphtilebuilder \
	test/search \
	test/node_search \
	test/matrix
test_mapmatch_DEPENDENCIES = test/data/utrecht_tiles
test_mapmatch_SOURCES = test/mapmatch.cc test/test.cc
test_mapmatch_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_mapmatch_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_utrecht_SOURCES = test/utrecht.cc test/test.cc
test_utrecht_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_utrecht_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_edgeinfobuilder_SOURCES = test/edgeinfobuilder.cc test/test.cc
test_edgeinfobuilder_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_edgeinfobuilder_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_uniquenames_SOURCES = test/uniquenames.cc test/test.cc
test_uniquenames_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_uniquenames_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_idtable_SOURCES = test/idtable.cc test/test.cc
test_idtable_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_idtable_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_graphbuilder_SOURCES = test/graphbuilder.cc test/test.cc
test_graphbuilder_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_graphbuilder_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_graphparser_SOURCES = test/graphparser.cc test/test.cc
test_graphparser_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_graphparser_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_names_SOURCES = test/names.cc test/test.cc
test_names_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_names_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_refs_SOURCES = test/refs.cc test/test.cc
test_refs_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_refs_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_signinfo_SOURCES = test/signinfo.cc test/test.cc
test_signinfo_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_signinfo_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_countryaccess_SOURCES = test/countryaccess.cc test/test.cc
test_countryaccess_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_countryaccess_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_graphtilebuilder_SOURCES = test/graphtilebuilder.cc test/test.cc
test_graphtilebuilder_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_graphtilebuilder_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_search_SOURCES = test/search.cc test/test.cc
test_search_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_search_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_node_search_SOURCES = test/node_search.cc test/test.cc
test_node_search_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @RAPIDJSON_CPPFLAGS@
test_node_search_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
test_matrix_DEPENDENCIES = test/data/utrecht_tiles
test_matrix_SOURCES = test/matrix.cc test/test.cc
test_matrix_CPPFLAGS = $(DEPS_CFLAGS) $(DATA_DEPS_CFLAGS) $(SERVICE_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_matrix_LDADD = $(DEPS_LIBS) $(DATA_DEPS_LIBS) $(SERVICE_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_LIBS) libvalhalla.la
endif

TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
