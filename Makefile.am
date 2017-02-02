ACLOCAL_AMFLAGS = -I m4
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
AM_LDFLAGS = @COVERAGE_LDFLAGS@ @PTHREAD_LIBS@ @PTHREAD_CFLAGS@
AM_CPPFLAGS = -I@abs_srcdir@/valhalla @PTHREAD_CFLAGS@ -Igenfiles
=======
AM_LDFLAGS = @BOOST_LDFLAGS@ @COVERAGE_LDFLAGS@
AM_CPPFLAGS = -I@abs_srcdir@/valhalla @BOOST_CPPFLAGS@ -Igenfiles
>>>>>>> baldr/master
=======
AM_LDFLAGS = @BOOST_LDFLAGS@ @COVERAGE_LDFLAGS@
AM_CPPFLAGS = -I@abs_srcdir@/valhalla @BOOST_CPPFLAGS@ -Igenfiles
>>>>>>> sif/master
AM_CXXFLAGS = -I@abs_srcdir@/valhalla @COVERAGE_CXXFLAGS@ -Igenfiles
=======

# At most systems, PTHREAD_CFLAGS will give -pthread which is actually
# a builtin flag of gcc (see man gcc). It seems to be ok without this
# flag. Since both c++ and c compilers accepts this flag, so we append
# it to AM_CPPFLAGS
AM_CPPFLAGS = @BOOST_CPPFLAGS@ @PTHREAD_CFLAGS@ -Igenfiles
AM_CXXFLAGS = @COVERAGE_CXXFLAGS@ -Igenfiles

# PTHREAD_LIBS should give -lpthread or -pthread however it also could
# be empty for unknown reason. Since it is required by the linker, so
# we append PTHREAD_CFLAGS as a workaround

# BTW. the order matters as linkers read libraries from left to right
AM_LDFLAGS = @BOOST_LDFLAGS@ @COVERAGE_LDFLAGS@ @PTHREAD_LIBS@ @PTHREAD_CFLAGS@

>>>>>>> meili/master
=======
AM_LDFLAGS = @BOOST_LDFLAGS@ @COVERAGE_LDFLAGS@ @PTHREAD_LIBS@ @PTHREAD_CFLAGS@
AM_CPPFLAGS = -I@abs_srcdir@/valhalla @BOOST_CPPFLAGS@ @PTHREAD_CFLAGS@ -Igenfiles
AM_CXXFLAGS = -I@abs_srcdir@/valhalla @COVERAGE_CXXFLAGS@ -Igenfiles
>>>>>>> skadi/master
LIBTOOL_DEPS = @LIBTOOL_DEPS@
libtool: $(LIBTOOL_DEPS)
	$(SHELL) ./config.status libtool

# things for versioning
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
pkgconfig_DATA = libvalhalla_midgard.pc
=======
pkgconfig_DATA = libvalhalla_baldr.pc
>>>>>>> baldr/master
=======
pkgconfig_DATA = libvalhalla_sif.pc
>>>>>>> sif/master
=======
pkgconfig_DATA = libvalhalla_meili.pc
>>>>>>> meili/master
=======
pkgconfig_DATA = libvalhalla_skadi.pc
>>>>>>> skadi/master
EXTRA_DIST = version.sh

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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> sif/master
=======

>>>>>>> meili/master
=======

>>>>>>> skadi/master
.PHONY: clean-gcno
clean-gcno:
	-find -name '*.gcno' -exec rm -rf {} \;

clean-local: clean-coverage clean-gcno clean-genfiles
else
clean-local: clean-genfiles
endif

clean-genfiles:
	-rm -rf genfiles

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
# libvalhalla_midgard compilation etc
lib_LTLIBRARIES = libvalhalla_midgard.la
nobase_include_HEADERS = \
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
	valhalla/midgard/logging.h
libvalhalla_midgard_la_SOURCES = \
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
	src/midgard/distanceapproximator.cc \
	src/midgard/ellipse.cc \
	src/midgard/logging.cc
libvalhalla_midgard_la_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
libvalhalla_midgard_la_LIBADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS)

# tests
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
	test/gridded_data
test_point2_SOURCES = test/point2.cc test/test.cc
test_point2_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_point2_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_distanceapproximator_SOURCES = test/distanceapproximator.cc test/test.cc
test_distanceapproximator_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_distanceapproximator_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_aabb2_SOURCES = test/aabb2.cc test/test.cc
test_aabb2_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_aabb2_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_obb2_SOURCES = test/obb2.cc test/test.cc
test_obb2_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_obb2_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_linesegment2_SOURCES = test/linesegment2.cc test/test.cc
test_linesegment2_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_linesegment2_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_vector2_SOURCES = test/vector2.cc test/test.cc
test_vector2_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_vector2_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_polyline2_SOURCES = test/polyline2.cc test/test.cc
test_polyline2_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_polyline2_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_pointll_SOURCES = test/pointll.cc test/test.cc
test_pointll_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_pointll_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_ellipse_SOURCES = test/ellipse.cc test/test.cc
test_ellipse_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_ellipse_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_encode_SOURCES = test/encode.cc test/test.cc
test_encode_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_encode_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_tiles_SOURCES = test/tiles.cc test/test.cc
test_tiles_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_tiles_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_util_midgard_SOURCES = test/util_midgard.cc test/test.cc
test_util_midgard_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_util_midgard_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_logging_SOURCES = test/logging.cc test/test.cc
test_logging_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) -DLOGGING_LEVEL_ALL
test_logging_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_sequence_SOURCES = test/sequence.cc test/test.cc
test_sequence_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_sequence_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_gridded_data_SOURCES = test/gridded_data.cc test/test.cc
test_gridded_data_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) 
test_gridded_data_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
=======
#timezone info from boost
genfiles/date_time_zonespec.h:
	-mkdir -p @abs_builddir@/genfiles && cd @abs_srcdir@ && xxd -i -s +$$(head -n 1 date_time/zonespec.csv | wc -c | xargs) date_time/zonespec.csv > @abs_builddir@/genfiles/date_time_zonespec.h
BUILT_SOURCES = genfiles/date_time_zonespec.h
nodist_libvalhalla_baldr_la_SOURCES = genfiles/date_time_zonespec.h

# libvalhalla-baldr compilation etc
lib_LTLIBRARIES = libvalhalla_baldr.la
nobase_include_HEADERS = \
	valhalla/baldr/accessrestriction.h \
	valhalla/baldr/admin.h \
	valhalla/baldr/admininfo.h \
	valhalla/baldr/complexrestriction.h \
	valhalla/baldr/connectivity_map.h \
	valhalla/baldr/datetime.h \
	valhalla/baldr/directededge.h \
	valhalla/baldr/double_bucket_queue.h \
	valhalla/baldr/edgeinfo.h \
	valhalla/baldr/errorcode_util.h \
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
	valhalla/baldr/verbal_text_formatter.h \
	valhalla/baldr/verbal_text_formatter_us.h \
	valhalla/baldr/verbal_text_formatter_us_co.h \
	valhalla/baldr/verbal_text_formatter_us_tx.h \
	valhalla/baldr/verbal_text_formatter_factory.h \
	valhalla/baldr/reutil.h \
	valhalla/baldr/merge.h
libvalhalla_baldr_la_SOURCES = \
	src/baldr/accessrestriction.cc \
	src/baldr/admin.cc \
	src/baldr/admininfo.cc \
	src/baldr/complexrestriction.cc \
	src/baldr/connectivity_map.cc \
	src/baldr/datetime.cc \
	src/baldr/directededge.cc \
	src/baldr/double_bucket_queue.cc \
	src/baldr/edgeinfo.cc \
	src/baldr/geojson.cc \
	src/baldr/graphid.cc \
	src/baldr/graphreader.cc \
	src/baldr/graphtile.cc \
	src/baldr/graphtileheader.cc \
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
	src/baldr/verbal_text_formatter.cc \
	src/baldr/verbal_text_formatter_us.cc \
	src/baldr/verbal_text_formatter_us_co.cc \
	src/baldr/verbal_text_formatter_us_tx.cc \
	src/baldr/verbal_text_formatter_factory.cc \
	src/baldr/date_time_zonespec.h
libvalhalla_baldr_la_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
libvalhalla_baldr_la_LIBADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_SYSTEM_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_THREAD_LIB) $(BOOST_REGEX_LIB) $(BOOST_DATE_TIME_LIB)

# tests
check_PROGRAMS = \
	test/location \
	test/admin \
	test/datetime \
	test/directededge \
	test/double_bucket_queue \
	test/edgecollapser \
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
	test/verbal_text_formatter_us_tx
test_location_SOURCES = test/location.cc test/test.cc
test_location_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) 
test_location_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_baldr.la
test_double_bucket_queue_SOURCES = test/double_bucket_queue.cc test/test.cc
test_double_bucket_queue_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_double_bucket_queue_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_baldr.la
test_admin_SOURCES = test/admin.cc test/test.cc
test_admin_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_admin_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_baldr.la
test_datetime_SOURCES = test/datetime.cc test/test.cc
test_datetime_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_datetime_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_baldr.la
test_directededge_SOURCES = test/directededge.cc test/test.cc
test_directededge_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_directededge_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_baldr.la
test_edgecollapser_SOURCES = test/edgecollapser.cc test/test.cc
test_edgecollapser_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_edgecollapser_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_baldr.la
test_graphid_SOURCES = test/graphid.cc test/test.cc
test_graphid_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_graphid_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_baldr.la
test_tilehierarchy_SOURCES = test/tilehierarchy.cc test/test.cc
test_tilehierarchy_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_tilehierarchy_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_baldr.la
test_graphtile_SOURCES = test/graphtile.cc test/test.cc
test_graphtile_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_graphtile_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_baldr.la
test_nodeinfo_SOURCES = test/nodeinfo.cc test/test.cc
test_nodeinfo_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_nodeinfo_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_baldr.la
test_turn_SOURCES = test/turn.cc test/test.cc
test_turn_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_turn_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_baldr.la
test_graphreader_SOURCES = test/graphreader.cc test/test.cc
test_graphreader_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_graphreader_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_baldr.la
test_streetname_SOURCES = test/streetname.cc test/test.cc
test_streetname_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_streetname_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_baldr.la
test_streetname_us_SOURCES = test/streetname_us.cc test/test.cc
test_streetname_us_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_streetname_us_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_baldr.la
test_streetnames_SOURCES = test/streetnames.cc test/test.cc
test_streetnames_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_streetnames_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_baldr.la
test_streetnames_us_SOURCES = test/streetnames_us.cc test/test.cc
test_streetnames_us_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_streetnames_us_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_baldr.la
test_streetnames_factory_SOURCES = test/streetnames_factory.cc test/test.cc
test_streetnames_factory_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_streetnames_factory_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_baldr.la
test_json_SOURCES = test/json.cc test/test.cc
test_json_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_json_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_baldr.la
test_verbal_text_formatter_SOURCES = test/verbal_text_formatter.cc test/test.cc
test_verbal_text_formatter_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_verbal_text_formatter_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_baldr.la @BOOST_LDFLAGS@
test_verbal_text_formatter_us_SOURCES = test/verbal_text_formatter_us.cc test/test.cc
test_verbal_text_formatter_us_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_verbal_text_formatter_us_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_baldr.la @BOOST_LDFLAGS@
test_verbal_text_formatter_us_co_SOURCES = test/verbal_text_formatter_us_co.cc test/test.cc
test_verbal_text_formatter_us_co_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_verbal_text_formatter_us_co_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_baldr.la @BOOST_LDFLAGS@
test_verbal_text_formatter_us_tx_SOURCES = test/verbal_text_formatter_us_tx.cc test/test.cc
test_verbal_text_formatter_us_tx_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_verbal_text_formatter_us_tx_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_baldr.la @BOOST_LDFLAGS@

>>>>>>> baldr/master
=======
# lib valhalla compilation etc
lib_LTLIBRARIES = libvalhalla_sif.la
nobase_include_HEADERS = \
	valhalla/sif/costconstants.h \
	valhalla/sif/costfactory.h \
	valhalla/sif/autocost.h \
	valhalla/sif/bicyclecost.h \
	valhalla/sif/pedestriancost.h \
	valhalla/sif/transitcost.h \
	valhalla/sif/truckcost.h \
	valhalla/sif/dynamiccost.h \
	valhalla/sif/hierarchylimits.h \
	valhalla/sif/edgelabel.h
libvalhalla_sif_la_SOURCES = \
	src/sif/autocost.cc \
	src/sif/bicyclecost.cc \
	src/sif/pedestriancost.cc \
	src/sif/transitcost.cc \
	src/sif/truckcost.cc \
	src/sif/dynamiccost.cc \
	src/sif/edgelabel.cc
libvalhalla_sif_la_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
libvalhalla_sif_la_LIBADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB)

# tests
check_PROGRAMS = \
	test/factory
test_factory_SOURCES = test/factory.cc test/test.cc
test_factory_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_factory_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_sif.la
>>>>>>> sif/master
=======
# libvalhalla_meili compilation etc
lib_LTLIBRARIES = libvalhalla_meili.la
nobase_include_HEADERS = \
	valhalla/meili/universal_cost.h \
	valhalla/meili/candidate_search.h \
	valhalla/meili/geometry_helpers.h \
	valhalla/meili/graph_helpers.h \
	valhalla/meili/grid_range_query.h \
	valhalla/meili/map_matching.h \
	valhalla/meili/priority_queue.h \
	valhalla/meili/service.h \
	valhalla/meili/routing.h \
	valhalla/meili/viterbi_search.h \
	valhalla/meili/geojson_reader.h \
	valhalla/meili/geojson_writer.h \
	valhalla/meili/measurement.h \
	valhalla/meili/match_result.h \
	valhalla/meili/bucket_queue.h \
	valhalla/meili/grid_traversal.h \
	valhalla/meili/map_matcher.h \
	valhalla/meili/map_matcher_factory.h \
	valhalla/meili/traffic_segment_matcher.h \
	valhalla/meili/match_route.h
libvalhalla_meili_la_SOURCES = \
	src/meili/universal_cost.cc \
	src/meili/routing.cc \
	src/meili/candidate_search.cc \
	src/meili/map_matching.cc \
	src/meili/service.cc \
	src/meili/geojson_reader.cc \
	src/meili/map_matcher.cc \
	src/meili/map_matcher_factory.cc \
	src/meili/match_route.cc \
	src/meili/traffic_segment_matcher.cc
libvalhalla_meili_la_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
libvalhalla_meili_la_LIBADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB)

#distributed executables
bin_PROGRAMS = valhalla_map_match_service valhalla_run_map_match valhalla_meili_worker

valhalla_map_match_service_SOURCES = src/meili/valhalla_map_match_service.cc
valhalla_map_match_service_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_map_match_service_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) libvalhalla_meili.la

valhalla_run_map_match_SOURCES = src/meili/valhalla_run_map_match.cc
valhalla_run_map_match_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_run_map_match_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) libvalhalla_meili.la

valhalla_meili_worker_SOURCES = src/meili/valhalla_meili_worker.cc
valhalla_meili_worker_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_meili_worker_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) libvalhalla_meili.la

# tests
check_PROGRAMS = \
	test/geometry_helpers \
	test/grid_range_query \
	test/grid_traversal \
	test/map_matcher_factory \
	test/queue \
	test/routing \
	test/viterbi_search \
	test/geojson_reader \
	test/geojson_writer

test_geometry_helpers_SOURCES = test/geometry_helpers.cc test/test.cc
test_geometry_helpers_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_geometry_helpers_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_meili.la

test_grid_traversal_SOURCES = test/grid_traversal.cc test/test.cc
test_grid_traversal_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_grid_traversal_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_meili.la

test_grid_range_query_SOURCES = test/grid_range_query.cc test/test.cc
test_grid_range_query_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_grid_range_query_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_meili.la

test_map_matcher_factory_SOURCES = test/map_matcher_factory.cc test/test.cc
test_map_matcher_factory_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_map_matcher_factory_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_meili.la

test_queue_SOURCES = test/queue.cc test/test.cc
test_queue_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_queue_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_meili.la

test_routing_SOURCES = test/routing.cc test/test.cc
test_routing_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_routing_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_meili.la

test_viterbi_search_SOURCES = test/viterbi_search.cc test/test.cc
test_viterbi_search_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_viterbi_search_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_meili.la

test_geojson_reader_SOURCES = test/geojson_reader.cc test/test.cc
test_geojson_reader_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_geojson_reader_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_meili.la

test_geojson_writer_SOURCES = test/geojson_writer.cc test/test.cc
test_geojson_writer_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_geojson_writer_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_meili.la
>>>>>>> meili/master
=======
# lib valhalla compilation etc
lib_LTLIBRARIES = libvalhalla_skadi.la
nobase_include_HEADERS = \
	valhalla/skadi/service.h \
	valhalla/skadi/sample.h \
	valhalla/skadi/util.h
libvalhalla_skadi_la_SOURCES = \
	src/skadi/service.cc \
	src/skadi/sample.cc \
	src/skadi/util.cc
libvalhalla_skadi_la_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
libvalhalla_skadi_la_LIBADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) $(BOOST_REGEX_LIB)

#distributed executables
bin_SCRIPTS = scripts/valhalla_build_elevation

# tests
check_PROGRAMS = test/util_skadi test/sample test/skadi_service
test_util_skadi_SOURCES = test/util_skadi.cc test/test.cc
test_util_skadi_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_util_skadi_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) libvalhalla_skadi.la
test_sample_SOURCES = test/sample.cc test/test.cc
test_sample_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_sample_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) libvalhalla_skadi.la
test_skadi_service_SOURCES = test/skadi_service.cc test/test.cc
test_skadi_service_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_skadi_service_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) libvalhalla_skadi.la
>>>>>>> skadi/master

TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
