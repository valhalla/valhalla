ACLOCAL_AMFLAGS = -I m4
AM_LDFLAGS = @BOOST_LDFLAGS@ @LUA_LIB@ @COVERAGE_LDFLAGS@
AM_CPPFLAGS = -Iinclude -Iinclude/proto @BOOST_CPPFLAGS@
AM_CXXFLAGS = -Iinclude -Iinclude/proto @COVERAGE_CXXFLAGS@ @LUA_INCLUDE@
if HAVE_SQLITE3
AM_CPPFLAGS += @SQLITE3_CFLAGS@
endif
LIBTOOL_DEPS = @LIBTOOL_DEPS@
libtool: $(LIBTOOL_DEPS)
	$(SHELL) ./config.status libtool

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

clean-local: clean-coverage clean-gcno
endif

# proto generation etc
PROTO_FILES = proto/trippath.proto proto/osmformat.proto proto/fileformat.proto
BUILT_SOURCES = $(patsubst %.proto,src/%.pb.cc,$(PROTO_FILES))
CLEANFILES = $(patsubst %.proto,include/%.pb.h,$(PROTO_FILES)) $(patsubst %.proto,src/%.pb.cc,$(PROTO_FILES))
nodist_libvalhalla_la_SOURCES = $(patsubst %.proto,src/%.pb.cc,$(PROTO_FILES))
src/proto/%.pb.cc: proto/%.proto
	@echo " PROTOC $<"; mkdir -p src/proto include/proto; @PROTOC_BIN@ -Iproto --cpp_out=include/proto $< && mv include/proto/$(@F) src/proto

# lib valhalla compilation etc
lib_LTLIBRARIES = libvalhalla.la
include_HEADERS = \
	include/baldr/directededge.h \
	include/baldr/edgeinfo.h \
	include/baldr/graphid.h \
	include/baldr/graphreader.h \
	include/baldr/graphtile.h \
	include/baldr/graphtileheader.h \
	include/baldr/nodeinfo.h \
	include/baldr/location.h \
	include/baldr/pathlocation.h \
	include/geo/linesegment2.h \
	include/geo/tiles.h \
	include/geo/aabbll.h \
	include/geo/polyline2.h \
	include/geo/obb2.h \
	include/geo/pointll.h \
	include/geo/vector2.h \
	include/geo/clipper2.h \
	include/geo/aabb2.h \
	include/geo/point2.h \
	include/geo/util.h \
	include/geo/distanceapproximator.h \
	include/geo/ellipse.h \
	include/geo/tilehierarchy.h \
	include/thor/adjacencylist.h \
	include/thor/astarheuristic.h \
	include/thor/edgecost.h \
	include/thor/edgelabel.h \
	include/thor/edgestatus.h \
	include/thor/pathalgorithm.h \
	include/thor/trippathbuilder.h \
	include/odin/narrativebuilder.h \
	include/mjolnir/directededgebuilder.h \
	include/mjolnir/graphtilebuilder.h \
	include/mjolnir/graphtileheaderbuilder.h \
	include/mjolnir/nodeinfobuilder.h \
	include/mjolnir/edgeinfobuilder.h 
libvalhalla_la_SOURCES = \
	src/baldr/directededge.cc \
	src/baldr/edgeinfo.cc \
	src/baldr/graphid.cc \
	src/baldr/graphreader.cc \
	src/baldr/graphtile.cc \
	src/baldr/graphtileheader.cc \
	src/baldr/nodeinfo.cc \
	src/baldr/location.cc \
	src/baldr/pathlocation.cc \
	src/geo/linesegment2.cc \
	src/geo/tiles.cc \
	src/geo/aabbll.cc \
	src/geo/polyline2.cc \
	src/geo/obb2.cc \
	src/geo/pointll.cc \
	src/geo/vector2.cc \
	src/geo/clipper2.cc \
	src/geo/aabb2.cc \
	src/geo/point2.cc \
	src/geo/util.cc \
	src/geo/distanceapproximator.cc \
	src/geo/ellipse.cc \
	src/geo/tilehierarchy.cc \
	src/thor/adjacencylist.cc \
	src/thor/astarheuristic.cc \
	src/thor/edgecost.cc \
	src/thor/edgelabel.cc \
	src/thor/edgestatus.cc \
	src/thor/pathalgorithm.cc \
	src/thor/trippathbuilder.cc \
	src/odin/narrativebuilder.cc \
	src/mjolnir/directededgebuilder.cc \
	src/mjolnir/graphtilebuilder.cc \
	src/mjolnir/graphtileheaderbuilder.cc \
	src/mjolnir/nodeinfobuilder.cc \
	src/mjolnir/edgeinfobuilder.cc \
	src/proto/fileformat.pb.cc \
	src/proto/osmformat.pb.cc \
	src/proto/trippath.pb.cc
libvalhalla_la_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
libvalhalla_la_LIBADD = $(DEPS_LIBS) @PROTOC_LIBS@ @BOOST_PROGRAM_OPTIONS_LIB@ @BOOST_FILESYSTEM_LIB@
if HAVE_SQLITE3
libvalhalla_la_LIBADD += @SQLITE3_LDFLAGS@
endif

#distributed executables
bin_PROGRAMS = pathtest pbfgraphbuilder
pathtest_SOURCES = \
	src/thor/pathtest/pathtest.cc
pathtest_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
pathtest_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ @BOOST_PROGRAM_OPTIONS_LIB@ \
        -lz libvalhalla.la
pbfgraphbuilder_SOURCES = \
	src/mjolnir/pbfgraphbuilder/pbfgraphbuilder.cc \
	src/mjolnir/pbfgraphbuilder/graphbuilder.cc \
	src/mjolnir/pbfgraphbuilder/luatagtransform.cc
pbfgraphbuilder_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
pbfgraphbuilder_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ @BOOST_PROGRAM_OPTIONS_LIB@ @BOOST_SYSTEM_LIB@ @BOOST_FILESYSTEM_LIB@ -lz libvalhalla.la

# tests
check_PROGRAMS = \
	test/point2 \
	test/distanceapproximator \
	test/aabb2 \
	test/linesegment2 \
	test/edgestatus \
	test/aabbll \
	test/vector2 \
	test/polyline2 \
	test/adjacencylist \
	test/pointll \
	test/location \
	test/tilehierarchy \
	test/graphid
test_point2_SOURCES = test/point2.cc test/test.cc
test_point2_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_point2_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla.la
test_distanceapproximator_SOURCES = test/distanceapproximator.cc test/test.cc
test_distanceapproximator_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_distanceapproximator_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla.la
test_aabb2_SOURCES = test/aabb2.cc test/test.cc
test_aabb2_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_aabb2_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla.la
test_linesegment2_SOURCES = test/linesegment2.cc test/test.cc
test_linesegment2_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_linesegment2_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla.la
test_edgestatus_SOURCES = test/edgestatus.cc test/test.cc
test_edgestatus_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_edgestatus_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla.la
test_aabbll_SOURCES = test/aabbll.cc test/test.cc
test_aabbll_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_aabbll_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla.la
test_vector2_SOURCES = test/vector2.cc test/test.cc
test_vector2_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_vector2_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla.la
test_polyline2_SOURCES = test/polyline2.cc test/test.cc
test_polyline2_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_polyline2_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla.la
test_adjacencylist_SOURCES = test/adjacencylist.cc test/test.cc
test_adjacencylist_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_adjacencylist_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla.la
test_pointll_SOURCES = test/pointll.cc test/test.cc
test_pointll_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_pointll_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla.la
test_location_SOURCES = test/location.cc test/test.cc
test_location_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_location_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla.la
test_tilehierarchy_SOURCES = test/tilehierarchy.cc test/test.cc
test_tilehierarchy_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_tilehierarchy_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla.la
test_graphid_SOURCES = test/graphid.cc test/test.cc
test_graphid_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_graphid_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla.la

TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
