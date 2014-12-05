ACLOCAL_AMFLAGS = -I m4
AM_LDFLAGS = @BOOST_LDFLAGS@ @COVERAGE_LDFLAGS@
AM_CPPFLAGS = \
	-Iinclude \
	@BOOST_CPPFLAGS@
AM_CXXFLAGS = @COVERAGE_CXXFLAGS@

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

# proto compilation etc
PROTO_FILES = proto/stub_message.proto proto/trippath.proto
BUILT_SOURCES = $(patsubst %.proto,include/%.pb.h,$(PROTO_FILES)) $(patsubst %.proto,src/%.pb.cc,$(PROTO_FILES))
CLEANFILES = $(patsubst %.proto,include/%.pb.h,$(PROTO_FILES)) $(patsubst %.proto,src/%.pb.cc,$(PROTO_FILES))
nodist_libvalhalla_la_SOURCES = $(patsubst %.proto,src/%.pb.cc,$(PROTO_FILES))
#src/proto/%.pb.cc: include/proto/%.pb.h
#include/proto/%.pb.h: proto/%.proto
src/proto/stub_message.pb.cc: include/proto/stub_message.pb.h
include/proto/stub_message.pb.h: proto/stub_message.proto
	@echo " PROTOC $<"; mkdir -p src/proto include/proto; @PROTOC_BIN@ -Iproto --cpp_out=src/proto $< && mv src/proto/$(@F) include/proto
src/proto/trippath.pb.cc: include/proto/trippath.pb.h
include/proto/trippath.pb.h: proto/trippath.proto
	@echo " PROTOC $<"; mkdir -p src/proto include/proto; @PROTOC_BIN@ -Iproto --cpp_out=src/proto $< && mv src/proto/$(@F) include/proto

# lib valhalla compilation etc
lib_LTLIBRARIES = libvalhalla.la
include_HEADERS = \
	include/valhalla.h \
	include/baldr/directededge.h \
	include/baldr/graphid.h \
	include/baldr/nodeinfo.h \
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
	include/thor/adjacencylist.h \
	include/thor/astarheuristic.h \
	include/thor/edgecost.h \
	include/thor/edgelabel.h \
	include/thor/edgestatus.h \
	include/odin/narrativebuilder.h
libvalhalla_la_SOURCES = \
	src/valhalla.cc \
	src/baldr/directededge.cc \
	src/baldr/graphid.cc \
	src/baldr/nodeinfo.cc \
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
	src/thor/adjacencylist.cc \
	src/thor/astarheuristic.cc \
	src/thor/edgecost.cc \
	src/thor/edgelabel.cc \
	src/thor/edgestatus.cc \
	src/odin/narrativebuilder.cc
libvalhalla_la_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
libvalhalla_la_LIBADD = $(DEPS_LIBS) @PROTOC_LIBS@ @BOOST_PROGRAM_OPTIONS_LIB@
if HAVE_SQLITE3
libvalhalla_la_LIBADD += @SQLITE3_LDFLAGS@
endif

# executable valhalla
bin_PROGRAMS = valhalla pbfgraphbuilder
lib_LTLIBRARIES = libvalhalla.la

valhalla_SOURCES = \
	src/valhalla.cc
valhalla_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
valhalla_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ @BOOST_PROGRAM_OPTIONS_LIB@

pbfgraphbuilder_SOURCES = \
	src/mjolnir/pbfgraphbuilder/pbfgraphbuilder.cc \
	src/mjolnir/pbfgraphbuilder/graphbuilder.cc
pbfgraphbuilder_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
pbfgraphbuilder_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ @BOOST_PROGRAM_OPTIONS_LIB@ \
	-losmpbf -lprotobuf -lz -lpthread libvalhalla.la

# tests
check_PROGRAMS = \
	test/valhalla \
	test/point2 \
	test/distanceapproximator \
	test/aabb2 \
	test/linesegment2 \
	test/edgestatus \
	test/aabbll
test_valhalla_SOURCES = test/valhalla.cc test/test.cc
test_valhalla_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_valhalla_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@
test_point2_SOURCES = test/point2.cc test/test.cc
test_point2_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_point2_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ -lvalhalla
test_distanceapproximator_SOURCES = test/distanceapproximator.cc test/test.cc
test_distanceapproximator_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_distanceapproximator_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ -lvalhalla
test_aabb2_SOURCES = test/aabb2.cc test/test.cc
test_aabb2_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_aabb2_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ -lvalhalla
test_linesegment2_SOURCES = test/linesegment2.cc test/test.cc
test_linesegment2_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_linesegment2_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ -lvalhalla
test_edgestatus_SOURCES = test/edgestatus.cc test/test.cc
test_edgestatus_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_edgestatus_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ -lvalhalla
test_aabbll_SOURCES = test/aabbll.cc test/test.cc
test_aabbll_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_aabbll_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ -lvalhalla


TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
