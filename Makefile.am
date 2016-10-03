ACLOCAL_AMFLAGS = -I m4
AM_LDFLAGS = @BOOST_LDFLAGS@ @LUA_LIB@ @COVERAGE_LDFLAGS@
AM_CPPFLAGS = -Ivalhalla -Ivalhalla/proto @BOOST_CPPFLAGS@
AM_CXXFLAGS = -Ivalhalla -Ivalhalla/proto @COVERAGE_CXXFLAGS@ @LUA_INCLUDE@
LIBTOOL_DEPS = @LIBTOOL_DEPS@
libtool: $(LIBTOOL_DEPS)
	$(SHELL) ./config.status libtool

# things for versioning
pkgconfig_DATA = libvalhalla_mjolnir.pc
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


.PHONY: clean-gcno
clean-gcno:
	-find -name '*.gcno' -exec rm -rf {} \;

clean-local: clean-coverage clean-gcno
endif

# proto generation etc
PROTO_FILES = proto/osmformat.proto proto/fileformat.proto proto/transit.proto
src/proto/%.pb.cc: proto/%.proto
	@echo " PROTOC $<"; mkdir -p src/proto valhalla/proto; @PROTOC_BIN@ -Iproto --cpp_out=valhalla/proto $< && mv valhalla/proto/$(@F) src/proto

#lua code for parsing
src/mjolnir/graph_lua_proc.h: lua/graph.lua
	-xxd -i lua/graph.lua > src/mjolnir/graph_lua_proc.h
src/mjolnir/admin_lua_proc.h: lua/admin.lua
	-xxd -i lua/admin.lua > src/mjolnir/admin_lua_proc.h
BUILT_SOURCES = $(patsubst %.proto,src/%.pb.cc,$(PROTO_FILES)) src/mjolnir/graph_lua_proc.h src/mjolnir/admin_lua_proc.h
nodist_libvalhalla_mjolnir_la_SOURCES = $(patsubst %.proto,src/%.pb.cc,$(PROTO_FILES)) src/mjolnir/graph_lua_proc.h src/mjolnir/admin_lua_proc.h
CLEANFILES = $(patsubst %.proto,valhalla/%.pb.h,$(PROTO_FILES)) $(patsubst %.proto,src/%.pb.cc,$(PROTO_FILES)) src/mjolnir/graph_lua_proc.h src/mjolnir/admin_lua_proc.h

# lib valhalla compilation etc
lib_LTLIBRARIES = libvalhalla_mjolnir.la
nobase_include_HEADERS = \
	valhalla/mjolnir/admin.h \
	valhalla/mjolnir/countryaccess.h \
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
	valhalla/mjolnir/shortcutbuilder.h \
	valhalla/mjolnir/transitbuilder.h \
	valhalla/mjolnir/util.h
libvalhalla_mjolnir_la_SOURCES = \
	src/proto/transit.pb.cc \
	src/mjolnir/admin.cc \
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
	src/mjolnir/idtable.cc \
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
	src/mjolnir/shortcutbuilder.cc \
	src/mjolnir/transitbuilder.cc \
	src/mjolnir/util.cc \
	src/mjolnir/graph_lua_proc.h \
	src/mjolnir/admin_lua_proc.h
libvalhalla_mjolnir_la_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
libvalhalla_mjolnir_la_LIBADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) @PROTOC_LIBS@ -lsqlite3 -lspatialite

#distributed executables
bin_SCRIPTS = scripts/valhalla_build_timezones
bin_PROGRAMS = \
	valhalla_benchmark_admins \
	valhalla_build_connectivity \
	valhalla_build_tiles \
	valhalla_build_admins \
	valhalla_build_transit \
	valhalla_query_transit \
	valhalla_ways_to_edges \
	valhalla_build_speeds \
	valhalla_build_statistics

valhalla_benchmark_admins_SOURCES = src/mjolnir/valhalla_benchmark_admins.cc
valhalla_benchmark_admins_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_benchmark_admins_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) @PROTOC_LIBS@ -lz -lgeos -lsqlite3 -lspatialite libvalhalla_mjolnir.la

valhalla_build_connectivity_SOURCES = src/mjolnir/valhalla_build_connectivity.cc
valhalla_build_connectivity_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_build_connectivity_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) libvalhalla_mjolnir.la

valhalla_build_tiles_SOURCES = src/mjolnir/valhalla_build_tiles.cc 
valhalla_build_tiles_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_build_tiles_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) @PROTOC_LIBS@ -lz -lsqlite3 -lspatialite libvalhalla_mjolnir.la

valhalla_build_admins_SOURCES = src/mjolnir/valhalla_build_admins.cc
valhalla_build_admins_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_build_admins_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) @PROTOC_LIBS@ -lz -lgeos -lsqlite3 -lspatialite libvalhalla_mjolnir.la

valhalla_build_transit_SOURCES = src/mjolnir/valhalla_build_transit.cc
valhalla_build_transit_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_build_transit_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) @PROTOC_LIBS@ -lz libvalhalla_mjolnir.la

valhalla_query_transit_SOURCES = src/mjolnir/valhalla_query_transit.cc
valhalla_query_transit_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_query_transit_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) @PROTOC_LIBS@ -lz libvalhalla_mjolnir.la

valhalla_ways_to_edges_SOURCES = src/mjolnir/valhalla_ways_to_edges.cc
valhalla_ways_to_edges_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_ways_to_edges_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) @PROTOC_LIBS@ -lz libvalhalla_mjolnir.la

valhalla_build_speeds_SOURCES = src/mjolnir/valhalla_build_speeds.cc
valhalla_build_speeds_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_build_speeds_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) @PROTOC_LIBS@ -lsqlite3 libvalhalla_mjolnir.la

valhalla_build_statistics_SOURCES = src/mjolnir/valhalla_build_statistics.cc src/mjolnir/statistics.cc src/mjolnir/statistics_database.cc src/mjolnir/statistics.h
valhalla_build_statistics_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_build_statistics_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) @PROTOC_LIBS@ -lz libvalhalla_mjolnir.la

# tests
check_PROGRAMS = \
	test/countryaccess \
	test/utrecht \
	test/edgeinfobuilder \
	test/uniquenames \
	test/idtable \
	test/graphtilebuilder \
	test/graphbuilder \
	test/graphparser \
	test/names \
	test/refs \
	test/signinfo
test_countryaccess_SOURCES = test/countryaccess.cc test/test.cc
test_countryaccess_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_countryaccess_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_mjolnir.la
test_utrecht_SOURCES = test/utrecht.cc test/test.cc
test_utrecht_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_utrecht_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_mjolnir.la
test_edgeinfobuilder_SOURCES = test/edgeinfobuilder.cc test/test.cc
test_edgeinfobuilder_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_edgeinfobuilder_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_mjolnir.la
test_uniquenames_SOURCES = test/uniquenames.cc test/test.cc
test_uniquenames_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_uniquenames_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_mjolnir.la
test_idtable_SOURCES = test/idtable.cc test/test.cc
test_idtable_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_idtable_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_mjolnir.la
test_graphtilebuilder_SOURCES = test/graphtilebuilder.cc test/test.cc
test_graphtilebuilder_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_graphtilebuilder_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_mjolnir.la
test_graphbuilder_SOURCES = test/graphbuilder.cc test/test.cc
test_graphbuilder_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_graphbuilder_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_mjolnir.la
test_graphparser_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_graphparser_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_mjolnir.la
test_graphparser_SOURCES = test/graphparser.cc test/test.cc
test_names_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_names_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_mjolnir.la
test_names_SOURCES = test/names.cc test/test.cc
test_refs_SOURCES = test/refs.cc test/test.cc
test_refs_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_refs_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_mjolnir.la
test_signinfo_SOURCES = test/signinfo.cc test/test.cc
test_signinfo_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_signinfo_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_mjolnir.la


TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
