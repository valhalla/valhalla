ACLOCAL_AMFLAGS = -I m4
AM_LDFLAGS = @BOOST_LDFLAGS@ @LUA_LIB@ @COVERAGE_LDFLAGS@
AM_CPPFLAGS = -Ivalhalla -Ivalhalla/proto @BOOST_CPPFLAGS@
AM_CXXFLAGS = -Ivalhalla -Ivalhalla/proto @COVERAGE_CXXFLAGS@ @LUA_INCLUDE@
VALHALLA_LDFLAGS = @VALHALLA_MIDGARD_LDFLAGS@ @VALHALLA_MIDGARD_LIB@ @VALHALLA_BALDR_LDFLAGS@ @VALHALLA_BALDR_LIB@ @VALHALLA_SKADI_LDFLAGS@ @VALHALLA_SKADI_LIB@
VALHALLA_CPPFLAGS = @VALHALLA_MIDGARD_CPPFLAGS@ @VALHALLA_BALDR_CPPFLAGS@ @VALHALLA_SKADI_CPPFLAGS@
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
PROTO_FILES = proto/osmformat.proto proto/fileformat.proto proto/transit.proto
src/proto/%.pb.cc: proto/%.proto
	@echo " PROTOC $<"; mkdir -p src/proto valhalla/proto; @PROTOC_BIN@ -Iproto --cpp_out=valhalla/proto $< && mv valhalla/proto/$(@F) src/proto

#lua code for parsing
src/mjolnir/graph_lua_proc.h:
	-xxd -i lua/graph.lua > src/mjolnir/graph_lua_proc.h
src/mjolnir/admin_lua_proc.h:
	-xxd -i lua/admin.lua > src/mjolnir/admin_lua_proc.h

BUILT_SOURCES = $(patsubst %.proto,src/%.pb.cc,$(PROTO_FILES)) src/mjolnir/graph_lua_proc.h src/mjolnir/admin_lua_proc.h
nodist_libvalhalla_mjolnir_la_SOURCES = $(patsubst %.proto,src/%.pb.cc,$(PROTO_FILES)) src/mjolnir/graph_lua_proc.h src/mjolnir/admin_lua_proc.h
CLEANFILES = $(patsubst %.proto,valhalla/%.pb.h,$(PROTO_FILES)) $(patsubst %.proto,src/%.pb.cc,$(PROTO_FILES)) src/mjolnir/graph_lua_proc.h src/mjolnir/admin_lua_proc.h

# lib valhalla compilation etc
lib_LTLIBRARIES = libvalhalla_mjolnir.la
nobase_include_HEADERS = \
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
	valhalla/mjolnir/osmadmin.h \
	valhalla/mjolnir/osmdata.h \
	valhalla/mjolnir/osmnode.h \
	valhalla/mjolnir/osmpbfparser.h \
	valhalla/mjolnir/osmaccessrestriction.h \
	valhalla/mjolnir/osmrestriction.h \
	valhalla/mjolnir/osmway.h \
	valhalla/mjolnir/pbfadminparser.h \
	valhalla/mjolnir/pbfgraphparser.h \
	valhalla/mjolnir/statistics.h \
	valhalla/mjolnir/transitbuilder.h \
	valhalla/mjolnir/util.h
libvalhalla_mjolnir_la_SOURCES = \
	src/proto/transit.pb.cc \
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
	src/mjolnir/osmadmin.cc \
	src/mjolnir/osmnode.cc \
	src/mjolnir/osmpbfparser.cc \
	src/mjolnir/osmaccessrestriction.cc \
	src/mjolnir/osmrestriction.cc \
	src/mjolnir/osmway.cc \
	src/mjolnir/pbfadminparser.cc \
	src/mjolnir/pbfgraphparser.cc \
	src/mjolnir/statistics.cc \
	src/mjolnir/transitbuilder.cc \
	src/mjolnir/util.cc \
	src/mjolnir/graph_lua_proc.h \
	src/mjolnir/admin_lua_proc.h
libvalhalla_mjolnir_la_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
libvalhalla_mjolnir_la_LIBADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) @PROTOC_LIBS@ -lsqlite3 -lspatialite

#distributed executables
bin_PROGRAMS = \
	adminbenchmark \
	connectivitymap \
	pbfgraphbuilder \
	pbfadminbuilder \
	transit_fetcher \
        transit_stop_query

adminbenchmark_SOURCES = src/mjolnir/adminbenchmark.cc
adminbenchmark_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
adminbenchmark_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) @PROTOC_LIBS@ -lz -lgeos -lsqlite3 -lspatialite libvalhalla_mjolnir.la

connectivitymap_SOURCES = src/mjolnir/connectivitymap.cc
connectivitymap_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
connectivitymap_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) libvalhalla_mjolnir.la

pbfgraphbuilder_SOURCES = src/mjolnir/pbfgraphbuilder.cc 
pbfgraphbuilder_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
pbfgraphbuilder_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) @PROTOC_LIBS@ -lz -lsqlite3 -lspatialite libvalhalla_mjolnir.la

pbfadminbuilder_SOURCES = src/mjolnir/pbfadminbuilder.cc
pbfadminbuilder_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
pbfadminbuilder_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) @PROTOC_LIBS@ -lz -lgeos -lsqlite3 -lspatialite libvalhalla_mjolnir.la

transit_fetcher_SOURCES = src/mjolnir/transit_fetcher.cc
transit_fetcher_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
transit_fetcher_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) @PROTOC_LIBS@ -lz libvalhalla_mjolnir.la

transit_stop_query_SOURCES = src/mjolnir/transit_stop_query.cc
transit_stop_query_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
transit_stop_query_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) @PROTOC_LIBS@ -lz libvalhalla_mjolnir.la

# tests
check_PROGRAMS = \
        test/utrecht\
	test/edgeinfobuilder \
	test/uniquenames \
	test/idtable \
	test/graphtilebuilder \
	test/graphbuilder \
	test/graphparser \
	test/refs \
	test/signinfo
test_utrecht_SOURCES = test/utrecht.cc test/test.cc
test_utrecht_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_utrecht_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libvalhalla_mjolnir.la
test_edgeinfobuilder_SOURCES = test/edgeinfobuilder.cc test/test.cc
test_edgeinfobuilder_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_edgeinfobuilder_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libvalhalla_mjolnir.la
test_uniquenames_SOURCES = test/uniquenames.cc test/test.cc
test_uniquenames_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_uniquenames_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libvalhalla_mjolnir.la
test_idtable_SOURCES = test/idtable.cc test/test.cc
test_idtable_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_idtable_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libvalhalla_mjolnir.la
test_graphtilebuilder_SOURCES = test/graphtilebuilder.cc test/test.cc
test_graphtilebuilder_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_graphtilebuilder_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libvalhalla_mjolnir.la
test_graphbuilder_SOURCES = test/graphbuilder.cc test/test.cc
test_graphbuilder_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_graphbuilder_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libvalhalla_mjolnir.la
test_refs_SOURCES = test/refs.cc test/test.cc
test_graphparser_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_graphparser_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libvalhalla_mjolnir.la
test_graphparser_SOURCES = test/graphparser.cc test/test.cc
test_refs_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_refs_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libvalhalla_mjolnir.la
test_signinfo_SOURCES = test/signinfo.cc test/test.cc
test_signinfo_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_signinfo_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libvalhalla_mjolnir.la


TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
