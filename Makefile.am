ACLOCAL_AMFLAGS = -I m4
AM_LDFLAGS = @BOOST_LDFLAGS@ @COVERAGE_LDFLAGS@
AM_CPPFLAGS = @BOOST_CPPFLAGS@
AM_CXXFLAGS = @COVERAGE_CXXFLAGS@
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

libmmpincludedir=$(includedir)/mmp

# libmmp compilation etc
lib_LTLIBRARIES = libmmp.la
libmmpinclude_HEADERS = \
	include/mmp/candidate.h \
	include/mmp/universal_cost.h \
	include/mmp/candidate_search.h \
	include/mmp/geometry_helpers.h \
	include/mmp/graph_helpers.h \
	include/mmp/grid_range_query.h \
	include/mmp/map_matching.h \
	include/mmp/priority_queue.h \
	include/mmp/service.h \
	include/mmp/routing.h \
	include/mmp/viterbi_search.h \
	include/mmp/geojson_reader.h \
	include/mmp/geojson_writer.h
libmmp_la_SOURCES = \
	src/universal_cost.cc \
	src/routing.cc \
	src/candidate_search.cc \
	src/map_matching.cc \
	src/service.cc \
	src/geojson_reader.cc
libmmp_la_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
libmmp_la_LIBADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB)

#distributed executables
bin_PROGRAMS = mmp_service

mmp_service_SOURCES = src/mmp_service.cc
mmp_service_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
mmp_service_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) -lz libmmp.la

EXTRA_PROGRAMS = simple_matcher
simple_matcher_SOURCES = tools/simple_matcher.cc
simple_matcher_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
simple_matcher_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) -lz libmmp.la

.PHONY: tools
tools: simple_matcher

CLEANFILES = $(EXTRA_PROGRAMS)

# tests
check_PROGRAMS = \
	test/geometry_helpers \
	test/grid_range_query \
	test/map_matching \
	test/queue \
	test/routing \
	test/viterbi_search

test_geometry_helpers_SOURCES = test/geometry_helpers.cc
test_geometry_helpers_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_geometry_helpers_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libmmp.la

test_grid_range_query_SOURCES = test/grid_range_query.cc
test_grid_range_query_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_grid_range_query_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libmmp.la

test_map_matching_SOURCES = test/map_matching.cc
test_map_matching_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_map_matching_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libmmp.la

test_queue_SOURCES = test/queue.cc
test_queue_CPPFLAGS = $(DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_queue_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libmmp.la

test_routing_SOURCES = test/routing.cc
test_routing_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_routing_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libmmp.la

test_viterbi_search_SOURCES = test/viterbi_search.cc
test_viterbi_search_CPPFLAGS = $(DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_viterbi_search_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libmmp.la

TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
