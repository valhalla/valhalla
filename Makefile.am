ACLOCAL_AMFLAGS = -I m4
AM_LDFLAGS = @BOOST_LDFLAGS@ @COVERAGE_LDFLAGS@
AM_CPPFLAGS = @BOOST_CPPFLAGS@
AM_CXXFLAGS = @COVERAGE_CXXFLAGS@
VALHALLA_LDFLAGS = @VALHALLA_MIDGARD_LDFLAGS@ @VALHALLA_MIDGARD_LIB@ @VALHALLA_BALDR_LDFLAGS@ @VALHALLA_BALDR_LIB@ @VALHALLA_SIF_LDFLAGS@ @VALHALLA_SIF_LIB@
VALHALLA_CPPFLAGS = @VALHALLA_MIDGARD_CPPFLAGS@ @VALHALLA_BALDR_CPPFLAGS@ @VALHALLA_SIF_CPPFLAGS@
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
	include/mmp/viterbi_search.h
libmmp_la_SOURCES = \
	src/universal_cost.cc \
	src/routing.cc \
	src/candidate_search.cc \
	src/map_matching.cc \
	src/service.cc
libmmp_la_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
libmmp_la_LIBADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB)

#distributed executables
bin_PROGRAMS = mmp_service

mmp_service_SOURCES = src/mmp_service.cc
mmp_service_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
mmp_service_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) -lz libmmp.la

EXTRA_PROGRAMS = simple_matcher mmp_psqlmatcher mmp_candidate_search mmp_stat mmp_attacher

simple_matcher_SOURCES = src/simple_matcher.cc
simple_matcher_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
simple_matcher_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) -lz libmmp.la

mmp_psqlmatcher_SOURCES = src/mmp_psqlmatcher.cc
mmp_psqlmatcher_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
mmp_psqlmatcher_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) -lz -lsqlite3 -lpq -lgeos libmmp.la

mmp_candidate_search_SOURCES = src/mmp_candidate_search.cc
mmp_candidate_search_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
mmp_candidate_search_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) -lz libmmp.la

mmp_stat_SOURCES = src/mmp_stat.cc
mmp_stat_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
mmp_stat_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) -lz -lsqlite3

mmp_attacher_SOURCES = src/mmp_attacher.cc
mmp_attacher_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
mmp_attacher_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) -lz libmmp.la -lsqlite3

.PHONY: tools
tools: mmp_psqlmatcher mmp_stat

.PHONY: mapillary-tools
mapillary-tools: tools mmp_attacher

# tests
check_PROGRAMS = \
	test/geometry_helpers \
	test/grid_range_query \
	test/map_matching \
	test/queue \
	test/routing \
	test/viterbi_search

test_geometry_helpers_SOURCES = test/geometry_helpers.cc
test_geometry_helpers_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_geometry_helpers_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libmmp.la

test_grid_range_query_SOURCES = test/grid_range_query.cc
test_grid_range_query_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_grid_range_query_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libmmp.la

test_map_matching_SOURCES = test/map_matching.cc
test_map_matching_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_map_matching_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libmmp.la

test_queue_SOURCES = test/queue.cc
test_queue_CPPFLAGS = $(DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_queue_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libmmp.la

test_routing_SOURCES = test/routing.cc
test_routing_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_routing_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libmmp.la

test_viterbi_search_SOURCES = test/viterbi_search.cc
test_viterbi_search_CPPFLAGS = $(DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_viterbi_search_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libmmp.la

TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
