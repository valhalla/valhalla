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

# libvalhalla_meili compilation etc
lib_LTLIBRARIES = libvalhalla_meili.la
nobase_include_HEADERS = \
	valhalla/meili/candidate.h \
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
	valhalla/meili/geojson_writer.h
libvalhalla_meili_la_SOURCES = \
	src/meili/universal_cost.cc \
	src/meili/routing.cc \
	src/meili/candidate_search.cc \
	src/meili/map_matching.cc \
	src/meili/service.cc \
	src/meili/geojson_reader.cc
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
	test/map_matching \
	test/queue \
	test/routing \
	test/viterbi_search \
	test/geojson_reader \
	test/geojson_writer

test_geometry_helpers_SOURCES = test/geometry_helpers.cc test/test.cc
test_geometry_helpers_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_geometry_helpers_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_meili.la

test_grid_range_query_SOURCES = test/grid_range_query.cc test/test.cc
test_grid_range_query_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_grid_range_query_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_meili.la

test_map_matching_SOURCES = test/map_matching.cc test/test.cc
test_map_matching_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_map_matching_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_meili.la

test_queue_SOURCES = test/queue.cc test/test.cc
test_queue_CPPFLAGS = $(DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_queue_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_meili.la

test_routing_SOURCES = test/routing.cc test/test.cc
test_routing_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_routing_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_meili.la

test_viterbi_search_SOURCES = test/viterbi_search.cc test/test.cc
test_viterbi_search_CPPFLAGS = $(DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_viterbi_search_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_meili.la

test_geojson_reader_SOURCES = test/geojson_reader.cc
test_geojson_reader_CPPFLAGS = $(DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_geojson_reader_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_meili.la

test_geojson_writer_SOURCES = test/geojson_writer.cc
test_geojson_writer_CPPFLAGS = $(DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_geojson_writer_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_meili.la

TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
