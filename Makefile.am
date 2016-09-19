ACLOCAL_AMFLAGS = -I m4

# At most systems, PTHREAD_CFLAGS will give -pthread which is actually
# a builtin flag of gcc (see man gcc). It seems to be ok without this
# flag. Since both c++ and c compilers accepts this flag, so we append
# it to AM_CPPFLAGS
AM_CPPFLAGS = @BOOST_CPPFLAGS@ @PTHREAD_CFLAGS@
AM_CXXFLAGS = @COVERAGE_CXXFLAGS@

# PTHREAD_LIBS should give -lpthread or -pthread however it also could
# be empty for unknown reason. Since it is required by the linker, so
# we append PTHREAD_CFLAGS as a workaround

# BTW. the order matters as linkers read libraries from left to right
AM_LDFLAGS = @BOOST_LDFLAGS@ @COVERAGE_LDFLAGS@ @PTHREAD_LIBS@ @PTHREAD_CFLAGS@

LIBTOOL_DEPS = @LIBTOOL_DEPS@
libtool: $(LIBTOOL_DEPS)
	$(SHELL) ./config.status libtool

# things for versioning
pkgconfig_DATA = libvalhalla_meili.pc
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
	src/meili/match_route.cc
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

TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
