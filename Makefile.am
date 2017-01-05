ACLOCAL_AMFLAGS = -I m4
AM_LDFLAGS = @BOOST_LDFLAGS@ @COVERAGE_LDFLAGS@ @PTHREAD_LIBS@ @PTHREAD_CFLAGS@
AM_CPPFLAGS = -I@abs_srcdir@/include -I@abs_srcdir@/include/proto @BOOST_CPPFLAGS@ -Igenfiles
AM_CXXFLAGS = -I@abs_srcdir@/include -I@abs_srcdir@/include/proto @COVERAGE_CXXFLAGS@ -Igenfiles
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

clean-local: clean-coverage clean-gcno clean-genfiles
else
clean-local: clean-genfiles
endif

clean-genfiles:
	-rm -rf genfiles

# protobuf generated code
PROTO_FILES = proto/segment.proto proto/tile.proto
src/proto/%.pb.cc: proto/%.proto
	@echo " PROTOC $<"; mkdir -p src/proto valhalla/proto; @PROTOC_BIN@ -Iproto --cpp_out=valhalla/proto $< && mv valhalla/proto/$(@F) src/proto

#distributed executables
bin_PROGRAMS = valhalla_skadi_worker \
	valhalla_loki_worker \
	valhalla_odin_worker \
	valhalla_thor_worker \
	valhalla_tyr_worker \
	valhalla_benchmark_loki \
	valhalla_benchmark_skadi \
	valhalla_elevation_service \
	valhalla_route_service \
	valhalla_run_isochrone \
	valhalla_run_route \
	valhalla_benchmark_adjacency_list \
	valhalla_run_matrix \
	valhalla_export_edges \
	valhalla_associate_segments
valhalla_skadi_worker_SOURCES = src/valhalla_skadi_worker.cc
valhalla_skadi_worker_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_skadi_worker_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB)
valhalla_loki_worker_SOURCES = src/valhalla_loki_worker.cc
valhalla_loki_worker_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_loki_worker_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB)
valhalla_odin_worker_SOURCES = src/valhalla_odin_worker.cc
valhalla_odin_worker_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_odin_worker_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB)
valhalla_thor_worker_SOURCES = src/valhalla_thor_worker.cc
valhalla_thor_worker_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_thor_worker_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB)
valhalla_tyr_worker_SOURCES = src/valhalla_tyr_worker.cc
valhalla_tyr_worker_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_tyr_worker_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB)
valhalla_benchmark_loki_SOURCES = src/valhalla_benchmark_loki.cc
valhalla_benchmark_loki_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_benchmark_loki_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB)
valhalla_benchmark_skadi_SOURCES = src/valhalla_benchmark_skadi.cc
valhalla_benchmark_skadi_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_benchmark_skadi_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB)
valhalla_elevation_service_SOURCES = src/valhalla_elevation_service.cc
valhalla_elevation_service_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_elevation_service_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB)
valhalla_route_service_SOURCES = src/valhalla_route_service.cc
valhalla_route_service_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_route_service_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB)
valhalla_run_isochrone_SOURCES = src/valhalla_run_isochrone.cc
valhalla_run_isochrone_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_run_isochrone_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB)
valhalla_run_route_SOURCES = src/valhalla_run_route.cc
valhalla_run_route_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_run_route_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB)
valhalla_benchmark_adjacency_list_SOURCES = src/valhalla_benchmark_adjacency_list.cc
valhalla_benchmark_adjacency_list_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_benchmark_adjacency_list_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB)
valhalla_run_matrix_SOURCES = src/valhalla_run_matrix.cc
valhalla_run_matrix_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_run_matrix_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB)
valhalla_export_edges_SOURCES = src/valhalla_export_edges.cc
valhalla_export_edges_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_export_edges_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB)
valhalla_associate_segments_SOURCES = src/valhalla_associate_segments.cc src/proto/segment.pb.cc src/proto/tile.pb.cc
valhalla_associate_segments_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
valhalla_associate_segments_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB)

EXTRA_PROGRAMS = city_test unconnected_ways
CLEANFILES = $(EXTRA_PROGRAMS)
city_test_SOURCES = src/city_test.cc
city_test_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
city_test_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB)
unconnected_ways_SOURCES = src/unconnected_ways.cc
unconnected_ways_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
unconnected_ways_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB)


# python bindings
pyexec_LTLIBRARIES = valhalla.la
valhalla_la_SOURCES = src/bindings/python/python.cc
valhalla_la_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@ @PYTHON_CPPFLAGS@
valhalla_la_LIBADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ @PYTHON_LIBS@ -l$(BOOST_PYTHON_LIB) $(VALHALLA_DEPS_LIBS)
valhalla_la_LDFLAGS = @COVERAGE_LDFLAGS@ -avoid-version -module -shared

# tests
#check_PROGRAMS = test/something
#test_something_SOURCES = test/something.cc test/test.cc
#test_something_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
#test_something_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB)

TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
