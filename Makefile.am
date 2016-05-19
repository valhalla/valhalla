ACLOCAL_AMFLAGS = -I m4
AM_LDFLAGS = @BOOST_LDFLAGS@ @COVERAGE_LDFLAGS@
AM_CPPFLAGS = -Ivalhalla @BOOST_CPPFLAGS@
AM_CXXFLAGS = -Ivalhalla @COVERAGE_CXXFLAGS@
LIBTOOL_DEPS = @LIBTOOL_DEPS@
libtool: $(LIBTOOL_DEPS)
	$(SHELL) ./config.status libtool

# things for versioning
pkgconfig_DATA = libvalhalla_thor.pc
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

# lib valhalla compilation etc
lib_LTLIBRARIES = libvalhalla_thor.la
nobase_include_HEADERS = \
	valhalla/thor/adjacencylist.h \
	valhalla/thor/astarheuristic.h \
	valhalla/thor/bidirectional_astar.h \
	valhalla/thor/edgestatus.h \
	valhalla/thor/optimizer.h \
	valhalla/thor/pathalgorithm.h \
	valhalla/thor/pathinfo.h \
	valhalla/thor/timedistancematrix.h \
	valhalla/thor/trippathbuilder.h \
	valhalla/thor/service.h
libvalhalla_thor_la_SOURCES = \
	src/thor/adjacencylist.cc \
	src/thor/astarheuristic.cc \
	src/thor/bidirectional_astar.cc \
	src/thor/edgestatus.cc \
	src/thor/matrix.cc \
	src/thor/multimodal.cc \
	src/thor/optimized_path.cc \
	src/thor/optimizer.cc \
	src/thor/pathalgorithm.cc \
	src/thor/timedistancematrix.cc \
	src/thor/trip_path.cc \
	src/thor/trippathbuilder.cc \
	src/thor/service.cc
libvalhalla_thor_la_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
libvalhalla_thor_la_LIBADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) @PROTOC_LIBS@

#distributed executables
bin_PROGRAMS = thor_service
thor_service_SOURCES = src/thor/thor_service.cc
thor_service_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
thor_service_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) -lz libvalhalla_thor.la

# tests
check_PROGRAMS = \
	test/edgestatus \
	test/optimizer \
	test/adjacencylist \
	test/thor_service
test_edgestatus_SOURCES = test/edgestatus.cc test/test.cc
test_edgestatus_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_edgestatus_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_thor.la
test_optimizer_SOURCES = test/optimizer.cc test/test.cc
test_optimizer_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_optimizer_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_thor.la
test_adjacencylist_SOURCES = test/adjacencylist.cc test/test.cc
test_adjacencylist_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_adjacencylist_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_thor.la
test_thor_service_SOURCES = test/thor_service.cc test/test.cc
test_thor_service_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_thor_service_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_thor.la

TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
