ACLOCAL_AMFLAGS = -I m4
AM_LDFLAGS = @BOOST_LDFLAGS@ @COVERAGE_LDFLAGS@
AM_CPPFLAGS = -Ivalhalla @BOOST_CPPFLAGS@
AM_CXXFLAGS = -Ivalhalla @COVERAGE_CXXFLAGS@
VALHALLA_LDFLAGS = @VALHALLA_MIDGARD_LDFLAGS@ @VALHALLA_MIDGARD_LIB@ @VALHALLA_BALDR_LDFLAGS@ @VALHALLA_BALDR_LIB@ @VALHALLA_SIF_LDFLAGS@ @VALHALLA_SIF_LIB@ @VALHALLA_LOKI_LDFLAGS@ @VALHALLA_LOKI_LIB@ @VALHALLA_ODIN_LDFLAGS@ @VALHALLA_ODIN_LIB@ 
VALHALLA_CPPFLAGS = @VALHALLA_MIDGARD_CPPFLAGS@ @VALHALLA_BALDR_CPPFLAGS@ @VALHALLA_SIF_CPPFLAGS@ @VALHALLA_LOKI_CPPFLAGS@ @VALHALLA_ODIN_CPPFLAGS@
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
	src/thor/multimodal.cc \
	src/thor/optimizer.cc \
	src/thor/pathalgorithm.cc \
	src/thor/timedistancematrix.cc \
	src/thor/trippathbuilder.cc \
	src/thor/service.cc
libvalhalla_thor_la_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
libvalhalla_thor_la_LIBADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) @PROTOC_LIBS@

#distributed executables
bin_PROGRAMS = \
	pathtest \
	citytest \
	adjlistbenchmark \
	timedistance_test \
	thor_service
pathtest_SOURCES = \
	src/thor/pathtest/pathtest.cc
pathtest_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
pathtest_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) -lz libvalhalla_thor.la
adjlistbenchmark_SOURCES = \
	src/thor/adjlistbenchmark/adjlistbenchmark.cc
adjlistbenchmark_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
adjlistbenchmark_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) -lz libvalhalla_thor.la
citytest_SOURCES = \
	src/thor/citytest/citytest.cc
citytest_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
citytest_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) -lz libvalhalla_thor.la
timedistance_test_SOURCES = \
	src/thor/timedistance_test/timedistance_test.cc
timedistance_test_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
timedistance_test_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) -lz libvalhalla_thor.la
thor_service_SOURCES = \
        src/thor/thor_service.cc
thor_service_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
thor_service_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) -lz libvalhalla_thor.la

# tests
check_PROGRAMS = \
	test/edgestatus \
	test/optimizer \ 
	test/adjacencylist 
test_edgestatus_SOURCES = test/edgestatus.cc test/test.cc
test_edgestatus_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_edgestatus_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libvalhalla_thor.la
test_optimizer_SOURCES = test/optimizer.cc test/test.cc
test_optimizer_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_optimizer_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libvalhalla_thor.la
test_adjacencylist_SOURCES = test/adjacencylist.cc test/test.cc
test_adjacencylist_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_adjacencylist_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libvalhalla_thor.la

TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
