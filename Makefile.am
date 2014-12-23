ACLOCAL_AMFLAGS = -I m4
AM_LDFLAGS = @BOOST_LDFLAGS@ @COVERAGE_LDFLAGS@
AM_CPPFLAGS = -Ivalhalla @BOOST_CPPFLAGS@
AM_CXXFLAGS = -Ivalhalla @COVERAGE_CXXFLAGS@
VALHALLA_LDFLAGS = @VALHALLA_MIDGARD_LDFLAGS@ @VALHALLA_MIDGARD_LIB@ @VALHALLA_BALDR_LDFLAGS@ @VALHALLA_BALDR_LIB@ @VALHALLA_ODIN_LDFLAGS@ @VALHALLA_ODIN_LIB@
VALHALLA_CPPFLAGS = @VALHALLA_MIDGARD_CPPFLAGS@ @VALHALLA_BALDR_CPPFLAGS@ @VALHALLA_ODIN_CPPFLAGS@
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
include_HEADERS = \
	valhalla/thor/adjacencylist.h \
	valhalla/thor/astarheuristic.h \
	valhalla/thor/autocost.h \
	valhalla/thor/dynamiccost.h \
	valhalla/thor/edgelabel.h \
	valhalla/thor/edgestatus.h \
	valhalla/thor/pathalgorithm.h \
	valhalla/thor/pedestriancost.h \
	valhalla/thor/trippathbuilder.h 
libvalhalla_thor_la_SOURCES = \
	src/thor/adjacencylist.cc \
	src/thor/astarheuristic.cc \
	src/thor/autocost.cc \
	src/thor/dynamiccost.cc \
	src/thor/edgelabel.cc \
	src/thor/edgestatus.cc \
	src/thor/pathalgorithm.cc \
	src/thor/pedestriancost.cc \
	src/thor/trippathbuilder.cc 
libvalhalla_thor_la_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
libvalhalla_thor_la_LIBADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_PROGRAM_OPTIONS_LIB@ @BOOST_FILESYSTEM_LIB@ @PROTOC_LIBS@

#distributed executables
bin_PROGRAMS = pathtest
pathtest_SOURCES = \
	src/thor/pathtest/pathtest.cc
pathtest_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
pathtest_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ @BOOST_PROGRAM_OPTIONS_LIB@ @BOOST_FILESYSTEM_LIB@ -lz libvalhalla_thor.la

# tests
check_PROGRAMS = \
	test/edgestatus \
	test/adjacencylist 
test_edgestatus_SOURCES = test/edgestatus.cc test/test.cc
test_edgestatus_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_edgestatus_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libvalhalla_thor.la
test_adjacencylist_SOURCES = test/adjacencylist.cc test/test.cc
test_adjacencylist_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_adjacencylist_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libvalhalla_thor.la

TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
