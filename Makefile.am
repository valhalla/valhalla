ACLOCAL_AMFLAGS = -I m4
AM_LDFLAGS = @BOOST_LDFLAGS@ @COVERAGE_LDFLAGS@
AM_CPPFLAGS = -Ivalhalla @BOOST_CPPFLAGS@
AM_CXXFLAGS = -Ivalhalla @COVERAGE_CXXFLAGS@
VALHALLA_LDFLAGS = @VALHALLA_MIDGARD_LDFLAGS@ @VALHALLA_MIDGARD_LIB@
VALHALLA_CPPFLAGS = @VALHALLA_MIDGARD_CPPFLAGS@
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

# libvalhalla-baldr compilation etc
lib_LTLIBRARIES = libvalhalla_baldr.la
nobase_include_HEADERS = \
	valhalla/baldr/directededge.h \
	valhalla/baldr/edgeinfo.h \
	valhalla/baldr/exitsign.h \
        valhalla/baldr/exitsigninfo.h \
	valhalla/baldr/graphid.h \
	valhalla/baldr/graphreader.h \
	valhalla/baldr/graphtile.h \
	valhalla/baldr/graphtileheader.h \
	valhalla/baldr/nodeinfo.h \
	valhalla/baldr/location.h \
	valhalla/baldr/pathlocation.h \
	valhalla/baldr/graphconstants.h \
	valhalla/baldr/tilehierarchy.h
libvalhalla_baldr_la_SOURCES = \
	src/baldr/directededge.cc \
	src/baldr/edgeinfo.cc \
	src/baldr/exitsign.cc \
        src/baldr/exitsigninfo.cc \
	src/baldr/graphid.cc \
	src/baldr/graphreader.cc \
	src/baldr/graphtile.cc \
	src/baldr/graphtileheader.cc \
	src/baldr/nodeinfo.cc \
	src/baldr/location.cc \
	src/baldr/pathlocation.cc \
	src/baldr/tilehierarchy.cc
libvalhalla_baldr_la_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
libvalhalla_baldr_la_LIBADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@

# tests
check_PROGRAMS = \
	test/location \
	test/graphid \
	test/tilehierarchy \
	test/graphtile \
	test/nodeinfo
test_location_SOURCES = test/location.cc test/test.cc
test_location_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS)
test_location_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) libvalhalla_baldr.la
test_graphid_SOURCES = test/graphid.cc test/test.cc
test_graphid_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS)
test_graphid_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) libvalhalla_baldr.la
test_tilehierarchy_SOURCES = test/tilehierarchy.cc test/test.cc
test_tilehierarchy_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS)
test_tilehierarchy_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) libvalhalla_baldr.la
test_graphtile_SOURCES = test/graphtile.cc test/test.cc
test_graphtile_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS)
test_graphtile_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) libvalhalla_baldr.la
test_nodeinfo_SOURCES = test/nodeinfo.cc test/test.cc
test_nodeinfo_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS)
test_nodeinfo_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) libvalhalla_baldr.la


TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
