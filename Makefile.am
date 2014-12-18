ACLOCAL_AMFLAGS = -I m4
AM_LDFLAGS = @BOOST_LDFLAGS@ @COVERAGE_LDFLAGS@
AM_CPPFLAGS = -Iinclude @BOOST_CPPFLAGS@
AM_CXXFLAGS = -Iinclude @COVERAGE_CXXFLAGS@ 
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

# libvalhalla_midgard compilation etc
lib_LTLIBRARIES = libvalhalla_midgard.la
include_HEADERS = \
	include/midgard/linesegment2.h \
	include/midgard/tiles.h \
	include/midgard/aabbll.h \
	include/midgard/polyline2.h \
	include/midgard/obb2.h \
	include/midgard/pointll.h \
	include/midgard/vector2.h \
	include/midgard/clipper2.h \
	include/midgard/aabb2.h \
	include/midgard/point2.h \
	include/midgard/util.h \
	include/midgard/distanceapproximator.h \
	include/midgard/ellipse.h \
	include/midgard/tilehierarchy.h 
libvalhalla_midgard_la_SOURCES = \
	src/midgard/linesegment2.cc \
	src/midgard/tiles.cc \
	src/midgard/aabbll.cc \
	src/midgard/polyline2.cc \
	src/midgard/obb2.cc \
	src/midgard/pointll.cc \
	src/midgard/vector2.cc \
	src/midgard/clipper2.cc \
	src/midgard/aabb2.cc \
	src/midgard/point2.cc \
	src/midgard/util.cc \
	src/midgard/distanceapproximator.cc \
	src/midgard/ellipse.cc \
	src/midgard/tilehierarchy.cc 
libvalhalla_midgard_la_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
libvalhalla_midgard_la_LIBADD = $(DEPS_LIBS) 

# tests
check_PROGRAMS = \
	test/point2 \
	test/distanceapproximator \
	test/aabb2 \
	test/linesegment2 \
	test/aabbll \
	test/vector2 \
	test/polyline2 \
	test/pointll \
	test/tilehierarchy 
test_point2_SOURCES = test/point2.cc test/test.cc
test_point2_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_point2_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_midgard.la
test_distanceapproximator_SOURCES = test/distanceapproximator.cc test/test.cc
test_distanceapproximator_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_distanceapproximator_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_midgard.la
test_aabb2_SOURCES = test/aabb2.cc test/test.cc
test_aabb2_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_aabb2_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_midgard.la
test_linesegment2_SOURCES = test/linesegment2.cc test/test.cc
test_linesegment2_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_linesegment2_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_midgard.la
test_aabbll_SOURCES = test/aabbll.cc test/test.cc
test_aabbll_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_aabbll_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_midgard.la
test_vector2_SOURCES = test/vector2.cc test/test.cc
test_vector2_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_vector2_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_midgard.la
test_polyline2_SOURCES = test/polyline2.cc test/test.cc
test_polyline2_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_polyline2_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_midgard.la
test_pointll_SOURCES = test/pointll.cc test/test.cc
test_pointll_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_pointll_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_midgard.la
test_tilehierarchy_SOURCES = test/tilehierarchy.cc test/test.cc
test_tilehierarchy_CPPFLAGS = $(DEPS_CFLAGS) $(BOOST_CPPFLAGS)
test_tilehierarchy_LDADD = $(DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_midgard.la

TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
