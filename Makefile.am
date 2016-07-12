ACLOCAL_AMFLAGS = -I m4
AM_LDFLAGS = @COVERAGE_LDFLAGS@
AM_CPPFLAGS = -Ivalhalla
AM_CXXFLAGS = -Ivalhalla @COVERAGE_CXXFLAGS@ 
LIBTOOL_DEPS = @LIBTOOL_DEPS@
libtool: $(LIBTOOL_DEPS)
	$(SHELL) ./config.status libtool

# things for versioning
pkgconfig_DATA = libvalhalla_midgard.pc
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

# libvalhalla_midgard compilation etc
lib_LTLIBRARIES = libvalhalla_midgard.la
nobase_include_HEADERS = \
	valhalla/midgard/linesegment2.h \
	valhalla/midgard/tiles.h \
	valhalla/midgard/gridded_data.h \
	valhalla/midgard/polyline2.h \
	valhalla/midgard/obb2.h \
	valhalla/midgard/pointll.h \
	valhalla/midgard/vector2.h \
	valhalla/midgard/constants.h \
	valhalla/midgard/aabb2.h \
	valhalla/midgard/point2.h \
	valhalla/midgard/util.h \
	valhalla/midgard/distanceapproximator.h \
	valhalla/midgard/ellipse.h \
	valhalla/midgard/sequence.h \
	valhalla/midgard/logging.h
libvalhalla_midgard_la_SOURCES = \
	src/midgard/linesegment2.cc \
	src/midgard/tiles.cc \
	src/midgard/gridded_data.cc \
	src/midgard/polyline2.cc \
	src/midgard/obb2.cc \
	src/midgard/pointll.cc \
	src/midgard/vector2.cc \
	src/midgard/aabb2.cc \
	src/midgard/point2.cc \
	src/midgard/util.cc \
	src/midgard/distanceapproximator.cc \
	src/midgard/ellipse.cc \
	src/midgard/logging.cc
libvalhalla_midgard_la_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
libvalhalla_midgard_la_LIBADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS)

# tests
check_PROGRAMS = \
	test/logging \
	test/point2 \
	test/distanceapproximator \
	test/aabb2 \
	test/obb2 \
	test/linesegment2 \
	test/vector2 \
	test/polyline2 \
	test/pointll \
	test/ellipse \
	test/encode \
	test/tiles \
	test/sequence \
	test/util_midgard \
	test/gridded_data
test_point2_SOURCES = test/point2.cc test/test.cc
test_point2_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_point2_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_distanceapproximator_SOURCES = test/distanceapproximator.cc test/test.cc
test_distanceapproximator_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_distanceapproximator_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_aabb2_SOURCES = test/aabb2.cc test/test.cc
test_aabb2_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_aabb2_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_obb2_SOURCES = test/obb2.cc test/test.cc
test_obb2_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_obb2_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_linesegment2_SOURCES = test/linesegment2.cc test/test.cc
test_linesegment2_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_linesegment2_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_vector2_SOURCES = test/vector2.cc test/test.cc
test_vector2_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_vector2_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_polyline2_SOURCES = test/polyline2.cc test/test.cc
test_polyline2_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_polyline2_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_pointll_SOURCES = test/pointll.cc test/test.cc
test_pointll_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_pointll_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_ellipse_SOURCES = test/ellipse.cc test/test.cc
test_ellipse_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_ellipse_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_encode_SOURCES = test/encode.cc test/test.cc
test_encode_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_encode_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_tiles_SOURCES = test/tiles.cc test/test.cc
test_tiles_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_tiles_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_util_midgard_SOURCES = test/util_midgard.cc test/test.cc
test_util_midgard_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_util_midgard_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_logging_SOURCES = test/logging.cc test/test.cc
test_logging_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) -DLOGGING_LEVEL_ALL
test_logging_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_logging_LDFLAGS = -pthread
test_sequence_SOURCES = test/sequence.cc test/test.cc
test_sequence_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS)
test_sequence_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la
test_gridded_data_SOURCES = test/gridded_data.cc test/test.cc
test_gridded_data_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) 
test_gridded_data_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) libvalhalla_midgard.la

TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
