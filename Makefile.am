ACLOCAL_AMFLAGS = -I m4
AM_LDFLAGS = @BOOST_LDFLAGS@ @COVERAGE_LDFLAGS@ @PTHREAD_LIBS@ @PTHREAD_CFLAGS@
AM_CPPFLAGS = -I@abs_srcdir@/valhalla @BOOST_CPPFLAGS@ @PTHREAD_CFLAGS@ -Igenfiles
AM_CXXFLAGS = -I@abs_srcdir@/valhalla @COVERAGE_CXXFLAGS@ -Igenfiles
LIBTOOL_DEPS = @LIBTOOL_DEPS@
libtool: $(LIBTOOL_DEPS)
	$(SHELL) ./config.status libtool

# things for versioning
pkgconfig_DATA = libvalhalla_loki.pc
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

clean-local: clean-coverage clean-gcno clean-genfiles
else
clean-local: clean-genfiles
endif

clean-genfiles:
	-rm -rf genfiles

# lib valhalla compilation etc
lib_LTLIBRARIES = libvalhalla_loki.la
nobase_include_HEADERS = \
	valhalla/loki/search.h \
	valhalla/loki/service.h
libvalhalla_loki_la_SOURCES = \
	src/loki/search.cc \
	src/loki/service.cc \
	src/loki/locate_action.cc \
	src/loki/route_action.cc \
	src/loki/matrix_action.cc \
	src/loki/isochrone_action.cc \
	src/loki/trace_route_action.cc
libvalhalla_loki_la_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
libvalhalla_loki_la_LIBADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB)

# tests
check_PROGRAMS = test/loki_service test/search
test_search_SOURCES = test/search.cc test/test.cc
test_search_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_search_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) libvalhalla_loki.la
test_loki_service_SOURCES = test/loki_service.cc test/test.cc
test_loki_service_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_loki_service_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) libvalhalla_loki.la

TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
