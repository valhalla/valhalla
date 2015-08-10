ACLOCAL_AMFLAGS = -I m4
AM_LDFLAGS = @BOOST_LDFLAGS@ @COVERAGE_LDFLAGS@
AM_CPPFLAGS = -Ivalhalla @BOOST_CPPFLAGS@
AM_CXXFLAGS = -Ivalhalla @COVERAGE_CXXFLAGS@
VALHALLA_LDFLAGS = @VALHALLA_MIDGARD_LDFLAGS@ @VALHALLA_MIDGARD_LIB@ @VALHALLA_BALDR_LDFLAGS@ @VALHALLA_BALDR_LIB@
VALHALLA_CPPFLAGS = @VALHALLA_MIDGARD_CPPFLAGS@ @VALHALLA_BALDR_CPPFLAGS@
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
lib_LTLIBRARIES = libvalhalla_skadi.la
nobase_include_HEADERS = \
	valhalla/skadi/service.h \
	valhalla/skadi/sample.h
libvalhalla_skadi_la_SOURCES = \
	src/skadi/service.cc \
	src/skadi/sample.cc
libvalhalla_skadi_la_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
libvalhalla_skadi_la_LIBADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB)

#distributed executables
bin_PROGRAMS = skadi_service skadi_benchmark skadi_simple_service
skadi_simple_service_SOURCES = src/skadi/simple_service.cc
skadi_simple_service_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
skadi_simple_service_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) libvalhalla_skadi.la
skadi_service_SOURCES = src/skadi/skadi_service.cc
skadi_service_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
skadi_service_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) libvalhalla_skadi.la
skadi_benchmark_SOURCES = src/skadi/benchmark.cc
skadi_benchmark_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
skadi_benchmark_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) libvalhalla_skadi.la

#data for tests
test/data/N40W077.hgt:
	-curl -s -o - http://s3.amazonaws.com/mapzen.valhalla/elevation/N40/N40W077.hgt.gz | gunzip > test/data/N40W077.hgt

# tests
check_PROGRAMS = test/sample test/service
test_sample_SOURCES = test/sample.cc test/test.cc
test_sample_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_sample_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) libvalhalla_skadi.la
test_sample_DEPENDENCIES = test/data/N40W077.hgt
test_service_SOURCES = test/service.cc test/test.cc
test_service_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_service_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB) libvalhalla_skadi.la
test_service_DEPENDENCIES = test/data/N40W077.hgt

TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
