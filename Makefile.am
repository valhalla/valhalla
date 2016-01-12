ACLOCAL_AMFLAGS = -I m4
AM_LDFLAGS = @BOOST_LDFLAGS@ @COVERAGE_LDFLAGS@
AM_CPPFLAGS = -Ivalhalla @BOOST_CPPFLAGS@
AM_CXXFLAGS = -Ivalhalla @COVERAGE_CXXFLAGS@
VALHALLA_LDFLAGS = @VALHALLA_MIDGARD_LDFLAGS@ @VALHALLA_MIDGARD_LIB@ @VALHALLA_BALDR_LDFLAGS@ @VALHALLA_BALDR_LIB@ @VALHALLA_SIF_LDFLAGS@ @VALHALLA_SIF_LIB@ @VALHALLA_SKADI_LIB@ @VALHALLA_SKADI_LDFLAGS@ @VALHALLA_MJOLNIR_LDFLAGS@ @VALHALLA_MJOLNIR_LIB@ @VALHALLA_LOKI_LDFLAGS@ @VALHALLA_LOKI_LIB@ @VALHALLA_THOR_LDFLAGS@ @VALHALLA_THOR_LIB@ @VALHALLA_ODIN_LDFLAGS@ @VALHALLA_ODIN_LIB@ @VALHALLA_TYR_LDFLAGS@ @VALHALLA_TYR_LIB@
VALHALLA_CPPFLAGS = @VALHALLA_MIDGARD_CPPFLAGS@ @VALHALLA_BALDR_CPPFLAGS@ @VALHALLA_SIF_CPPFLAGS@ @VALHALLA_SKADI_CPPFLAGS@ @VALHALLA_MJOLNIR_CPPFLAGS@ @VALHALLA_LOKI_CPPFLAGS@ @VALHALLA_THOR_CPPFLAGS@ @VALHALLA_ODIN_CPPFLAGS@ @VALHALLA_TYR_CPPFLAGS@
LIBTOOL_DEPS = @LIBTOOL_DEPS@
libtool: $(LIBTOOL_DEPS)
	$(SHELL) ./config.status libtool

#config files for tools, you can put a commit hash here if you are ahead of master
CONF=master
conf:
	-wget -q -O conf-$(CONF).zip https://github.com/valhalla/conf/archive/$(CONF).zip
	-unzip conf-$(CONF).zip
	-rm -rf conf-$(CONF).zip conf
	-mv conf-$(CONF) conf
clean-conf:
	-rm -rf conf

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

clean-local: clean-coverage clean-gcno clean-conf
else
clean-local: clean-conf
endif

#everybody needs confs
src/*.cc: conf

#distributed executables
bin_PROGRAMS = loki_benchmark skadi_benchmark skadi_simple_service
loki_benchmark_SOURCES = src/loki_benchmark.cc
loki_benchmark_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
loki_benchmark_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB)
skadi_benchmark_SOURCES = src/skadi_benchmark.cc
skadi_benchmark_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
skadi_benchmark_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB)
skadi_simple_service_SOURCES = src/skadi_simple_service.cc
skadi_simple_service_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
skadi_simple_service_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB)

# tests
#check_PROGRAMS = test/something
#test_something_SOURCES = test/something.cc test/test.cc
#test_something_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
#test_something_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_PROGRAM_OPTIONS_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB)

TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
