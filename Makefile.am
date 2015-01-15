ACLOCAL_AMFLAGS = -I m4
AM_LDFLAGS = @PYTHON_LDFLAGS@ @BOOST_LDFLAGS@ @LUA_LIB@ @COVERAGE_LDFLAGS@
AM_CPPFLAGS = -Ivalhalla @BOOST_CPPFLAGS@ @PYTHON_CPPFLAGS@
AM_CXXFLAGS = -Ivalhalla @COVERAGE_CXXFLAGS@ @LUA_INCLUDE@
VALHALLA_LDFLAGS = @VALHALLA_MIDGARD_LDFLAGS@ @VALHALLA_MIDGARD_LIB@ @VALHALLA_BALDR_LDFLAGS@ @VALHALLA_BALDR_LIB@ @VALHALLA_LOKI_LDFLAGS@ @VALHALLA_LOKI_LIB@ @VALHALLA_MJOLNIR_LDFLAGS@ @VALHALLA_MJOLNIR_LIB@ @VALHALLA_ODIN_LDFLAGS@ @VALHALLA_ODIN_LIB@ @VALHALLA_THOR_LDFLAGS@ @VALHALLA_THOR_LIB@
VALHALLA_CPPFLAGS = @VALHALLA_MIDGARD_CPPFLAGS@ @VALHALLA_BALDR_CPPFLAGS@ @VALHALLA_LOKI_CPPFLAGS@ @VALHALLA_MJOLNIR_CPPFLAGS@ @VALHALLA_ODIN_CPPFLAGS@ @VALHALLA_THOR_CPPFLAGS@
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
lib_LTLIBRARIES = libvalhalla_tyr.la
nobase_include_HEADERS = \
	valhalla/tyr/json.h \
	valhalla/tyr/handler.h \
	valhalla/tyr/route_handler.h \
	valhalla/tyr/locate_handler.h \
	valhalla/tyr/nearest_handler.h

libvalhalla_tyr_la_SOURCES = \
	src/tyr/handler.cc \
	src/tyr/route_handler.cc \
	src/tyr/locate_handler.cc \
	src/tyr/nearest_handler.cc
libvalhalla_tyr_la_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@ @PYTHON_CPPFLAGS@
libvalhalla_tyr_la_LIBADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ @PYTHON_LDFLAGS@ -l$(BOOST_PYTHON_LIB)

# python bindings
pyexec_LTLIBRARIES = tyr_service.la
tyr_service_la_SOURCES = \
src/tyr/service_handlers_python.cc
tyr_service_la_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@ @PYTHON_CPPFLAGS@
tyr_service_la_LIBADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ @PYTHON_LDFLAGS@ -l$(BOOST_PYTHON_LIB) libvalhalla_tyr.la
tyr_service_la_LDFLAGS = @COVERAGE_LDFLAGS@ -module -shared

# tests
check_PROGRAMS = \
	test/handlers \
	test/json
test_handlers_SOURCES = test/handlers.cc test/test.cc
test_handlers_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@ @PYTHON_CPPFLAGS@
test_handlers_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ @PYTHON_LDFLAGS@ -l$(BOOST_PYTHON_LIB) libvalhalla_tyr.la
test_json_SOURCES = test/json.cc test/test.cc
test_json_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_json_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libvalhalla_tyr.la

TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
