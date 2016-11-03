ACLOCAL_AMFLAGS = -I m4
AM_LDFLAGS = @BOOST_LDFLAGS@ @COVERAGE_LDFLAGS@
AM_CPPFLAGS = -I@abs_srcdir@/valhalla @BOOST_CPPFLAGS@ -Igenfiles
AM_CXXFLAGS = -I@abs_srcdir@/valhalla @COVERAGE_CXXFLAGS@ -Igenfiles
LIBTOOL_DEPS = @LIBTOOL_DEPS@
libtool: $(LIBTOOL_DEPS)
	$(SHELL) ./config.status libtool

# things for versioning
pkgconfig_DATA = libvalhalla_sif.pc
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
lib_LTLIBRARIES = libvalhalla_sif.la
nobase_include_HEADERS = \
	valhalla/sif/costconstants.h \
	valhalla/sif/costfactory.h \
	valhalla/sif/autocost.h \
	valhalla/sif/bicyclecost.h \
	valhalla/sif/pedestriancost.h \
	valhalla/sif/transitcost.h \
	valhalla/sif/truckcost.h \
	valhalla/sif/dynamiccost.h \
	valhalla/sif/hierarchylimits.h \
	valhalla/sif/edgelabel.h
libvalhalla_sif_la_SOURCES = \
	src/sif/autocost.cc \
	src/sif/bicyclecost.cc \
	src/sif/pedestriancost.cc \
	src/sif/transitcost.cc \
	src/sif/truckcost.cc \
	src/sif/dynamiccost.cc \
	src/sif/edgelabel.cc
libvalhalla_sif_la_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
libvalhalla_sif_la_LIBADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ $(BOOST_SYSTEM_LIB) $(BOOST_THREAD_LIB)

# tests
check_PROGRAMS = \
	test/factory
test_factory_SOURCES = test/factory.cc test/test.cc
test_factory_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_DEPS_CFLAGS) @BOOST_CPPFLAGS@
test_factory_LDADD = $(DEPS_LIBS) $(VALHALLA_DEPS_LIBS) @BOOST_LDFLAGS@ libvalhalla_sif.la

TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
