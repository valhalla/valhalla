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

#timezone info from boost
valhalla/baldr/date_time_zonespec.h:
	-wget -q https://raw.githubusercontent.com/boostorg/date_time/master/data/date_time_zonespec.csv
	-sed -i -e '1d' date_time_zonespec.csv
	-xxd -i date_time_zonespec.csv > valhalla/baldr/date_time_zonespec.h
CLEANFILES = date_time_zonespec.csv valhalla/baldr/date_time_zonespec.h


# libvalhalla-baldr compilation etc
lib_LTLIBRARIES = libvalhalla_baldr.la
nobase_include_HEADERS = \
	valhalla/baldr/admin.h \
        valhalla/baldr/admininfo.h \
	valhalla/baldr/connectivity_map.h \
	valhalla/baldr/datetime.h \
	valhalla/baldr/directededge.h \
	valhalla/baldr/edgeinfo.h \
	valhalla/baldr/graphconstants.h \
	valhalla/baldr/graphid.h \
	valhalla/baldr/graphreader.h \
	valhalla/baldr/graphtile.h \
	valhalla/baldr/graphtileheader.h \
	valhalla/baldr/json.h \
	valhalla/baldr/nodeinfo.h \
	valhalla/baldr/location.h \
	valhalla/baldr/pathlocation.h \
	valhalla/baldr/sign.h \
        valhalla/baldr/signinfo.h \
	valhalla/baldr/srtmtile.h \
	valhalla/baldr/tilehierarchy.h \
	valhalla/baldr/turn.h \
	valhalla/baldr/streetname.h \
	valhalla/baldr/streetnames.h \
	valhalla/baldr/streetnames_factory.h \
	valhalla/baldr/streetname_us.h \
	valhalla/baldr/streetnames_us.h \
	valhalla/baldr/transitcalendar.h \
	valhalla/baldr/transitdeparture.h \
	valhalla/baldr/transitroute.h \
	valhalla/baldr/transitstop.h \
	valhalla/baldr/transittransfer.h \
	valhalla/baldr/transittrip.h \
	valhalla/baldr/verbal_text_formatter.h \
	valhalla/baldr/verbal_text_formatter_us.h \
	valhalla/baldr/verbal_text_formatter_us_co.h \
	valhalla/baldr/verbal_text_formatter_us_tx.h \
	valhalla/baldr/verbal_text_formatter_factory.h \
        valhalla/baldr/date_time_zonespec.h
libvalhalla_baldr_la_SOURCES = \
        src/baldr/admin.cc \
	src/baldr/admininfo.cc \
	src/baldr/connectivity_map.cc \
	src/baldr/datetime.cc \
	src/baldr/directededge.cc \
	src/baldr/edgeinfo.cc \
	src/baldr/graphid.cc \
	src/baldr/graphreader.cc \
	src/baldr/graphtile.cc \
	src/baldr/graphtileheader.cc \
	src/baldr/nodeinfo.cc \
	src/baldr/location.cc \
	src/baldr/pathlocation.cc \
	src/baldr/sign.cc \
        src/baldr/signinfo.cc \
        src/baldr/srtmtile.cc \
	src/baldr/tilehierarchy.cc \
	src/baldr/turn.cc \
	src/baldr/streetname.cc \
	src/baldr/streetnames.cc \
	src/baldr/streetnames_factory.cc \
	src/baldr/streetname_us.cc \
	src/baldr/streetnames_us.cc \
	src/baldr/transitcalendar.cc \
	src/baldr/transitdeparture.cc \
	src/baldr/transitroute.cc \
	src/baldr/transitstop.cc \
	src/baldr/transittransfer.cc \
	src/baldr/transittrip.cc \
	src/baldr/verbal_text_formatter.cc \
	src/baldr/verbal_text_formatter_us.cc \
	src/baldr/verbal_text_formatter_us_co.cc \
	src/baldr/verbal_text_formatter_us_tx.cc \
	src/baldr/verbal_text_formatter_factory.cc
libvalhalla_baldr_la_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
libvalhalla_baldr_la_LIBADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ $(BOOST_SYSTEM_LIB) $(BOOST_FILESYSTEM_LIB) $(BOOST_THREAD_LIB) $(BOOST_SERIALIZATION_LIB) $(BOOST_DATE_TIME_LIB)

#distributed executables
bin_PROGRAMS = srtmtest
srtmtest_SOURCES = \
        src/baldr/srtmtest/srtmtest.cc
srtmtest_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
srtmtest_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@  $(BOOST_FILESYSTEM_LIB) -lz libvalhalla_baldr.la

# tests
check_PROGRAMS = \
	test/location \
	test/datetime \
	test/directededge \
	test/graphid \
	test/tilehierarchy \
	test/graphtile \
	test/nodeinfo \
	test/turn \
	test/graphreader \
	test/streetname \
	test/streetname_us \
	test/streetnames \
	test/streetnames_us \
	test/streetnames_factory \
	test/json \
	test/verbal_text_formatter \
	test/verbal_text_formatter_us \
	test/verbal_text_formatter_us_co \
	test/verbal_text_formatter_us_tx
test_location_SOURCES = test/location.cc test/test.cc
test_location_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS)
test_location_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) libvalhalla_baldr.la
test_datetime_SOURCES = test/datetime.cc test/test.cc
test_datetime_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS)
test_datetime_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) libvalhalla_baldr.la
test_directededge_SOURCES = test/directededge.cc test/test.cc
test_directededge_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS)
test_directededge_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) libvalhalla_baldr.la
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
test_turn_SOURCES = test/turn.cc test/test.cc
test_turn_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS)
test_turn_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) libvalhalla_baldr.la
test_graphreader_SOURCES = test/graphreader.cc test/test.cc
test_graphreader_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS)
test_graphreader_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) libvalhalla_baldr.la
test_streetname_SOURCES = test/streetname.cc test/test.cc
test_streetname_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS)
test_streetname_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) libvalhalla_baldr.la
test_streetname_us_SOURCES = test/streetname_us.cc test/test.cc
test_streetname_us_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS)
test_streetname_us_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) libvalhalla_baldr.la
test_streetnames_SOURCES = test/streetnames.cc test/test.cc
test_streetnames_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS)
test_streetnames_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) libvalhalla_baldr.la
test_streetnames_us_SOURCES = test/streetnames_us.cc test/test.cc
test_streetnames_us_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS)
test_streetnames_us_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) libvalhalla_baldr.la
test_streetnames_factory_SOURCES = test/streetnames_factory.cc test/test.cc
test_streetnames_factory_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS)
test_streetnames_factory_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) libvalhalla_baldr.la
test_json_SOURCES = test/json.cc test/test.cc
test_json_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_json_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libvalhalla_baldr.la
test_verbal_text_formatter_SOURCES = test/verbal_text_formatter.cc test/test.cc
test_verbal_text_formatter_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS)
test_verbal_text_formatter_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) libvalhalla_baldr.la
test_verbal_text_formatter_us_SOURCES = test/verbal_text_formatter_us.cc test/test.cc
test_verbal_text_formatter_us_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS)
test_verbal_text_formatter_us_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) libvalhalla_baldr.la
test_verbal_text_formatter_us_co_SOURCES = test/verbal_text_formatter_us_co.cc test/test.cc
test_verbal_text_formatter_us_co_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS)
test_verbal_text_formatter_us_co_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) libvalhalla_baldr.la
test_verbal_text_formatter_us_tx_SOURCES = test/verbal_text_formatter_us_tx.cc test/test.cc
test_verbal_text_formatter_us_tx_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS)
test_verbal_text_formatter_us_tx_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) libvalhalla_baldr.la


TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
