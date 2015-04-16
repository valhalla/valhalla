ACLOCAL_AMFLAGS = -I m4
AM_LDFLAGS = @BOOST_LDFLAGS@ @COVERAGE_LDFLAGS@
AM_CPPFLAGS = -Ivalhalla -Ivalhalla/proto @BOOST_CPPFLAGS@
AM_CXXFLAGS = -Ivalhalla -Ivalhalla/proto @COVERAGE_CXXFLAGS@
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

# proto generation etc
PROTO_FILES = \
	proto/trippath.proto \
	proto/tripdirections.proto \
	proto/directions_options.proto
BUILT_SOURCES = $(patsubst %.proto,src/%.pb.cc,$(PROTO_FILES))
CLEANFILES = $(patsubst %.proto,valhalla/%.pb.h,$(PROTO_FILES)) $(patsubst %.proto,src/%.pb.cc,$(PROTO_FILES))
nodist_libvalhalla_odin_la_SOURCES = $(patsubst %.proto,src/%.pb.cc,$(PROTO_FILES))
src/proto/%.pb.cc: proto/%.proto
	@echo " PROTOC $<"; mkdir -p src/proto valhalla/proto; @PROTOC_BIN@ -Iproto --cpp_out=valhalla/proto $< && mv valhalla/proto/$(@F) src/proto

# lib valhalla compilation etc
lib_LTLIBRARIES = libvalhalla_odin.la
nobase_include_HEADERS = \
	valhalla/proto/trippath.pb.h \
	valhalla/proto/tripdirections.pb.h \
	valhalla/proto/directions_options.pb.h \
	valhalla/odin/directionsbuilder.h \
	valhalla/odin/maneuversbuilder.h \
	valhalla/odin/narrativebuilder.h \
	valhalla/odin/enhancedtrippath.h \
	valhalla/odin/maneuver.h \
	valhalla/odin/sign.h \
	valhalla/odin/signs.h \
	valhalla/odin/util.h
libvalhalla_odin_la_SOURCES = \
	src/proto/trippath.pb.cc \
	src/proto/tripdirections.pb.cc \
	src/proto/directions_options.pb.cc \
	src/odin/directionsbuilder.cc \
	src/odin/maneuversbuilder.cc \
	src/odin/narrativebuilder.cc \
	src/odin/enhancedtrippath.cc \
	src/odin/maneuver.cc \
	src/odin/sign.cc \
	src/odin/signs.cc \
	src/odin/util.cc
libvalhalla_odin_la_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
libvalhalla_odin_la_LIBADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS)

#distributed executables
#bin_PROGRAMS = odin
#odin_SOURCES = 
#odin_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
#odin_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ @BOOST_PROGRAM_OPTIONS_LIB@ @BOOST_FILESYSTEM_LIBS@ @PROTOC_LIBS@ -lz libvalhalla_odin.la

# tests
check_PROGRAMS = \
	test/maneuversbuilder \
	test/narrativebuilder \
	test/enhancedtrippath \
	test/sign 
test_maneuversbuilder_SOURCES = test/maneuversbuilder.cc test/test.cc
test_maneuversbuilder_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_maneuversbuilder_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libvalhalla_odin.la
test_narrativebuilder_SOURCES = test/narrativebuilder.cc test/test.cc
test_narrativebuilder_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_narrativebuilder_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libvalhalla_odin.la
test_enhancedtrippath_SOURCES = test/enhancedtrippath.cc test/test.cc
test_enhancedtrippath_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_enhancedtrippath_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libvalhalla_odin.la
test_sign_SOURCES = test/sign.cc test/test.cc
test_sign_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_sign_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libvalhalla_odin.la
test_streetnames_SOURCES = test/streetnames.cc test/test.cc
test_streetnames_CPPFLAGS = $(DEPS_CFLAGS) $(VALHALLA_CPPFLAGS) @BOOST_CPPFLAGS@
test_streetnames_LDADD = $(DEPS_LIBS) $(VALHALLA_LDFLAGS) @BOOST_LDFLAGS@ libvalhalla_odin.la

TESTS = $(check_PROGRAMS)
TEST_EXTENSIONS = .sh
SH_LOG_COMPILER = sh

test: check
