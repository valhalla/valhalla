# ============================================================
# Check for protoc
#
# Looks for the protoc binary and the protocol buffers
# headers.
#
# If it succeeds, it calls:
#   AC_SUBST(PROTOC_BIN), AC_SUBST(PROTOC_CPPFLAGS) and
#   AC_SUBST(PROTOC_LIBS)
# and sets HAVE_PROTOC
#
# Note: based, in part, on ax_boost_base.m4
#
# LICENSE
#
#   Copyright (c) 2014 MapQuest Inc.
#   Copyright (c) 2008 Thomas Porschberg <thomas@randspringer.de>
#   Copyright (c) 2009 Peter Adolphs
#
#   Copying and distribution of this file, with or without modification, are
#   permitted in any medium without royalty provided the copyright notice
#   and this notice are preserved. This file is offered as-is, without any
#   warranty.

AC_DEFUN([REQUIRE_PROTOC],
[
AC_ARG_WITH([protoc],
  [AS_HELP_STRING([--with-protoc=PATH],
    [full path to the protoc program.])],
  [ac_protoc_bin="$withval"],
  [ac_protoc_bin=""])

if test "x$ac_protoc_bin" = "x" ; then
   AC_PATH_PROG([PROTOC_BIN],[protoc])
else
   PROTOC_BIN="$ac_protoc_bin"
fi

AC_ARG_WITH([protobuf-includes],
  [AS_HELP_STRING([--with-protobuf-includes=PATH],
    [path to base of include files for protobuf (without /google at the end)])],
  [ac_protobuf_includes="$withval"],
  [ac_protobuf_includes="/usr/include"])

AC_ARG_WITH([protobuf-libdir],
  [AS_HELP_STRING([--with-protobuf-libdir=PATH],
    [path to directory containing libraries for protobuf])],
  [ac_protobuf_libdir="$withval"],
  [ac_protobuf_libdir=""])

dnl Check that the binary is present

AC_MSG_CHECKING(for protoc)
if test -x "$PROTOC_BIN" ; then
  AC_MSG_RESULT([yes])
else
  AC_MSG_RESULT([no])
  AC_MSG_ERROR([cannot find protoc in your path, but it is required. Please install protobuf-compiler.])
fi
AC_SUBST(PROTOC_BIN)

dnl Check that the headers are present

old_cppflags=${CPPFLAGS}
PROTOC_CPPFLAGS=""
if test -n "${ac_protobuf_includes}" ; then
   PROTOC_CPPFLAGS="-I${ac_protobuf_includes}"
   CPPFLAGS="${CPPFLAGS} ${PROTOC_CPPFLAGS}"
fi

AC_MSG_CHECKING(for protobuf headers)
AC_COMPILE_IFELSE(
  [AC_LANG_PROGRAM([#include <google/protobuf/stubs/common.h>])],
  [AC_MSG_RESULT([yes])],
  [AC_MSG_RESULT([no])
   AC_MSG_ERROR([cannot find protobuf headers, please install libprotobuf-dev.])])

CPPFLAGS=${old_cppflags}
AC_SUBST(PROTOC_CPPFLAGS)

dnl On 64-bit systems check for system libraries in both lib64 and lib.
dnl The former is specified by FHS, but e.g. Debian does not adhere to
dnl this (as it rises problems for generic multi-arch support).
dnl The last entry in the list is chosen by default when no libraries
dnl are found, e.g. when only header-only libraries are installed!
ax_pb_libsubdirs="lib"
ax_arch=`uname -m`
case $ax_arch in
  x86_64|ppc64|s390x|sparc64|aarch64)
    ax_pb_libsubdirs="lib64 lib lib64"
    ;;
esac

dnl allow for real multi-arch paths e.g. /usr/lib/x86_64-linux-gnu. Give
dnl them priority over the other paths since, if libs are found there, they
dnl are almost assuredly the ones desired.
AC_REQUIRE([AC_CANONICAL_HOST])
ax_pb_libsubdirs="lib/${host_cpu}-${host_os} $ax_pb_libsubdirs"

case ${host_cpu} in
  i?86)
    ax_pb_libsubdirs="lib/i386-${host_os} $ax_pb_libsubdirs"
    ;;
esac

ax_pb_libdirs="/usr /usr/local /opt /opt/local"

AC_MSG_CHECKING(for -lprotobuf)

if test -n "$ac_protobuf_libdir" ; then
  ax_pb_old_libs=${LIBS}
  LIBS="-L${ac_protobuf_libdir} -lprotobuf"
  ax_pb_old_cppflags=${CPPFLAGS}
  CPPFLAGS="$CPPFLAGS $PROTOC_CPPFLAGS"
  AC_LINK_IFELSE(
    [AC_LANG_PROGRAM([[#include <google/protobuf/stubs/common.h>]],
		     [[google::protobuf::ShutdownProtobufLibrary();]]
       		    )],
    [PROTOC_LIBS="-L${ac_protobuf_libdir} -lprotobuf"],
    [AC_MSG_RESULT([no])])
  LIBS=${ax_pb_old_libs}
  CPPFLAGS=${ax_pb_old_cppflags}
fi

if test -z "$PROTOC_LIBS" ; then
  for ax_pb_libsubdir in $ax_pb_libsubdirs; do
    for ax_pb_libdir in $ax_pb_libdirs; do
      ax_pb_old_libs=${LIBS}
      LIBS="-L${ax_pb_libdir}/${ax_pb_libsubdir} -lprotobuf"
      ax_pb_old_cppflags=${CPPFLAGS}
      CPPFLAGS="$CPPFLAGS $PROTOC_CPPFLAGS"
      AC_LINK_IFELSE(
        [AC_LANG_PROGRAM([[#include <google/protobuf/stubs/common.h>]],
	                 [[google::protobuf::ShutdownProtobufLibrary();]]
       		    	 )],
        [PROTOC_LIBS="-L${ax_pb_libdir}/${ax_pb_libsubdir} -lprotobuf"
         break
        ],[])
      CPPFLAGS=${ax_pb_old_cppflags}
      LIBS=${ax_pb_old_libs}
    done
    if test -n "$PROTOC_LIBS" ; then break; fi
  done
fi

if test -n "$PROTOC_LIBS" ; then
  AC_MSG_RESULT([yes])
else
  AC_MSG_RESULT([no])
  AC_MSG_ERROR([cannot find protobuf library, please install libprotobuf7 or libprotobuf8.])
fi

AC_SUBST(PROTOC_LIBS)
AC_DEFINE(HAVE_PROTOC,,[define if protocol buffers library is available])
])
