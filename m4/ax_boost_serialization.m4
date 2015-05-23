# ===========================================================================
#  http://www.gnu.org/software/autoconf-archive/ax_boost_serialization.html
# ===========================================================================
#
# SYNOPSIS
#
#   AX_BOOST_SERIALIZATION
#
# DESCRIPTION
#
#   Test for Serialization library from the Boost C++ libraries. The macro
#   requires a preceding call to AX_BOOST_BASE. Further documentation is
#   available at <http://randspringer.de/boost/index.html>.
#
#   This macro calls:
#
#     AC_SUBST(BOOST_SERIALIZATION_LIB)
#
#   And sets:
#
#     HAVE_BOOST_SERIALIZATION
#
# LICENSE
#
#   Copyright (c) 2008 Thomas Porschberg <thomas@randspringer.de>
#
#   Copying and distribution of this file, with or without modification, are
#   permitted in any medium without royalty provided the copyright notice
#   and this notice are preserved. This file is offered as-is, without any
#   warranty.

#serial 21

AC_DEFUN([AX_BOOST_SERIALIZATION],
[
	AC_ARG_WITH([boost-serialization],
	AS_HELP_STRING([--with-boost-serialization@<:@=special-lib@:>@],
                   [use the Serialization library from boost - it is possible to specify a certain library for the linker
                        e.g. --with-boost-serialization=boost_serialization-gcc-mt-d-1_33_1 ]),
        [
        if test "$withval" = "no"; then
			want_boost="no"
        elif test "$withval" = "yes"; then
            want_boost="yes"
            ax_boost_user_serialization_lib=""
        else
		    want_boost="yes"
		ax_boost_user_serialization_lib="$withval"
		fi
        ],
        [want_boost="yes"]
	)

	if test "x$want_boost" = "xyes"; then
        AC_REQUIRE([AC_PROG_CC])
		CPPFLAGS_SAVED="$CPPFLAGS"
		CPPFLAGS="$CPPFLAGS $BOOST_CPPFLAGS"
	    AC_MSG_WARN(BOOST_CPPFLAGS $BOOST_CPPFLAGS)
		export CPPFLAGS

		LDFLAGS_SAVED="$LDFLAGS"
		LDFLAGS="$LDFLAGS $BOOST_LDFLAGS"
		export LDFLAGS

        AC_CACHE_CHECK(whether the Boost::Serialization library is available,
					   ax_cv_boost_serialization,
        [AC_LANG_PUSH([C++])
			 AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[@%:@include <fstream>
												 @%:@include <boost/archive/text_oarchive.hpp>
                                                 @%:@include <boost/archive/text_iarchive.hpp>
												]],
                                   [[std::ofstream ofs("filename");
									boost::archive::text_oarchive oa(ofs);
									 return 0;
                                   ]])],
                   ax_cv_boost_serialization=yes, ax_cv_boost_serialization=no)
         AC_LANG_POP([C++])
		])
		if test "x$ax_cv_boost_serialization" = "xyes"; then
			AC_DEFINE(HAVE_BOOST_SERIALIZATION,,[define if the Boost::Serialization library is available])
            BOOSTLIBDIR=`echo $BOOST_LDFLAGS | sed -e 's/@<:@^\/@:>@*//'`
            if test "x$ax_boost_user_serialization_lib" = "x"; then
                for libextension in `ls $BOOSTLIBDIR/libboost_serialization*.so* $BOOSTLIBDIR/libboost_serialization*.dylib* $BOOSTLIBDIR/libboost_serialization*.a* 2>/dev/null | sed 's,.*/,,' | sed -e 's;^lib\(boost_serialization.*\)\.so.*$;\1;' -e 's;^lib\(boost_serialization.*\)\.dylib.*$;\1;' -e 's;^lib\(boost_serialization.*\)\.a*$;\1;'` ; do
                     ax_lib=${libextension}
				    AC_CHECK_LIB($ax_lib, exit,
                                 [BOOST_SERIALIZATION_LIB="-l$ax_lib"; AC_SUBST(BOOST_SERIALIZATION_LIB) link_serialization="yes"; break],
                                 [link_serialization="no"])
				done
                if test "x$link_serialization" != "xyes"; then
                for libextension in `ls $BOOSTLIBDIR/boost_serialization*.dll* $BOOSTLIBDIR/boost_serialization*.a* 2>/dev/null | sed 's,.*/,,' | sed -e 's;^\(boost_serialization.*\)\.dll.*$;\1;' -e 's;^\(boost_serialization.*\)\.a.*$;\1;'` ; do
                     ax_lib=${libextension}
				    AC_CHECK_LIB($ax_lib, exit,
                                 [BOOST_SERIALIZATION_LIB="-l$ax_lib"; AC_SUBST(BOOST_SERIALIZATION_LIB) link_serialization="yes"; break],
                                 [link_serialization="no"])
				done
                fi

            else
               for ax_lib in $ax_boost_user_serialization_lib boost_serialization-$ax_boost_user_serialization_lib; do
				      AC_CHECK_LIB($ax_lib, main,
                                   [BOOST_SERIALIZATION_LIB="-l$ax_lib"; AC_SUBST(BOOST_SERIALIZATION_LIB) link_serialization="yes"; break],
                                   [link_serialization="no"])
                  done

            fi
            if test "x$ax_lib" = "x"; then
                AC_MSG_ERROR(Could not find a version of the library!)
            fi
			if test "x$link_serialization" != "xyes"; then
				AC_MSG_ERROR(Could not link against $ax_lib !)
			fi
		fi

		CPPFLAGS="$CPPFLAGS_SAVED"
	LDFLAGS="$LDFLAGS_SAVED"
	fi
])
