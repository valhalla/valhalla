dnl ===================================================================
dnl   Licensed to the Apache Software Foundation (ASF) under one
dnl   or more contributor license agreements.  See the NOTICE file
dnl   distributed with this work for additional information
dnl   regarding copyright ownership.  The ASF licenses this file
dnl   to you under the Apache License, Version 2.0 (the
dnl   "License"); you may not use this file except in compliance
dnl   with the License.  You may obtain a copy of the License at
dnl
dnl     http://www.apache.org/licenses/LICENSE-2.0
dnl
dnl   Unless required by applicable law or agreed to in writing,
dnl   software distributed under the License is distributed on an
dnl   "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
dnl   KIND, either express or implied.  See the License for the
dnl   specific language governing permissions and limitations
dnl   under the License.
dnl ===================================================================
dnl
dnl The default behaviour is to use pkg-config to look for an LZ4
dnl library and if that fails to simply try linking -llz4.
dnl
dnl The user can specify --with-lz4=PREFIX to look in PREFIX or
dnl --with-lz4=internal to use the internal copy of the LZ4 code.

AC_DEFUN([SVN_LZ4],
[
  AC_ARG_WITH([lz4],
    [AS_HELP_STRING([--with-lz4=PREFIX|internal],
                    [look for lz4 in PREFIX or use the internal code])],
    [
      if test "$withval" = internal; then
        lz4_prefix=internal
      elif test "$withval" = yes; then
        lz4_prefix=std
      else
        lz4_prefix="$withval"
      fi
    ],
    [lz4_prefix=std])

  if test "$lz4_prefix" = "internal"; then
    AC_MSG_NOTICE([using internal lz4])
    AC_DEFINE([SVN_INTERNAL_LZ4], [1],
               [Define to use internal LZ4 code])
  else
    if test "$lz4_prefix" = "std"; then
      SVN_LZ4_STD
    else
      SVN_LZ4_PREFIX
    fi
    if test "$lz4_found" != "yes"; then
      AC_MSG_ERROR([Subversion requires LZ4 >= r129, or use --with-lz4=internal])
    fi
  fi
  AC_SUBST(SVN_LZ4_INCLUDES)
  AC_SUBST(SVN_LZ4_LIBS)
])

dnl LZ4 changed versioning schemes after r131, the next version being 1.7.3, so
dnl we need to check for both schemes.  We can't use "--atleast-version=1.7.3"
dnl because that will result in a >= 1 check for the older versioning scheme.
dnl Instead, fallback to --max-version=3 if the old scheme fails.  This gives a
dnl little room for major version changes, but won't falsely identify old
dnl versions as supported.
AC_DEFUN([SVN_LZ4_STD],
[
  if test -n "$PKG_CONFIG"; then
    AC_MSG_CHECKING([for lz4 library via pkg-config])
    if $PKG_CONFIG liblz4 --atleast-version=129 || $PKG_CONFIG liblz4 --max-version=3; then
      AC_MSG_RESULT([yes])
      lz4_found=yes
      SVN_LZ4_INCLUDES=`$PKG_CONFIG liblz4 --cflags`
      SVN_LZ4_LIBS=`$PKG_CONFIG liblz4 --libs`
      SVN_LZ4_LIBS="`SVN_REMOVE_STANDARD_LIB_DIRS($SVN_LZ4_LIBS)`"
    else
      AC_MSG_RESULT([no])
    fi
  fi
  if test "$lz4_found" != "yes"; then
    AC_MSG_NOTICE([lz4 configuration without pkg-config])
    AC_CHECK_LIB(lz4, LZ4_compress_default, [
      lz4_found=yes
      SVN_LZ4_LIBS="-llz4"
    ])
  fi
])

AC_DEFUN([SVN_LZ4_PREFIX],
[
  AC_MSG_NOTICE([lz4 configuration via prefix])
  save_cppflags="$CPPFLAGS"
  CPPFLAGS="$CPPFLAGS -I$lz4_prefix/include"
  save_ldflags="$LDFLAGS"
  LDFLAGS="$LDFLAGS -L$lz4_prefix/lib"
  AC_CHECK_LIB(lz4, LZ4_compress_default, [
    lz4_found=yes
    SVN_LZ4_INCLUDES="-I$lz4_prefix/include"
    SVN_LZ4_LIBS="`SVN_REMOVE_STANDARD_LIB_DIRS(-L$lz4_prefix/lib)` -llz4"
  ])
  LDFLAGS="$save_ldflags"
  CPPFLAGS="$save_cppflags"
])
