#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.65])
AC_INIT([kavrakilab-scene], [0.0.0], [ntd@rice.edu])
AM_INIT_AUTOMAKE([-Wall subdir-objects])
AC_CONFIG_SRCDIR([kavrakilab_scene/classFancy.robray])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

# Checks for programs.
AC_USE_SYSTEM_EXTENSIONS
AC_PROG_CC
AC_PROG_CC_C99
AM_PROG_AR
AC_PROG_LIBTOOL

AC_C_RESTRICT
AC_C_INLINE
AC_C_CONST


# Enable maximum warnings
m4_ifdef([AX_CFLAGS_WARN_ALL],
         [AX_CFLAGS_WARN_ALL])

dnl Checking for both versions
m4_ifdef([AX_CHECK_COMPILE_FLAG],
         [AC_DEFUN([APPEND_FLAG],
                   [AX_CHECK_COMPILE_FLAG([$1], [CFLAGS="$1 $CFLAGS"])])])

m4_ifdef([AX_CHECK_COMPILER_FLAGS],
         [AC_DEFUN([APPEND_FLAG],
                   [AX_CHECK_COMPILER_FLAGS([$1], [CFLAGS="$1 $CFLAGS"])])])

dnl If we found the flag checking macro, check some flags
m4_ifdef([APPEND_FLAG],
         [APPEND_FLAG([-Wextra])
          APPEND_FLAG([-Wconversion])
          APPEND_FLAG([-Wpointer-arith])
          APPEND_FLAG([-Wfloat-equal])
          APPEND_FLAG([-Wshadow])
          APPEND_FLAG([-Wwrite-strings])
          APPEND_FLAG([-Wc++-compat])])

dnl # Doxygen

AC_ARG_WITH([doxygen],
            AS_HELP_STRING([--without-doxygen], [Disable doxygen support]),
            [],
            [with_doxygen=auto])
AS_IF([test "x$with_doxygen" != xno],
      [AC_CHECK_PROGS([DOXYGEN],[doxygen])])
AS_IF([test "x$with_doxygen" = xyes -a "x$DOXYGEN" = x],
      [AC_MSG_ERROR([Doxygen not found])])
AM_CONDITIONAL([HAVE_DOXYGEN], [test "x$DOXYGEN" != x])

## Amino ##
PKG_CHECK_MODULES([AMINO],
                  [amino],
                  [HAVE_AMINO=yes])
CFLAGS="$CFLAGS $AMINO_CFLAGS"
LDFLAGS="$LDFLAGS $AMINO_LDFLAGS"


AC_CHECK_PROGS([AARXC], [aarxc])

AC_CONFIG_FILES([Makefile])

AC_OUTPUT
