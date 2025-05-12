## Declare C++ build configuration variables as part of HandleLibcxxFlags.
#
# - LIBCXX_COMPILE_FLAGS: flags used to compile libc++
# - LIBCXX_LINK_FLAGS: flags used to link libc++
# - LIBCXX_LIBRARIES: libraries to link libc++ to
set(LIBCXX_COMPILE_FLAGS "")
set(LIBCXX_LINK_FLAGS "")
set(LIBCXX_LIBRARIES "")

# Include build macros for updating configuration variables
include(HandleLibcxxFlags)

function (cxx_add_warning_flags target)
  target_add_compile_flags_if_supported(${target} PRIVATE -Wall -Wextra)
  if (ENABLE_WERROR)
   target_add_compile_flags_if_supported(${target} PRIVATE -Werror)
  endif()
endfunction()
