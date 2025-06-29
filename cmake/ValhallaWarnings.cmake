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

function (configure_cxx_warnings target)
  if (ENABLE_COMPILER_WARNINGS)
    cxx_add_warning_flags(${target})
  else()
    if(MSVC)
      # Turn off some warnings that currently causes a lot of noise.
      target_compile_options(${target} PRIVATE
        /wd4244 # 'argument': conversion from 'type1' to 'type2', possible loss of data
        /wd4267 # 'var': conversion from 'size_t' to 'type', possible loss of data
        /wd4305 # 'argument': truncation from 'type1' to 'type2'
        /wd4334 # 'operator': result of 32-bit shift implicitly converted to 64 bits (potential portability problem)
      )
      # target_compile_definitions(${target} PRIVATE
      #   # TODO: Not ideal to mute this. But it's almost ~700 warnings in the codebase.
      #   _CRT_SECURE_NO_WARNINGS # Disable warnings about unsafe functions
      # )
    endif()
  endif()
endfunction()
