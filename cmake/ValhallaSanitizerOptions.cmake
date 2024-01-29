if (ENABLE_SANITIZERS)
  set(ENABLE_ADDRESS_SANITIZER ON)
  set(ENABLE_UNDEFINED_SANITIZER ON)
endif()

# Include build macros for updating configuration variables
include(HandleLibcxxFlags)

set (SANITIZER_FLAGS_LIST "")
set (SANITIZER_EXE_LINKER_FLAGS_LIST "")
set (SANITIZER_SHARED_LINKER_FLAGS_LIST "")

if(ENABLE_ADDRESS_SANITIZER)
  message(STATUS "Enabling address sanitizer (ASan).")
  set(OLD_CMAKE_REQUIRED_FLAGS ${CMAKE_REQUIRED_FLAGS})
  set(CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS} -fsanitize=address")
  append_flags_if_supported(SANITIZER_FLAGS_LIST -fsanitize=address -fno-omit-frame-pointer -fno-optimize-sibling-calls)
  append_flags_if_supported(SANITIZER_EXE_LINKER_FLAGS_LIST -fsanitize=address)
  append_flags_if_supported(SANITIZER_SHARED_LINKER_FLAGS_LIST -fsanitize=address)
  set(CMAKE_REQUIRED_FLAGS ${OLD_CMAKE_REQUIRED_FLAGS})
endif()

if(ENABLE_UNDEFINED_SANITIZER)
  message(STATUS "Enabling undefined behavior sanitizer (UBSan).")
  append_flags_if_supported(SANITIZER_FLAGS_LIST -fsanitize=undefined -fno-sanitize=vptr)
  append_flags_if_supported(SANITIZER_EXE_LINKER_FLAGS_LIST -fsanitize=undefined -fno-sanitize=vptr)
  append_flags_if_supported(SANITIZER_SHARED_LINKER_FLAGS_LIST -fsanitize=undefined -fno-sanitize=vptr)
endif()

list(JOIN SANITIZER_FLAGS_LIST " " SANITIZER_FLAGS_LIST)
list(JOIN SANITIZER_EXE_LINKER_FLAGS_LIST " " SANITIZER_EXE_LINKER_FLAGS_LIST)
list(JOIN SANITIZER_SHARED_LINKER_FLAGS_LIST " " SANITIZER_SHARED_LINKER_FLAGS_LIST)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SANITIZER_FLAGS_LIST}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${SANITIZER_EXE_LINKER_FLAGS_LIST}")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${SANITIZER_SHARED_LINKER_FLAGS_LIST}")
