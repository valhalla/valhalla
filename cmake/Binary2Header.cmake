include(CMakeParseArguments)

# Function to wrap a given string into multiple lines at the given column position.
# Parameters:
#   VARIABLE    - The name of the CMake variable holding the string.
#   AT_COLUMN   - The column position at which string will be wrapped.
function(WRAP_STRING)
    set(oneValueArgs VARIABLE AT_COLUMN)
    cmake_parse_arguments(WRAP_STRING "${options}" "${oneValueArgs}" "" ${ARGN})

    string(LENGTH ${${WRAP_STRING_VARIABLE}} stringLength)
    math(EXPR offset "0")

    while(stringLength GREATER 0)

        if(stringLength GREATER ${WRAP_STRING_AT_COLUMN})
            math(EXPR length "${WRAP_STRING_AT_COLUMN}")
        else()
            math(EXPR length "${stringLength}")
        endif()

        string(SUBSTRING ${${WRAP_STRING_VARIABLE}} ${offset} ${length} line)
        set(lines "${lines}\n${line}")

        math(EXPR stringLength "${stringLength} - ${length}")
        math(EXPR offset "${offset} + ${length}")
    endwhile()

    set(${WRAP_STRING_VARIABLE} "${lines}" PARENT_SCOPE)
endfunction()

# Function to embed contents of a file as byte array in C/C++ header file(.h). The header file
# will contain a byte array and integer variable holding the size of the array.
# Parameters
#   SOURCE_FILE     - The path of source file whose contents will be embedded in the header file.
#   VARIABLE_NAME   - The name of the variable for the byte array. The string "_SIZE" will be append
#                     to this name and will be used a variable name for size variable.
#   HEADER_FILE     - The path of header file.
#   APPEND          - If specified appends to the header file instead of overwriting it
#   NULL_TERMINATE  - If specified a null byte(zero) will be append to the byte array. This will be
#                     useful if the source file is a text file and we want to use the file contents
#                     as string. But the size variable holds size of the byte array without this
#                     null byte.
#   SKIP_LINES      - Number of lines to skip (max 1).
# Usage:
#   bin2h(SOURCE_FILE "Logo.png" HEADER_FILE "Logo.h" VARIABLE_NAME "LOGO_PNG")
function(BIN2H)
    set(options APPEND NULL_TERMINATE RAW)
    set(oneValueArgs SOURCE_FILE VARIABLE_NAME HEADER_FILE SKIP_LINES)
    cmake_parse_arguments(BIN2H "${options}" "${oneValueArgs}" "" ${ARGN})

    if(BIN2H_SKIP_LINES AND NOT BIN2H_SKIP_LINES EQUAL 1)
      message(AUTHOR_WARNING "Only 1 line skip is supported")
    endif()

    set(MSVC ON)
    # First workaround for MSVC limits (see below), generate data in hex:
    # cmake -DMSVC=ON Binary2Header.cmake...
    if(BIN2H_RAW AND NOT MSVC)
      file(READ ${BIN2H_SOURCE_FILE} string)
      if(BIN2H_SKIP_LINES)
        string(FIND "${string}" "\n" pos)
        math(EXPR pos "${pos}+1")
        string(SUBSTRING "${string}" ${pos} -1 string)
      endif()
      
      string(LENGTH "${string}" arraySize)
      if(MSVC) # TODO: Effectively deactivated by earlier NOT MSVC, still on brainstorming
        # Alternative workaround for MSVC limits to generate long string literals with concatenation:
        # - While an individual quoted string cannot be longer than 2048 bytes,
        #   a string literal of roughly 65535 bytes can be constructed by concatenating strings.
        # - otherwise, compiler error C2026: string too big, trailing characters truncated
        # - 65K bytes of total length must not be exceeded.
        set(arrayValues "")
        foreach(_split_pos RANGE 0 ${arraySize} 2000) # Visual C++ max is 2048 bytes
          string(SUBSTRING "${string}" ${_split_pos} 2000 _concat_string)
          set(arrayValues "${arrayValues}R\"bin(${_concat_string})bin\"\n")
          unset(_concat_string)
        endforeach()
      else()
        set(arrayValues "R\"bin(${string})bin\"")
      endif()
    else()
      # reads source file contents as hex string
      file(READ ${BIN2H_SOURCE_FILE} hexString HEX)
      if(BIN2H_SKIP_LINES)
        string(FIND "${hexString}" "0a" pos)
        math(EXPR pos "${pos}+2")
        string(SUBSTRING ${hexString} ${pos} -1 hexString)
      endif()

      string(LENGTH ${hexString} hexStringLength)

      # appends null byte if asked
      if(BIN2H_NULL_TERMINATE)
        set(hexString "${hexString}00")
      endif()

      # wraps the hex string into multiple lines at column 32(i.e. 16 bytes per line)
      wrap_string(VARIABLE hexString AT_COLUMN 32)
      math(EXPR arraySize "${hexStringLength} / 2")

      # adds '0x' prefix and comma suffix before and after every byte respectively
      string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1, " arrayValues ${hexString})
      # remove trailing spaces
      string(REGEX REPLACE " \n" "\n" arrayValues ${arrayValues})
      # removes trailing comma
      string(REGEX REPLACE ", $" "" arrayValues ${arrayValues})
      set(arrayValues "{${arrayValues}}")
    endif()

    # converts the variable name into proper C identifier
    if(NOT BIN2H_VARIABLE_NAME)
      set(BIN2H_VARIABLE_NAME "${BIN2H_SOURCE_FILE}")
    endif()
    string(MAKE_C_IDENTIFIER "${BIN2H_VARIABLE_NAME}" BIN2H_VARIABLE_NAME)
    string(TOLOWER "${BIN2H_VARIABLE_NAME}" BIN2H_VARIABLE_NAME)

    # declares byte array and the length variables
    set(arrayDefinition "const unsigned char ${BIN2H_VARIABLE_NAME}[] = ${arrayValues};")
    set(arraySizeDefinition "const size_t ${BIN2H_VARIABLE_NAME}_len = ${arraySize};")
    set(declarations "${arrayDefinition}\n\n${arraySizeDefinition}\n\n")
    if(BIN2H_APPEND)
        file(APPEND ${BIN2H_HEADER_FILE} "${declarations}")
    else()
        file(WRITE ${BIN2H_HEADER_FILE} "${declarations}")
    endif()
endfunction()

if(CMAKE_SCRIPT_MODE_FILE AND "${CMAKE_SCRIPT_MODE_FILE}" MATCHES "Binary2Header.cmake$")
  # Parse command line argmuents
  set(ARG_NUM 1)
  set(conversion_type "HEADER")
  set(options "")
  while(ARG_NUM LESS CMAKE_ARGC)
    set(CURRENT_ARG ${CMAKE_ARGV${ARG_NUM}})
    if(${CURRENT_ARG} MATCHES "^--usage$")
      message("Usage:
       cmake -P cmake/Binary2Header.cmake [options] infile outfile
Options:
    --header                convert to a header file
    --locales               convert locales json files to a header file
    --variable-name [NAME]  variable name, default is converted to C identifier <infile>
    --skip-line [NUM]       skip NUM lines, default 0
    --append                append to a file
    --raw                   generate raw string literals
    --null                  add a null byte(zero) to the byte array")
      set(exit TRUE)
    elseif(${CURRENT_ARG} MATCHES "^--header")
      set(conversion_type "HEADER")
    elseif(${CURRENT_ARG} MATCHES "--locales")
      set(conversion_type "LOCALES")
    elseif(${CURRENT_ARG} MATCHES "^--append$")
      list(APPEND options "APPEND")
    elseif(${CURRENT_ARG} MATCHES "^--raw")
      list(APPEND options "RAW")
    elseif(${CURRENT_ARG} MATCHES "^--null$")
      list(APPEND options "NULL_TERMINATE")
    elseif(${CURRENT_ARG} MATCHES "^--variable-name$")
      math(EXPR ARG_NUM "${ARG_NUM}+1")
      list(APPEND options "VARIABLE_NAME;${CMAKE_ARGV${ARG_NUM}}")
    elseif(${CURRENT_ARG} MATCHES "^--skip$")
      math(EXPR ARG_NUM "${ARG_NUM}+1")
      list(APPEND options "SKIP_LINES;${CMAKE_ARGV${ARG_NUM}}")
    elseif(${CURRENT_ARG} MATCHES "^-P")
      math(EXPR ARG_NUM "${ARG_NUM}+1") # skip -P and self
    elseif(${CURRENT_ARG} MATCHES "^-D")
      set(_) # skip self and any -D variables
    elseif(${CURRENT_ARG} MATCHES "^[^-]")
      if(NOT source)
        set(source ${CURRENT_ARG})
      elseif(NOT target)
        set(target ${CURRENT_ARG})
      endif()
    endif()
    math(EXPR ARG_NUM "${ARG_NUM}+1")
  endwhile()

  if(NOT exit)
    if(NOT source OR NOT EXISTS ${source})
      message(FATAL_ERROR "no source file or ${source} does not exist")
    elseif(NOT target)
      message(FATAL_ERROR "no target file")
    endif()

    if(conversion_type MATCHES HEADER)
      bin2h(SOURCE_FILE ${source} HEADER_FILE ${target} ${options})
    elseif(conversion_type MATCHES LOCALES)
      file(WRITE ${target} "#include <unordered_map>\n")
      file(GLOB json_files LIST_DIRECTORIES FALSE "${source}/*.json")
      foreach(file ${json_files})
        get_filename_component(name "${file}" NAME)
        bin2h(SOURCE_FILE ${file} HEADER_FILE ${target} VARIABLE_NAME ${name} APPEND RAW)
      endforeach()

      set(map "\nconst std::unordered_map<std::string, std::string> locales_json = {\n")
      foreach(file ${json_files})
        get_filename_component(name "${file}" NAME)
        get_filename_component(locale "${file}" NAME_WE)
        string(MAKE_C_IDENTIFIER "${name}" name)
        string(TOLOWER "${name}" name)
        set(map "${map}    {\"${locale}\", {${name}, ${name} + ${name}_len}},\n")
      endforeach()
      set(map "${map}};\n")
      file(APPEND ${target} "${map}")
    endif()
  endif()
endif()
