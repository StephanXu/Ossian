
function(JSON_SCHEMA_GENERATE_CPP IS_ON_CONFIGURATION HDRS DEST)
  if(NOT ARGN)
    message(SEND_ERROR "Error: JSON_SCHEMA_GENERATE_CPP() called without any json-schema files")
    return()
  endif()

  if (WIN32)
    find_program(_QUICK_TYPE_EXECUTABLE "quicktype.cmd")
  else()
    find_program(_QUICK_TYPE_EXECUTABLE "quicktype")
  endif()
  message(STATUS "quicktype location: ${_QUICK_TYPE_EXECUTABLE}")

  foreach(FIL ${ARGN})
    get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
    get_filename_component(FIL_WE ${FIL} NAME_WE)
  
    LIST(APPEND HDRS "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.schema.hpp")

    if (NOT IS_ON_CONFIGURATION)
        add_custom_command(
            WORKING_DIRECTORY
                ${CMAKE_CURRENT_SOURCE_DIR}
            OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.schema.hpp"
            COMMAND ${_QUICK_TYPE_EXECUTABLE}
            ARGS 
                --lang c++ 
                --include-location global-include
                --no-boost
                --namespace ${FIL_WE}
                --source-style single-source
                --code-format with-struct
                --type-style pascal-case
                --member-style camel-case
                --enumerator-style upper-underscore-case
                --src-lang schema
                --out "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.schema.hpp"
                "${FIL}"
            DEPENDS "${FIL}"
            )
    else()
        execute_process(
            COMMAND ${_QUICK_TYPE_EXECUTABLE}
                --lang c++ 
                --include-location global-include
                --no-boost
                --namespace ${FIL_WE}
                --source-style single-source
                --code-format with-struct
                --type-style pascal-case
                --member-style camel-case
                --enumerator-style upper-underscore-case
                --src-lang schema
                --out "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.schema.hpp"
                "${FIL}"
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            RESULT_VARIABLE QUICK_TYPE_GENERATE_STATUS
            )
        if(QUICK_TYPE_GENERATE_STATUS AND NOT QUICK_TYPE_GENERATE_STATUS EQUAL 0)
            message(STATUS "quicktype generate schema ${FIL} failed: ${QUICK_TYPE_GENERATE_STATUS}")
        else()
            message(STATUS "quicktype generate schema ${FIL} success")
        endif()
    endif()
  endforeach()

  set_source_files_properties(${${HDRS}} PROPERTIES GENERATED TRUE)
  set(${HDRS} ${${HDRS}} PARENT_SCOPE)
endfunction()