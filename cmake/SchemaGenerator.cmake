
function(JSON_SCHEMA_GENERATE_CPP IS_ON_CONFIGURATION HDRS DEST)
  if(NOT ARGN)
    message(SEND_ERROR "Error: JSON_SCHEMA_GENERATE_CPP() called without any json-schema files")
    return()
  endif()

  find_program(_QUICK_TYPE_EXECUTABLE "quicktype")

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
                --namespace OssianConfig
                --source-style single-source
                --code-format with-getter-setter
                --type-style camel-case
                --member-style pascal-case
                --src-lang schema
                --out "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.schema.hpp"
                "${FIL}"
            DEPENDS "${FIL}")
    else()
        execute_process(
            COMMAND ${_QUICK_TYPE_EXECUTABLE}
                --lang c++ 
                --include-location global-include
                --no-boost
                --namespace OssianConfig
                --source-style single-source
                --code-format with-getter-setter
                --type-style camel-case
                --member-style pascal-case
                --src-lang schema
                --out "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.schema.hpp"
                "${FIL}"
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    endif()
  endforeach()

  set_source_files_properties(${${HDRS}} PROPERTIES GENERATED TRUE)
  set(${HDRS} ${${HDRS}} PARENT_SCOPE)
endfunction()