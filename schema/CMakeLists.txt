
SET(SCHEMA_META_BASE_DIR ${CMAKE_CURRENT_BINARY_DIR})

function(JSON_SCHEMA_GENERATE_CPP HDRS DEST)
  if(NOT ARGN)
    message(SEND_ERROR "Error: JSON_SCHEMA_GENERATE_CPP() called without any json-schema files")
    return()
  endif()

  find_program(_QUICK_TYPE_EXECUTABLE "quicktype")

  foreach(FIL ${ARGN})
    get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
    get_filename_component(FIL_WE ${FIL} NAME_WE)
  
    LIST(APPEND HDRS "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.schema.hpp")

    add_custom_command(
        WORKING_DIRECTORY
            ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.schema.hpp"
        COMMAND ${_QUICK_TYPE_EXECUTABLE}
        ARGS 
            --lang c++ 
            --no-boost
            --namespace ns::config
            --source-style single-source
            --code-format with-getter-setter
            --type-style camel-case
            --member-style pascal-case
            --src-lang schema
            --out "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.schema.hpp"
            "${FIL}"
        DEPENDS "${FIL}")
  endforeach()

  set_source_files_properties(${${HDRS}} PROPERTIES GENERATED TRUE)
  set(${HDRS} ${${HDRS}} PARENT_SCOPE)
endfunction()

json_schema_generate_cpp(SCHEMA_HDRS ${SCHEMA_META_BASE_DIR} 
    "Config.json"
    )


add_custom_target(SchemaGenerator
    WORKING_DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}
    DEPENDS
        ${SCHEMA_HDRS}
    COMMENT
        "Intermediate schema generator target"
    VERBATIM
    )

find_package(nlohmann_json CONFIG REQUIRED)

add_library(Schema ${SCHEMA_HDRS})
target_include_directories(Schema
        PUBLIC
          "${CMAKE_CURRENT_BINARY_DIR}"
        )
target_link_libraries(Schema PUBLIC nlohmann_json::nlohmann_json)
add_dependencies(Schema SchemaGenerator)
set_target_properties(Schema PROPERTIES LINKER_LANGUAGE CXX)