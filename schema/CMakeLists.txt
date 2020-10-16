
SET(SCHEMA_META_BASE_DIR ${CMAKE_CURRENT_BINARY_DIR})

include(SchemaGenerator)

# By default json schema header files would be generated during compile time.
# But some remote development tools (e.g. Visual Studio) couldn't sync headers 
# correctly. You could use -DJSON_SCHEMA_GEN_ON_CONFIG=TRUE for generating
# headers at cmake configuration progress.
if(NOT DEFINED JSON_SCHEMA_GEN_ON_CONFIG)
  set(JSON_SCHEMA_GEN_ON_CONFIG FALSE)
endif()

json_schema_generate_cpp(${JSON_SCHEMA_GEN_ON_CONFIG} SCHEMA_HDRS ${SCHEMA_META_BASE_DIR} 
    "Config.json"
    )

find_package(nlohmann_json CONFIG REQUIRED)

add_library(Schema ${SCHEMA_HDRS})
target_include_directories(Schema
        PUBLIC
          "${CMAKE_CURRENT_BINARY_DIR}"
        )
target_link_libraries(Schema PUBLIC nlohmann_json::nlohmann_json)
set_target_properties(Schema PROPERTIES LINKER_LANGUAGE CXX)

if (NOT JSON_SCHEMA_GEN_ON_CONFIG)
    add_custom_target(SchemaGenerator
        WORKING_DIRECTORY
            ${CMAKE_CURRENT_SOURCE_DIR}
        DEPENDS
            ${SCHEMA_HDRS}
        COMMENT
            "Intermediate schema generator target"
        VERBATIM)
    add_dependencies(Schema SchemaGenerator)
endif()