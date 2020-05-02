
find_package(protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)
SET(PROTO_META_BASE_DIR ${CMAKE_CURRENT_BINARY_DIR})

# By default have GRPC_GENERATE_CPP macro pass -I to protoc
# for each directory where a proto file is referenced.
if(NOT DEFINED GRPC_GENERATE_CPP_APPEND_PATH)
  set(GRPC_GENERATE_CPP_APPEND_PATH TRUE)
endif()

function(GRPC_GENERATE_CPP SRCS HDRS DEST)
  if(NOT ARGN)
    message(SEND_ERROR "Error: GRPC_GENERATE_CPP() called without any proto files")
    return()
  endif()

  #find_program(_GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)
  set(_GRPC_CPP_PLUGIN_EXECUTABLE "$<TARGET_FILE:gRPC::grpc_cpp_plugin>")
  set(_PROTOBUF_PROTOC "$<TARGET_FILE:protobuf::protoc>")

  if(GRPC_GENERATE_CPP_APPEND_PATH)
    # Create an include path for each file specified
    foreach(FIL ${ARGN})
      get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
      get_filename_component(ABS_PATH ${ABS_FIL} PATH)
      list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
      if(${_contains_already} EQUAL -1)
          list(APPEND _protobuf_include_path -I ${ABS_PATH})
      endif()
    endforeach()
  else()
    set(_protobuf_include_path -I ${CMAKE_CURRENT_SOURCE_DIR})
  endif()

  if(DEFINED PROTOBUF_IMPORT_DIRS)
    foreach(DIR ${PROTOBUF_IMPORT_DIRS})
      get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
      list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
      if(${_contains_already} EQUAL -1)
          list(APPEND _protobuf_include_path -I ${ABS_PATH})
      endif()
    endforeach()
  endif()

  foreach(FIL ${ARGN})
    get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
    get_filename_component(FIL_WE ${FIL} NAME_WE)
  
    LIST(APPEND SRCS "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.grpc.pb.cc")
    LIST(APPEND HDRS "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.grpc.pb.h")

    add_custom_command(
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.grpc.pb.cc" "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.grpc.pb.h"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --grpc_out "${DEST}"
        ${_protobuf_include_path}
        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        "${FIL}"
      DEPENDS "${FIL}")
  endforeach()

  set_source_files_properties(${${SRCS}} ${${HDRS}} PROPERTIES GENERATED TRUE)
  set(${SRCS} ${${SRCS}} PARENT_SCOPE)
  set(${HDRS} ${${HDRS}} PARENT_SCOPE)
endfunction()

function(PROTOBUF_GENERATE_CPP SRCS HDRS DEST)
  if(NOT ARGN)
    message(SEND_ERROR "Error: GRPC_GENERATE_CPP() called without any proto files")
    return()
  endif()

  set(_PROTOBUF_PROTOC "$<TARGET_FILE:protobuf::protoc>")

  if(GRPC_GENERATE_CPP_APPEND_PATH)
    # Create an include path for each file specified
    foreach(FIL ${ARGN})
      get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
      get_filename_component(ABS_PATH ${ABS_FIL} PATH)
      list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
      if(${_contains_already} EQUAL -1)
          list(APPEND _protobuf_include_path -I ${ABS_PATH})
      endif()
    endforeach()
  else()
    set(_protobuf_include_path -I ${CMAKE_CURRENT_SOURCE_DIR})
  endif()

  if(DEFINED PROTOBUF_IMPORT_DIRS)
    foreach(DIR ${PROTOBUF_IMPORT_DIRS})
      get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
      list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
      if(${_contains_already} EQUAL -1)
          list(APPEND _protobuf_include_path -I ${ABS_PATH})
      endif()
    endforeach()
  endif()

  foreach(FIL ${ARGN})
    get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
    get_filename_component(FIL_WE ${FIL} NAME_WE)
  
    LIST(APPEND SRCS "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.cc")
    LIST(APPEND HDRS "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.h")

    add_custom_command(
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.cc" "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.h"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --cpp_out "${DEST}"
        ${_protobuf_include_path}
        "${FIL}"
      DEPENDS "${FIL}")
  endforeach()

  set_source_files_properties(${${SRCS}} ${${HDRS}} PROPERTIES GENERATED TRUE)
  set(${SRCS} ${${SRCS}} PARENT_SCOPE)
  set(${HDRS} ${${HDRS}} PARENT_SCOPE)
endfunction()

grpc_generate_cpp(GRPC_SRCS GRPC_HDRS ${PROTO_META_BASE_DIR}
  "${CMAKE_CURRENT_SOURCE_DIR}/Log.proto"
  "${CMAKE_CURRENT_SOURCE_DIR}/Config.proto"
)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_META_BASE_DIR}
  "${CMAKE_CURRENT_SOURCE_DIR}/Log.proto"
  "${CMAKE_CURRENT_SOURCE_DIR}/Config.proto"
)

add_library(proto "${PROTO_SRCS}" "${GRPC_SRCS}")
target_include_directories(proto
        PUBLIC
          "${PROTO_META_BASE_DIR}"
        )
target_sources(proto
        PUBLIC
          
          
        )
target_link_libraries(proto PUBLIC gRPC::grpc++ protobuf::libprotobuf)