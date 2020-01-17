cmake_minimum_required(VERSION 3.2)

project(OssianCore)

# set thread library
if(CMAKE_COMPILER_IS_GNUCXX)
	set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g3 -ggdb") 
	set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

    add_definitions("-pthread")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")
endif(CMAKE_COMPILER_IS_GNUCXX)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src DIR_SRC)

find_package(fmt CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(Protobuf REQUIRED)
find_package(ZLIB REQUIRED)

if (WIN32)
	include_directories(C:/Users/xuzih/Documents/workspace/libs/mimalloc-1.2.2/include)
	link_directories(C:/Users/xuzih/Documents/workspace/libs/mimalloc-1.2.2/out/msvc-x64/Release)
	set(mimalloc_LIBS mimalloc-override)
elseif (UNIX)
	find_package(mimalloc 1.2 REQUIRED)
	set(mimalloc_LIBS mimalloc)
endif()

add_library(OssianCore ${DIR_SRC})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(OssianCore
	fmt::fmt
	spdlog::spdlog
	ZLIB::ZLIB
	protobuf::libprotobuf
	${mimalloc_LIBS})

set(OSSIAN_CORE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE STRING INTERNAL)
set(OSSIAN_CORE_LIBS OssianCore CACHE STRING INTERNAL)
