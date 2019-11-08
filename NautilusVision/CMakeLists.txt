cmake_minimum_required(VERSION 3.2)

project(NautilusVisionFramework)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g3 -ggdb") 
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

# set thread library
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions("-pthread")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")
endif(CMAKE_COMPILER_IS_GNUCXX)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src DIR_SRC)

add_library(NautilusVisionFramework ${DIR_SRC})