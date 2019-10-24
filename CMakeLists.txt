cmake_minimum_required(VERSION 3.2)

# set project name
project(RoboMaster)

# import NautilusVisionFramework
add_subdirectory(NautilusVision)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/NautilusVision/include)
set(NVF_LIBS NautilusVisionFramework)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g3 -ggdb") 
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

add_definitions("-pthread")

# test CXX17 support
add_definitions("-std=c++17")
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if(COMPILER_SUPPORTS_CXX17)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else()
     message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

# find libraries
# set(OpenCV_STATIC ON)
find_package(OpenCV REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})

aux_source_directory(
    ./src DIR_SRC
)

add_executable(${PROJECT_NAME} ${DIR_SRC})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${NVF_LIBS})