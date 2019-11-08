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

# set thread library
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions("-pthread")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")
endif(CMAKE_COMPILER_IS_GNUCXX)


# find libraries
find_package(OpenCV REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})

aux_source_directory(
    ./src DIR_SRC
)

add_executable(${PROJECT_NAME} ${DIR_SRC})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${NVF_LIBS})