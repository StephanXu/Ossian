cmake_minimum_required(VERSION 3.2)

# set project name
project(NautilusVision)

# set(CMAKE_BUILD_TYPE "Release")
# set thread library
if(CMAKE_COMPILER_IS_GNUCXX)
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} $ENV{CXXFLAGS} -O0 -Wall -g3 -ggdb") 
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} $ENV{CXXFLAGS} -O3 -Wall")

    add_definitions("-pthread")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")
endif(CMAKE_COMPILER_IS_GNUCXX)

message(${CMAKE_CXX_FLAGS_DEBUG})
message(${CMAKE_CXX_FLAGS_RELEASE})

# import NautilusVisionFramework
add_subdirectory(NautilusVision)
include_directories(${NVF_INCLUDE_DIR})

# import 3rdparty libraries
add_subdirectory(3rdparty)
include_directories(${3RDPARTY_DIR})
link_directories(${3RDPARTY_DIR})

# import OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
aux_source_directory(
    ./src DIR_SRC
)

add_executable(${PROJECT_NAME} ${DIR_SRC})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}
	${OpenCV_LIBS}
	${NVF_LIBS}
	HKCamera/MvCameraControl)