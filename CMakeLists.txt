cmake_minimum_required(VERSION 3.12.4)

# set project name
project(NautilusVision)

# set thread library
if(CMAKE_COMPILER_IS_GNUCXX)
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} $ENV{CXXFLAGS} -O0 -Wall -g3 -ggdb") 
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} $ENV{CXXFLAGS} -O3 -Wall")

    add_definitions("-pthread")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")
endif(CMAKE_COMPILER_IS_GNUCXX)

# import OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# import fmt
find_package(fmt CONFIG REQUIRED)

# import spdlog
find_package(spdlog CONFIG REQUIRED)

if (WIN32)
	include_directories(C:/Users/xuzih/Documents/workspace/libs/mimalloc-1.2.2/include)
	link_directories(C:/Users/xuzih/Documents/workspace/libs/mimalloc-1.2.2/out/msvc-x64/Release)
	set(mimalloc_LIBS mimalloc-override)
elseif (UNIX)
	find_package(mimalloc 1.2 REQUIRED)
	set(mimalloc_LIBS mimalloc)
endif()


# import NautilusVisionFramework
add_subdirectory(NautilusVisionFramework)
include_directories(${NVF_INCLUDE_DIR})

if(WIN32)
	set(HKCAMERA_INCLUDE_DIR 3rdparty/HKCamera/includes CACHE STRING INTERNAL)
	set(HKCAMERA_LIBS MvCameraControl CACHE STRING INTERNAL)
	include_directories(${HKCAMERA_INCLUDE_DIR})
	link_directories(3rdparty/HKCamera/libraries/win64)
elseif(UNIX)
	set(HKCAMERA_INCLUDE_DIR /opt/MVS/include CACHE STRING INTERNAL)
	set(HKCAMERA_LIBS MvCameraControl CACHE STRING INTERNAL)
	include_directories(${HKCAMERA_INCLUDE_DIR})
	link_directories(/opt/MVS/lib/aarch64)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
aux_source_directory(
    ./src DIR_SRC
)

add_executable(${PROJECT_NAME} ${DIR_SRC})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}
	spdlog::spdlog 
	fmt::fmt
	${mimalloc_LIBS}
	${HKCAMERA_LIBS} 
	${OpenCV_LIBS} 
	${NVF_LIBS})

add_subdirectory(bench)