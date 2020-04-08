# import HKCameraLibrary
add_library(HKCameraLibrary UNKNOWN IMPORTED)

# Need to set the directories path in CMAKE_PREFIX_PATH
# By default in Linux, it could be "/opt/MVS;/opt/MVS/lib/64"
find_library(HKCameraLibrary_LIB MvCameraControl)
find_path(HKCameraLibrary_INCLUDE MvCameraControl.h)
message("FOUND HKCameraLibrary: ${HKCameraLibrary_LIB}")
message("FOUND HKCameraLibrary Headers: ${HKCameraLibrary_INCLUDE}")

set_property(TARGET HKCameraLibrary PROPERTY IMPORTED_LOCATION "${HKCameraLibrary_LIB}")
set_property(TARGET HKCameraLibrary PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${HKCameraLibrary_INCLUDE}")

find_package(fmt CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(CUDA REQUIRED)

# The target wrap HKCameraLibrary
add_library(HKCamera)
target_sources(HKCamera
        PUBLIC
            HKCamera.cpp
            HKCamera.hpp
        )

set(CUDA_NVCC_FLAGS -G;-g;-std=c++11)
message(STATUS "CUDA_INCLUDE_DIRS = ${CUDA_INCLUDE_DIRS}")

target_include_directories(HKCamera PUBLIC ./)
target_link_libraries(HKCamera
        PUBLIC
            HKCameraLibrary
            spdlog::spdlog
            fmt::fmt
            opencv_core
            opencv_imgproc
            ${CUDA_LIBRARIES}
        )
