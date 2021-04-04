cmake_minimum_required(VERSION 3.16.0)

project(Ossian VERSION 1.0)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

add_executable(Ossian)

find_package(OpenCV REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(mimalloc 1.6 REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(CUDA)

message(STATUS "CUDA_FOUND: ${CUDA_FOUND}")

add_subdirectory(schema)
add_subdirectory(core)
add_subdirectory(3rdparty)
add_subdirectory(bench)
add_subdirectory(tests)
add_subdirectory(tools)

target_compile_features(Ossian PUBLIC cxx_std_17)
target_sources(Ossian
        PRIVATE
            src/Aimbot.cpp
            src/ColorFilter.cpp
            src/Main.cpp
            src/Startup.cpp
            src/Utils.cpp
            src/WindmillDetection.cpp
            src/Gimbal.cpp
            src/Chassis.cpp
            src/OnlineDebug.cpp
            src/Remote.cpp
            src/Capacitor.cpp
            src/Gun.cpp
            src/Referee.cpp
            src/IOPeeker.cpp
            src/CameraPeeker.cpp
            src/Gyro.cpp
            src/GyroA204.cpp
            src/GyroA110.cpp
            src/TOFNooploop.cpp
            src/Phototube.cpp
            src/ClientGraphic.cpp
            src/DJICRCHelper.cpp
        )
target_precompile_headers(Ossian
        PRIVATE
            src/ColorFilter.hpp
            src/Startup.hpp
            src/Utils.hpp
            src/WindmillDetection.hpp
            src/OnlineDebug.hpp
            src/Remote.hpp
            src/Capacitor.hpp
            src/Referee.hpp
            src/IOPeeker.hpp
            src/CameraPeeker.hpp
            src/Gyro.hpp
            src/GyroA204.hpp
            src/GyroA110.hpp
            src/TOFNooploop.hpp
            src/Phototube.hpp
            src/ClientGraphic.hpp
            src/DJICRCHelper.hpp
            <mutex>
            <atomic>
            <tuple>
            <thread>
            <vector>
            <chrono>
            <array>
            <functional>
        )
target_include_directories(Ossian
        PRIVATE
            src
            "$<$<BOOL:${CUDA_FOUND}>:${CUDA_INCLUDE_DIRS}>"
        )
target_link_libraries(Ossian
        PRIVATE
            signalrclient
            HKCamera
            spdlog::spdlog
            fmt::fmt
            mimalloc
            Schema
            ${OpenCV_LIBS}
            "$<$<BOOL:${CUDA_FOUND}>:${CUDA_LIBRARIES}>"
            OssianCore
            Eigen3::Eigen
            "$<$<BOOL:${ENABLE_GPERF}>:-lprofiler>"
        )

if(NOT DEFINED VISION_ONLY)
  set(VISION_ONLY FALSE)
endif()
target_compile_definitions(Ossian PRIVATE "$<$<BOOL:${VISION_ONLY}>:VISION_ONLY>")

set(gcc_like_cxx "$<COMPILE_LANG_AND_ID:CXX,ARMClang,AppleClang,Clang,GNU>")
set(msvc_cxx "$<COMPILE_LANG_AND_ID:CXX,MSVC>")
set(CUDA_NVCC_FLAGS -G;-g;-std=c++11)
target_link_options(Ossian PRIVATE "-fsanitize=leak")
target_compile_options(Ossian
        PRIVATE
            "$<${gcc_like_cxx}:$<$<CONFIG:Debug>:-Wall;-ggdb;-g3>>"
            "$<${msvc_cxx}:$<$<CONFIG:Debug>:-W3>>"
        )
target_link_options(Ossian
        PRIVATE
            "$<${gcc_like_cxx}:-pthread>"
        )
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")

target_compile_definitions(Ossian 
        PRIVATE 
            "$<$<BOOL:${CUDA_FOUND}>:WITH_CUDA>"
            "SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE"
            "$<$<BOOL:${ENABLE_GPERF}>:ENABLE_GPERF>"
        )

# Copy AppSettings.json to build directory.
add_custom_target(AppSettingsFile ALL
    COMMAND ${CMAKE_COMMAND} -E copy
    ${CMAKE_CURRENT_SOURCE_DIR}/AppSettings.*
    ${CMAKE_CURRENT_BINARY_DIR}/
)
add_dependencies(Ossian AppSettingsFile)

# Install
install(TARGETS Ossian RUNTIME DESTINATION bin)
# [ATTENTION]: We need the AppSettings file copied correctly as install command
# can not directly copy them from source directory.
file(GLOB_RECURSE APP_SETTINGS_FILES ${CMAKE_CURRENT_SOURCE_DIR}/AppSettings.*.json)
install(FILES ${APP_SETTINGS_FILES} DESTINATION etc/Ossian)