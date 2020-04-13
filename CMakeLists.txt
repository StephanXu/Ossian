cmake_minimum_required(VERSION 3.13.0)

project(Ossian VERSION 1.0)

add_executable(Ossian)

find_package(OpenCV REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(mimalloc 1.6 REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(CUDA)

message(STATUS "CUDA_FOUND: ${CUDA_FOUND}")

add_subdirectory(proto)
add_subdirectory(core)
add_subdirectory(3rdparty)
add_subdirectory(bench)
add_subdirectory(tests)

target_compile_features(Ossian PUBLIC cxx_std_17)
target_sources(Ossian
        PRIVATE
            src/Aimbot.cpp
            src/ColorFilter.cpp
            src/main.cpp
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
            src/Gyro.cpp
            src/IOPeeker.cpp
            src/CameraPeeker.cpp)
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
            proto
            ${OpenCV_LIBS}
            "$<$<BOOL:${CUDA_FOUND}>:${CUDA_LIBRARIES}>"
            OssianCore
            Eigen3::Eigen
        )

set(gcc_like_cxx "$<COMPILE_LANG_AND_ID:CXX,ARMClang,AppleClang,Clang,GNU>")
set(msvc_cxx "$<COMPILE_LANG_AND_ID:CXX,MSVC>")
set(CUDA_NVCC_FLAGS -G;-g;-std=c++11)

target_compile_options(Ossian
        PRIVATE
            "$<${gcc_like_cxx}:$<$<CONFIG:Debug>:-Wall;-Wextra;-Wshadow;-Wformat=2;-Wunused;-ggdb;-g3>>"
            "$<${msvc_cxx}:$<$<CONFIG:Debug>:-W3>>"
        )
target_link_options(Ossian
        PRIVATE
            "$<${gcc_like_cxx}:-pthread>"
        )
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")

target_compile_definitions(Ossian PRIVATE "$<$<BOOL:${CUDA_FOUND}>:WITH_CUDA>")