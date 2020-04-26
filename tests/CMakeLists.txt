
add_executable (IOTest)
target_compile_features(IOTest PRIVATE cxx_std_17)
target_link_libraries(IOTest PRIVATE OssianCore spdlog::spdlog)
target_link_libraries(IOTest PRIVATE mimalloc)
target_sources(IOTest PRIVATE IOTest.cpp)

add_executable (WITGyroTest)
target_compile_features(WITGyroTest PRIVATE cxx_std_17)
target_link_libraries(WITGyroTest PRIVATE OssianCore spdlog::spdlog)
target_link_libraries(WITGyroTest PRIVATE mimalloc)
target_sources(WITGyroTest PRIVATE "WITGyroTest.cpp")

add_executable(RefereeTest)
target_compile_features(RefereeTest PRIVATE cxx_std_17)
target_link_libraries(RefereeTest PRIVATE OssianCore spdlog::spdlog signalrclient mimalloc)
target_sources(RefereeTest PRIVATE RefereeTest.cpp ../src/OnlineDebug.cpp)

add_executable(CameraTest)
target_compile_features(CameraTest PRIVATE cxx_std_17)
target_link_libraries(CameraTest PRIVATE OssianCore HKCamera spdlog::spdlog signalrclient mimalloc)
target_sources(CameraTest PRIVATE CameraTest.cpp ../src/OnlineDebug.cpp)

add_executable(DITest)
target_compile_features(DITest PRIVATE cxx_std_17)
target_link_libraries(DITest PRIVATE OssianCore HKCamera spdlog::spdlog signalrclient mimalloc)
target_sources(DITest PRIVATE DependencyInjection.cpp)

add_executable (IOIntervalTest)
target_compile_features(IOIntervalTest PRIVATE cxx_std_17)
target_link_libraries(IOIntervalTest PRIVATE OssianCore spdlog::spdlog signalrclient mimalloc)
target_sources(IOIntervalTest PRIVATE "IOIntervalTest.cpp" ../src/OnlineDebug.cpp)