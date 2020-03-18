
add_executable (IOTest)

target_compile_features(IOTest PRIVATE cxx_std_17)
target_link_libraries(IOTest PRIVATE OssianCore spdlog::spdlog)
target_link_libraries(IOTest PRIVATE mimalloc)
target_sources(IOTest PRIVATE IOTest.cpp)

add_executable(RefereeTest)

target_compile_features(RefereeTest PRIVATE cxx_std_17)
target_link_libraries(RefereeTest PRIVATE OssianCore spdlog::spdlog signalrclient mimalloc)
target_sources(RefereeTest PRIVATE RefereeTest.cpp ../src/OnlineDebug.cpp)