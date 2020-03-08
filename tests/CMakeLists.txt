
add_executable (IOTest)

target_compile_features(IOTest PRIVATE cxx_std_17)
target_link_libraries(IOTest PRIVATE OssianCore spdlog::spdlog)
target_link_libraries(IOTest PRIVATE mimalloc)
target_sources(IOTest PRIVATE IOTest.cpp)
