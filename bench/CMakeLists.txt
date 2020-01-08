cmake_minimum_required (VERSION 3.8)

find_package(benchmark CONFIG REQUIRED)

add_executable (VideoBench VideoBench.cpp)
target_compile_features(VideoBench PUBLIC cxx_std_17)
target_link_libraries(VideoBench benchmark::benchmark_main)

