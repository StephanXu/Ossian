
add_executable (VideoBench VideoBench.cpp)

find_package(benchmark CONFIG REQUIRED)
target_compile_features(VideoBench PRIVATE cxx_std_17)
target_sources(VideoBench
        PRIVATE
            VideoBench.cpp
        )
target_link_libraries(VideoBench
        PRIVATE
            proto
            benchmark::benchmark_main
        )

