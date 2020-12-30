
include(TestMacro)

find_package(gflags REQUIRED)
find_package(fmt CONFIG REQUIRED)

test_target(
	NAME FetchConfig
	SRCS "FetchConfig.cpp"
	LIBS gflags fmt::fmt
)

install(TARGETS FetchConfig DESTINATION bin)