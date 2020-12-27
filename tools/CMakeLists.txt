
include(TestMacro)

find_package(gflags REQUIRED)
find_package(fmt CONFIG REQUIRED)

test_target(
	NAME FetchConfig
	SRCS "FetchConfig.cpp"
	LIBS httplib::httplib gflags fmt::fmt
)

install(TARGETS FetchConfig DESTINATION bin)