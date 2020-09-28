
include(TestMacro)

find_package(gflags REQUIRED)

test_target(
	NAME FetchConfig
	SRCS "FetchConfig.cpp"
	LIBS httplib::httplib gflags
)
