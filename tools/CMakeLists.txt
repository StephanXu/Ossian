
include(TestMacro)

find_package(gflags REQUIRED)

test_target(
	NAME FetchConfig
	SRCS "FetchConfig.cpp"
	LIBS httplib::httplib gflags
)

test_target(
	NAME UploadLog
	SRCS "UploadLog.cpp"
	LIBS httplib::httplib gflags
)