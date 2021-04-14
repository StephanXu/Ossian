
include(TestMacro)

find_package(gflags REQUIRED)
find_package(fmt CONFIG REQUIRED)

test_target(
	NAME FetchConfig
	SRCS "FetchConfig.cpp"
	LIBS gflags fmt::fmt
)

install(TARGETS FetchConfig DESTINATION bin)

test_target(
	NAME UploadLog
	SRCS "UploadLog.cpp" "../src/OnlineDebug.cpp"
	LIBS OssianCore gflags fmt::fmt signalrclient mimalloc
)

install(TARGETS UploadLog DESTINATION bin)