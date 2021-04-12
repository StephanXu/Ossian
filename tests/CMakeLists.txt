
include(TestMacro)

test_target(
	NAME IOTest
	SRCS "IOTest.cpp"
	LIBS OssianCore spdlog::spdlog mimalloc
)

test_target(
	NAME WITGyroTest
	SRCS "WITGyroTest.cpp"
	LIBS OssianCore spdlog::spdlog mimalloc
)

test_target(
	NAME RefereeTest
	SRCS "RefereeTest.cpp" "../src/OnlineDebug.cpp"
	LIBS OssianCore spdlog::spdlog signalrclient mimalloc
)

test_target(
	NAME CameraTest
	SRCS "CameraTest.cpp" "../src/OnlineDebug.cpp"
	LIBS OssianCore HKCamera spdlog::spdlog signalrclient mimalloc
)

test_target(
	NAME DITest
	SRCS "DependencyInjection.cpp"
	LIBS OssianCore HKCamera spdlog::spdlog signalrclient mimalloc
)

test_target(
	NAME IOIntervalTest
	SRCS "IOIntervalTest.cpp" "../src/OnlineDebug.cpp"
	LIBS OssianCore spdlog::spdlog signalrclient mimalloc
)

test_target(
	NAME OnlineDebuggerTest
	SRCS "OnlineDebuggerTest.cpp" "../src/OnlineDebug.cpp"
	LIBS OssianCore spdlog::spdlog signalrclient mimalloc
)

test_target(
	NAME SensorTest
	SRCS "SensorTest.cpp" "../src/OnlineDebug.cpp"
	LIBS OssianCore spdlog::spdlog signalrclient mimalloc
)

test_target(
	NAME ConfigTest
	SRCS "ConfigTest.cpp"
	LIBS OssianCore spdlog::spdlog Schema mimalloc
)

test_target(
	NAME ServoTest
	SRCS "ServoMotorTest.cpp"
)
target_compile_definitions(ServoTest PRIVATE __linux__)