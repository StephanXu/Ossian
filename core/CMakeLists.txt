add_library(OssianCore)

target_sources(OssianCore
		PUBLIC
			"src/Dispatcher.cpp"
			"src/SerialPort.cpp"
			"src/IOListener.cpp"
			"src/io/CAN.cpp"
			"src/io/UART.cpp"
			"src/io/I2C.cpp"
			"src/Utility.cpp")
target_include_directories(OssianCore
		PUBLIC
			"include"
		)

find_package(fmt CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(ZLIB REQUIRED)
find_package(mimalloc 1.6 REQUIRED)
find_package(Boost REQUIRED)

target_compile_features(OssianCore PUBLIC cxx_std_17)
target_link_libraries(OssianCore
		PRIVATE
			fmt::fmt
			spdlog::spdlog
			ZLIB::ZLIB
			mimalloc
			Boost::boost
			"$<$<BOOL:${ENABLE_GPERF}>:-lprofiler>"
		)

set(gcc_like_cxx "$<COMPILE_LANG_AND_ID:CXX,ARMClang,AppleClang,Clang,GNU>")
set(msvc_cxx "$<COMPILE_LANG_AND_ID:CXX,MSVC>")

target_compile_options(OssianCore
		PRIVATE
			"$<${gcc_like_cxx}:$<$<CONFIG:Debug>:-Wall;-ggdb;-g3>>"
			"$<${msvc_cxx}:$<$<CONFIG:Debug>:-W3>>"
		)
target_link_options(OssianCore
		PRIVATE
			"$<${gcc_like_cxx}:-pthread>"
		)

target_compile_definitions(OssianCore
        PUBLIC 
            "SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE"
			"$<$<BOOL:${ENABLE_GPERF}>:ENABLE_GPERF>"
        )