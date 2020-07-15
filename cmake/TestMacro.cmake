function(test_target)
	cmake_parse_arguments(TEST_TARGET "" "NAME" "SRCS;LIBS" ${ARGN})

	add_executable(${TEST_TARGET_NAME})
	target_compile_features(${TEST_TARGET_NAME} PRIVATE cxx_std_17)
	target_link_libraries(${TEST_TARGET_NAME} PRIVATE ${TEST_TARGET_LIBS})
	target_sources(${TEST_TARGET_NAME} PRIVATE ${TEST_TARGET_SRCS})
endfunction()