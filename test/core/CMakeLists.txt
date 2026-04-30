

SET (test_base64_SRCS
	test_base64.cpp
)
ADD_EXECUTABLE(test_base64
    ${test_base64_SRCS}
)
TARGET_LINK_LIBRARIES(test_base64
    indiclient
    ${GTEST_BOTH_LIBRARIES}
    ${GMOCK_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
ADD_TEST(test_base64 test_base64)

SET (test_property_class_SRCS
    test_property_class.cpp
)
ADD_EXECUTABLE(test_property_class
    ${test_property_class_SRCS}
)
TARGET_LINK_LIBRARIES(test_property_class
	indiclient
	${GTEST_BOTH_LIBRARIES}
	${GMOCK_LIBRARIES}
	${CMAKE_THREAD_LIBS_INIT}
)
ADD_TEST(test_property_class test_property_class)

SET (test_horizontal_truth_SRCS
    test_horizontal_truth.cpp
)
ADD_EXECUTABLE(test_horizontal_truth
    ${test_horizontal_truth_SRCS}
)
TARGET_LINK_LIBRARIES(test_horizontal_truth
    indidriver
    ${GTEST_BOTH_LIBRARIES}
    ${GMOCK_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
TARGET_COMPILE_DEFINITIONS(test_horizontal_truth PRIVATE
    TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/test/data"
)
ADD_TEST(test_horizontal_truth test_horizontal_truth)

SET (test_engine_comparison_SRCS
    test_engine_comparison.cpp
)
ADD_EXECUTABLE(test_engine_comparison
    ${test_engine_comparison_SRCS}
)
TARGET_LINK_LIBRARIES(test_engine_comparison
    indidriver
    ${GTEST_BOTH_LIBRARIES}
    ${GMOCK_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
TARGET_COMPILE_DEFINITIONS(test_engine_comparison PRIVATE
    TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/test/data"
)
ADD_TEST(test_engine_comparison test_engine_comparison)

SET (test_eph_library_SRCS
    test_eph_library.cpp
)
ADD_EXECUTABLE(test_eph_library
    ${test_eph_library_SRCS}
)
TARGET_LINK_LIBRARIES(test_eph_library
    eph
    PkgConfig::ERFA
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
# Ensure it gets the same data dir definition
TARGET_COMPILE_DEFINITIONS(test_eph_library PRIVATE INDI_DATA_DIR="${CMAKE_SOURCE_DIR}/libs/indicore")

ADD_TEST(test_eph_library test_eph_library)

SET (test_rotator_limits_SRCS
    test_rotator_limits.cpp
)
ADD_EXECUTABLE(test_rotator_limits
    ${test_rotator_limits_SRCS}
)
TARGET_LINK_LIBRARIES(test_rotator_limits   
    ${GTEST_BOTH_LIBRARIES}
	${GMOCK_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
ADD_TEST(test_rotator_limits test_rotator_limits)


