find_package(PkgConfig REQUIRED)
pkg_check_modules(ERFA REQUIRED erfa)

ADD_EXECUTABLE(test_erfa_vs_libnova
    test_erfa_vs_libnova.cpp
)

TARGET_INCLUDE_DIRECTORIES(test_erfa_vs_libnova PRIVATE
    ${CMAKE_SOURCE_DIR}/libs/indicore
    ${ERFA_INCLUDE_DIRS}
)

TARGET_LINK_LIBRARIES(test_erfa_vs_libnova
    indidriver
    ${ERFA_LDFLAGS}
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

ADD_TEST(test-erfa-vs-libnova test_erfa_vs_libnova)
