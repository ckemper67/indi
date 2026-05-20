INCLUDE_DIRECTORIES( ${INDI_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( "../../drivers/ccd" )

ADD_EXECUTABLE(test_ccd_simulator
    "${CMAKE_CURRENT_SOURCE_DIR}/../../drivers/ccd/ccd_simulator.cpp"
    test_ccd_simulator.cpp
)

TARGET_LINK_LIBRARIES(test_ccd_simulator
    indidriver
    ${GTEST_BOTH_LIBRARIES}
    ${GMOCK_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

ADD_TEST(test_ccd_simulator test_ccd_simulator)

# Atmospheric refraction tests require ERFA as an oracle.
# Prefer the local build at ../erfa/ (sibling of the indi tree) when present;
# fall back to the system pkg-config erfa.
set(ERFA_LOCAL_INC "${CMAKE_SOURCE_DIR}/../erfa/src")
set(ERFA_LOCAL_LIB "${CMAKE_SOURCE_DIR}/../erfa/builddir/src/liberfa.dylib")
if(EXISTS "${ERFA_LOCAL_LIB}")
    set(ERFA_REFRACTION_INCLUDE "${ERFA_LOCAL_INC}")
    set(ERFA_REFRACTION_LIB     "${ERFA_LOCAL_LIB}")
    set(ERFA_REFRACTION_FOUND   TRUE)
else()
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(ERFA IMPORTED_TARGET erfa)
        if(ERFA_FOUND)
            set(ERFA_REFRACTION_INCLUDE "")
            set(ERFA_REFRACTION_LIB     PkgConfig::ERFA)
            set(ERFA_REFRACTION_FOUND   TRUE)
        endif()
    endif()
endif()

if(ERFA_REFRACTION_FOUND)
    ADD_EXECUTABLE(test_telescope_refraction
        test_telescope_refraction.cpp
    )
    TARGET_INCLUDE_DIRECTORIES(test_telescope_refraction PRIVATE
        ${CMAKE_SOURCE_DIR}/libs
        ${CMAKE_SOURCE_DIR}/libs/indibase
        ${CMAKE_SOURCE_DIR}/libs/indicore
        ${ERFA_REFRACTION_INCLUDE}
    )
    TARGET_LINK_LIBRARIES(test_telescope_refraction
        indidriver
        ${ERFA_REFRACTION_LIB}
        ${GTEST_BOTH_LIBRARIES}
        ${GMOCK_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
    )
    ADD_TEST(test_telescope_refraction test_telescope_refraction)
endif()
