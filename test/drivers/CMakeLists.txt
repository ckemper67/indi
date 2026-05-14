INCLUDE_DIRECTORIES( ${INDI_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( "../../drivers/ccd" )
INCLUDE_DIRECTORIES( "../../drivers/ao/orion_steadystar" )

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

ADD_EXECUTABLE(test_orion_steadystar_derotation
    "${CMAKE_CURRENT_SOURCE_DIR}/../../drivers/ao/orion_steadystar/orion_steadystar_driver.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../drivers/ao/orion_steadystar/orion_steadystar_hardware.cpp"
    test_orion_steadystar_derotation.cpp
)

TARGET_LINK_LIBRARIES(test_orion_steadystar_derotation
    indidriver
    ${USB1_LIBRARIES}
    ${GTEST_BOTH_LIBRARIES}
    ${GMOCK_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

ADD_TEST(test_orion_steadystar_derotation test_orion_steadystar_derotation)
