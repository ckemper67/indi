INCLUDE_DIRECTORIES( ${INDI_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( "../../drivers/ccd" )
INCLUDE_DIRECTORIES( "../../drivers/telescope" )

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

ADD_EXECUTABLE(test_driver_snoop
    "${CMAKE_CURRENT_SOURCE_DIR}/../../drivers/telescope/telescope_simulator.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../drivers/telescope/scopesim_helper.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../drivers/ccd/ccd_simulator.cpp"
    test_driver_snoop.cpp
)

TARGET_COMPILE_DEFINITIONS(test_driver_snoop PRIVATE NO_DEVICE_SINGLETON)

TARGET_LINK_LIBRARIES(test_driver_snoop
    indidriver
    ${GTEST_BOTH_LIBRARIES}
    ${GMOCK_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

ADD_TEST(test_driver_snoop test_driver_snoop)
