# ########## CCD Simulator ##############
SET(ccdsimulator_SRC
    ccd_simulator.cpp)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ERFA erfa)

add_executable(indi_simulator_ccd ${ccdsimulator_SRC})
target_link_libraries(indi_simulator_ccd indidriver)
if(ERFA_FOUND)
    message(STATUS "CCD simulator: ERFA found, enabling ERFA-based star positions")
    target_compile_definitions(indi_simulator_ccd PRIVATE HAVE_ERFA)
    target_include_directories(indi_simulator_ccd PRIVATE ${ERFA_INCLUDE_DIRS})
    target_link_libraries(indi_simulator_ccd ${ERFA_LDFLAGS})
else()
    message(STATUS "CCD simulator: ERFA not found, using libnova J2000 star positions")
endif()
install(TARGETS indi_simulator_ccd RUNTIME DESTINATION bin)

# ########## Guide Simulator ##############
SET(guidesimulator_SRC
    guide_simulator.cpp)

add_executable(indi_simulator_guide ${guidesimulator_SRC})
target_link_libraries(indi_simulator_guide indidriver)
install(TARGETS indi_simulator_guide RUNTIME DESTINATION bin)

# ########## Alpaca CCD ##############
SET(alpacaccd_SRC
    indi_alpaca_ccd.cpp)

add_executable(indi_alpaca_ccd ${alpacaccd_SRC})
target_link_libraries(indi_alpaca_ccd indidriver ${HTTPLIB_LIBRARY})
install(TARGETS indi_alpaca_ccd RUNTIME DESTINATION bin)

# ########## Seestar CCD ##############
SET(seestarccd_SRC
    indi_seestar_ccd.cpp
    indi_alpaca_ccd.cpp)

add_executable(indi_seestar_ccd ${seestarccd_SRC})
target_compile_definitions(indi_seestar_ccd PRIVATE INDI_ALPACA_CCD_NO_INSTANCE)
target_link_libraries(indi_seestar_ccd indidriver ${HTTPLIB_LIBRARY})
install(TARGETS indi_seestar_ccd RUNTIME DESTINATION bin)
