include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../libs/indibase)

set(orion_steadystar_SRC
    orion_steadystar_driver.cpp
    orion_steadystar_hardware.cpp
)

add_executable(indi_orion_steadystar ${orion_steadystar_SRC})
target_link_libraries(indi_orion_steadystar indidriver ${USB1_LIBRARIES})
install(TARGETS indi_orion_steadystar RUNTIME DESTINATION bin)
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    install(FILES 99-orion_steadystar.rules DESTINATION ${UDEVRULES_INSTALL_DIR})
endif()
