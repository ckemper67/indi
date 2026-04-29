# ########## eph_threshold_analysis ##############
add_executable(eph_threshold_analysis eph_threshold_analysis.c)
target_include_directories(eph_threshold_analysis PRIVATE ${CMAKE_SOURCE_DIR}/libs/indicore)
target_link_libraries(eph_threshold_analysis eph ${M_LIB})

# ########## indi_eph_packer ##############
add_executable(indi_eph_packer indi_eph_packer.c)
target_include_directories(indi_eph_packer PRIVATE ${CMAKE_SOURCE_DIR}/libs/indicore)
target_link_libraries(indi_eph_packer eph ${M_LIB})

# ########## getINDI ##############
add_executable(indi_getprop getINDIproperty.c)

target_link_libraries(indi_getprop indicore eventloop ${NOVA_LIBRARIES} ${M_LIB} ${ZLIB_LIBRARY})

install(TARGETS indi_getprop RUNTIME DESTINATION bin)

# ########## setINDI ##############
add_executable(indi_setprop setINDIproperty.c)

target_link_libraries(indi_setprop indicore eventloop ${NOVA_LIBRARIES} ${M_LIB} ${ZLIB_LIBRARY})

install(TARGETS indi_setprop RUNTIME DESTINATION bin)

# ########## evalINDI ##############
add_executable(indi_eval compiler.c evalINDI.c)

target_link_libraries(indi_eval indicore eventloop ${NOVA_LIBRARIES} ${M_LIB} ${ZLIB_LIBRARY})

install(TARGETS indi_eval RUNTIME DESTINATION bin)

# ########## getINDIdevice ##############
add_executable(indi_getdevice getINDIdevice.c)

target_link_libraries(indi_getdevice indicore eventloop ${NOVA_LIBRARIES} ${M_LIB} ${ZLIB_LIBRARY})

install(TARGETS indi_getdevice RUNTIME DESTINATION bin)
