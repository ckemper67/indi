set(eph_SOURCES
    bopr.c
    earth.c
    moon.c
    mooni.c
    moonc.c
    planc.c
    planc_indi.c
    planet.c
    plani.c
    rdplan.c
    rdplanq.c
)

add_library(eph STATIC ${eph_SOURCES})

# Include current directory to satisfy <sofa.h> requirement without code changes
target_include_directories(eph PRIVATE
    .
    ${ERFA_INCLUDE_DIRS}
)

# ########## eph_plan_bin ##############
# Offline tool: reads VSOP2013p#.dat ASCII files, writes VSOP2013_#.ctx binary
# contexts used at runtime by EphEngineFull / EphEngineINDI.
# Run manually after updating the VSOP2013 data files:
#   cd <build>/libs/indicore/eph && ./eph_plan_bin <path-to-dat-files>
add_executable(eph_plan_bin plan_bin.c)
target_include_directories(eph_plan_bin PRIVATE . ${ERFA_INCLUDE_DIRS})
target_link_libraries(eph_plan_bin eph ${M_LIB})
