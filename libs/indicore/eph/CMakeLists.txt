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
