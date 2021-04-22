# Set up ARGoS compilation information
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})
link_directories(
	${CMAKE_BINARY_DIR}/src
    )

add_library(hab_st_loopfunc SHARED HabStLoopFunc.h HabStLoopFunc.cpp)
target_link_libraries(hab_st_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(hab_ag_loopfunc SHARED HabAgLoopFunc.h HabAgLoopFunc.cpp)
target_link_libraries(hab_ag_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(hab_hz_loopfunc SHARED HabHzLoopFunc.h HabHzLoopFunc.cpp)
target_link_libraries(hab_hz_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(hab_ren_loopfunc SHARED HabRenLoopFunc.h HabRenLoopFunc.cpp)
target_link_libraries(hab_ren_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(hab_dec_loopfunc SHARED HabDecLoopFunc.h HabDecLoopFunc.cpp)
target_link_libraries(hab_dec_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)



