# Set up ARGoS compilation information
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})
link_directories(
	${CMAKE_BINARY_DIR}/src
    )

add_library(tutti_st_loopfunc SHARED TuttiStLoopFunc.h TuttiStLoopFunc.cpp)
target_link_libraries(tutti_st_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(tutti_ag_loopfunc SHARED TuttiAgLoopFunc.h TuttiAgLoopFunc.cpp)
target_link_libraries(tutti_ag_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(tutti_fr_loopfunc SHARED TuttiFrLoopFunc.h TuttiFrLoopFunc.cpp)
target_link_libraries(tutti_fr_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(xmas_sw_loopfunc SHARED XmasSwLoopFunc.h XmasSwLoopFunc.cpp)
target_link_libraries(xmas_sw_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(easter_sw_loopfunc SHARED EasterSwLoopFunc.h EasterSwLoopFunc.cpp)
target_link_libraries(easter_sw_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)


