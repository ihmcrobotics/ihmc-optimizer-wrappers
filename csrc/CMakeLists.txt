project(ConvexOptimizationAdapter)
cmake_minimum_required(VERSION 2.8)

if(WIN32)
	set(CMAKE_C_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_MINSIZEREL} /MT")
	set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MT")
	set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} /MT")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
	include_directories("c:\\")
endif(WIN32)


include_directories(${JNI_INCLUDE_DIRS}) 

include_directories(qpOASES/include)
FILE(GLOB qpOASES_SOURCE "qpOASES/src/*.cpp")
LIST(APPEND qpOASES_SOURCE "OASESConstrainedQPSolver.cpp")

add_library(qpOASES ${qpOASES_SOURCE})
add_library(OASESConstrainedQPSolver SHARED  ${qpOASES_SOURCE})

include_directories(quadprog/src)
add_library(uQuadProg SHARED quadprog/src/uQuadProg++.cc QuadProgSolver.cpp)

if (NOT WIN32)
	target_link_libraries(OASESConstrainedQPSolver m qpOASES)
endif()

INSTALL(TARGETS   OASESConstrainedQPSolver RUNTIME DESTINATION resources LIBRARY DESTINATION resources)
INSTALL(TARGETS   uQuadProg RUNTIME DESTINATION resources LIBRARY DESTINATION resources)
