project(ConvexOptimizationAdapter)
cmake_minimum_required(VERSION 2.8)

if(WIN32)
	set(CMAKE_C_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_MINSIZEREL} /MT")
	set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MT")
	set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} /MT")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
	include_directories("c:\\")
endif(WIN32)


FIND_PACKAGE(JNI)
FIND_PACKAGE(SWIG REQUIRED)

set( IHMC_PACK us.ihmc.convexOptimization)
set( IHMC_PATH us/ihmc/convexOptimization)

include_directories(${JNI_INCLUDE_DIRS}) 
############## qpOASES ####################
include_directories(qpOASES/include)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__NO_COPYRIGHT__")
FILE(GLOB qpOASES_SOURCE "qpOASES/src/*.cpp")
LIST(APPEND qpOASES_SOURCE "OASESConstrainedQPSolver.cpp")
add_library(qpOASES ${qpOASES_SOURCE})

############# qpOASES - JNA ##############
add_library(OASESConstrainedQPSolver SHARED  ${qpOASES_SOURCE})
install(TARGETS  OASESConstrainedQPSolver RUNTIME DESTINATION resources LIBRARY DESTINATION ${CMAKE_SOURCE_DIR}/resources/${IHMC_PATH})
set_target_properties(OASESConstrainedQPSolver PROPERTIES SUFFIX ".jnilib")

############# qpOASES - SWIG #############

INCLUDE(${SWIG_USE_FILE})
SET(CMAKE_SWIG_FLAGS "")

SET_SOURCE_FILES_PROPERTIES(qpOASES.i PROPERTIES CPLUSPLUS ON)
SET_SOURCE_FILES_PROPERTIES(qpOASES.i PROPERTIES SWIG_FLAGS "-includeall")
SET_SOURCE_FILES_PROPERTIES(qpOASES.i PROPERTIES SWIG_FLAGS "-cpperraswarn")

SET( CMAKE_SWIG_FLAGS -package ${IHMC_PACK}.qpOASES)
SET(CMAKE_SWIG_OUTDIR ${CMAKE_SOURCE_DIR}/generated-src/${IHMC_PATH}/qpOASES)

SWIG_ADD_MODULE(qpOASESSwig   java qpOASES.i ${SRC_LIST})
SWIG_LINK_LIBRARIES(qpOASESSwig ${JAVA_LIBRARIES}  qpOASES )

install(TARGETS qpOASESSwig DESTINATION ${CMAKE_SOURCE_DIR}/resources/${IHMC_PATH})


############# quadProg ###################
include_directories(quadprog/src)
add_library(uQuadProg SHARED quadprog/src/uQuadProg++.cc QuadProgSolver.cpp)
INSTALL(TARGETS  uQuadProg RUNTIME DESTINATION resources LIBRARY DESTINATION ${CMAKE_SOURCE_DIR}/resources/${IHMC_PATH})
set_target_properties(uQuadProg PROPERTIES SUFFIX ".jnilib")
