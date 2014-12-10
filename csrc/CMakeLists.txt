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

if(UNIX)
SET(CMAKE_CXX_FLAGS "-fPIC")
SET(CMAKE_C_FLAGS "-fPIC")
endif()

include_directories(${JNI_INCLUDE_DIRS}) 
############## qpOASES ####################
include_directories(qpOASES/include)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__NO_COPYRIGHT__")
FILE(GLOB qpOASES_SOURCE "qpOASES/src/*.cpp")
add_library(qpOASES ${qpOASES_SOURCE})

############# qpOASES - JNA ##############
add_library(OASESConstrainedQPSolver SHARED  "OASESConstrainedQPSolver.cpp")
target_link_libraries(OASESConstrainedQPSolver qpOASES)
install(TARGETS  OASESConstrainedQPSolver RUNTIME DESTINATION ${CMAKE_SOURCE_DIR}/resources/${IHMC_PATH} LIBRARY DESTINATION ${CMAKE_SOURCE_DIR}/resources/${IHMC_PATH})
if(DARWIN)
	set_target_properties(OASESConstrainedQPSolver PROPERTIES SUFFIX ".jnilib")
endif(DARWIN)

############# qpOASES - SWIG #############

INCLUDE(${SWIG_USE_FILE})
SET(CMAKE_SWIG_FLAGS "")

SET_SOURCE_FILES_PROPERTIES(qpOASES.i PROPERTIES CPLUSPLUS ON)
SET_SOURCE_FILES_PROPERTIES(qpOASES.i PROPERTIES SWIG_FLAGS "-includeall")
SET_SOURCE_FILES_PROPERTIES(qpOASES.i PROPERTIES SWIG_FLAGS "-cpperraswarn")

SET( CMAKE_SWIG_FLAGS -package ${IHMC_PACK}.qpOASES)
SET(CMAKE_SWIG_OUTDIR ${CMAKE_SOURCE_DIR}/generated-src/${IHMC_PATH}/qpOASES)

SWIG_ADD_MODULE(qpOASESSwig   java qpOASES.i)
SET_SOURCE_FILES_PROPERTIES(${SWIG_GENERATED_FILE_FULLNAME} PROPERTIES COMPILE_FLAGS "-fPIC")
SWIG_LINK_LIBRARIES(qpOASESSwig ${JAVA_LIBRARIES}  qpOASES )

install(TARGETS qpOASESSwig DESTINATION ${CMAKE_SOURCE_DIR}/resources/${IHMC_PATH})


############# quadProg ###################
include_directories(quadprog/src)
add_library(uQuadProg SHARED quadprog/src/uQuadProg++.cc QuadProgSolver.cpp)
INSTALL(TARGETS  uQuadProg RUNTIME DESTINATION ${CMAKE_SOURCE_DIR}/resources/${IHMC_PATH} LIBRARY DESTINATION ${CMAKE_SOURCE_DIR}/resources/${IHMC_PATH})
if(DARWIN)
	set_target_properties(uQuadProg PROPERTIES SUFFIX ".jnilib")
endif()
