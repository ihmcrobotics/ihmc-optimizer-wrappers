cmake_minimum_required(VERSION 2.6)
project(ConvexOptimizationAdapter)

set(CMAKE_MINSIZEREL_POSTFIX _msz)
set(CMAKE_RELEASE_POSTFIX _rel)
set(CMAKE_DEBUG_POSTFIX _dbg)


if(WIN32)
	set(JDK "C:/Program Files/Java/jdk1.7.0_45")
	set(JAVA_AWT_INCLUDE_PATH "${JDK}/include")
	set(JAVA_AWT_LIBRARY "${JDK}/jawt.lib")
	set(JAVA_INCLUDE_PATH "${JDK}/include")
	set(JAVA_INCLUDE_PATH2 "${JDK}/include/win32")
	set(JAVA_JVM_LIBRARY "${JDK}/lib/jvm.lib")
endif()

add_subdirectory(csrc)
