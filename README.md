# Java wrapper for serveral optimization libraries

This library wraps qpOASES C/C++ library in Java. 


## Compile native code

For your convenience compiled libraries are placed in the resources directory. It's only necessary to compile native code if you want to change things or port to different platforms.

### Linux & Mac
- Install OpenJDK (>6), SWIG
- download qpOASES source (if not included in IHMCOptimizerWrappers)
  	- go to qpOASES website  https://projects.coin-or.org/qpOASES/wiki/QpoasesDownload
	- download the source code into ./csrc/qpOASES/ such as you have the strucure ./csrc/qpOASES/{src, include}
- Go to IHMCOptimizerWrapperss directory
	- ./build_native_libs.sh

### Windows
- Install Visual Studio Express 2013 for Windows Desktop
	- Continue while this is downloading and get some coffee
- Install the 64 bit JDK for Windows (>6)
- Install SWIG, put path to swig binary into %PATH% variable
- Install CMake using the installer http://www.cmake.org/download/
	- No binaries have been released as of 05/09/2014 yet. Get the swigwin 3.0.2 binary and replace Lib/java/various.i with https://github.com/swig/swig/blob/master/Lib/java/various.i
- download qpOASES source (if not included in IHMCOptimizerWrappers)
  	- go to qpOASES website  https://projects.coin-or.org/qpOASES/wiki/QpoasesDownload
	- download the source code into ./csrc/qpOASES/ such as you have the strucure ./csrc/qpOASES/{src, include}
- Start the cmake-gui
	- Point source directory to IHMCOptimizerWrapperss sources
	- Point build directory to [sources]/build
	- Configure
		- Choose the Visual Studio 12 2014 Win64 generator
		- Choose native toolchain
		- Set all paths correctly
		- Configure
	- Generate
- Go to IHMCOptimizerWrapperss/build
	- Double click on ALL_BUILD.vcxproj
	- Wait for VS2013 to start
	- Select "Release" build type
	- Right click ALL_BUILD and select build
		- Ignore the LNK2019 errors
	- Right click INSTALL and select build

## Publishing

To publish to Nexus, add the following lines to ~/.gradle/gradle.properties

nexus_username=[username]
nexus_password=[password]


Replace [username] and [password] with your nexus credentials

Then execute 

gradle publish
