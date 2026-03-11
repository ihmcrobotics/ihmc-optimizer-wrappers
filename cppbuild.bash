#!/bin/bash
# This build script is designed to work on Linux and Windows. For Windows, run from a bash shell launched with launchBashWindows.bat

REPO_ROOT=$(pwd)

# Clean and recreate build dir
rm -rf build generated-src
mkdir build

########################################
# Install boost into build dir
########################################
cd build
echo "Downloading Boost 1.90.0 into build/boost"
BOOST_VERSION=1_90_0
BOOST_ARCHIVE="boost_${BOOST_VERSION}.zip"
BOOST_DIR="boost_$BOOST_VERSION"

if [ ! -f "${BOOST_ARCHIVE}" ]; then
  curl -L "https://archives.boost.io/release/1.90.0/source/boost_1_90_0.zip" -o "${BOOST_ARCHIVE}"
fi
unzip -q "${BOOST_ARCHIVE}"
BOOST_ROOT="$(pwd)/$BOOST_DIR"
cd $REPO_ROOT

########################################
# Configure with CMake
########################################
cd build
if [ "${MAC_CROSS_COMPILE_ARM:-0}" == "1" ]; then
  cmake -DCMAKE_BUILD_TYPE=Release \
        -DBOOST_ROOT="$BOOST_ROOT" \
        -DCMAKE_PREFIX_PATH="$BOOST_ROOT" \
        -DCMAKE_OSX_ARCHITECTURES="arm64" \
        ..
elif [ "${LINUX_CROSS_COMPILE_ARM:-0}" == "1" ]; then
  cmake -DCMAKE_BUILD_TYPE=Release \
        -DBOOST_ROOT="$BOOST_ROOT" \
        -DCMAKE_PREFIX_PATH="$BOOST_ROOT" \
        -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc \
        -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ \
        -DCMAKE_FIND_ROOT_PATH=/usr/aarch64-linux-gnu \
        -DCMAKE_PROGRAM_PATH=/usr/aarch64-linux-gnu/bin \
        ..
else
  cmake -DCMAKE_BUILD_TYPE=Release \
        -DBOOST_ROOT="$BOOST_ROOT" \
        -DCMAKE_PREFIX_PATH="$BOOST_ROOT" \
        ..
fi
cmake --build .
cd "$REPO_ROOT"

#### Copy shared libs to resources ####
cd build
# Linux
mkdir -p ../src/main/resources/ihmc-optimizer-wrappers/native/linux-arm64
mkdir -p ../src/main/resources/ihmc-optimizer-wrappers/native/linux-x86_64
if [ -f "csrc/libIHMCOASESConstrainedQPSolver_rel.so" ]; then
  if [ "${LINUX_CROSS_COMPILE_ARM:-0}" == "1" ]; then
    cp csrc/libIHMCOASESConstrainedQPSolver_rel.so ../src/main/resources/ihmc-optimizer-wrappers/native/linux-arm64/libIHMCOASESConstrainedQPSolver_rel-arm64.so
  else
    cp csrc/libIHMCOASESConstrainedQPSolver_rel.so ../src/main/resources/ihmc-optimizer-wrappers/native/linux-x86_64/
  fi
fi
if [ -f "csrc/libOASESConstrainedQPSolver_rel.so" ]; then
  if [ "${LINUX_CROSS_COMPILE_ARM:-0}" == "1" ]; then
    cp csrc/libOASESConstrainedQPSolver_rel.so ../src/main/resources/ihmc-optimizer-wrappers/native/linux-arm64/libOASESConstrainedQPSolver_rel-arm64.so
  else
    cp csrc/libOASESConstrainedQPSolver_rel.so ../src/main/resources/ihmc-optimizer-wrappers/native/linux-x86_64/
  fi
fi
if [ -f "csrc/libuQuadProg_rel.so" ]; then
  if [ "${LINUX_CROSS_COMPILE_ARM:-0}" == "1" ]; then
    cp csrc/libuQuadProg_rel.so ../src/main/resources/ihmc-optimizer-wrappers/native/linux-arm64/libuQuadProg_rel-arm64.so
  else
    cp csrc/libuQuadProg_rel.so ../src/main/resources/ihmc-optimizer-wrappers/native/linux-x86_64/
  fi
fi
# macOS
mkdir -p ../src/main/resources/ihmc-optimizer-wrappers/native/macos-arm64
mkdir -p ../src/main/resources/ihmc-optimizer-wrappers/native/macos-x86_64
if [ -f "csrc/libIHMCOASESConstrainedQPSolver_rel.dylib" ]; then
  if [ "${MAC_CROSS_COMPILE_ARM:-0}" == "1" ]; then
    cp csrc/libIHMCOASESConstrainedQPSolver_rel.dylib \
      ../src/main/resources/ihmc-optimizer-wrappers/native/macos-arm64/libIHMCOASESConstrainedQPSolver_rel-arm64.dylib
  else
    cp csrc/libIHMCOASESConstrainedQPSolver_rel.dylib \
      ../src/main/resources/ihmc-optimizer-wrappers/native/macos-x86_64/
  fi
fi
if [ -f "csrc/libOASESConstrainedQPSolver_rel.dylib" ]; then
  if [ "${MAC_CROSS_COMPILE_ARM:-0}" == "1" ]; then
    cp csrc/libOASESConstrainedQPSolver_rel.dylib \
      ../src/main/resources/ihmc-optimizer-wrappers/native/macos-arm64/libOASESConstrainedQPSolver_rel-arm64.dylib
  else
    cp csrc/libOASESConstrainedQPSolver_rel.dylib \
      ../src/main/resources/ihmc-optimizer-wrappers/native/macos-x86_64/
  fi
fi
if [ -f "csrc/libuQuadProg_rel.dylib" ]; then
  if [ "${MAC_CROSS_COMPILE_ARM:-0}" == "1" ]; then
    cp csrc/libuQuadProg_rel.dylib \
      ../src/main/resources/ihmc-optimizer-wrappers/native/macos-arm64/libuQuadProg_rel-arm64.dylib
  else
    cp csrc/libuQuadProg_rel.dylib \
      ../src/main/resources/ihmc-optimizer-wrappers/native/macos-x86_64/
  fi
fi
cd "$REPO_ROOT"
