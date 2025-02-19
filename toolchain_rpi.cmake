# RPi4 Cross Compilation Toolchain File
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(EIGEN_INCLUDE_DIR /usr/include/eigen3)
include_directories(${EIGEN_INCLUDE_DIR})
# Set cross-compilers
# set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
# set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)

# Set search paths
set(CMAKE_FIND_ROOT_PATH /usr/aarch64-linux-gnu)

# Search for programs in the host environment
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# Search for libraries and headers in the target environment
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# OpenCV
# set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4")

# # PCL
# set(PCL_DIR "/usr/lib/aarch64-linux-gnu/cmake/pcl")