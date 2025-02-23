# RPi4 Cross Compilation Toolchain File
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Use the default compilers on the Pi
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)