# Native compilation for x86_64 PC
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

# Use system compilers
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)

# Add path for rplidar_sdk headers
# include_directories(${CMAKE_SOURCE_DIR}/include/rplidar_sdk/sdk/include)
# include_directories(${CMAKE_SOURCE_DIR}/include/rplidar_sdk/sdk/src)






