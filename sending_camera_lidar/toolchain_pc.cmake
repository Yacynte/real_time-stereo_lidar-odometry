# Native compilation for x86_64 PC
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

# Use system compilers
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)

# Add path for rplidar_sdk headers
# include_directories(${CMAKE_SOURCE_DIR}/include/rplidar_sdk/sdk/include)
# include_directories(${CMAKE_SOURCE_DIR}/include/rplidar_sdk/sdk/src)

# Specify the path to the RPLIDAR SDK
set(RPLIDAR_SDK_DIR ${CMAKE_SOURCE_DIR}/include/rplidar_sdk) # Update this path to the location of your RPLIDAR SDK
set(RPLIDAR_SDK_LIB_DIR ${RPLIDAR_SDK_DIR}/sdk)




