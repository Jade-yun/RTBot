
# toolchain.cmake
# set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
# set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")
# set(CMAKE_C_COMPILER /opt/EmbedSky/gcc-linaro-7.4.1-2019.02-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
# set(CMAKE_CXX_COMPILER /opt/EmbedSky/gcc-linaro-7.4.1-2019.02-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)

# # 设置目标系统信息
# set(CMAKE_SYSTEM_NAME Linux)
# set(CMAKE_SYSTEM_PROCESSOR aarch64)

# # 设置查找路径模式
# set(CMAKE_FIND_ROOT_PATH /opt/EmbedSky/gcc-linaro-7.4.1-2019.02-x86_64_aarch64-linux-gnu)
# set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
# set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# 打印编译器路径（可选）
message(STATUS "CMAKE_C_COMPILER: ${CMAKE_C_COMPILER}")
message(STATUS "CMAKE_CXX_COMPILER: ${CMAKE_CXX_COMPILER}")