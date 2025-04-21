cmake_minimum_required(VERSION 3.14)

# 设置项目名称
project(RTBotControl LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)  # 禁用 GNU 扩展

# set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
# set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")

# 设置构建类型（Debug 或 Release）
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()

set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)

# 设置库、可执行文件的输出目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# # 添加 install 规则，在构建完成后将文件拷贝到 install 目录
# install(TARGETS shared_memory
#     DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
# )

# # 为头文件设置安装路径
# install(
#     DIRECTORY ${CMAKE_SOURCE_DIR}/shared/include/ 
#     DESTINATION ${CMAKE_INSTALL_PREFIX}/include
# )

# 添加子模块
add_subdirectory(shared)          # 共享内存模块
# add_subdirectory(motion_control)  # 运动控制模块
add_subdirectory(test) 
# add_subdirectory(qt_hmi)        # HMI 模块