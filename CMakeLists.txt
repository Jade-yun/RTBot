cmake_minimum_required(VERSION 3.14)

# 包含交叉编译链文件 必须在 project() 之前
include(${CMAKE_SOURCE_DIR}/cmake/toolchain.cmake)

# 设置项目名称
project(RT-Robot LANGUAGES CXX)


# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)  # 禁用 GNU 扩展


if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build." FORCE)
endif()

# # 针对Release构建类型添加优化选项
# if(CMAKE_BUILD_TYPE STREQUAL "Release")
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
#     set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -DNDEBUG")
# endif()

set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install)

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
add_subdirectory(motion_control)  # 运动控制模块
add_subdirectory(common) 
add_subdirectory(test) 
# add_subdirectory(qt_hmi)        # HMI 模块
