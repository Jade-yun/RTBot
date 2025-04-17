# 添加共享内存库
include(${CMAKE_SOURCE_DIR}/cmake/toolchain.cmake)

add_library(shared_memory STATIC src/shared_data.cpp)

# 设置头文件路径
target_include_directories(shared_memory PUBLIC ${CMAKE_SOURCE_DIR}/include)

# # 设置编译选项
# target_compile_features(shared_memory PRIVATE cxx_std_17)