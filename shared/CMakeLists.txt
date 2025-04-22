# 添加共享内存库
include(${CMAKE_SOURCE_DIR}/cmake/toolchain.cmake)

add_library(shared_memory STATIC src/SharedMemoryManager.cpp)
# add_library(shared SHARED src/SharedMemoryManager.cpp.cpp)

# 设置头文件路径
target_include_directories(shared_memory PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(TARGETS shared_memory
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)