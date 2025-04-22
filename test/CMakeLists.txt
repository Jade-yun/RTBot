project(test)

# 获取所有 .cpp 文件
file(GLOB SRC ${CMAKE_CURRENT_SOURCE_DIR}/shared_memory/*.cpp)

# 创建可执行文件目标
add_executable(shm_test ${SRC})

# 设置头文件路径
target_include_directories(shm_test PRIVATE
    ${CMAKE_SOURCE_DIR}/shared/include
)

# 链接库
target_link_libraries(shm_test PRIVATE shared_memory)