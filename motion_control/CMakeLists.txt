project(motion_control)

# EtherCAT 第三方库配置
set(EtherCAT_ROOT ${CMAKE_SOURCE_DIR}/third_party/etherlab)
list(APPEND CMAKE_MODULE_PATH "${EtherCAT_ROOT}/lib/cmake/ethercat")
include(${CMAKE_SOURCE_DIR}/cmake/ethercat-config.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/toolchain.cmake)

# 收集源文件
file(GLOB_RECURSE SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
    # ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

# 链接第三方库
# find_package(Threads REQUIRED)  # 确保线程支持

# 可执行文件
add_executable(${PROJECT_NAME} ${SRC})
# 头文件路径
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/shared/include
    ${CMAKE_SOURCE_DIR}/third_party/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME} PRIVATE
    EtherLab::ethercat 
    pthread 
    rt 
    shared_memory)