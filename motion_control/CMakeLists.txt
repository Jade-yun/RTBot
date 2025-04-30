
# 收集源文件
file(GLOB_RECURSE SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
    # ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

# 链接第三方库
# find_package(Threads REQUIRED)  # 确保线程支持

# 可执行文件
add_executable(motion_control ${SRC})
# 头文件路径
include_directories("/usr/include/eigen3")
include_directories("/usr/include/boost")
target_include_directories(motion_control PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/third_party/include
    ${CMAKE_SOURCE_DIR}/common/include
    ${CMAKE_SOURCE_DIR}/third_party/include/readerwriterqueue/
)

target_link_libraries(motion_control PRIVATE
    EtherLab::ethercat 
    pthread 
    rt 
    robot_utility)