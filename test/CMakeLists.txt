# 获取所有 .cpp 文件
# file(GLOB SRC ${CMAKE_CURRENT_SOURCE_DIR}/shared_memory/*.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/)
set (SRC ${CMAKE_CURRENT_SOURCE_DIR}/test_HMI.cpp  
    ${CMAKE_SOURCE_DIR}/common/src/Utilities/PeriodicTask.cpp
    )

# 创建可执行文件目标
add_executable(test ${SRC})

# 设置头文件路径
target_include_directories(test PRIVATE
    ${CMAKE_SOURCE_DIR}/common/include
    ${CMAKE_SOURCE_DIR}/third_party/include/readerwriterqueue/
)

# 链接库
target_link_libraries(test PRIVATE pthread robot_utility rt)

if(QNX)
    set(TARGET_INSTALL_PATH "/tmp/${PROJECT_NAME}/bin")
elseif(UNIX AND NOT ANDROID)
    set(TARGET_INSTALL_PATH "/opt/${PROJECT_NAME}/bin")
endif()

# 检查是否设置了目标路径，并添加安装规则
if(TARGET_INSTALL_PATH)
    install(TARGETS test DESTINATION ${TARGET_INSTALL_PATH})
endif()
