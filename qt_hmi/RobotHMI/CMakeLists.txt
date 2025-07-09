# 设置最低版本要求
cmake_minimum_required(VERSION 3.14)

# 项目名称和使用的语言
project(RobotHMI LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 查找 Qt 包并加载所需模块
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

include(${CMAKE_SOURCE_DIR}/cmake/toolchain.cmake)

# 添加可执行文件
add_executable(RobotHMI
    src/main.cpp
    src/mainwindow.cpp
    src/mainwindow.h
    src/mainwindow.ui
)
# file(GLOB_RECURSE SOURCES
#     src/*.cpp
#     src/*.h
# )

# file(GLOB UI_FILES
#     src/*.ui
# )

# # 自动生成 MOC 和 UI 文件
# qt5_wrap_ui(UIS_HDRS ${UI_FILES})
# qt5_wrap_cpp(MOC_SRCS ${SOURCES})

# # 添加可执行文件
# add_executable(RobotHMI
#     ${SOURCES}
#     ${UIS_HDRS}
#     ${MOC_SRCS}
# )

# 包含目录
target_include_directories(RobotHMI PRIVATE 
    shared/include
)

target_link_directories(${CMAKE_SOURCE_DIR}/install/lib)
# 链接库
target_link_libraries(RobotHMI PRIVATE 
    Qt5=5::Core Qt5::Gui Qt5::Widgets
    shared_memory   
)

# 自动处理 .ui 文件
qt_wrap_ui(RobotHMI UI_HEADERS src/mainwindow.ui)

# 如果有资源文件（如 application.qrc），添加如下：
# qt_add_resources(RobotHMI RESOURCES resources/application.qrc)

# 设置输出目录
set_target_properties(RobotHMI PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/build/debug
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/build/release
)

# 启用编译器警告
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(RobotHMI PRIVATE -Wall -Wextra)
endif()