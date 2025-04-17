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

set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")

# 添加可执行文件
add_executable(RobotHMI
    src/main.cpp
    src/mainwindow.cpp
    src/mainwindow.h
    src/mainwindow.ui
)

# 包含目录
target_include_directories(RobotHMI PRIVATE src)

# 链接 Qt 库
target_link_libraries(RobotHMI PRIVATE Qt5=5::Core Qt5::Gui Qt5::Widgets)

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