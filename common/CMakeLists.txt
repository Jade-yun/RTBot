# Source/Include files
include_directories(".")
include_directories("include/" )                 # common includes
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/third_party/include/readerwriterqueue/)
file(GLOB_RECURSE sources "src/*.cpp")                   # common library cpp files

# Library
add_library(robot_utility STATIC ${sources})       # produce a library used by sim/robot