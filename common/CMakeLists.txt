# Source/Include files
include_directories(".")
include_directories("include/" )                 # common includes
include_directories("/usr/include/boost")
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
file(GLOB_RECURSE sources "src/*.cpp")                   # common library cpp files

# Library
add_library(robot_utility SHARED ${sources})       # produce a library used by sim/robot