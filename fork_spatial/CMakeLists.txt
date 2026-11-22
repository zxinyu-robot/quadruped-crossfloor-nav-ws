cmake_minimum_required(VERSION 3.20)
project(
  fork_spatial_runtime
  VERSION 0.1.0
  DESCRIPTION "Semantic-state, behavior-memory and edge continual-learning runtime"
  LANGUAGES CXX
)

include(CTest)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(FSR_BUILD_EXAMPLES "Build the runtime demo" ON)

add_subdirectory(src)

if(FSR_BUILD_EXAMPLES OR BUILD_TESTING)
  add_subdirectory(examples/cpp)
endif()
