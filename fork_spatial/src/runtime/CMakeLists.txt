add_library(fork_spatial_runtime
  spatial_runtime.cpp
  state_store.cpp
)
add_library(fork_spatial::runtime ALIAS fork_spatial_runtime)

target_include_directories(fork_spatial_runtime
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(fork_spatial_runtime PUBLIC cxx_std_20)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(fork_spatial_runtime PRIVATE -Wall -Wextra -Wpedantic)
elseif(MSVC)
  target_compile_options(fork_spatial_runtime PRIVATE /W4)
endif()
