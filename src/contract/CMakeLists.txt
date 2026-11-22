add_library(fork_spatial_contract
  spatial_state.cpp
  fault_injection.cpp
)
add_library(fork_spatial::contract ALIAS fork_spatial_contract)

target_include_directories(fork_spatial_contract
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(fork_spatial_contract PUBLIC cxx_std_20)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(fork_spatial_contract PRIVATE -Wall -Wextra -Wpedantic)
elseif(MSVC)
  target_compile_options(fork_spatial_contract PRIVATE /W4)
endif()
