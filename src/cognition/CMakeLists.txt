add_library(fork_spatial_cognition
  semantic_token.cpp
  behavior_memory.cpp
  edge_policy.cpp
)
add_library(fork_spatial::cognition ALIAS fork_spatial_cognition)

target_include_directories(fork_spatial_cognition
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(fork_spatial_cognition PUBLIC fork_spatial::runtime fork_spatial::contract)
target_compile_features(fork_spatial_cognition PUBLIC cxx_std_20)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(fork_spatial_cognition PRIVATE -Wall -Wextra -Wpedantic)
elseif(MSVC)
  target_compile_options(fork_spatial_cognition PRIVATE /W4)
endif()
