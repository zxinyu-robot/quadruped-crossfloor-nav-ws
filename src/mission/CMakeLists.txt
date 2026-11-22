add_library(fork_spatial_mission
  mission_orchestrator.cpp
)
add_library(fork_spatial::mission ALIAS fork_spatial_mission)

target_include_directories(fork_spatial_mission
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(fork_spatial_mission PUBLIC fork_spatial::runtime fork_spatial::contract)
target_compile_features(fork_spatial_mission PUBLIC cxx_std_20)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(fork_spatial_mission PRIVATE -Wall -Wextra -Wpedantic)
elseif(MSVC)
  target_compile_options(fork_spatial_mission PRIVATE /W4)
endif()
