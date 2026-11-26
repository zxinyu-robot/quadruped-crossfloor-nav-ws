cmake_minimum_required(VERSION 3.20)
project(legged_nav LANGUAGES CXX VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(legged_nav STATIC
    src/json.cpp
    src/task_intent.cpp
    src/spatial_state.cpp
    src/admission.cpp
    src/skills.cpp
    src/feedback.cpp
    src/runtime.cpp
    src/transport.cpp
    src/spatial_context.cpp
    src/vision_loop.cpp
)
target_include_directories(legged_nav PUBLIC include)

find_package(Threads REQUIRED)
target_link_libraries(legged_nav PUBLIC Threads::Threads)
target_compile_features(legged_nav PUBLIC cxx_std_20)

option(LEGGED_NAV_BUILD_TESTS "build tests" ON)
option(LEGGED_NAV_BUILD_DDS "build DDS type support (needs libddsc)" OFF)

if(LEGGED_NAV_BUILD_TESTS)
    enable_testing()
    add_executable(test_all tests/test_all.cpp)
    target_link_libraries(test_all PRIVATE legged_nav)
    target_compile_definitions(test_all PRIVATE
        LEGGED_NAV_CONTRACT_CASES="${CMAKE_CURRENT_SOURCE_DIR}/tests/contract_cases.json"
        TASK_GATE_CASES="${CMAKE_CURRENT_SOURCE_DIR}/tests/task_gate_cases.json")
    add_test(NAME legged_nav_all COMMAND test_all)
endif()

if(LEGGED_NAV_BUILD_DDS)
    add_subdirectory(dds)
endif()
