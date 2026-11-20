# CycloneDDS 原生类型支持（由 scripts/gen_dds_types.py 从 .msg 确定性生成）
#
# 这些 .c/.h 是**生成物**，不要手改；修改 .msg 后运行：
#   python3 scripts/gen_dds_types.py --msg <pkg>/msg/<Type>
#
# 只依赖 libddsc（+ Threads），不依赖 rcl / rclcpp / rmw。
option(LEGGED_NAV_BUILD_DDS "构建 DDS 类型支持与互通测试" OFF)

if(LEGGED_NAV_BUILD_DDS)
    enable_language(C)
    find_package(CycloneDDS REQUIRED)
    find_package(Threads REQUIRED)

    file(GLOB_RECURSE LEGGED_NAV_DDS_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/gen/*/*/*.c")

    add_library(legged_nav_dds_types STATIC ${LEGGED_NAV_DDS_SOURCES})
    target_include_directories(legged_nav_dds_types PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/gen")
    target_link_libraries(legged_nav_dds_types PUBLIC CycloneDDS::ddsc Threads::Threads)
    target_compile_features(legged_nav_dds_types PUBLIC c_std_99)

    if(LEGGED_NAV_BUILD_TESTS)
        add_executable(test_dds_type tests/test_dds_type.cpp)
        target_link_libraries(test_dds_type PRIVATE legged_nav_dds_types)
        add_test(NAME legged_nav_dds_type COMMAND test_dds_type)
    endif()

    # 互通实验工具（MVP）：需要一台跑着 ROS 2 的机器来做对端验证
    add_executable(interop_native lab/interop_native.cpp)
    target_link_libraries(interop_native PRIVATE legged_nav_dds_types)
endif()
