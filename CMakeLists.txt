cmake_minimum_required(VERSION 3.8)
project(robot_omni)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================================================================
# 1. TÌM CÁC THƯ VIỆN CẦN THIẾT
# =====================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)          # <--- THÊM ĐỂ BUILD NÉ GHẾ
find_package(behaviortree_cpp REQUIRED)  # <--- THÊM ĐỂ BUILD NÉ GHẾ

# =====================================================================
# 2. BIÊN DỊCH CODE C++ (PLUGIN NÉ VẬT CẢN)
# =====================================================================
add_library(is_space_clear_plugin SHARED src/is_space_clear.cpp)

target_compile_definitions(is_space_clear_plugin PRIVATE BT_PLUGIN_EXPORT)

target_include_directories(is_space_clear_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(is_space_clear_plugin rclcpp behaviortree_cpp nav_msgs geometry_msgs)

install(TARGETS is_space_clear_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# =====================================================================
# 3. CÀI ĐẶT FILE PYTHON (Giữ nguyên của bạn, gộp lại cho gọn)
# =====================================================================
install(PROGRAMS
  scripts/scan_relay.py
  scripts/waypoint_recorder.py
  scripts/mission_control.py
  scripts/keepout_generator.py
  scripts/check_camera_points.py
  DESTINATION lib/${PROJECT_NAME}
)

# =====================================================================
# 4. CÀI ĐẶT CÁC THƯ MỤC MÔ PHỎNG VÀ NAV2
# =====================================================================
install(
  DIRECTORY launch config meshes urdf worlds maps behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()