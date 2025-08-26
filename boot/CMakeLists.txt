cmake_minimum_required(VERSION 3.8)
project(boot)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# =============================
# Install resources
# =============================

# Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Worlds
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# Models
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Config
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# RViz config
install(DIRECTORY rviz_config
  DESTINATION share/${PROJECT_NAME}
)

# Maps
install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

# =============================
# Env hook for GAZEBO_MODEL_PATH
# =============================

# Install env hook script
install(
  FILES env_hook/gazebo_model_path.sh
  DESTINATION share/${PROJECT_NAME}/environment
)


# =============================
ament_package()
