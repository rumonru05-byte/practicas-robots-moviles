cmake_minimum_required(VERSION 3.8)
project(seg_tray)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Buscar dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Nodos
add_executable(nav_p2p src/nav_p2p.cpp)
ament_target_dependencies(nav_p2p rclcpp geometry_msgs nav_msgs)

# Comentado para evitar conflictos con el nodo de navegación global

# add_executable(corr_nav src/corr_nav.cpp)
# ament_target_dependencies(corr_nav rclcpp geometry_msgs nav_msgs sensor_msgs)

# Instalar ejecutables
install(TARGETS
  # nav_p2p
  corr_nav
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar archivos de soporte (Incluyendo 'scenes')
install(DIRECTORY
  launch
  config
  scenes
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()