cmake_minimum_required(VERSION 3.8)
project(darwin_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(Torch_DIR /home/orin/.local/lib/python3.10/site-packages/torch/share/cmake/Torch)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(urdf REQUIRED)
find_package(Torch REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(hri_humanoid_interfaces REQUIRED)

include_directories(include)

add_executable(torchscript src/torchscript.cpp)
target_link_libraries(torchscript "${TORCH_LIBRARIES}")
target_include_directories(torchscript PRIVATE ${TORCH_INCLUDE_DIRS})
target_link_libraries(torchscript rclcpp::rclcpp)

ament_target_dependencies(torchscript rclcpp sensor_msgs nav_msgs urdf geometry_msgs  std_msgs trajectory_msgs)

add_executable(inference_service src/inference_service.cpp)
target_link_libraries(inference_service "${TORCH_LIBRARIES}")
target_include_directories(inference_service PRIVATE ${TORCH_INCLUDE_DIRS})
target_link_libraries(inference_service rclcpp::rclcpp)
ament_target_dependencies(inference_service rclcpp hri_humanoid_interfaces)


add_executable(dreamwaq_inference_service src/dreamwaq_inference_service.cpp)
target_link_libraries(dreamwaq_inference_service "${TORCH_LIBRARIES}")
target_include_directories(dreamwaq_inference_service PRIVATE ${TORCH_INCLUDE_DIRS})
target_link_libraries(dreamwaq_inference_service rclcpp::rclcpp)
ament_target_dependencies(dreamwaq_inference_service rclcpp hri_humanoid_interfaces geometry_msgs sensor_msgs)

add_executable(dreamwaq_inference_tcp_ip src/dreamwaq_inference_tcp_ip.cpp)
target_link_libraries(dreamwaq_inference_tcp_ip "${TORCH_LIBRARIES}")
target_include_directories(dreamwaq_inference_tcp_ip PRIVATE ${TORCH_INCLUDE_DIRS})
target_link_libraries(dreamwaq_inference_tcp_ip rclcpp::rclcpp)
ament_target_dependencies(dreamwaq_inference_tcp_ip rclcpp hri_humanoid_interfaces geometry_msgs sensor_msgs)

add_executable(test_joint_command src/test_joint_command.cpp)
ament_target_dependencies(test_joint_command rclcpp trajectory_msgs)

install(TARGETS
  torchscript
  test_joint_command
  inference_service
  dreamwaq_inference_service
  dreamwaq_inference_tcp_ip
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/openRB_traj.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch checkpoint meshes urdf params behaviors
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
