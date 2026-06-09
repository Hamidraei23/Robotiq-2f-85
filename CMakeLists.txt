cmake_minimum_required(VERSION 3.0.2)
project(robotiq_2f85_driver)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  std_msgs
  message_generation
)

# libmodbus via pkg-config (install: sudo apt install libmodbus-dev)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)

# ── Messages and services ──────────────────────────────────────────────────────
add_message_files(
  FILES
  GripperStatus.msg
)

add_service_files(
  FILES
  GripperMove.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# ── catkin package declaration ─────────────────────────────────────────────────
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robotiq_2f85_driver_lib
  CATKIN_DEPENDS roscpp std_srvs std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${MODBUS_INCLUDE_DIRS}
)

# ── Driver library (pure Modbus RTU, no ROS spin logic) ───────────────────────
add_library(robotiq_2f85_driver_lib
  src/robotiq_2f85_driver.cpp
)
target_link_libraries(robotiq_2f85_driver_lib
  ${catkin_LIBRARIES}
  ${MODBUS_LIBRARIES}
)
add_dependencies(robotiq_2f85_driver_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# ── ROS node ──────────────────────────────────────────────────────────────────
add_executable(robotiq_2f85_node
  src/robotiq_2f85_node.cpp
)
target_link_libraries(robotiq_2f85_node
  robotiq_2f85_driver_lib
  ${catkin_LIBRARIES}
)
add_dependencies(robotiq_2f85_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# ── Standalone pulse-latency tester (no ROS spin) ─────────────────────────────
add_executable(gripper_pulse
  src/gripper_pulse.cpp
)
target_link_libraries(gripper_pulse
  robotiq_2f85_driver_lib
  ${catkin_LIBRARIES}
  ${MODBUS_LIBRARIES}
)
add_dependencies(gripper_pulse
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
