cmake_minimum_required(VERSION 3.8)
project(rover2_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)

# Check for lgpio library (Pi 5 GPIO)
find_library(LGPIO_LIBRARY lgpio)
if(LGPIO_LIBRARY)
  message(STATUS "Found lgpio: ${LGPIO_LIBRARY}")
  add_definitions(-DHAS_LGPIO)
else()
  message(WARNING "lgpio not found - building with mock GPIO for testing")
endif()

# Hardware interface library
add_library(rover2_hardware SHARED
  src/rover2_hardware.cpp
  src/gpio_motor.cpp
  src/gpio_encoder.cpp
)

target_include_directories(rover2_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rover2_hardware
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

if(LGPIO_LIBRARY)
  target_link_libraries(rover2_hardware ${LGPIO_LIBRARY})
endif()

# Export hardware interface plugin
pluginlib_export_plugin_description_file(hardware_interface rover2_base.xml)

# Install
install(
  TARGETS rover2_hardware
  EXPORT rover2_hardware
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES rover2_base.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(rover2_hardware)
ament_export_targets(rover2_hardware HAS_LIBRARY_TARGET)

ament_package()
