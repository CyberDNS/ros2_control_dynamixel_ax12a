cmake_minimum_required(VERSION 3.16)
project(dynamixel_hw LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  dynamixel_sdk_vendor
)

# Find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

# Build
add_library(
  dynamixel_hw
  SHARED
  src/dynamixel_hw.cpp
  src/dynamixel_connection.cpp
  src/dynamixel_wheel.cpp
)
target_compile_features(dynamixel_hw PUBLIC cxx_std_17)
target_include_directories(dynamixel_hw PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/dynamixel_hw>
)


ament_target_dependencies(
  dynamixel_hw PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "DYNAMIXEL_HW_2_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface dynamixel_hw.xml)



# Install
install(
  DIRECTORY include/
  DESTINATION include/dynamixel_hw
)

install(TARGETS dynamixel_hw
  EXPORT export_dynamixel_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_dynamixel_hw HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
