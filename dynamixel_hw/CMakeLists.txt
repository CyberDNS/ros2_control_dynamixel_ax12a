cmake_minimum_required(VERSION 3.16)
project(dynamixel_hw LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# ---- Dependencies -----------------------------------------------------------
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  dynamixel_sdk_vendor
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# ---- Library ---------------------------------------------------------------
add_library(dynamixel_hw SHARED
  src/dynamixel_hw.cpp
  src/dynamixel_connection.cpp
  src/dynamixel_wheel.cpp
)

target_compile_features(dynamixel_hw PUBLIC cxx_std_17)

# Public headers live in <pkg>/include/...
# (your previous BUILD_INTERFACE path pointed to hardware/include — fixed)
target_include_directories(dynamixel_hw PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dynamixel_hw PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# 🔑 Link against the exported, namespaced target from the vendor package
# (this avoids raw -l flags and fixes the "cannot find -ldynamixel_sdk_cpp" error)
target_link_libraries(dynamixel_hw PUBLIC
  dynamixel_sdk_vendor::dynamixel_sdk_cpp
)

target_compile_definitions(${PROJECT_NAME} PRIVATE DYNAMIXEL_HW_2_BUILDING_DLL)

# ---- Plugin export ----------------------------------------------------------
pluginlib_export_plugin_description_file(hardware_interface dynamixel_hw.xml)

# ---- Install ---------------------------------------------------------------
install(DIRECTORY include/ DESTINATION include)

install(TARGETS dynamixel_hw
  EXPORT export_dynamixel_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ---- Tests ------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_position_unwrapper test/test_position_unwrapper.cpp)
  target_include_directories(test_position_unwrapper PRIVATE include)
  ament_add_gtest(test_velocity_loop test/test_velocity_loop.cpp)
  target_include_directories(test_velocity_loop PRIVATE include)
  ament_add_gtest(test_voltage_monitor test/test_voltage_monitor.cpp)
  target_include_directories(test_voltage_monitor PRIVATE include)
endif()

# ---- Ament exports ----------------------------------------------------------
ament_export_targets(export_dynamixel_hw HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_libraries(dynamixel_hw)

ament_package()