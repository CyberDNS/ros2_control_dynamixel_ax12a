cmake_minimum_required(VERSION 3.16)
project(dynamixel_sdk_vendor)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
include(FetchContent)

set(DYNAMIXEL_SDK_GIT "https://github.com/ROBOTIS-GIT/DynamixelSDK.git")
set(DYNAMIXEL_SDK_TAG "jazzy")

FetchContent_Declare(dxl
  GIT_REPOSITORY ${DYNAMIXEL_SDK_GIT}
  GIT_TAG        ${DYNAMIXEL_SDK_TAG}
  GIT_SHALLOW    TRUE
  UPDATE_DISCONNECTED TRUE
)
FetchContent_MakeAvailable(dxl)

set(DXL_INC_DIR ${dxl_SOURCE_DIR}/c++/include)
set(DXL_SRC_DIR ${dxl_SOURCE_DIR}/c++/src/dynamixel_sdk)

set(DXL_SOURCES
  ${DXL_SRC_DIR}/packet_handler.cpp
  ${DXL_SRC_DIR}/protocol1_packet_handler.cpp    
  ${DXL_SRC_DIR}/protocol2_packet_handler.cpp  
  ${DXL_SRC_DIR}/port_handler.cpp
  ${DXL_SRC_DIR}/port_handler_linux.cpp
  ${DXL_SRC_DIR}/group_bulk_read.cpp
  ${DXL_SRC_DIR}/group_bulk_write.cpp
  ${DXL_SRC_DIR}/group_sync_read.cpp
  ${DXL_SRC_DIR}/group_sync_write.cpp
)

add_library(dynamixel_sdk_cpp SHARED ${DXL_SOURCES})
target_include_directories(dynamixel_sdk_cpp
  PUBLIC
    $<BUILD_INTERFACE:${DXL_INC_DIR}>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${DXL_INC_DIR}/dynamixel_sdk
)
target_compile_definitions(dynamixel_sdk_cpp PUBLIC __LINUX__)
target_compile_features(dynamixel_sdk_cpp PUBLIC cxx_std_17)
set_target_properties(dynamixel_sdk_cpp PROPERTIES
  OUTPUT_NAME "dynamixel_sdk_cpp"
  POSITION_INDEPENDENT_CODE ON
)

find_package(Threads)
if(Threads_FOUND)
  target_link_libraries(dynamixel_sdk_cpp PUBLIC Threads::Threads)
endif()

install(TARGETS dynamixel_sdk_cpp
  EXPORT dynamixel_sdk_vendorTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY ${DXL_INC_DIR}/ DESTINATION include)

install(EXPORT dynamixel_sdk_vendorTargets
  NAMESPACE dynamixel_sdk_vendor::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(include)
ament_export_libraries(dynamixel_sdk_cpp)
ament_export_targets(dynamixel_sdk_vendorTargets HAS_LIBRARY_TARGET)

ament_package()