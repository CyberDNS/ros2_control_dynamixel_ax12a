cmake_minimum_required(VERSION 3.16)
project(dynamixel_sdk_vendor)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
include(FetchContent)

# ---- Options ---------------------------------------------------------------
# Allow override to use a local SDK checkout (for offline/dev)
set(DYNAMIXEL_SDK_SOURCE "" CACHE PATH "Use existing DynamixelSDK source instead of fetching")

# Pin upstream repo & ref (can be a branch, tag, or commit SHA)
set(DYNAMIXEL_SDK_GIT "https://github.com/ROBOTIS-GIT/DynamixelSDK.git")
set(DYNAMIXEL_SDK_TAG "jazzy")  # change to a specific commit for full reproducibility

# ---- Obtain sources --------------------------------------------------------
if(DYNAMIXEL_SDK_SOURCE)
  set(dxl_SOURCE_DIR ${DYNAMIXEL_SDK_SOURCE})
else()
  FetchContent_Declare(dxl
    GIT_REPOSITORY ${DYNAMIXEL_SDK_GIT}
    GIT_TAG        ${DYNAMIXEL_SDK_TAG}
    GIT_SHALLOW    TRUE
    UPDATE_DISCONNECTED TRUE
  )
  FetchContent_MakeAvailable(dxl)    # sets dxl_SOURCE_DIR
endif()

# ---- Build the C SDK as a static lib --------------------------------------
set(DXL_C_INC     ${dxl_SOURCE_DIR}/c++/include/dynamixel_sdk)
set(DXL_C_SRC_DIR ${dxl_SOURCE_DIR}/c++/src/dynamixel_sdk)

set(DXL_C_SOURCES
  ${DXL_C_SRC_DIR}/packet_handler.cpp
  ${DXL_C_SRC_DIR}/port_handler.cpp
  ${DXL_C_SRC_DIR}/port_handler_linux.cpp
  ${DXL_C_SRC_DIR}/group_bulk_read.cpp
  ${DXL_C_SRC_DIR}/group_bulk_write.cpp
  ${DXL_C_SRC_DIR}/group_sync_read.cpp
  ${DXL_C_SRC_DIR}/group_sync_write.cpp
)

add_library(dynamixel_sdk_cpp STATIC ${DXL_C_SOURCES})
target_include_directories(dynamixel_sdk_cpp PUBLIC ${DXL_C_INC})
target_compile_features(dynamixel_sdk_cpp PUBLIC cxx_std_17)

# pthread if available (usually harmless and often pulled anyway)
find_package(Threads)
if(Threads_FOUND)
  target_link_libraries(dynamixel_sdk_cpp PUBLIC Threads::Threads)
endif()

# ---- Install & export ------------------------------------------------------
install(TARGETS dynamixel_sdk_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY ${dxl_SOURCE_DIR}/c++/include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(dynamixel_sdk_cpp)

ament_package()