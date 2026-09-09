cmake_minimum_required(VERSION 3.16)
project(asdt1_ros2_driver)

option(ADDRESS_SANITIZER OFF)

set(DEPENDENCIES_DIR ${CMAKE_CURRENT_LIST_DIR}/dependencies)
set(LICENSES_DIR ${CMAKE_CURRENT_LIST_DIR}/licenses)

## ----------------------------------------------------------------------------
## Generic compile options
## ----------------------------------------------------------------------------
add_compile_options(-Wno-deprecated-declarations)

# Default to C++20
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Generic compile options
## ----------------------------------------------------------------------------
if(MSVC)
    add_definitions(-D_USE_MATH_DEFINES=1)
else()
    add_compile_options(-Wno-deprecated-declarations)
endif()

## Address sanitizer
## ----------------------------------------------------------------------------
if(ADDRESS_SANITIZER)
    message(STATUS "Building with AddressSanitizer")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address,undefined")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address,undefined")
endif()


## ----------------------------------------------------------------------------
## Generate version string to generated/version.h
## ----------------------------------------------------------------------------
execute_process(
    COMMAND git describe --tags --abbrev=0
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_LATEST_TAG
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Get number of commits since the latest tag
execute_process(
    COMMAND git rev-list --count HEAD ^${GIT_LATEST_TAG}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_COMMIT_COUNT
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Remove the "v" prefix from the tag
string(REPLACE "v" "" GIT_TAG_VERSION "${GIT_LATEST_TAG}")

set(VERSION_STRING "${GIT_TAG_VERSION}.${GIT_COMMIT_COUNT}")

if(NOT GIT_TAG_VERSION)
    message(STATUS "Not able to determine driver version from git, will fall back to package.xml at runtime")
    set(VERSION_STRING "")
else()
    message(STATUS "AS-DT1 ROS2 driver version is ${VERSION_STRING}")
endif()

configure_file(
    ${CMAKE_SOURCE_DIR}/version.h.in
    ${CMAKE_BINARY_DIR}/generated/version.h
    @ONLY
)

## Dependencies
## -----------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include(cmake/utility_functions.cmake)

include(cmake/configure_serial.cmake)
include(cmake/configure_eigen.cmake)

## Define target asdt1_driver
## -----------------------------------
set(TARGET_NAME asdt1_driver)

add_library(${TARGET_NAME}_component SHARED
    src/asdt1_driver_node.cpp
    src/calibration_parser.cpp
    src/licenses_info.cpp
    src/device/dt1_device.cpp
    src/device/dt1_device_cdc.cpp
    src/device/dt1_device_hist.cpp
    src/device/dt1_modes.cpp
)
set_target_properties(${TARGET_NAME}_component PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ONLICENSES_DIR
)

target_compile_options(${TARGET_NAME}_component PRIVATE -Wno-unused-parameter)

target_include_directories(${TARGET_NAME}_component PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/device/include ${CMAKE_BINARY_DIR} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(${TARGET_NAME}_component
  rclcpp
  rclcpp_components
  ament_index_cpp
  std_srvs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

target_link_libraries(${TARGET_NAME}_component
  serial
  Eigen3::Eigen
  yaml-cpp
)

rclcpp_components_register_node(${TARGET_NAME}_component
  PLUGIN "ASDT1_Driver"
  EXECUTABLE ${TARGET_NAME}
)

install(TARGETS ${TARGET_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

make_licenses(${TARGET_NAME}_component)

## Define target point_clouud_combiner
## -----------------------------------
set(TARGET_NAME point_cloud_combiner)

add_library(${TARGET_NAME}_component SHARED
  src/point_cloud_combiner_node.cpp
  src/licenses_info.cpp
)
set_target_properties(${TARGET_NAME}_component PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
)

target_include_directories(${TARGET_NAME}_component PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/asdt1/include PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(${TARGET_NAME}_component
  rclcpp
  rclcpp_components
  std_srvs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

target_link_libraries(${TARGET_NAME}_component
  Eigen3::Eigen
)

rclcpp_components_register_node(${TARGET_NAME}_component
  PLUGIN "PointCloudCombiner"
  EXECUTABLE ${TARGET_NAME}
)

install(TARGETS ${TARGET_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  ../../doc/user_manual.pdf
  DESTINATION share/${PROJECT_NAME}/doc
)

## ----------------------------------------------------------------------------
## Hardware-in-the-loop tests
## Skipped unless ASDT1_HW_TESTS=ON and ASDT1_CALIBRATION_FILE is set.
## Run manually: pytest test/hardware/ --hw
## ----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  if(DEFINED ENV{ASDT1_HW_TESTS})
    ament_add_pytest_test(asdt1_driver_hw_tests
      test/hardware/test_asdt1_driver.py
      PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
      APPEND_ENV
        ASDT1_CALIBRATION_FILE=$ENV{ASDT1_CALIBRATION_FILE}
        ASDT1_MODE=$ENV{ASDT1_MODE}
        ASDT1_OUTPUT_FORMAT=$ENV{ASDT1_OUTPUT_FORMAT}
      TIMEOUT 120
    )

    install(DIRECTORY test/hardware/
      DESTINATION share/${PROJECT_NAME}/test/hardware
    )
  endif()
endif()

ament_package()

make_licenses(${TARGET_NAME}_component)
