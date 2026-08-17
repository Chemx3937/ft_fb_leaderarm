cmake_minimum_required(VERSION 3.8)
project(ft_fb_leaderarm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(Boost_NO_WARN_NEW_VERSIONS ON)
set(COAL_DISABLE_HPP_FCL_WARNINGS ON CACHE BOOL "Suppress hpp-fcl compatibility warnings" FORCE)
add_compile_definitions(COAL_DISABLE_HPP_FCL_WARNINGS)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(contact_observer_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

find_path(TINYXML2_INCLUDE_DIR tinyxml2.h
  PATHS /usr/include /usr/local/include
)
find_library(TINYXML2_LIBRARY tinyxml2
  PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
)
if(NOT TINYXML2_INCLUDE_DIR OR NOT TINYXML2_LIBRARY)
  message(FATAL_ERROR "Could not find tinyxml2. Install libtinyxml2-dev for FT payload gravity compensation.")
endif()

if(pinocchio_DIR)
  get_filename_component(_PINOCCHIO_CMAKE_PARENT "${pinocchio_DIR}" DIRECTORY)
  get_filename_component(_PINOCCHIO_LIB_DIR "${_PINOCCHIO_CMAKE_PARENT}" DIRECTORY)
  list(APPEND CMAKE_BUILD_RPATH "${_PINOCCHIO_LIB_DIR}")
  list(APPEND CMAKE_INSTALL_RPATH "${_PINOCCHIO_LIB_DIR}")
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()

# Dynamixel SDK: use the installed library when available, otherwise compile
# the workstation SDK checkout exactly as fb_leaderarm does.
set(DYNAMIXEL_SDK_ROOT "$ENV{HOME}/DynamixelSDK/c++" CACHE PATH "DynamixelSDK C++ root")
find_path(DXL_INCLUDE
  dynamixel_sdk/dynamixel_sdk.h
  PATHS
    /usr/local/include
    ${DYNAMIXEL_SDK_ROOT}/include
)
if(NOT DXL_INCLUDE)
  message(FATAL_ERROR "Could not find dynamixel_sdk/dynamixel_sdk.h. Set DYNAMIXEL_SDK_ROOT.")
endif()

find_library(DXL_LIB
  NAMES dxl_x64_cpp dxl_cpp dynamixel_sdk
  PATHS
    /usr/local/lib
    ${DYNAMIXEL_SDK_ROOT}/build/linux64
    ${DYNAMIXEL_SDK_ROOT}/build
)

if(DXL_LIB)
  set(DXL_TARGET ${DXL_LIB})
else()
  set(DXL_SRC_ROOT "${DYNAMIXEL_SDK_ROOT}/src/dynamixel_sdk")
  if(NOT EXISTS "${DXL_SRC_ROOT}/packet_handler.cpp")
    message(FATAL_ERROR "Dynamixel SDK library not found and source fallback missing under ${DXL_SRC_ROOT}")
  endif()
  add_library(ft_fb_leaderarm_dynamixel_sdk STATIC
    ${DXL_SRC_ROOT}/group_bulk_read.cpp
    ${DXL_SRC_ROOT}/group_bulk_write.cpp
    ${DXL_SRC_ROOT}/group_sync_read.cpp
    ${DXL_SRC_ROOT}/group_sync_write.cpp
    ${DXL_SRC_ROOT}/group_fast_bulk_read.cpp
    ${DXL_SRC_ROOT}/group_fast_sync_read.cpp
    ${DXL_SRC_ROOT}/group_handler.cpp
    ${DXL_SRC_ROOT}/packet_handler.cpp
    ${DXL_SRC_ROOT}/port_handler.cpp
    ${DXL_SRC_ROOT}/protocol1_packet_handler.cpp
    ${DXL_SRC_ROOT}/protocol2_packet_handler.cpp
    ${DXL_SRC_ROOT}/port_handler_linux.cpp
  )
  target_include_directories(ft_fb_leaderarm_dynamixel_sdk PUBLIC
    ${DXL_INCLUDE}
    ${DXL_INCLUDE}/dynamixel_sdk
  )
  target_compile_options(ft_fb_leaderarm_dynamixel_sdk PRIVATE -w)
  target_link_libraries(ft_fb_leaderarm_dynamixel_sdk PUBLIC rt)
  set(DXL_TARGET ft_fb_leaderarm_dynamixel_sdk)
endif()

set(LEADER_SOURCES
  src/single_impedance_teleop_node.cpp
  src/dynamixel_bus.cpp
  src/single_impedance_gravity_compensation.cpp
  src/single_impedance_pose_publisher.cpp
  src/single_impedance_wrench_feedback.cpp
  src/single_impedance_keyboard_fsm.cpp
)

add_executable(ft_fb_leader_single_impedance_teleop
  src/single_impedance_teleop_main.cpp
  ${LEADER_SOURCES}
)
target_include_directories(ft_fb_leader_single_impedance_teleop PRIVATE
  include
  ${DXL_INCLUDE}
  ${TINYXML2_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${PINOCCHIO_INCLUDE_DIRS}
)
target_link_libraries(ft_fb_leader_single_impedance_teleop
  ${DXL_TARGET}
  ${TINYXML2_LIBRARY}
  pinocchio::pinocchio
)
ament_target_dependencies(ft_fb_leader_single_impedance_teleop
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
  contact_observer_msgs
)
target_compile_definitions(ft_fb_leader_single_impedance_teleop PRIVATE
  FT_FB_LEADERARM_PACKAGE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS ft_fb_leader_single_impedance_teleop
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/ft_free_space_collect
  scripts/ft_free_space_validate
  scripts/ft_free_space_train
  scripts/ft_contact_observer
  scripts/ft_contact_evaluate
  scripts/ft_observer_runtime_evaluate
  scripts/ft_feedback_analyze
  scripts/ft_feedback_authorize
  scripts/ft_free_space_collection_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch document
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_pytest test TIMEOUT 180)
endif()

ament_package()
