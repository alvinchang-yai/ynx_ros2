cmake_minimum_required(VERSION 3.8)
project(ynx_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

# Find gRPC and Protobuf
find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)

# Protobuf/gRPC Generation Setup
set(PROTO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/proto)
set(PROTO_SRC_DIR ${CMAKE_CURRENT_BINARY_DIR}/gen)
file(MAKE_DIRECTORY ${PROTO_SRC_DIR})

# Explicitly list all proto files with their nested paths
set(PROTO_FILES
  ${PROTO_DIR}/rcs/v1/alarm_api.proto
  ${PROTO_DIR}/rcs/v1/event_api.proto
  ${PROTO_DIR}/rcs/v1/file_api.proto
  ${PROTO_DIR}/rcs/v1/io_api.proto
  ${PROTO_DIR}/rcs/v1/job_control.proto
  ${PROTO_DIR}/rcs/v1/mode_get.proto
  ${PROTO_DIR}/rcs/v1/monitor_api.proto
  ${PROTO_DIR}/rcs/v1/motion_api.proto
  ${PROTO_DIR}/rcs/v1/position_types.proto
  ${PROTO_DIR}/rcs/v1/system_info.proto
  ${PROTO_DIR}/rcs/v1/timestamp.proto
  ${PROTO_DIR}/rcs/v1/variable_api.proto
)

# Initialize empty lists to hold the generated files
set(HW_PROTO_SRCS "")
set(HW_PROTO_HDRS "")
set(HW_GRPC_SRCS "")
set(HW_GRPC_HDRS "")

# Iterate over each proto file and set up generation commands
foreach(PROTO_FILE IN LISTS PROTO_FILES)
  # Get the path relative to PROTO_DIR (e.g., "rcs/v1/alarm_api.proto")
  file(RELATIVE_PATH REL_PROTO_FILE "${PROTO_DIR}" "${PROTO_FILE}")
  
  # Extract the directory path to ensure it exists in the build dir
  get_filename_component(REL_DIR "${REL_PROTO_FILE}" DIRECTORY)
  file(MAKE_DIRECTORY "${PROTO_SRC_DIR}/${REL_DIR}")

  # Strip the .proto extension (e.g., "rcs/v1/alarm_api")
  string(REGEX REPLACE "\\.proto$" "" REL_FIL_WE "${REL_PROTO_FILE}")
  
  # Define the expected output paths preserving the directory structure
  set(PROTO_SRC "${PROTO_SRC_DIR}/${REL_FIL_WE}.pb.cc")
  set(PROTO_HDR "${PROTO_SRC_DIR}/${REL_FIL_WE}.pb.h")
  set(GRPC_SRC "${PROTO_SRC_DIR}/${REL_FIL_WE}.grpc.pb.cc")
  set(GRPC_HDR "${PROTO_SRC_DIR}/${REL_FIL_WE}.grpc.pb.h")
  
  # Append to our master lists
  list(APPEND HW_PROTO_SRCS ${PROTO_SRC})
  list(APPEND HW_PROTO_HDRS ${PROTO_HDR})
  list(APPEND HW_GRPC_SRCS ${GRPC_SRC})
  list(APPEND HW_GRPC_HDRS ${GRPC_HDR})
  
  # Tell CMake how to generate these specific C++ files
  add_custom_command(
        OUTPUT "${PROTO_SRC}" "${PROTO_HDR}" "${GRPC_SRC}" "${GRPC_HDR}"
        COMMAND protobuf::protoc
        ARGS --grpc_out "${PROTO_SRC_DIR}"
             --cpp_out "${PROTO_SRC_DIR}"
             -I "${PROTO_DIR}"
             --plugin=protoc-gen-grpc="$<TARGET_FILE:gRPC::grpc_cpp_plugin>"
             "${PROTO_FILE}"
        DEPENDS "${PROTO_FILE}"
  )
endforeach()

# Build the hardware interface library
add_library(${PROJECT_NAME} SHARED
  src/ynx_hardware_interface.cpp
  ${HW_PROTO_SRCS}
  ${HW_GRPC_SRCS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PROTO_SRC_DIR} # Include path for generated proto headers
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  sensor_msgs
)

target_link_libraries(${PROJECT_NAME}
  protobuf::libprotobuf
  gRPC::grpc++
)

# Export the plugin (updated to match your current tree)
pluginlib_export_plugin_description_file(hardware_interface ynx_hardware_interface.xml)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Installed so other packages (e.g. ynx_robot_manager) can generate their own stubs
install(
  DIRECTORY proto
  DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  sensor_msgs
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
