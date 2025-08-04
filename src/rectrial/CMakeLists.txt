# ===================================================================
# Compiling for BASLER Camera
# ===================================================================
cmake_minimum_required(VERSION 3.0.2)
project(rectrial)

set(CMAKE_CXX_STANDARD 11)
add_definitions("-std=gnu++11")

# Find required ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_runtime
  sensor_msgs
  image_transport
  cv_bridge
  roslint
  message_generation
  geometry_msgs
)

# Find OpenCV
#find_package(OpenCV REQUIRED)

# Find Pylon for Basler Camera
# This assumes the Pylon SDK is installed in a standard location.
# find_package(Pylon REQUIRED)
set(Pylon_INCLUDE_DIRS /opt/pylon/include)
set(OpenCV_DIR "$ENV{HOME}/fish-experiment/deps/opencv/lib/cmake/opencv4")



# ===================================================================
# ROS Messages
# ===================================================================
add_message_files(
  FILES
  pub_data.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)


# ===================================================================
# Catkin Package Configuration
# ===================================================================
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime sensor_msgs image_transport cv_bridge
)

catkin_install_python(PROGRAMS
  scripts/my_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



# ===================================================================
# Include Directories
# ===================================================================
# Add directories needed for compilation
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Pylon_INCLUDE_DIRS}
)


# ===================================================================
# Build Libraries & Executables
# ===================================================================


# --- Libraries ---
add_library(GlWindow STATIC
  scripts/GLWindow.cpp
  scripts/gl/src/glad.c
)
# Add include directory for this specific library
target_include_directories(GlWindow PUBLIC scripts/gl/include)


add_library(Epos STATIC
  scripts/epos_lib/epos_lib.cpp
)
target_include_directories(Epos PUBLIC 
  scripts/epos_lib
  scripts
)


# --- Executables ---
add_executable(basler_camera_node       scripts/camera_node.cpp)
add_executable(experiment_controller    scripts/experiment_controller.cpp scripts/adaptive_filter.cpp)
add_executable(video_recorder_node      scripts/rec_node.cpp)
add_executable(epos_node                scripts/epos_node.cpp)
add_executable(real_time_tracker        scripts/real_time_tracker.cpp)
add_executable(main_node                scripts/main_node.cpp)
add_executable(csv_writer_node          scripts/csv_writer.cpp)
add_executable(refuge_tracker_node      scripts/refuge_tracker.cpp)



# ===================================================================
# Link Libraries
# ===================================================================
# Link libraries to their corresponding executables

link_directories(/opt/pylon/lib)

# Link Basler camera node
target_link_libraries(basler_camera_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  /opt/pylon/lib/libpylonbase.so
  /opt/pylon/lib/libpylonutility.so
  /opt/pylon/lib/libpylonc.so
  /opt/pylon/lib/libGCBase_gcc_v3_1_Basler_pylon.so
)

# Link other nodes
target_link_libraries(video_recorder_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(experiment_controller
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  GlWindow
  dl
  SDL2
  # You may need to add -lJetsonGPIO here if you are on a Jetson
)

target_link_libraries(epos_node
  ${catkin_LIBRARIES}
  Epos
  EposCmd
  dl
  rt
)

target_link_libraries(real_time_tracker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(main_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  GlWindow
  SDL2
  dl
)

target_link_libraries(csv_writer_node 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(refuge_tracker_node 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# ===================================================================
# Dependencies
# ===================================================================
# Ensure messages are generated before nodes that use them are built
add_dependencies(basler_camera_node    ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(experiment_controller ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(video_recorder_node   ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(epos_node             ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(main_node             ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(real_time_tracker     ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(csv_writer_node       ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(refuge_tracker_node   ${PROJECT_NAME}_generate_messages_cpp)


