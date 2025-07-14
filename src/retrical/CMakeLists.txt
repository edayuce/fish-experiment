#AYSE - KAAN

string(ASCII 27 Esc)
set(ColourReset "${Esc}[m")
set(RED "${Esc}[31m")
set(Green       "${Esc}[32m")
set(Yellow      "${Esc}[33m")

if(DEFINED ENV{BASLER})
    message("${Yellow}=============================${ColourReset}")
    message("${Yellow} Compiling for BASLER Camera!${ColourReset}")
    message("${Yellow}=============================${ColourReset}")
else()
    message("${Yellow}=============================${ColourReset}")
    message("${Yellow} Compiling for DALSA Camera!${ColourReset}")
    message("${Yellow}=============================${ColourReset}")
endif()

cmake_minimum_required(VERSION 3.0.2)
project(rectrial)

set (CMAKE_CXX_STANDARD 11)

add_definitions("-std=gnu++11")
#add_definitions("-ggdb")

FIND_LIBRARY(GEVAPI_LIBRARY GevApi /usr/local/lib/)
FIND_LIBRARY(GCBASE_LIBRARY GCBase_gcc54_v3_0 /opt/genicam_v3_0/bin/Linux64_ARM)
FIND_LIBRARY(ROSCPP_LIBRARY roscpp /opt/ros/noetic/lib)

#set(CMAKE_CXX_FLAGS "-g -Wall -Wno-unknown-pragmas -Wno-delete-non-virtual-dtor -Wno-unused-variable")
set(
    CATKIN_COMPONENTS
    roscpp
    rospy
    std_msgs
    message_runtime
    sensor_msgs
    image_transport
    cv_bridge
)

set(OpenCV_DIR /usr/local/include)

if(DEFINED ENV{BASLER})
    find_package(Pylon QUIET)
    if (NOT ${Pylon_FOUND})
        include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
    endif()
endif()

find_package(
    catkin REQUIRED
    COMPONENTS
     ${CATKIN_COMPONENTS}
     roslint
     message_generation
)

find_package(OpenCV REQUIRED)

 add_message_files(
   FILES
   pub_data.msg
 )


 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )


catkin_package(
    INCLUDE_DIRS
     include
    LIBRARIES
     ${PROJECT_NAME}
    CATKIN_DEPENDS
     ${CATKIN_COMPONENTS}
)

set(
    ROSLINT_CPP_OPTS
     "--extensions=cpp,h,hpp" "--filter=-runtime/references,-readability/todo,-build/include_what_you_use"
)

# CHECKPOINT-1
roslint_cpp(
    scripts/${PROJECT_NAME}/genicam_cpp_demo.cpp
    )

#roslaunch_add_file_check(launch)

if(DEFINED ENV{BASLER})
    include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/rectrial/scripts/include/${PROJECT_NAME}/
        ${catkin_INCLUDE_DIRS}
        ${Pylon_INCLUDE_DIRS}
        /usr/dalsa/GigeV/examples/common
        /usr/local/lib
        /usr/local/include/opencv2
        /usr/local/include/opencv2/core
        /usr/dalsa/GigeV/include/
        /opt/genicam_v3_0/library/CPP/include/
        /usr/dalsa/GigeV/examples/common/
        /usr/lib
        /usr/include
        /opt/ros/noetic/include
    )
else()
    include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/rectrial/scripts/include/${PROJECT_NAME}/
        ${catkin_INCLUDE_DIRS}
        /usr/dalsa/GigeV/examples/common
        /usr/local/lib
        /usr/local/include/opencv2
        /usr/local/include/opencv2/core
        /usr/dalsa/GigeV/include/
        /opt/genicam_v3_0/library/CPP/include/
        /usr/dalsa/GigeV/examples/common/
        /usr/lib
        /usr/include
        /opt/ros/noetic/include
    )
endif()


add_library(GlWindow STATIC
    ${CMAKE_SOURCE_DIR}/rectrial/scripts/GLWindow.cpp
    ${CMAKE_SOURCE_DIR}/rectrial/scripts/gl/src/glad.c
)
target_include_directories(GlWindow PUBLIC ${CMAKE_SOURCE_DIR}/rectrial/scripts/gl/include)
target_link_libraries(GlWindow SDL2 dl)


if(DEFINED ENV{BASLER})
    add_executable(genicam_cpp_demo ${CMAKE_SOURCE_DIR}/rectrial/scripts/camera_node.cpp)
else()
    add_executable(genicam_cpp_demo ${CMAKE_SOURCE_DIR}/rectrial/scripts/dalsa_camera.cpp)
endif()

add_executable(experiment_controller ${CMAKE_SOURCE_DIR}/rectrial/scripts/experiment_controller.cpp)
add_executable(rec_node ${CMAKE_SOURCE_DIR}/rectrial/scripts/rec_node.cpp)
add_executable(HelloEposCmd ${CMAKE_SOURCE_DIR}/rectrial/scripts/HelloEposCmd.cpp)
add_executable(displayer ${CMAKE_SOURCE_DIR}/rectrial/scripts/displayer.cpp)


set(
    DALSA_LIBS 
    -L/usr/lib/aarch64-linux-gnu
    -L/usr/local/lib
    ${ROSCPP_LIBRARY}
    -lopencv_core
    -lopencv_highgui 
    -lopencv_imgcodecs 
    -lopencv_imgproc 
    -lopencv_videoio   
    -lpthread   
    ${GCBASE_LIBRARY} 
    -lm 
    -lz  
    -lXext 
    -lX11  
    -lGevApi
)


target_link_libraries( rec_node -L/usr/lib/aarch64-linux-gnu -L/usr/local/lib ${catkin_LIBRARIES} ${ROSCPP_LIBRARY} -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_videoio)
target_link_libraries( experiment_controller -L/usr/lib/aarch64-linux-gnu -L/usr/local/lib ${catkin_LIBRARIES} ${ROSCPP_LIBRARY} -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_videoio -lJetsonGPIO)
target_link_libraries(HelloEposCmd -lEposCmd ${catkin_LIBRARIES} ${ROSCPP_LIBRARY})
target_link_libraries( displayer -L/usr/lib/aarch64-linux-gnu -L/usr/local/lib ${catkin_LIBRARIES} ${ROSCPP_LIBRARY} GlWindow)

if(DEFINED ENV{BASLER})
    target_link_libraries(
        genicam_cpp_demo
        ${catkin_LIBRARIES}
        ${Pylon_LIBRARIES}
        ${GEVAPI_LIBRARY}
        ${DALSA_LIBS}
    )
else()
    target_link_libraries(
        genicam_cpp_demo
        ${catkin_LIBRARIES}
        ${Pylon_LIBRARIES}
        ${GEVAPI_LIBRARY}
        ${DALSA_LIBS}
    )
endif()

target_link_libraries(experiment_controller GlWindow)

add_dependencies(
    genicam_cpp_demo
     ${catkin_EXPORTED_TARGETS}
     rectrial_generate_messages_cpp
)
add_dependencies(
    experiment_controller
     ${catkin_EXPORTED_TARGETS}
     rectrial_generate_messages_cpp
)
add_dependencies(
    rec_node
     ${catkin_EXPORTED_TARGETS}
     rectrial_generate_messages_cpp
)
add_dependencies(
    HelloEposCmd
     ${catkin_EXPORTED_TARGETS}
     rectrial_generate_messages_cpp
)
add_dependencies(
    HelloEposCmd
     ${catkin_EXPORTED_TARGETS}
     rectrial_generate_messages_cpp
)

