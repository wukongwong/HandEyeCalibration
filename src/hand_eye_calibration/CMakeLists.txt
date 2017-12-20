cmake_minimum_required(VERSION 2.8.3)
project(hand_eye_calibration)
set(CMAKE_CXX_FLAGS "-std=c++11")
find_package(PCL 1.7 REQUIRED COMPONENTS common io)
find_package(VISP REQUIRED)
find_package(OpenCV  REQUIRED)
find_package(kdl_parser)

find_package(

    catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    image_transport
    roscpp
    rospy
    roslib
    std_msgs
    geometry_msgs
    message_generation
    tf
    camera_info_manager
    cv_bridge
    visp_bridge

    VTK REQUIRED
    )



## Generate messages in the 'msg' folder
add_message_files(
  FILES
  pose_element.msg
)

add_service_files(
  FILES
  shooting_pose.srv
  find_center.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
# glog
#find_package(Glog REQUIRED)
#include_directories(BEFORE ${GLOG_INCLUDE_DIRS})



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
#CATKIN_DEPENDS message_runtime
)

include_directories($(OPENCV_INCLUDE_DIRS))
include_directories("/usr/include/eigen3")
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${OpenCV_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${OpenCV_DEFINITIONS}
  ${PCL_DEFINITIONS}
)




add_executable(hand_eye
               hand_eye.cpp)


target_link_libraries(hand_eye
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${VISP_LIBS}
   ${PCL_LIBRARIES}
 )

add_executable(test_pic
               test_pic.cpp)


target_link_libraries(test_pic
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${VISP_LIBS}
   ${PCL_LIBRARIES}
 )

add_executable(center_calib
                center_calib.cpp
              )


target_link_libraries(center_calib
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${VISP_LIBS}
   ${PCL_LIBRARIES}
 )

#add_executable(main main.cpp )


#target_link_libraries(main
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
#   ${VISP_LIBS}

# )

#add_executable(find_center find_center.cpp)


#target_link_libraries(find_center
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
#   ${VISP_LIBS}
# )





