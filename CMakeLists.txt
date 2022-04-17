cmake_minimum_required(VERSION 3.0.2)
project(mvs_image_ros)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  image_transport
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mvs_image_ros
  CATKIN_DEPENDS cv_bridge roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /opt/MVS/include
)
LINK_DIRECTORIES(
  /opt/MVS/lib/aarch64/
)

# link_directories(${OpenCV_LIBRARY_DIRS})
add_executable(pub_image src/pub_image.cpp)
target_link_libraries(pub_image
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  GCBase_gcc48_v3_0
   MvCameraControl
   GenApi_gcc48_v3_0
   MVGigEVisionSDK
   log4cpp_gcc48_v3_0
   MVRender
   Log_gcc48_v3_0
   MvUsb3vTL
   MathParser_gcc48_v3_0
   NodeMapData_gcc48_v3_0
   MediaProcess
   XmlParser_gcc48_v3_0
   X11
)
