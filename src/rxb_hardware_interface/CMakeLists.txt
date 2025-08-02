cmake_minimum_required(VERSION 2.8.3)
project(rxb_hardware_interface)
set(LINK_DIR /usr/include)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages 表示CATKIN需要的包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg 
  actionlib_msgs 
  actionlib
  control_msgs
  geometry_msgs

  rxb_msgs
)

find_package(Boost REQUIRED COMPONENTS system) ##表示boost需要的包

catkin_package(
    INCLUDE_DIRS include/rxb_hardware_interface  #表示编译时用在工程（包）目录下include里的头文件
    LIBRARIES rxb_hardware_interface
#  表示依赖包
 CATKIN_DEPENDS actionlib_msgs actionlib genmsg message_generation roscpp rospy std_msgs control_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
## include ${catkin_INCLUDE_DIRS}
#  include /rxb_control
  include/rxb_hardware_interface
#  表示指定额外的include目录，把路径下载这句话下面

#  ${Boost_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)
include_directories( "/usr/include/eigen3" )

link_directories(
    lib
    ${catkin_LIB_DIRS} 
)

add_library(rxb_hardware_interface
    src/global.cpp
)

add_executable(rxb_hardware_interface_node
src/Multi_rxb_hardware_interface_node.cpp  
src/global.cpp 
src/rxb_hardware_interface.cpp  
src/rxb_model_interface.cpp 
src/motor_driver.cpp   
src/Spline.cpp
src/rxb_algorithm.cpp
src/usbcan.cpp
src/rxb_close_control.cpp
src/rxb_open_control.cpp
src/rxb_kinematics.cpp
src/modern_robotics.cpp

)
add_dependencies(rxb_hardware_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries( rxb_hardware_interface_node
    ${catkin_LIBRARIES}
)
target_link_libraries(rxb_hardware_interface_node
  ${catkin_LIBRARIES} controlcan
)

# add_executable(rxb_comm
 # include/rxb_control/rxb_comm.h
#  src/rxb_comm.cpp
#)
# add_dependencies(rxb_comm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(rxb_comm
 # ${catkin_LIBRARIES}
#  include/rxb_control/rxb_comm.h
#)

