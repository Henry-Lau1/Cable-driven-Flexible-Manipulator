cmake_minimum_required(VERSION 2.8.3)
project(rviz_control_plugin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  std_msgs
  controller_manager_msgs
  pluginlib
  std_srvs
  sensor_msgs
  rxb_msgs

  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  
)
find_package(Qt5Widgets REQUIRED)

set(rviz_control_panel_SRCS
  src/rviz_control_panel.cpp
)
set(rviz_control_panel_HDRS
  include/rviz_control_plugin/rviz_control_panel.h
)
set(rviz_control_panel_UIS
  ui/rviz_control_panel.ui
)
set(rviz_control_panel_INCLUDE_DIRECTORIES
  include
  include/rviz_control_plugin
)

catkin_package(
  INCLUDE_DIRS ${rviz_control_panel_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs
)

qt5_wrap_cpp(rviz_control_panel_MOCS ${rviz_control_panel_HDRS})
qt5_wrap_ui(rviz_control_panel_UIS_H ${rviz_control_panel_UIS})

include_directories(${rviz_control_panel_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})
add_library(${PROJECT_NAME} ${rviz_control_panel_SRCS} ${rviz_control_panel_MOCS} ${rviz_control_panel_UIS_H})
add_dependencies(rviz_control_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})
include_directories(
   include
  ${catkin_INCLUDE_DIRS}
)
include_directories( "/usr/include/eigen3" )
##include_directories( "/home/huangxi/qt_ws/myrxb_qt_ws/src/rxb_hardware_interface/include/rxb_hardware_interface" )

class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES rviz_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)



