cmake_minimum_required(VERSION 2.8.3)
project(rxb_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
  roscpp#msgs!!!!
)


add_message_files(
  FILES
  serialCommu.msg
  sensorRead.msg
  jointControl.msg
  forceSenseResult.msg
  neuralNetworkCommand.msg
  trajectoryDemo.msg
  neuralNetworkProcess.msg
)

add_service_files(
    FILES
    RxbMode.srv
    MotorMode.srv
    RxbPose.srv
    RxbPoseCmd.srv
    taskSpaceCtrl_singleSeg.srv
    corCableForceNN.srv
)


generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS  message_runtime  std_msgs  geometry_msgs
)
