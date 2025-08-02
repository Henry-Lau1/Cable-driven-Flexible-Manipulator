#include "rxb_hardware_interface.h"

#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "rxb_hardware_interface_node");
    ros::NodeHandle node;
    RxbHardwareInterface::RxbHardwareInterface rxb(node);

    rxb.mainLoop();

    return 0;
}


