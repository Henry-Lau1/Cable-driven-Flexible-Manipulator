#include "serial_communication/SerialCom.h"

int timee = 0;

void timer_handler(const boost::system::error_code &ec) {
    timee++;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "serial_communication_node");
    ros::NodeHandle node;
    ros::Rate loop_rate(10);
    ROS_INFO_STREAM("\033[42;37m Multi_Level_Linkage SYSTEM Loading \033[0m");
    MySerialCom my_sp("/dev/ttyUSB0", node);
//    my_sp.write_to_serial("ALL_INFO\n", 9);
//    my_sp.write_to_serial("A\r\n", 3);
    my_sp.read_sensor_from_serial();//阻塞在这里了
    //my_sp.call_handle();

    return 0;
}

