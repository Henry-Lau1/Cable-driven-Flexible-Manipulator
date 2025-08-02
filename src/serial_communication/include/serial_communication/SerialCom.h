#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <string>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include "rxb_msgs/serialCommu.h"
#include "rxb_msgs/sensorRead.h"

//#define Motor_num_all 3
//#define Controllable_DOF_num_all 2

using namespace std;

typedef string any_type;

/*
sersor data:
0xaa 0xaa 'C' 12*cb_force(5*char) CRC1 'E' 8*jnt_angle(3*char) CRC2   90 bit

*/

class MySerialCom {
private:
    bool init_port(const any_type port, const unsigned int char_size);

public:
    MySerialCom(const any_type &port_name, ros::NodeHandle n);
    ~MySerialCom();

    //write data to your port
    //  void write_to_serial(const any_type data);

    void write_to_serial(const std::string, size_t len);
    // void read_sensor_from_serial();
    void read_sensor_from_serial();


    // void handle_read( const boost::system::error_code &ec, std::size_t bytes_transferred);
    void handle_write(const boost::system::error_code &ec, std::size_t bytes_transferred);

    bool sensordata_analysis(std::string );
    void call_handle();
    void serialPort_subFun(const rxb_msgs::serialCommu::ConstPtr &commu);


    static std::string sign_sensorData;

private:

//    double init_joint_angle[Controllable_DOF_num_all];
//    double real_joint_angle[Controllable_DOF_num_all];

    boost::asio::io_service io_sev;

    std::string rec_data;
    float cable_force[21];
    float joint_angle[6];
    const int g_cableForce_ID[21] = {1, 2, 3, 4, 5, 6, 8, 7, 10, 9, 12, 11, 13, 14, 15, 16, 17, 18, 19, 20, 21};//绳索号对应的拉力传感器号

    std::string write_data;
    boost::asio::serial_port *pSerialPort;

    any_type m_port;

    boost::system::error_code ec;

    ros::NodeHandle nh_;

    ros::Subscriber serial_sub;
    ros::Publisher serial_pub;
    ros::Subscriber joint_zero_sub;

    ros::Rate loop_rate;
    ros::AsyncSpinner spinner;


};

