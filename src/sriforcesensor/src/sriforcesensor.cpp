#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include "geometry_msgs/Twist.h"
#include <signal.h>
#include "TCPClient.h"
#include <rxb_msgs/sensorRead.h>
#include <Eigen/Dense>
#include <cmath>
using namespace Eigen;
using namespace std;
TCPClient tcp;
#define M812X_CHN_NUMBER	6
//#define JOINT_NUM 10
#define MG 0.92
#define L 40
#define PI 3.14159264
MatrixXd m_dResultChValue=MatrixXd::Zero(1,M812X_CHN_NUMBER);   //engineering output of each channel
MatrixXd m_dDecouplingValue=MatrixXd::Zero(1,M812X_CHN_NUMBER); //final output
MatrixXd m_nADCounts=MatrixXd::Zero(1,M812X_CHN_NUMBER);        //ad output
MatrixXd m_dAmpZero=MatrixXd::Zero(1,M812X_CHN_NUMBER);         //read
MatrixXd m_dChnGain=MatrixXd::Zero(1,M812X_CHN_NUMBER);         //read
MatrixXd m_dChnEx=MatrixXd::Zero(1,M812X_CHN_NUMBER);           //read
MatrixXd m_dDecouplingCoefficient(6, 6);

MatrixXd initval=MatrixXd::Zero(1,M812X_CHN_NUMBER);
int initcounts = 0;

//extern double jointangle[JOINT_NUM];
//double jointangle[JOINT_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//extern double fuyanangle;
//double fuyanangle = 0.0;
//extern double phangangle;
//double phangangle = 0.0;

void sig_exit(int s)
{
  tcp.exit();
  exit(0);
}


bool ConfigSystem(void)
{

  //string rec = tcp.receive();
  tcp.Send("AT+SGDM=(A01,A02,A03,A04,A05,A06);C;1;(WMA:1)\r\n");
  string rec = tcp.read();
  if( rec != "" )
  {
    std::cout << "Server Response:" << rec << endl;
  } else
  {
    std::cout << "Server Not Response:"  << endl;
  }

  tcp.Send("AT+AMPZ=?\r\n");
  if(tcp.GetChParameter(m_dAmpZero))
  {
    std::cout << "AT+AMPZ=: " << m_dAmpZero << endl;
  }else{
    std::cout << "Server no Response:" << endl;
  }

  tcp.Send("AT+CHNAPG=?\r\n");
  if(tcp.GetChParameter(m_dChnGain))
  {
    std::cout << "AT+CHNAPG=: " << m_dChnGain << endl;
  }else{
    std::cout << "Server no Response:" << endl;
  }
  tcp.Send("AT+GOD\r\n");
  if(tcp.GetADCounts(m_nADCounts))
  {
    std::cout << "AD value:" << m_nADCounts << endl;
  }else{
    std::cout << "Server no Response:" << endl;
  }

  for (unsigned  int i = 0; i < 6; ++i) {
    m_dResultChValue(i) = 1000*( (m_nADCounts(i) - m_dAmpZero(i)) / (double)65535*(double)5 ) / m_dChnGain(i);
  }
  std::cout << "result value is : "  <<m_dResultChValue << std::endl;

  m_dDecouplingCoefficient<<-1.54943,-0.71890,-0.83840,73.83278,-1.56837,-73.19899,
                            -0.06329,-83.63168,0.58668,41.81031,-0.28479,43.12496,
                            224.23093,-0.04867,233.96484,-0.99719,232.95643,-1.50553,
                            0.02799,0.08642,-8.42477,-0.02789,8.35025,-0.06303,
                            9.01379,-0.04674,-4.85699,0.05790,-4.93189,-0.03195,
                            0.01235,3.13525,-0.01961,3.25311,0.05655,3.40737;
  std::cout << "Decoupling matrix is set as: " <<endl <<m_dDecouplingCoefficient << std::endl;
  m_dDecouplingValue=m_dResultChValue*m_dDecouplingCoefficient.transpose();
  std::cout << "force is: " <<endl <<m_dDecouplingValue << std::endl;
  return true;
}

//void serial_data_subcallback(const rxb_msgs::sensorRead::ConstPtr &data)
//{
//  phangangle = 0.0;
//  fuyanangle = 0.0;
//  for (int i = 0; i <= JOINT_NUM; i++) {
//    jointangle[i] = data ->joint_angle[i];
//    if(i == 0 && i == 3 && i == 4 && i == 7 && i == 8){
//      fuyanangle += data ->joint_angle[i];
//    }
//    else {
//      phangangle += data ->joint_angle[i];
//    }
//  }
//}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  signal(SIGINT, sig_exit);
  if(tcp.setup("192.168.0.108",4008)==true)
  {
    cout<<"Force sensor has been connected!"<<endl;
  }else{
    cout<<"Force sensor connection failed!!!!!!!!!!!!!!!!!"<<endl;
  }
  //initialize the setting of the force sensor
  ConfigSystem();
  //get real time force sensor data
  tcp.Send("AT+GSD\r\n");
  cout<<"Force sensor has been connected!1"<<endl;
  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */


  geometry_msgs::Twist Forcevalue;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("forceSensor/force", 1000);
//  ros::Subscriber serial_sub = n.subscribe<rxb_msgs::sensorRead>("serialport/sensor_data", 1, &serial_data_subcallback);
  ros::Rate loop_rate(1000);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    tcp.readrecieveBuffer(m_nADCounts);
    for (unsigned  int i = 0; i < 6; ++i) {
      m_dResultChValue(i) = 1000*( (m_nADCounts(i) - m_dAmpZero(i)) / (double)65535*(double)1 ) / m_dChnGain(i);
    }
    m_dDecouplingValue=m_dResultChValue*m_dDecouplingCoefficient.transpose();

    if(initcounts < 100) {
        initval += m_dDecouplingValue;
        initcounts++;
    }
    else if(initcounts == 100) {
        initval /= 100;
        initcounts++;
    }
    else {
        m_dDecouplingValue -= initval;
    }

    std::cout << "force is: " <<endl <<m_dDecouplingValue << std::endl;

//    Forcevalue.linear.x=m_dDecouplingValue(0,0) - 0.122 + 0.296;
//    Forcevalue.linear.y=m_dDecouplingValue(0,1) + 0.05 + 1.71 + MG * cos(fuyanangle * PI / 180);
//    Forcevalue.linear.z=m_dDecouplingValue(0,2) + 6.7 - 0.8 + MG * sin(fuyanangle * PI / 180);
//    Forcevalue.angular.x=m_dDecouplingValue(0,3) * 1000 + 2;
//    Forcevalue.angular.y=m_dDecouplingValue(0,4) * 1000 - 260.3 + 50 + L * MG * cos(fuyanangle * PI / 180);
//    Forcevalue.angular.z=m_dDecouplingValue(0,5) * 1000 - 48.2 - 10;

//    Forcevalue.linear.x=m_dDecouplingValue(0,0) + 0.522;
//    Forcevalue.linear.y=m_dDecouplingValue(0,1) + 0.05 + 0.66 + MG * cos(fuyanangle * PI / 180);
//    Forcevalue.linear.z=m_dDecouplingValue(0,2) + 6.7 - 1.43 + MG * sin(fuyanangle * PI / 180);
//    Forcevalue.angular.x=m_dDecouplingValue(0,3) * 1000 + 20;
//    Forcevalue.angular.y=m_dDecouplingValue(0,4) * 1000 - 260.3 + 14 - L * MG * cos(fuyanangle * PI / 180);
//    Forcevalue.angular.z=m_dDecouplingValue(0,5) * 1000 - 48.2 - 7;

    Forcevalue.linear.x=-m_dDecouplingValue(0,1);
    Forcevalue.linear.y=-m_dDecouplingValue(0,0);
    Forcevalue.linear.z=-m_dDecouplingValue(0,2);
    Forcevalue.angular.x=-m_dDecouplingValue(0,4);   // 单位为N m
    Forcevalue.angular.y=-m_dDecouplingValue(0,3);   // 单位为N m
    Forcevalue.angular.z=-m_dDecouplingValue(0,5);   // 单位为N m


    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(Forcevalue);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

