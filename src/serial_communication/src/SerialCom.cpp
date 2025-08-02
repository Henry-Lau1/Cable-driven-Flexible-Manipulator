#include "serial_communication/SerialCom.h"
#include <vector>
#include <iomanip>

std::string MySerialCom::sign_sensorData = "ALL_INFO\n";
//std::string MySerialCom::sign_sensorData = "Y\r\n";

MySerialCom::MySerialCom(const any_type &port_name, ros::NodeHandle n): pSerialPort(NULL), loop_rate(10), nh_(n), spinner(5) {
    pSerialPort = new boost::asio::serial_port(io_sev);

    if(pSerialPort) {
        init_port(port_name, 8);
    }
}

MySerialCom::~MySerialCom() {
    if(pSerialPort) {
        delete  pSerialPort;
    }
}

bool MySerialCom::init_port(const any_type port, const unsigned int char_size) {
    if(!pSerialPort) {
        return false;
    }
    serial_sub = nh_.subscribe<rxb_msgs::serialCommu>("serialport/commu", 1, &MySerialCom::serialPort_subFun, this);//hw通知串口发送数据过来
    serial_pub = nh_.advertise<rxb_msgs::sensorRead>("serialport/sensor_data", 1);//发送数据给上位机界面和hw
    //joint_zero_sub = nh_.subscribe("control_plugin/joint_zero_signal", 1, &MySerialCom::joint_zero_subFun, this);


    pSerialPort->open(port, ec);
    pSerialPort->set_option(boost::asio::serial_port::baud_rate(115200), ec);
    pSerialPort->set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none), ec);
    pSerialPort->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none ), ec);
    pSerialPort->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one), ec);
    pSerialPort->set_option(boost::asio::serial_port::character_size(char_size));

    spinner.start();
    return true;
}

void MySerialCom::serialPort_subFun(const rxb_msgs::serialCommu::ConstPtr &commu) {
    if(commu->data[0] == 'S' && commu->data[1] == 'A') {
        if(commu->data[2] == 0x66 &&//对应char f
                commu->data[3] == 0x66 &&
                commu->data[4] == 0x66 &&
                commu->data[5] == 0x66 &&
                commu->data[6] == 0x66 &&
                commu->data[7] == 0x66 &&
                commu->data[8] == 0x66 &&
                commu->data[9] == 0x66 &&
                commu->data[10] == 0x66 &&
                commu->data[11] == 0x66) {
//            write_to_serial(sign_sensorData, 9);//"A\r\n"
//            write_to_serial("Y\r\n", 4);//"A\r\n"
            write_to_serial("Y\r\n", 3);
            //write_to_serial("\n", 1);
        }
    }
    if(commu->data[0] == 'M') {// no use
        switch (commu->data[1]) {
            case 'R'://reset
                break;
            case 'M'://mode change
                break;
            case 'C'://config
                break;
            case 'P'://position mode
                break;
            case 'V'://velocity mode
                break;
            default:
                ROS_ERROR("Wrong Serial Command! ");
                break;
        }
    }
}


void MySerialCom::write_to_serial(const std::string send_data, size_t len) {
    boost::asio::write(*pSerialPort, boost::asio::buffer(send_data, len));
}


void MySerialCom::read_sensor_from_serial() {
    while(1) {
        boost::asio::streambuf buff;
        boost::asio::read_until(*pSerialPort, buff, "\r\n");//读到0x0d 0x0a(换行、结束符号)时结束读,如果没读到则会阻塞在这里
        std::istream is(&buff);

        is >> rec_data;
        //cout << rec_data.length() << endl;//不计算最后的回车和换行数目
        //for(int i = 0; i < rec_data.length(); i++){
        //    printf("\t%c", rec_data[i]);
        //}

        if(sensordata_analysis(rec_data) == true) {
            rxb_msgs::sensorRead sensorData;
            sensorData.cable_force.resize(21);
            for(int i = 0; i < 21; i++) {
                sensorData.cable_force[i] = cable_force[i];
//                cout << cable_force[i] << '\t';


            }
            cout << '\n';
            serial_pub.publish(sensorData);
        } else {
            ROS_ERROR("Serial_Com: data analysis wrong");
        }
        rec_data.clear();
    }
}


//void MySerialCom::handle_write(const boost::system::error_code &ec, std::size_t bytes_transferred) {
//    if(!ec) {
//        std::cout << "Enter write_handle Function......."  << std::endl;
//        std::cout << "I want to send the data :" << write_data[0] << write_data[1] << write_data[2] << std::endl;
//    }
//

//串口数据解析
bool MySerialCom::sensordata_analysis(std::string data) {
    char crc1;
    cout << "x" << endl;
    if(data[0] == 'a' && data[1] == 'a') {
        if(data[2] == 'C') {
            crc1 = (data[6] ^ data[11] ^ data[16] ^ data[21] ^ data[26] ^ data[31] ^ data[36] ^ data[41] ^ data[46] ^ data[51] ^ data[56] ^ data[61] ^
                    data[66] ^ data[71] ^ data[76] ^ data[81] ^ data[86] ^ data[91] ^ data[96] ^ data[101] ^ data[106]) + 0x30;//校验位，12个张力传感器，
            //printf("crc1 = %x\n",crc1);//用16进制输出
            if(crc1 != data[108]) {
                ROS_ERROR(" Serial_Com: Cable_force data CRC1 is wrong !");
                return false;
            } else {
                for(int i = 0; i < 21; i++) {
                    cable_force[g_cableForce_ID[i] - 1] = (100 * (float)(data[3 + 5 * i] - 0x30) + 10 * (float)(data[4 + 5 * i] - 0x30) + (float)(data[5 + 5 * i] - 0x30) + 0.1 * (float)(data[6 + 5 * i] - 0x30) + 0.01 * (float)(data[7 + 5 * i] - 0x30));
                }
                for(int i = 0; i < 21; i++) {
                    cout << "force" << i+1 << " : "<< cable_force[i] << endl;
                }
                return true;
            }//省略编码器读数
        } else {
            ROS_ERROR(" Serial_Com: 'C'not exist !");
            cout << data[2] << endl;
            return false;
        }
    } else {
        ROS_ERROR(" Serial_Com: the data doesnt begin with 'a' 'a' !");
        return false;
    }
}

//阻塞函数
//void MySerialCom::call_handle() {
//    ROS_INFO(" Serial_Com: Enter the io_sev.run() Function ......");
//    io_sev.run();//阻塞在这里执行任务
//}


