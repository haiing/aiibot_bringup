#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <iomanip>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <math.h>

using namespace std;
using namespace boost::asio;

double vx = 0.0;
double vth = 0.0;
unsigned char startByte = 0xff;
unsigned char endByte = 0xfe;
std::string Base_Port = "";

unsigned char buf1[10];
float Data_vx;
float Data_vth;
int HerdWore_vx;
int HerdWore_vth;
double Last_vx;
double Last_vth;
ros::Publisher chatter_pub;


union Max_Value{
    unsigned char buf[8];
    struct _Float_{
        float _float_vX;
        float _float_vTh;
    }Float_RAM;
}Send_Data;

void Monitor_Serial_Data(const geometry_msgs::Twist &msg)
{
    io_service iosev;
    serial_port sp(iosev, Base_Port);
    sp.set_option(serial_port::baud_rate(9600));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));

    //------------------------ Send data -------------------------------------
    float msg_vx = msg.linear.x;
    float msg_vth = msg.angular.z;

    Send_Data.Float_RAM._float_vX = msg_vx;
    Send_Data.Float_RAM._float_vTh = msg_vth;
    // ROS_INFO("cmd_vel linear.x  is %f",msg_vx);
    // ROS_INFO("cmd_vel angular.z is %f\n",msg_vth);

    write(sp, buffer(&startByte, 1));
    write(sp, buffer(Send_Data.buf, 8));
    write(sp, buffer(&endByte, 1));

    //------------------------- Recieve data ------------------------------------
    geometry_msgs::Twist msg_encoder;
    read(sp, buffer(buf1));
    if(buf1[0] == 0xff && buf1[9] == 0xfe)//对串口数据进行头尾校验
    {
        HerdWore_vx  = (buf1[4]<<24 | buf1[3]<<16 | buf1[2]<<8 | buf1[1]); //获取串口速度数据
        HerdWore_vth = (buf1[8]<<24 | buf1[7]<<16 | buf1[6]<<8 | buf1[5]);
        Data_vx  = *((float*)&HerdWore_vx);
        Data_vth = *((float*)&HerdWore_vth);
        vx = (double)Data_vx;
        vth = (double)Data_vth;
    }
    msg_encoder.linear.x  = vx;//填充odom_tf_package::serial_boost消息变量
    msg_encoder.angular.z = vth;
    chatter_pub.publish(msg_encoder);//在"talker_boost_serial"话题下发布数据
    ROS_INFO("msg_encoder.linear_x  is %f",msg_encoder.linear.x);
    ROS_INFO("msg_encoder.angular_z is %f",msg_encoder.angular.z);

    iosev.run();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "listener_cmd_vel");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string cmd_topic = "";
    if(!nh_private.getParam("cmd_topic", cmd_topic)){
        cmd_topic = "/cmd_vel";
    }
    if(!nh_private.getParam("Base_Port", Base_Port)){
        Base_Port = "/dev/ttyUSB0";
    }
    std::string talker_topic = "";
    if(!nh_private.getParam("talker_topic", talker_topic)){
        talker_topic = "/talker_boost_serial";
    }
    std::cout<<"cmd_vel: "<<cmd_topic<< "\tPort: "<< Base_Port<<std::endl;

    ros::Subscriber cmd_vel_sub = nh.subscribe(cmd_topic, 10, Monitor_Serial_Data);
    chatter_pub = nh.advertise<geometry_msgs::Twist>(talker_topic, 10);//发布话题
    ros::spin();
}

