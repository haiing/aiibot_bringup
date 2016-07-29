#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <iomanip>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <math.h>

using namespace std;
using namespace boost::asio;

static std::string Base_Port = "";
static std::string cmd_topic = "";
unsigned char startByte = 0xff;
unsigned char endByte = 0xfe;

union Max_Value{
    unsigned char buf[8];
    struct _Float_{
        float _float_vX;
        float _float_vTh;
    }Float_RAM;
}Send_Data;

io_service iosev;
serial_port sp(iosev);

void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
    Send_Data.Float_RAM._float_vX = twist_aux.linear.x;
    Send_Data.Float_RAM._float_vTh = twist_aux.angular.z;

    ROS_INFO("cmd_vel linear.x  is %f", twist_aux.linear.x);
    ROS_INFO("cmd_vel angular.z is %f\n", twist_aux.angular.z);

    write(sp, buffer(&startByte, 1));
    write(sp, buffer(Send_Data.buf, 8));
    write(sp, buffer(&endByte, 1));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "velocity_publisher");
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

    if(!nh_private.getParam("cmd_topic", cmd_topic)){
        cmd_topic = "cmd_vel";
    }

    if(!nh_private.getParam("Base_Port", Base_Port)){
        Base_Port = "/dev/ttyUSB1";
    }

    std::cout<<"cmd_vel: "<<cmd_topic<< "\tPort: "<< Base_Port<<std::endl;
    ros::Subscriber cmd_vel_sub = nh.subscribe(cmd_topic, 10, cmd_velCallback);

    sp.open(Base_Port);
    sp.set_option(serial_port::baud_rate(9600));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));

    ros::spin();                    // check for incoming messages
}

