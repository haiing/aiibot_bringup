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

double x = 0.0;
double y = 0.0;
double th = 0.0;
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;
unsigned char startByte = 0xff;
unsigned char endByte = 0xfe;

static ros::Time current_time;
static ros::Time last_time;
geometry_msgs::Quaternion odom_quat;

unsigned char buf1[10];
float Data_vx;
float Data_vth;

int HerdWore_vx;
int HerdWore_vth;


int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

    static std::string Base_Port = "";
    if(!nh_private.getParam("Base_Port", Base_Port)){
        Base_Port = "/dev/ttyUSB1";
    }

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    io_service iosev;
    serial_port sp(iosev);
    sp.open(Base_Port);
    sp.set_option(serial_port::baud_rate(9600));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    while(ros::ok()){
        current_time = ros::Time::now();

        read(sp, buffer(buf1));
        if(buf1[0] == startByte && buf1[9] == endByte){
            HerdWore_vx = (buf1[4] << 24 | buf1[3] << 16 | buf1[2] << 8 | buf1[1]);
            HerdWore_vth = (buf1[8] << 24 | buf1[7] << 16 | buf1[6] << 8 | buf1[5]);
            Data_vx = *((float *)&HerdWore_vx);
            Data_vth = *((float *)&HerdWore_vth);
            vx = (double)Data_vx;
            vth = (double)Data_vth;

            ROS_INFO("Encores linear is %f", vx);
            ROS_INFO("Encores angular is %f\n ", vth);

            if(vx <= -5.0 || vx >= 5.0)
                vx = 0.0;
            if(vth <= -5.0 || vth >= 5.0)
                vth = 0.0;

            //compute odometry in a typical way given the velocities of the robot
            double dt = (current_time - last_time).toSec();
            double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
            double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
            double delta_th = vth * dt;
            x += delta_x;
            y += delta_y;
            th += delta_th;
        }
        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);
        last_time = current_time;
    }
    iosev.run();
}

