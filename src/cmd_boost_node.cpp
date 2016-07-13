#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <iomanip>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <math.h>

class SubscribeAndPublish 
{ 
public: 
    double x;
    double y ;
    double th;

    double vx;
    double vy;
    double vth;

    ros::NodeHandle n; 
    ros::Publisher odom_pub;
    ros::Subscriber sub;
    ros::Time current_time, last_time;
    tf::TransformBroadcaster odom_broadcaster;

    SubscribeAndPublish(std::string talker_topic)
    { 
        odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
        sub = n.subscribe(talker_topic, 10, &SubscribeAndPublish::chatterCallback,this); //监听
    } 

    void chatterCallback(const geometry_msgs::Twist & msg) 
    { 
        current_time = ros::Time::now();
        vx = msg.linear.x;
        vy = msg.linear.y;
        vth = msg.angular.z;    
        if(vx <= -5.0 || vx >= 5.0)   
            vx = 0.0;
        if(vth <= -5.0 || vth >= 5.0)
            vth = 0.0;
        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        odom_pub.publish(odom);
        last_time = current_time;
        //std::cout<<"Time: "<< dt<<std::endl; 
    } 

};

int main(int argc, char **argv) 
{ 

    ros::init(argc, argv, "Odometry_publisher");
    ros::NodeHandle nh_private("~");
    std::string talker_topic = "";
    if(!nh_private.getParam("talker_topic", talker_topic)){
        talker_topic = "/talker_boost_serial";
    }

    SubscribeAndPublish SAPObject(talker_topic); 
    
    SAPObject.current_time = ros::Time::now();
    SAPObject.last_time = ros::Time::now();
    ros::spin();
}





