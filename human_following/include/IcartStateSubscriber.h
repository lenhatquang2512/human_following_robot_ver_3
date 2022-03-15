#pragma once

#include <tf/transform_listener.h>
#include <StampedState.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>

namespace iros{
    class IcartStateSubscriber
    {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber stateSub_;
        std::string topicName;
        StampedState icartState_;

        std::string child_frame_id_;
        std_msgs::Header header_;
        
        void quat2rpy(float qx, float qy, float qz, float qw,
                    double *roll, double *pitch, double *yaw) {
            // roll (x-axis rotation)
            double sinr_cosp = 2 * (qw * qx + qy * qz);
            double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
            *roll = std::atan2(sinr_cosp, cosr_cosp);

            // pitch (y-axis rotation)
            double sinp = 2 * (qw * qy - qz * qx);
            if (std::abs(sinp) >= 1)
                *pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
            else
                *pitch = std::asin(sinp);

            // yaw (z-axis rotation)
            double siny_cosp = 2 * (qw * qz + qx * qy);
            double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
            *yaw = std::atan2(siny_cosp, cosy_cosp);
        }
    public:
        // tf::TransformListener listener;
        IcartStateSubscriber(ros::NodeHandle nh);
        // ~IcartStateSubscriber();
        inline StampedState getIcartState(void);
        void statesCallback(const nav_msgs::Odometry::ConstPtr &msg);
    }; //class icartStateSubscriber
    
    IcartStateSubscriber::IcartStateSubscriber(ros::NodeHandle nh):
        nh_(nh),
        topicName("/icart_mini/odom")
    {
        // topicName = "/ypspur_ros/odom";
        stateSub_ = nh_.subscribe(topicName,100,&IcartStateSubscriber::statesCallback,this);
        
    }

    inline StampedState IcartStateSubscriber::getIcartState(void)
    {
        return icartState_;
    }

    void IcartStateSubscriber::statesCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        //pose
        float x = msg->pose.pose.position.x;
        float y = msg->pose.pose.position.y;
        float z = msg->pose.pose.position.z;

        //orientation
        float qx = msg->pose.pose.orientation.x;
        float qy = msg->pose.pose.orientation.y;
        float qz = msg->pose.pose.orientation.z;
        float qw = msg->pose.pose.orientation.w;

        double roll, pitch, yaw;
        quat2rpy(qx, qy, qz, qw, &roll, &pitch, &yaw);

        //linear velocity
        float vx = msg->twist.twist.linear.x;
        float vy = msg->twist.twist.linear.y;
        float vz = msg->twist.twist.linear.z;
        
        //angular velocity
        float wx = msg->twist.twist.angular.x;
        float wy = msg->twist.twist.angular.y;
        float wz = msg->twist.twist.angular.z;

        //setting for icartstate
        icartState_.setStamp(ros::Time::now().toSec());
        icartState_.setPose(x, y, z, roll, pitch, yaw);
        icartState_.setQuaternion(qx,qy,qz,qw);
        icartState_.setVelocities(vx,vy,vz,wx,wy,wz);
    }
    
    // IcartStateSubscriber::~IcartStateSubscriber()
    // {
    // }
    
} //namespace iros