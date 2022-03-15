#pragma once

#include <ros/ros.h>
#include <StampedState.h>
#include <geometry_msgs/Pose.h>

namespace iros
{
    class humanStateSubscriber
    {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber human_state_sub_;
        std::string topic_name_;
        StampedState humanState_;

    public:
        humanStateSubscriber(ros::NodeHandle nh);
        inline StampedState getHumanState(void);
        void human_state_CB(const geometry_msgs::Pose::ConstPtr &msg);
    };
    
    humanStateSubscriber::humanStateSubscriber(ros::NodeHandle nh):
        nh_(nh),
        topic_name_("/object3d_detector/human_pose")
    {
        human_state_sub_ = nh_.subscribe(topic_name_,100,&humanStateSubscriber::human_state_CB,this);
    }

    inline StampedState humanStateSubscriber::getHumanState(void)
    {
        return humanState_;
    }

    void humanStateSubscriber::human_state_CB(const geometry_msgs::Pose::ConstPtr &msg)
    {
        //position
        float x = msg->position.x;
        float y = msg->position.y;
        float z = msg->position.z;

        //orientation
        float qx = msg->orientation.x;
        float qy = msg->orientation.y;
        float qz = msg->orientation.z;
        float qw = msg->orientation.w;

        //setting for human state
        humanState_.setPoseforHuman(x,y,z);
        humanState_.setQuaternion(qx,qy,qz,qw);


    }
    
    
} //namespace iros