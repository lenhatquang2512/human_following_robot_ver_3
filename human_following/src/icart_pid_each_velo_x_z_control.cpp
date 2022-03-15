#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include<IcartStateSubscriber.h>
#include<cmath>
#include <Eigen/Dense>
#include <iostream>

#include <boost/algorithm/string.hpp>

float deg2rad(float angle)
{
    return M_PI * angle / 180.0;
}

float rad2deg(float angle)
{
    return angle * 180.0/ M_PI;
}

ros::Publisher velo_pub;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"icart_pid_each_velo_x_z_control");
    ros::NodeHandle nh;
    iros::IcartStateSubscriber stateSubscriber(nh);

    geometry_msgs::Twist velo_msgs;
    velo_pub = nh.advertise<geometry_msgs::Twist>("/icart_mini/cmd_vel",100);
    // velo_pub = nh.advertise<geometry_msgs::Twist>("/ypspur_ros/cmd_vel",100);

    float target_goal_angular =   deg2rad(90.0);
    float target_goal_linear  =  2.5f;

    std::cout << "GOAL ANGULAR IS  " << rad2deg(target_goal_angular) << "[deg] " << std::endl;
    std::cout << "GOAL LINEAR IS " << target_goal_linear << "[m]" << std::endl;


    //----------------PID control for wz-----------------------
    ros::Rate loopRate_for_angle_z(100.0);

    double Kp = 0.5;
    double Ki = 0.001;
    double Kd = 0.01;
    double dt = 0.001;
    double tolerance = 0.02;

    //for angular z
    float pre_stateDiffs_for_angle_z = 0.0f;
    // float stateDiffs_for_angle_z = 0.0f;
    float integral_for_angle_z = 0.0f;
    float  targetGoal_angle_z = target_goal_angular;
    
    int i = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        iros::StampedState icartState_for_angle_z = stateSubscriber.getIcartState();

        float stateDiffs_for_angle_z ;
        stateDiffs_for_angle_z = targetGoal_angle_z - icartState_for_angle_z.getYaw();
        
        // Proportional term
        float Pout_for_angle_z = Kp * stateDiffs_for_angle_z;

        // Integral term
        // float integral_for_angle_z;
        integral_for_angle_z +=  stateDiffs_for_angle_z  * dt;
    

        float Iout_for_angle_z = Ki * integral_for_angle_z;
        
        // Derivative term
        float derivative_for_angle_z = (stateDiffs_for_angle_z - pre_stateDiffs_for_angle_z) / dt;
        float Dout_for_angle_z = Kd * derivative_for_angle_z;

        // Calculate total output for velocity
        float output_for_angle_z = Pout_for_angle_z + Iout_for_angle_z + Dout_for_angle_z;
        
        //set the velocity
        velo_msgs.angular.z = output_for_angle_z;
        velo_pub.publish(velo_msgs);

        // Save error to previous error
        pre_stateDiffs_for_angle_z = stateDiffs_for_angle_z;

        if(std::abs(stateDiffs_for_angle_z) <= tolerance)
        {
            velo_msgs.angular.z = 0.0;
            velo_pub.publish(velo_msgs);
            break;
        }

        
        printf("Icart state for angular control\n");
        printf("Num = %d\n", i);
        printf("x = %f, y = %f\n", icartState_for_angle_z.getX(), icartState_for_angle_z.getY());
        printf("YAW (degree) = %f \n", rad2deg(icartState_for_angle_z.getYaw()));
        printf("wz [rad]  = %f\n",icartState_for_angle_z.getWz());
        printf("controlInputs: wz [rad] = %f\n",output_for_angle_z);
        //std::cout << " delta angle = "<< " " << std::abs(stateDiffs_for_angle_z) << std::endl;
        printf("\n");
        i++;

        loopRate_for_angle_z.sleep();

    }

    std::cout << "-------------MOVING TO VX LINEAR CONTROL---------------" << std::endl;


    //----------------PID control for vx-----------------------
    ros::Rate loopRate_for_x(100.0);

    //for linear x
    float pre_stateDiffs_for_x = 0.0f;
    // float stateDiffs_for_x = 0.0f;
    float integral_for_x = 0.0f;
    float  targetGoal_x = target_goal_linear;

    while (ros::ok())
    {
        ros::spinOnce();
        iros::StampedState icartState_for_x = stateSubscriber.getIcartState();

        float distance;

        distance = std::sqrt(std::pow(icartState_for_x.getX(),2)+ std::pow(icartState_for_x.getY(),2) + std::pow(icartState_for_x.getZ(),2));

        float stateDiffs_for_x ;
        // stateDiffs_for_x = distance -targetGoal_x;
         stateDiffs_for_x = targetGoal_x -  distance;
        
        // Proportional term
        float Pout_for_x = Kp * stateDiffs_for_x;

        // Integral term
        // float integral_for_x;
        integral_for_x +=  stateDiffs_for_x  * dt;
    

        float Iout_for_x = Ki * integral_for_x;
        
        // Derivative term
        float derivative_for_x = (stateDiffs_for_x - pre_stateDiffs_for_x) / dt;
        float Dout_for_x = Kd * derivative_for_x;

        // Calculate total output for velocity
        float output_for_x = Pout_for_x + Iout_for_x + Dout_for_x;
        
        //set the velocity
        velo_msgs.linear.x = output_for_x;
        velo_pub.publish(velo_msgs);

        // Save error to previous error
        pre_stateDiffs_for_x = stateDiffs_for_x;

        if(std::abs(stateDiffs_for_x) <= tolerance)
        {
            velo_msgs.linear.x = 0.0;
            velo_pub.publish(velo_msgs);
            break;
        }

        
        printf("Icart state for linear control \n");
        printf("Num = %d\n", i);
        printf("x = %f, y = %f, distance =  %f\n", icartState_for_x.getX(), icartState_for_x.getY(),distance);
        //printf("YAW (degree) = %f \n", rad2deg(icartState.getYaw()));
        printf("vx = %f\n", icartState_for_x.getVx());
        printf("controlInputs: vx = %f \n",output_for_x);
        //std::cout << "delta x = " <<  std::abs(stateDiffs_for_x) << std::endl;
        printf("\n");
        i++;

        loopRate_for_x.sleep();

    }

    return 0;
}

