#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include<IcartStateSubscriber.h>
#include<cmath>
#include <Eigen/Dense>
#include <iostream>
#include <tf/transform_listener.h>
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
    ros::init(argc,argv,"icart_pid_velocity_control");
    ros::NodeHandle nh;
    iros::IcartStateSubscriber stateSubscriber(nh);
    // double publishHz_= 10.0;
    // double distance;
    //----------------PID control-----------------------
    ros::Rate loopRate(100.0);
    // ros::Rate loopRate(publishHz_);

    double Kp = 0.05;
    double Ki = 0.005;
    double Kd = 0.001;
    double dt = 0.008;
    Eigen::Vector2f pre_stateDiffs = Eigen::VectorXf::Zero(2);
    Eigen::Vector2f stateDiffs = Eigen::VectorXf::Zero(2);
    Eigen::Vector2f integral = Eigen::VectorXf::Zero(2);
    Eigen::Vector2f targetGoal = Eigen::VectorXf::Zero(2);
    // Eigen::Vector2f Pout = Eigen::VectorXf::Zero(2);
    //  Eigen::Vector2d integral = Eigen::VectorXd::Zero(2);
    // Eigen::Vector2f Iout = Eigen::VectorXf::Zero(2);
    //  Eigen::Vector2f derivative = Eigen::VectorXf::Zero(2);
    //  Eigen::Vector2f Dout = Eigen::VectorXf::Zero(2);
    //  Eigen::Vector2f output = Eigen::VectorXf::Zero(2);
    Eigen::Vector2f humanPose;
    Eigen::Vector2f tolerance;
    tolerance(0) = 0.2;
    tolerance(1) = deg2rad(10);
    humanPose(0) = 5;
    humanPose(1) = deg2rad(40.0f);
    targetGoal(0) = 0.5; 
    targetGoal(1) = deg2rad(0);

    geometry_msgs::Twist velo_msgs;

    velo_pub = nh.advertise<geometry_msgs::Twist>("/icart_mini/cmd_vel",100);
    // velo_pub = nh.advertise<geometry_msgs::Twist>("/ypspur_ros/cmd_vel",100);
    tf::StampedTransform transform;
    int i = 0;
    double theta_rb_pre = 0;
    while (ros::ok())
    // while(true) 
    {
        ros::spinOnce();
        iros::StampedState icartState = stateSubscriber.getIcartState();
        theta_rb_pre = icartState.getYaw();
        Eigen::Vector2f stateDiffs ;
        try{
        // stateSubscriber.listener.lookupTransform("/odom", "/human",ros::Time(0), transform);
        stateDiffs(0) = targetGoal(0) - humanPose(0);
        stateDiffs(1) = targetGoal(1) - humanPose(1);
        // stateDiffs(1) = targetGoal(1) - icartState.getY();
        
        // Proportional term
        Eigen::Vector2f Pout = Kp * stateDiffs;

        // Integral term
        Eigen::Vector2f integral;
        integral(0) +=  stateDiffs(0)  * dt;
        integral(1) +=  stateDiffs(1)  * dt;

        Eigen::Vector2f Iout = Ki * integral;
        
        // Derivative term
        Eigen::Vector2f derivative = (stateDiffs - pre_stateDiffs) / dt;
        Eigen::Vector2f Dout = Kd * derivative;

        // Calculate total output for velocity
        Eigen::Vector2f output = Pout + Iout + Dout;
        //set the velocity
        if (output(0) >= 0.5)
        {
            velo_msgs.linear.x = 0.5;

        }
        else
        {
            velo_msgs.linear.x = output(0);
        }
        if (humanPose(1) > targetGoal(1) && humanPose(1) >= 1.5)
        {
            velo_msgs.angular.z = -1.5;
        }
        else
        {
            velo_msgs.angular.z = 1.5;
        }
        velo_pub.publish(velo_msgs);

        // Save error to previous error
        pre_stateDiffs = stateDiffs;

        if((std::abs(stateDiffs(0)) <= tolerance(0)) && (std::abs(stateDiffs(1)) <= tolerance(1)))
        {
            velo_msgs.linear.x = 0.0;
            velo_msgs.angular.z = 0.0;
            velo_pub.publish(velo_msgs);
            break;
        }
   
        // std transform.getRotation().getAngle();
        double x_human = transform.getOrigin().x();
        double y_human = transform.getOrigin().y();
        double x_robot = icartState.getX();
        double y_robot = icartState.getY();
        std::cout << 
        x_human << " " <<
        y_human << " " <<
        x_robot << " " <<
        y_robot << std::endl;
        double distance = sqrt(pow((y_human - y_robot),2) + pow((x_human - x_robot),2));
        // std::cout << distance << std::endl;
        double theta_hr = atan2((y_human - y_robot),(x_human - x_robot));
        double theta_rb = icartState.getYaw() - theta_rb_pre;
        double theta = theta_rb - theta_hr;

        humanPose(0) = distance;
        humanPose(1) = theta;

        
        // printf("Icart state\n");
        // printf("Num = %d\n", i);
        // printf("x = %f, y = %f\n", icartState.getX(), icartState.getY());
        // // printf("YAW (degree) = %f \n", rad2deg(icartState.getYaw()));
        // // printf("vx = %f, wz [rad]  = %f\n", icartState.getVx(), icartState.getWz());
        // printf("controlInputs: vx = %f, wz [rad] = %f\n",output(0), output(1));
        // std::cout << "delta x = " <<  std::abs(stateDiffs(0)) <<" delta angle = "<< " " << std::abs(stateDiffs(1)) << std::endl;
        // printf("\n");
        i++;

        // distance = std::sqrt(std::pow(icartState.getX(),2)+ std::pow(icartState.getY(),2) + std::pow(icartState.getZ(),2));

        // if(distance> 8) break;

        loopRate.sleep();

    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    }
    
    // velo_msgs.linear.x = 0.0;
    // velo_pub.publish(velo_msgs);

    // ros::Rate loopRate(10.0);
    // for (int i = 0; i < 100; i++) {
    //     ros::spinOnce();
    //     iros::StampedState icartState = stateSubscriber.getIcartState();
    //     printf("Num = %d\n", i);
    //     printf("x = %f, y = %f, z = %f\n", icartState.getX(), icartState.getY(), icartState.getZ());
    //     printf("qx = %f, qy = %f, qz = %f, qw = %f\n", icartState.getQx(), icartState.getQy(), icartState.getQz(), icartState.getQw());
    //     printf("vx = %f, vy = %f, vz = %f\n", icartState.getVx(), icartState.getVy(), icartState.getVz());
    //     printf("wx = %f, wy = %f, wz = %f\n", icartState.getWx(), icartState.getWy(), icartState.getWz());
    //     printf("\n");
    //     loopRate.sleep();
    // }


    return 0;
}