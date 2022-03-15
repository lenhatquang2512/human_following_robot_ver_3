#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <Eigen/Dense>
#include <iostream>
#include <tf/transform_listener.h>
#include <humanStateSubscriber.h>
#include <IcartStateSubscriber.h>



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
    ros::init(argc,argv,"icart_BOTH_velo_pid");
    ros::NodeHandle nh;
    iros::humanStateSubscriber peopleStateSubscriber(nh);
    iros::IcartStateSubscriber robotStateSubcriber(nh);


    geometry_msgs::Twist velo_msgs;

    velo_pub = nh.advertise<geometry_msgs::Twist>("/icart_mini/cmd_vel",100);
    // velo_pub = nh.advertise<geometry_msgs::Twist>("/ypspur_ros/cmd_vel",100);


    float target_goal_angular =   deg2rad(5.0);
    float target_goal_linear  =  1.5f;


    //----------------PID control-----------------------
    ros::Rate loopRate(100.0);

    Eigen::Vector2f Kp;
    Eigen::Vector2f Ki;
    Eigen::Vector2f Kd;
    double dt = 0.001;
    Eigen::Vector2f pre_stateDiffs = Eigen::VectorXf::Zero(2);
    Eigen::Vector2f stateDiffs = Eigen::VectorXf::Zero(2);
    Eigen::Vector2f integral = Eigen::VectorXf::Zero(2);
    Eigen::Vector2f targetGoal = Eigen::VectorXf::Zero(2);

    Eigen::Vector2f humanPose;
    Eigen::Vector2f tolerance;

    Kp(0) = 2;
    Kp(1) = 0.05;
    Ki(0) = 0.0023;
    Ki(1) = 0.001;
    Kd(0) = 0.01;
    Kd(1) = 0.01;

    tolerance(0) = 0.0;
    tolerance(1) = 0.1;

    targetGoal(0) = target_goal_angular;
    targetGoal(1) = target_goal_linear;

    FILE *fp = fopen("icart_x_trajectory.txt","w");

    int i = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        iros::StampedState humanState = peopleStateSubscriber.getHumanState();
        iros::StampedState icartState = robotStateSubcriber.getIcartState();

        double distance = std::sqrt((double)std::pow(humanState.getX(),2)+ (double)std::pow(humanState.getY(),2));
        double angle =  std::atan2((double) humanState.getY() , (double) humanState.getX());
        std::cout << "atan2: " << angle << std::endl;
        Eigen::Vector2f stateDiffs;
        stateDiffs(0) = (float) angle - targetGoal(0);
        stateDiffs(1) = (float) distance -targetGoal(1);

        // Proportional term
        Eigen::Vector2f Pout;
        Pout(0) = Kp(0) *stateDiffs(0);
        Pout(1) = Kp(1) *stateDiffs(1);
        // Integral term
        integral(0) +=  stateDiffs(0)  * dt;
        integral(1) +=  stateDiffs(1)  * dt;

        Eigen::Vector2f Iout;
        Iout(0) = Ki(0) * integral(0);
        Iout(1) = Ki(1) * integral(1);

        // Derivative term
        Eigen::Vector2f derivative = (stateDiffs - pre_stateDiffs) / dt;
        Eigen::Vector2f Dout;
        Dout(0) = Kd(0) * derivative(0);
        Dout(1) = Kd(1) * derivative(1);
        // Calculate total output for velocity
        Eigen::Vector2f output = Pout + Iout + Dout;

        //publish the velocities
        velo_msgs.linear.x = output(1);

        velo_msgs.angular.z = output(0) ;

        std::cout << "Test " << output(0) << std::endl;
        velo_pub.publish(velo_msgs);

        // Save error to previous error
        pre_stateDiffs = stateDiffs;

        //if smalller than tolerance then break
        if((std::abs(stateDiffs(0)) <= tolerance(0)) && (std::abs(stateDiffs(1)) <= tolerance(1)))
        {
            velo_msgs.linear.x = 0.0;
            velo_msgs.angular.z = 0.0;
            velo_pub.publish(velo_msgs);
            break;
        }

        fprintf(fp,"%f %lf\n",icartState.getX(),icartState.getStamp());

        printf("Icart state\n");
        printf("Num = %d\n", i);
        printf("Time stamp = %lf\n",icartState.getStamp());
        printf("x = %f, y = %f\n", icartState.getX(), icartState.getY());
        printf("YAW (degree) = %f \n", rad2deg(icartState.getYaw()));
        printf("vx = %f, wz [rad]  = %f\n", icartState.getVx(), icartState.getWz());
        printf("controlInputs: vx = %f, wz [rad] = %f\n",output(1), output(0));
        std::cout << "del_x = " <<  stateDiffs(1) <<" del_angle = "<< " " << stateDiffs(0) << std::endl;
        printf("\n");
        i++;
        
        // printf("Human state\n");
        // printf("Num = %d\n", i);
        // printf("x = %f, y = %f\n", humanState.getX(), humanState.getY());
        // printf("controlInputs: vx = %f, wz [rad] = %f\n",output(0), output(1));
        // std::cout << "del_angle = " <<  std::abs(stateDiffs(0)) <<" del_x = "<< " " << std::abs(stateDiffs(1)) << std::endl;
        // printf("\n");
        // i++;
        

        loopRate.sleep();

    }

    fclose(fp);
    
    return 0;
}