#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include<IcartStateSubscriber.h>
#include<cmath>

// #include<ypspur_ros/ControlMode.h>

float rad2deg(float angle)
{
    return angle * 180.0/ M_PI;
}

ros::Publisher velo_pub;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"icart_test");
    ros::NodeHandle nh;
    iros::IcartStateSubscriber stateSubscriber(nh);

    iros::StampedState icartState_initial = stateSubscriber.getIcartState();

    double publishHz_= 10.0;
    double distance;

    //small test
    geometry_msgs::Twist velo_msgs;

    // velo_pub = nh.advertise<geometry_msgs::Twist>("/icart_mini/cmd_vel",100);
    velo_pub = nh.advertise<geometry_msgs::Twist>("/ypspur_ros/cmd_vel",100);

    
    ros::Rate loopRate(publishHz_);
    int i = 0;
    double old_time = ros::Time::now().toSec();
    double initial_yaw = icartState_initial.getYaw();
    std::cout << "Yaw at the beginning is " << initial_yaw << std::endl;
    while (ros::ok()) 
    {
        ros::spinOnce();
        iros::StampedState icartState = stateSubscriber.getIcartState();

        velo_msgs.linear.x = 0.2;
        // velo_msgs.angular.z = 0.2;
        velo_pub.publish(velo_msgs);


        printf("Num = %d\n", i);
        printf("x = %f, y = %f, z = %f\n", icartState.getX(), icartState.getY(), icartState.getZ());
        // printf("qx = %f, qy = %f, qz = %f, qw = %f\n", icartState.getQx(), icartState.getQy(), icartState.getQz(), icartState.getQw());
        // printf("vx = %f, vy = %f, vz = %f\n", icartState.getVx(), icartState.getVy(), icartState.getVz());
        // printf("wx = %f, wy = %f, wz = %f\n", icartState.getWx(), icartState.getWy(), icartState.getWz());
        printf(" wz = %f\n", icartState.getWz());
        printf(" YAW  = %f\n", rad2deg(icartState.getYaw()) );
        printf("\n");
        i++;

        distance = std::sqrt(std::pow(icartState.getX(),2)+ std::pow(icartState.getY(),2) + std::pow(icartState.getZ(),2));

        if(distance > 0.5)
        {
            velo_msgs.linear.x = 0.0;
            velo_pub.publish(velo_msgs);
            break;

        }

        // double cur_time = ros::Time::now().toSec();
        // if(cur_time - old_time > 3.0) 
        // {
        //     // velo_msgs.linear.x = 0.0;
        //     velo_msgs.angular.z = 0.0;
        //     velo_pub.publish(velo_msgs);
        //     break;
        // }

        // if(rad2deg(icartState.getYaw() - initial_yaw) > 20.0 )
        // {
        //     velo_msgs.angular.z = 0.0;
        //     velo_pub.publish(velo_msgs);
        //     break;
        // }
        loopRate.sleep();

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

