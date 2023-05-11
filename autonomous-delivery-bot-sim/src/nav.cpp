#include <iostream>
# include <vector>
#include <ros/time.h>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>

#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include <visualization_msgs/Marker.h>

class Robot
{
public:
    std::string m_name {};
    int m_gotRobotNameFlag{0};
    ros::Subscriber nameSub{};

    Robot(ros::NodeHandle* n)
    {
        nameSub = n->subscribe("/model_name", 1000, &Robot::name_parser_callback, this);
    }

    void name_parser_callback(const std_msgs::String::ConstPtr &model)
    {
        ROS_INFO("Robot Name: %s", model->data.c_str());
        m_name = model->data.c_str();
        m_gotRobotNameFlag = 1;
    }
};

class Gps
{
public:
    ros::Subscriber m_gpsSub{};
    double m_currLocation[3] {};

    Gps(ros::NodeHandle* n, Robot* robot)
    {
        // Enabling GPS
        ros::ServiceClient gpsClient = n->serviceClient<webots_ros::set_int>(robot->m_name+"/gps/enable");
        webots_ros::set_int gpsSrv;
        gpsSrv.request.value = 1;

        if (!gpsClient.call(gpsSrv) || !gpsSrv.response.success)
        {
            std::cout << "Failed to enable GPS\n";
        }

        m_gpsSub = n->subscribe(robot->m_name+"/gps/values", 1000, &Gps::GPS_callback, this);
        std::cout << "Initialized gps\n";

    }

    void GPS_callback(const geometry_msgs::PointStamped::ConstPtr &values)
    {   
        m_currLocation[0] = values->point.x;
        m_currLocation[1] = values->point.y;
        m_currLocation[2] = values->point.z;
    }
};

class Imu
{
public:
    ros::Subscriber m_imuSub{};
    double m_yaw {0.0};

    Imu(ros::NodeHandle* n, Robot* robot)
    {
        // Enabling IMU
        ros::ServiceClient imuClient = n->serviceClient<webots_ros::set_int>(robot->m_name+"/imu/enable");
        webots_ros::set_int imuSrv;
        imuSrv.request.value = 1;

        if (!imuClient.call(imuSrv) || !imuSrv.response.success)
        {
            std::cout << "Failed to enable IMU\n";
        }

        m_imuSub = n->subscribe(robot->m_name+"/imu/quaternion", 1000, &Imu::IMU_callback, this);

    }

    void IMU_callback(const sensor_msgs::Imu::ConstPtr &data)
    {
        double q_x {data->orientation.x};       // q prefix stands for quaternion
        double q_y {data->orientation.y};
        double q_z {data->orientation.z};
        double q_w {data->orientation.w};
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav");
    ros::NodeHandle n;

    Robot robot{&n};

    while (robot.m_gotRobotNameFlag == 0)
    {
        ros::spinOnce();    // Wait unitl robot's name is received from the simulator
    }

    Gps gps{&n, &robot};
    Imu imu{&n, &robot};

    ros::Rate loopRate(10);

    ros::Publisher abs_pose_pub = n.advertise<nav_msgs::Odometry>("/absolute_pose",1000);
    ros::Publisher way_pts_pub = n.advertise<nav_msgs::Path>("/waypoints_input",1000);
    ros::Publisher ext_speed_pub = n.advertise<std_msgs::Float64>("/external_speed",1000);

    std_msgs::Float64 ext_speed;
    ext_speed.data = 20.0;

    // Prepping waypoint data to be published for waypoint tracking controller package 
    nav_msgs::Path way_pts;     // Need to hard code this for testing purpose

    // ros::Time way_pts_time;
    way_pts.header.frame_id = "way_pts";
    
    // Hardcoded waypoints = (-44.7, 0, 30.5), (-42.4, 0, 42.29), (-25.8, 0, 44), (-5.4, 0, 49.1)
    geometry_msgs::PoseStamped posestamp;

    posestamp.pose.position.x = -44.7;
    posestamp.pose.position.y = 30.5;
    posestamp.pose.position.z = 0.0;
    posestamp.pose.orientation.x = 0.0;
    posestamp.pose.orientation.y = 0.0;
    posestamp.pose.orientation.z = 0.0;
    posestamp.pose.orientation.w = 1.0;

    way_pts.header.stamp = ros::Time::now();
    way_pts.poses.push_back(posestamp);

    posestamp.pose.position.x = -42.4;
    posestamp.pose.position.y = 42.29;
    posestamp.pose.position.z = 0.0;
    posestamp.pose.orientation.x = 0.0;
    posestamp.pose.orientation.y = 0.0;
    posestamp.pose.orientation.z = 0.0;
    posestamp.pose.orientation.w = 1.0;

    way_pts.header.stamp = ros::Time::now();
    way_pts.poses.push_back(posestamp);

    posestamp.pose.position.x = -25.8;
    posestamp.pose.position.y = 44;
    posestamp.pose.position.z = 0.0;
    posestamp.pose.orientation.x = 0.0;
    posestamp.pose.orientation.y = 0.0;
    posestamp.pose.orientation.z = 0.0;
    posestamp.pose.orientation.w = 1.0;

    way_pts.header.stamp = ros::Time::now();
    way_pts.poses.push_back(posestamp);

    posestamp.pose.position.x = -5.4;
    posestamp.pose.position.y = 49.1;
    posestamp.pose.position.z = 0.0;
    posestamp.pose.orientation.x = 0.0;
    posestamp.pose.orientation.y = 0.0;
    posestamp.pose.orientation.z = 0.0;
    posestamp.pose.orientation.w = 1.0;

    way_pts.header.stamp = ros::Time::now();
    way_pts.poses.push_back(posestamp);

    nav_msgs::Odometry abs_pose;

    while (ros::ok())
    {
        abs_pose.pose.pose.position.x = gps.m_currLocation[0];  // X axis and Z axis are parallel to the ground in Webots
        abs_pose.pose.pose.position.y = gps.m_currLocation[2];  
        abs_pose.pose.pose.orientation.w = 1.0;

        abs_pose_pub.publish(abs_pose);         // Causes prblm in TF

        ext_speed_pub.publish(ext_speed);
        way_pts_pub.publish(way_pts);

        ros::spinOnce();
        loopRate.sleep();
    }

    
}
