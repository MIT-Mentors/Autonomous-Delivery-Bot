#include <iostream>
# include <vector>
#include <cmath>
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

double speed = 0.0;
double steer = 0.0;

double linear_x = 0.0;
double angular_z = 0.0;

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
    double q_x;       // q prefix stands for quaternion
    double q_y;
    double q_z;
    double q_w;

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
        std::cout << "Initialized imu\n";
    }

    void IMU_callback(const sensor_msgs::Imu::ConstPtr &data)
    {
        q_x = data->orientation.x;       // q prefix stands for quaternion
        q_y = data->orientation.y;
        q_z = data->orientation.z;
        q_w = data->orientation.w;

        double q_z_sqr {q_z * q_z};
        double t0 {-2.0 * (q_z_sqr + q_w * q_w) + 1.0};
        double t1 {+2.0 * (q_y * q_z + q_x * q_w)};

        m_yaw = atan2(t1, t0);
    }
};

class Navigation
{
public:
    const double m_maxSpeed {20};
    const double wheelRadius {0.165};
    const double lengthBtnWheels {0.8};

    webots_ros::set_float m_leftWheelSrv;
    webots_ros::set_float m_rightWheelSrv;

    ros::ServiceClient m_leftWheelPosClient{};
    ros::ServiceClient m_rightWheelPosClient{};
    ros::ServiceClient m_leftWheelVelClient{};
    ros::ServiceClient m_rightWheelVelClient{};

    Navigation(ros::NodeHandle* n, Robot* robot)
    {
        m_leftWheelPosClient = n->serviceClient<webots_ros::set_float>(robot->m_name+"/left_wheel_motor/set_position");
        m_rightWheelPosClient = n->serviceClient<webots_ros::set_float>(robot->m_name+"/right_wheel_motor/set_position");

        m_leftWheelVelClient = n->serviceClient<webots_ros::set_float>(robot->m_name+"/left_wheel_motor/set_velocity");
        m_rightWheelVelClient = n->serviceClient<webots_ros::set_float>(robot->m_name+"/right_wheel_motor/set_velocity");
    
        //Setting position to infinity so that the bot can run forever.
        double rightPos {INFINITY};
        double leftPos {INFINITY};

        m_leftWheelSrv.request.value = leftPos;
        m_rightWheelSrv.request.value = rightPos;

        if (!m_leftWheelPosClient.call(m_leftWheelSrv) || !m_rightWheelPosClient.call(m_rightWheelSrv) || !m_leftWheelSrv.response.success || !m_rightWheelSrv.response.success)
        {
            std::cout << "Failed to set position\n";
        }

        set_velocity(0.0, 0.0);
    }

    void navigate_to_point(Imu* imu)
    {
        double desiredYaw = steer;

        double phiErrorUncorrected {-steer};
        double phiError {atan2(sin(phiErrorUncorrected), cos(phiErrorUncorrected))};

        // // Differential drive velocities
        double rightVelocity {((2.0*speed + phiError*lengthBtnWheels)/2.0*wheelRadius)*60/(2*3.14159)};
        double leftVelocity  {((2.0*speed - phiError*lengthBtnWheels)/2.0*wheelRadius)*60/(2*3.14159)};

        set_velocity(rightVelocity, leftVelocity);
    }

    void set_velocity(double rightVelocity, double leftVelocity)
    {
        if (isnan(rightVelocity) || isnan(leftVelocity))
        {
            rightVelocity = 0.0;
            leftVelocity = 0.0;
        }

        // Limiting velocities
        if (rightVelocity > m_maxSpeed)
            rightVelocity = m_maxSpeed;
        if (leftVelocity > m_maxSpeed)
            leftVelocity = m_maxSpeed;

        if (rightVelocity < -m_maxSpeed)
            rightVelocity = -m_maxSpeed;
        if (leftVelocity < -m_maxSpeed)
            leftVelocity = -m_maxSpeed;

        std::cout << rightVelocity << ' ' << leftVelocity << "\n\n";

        m_rightWheelSrv.request.value = rightVelocity;
        m_leftWheelSrv.request.value = leftVelocity;

        if (!m_leftWheelVelClient.call(m_leftWheelSrv) || !m_rightWheelVelClient.call(m_rightWheelSrv) || !m_leftWheelSrv.response.success || !m_rightWheelSrv.response.success)
        {
            std::cout << "Failed to set velocity\n";
        }
    }
};


void speed_callback(const std_msgs::Float64::ConstPtr& msg)
{
    speed = msg->data;
}

void steer_callback(const std_msgs::Float64::ConstPtr& msg)
{
    steer = msg->data;
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    linear_x = msg->linear.x;
    angular_z = msg->angular.y;
}

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
    Navigation nav{&n,&robot};

    ros::Rate loopRate(10);

    ros::Publisher abs_pose_pub = n.advertise<nav_msgs::Odometry>("/absolute_pose",1000);
    ros::Publisher way_pts_pub = n.advertise<nav_msgs::Path>("/waypoints_input",1000);
    ros::Publisher ext_speed_pub = n.advertise<std_msgs::Float64>("/external_speed",1000);

    std_msgs::Float64 ext_speed;
    ext_speed.data = 1000.0;

    // Prepping waypoint data to be published for waypoint tracking controller package 
    nav_msgs::Path way_pts;     // Need to hard code this for testing purpose

    // ros::Time way_pts_time;
    way_pts.header.frame_id = "map";
    
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

    ros::Subscriber speed_sub = n.subscribe("/speed",1,speed_callback);
    ros::Subscriber steer_sub = n.subscribe("/steer",1,steer_callback);

    abs_pose.header.frame_id = "map";

    while (ros::ok())
    {
        abs_pose.pose.pose.position.x = gps.m_currLocation[0];  // X axis and Z axis are parallel to the ground in Webots
        abs_pose.pose.pose.position.y = gps.m_currLocation[2];
        abs_pose.pose.pose.orientation.x = imu.q_x;
        abs_pose.pose.pose.orientation.y = imu.q_y;
        abs_pose.pose.pose.orientation.z = imu.q_z;
        abs_pose.pose.pose.orientation.w = imu.q_w;

        abs_pose_pub.publish(abs_pose);         // Causes prblm in TF

        ext_speed_pub.publish(ext_speed);
        way_pts_pub.publish(way_pts);

        nav.navigate_to_point(&imu);

        ros::spinOnce();
        loopRate.sleep();
    }

    
}
