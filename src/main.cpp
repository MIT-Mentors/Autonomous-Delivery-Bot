#include <cmath>
#include <cstdlib>
#include <iostream>
#include <math.h>
#include <string>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>

std::string robotName {};
std::string senderLocation {};
std::string receiverLocation {};

int gotRobotNameFlag{0};
double yaw {0.0};
double currLocation[3] {};
double g_setpoint[3] {100000.0,100000.0,100000.0};

double findDistanceBetweenPoints(double point1[3], double point2[3])
{
    // Finding the distance between 2 points on a 2D plane using 
    // the formula d = √((x_2-x_1)² + (y_2-y_1)²). 
    // Here since the robot's x and z axes are parallel to the
    // ground we use those coordinates to calculate the distance.
    return sqrt((pow((point1[0]-point2[0]), 2)+pow((point1[2]-point2[2]), 2)));
}

void nameParserCallback(const std_msgs::String::ConstPtr &model)
{
    ROS_INFO("Robot Name: %s", model->data.c_str());
    robotName = model->data.c_str();
    gotRobotNameFlag = 1;
}

void setpointCallback(const geometry_msgs::Vector3::ConstPtr &array)
{
    g_setpoint[0] = array->x;
    g_setpoint[1] = array->y;
    g_setpoint[2] = array->z;
}

void initNavigation(ros::NodeHandle *nodehandle)
{
    //Setting position to infinity so that the bot can run forever.
    double rightPos {INFINITY};
    double leftPos {INFINITY};

    static webots_ros::set_float leftWheelSrv;
    static webots_ros::set_float rightWheelSrv;

    static ros::ServiceClient leftWheelPosClient = nodehandle->serviceClient<webots_ros::set_float>(robotName+"/left_wheel_motor/set_position");
    static ros::ServiceClient rightWheelPosClient = nodehandle->serviceClient<webots_ros::set_float>(robotName+"/right_wheel_motor/set_position");

    leftWheelSrv.request.value = leftPos;
    rightWheelSrv.request.value = rightPos;

    if (!leftWheelPosClient.call(leftWheelSrv) || !rightWheelPosClient.call(rightWheelSrv) || !leftWheelSrv.response.success || !rightWheelSrv.response.success)
    {
        std::cout << "Failed to set position\n";
    }
}

void setVelocity(ros::NodeHandle *nodehandle, double rightVelocity, double leftVelocity)
{
    static const double maxSpeed {20};

    static webots_ros::set_float leftWheelSrv;
    static webots_ros::set_float rightWheelSrv;

    static ros::ServiceClient leftWheelClient = nodehandle->serviceClient<webots_ros::set_float>(robotName+"/left_wheel_motor/set_velocity");
    static ros::ServiceClient rightWheelClient = nodehandle->serviceClient<webots_ros::set_float>(robotName+"/right_wheel_motor/set_velocity");

    // Limiting velocities
    if (rightVelocity > maxSpeed)
        rightVelocity = maxSpeed;
    if (leftVelocity > maxSpeed)
        leftVelocity = maxSpeed;

    if (rightVelocity < -maxSpeed)
        rightVelocity = -maxSpeed;
    if (leftVelocity < -maxSpeed)
        leftVelocity = -maxSpeed;

    rightWheelSrv.request.value = rightVelocity;
    leftWheelSrv.request.value = leftVelocity;

    if (!leftWheelClient.call(leftWheelSrv) || !rightWheelClient.call(rightWheelSrv) || !leftWheelSrv.response.success || !rightWheelSrv.response.success)
    {
        std::cout << "Failed to set velocity\n";
    }
}

void navigateToPoint(double setpoint[3], ros::Publisher *reachedSetpointPub, ros::NodeHandle *nodehandle)
{
    static const double Kp {2};
    static const double wheelRadius {0.165};
    static const double lengthBtnWheels {0.8};

    double xComponent {setpoint[0] - currLocation[0]};
    double yComponent {setpoint[2] - currLocation[2]};

    double desiredYaw {atan2(yComponent, xComponent)};

    double phiErrorUncorrected {desiredYaw-yaw};
    double phiError {atan2(sin(phiErrorUncorrected), cos(phiErrorUncorrected))};

    double distanceError {findDistanceBetweenPoints(currLocation, setpoint)};

    // Unicycle angle and velocity
    double unicycleAngle {distanceError*phiError};
    double unicycleVelocity {Kp*distanceError};

    // Differential drive velocities
    double rightVelocity {(2.0*unicycleVelocity + unicycleAngle*lengthBtnWheels)/2.0*wheelRadius};
    double leftVelocity  {(2.0*unicycleVelocity - unicycleAngle*lengthBtnWheels)/2.0*wheelRadius};

    setVelocity(nodehandle, rightVelocity, leftVelocity);

    static const double threshold {5.0};

    if (distanceError < threshold)
    {
        std_msgs::Bool value;
        value.data = 1;
        reachedSetpointPub->publish(value);
    }
    else
    {
        std_msgs::Bool value;
        value.data = 0;
        reachedSetpointPub->publish(value);
    }
}

void senderLocationCallback(const std_msgs::String::ConstPtr &name)
{
    senderLocation = name->data.c_str();
}

void receiverLocationCallback(const std_msgs::String::ConstPtr &name)
{
    receiverLocation = name->data.c_str();
}

void enableGPS(ros::NodeHandle *nodehandle)
{
    ros::ServiceClient gpsClient = nodehandle->serviceClient<webots_ros::set_int>(robotName+"/gps/enable");
    webots_ros::set_int gpsSrv;
    gpsSrv.request.value = 1;

    if (!gpsClient.call(gpsSrv) || !gpsSrv.response.success)
    {
        std::cout << "Failed to enable GPS\n";
    }
}

void enableIMU(ros::NodeHandle *nodehandle)
{
    ros::ServiceClient imuClient = nodehandle->serviceClient<webots_ros::set_int>(robotName+"/imu/enable");
    webots_ros::set_int imuSrv;
    imuSrv.request.value = 1;

    if (!imuClient.call(imuSrv) || !imuSrv.response.success)
    {
        std::cout << "Failed to enable IMU\n";
    }
}

void GPSCallback(const geometry_msgs::PointStamped::ConstPtr &values)
{    
    currLocation[0] = values->point.x;
    currLocation[1] = values->point.y;
    currLocation[2] = values->point.z;
}

void IMUCallback(const sensor_msgs::Imu::ConstPtr &data)
{
    double q_x {data->orientation.x};       // q prefix stands for quaternion
    double q_y {data->orientation.y};
    double q_z {data->orientation.z};
    double q_w {data->orientation.w};

    // Calculating yaw using quaternion values
    // Reference: https://stackoverflow.com/a/37560411

    double q_z_sqr {q_z * q_z};
    double t0 {-2.0 * (q_z_sqr + q_w * q_w) + 1.0};
    double t1 {+2.0 * (q_y * q_z + q_x * q_w)};

    yaw = atan2(t1, t0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nodehandle;

    ros::Subscriber sub = nodehandle.subscribe("/model_name", 1000, nameParserCallback);

    while (gotRobotNameFlag == 0)
    {
        ros::spinOnce();    // Wait unitl robot's name is received from the simulator
    }

    initNavigation(&nodehandle);
    enableGPS(&nodehandle);
    enableIMU(&nodehandle);

    setVelocity(&nodehandle, 0.0, 0.0);

    // Publishers
    ros::Publisher reachedSetpointPub = nodehandle.advertise<std_msgs::Bool>("isReachedSetPoint", 1000);

    // Subscribers
    ros::Subscriber gpsSub = nodehandle.subscribe(robotName+"/gps/values", 1000, GPSCallback);
    ros::Subscriber imuSub = nodehandle.subscribe(robotName+"/imu/quaternion", 1000, IMUCallback);
    ros::Subscriber setpointSub = nodehandle.subscribe("setpoint", 1, setpointCallback);
    ros::Subscriber senderLocationSub = nodehandle.subscribe("senderLocation", 1000, senderLocationCallback);
    ros::Subscriber receiverLocationSub = nodehandle.subscribe("receiverLocation", 1000, receiverLocationCallback);
    
    ros::Rate loop_rate(10); //10 Hz

    while (ros::ok())
    {
        if ((senderLocation.compare("nil") != 0) && (receiverLocation.compare("nil") !=0) && g_setpoint[0] != 100000.0)     //if both are not nill and setpoint has set, then true
        {
            navigateToPoint(g_setpoint, &reachedSetpointPub, &nodehandle);
            loop_rate.sleep();
        }
        else
        {
            setVelocity(&nodehandle,0.0, 0.0);

            std_msgs::Bool value;
            value.data = 0;
            reachedSetpointPub.publish(value);
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }   
    return 0;
}
