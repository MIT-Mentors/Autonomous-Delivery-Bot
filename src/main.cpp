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
double wheelRadius {0.165};
double lengthBtnWheels {0.8};
double maxSpeed {20};

double currLocation[3] {};
double g_setpoint[3] {100000.0,100000.0,100000.0};

ros::NodeHandle* nh; //pointer

webots_ros::set_float leftWheelSrv;
webots_ros::set_float rightWheelSrv;

ros::ServiceClient leftWheelClient;
ros::ServiceClient rightWheelClient;

ros::Publisher reachedSetpointPub;

double findDistanceBetweenPoints(double point1[3], double point2[3])
{
    return sqrt((pow((point1[0]-point2[0]), 2)+pow((point1[2]-point2[2]),2)));
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

void initNavigation()
{
    //Setting position to infinity so that the bot can run forever.

    double rightPos {INFINITY};
    double leftPos {INFINITY};

    ros::ServiceClient leftWheelPosClient = nh->serviceClient<webots_ros::set_float>(robotName+"/left_wheel_motor/set_position");
    ros::ServiceClient rightWheelPosClient = nh->serviceClient<webots_ros::set_float>(robotName+"/right_wheel_motor/set_position");

    leftWheelSrv.request.value = leftPos;
    rightWheelSrv.request.value = rightPos;

    if (!leftWheelPosClient.call(leftWheelSrv) || !rightWheelPosClient.call(rightWheelSrv) || !leftWheelSrv.response.success || !rightWheelSrv.response.success)
    {
        std::cout << "Failed to set position\n";
    }

    leftWheelClient = nh->serviceClient<webots_ros::set_float>(robotName+"/left_wheel_motor/set_velocity");
    rightWheelClient = nh->serviceClient<webots_ros::set_float>(robotName+"/right_wheel_motor/set_velocity");
}

void setVelocity(double rightVelocity, double leftVelocity)
{
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

void navigateToPoint(double setpoint[3])
{
    double Kp {2};

    double xComponent {setpoint[0] - currLocation[0]};
    double yComponent {setpoint[2] - currLocation[2]};

    double desiredYaw {atan2(yComponent, xComponent)};

    double phiErrorUncorrected {desiredYaw-yaw};
    double phiError {atan2(sin(phiErrorUncorrected),cos(phiErrorUncorrected))};

    double distanceError {findDistanceBetweenPoints(currLocation,setpoint)};

    // Unicycle angle and velocity
    double unicycleAngle {distanceError*phiError};
    double unicycleVelocity {Kp*distanceError};

    // Differential drive velocities
    double rightVelocity {(2.0*unicycleVelocity + unicycleAngle*lengthBtnWheels)/2.0*wheelRadius};
    double leftVelocity  {(2.0*unicycleVelocity - unicycleAngle*lengthBtnWheels)/2.0*wheelRadius};

    setVelocity(rightVelocity, leftVelocity);

    double threshold {5.0};

    if (distanceError < threshold)
    {
        std_msgs::Bool value;
        value.data = 1;
        reachedSetpointPub.publish(value);
    }
    else
    {
        std_msgs::Bool value;
        value.data = 0;
        reachedSetpointPub.publish(value);
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

void enableGPS()
{
    ros::ServiceClient gpsClient = nh->serviceClient<webots_ros::set_int>(robotName+"/gps/enable");
    webots_ros::set_int gpsSrv;
    gpsSrv.request.value = 1;

    if (!gpsClient.call(gpsSrv) || !gpsSrv.response.success)
    {
        std::cout << "Failed to enable GPS\n";
    }
}

void enableIMU()
{
    ros::ServiceClient imuClient = nh->serviceClient<webots_ros::set_int>(robotName+"/imu/enable");
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
    // Borrowed the conversion code from here https://stackoverflow.com/a/37560411

    double q_z_sqr {q_z * q_z};
    double t0 {-2.0 * (q_z_sqr + q_w * q_w) + 1.0};
    double t1 {+2.0 * (q_y * q_z + q_x * q_w)};

    yaw = atan2(t1, t0);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"main");
    ros::NodeHandle n;

    nh = &n;    // assigning node handle object to global pointer

    ros::Subscriber sub = n.subscribe("/model_name",1000,nameParserCallback);

    while (gotRobotNameFlag == 0) {ros::spinOnce();}    // Wait unitl robot's name is received from the simulator

    initNavigation();
    enableGPS();
    enableIMU();

    setVelocity(0.0, 0.0);

    // Publishers
    reachedSetpointPub = n.advertise<std_msgs::Bool>("reachedSetpointBool",1000);

    // Subscribers
    ros::Subscriber gpsSub = n.subscribe(robotName+"/gps/values",1000,GPSCallback);
    ros::Subscriber imuSub = n.subscribe(robotName+"/imu/quaternion", 1000, IMUCallback);
    ros::Subscriber setpointSub = n.subscribe("setpoint", 1, setpointCallback);
    ros::Subscriber senderLocationSub = n.subscribe("senderLocation",1000,senderLocationCallback);
    ros::Subscriber receiverLocationSub = n.subscribe("receiverLocation",1000,receiverLocationCallback);
    
    ros::Rate loop_rate(10); //10 Hz

    while (ros::ok())
    {
        if ((senderLocation.compare("nil") != 0) && (receiverLocation.compare("nil") !=0) && g_setpoint[0] != 100000.0)     //if both are not nill and setpoint has set, then true
        {
            navigateToPoint(g_setpoint);
            loop_rate.sleep();
        }
        else
        {
            setVelocity(0.0, 0.0);

            std_msgs::Bool value;
            value.data = 0;
            reachedSetpointPub.publish(value);
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }   
    return 0;
}
