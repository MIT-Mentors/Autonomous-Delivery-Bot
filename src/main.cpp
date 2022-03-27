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

class SetPoint
{
public:
    ros::NodeHandle* m_nodehandle{};
    ros::Publisher m_reachedSetpointPub{};
    ros::Subscriber m_setpointSub{};
    double m_setpoint[3]{100000.0,100000.0,100000.0};
    
    SetPoint(ros::NodeHandle* n)
    {
        m_nodehandle = n;
        m_reachedSetpointPub = n->advertise<std_msgs::Bool>("isReachedSetPoint", 1000);
        m_setpointSub = n->subscribe("setpoint", 1, &SetPoint::setpointCallback, this);
    }

    void setpointCallback(const geometry_msgs::Vector3::ConstPtr &array)
    {
        m_setpoint[0] = array->x;
        m_setpoint[1] = array->y;
        m_setpoint[2] = array->z;
    }

    void publishSetpoint(int boolVal)
    {
        std_msgs::Bool value;
        value.data = boolVal;
        m_reachedSetpointPub.publish(value);
    }
};

void initNavigation(ros::NodeHandle *n)
{
    //Setting position to infinity so that the bot can run forever.
    double rightPos {INFINITY};
    double leftPos {INFINITY};

    static webots_ros::set_float leftWheelSrv;
    static webots_ros::set_float rightWheelSrv;

    static ros::ServiceClient leftWheelPosClient = n->serviceClient<webots_ros::set_float>(robotName+"/left_wheel_motor/set_position");
    static ros::ServiceClient rightWheelPosClient = n->serviceClient<webots_ros::set_float>(robotName+"/right_wheel_motor/set_position");

    leftWheelSrv.request.value = leftPos;
    rightWheelSrv.request.value = rightPos;

    if (!leftWheelPosClient.call(leftWheelSrv) || !rightWheelPosClient.call(rightWheelSrv) || !leftWheelSrv.response.success || !rightWheelSrv.response.success)
    {
        std::cout << "Failed to set position\n";
    }
}

void setVelocity(ros::NodeHandle *n, double rightVelocity, double leftVelocity)
{
    static const double maxSpeed {20};

    static webots_ros::set_float leftWheelSrv;
    static webots_ros::set_float rightWheelSrv;

    static ros::ServiceClient leftWheelClient = n->serviceClient<webots_ros::set_float>(robotName+"/left_wheel_motor/set_velocity");
    static ros::ServiceClient rightWheelClient = n->serviceClient<webots_ros::set_float>(robotName+"/right_wheel_motor/set_velocity");

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

void navigateToPoint(SetPoint* setpoint, ros::NodeHandle *n)
{
    static const double Kp {2};
    static const double wheelRadius {0.165};
    static const double lengthBtnWheels {0.8};

    double xComponent {setpoint->m_setpoint[0] - currLocation[0]};
    double yComponent {setpoint->m_setpoint[2] - currLocation[2]};

    double desiredYaw {atan2(yComponent, xComponent)};

    double phiErrorUncorrected {desiredYaw-yaw};
    double phiError {atan2(sin(phiErrorUncorrected), cos(phiErrorUncorrected))};

    double distanceError {findDistanceBetweenPoints(currLocation, setpoint->m_setpoint)};

    // Unicycle angle and velocity
    double unicycleAngle {distanceError*phiError};
    double unicycleVelocity {Kp*distanceError};

    // Differential drive velocities
    double rightVelocity {(2.0*unicycleVelocity + unicycleAngle*lengthBtnWheels)/2.0*wheelRadius};
    double leftVelocity  {(2.0*unicycleVelocity - unicycleAngle*lengthBtnWheels)/2.0*wheelRadius};

    setVelocity(n, rightVelocity, leftVelocity);

    static const double threshold {5.0};

    if (distanceError < threshold)
    {
        setpoint->publishSetpoint(1);
    }
    else
    {
        setpoint->publishSetpoint(0);
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

void enableGPS(ros::NodeHandle *n)
{
    ros::ServiceClient gpsClient = n->serviceClient<webots_ros::set_int>(robotName+"/gps/enable");
    webots_ros::set_int gpsSrv;
    gpsSrv.request.value = 1;

    if (!gpsClient.call(gpsSrv) || !gpsSrv.response.success)
    {
        std::cout << "Failed to enable GPS\n";
    }
}

void enableIMU(ros::NodeHandle *n)
{
    ros::ServiceClient imuClient = n->serviceClient<webots_ros::set_int>(robotName+"/imu/enable");
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
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/model_name", 1000, nameParserCallback);

    while (gotRobotNameFlag == 0)
    {
        ros::spinOnce();    // Wait unitl robot's name is received from the simulator
    }

    initNavigation(&n);
    enableGPS(&n);
    enableIMU(&n);

    setVelocity(&n, 0.0, 0.0);

    SetPoint setpoint{&n};

    // Publishers

    // Subscribers
    ros::Subscriber gpsSub = n.subscribe(robotName+"/gps/values", 1000, GPSCallback);
    ros::Subscriber imuSub = n.subscribe(robotName+"/imu/quaternion", 1000, IMUCallback);
    ros::Subscriber senderLocationSub = n.subscribe("senderLocation", 1000, senderLocationCallback);
    ros::Subscriber receiverLocationSub = n.subscribe("receiverLocation", 1000, receiverLocationCallback);
    
    ros::Rate loop_rate(10); //10 Hz

    while (ros::ok())
    {
        if ((senderLocation.compare("nil") != 0) && (receiverLocation.compare("nil") !=0) && setpoint.m_setpoint[0] != 100000.0)     //if both are not nill and setpoint has set, then true
        {
            navigateToPoint(&setpoint, &n);
            loop_rate.sleep();
        }
        else
        {
            setVelocity(&n,0.0, 0.0);

            setpoint.publishSetpoint(0);
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }   
    return 0;
}
