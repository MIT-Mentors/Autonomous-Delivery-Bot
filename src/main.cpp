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

        // Calculating yaw using quaternion values
        // Reference: https://stackoverflow.com/a/37560411

        double q_z_sqr {q_z * q_z};
        double t0 {-2.0 * (q_z_sqr + q_w * q_w) + 1.0};
        double t1 {+2.0 * (q_y * q_z + q_x * q_w)};

        m_yaw = atan2(t1, t0);
    }
};

class SetPoint
{
public:
    ros::Publisher m_reachedSetpointPub{};
    ros::Subscriber m_setpointSub{};
    double m_setpoint[3]{100000.0,100000.0,100000.0};
    
    SetPoint(ros::NodeHandle* n)
    {
        m_reachedSetpointPub = n->advertise<std_msgs::Bool>("isReachedSetPoint", 1000);
        m_setpointSub = n->subscribe("setpoint", 1, &SetPoint::setpoint_callback, this);
    }

    void setpoint_callback(const geometry_msgs::Vector3::ConstPtr &array)
    {
        m_setpoint[0] = array->x;
        m_setpoint[1] = array->y;
        m_setpoint[2] = array->z;
    }

    void publish_setpoint(int boolVal)
    {
        std_msgs::Bool value;
        value.data = boolVal;
        m_reachedSetpointPub.publish(value);
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

    void navigate_to_point(SetPoint* setpoint, Imu* imu, Gps* gps)
    {
        static const double Kp {2};

        // The robot's x and z axes are parallel to the ground
        double xComponent {setpoint->m_setpoint[0] - gps->m_currLocation[0]};
        double zComponent {setpoint->m_setpoint[2] - gps->m_currLocation[2]};

        double desiredYaw {atan2(zComponent, xComponent)};

        double phiErrorUncorrected {desiredYaw - imu->m_yaw};
        double phiError {atan2(sin(phiErrorUncorrected), cos(phiErrorUncorrected))};

        double distanceError {find_distance_between_points(gps->m_currLocation, setpoint->m_setpoint)};

        // Unicycle angle and velocity
        double unicycleAngle {distanceError*phiError};
        double unicycleVelocity {Kp*distanceError};

        // Differential drive velocities
        double rightVelocity {(2.0*unicycleVelocity + unicycleAngle*lengthBtnWheels)/2.0*wheelRadius};
        double leftVelocity  {(2.0*unicycleVelocity - unicycleAngle*lengthBtnWheels)/2.0*wheelRadius};

        set_velocity(rightVelocity, leftVelocity);

        static const double threshold {5.0};

        if (distanceError < threshold)
        {
            setpoint->publish_setpoint(1);
        }
        else
        {
            setpoint->publish_setpoint(0);
        }
    }

    void set_velocity(double rightVelocity, double leftVelocity)
    {
        // Limiting velocities
        if (rightVelocity > m_maxSpeed)
            rightVelocity = m_maxSpeed;
        if (leftVelocity > m_maxSpeed)
            leftVelocity = m_maxSpeed;

        if (rightVelocity < -m_maxSpeed)
            rightVelocity = -m_maxSpeed;
        if (leftVelocity < -m_maxSpeed)
            leftVelocity = -m_maxSpeed;

        m_rightWheelSrv.request.value = rightVelocity;
        m_leftWheelSrv.request.value = leftVelocity;

        if (!m_leftWheelVelClient.call(m_leftWheelSrv) || !m_rightWheelVelClient.call(m_rightWheelSrv) || !m_leftWheelSrv.response.success || !m_rightWheelSrv.response.success)
        {
            std::cout << "Failed to set velocity\n";
        }
    }

    double find_distance_between_points(double point1[3], double point2[3])
    {
        // Finding the distance between 2 points on a 2D plane using 
        // the formula d = √((x_2-x_1)² + (y_2-y_1)²). 
        // Here since the robot's x and z axes are parallel to the
        // ground we use those coordinates to calculate the distance.
        return sqrt((pow((point1[0]-point2[0]), 2)+pow((point1[2]-point2[2]), 2)));
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle n;

    Robot robot{&n};

    while (robot.m_gotRobotNameFlag == 0)
    {
        ros::spinOnce();    // Wait unitl robot's name is received from the simulator
    }

    Navigation navigation{&n, &robot};
    Gps gps{&n, &robot};
    Imu imu{&n, &robot};
    SetPoint setpoint{&n};

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (setpoint.m_setpoint[0] != 100000.0)     //if the setpoint has been set, then true
        {
            navigation.navigate_to_point(&setpoint, &imu, &gps);
            loop_rate.sleep();
        }
        else
        {
            navigation.set_velocity(0.0, 0.0);

            setpoint.publish_setpoint(0);
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }   
    return 0;
}
