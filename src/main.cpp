#include <cmath>
#include <iostream>
#include <math.h>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>


std::string robotName{""};
std::string direction{""};

int flag{0};
int straightFlag{0};
int turnFlag{0};

double yaw{0.0};
double wheelRadius{0.165};
double lengthBtnWheels{0.8};
double maxSpeed{20};

double currLocation[3]{};
double GPSValIntersection[3]{-43,0.9,44};

ros::Publisher reachedSetpointPub;

double setpoint[3]{};


ros::NodeHandle* nh; //pointer

webots_ros::set_float leftWheelSrv;
webots_ros::set_float rightWheelSrv;

ros::ServiceClient leftWheelClient;
ros::ServiceClient rightWheelClient;

double findDistanceBetweenPoints(double point1[3], double point2[3])
{
    return sqrt((pow((point1[0]-point2[0]), 2)+pow((point1[2]-point2[2]),2)));
}

void callbackNameParser(const std_msgs::String::ConstPtr &model)
{
    ROS_INFO("Robot Name: [%s]", model->data.c_str());
    robotName = static_cast<std::string>(model->data.c_str());
    flag = 1;
}

void setpointCallback(const geometry_msgs::Vector3::ConstPtr &array)
{
    setpoint[0] = array->x;
    setpoint[1] = array->y;
    setpoint[2] = array->z;

}

/* void callbackDirections(const std_msgs::String::ConstPtr& input_direction)
{
    ROS_INFO("Received direction command: %s", input_direction->data.c_str());
    direction = static_cast<std::string>(input_direction->data.c_str());

    if (direction.compare("straight") == 0)
        {
            straightFlag = 1;
        }
    else
        {
            straightFlag = 0;
        }
}*/

void initNavigation()
{
    //Setting position   

    double rightPos = INFINITY;
    double leftPos =  INFINITY;

    ros::ServiceClient leftWheelClientPos = nh->serviceClient<webots_ros::set_float>(robotName+"/left_wheel_motor/set_position");
    ros::ServiceClient rightWheelClientPos = nh->serviceClient<webots_ros::set_float>(robotName+"/right_wheel_motor/set_position");

    leftWheelSrv.request.value = leftPos;
    rightWheelSrv.request.value = rightPos;

    if (!leftWheelClientPos.call(leftWheelSrv) || !rightWheelClientPos.call(rightWheelSrv) || !leftWheelSrv.response.success || !rightWheelSrv.response.success)
        {
            std::cout << "Failed to set position\n";
        }

    leftWheelClient = nh->serviceClient<webots_ros::set_float>(robotName+"/left_wheel_motor/set_velocity");
    rightWheelClient = nh->serviceClient<webots_ros::set_float>(robotName+"/right_wheel_motor/set_velocity");
}

void setVelocity(double rightVelocity, double leftVelocity)
{
    rightWheelSrv.request.value = rightVelocity;
    leftWheelSrv.request.value = leftVelocity;

    if (!leftWheelClient.call(leftWheelSrv) || !rightWheelClient.call(rightWheelSrv) || !leftWheelSrv.response.success || !rightWheelSrv.response.success)
        {
            std::cout << "Failed to set velocity\n";
        }
}

void navigateToPoint(double setpoint[3])
{
    // double error = findDistanceBetweenPoints(currLocation, setpoint);
    // double Kp = 1;

    // double velocity = Kp*error;

    // if (velocity > 20)
    // {
    //     velocity = 20;
    // }
    // setVelocity(0.0);

    double x_component{setpoint[0] - currLocation[0]};
    double y_component{setpoint[2] - currLocation[2]};

    double phi_ref = atan2(y_component, x_component);

    double phi_error1 = phi_ref-yaw;
    double phi_error = atan2(sin(phi_error1),cos(phi_error1));

    // double w = 20*phi_error;
    // double v = 2*(sqrt(x_component*x_component + y_component*y_component));

    // Unicycle
    double w = 20*phi_error;
    double v = 2*(sqrt(x_component*x_component + y_component*y_component));

    

    double right_velocity = (2.0*v + w*lengthBtnWheels)/2.0*wheelRadius;
    double left_velocity = (2.0*v - w*lengthBtnWheels)/2.0*wheelRadius;

    if (right_velocity > maxSpeed)
        right_velocity = maxSpeed;
    if (left_velocity > maxSpeed)
        left_velocity = maxSpeed;

    if (right_velocity < -maxSpeed)
        right_velocity = -maxSpeed;
    if (left_velocity < -maxSpeed)
        left_velocity = -maxSpeed;

    setVelocity(right_velocity, left_velocity);
    // std::cout << phi_error << "          " << phi_ref << "          " << yaw << '\n';
    // std::cout << right_velocity << ' ' << left_velocity << '\n' << '\n';

    double dist = findDistanceBetweenPoints(currLocation,setpoint);
    std::cout << "dist: " << dist << '\n';
    
    if (dist<5)
    {
        std::cout << "Reached setpoint\n";
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

    double intersectionDistance =  findDistanceBetweenPoints(currLocation, GPSValIntersection);

    // std::cout << "Distance: " << intersectionDistance << '\n';

    if (intersectionDistance < 3.5)
    {
        turnFlag = 1;
    }
    else
    {
        turnFlag = 0;
    }
}

void IMUCallback(const sensor_msgs::Imu::ConstPtr &data)
{
    double q_x = data->orientation.x;       // q prefix stands for quaternion
    double q_y = data->orientation.y;
    double q_z = data->orientation.z;
    double q_w = data->orientation.w;

    // Calculating yaw using quaternion values

    double q_z_sqr = q_z * q_z;
    double t0 = -2.0 * (q_z_sqr + q_w * q_w) + 1.0;
    double t1 = +2.0 * (q_y * q_z + q_x * q_w);
    double t2 = -2.0 * (q_y * q_w - q_x * q_z);
    double t3 = +2.0 * (q_z * q_w + q_x * q_y);
    double t4 = -2.0 * (q_y * q_y + q_z_sqr) + 1.0;

    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;

    double pitch = asin(t2);
    double roll = atan2(t3, t4);
    yaw = atan2(t1, t0);

    // yaw = asin(t2);
    // std::cout << pitch << ' ' << roll << ' ' << yaw << '\n';
}

double getVelocity()
{
    if (straightFlag)
        return 5.0;
    else
        return 0.0;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"main");
    ros::NodeHandle n;

    nh = &n;    // assigning node handle object to global pointer

    ros::Subscriber sub = n.subscribe("/model_name",1000,callbackNameParser);
    

    while (flag == 0) {ros::spinOnce();}

    initNavigation();
    enableGPS();
    enableIMU();

    // Gps
    ros::Subscriber gps_sub = n.subscribe(robotName+"/gps/values",1000,GPSCallback);
    ros::Subscriber imu_sub = n.subscribe(robotName+"/imu/quaternion", 1000, IMUCallback);
    ros::Subscriber setpointSub = n.subscribe("setpoint", 1000, setpointCallback);

    //ros::Subscriber dir_sub = n.subscribe("/directions",1000,callbackDirections);

    reachedSetpointPub = n.advertise<std_msgs::Bool>("reachedSetpointBool",1000);
   
    ros::Rate loop_rate(10); //100 Hz

    while (ros::ok())
    {
        
        navigateToPoint(setpoint);
        
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
