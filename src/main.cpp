#include <cmath>
#include <iostream>
#include <math.h>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <geometry_msgs/PointStamped.h>


std::string robotName{""};
std::string direction{""};

int flag{0};
int straightFlag{0};
int turnFlag{0};

double currLocation[3]{};
double GPSValIntersection[3] = {-43,0.9,44};

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

    double rightPos = 10000.0;
    double leftPos =  10000.0;

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

void setVelocity(double velocity)
{
    double left_velocity = velocity;
    double right_velocity = velocity;

    leftWheelSrv.request.value = left_velocity;
    rightWheelSrv.request.value = right_velocity;

    if (!leftWheelClient.call(leftWheelSrv) || !rightWheelClient.call(rightWheelSrv) || !leftWheelSrv.response.success || !rightWheelSrv.response.success)
        {
            std::cout << "Failed to set velocity\n";
        }
}
void navigateToPoint(double setpoint[3])
{
    double error = findDistanceBetweenPoints(currLocation, setpoint);
    double Kp = 1;

    double velocity = Kp*error;

    if (velocity > 20)
    {
        velocity = 20;
    }
    setVelocity(velocity);
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

    // Gps
    ros::Subscriber gps_sub = n.subscribe(robotName+"/gps/values",1000,GPSCallback);

    //ros::Subscriber dir_sub = n.subscribe("/directions",1000,callbackDirections);

   
    ros::Rate loop_rate(100); //100 Hz

    while (ros::ok())
    {
        
        navigateToPoint(GPSValIntersection);
        
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
